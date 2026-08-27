#!/usr/bin/env python3
"""robotac_mission 状态机节点壳（M2 飞行轮）。

预启动：WAIT_READY / WAIT_START，只读安全门；`mission.flight_enabled=false`
时 start 保持占位语义（G1 预启动范围）。`flight_enabled=true` 后 start 捕获
home（起飞点局部归零）并进入 TAKEOFF，由 20Hz 定时器驱动 FlightDriver 逐态
推进 C1-C5（真实控制受 interfaces.dry_run 门控）。

订阅传感器/飞控话题 -> 计算安全门 -> 驱动 state_machine -> 发布
state / state_reason / active / target / result，并暴露 start / stop / reset 服务。

BOOT 可重入：mission_reset（ERROR -> BOOT）会重新从磁盘读取 mission.yaml，
不得复用进程内存中缓存的旧参数。
"""

import math
import threading

import rospy
import tf2_ros
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import EstimatorStatus, ExtendedState, State, TimesyncStatus
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, TriggerResponse

from robotac_mission import guards
from robotac_mission.config import ConfigError, load_config
from robotac_mission.coordinates import Coordinates
from robotac_mission.flight_driver import FlightDriver
from robotac_mission.interfaces import MissionInterfaces
from robotac_mission.landing import LandingConfirmation
from robotac_mission.state_machine import MissionState, MissionStateMachine


class MissionNode(object):
    def __init__(self):
        self.config_path = rospy.get_param("~mission_yaml", "mission.yaml")
        self.config = None
        self.machine = MissionStateMachine()
        # rospy 服务回调（线程池）、Timer（独立线程）与订阅回调（主线程）并发触达
        # machine；所有处理器在进程级 RLock 下串行化，保证操作员 stop / 安全门对
        # 任务推进的原子抢占（飞行轮要求）。锁须覆盖整个 handler 体（评估+转移），
        # 仅锁 state_machine 内部挡不住 "评估前置 → 转移" 之间的窗口。
        self._lock = threading.RLock()

        # 最新消息缓存（供安全门使用；None 表示尚未收到）。各话题另存最近一次
        # 到达时刻 *_received，用于数据年龄门控——统一按到达时刻判龄（不依赖
        # 发送方 header.stamp：Bool/String 无 header，第三方 fake 仅 pose 打戳）。
        self.fcu_state = None
        self.fcu_received = None
        self.extended_state = None
        self.extended_received = None
        self.extended_sequence = 0
        self.pose = None
        self.pose_received = None
        self.velocity = None
        self.velocity_received = None
        self.estimator = None
        self.estimator_received = None
        self.timesync = None
        self.timesync_received = None
        self.vision_healthy = None
        self.vision_healthy_received = None
        self.vision_state = None
        self.vision_state_received = None
        self.tag = None
        self.tag_received = None
        self.payload_release_confirmed = False
        self.payload_release_received = None
        self.payload_release_sequence = 0

        self.state_pub = rospy.Publisher("/robotac_mission/state", String,
                                         queue_size=1, latch=True)
        self.reason_pub = rospy.Publisher("/robotac_mission/state_reason",
                                          String, queue_size=1, latch=True)
        self.active_pub = rospy.Publisher("/robotac_mission/active", Bool,
                                          queue_size=1, latch=True)
        self.target_pub = rospy.Publisher("/robotac_mission/target",
                                          PoseStamped, queue_size=1, latch=True)
        self.result_pub = rospy.Publisher("/robotac_mission/result", String,
                                          queue_size=1, latch=True)

        rospy.Service("/robotac_mission/start", Trigger, self._on_start)
        rospy.Service("/robotac_mission/stop", Trigger, self._on_stop)
        rospy.Service("/robotac_mission/reset", Trigger, self._on_reset)
        rospy.Service("/robotac_mission/manual_takeover", Trigger,
                      self._on_manual_takeover)

        rospy.Subscriber("/mavros/state", State, self._cb_fcu_state,
                         queue_size=5)
        rospy.Subscriber("/mavros/extended_state", ExtendedState,
                         self._cb_extended, queue_size=5)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped,
                         self._cb_pose, queue_size=10)
        rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped,
                         self._cb_velocity, queue_size=10)
        rospy.Subscriber("/mavros/estimator_status", EstimatorStatus,
                         self._cb_estimator, queue_size=5)
        rospy.Subscriber("/mavros/timesync_status", TimesyncStatus,
                         self._cb_timesync, queue_size=5)
        rospy.Subscriber("/vision_pose_bridge/healthy", Bool,
                         self._cb_vision_healthy, queue_size=1)
        rospy.Subscriber("/vision_pose_bridge/state", String,
                         self._cb_vision_state, queue_size=1)
        rospy.Subscriber("/tag_detections", AprilTagDetectionArray,
                         self._cb_tag, queue_size=5)
        rospy.Subscriber("/robotac_servo/release_confirmed", Bool,
                         self._cb_payload_release_confirmed, queue_size=5)

        self.interfaces = MissionInterfaces(
            dry_run=True, target_sink=self.target_pub.publish)
        # M2 飞行轮：坐标系（起飞点局部归零）、TF（C3 Tag→map）、飞行驱动。
        self.coord = Coordinates()
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.camera_frame = rospy.get_param(
            "~camera_frame", "camera_rgb_optical_frame")
        self.driver = None
        self.flight_enabled = False
        self._landing_confirmation = None
        self._landing_active = False
        self._landing_target = None
        self._landing_started = None
        self._landing_last_request = None
        self._landing_requests = 0
        self._landing_mode_confirmed = False
        self._landing_confirmation_started = False
        self._landing_failure_reported = False
        self._boot()
        rospy.Timer(rospy.Duration(0.2), self._on_timer)
        # L1：飞行控制频率用 config.control.rate_hz（参数校验失败时退回 20Hz）
        rate_hz = (20.0 if self.config is None
                   else float(self.config["control"]["rate_hz"]))
        rospy.Timer(rospy.Duration(1.0 / rate_hz), self._on_flight_timer)

    # ---- BOOT / 参数 ----

    def _boot(self):
        """读取并校验 mission.yaml。成功 -> WAIT_READY；失败 -> ERROR。"""
        self.config = None
        self.driver = None
        self.flight_enabled = False
        self.cancel_landing()
        try:
            self.config = load_config(self.config_path)
            self.interfaces.dry_run = bool(
                self.config.get("mission", {}).get("dry_run", True))
            self.flight_enabled = bool(
                self.config.get("mission", {}).get("flight_enabled", False))
            self._landing_confirmation = LandingConfirmation(
                self.config["landing"]["confirm_samples"])
            self.machine.handle_boot_params(True, "参数校验通过")
        except ConfigError as exc:
            rospy.logerr("mission.yaml 校验失败: %s", exc)
            self.machine.handle_boot_params(False, "参数无效：%s" % exc)
        self._publish_all()

    # ---- 订阅回调 ----

    def _cb_fcu_state(self, message):
        self.fcu_state = message
        self.fcu_received = rospy.Time.now()

    def _cb_extended(self, message):
        self.extended_state = message
        self.extended_received = rospy.Time.now()
        self.extended_sequence += 1

    def _cb_pose(self, message):
        self.pose = message
        self.pose_received = rospy.Time.now()

    def _cb_velocity(self, message):
        self.velocity = message
        self.velocity_received = rospy.Time.now()

    def _cb_estimator(self, message):
        self.estimator = message
        self.estimator_received = rospy.Time.now()

    def _cb_timesync(self, message):
        self.timesync = message
        self.timesync_received = rospy.Time.now()

    def _cb_vision_healthy(self, message):
        self.vision_healthy = bool(message.data)
        self.vision_healthy_received = rospy.Time.now()

    def _cb_vision_state(self, message):
        self.vision_state = message.data
        self.vision_state_received = rospy.Time.now()

    def _cb_tag(self, message):
        self.tag = message
        self.tag_received = rospy.Time.now()

    def _cb_payload_release_confirmed(self, message):
        self.payload_release_confirmed = bool(message.data)
        self.payload_release_received = rospy.Time.now()
        self.payload_release_sequence += 1

    # ---- 安全门 ----

    def _evaluate_preconditions(self):
        """计算前置安全门。返回 (全部通过, 失败原因列表)。"""
        # config.validate 已保证 timing / topic_timeout / limits 各键存在，直接下标
        # 访问：缺键时 loud KeyError，而非静默采用代码内现场常量（R4-4）。
        timing = self.config["timing"]
        timeout = timing["topic_timeout"]
        checks = []

        if self.fcu_state is None:
            checks.append((False, "未收到飞控状态"))
        else:
            checks.append(guards.fcu_connected(self.fcu_state.connected))
            checks.append(guards.not_armed(self.fcu_state.armed))
            checks.append(guards.topic_fresh(
                self._topic_age(self.fcu_received),
                timeout["fcu_state"], label="飞控状态"))
        if self.extended_state is None:
            checks.append((False, "未收到落地状态"))
        else:
            checks.append(guards.on_ground(self.extended_state.landed_state))
            checks.append(guards.topic_fresh(
                self._topic_age(self.extended_received),
                timeout["extended_state"], label="落地状态"))
        if self.pose is None:
            checks.append((False, "未收到本地位姿"))
        else:
            position = (self.pose.pose.position.x,
                        self.pose.pose.position.y,
                        self.pose.pose.position.z)
            # home 未捕获，场地边界无 map 锚点（§16.10）：只验新鲜度 + 有限数
            checks.append(guards.pose_fresh(
                position, self._topic_age(self.pose_received),
                timing["pose_timeout"]))
        if self.estimator is None:
            checks.append((False, "未收到估计器状态"))
        else:
            checks.append(guards.estimator_ok({
                "attitude": self.estimator.attitude_status_flag,
                "pos_horiz_rel": self.estimator.pos_horiz_rel_status_flag,
                "pos_vert_abs": self.estimator.pos_vert_abs_status_flag,
            }))
            checks.append(guards.topic_fresh(
                self._topic_age(self.estimator_received),
                timeout["estimator_status"], label="估计器状态"))
        if self.timesync is None:
            checks.append((False, "未收到时间同步"))
        else:
            checks.append(guards.timesync_ok(
                self.timesync.round_trip_time_ms,
                timing["max_rtt_ms"]))
            checks.append(guards.topic_fresh(
                self._topic_age(self.timesync_received),
                timeout["timesync_status"], label="时间同步"))
        if self.vision_healthy is None or self.vision_state is None:
            checks.append((False, "未收到外部视觉"))
        else:
            checks.append(guards.topic_fresh(
                self._topic_age(self.vision_healthy_received),
                timeout["vision_healthy"], label="视觉健康"))
            checks.append(guards.topic_fresh(
                self._topic_age(self.vision_state_received),
                timeout["vision_state"], label="视觉状态"))
            checks.append(guards.vision_healthy(self.vision_healthy,
                                                self.vision_state))
        ok, failed = guards.readiness(checks)
        return ok, failed

    def _topic_age(self, received):
        """最近一次到达时刻距今秒数；尚未收到则返回 inf。"""
        if received is None:
            return float("inf")
        try:
            return (rospy.Time.now() - received).to_sec()
        except Exception:
            return float("inf")

    def topic_age(self, name):
        """FlightDriver 用：按名字取话题到达时刻龄（秒）。"""
        received = getattr(self, {"pose": "pose_received",
                                  "fcu_state": "fcu_received",
                                  "extended_state": "extended_received",
                                  "velocity": "velocity_received",
                                  "timesync": "timesync_received",
                                  "vision_healthy": "vision_healthy_received",
                                  "tag": "tag_received"
                                  }.get(name, ""), None)
        return self._topic_age(received)

    def payload_confirmed_after(self, sequence):
        return (self.payload_release_confirmed and
                self.payload_release_sequence > sequence)

    # ---- 服务 ----

    def _on_start(self, unused_request):
        del unused_request
        with self._lock:
            # start 的接受依据必须是最新前置条件，而非最近一次 0.2s 定时器的结果
            # （≤0.2s 陈旧）；飞行轮 start 触发 TAKEOFF 前必须消除（R5-3）。
            if self.config is None:
                return TriggerResponse(success=False, message="参数未加载")
            if self.machine.state in MissionState.PRE_START:
                ok, failed = self._evaluate_preconditions()
                reason = "；".join(failed) if failed else "前置条件满足"
                self.machine.handle_preconditions(ok, reason)
                if ok and self.flight_enabled:
                    if not self.config["payload"]["enable"]:
                        self._publish_all()
                        return TriggerResponse(
                            success=False,
                            message="正式任务禁止 payload.enable=false")
                    if self.payload_release_confirmed:
                        self._publish_all()
                        return TriggerResponse(
                            success=False,
                            message="投放机构已处于释放确认状态，请重新挂载后再启动")
                    if self.pose is None:
                        self._publish_all()
                        return TriggerResponse(success=False,
                                              message="本地位姿不可用，无法启动")
                    home_xyz, home_yaw = self._capture_home()
            success, message = self.machine.request_start(
                flight_enabled=self.flight_enabled)
            if success and self.flight_enabled:
                # 进入 TAKEOFF：用捕获的 home 构造飞行驱动（起飞点局部归零）
                self.driver = FlightDriver(self, self.machine,
                                           self.interfaces, self.coord,
                                           self.config)
                self.driver.start(home_xyz, home_yaw)
            self._publish_all()
        return TriggerResponse(success=success, message=message)

    def _on_stop(self, unused_request):
        del unused_request
        with self._lock:
            success, message = self.machine.request_stop()
            if self.machine.state == MissionState.ABORT_LAND:
                self.begin_landing()
            self._publish_all()
        return TriggerResponse(success=success, message=message)

    def _on_reset(self, unused_request):
        del unused_request
        with self._lock:
            success, message = self.machine.request_reset()
            if success:
                self.driver = None
                self.cancel_landing()
            if self.machine.state == MissionState.BOOT:
                # ERROR -> BOOT：从磁盘重读参数（BOOT 可重入）
                self._boot()
            self._publish_all()
        return TriggerResponse(success=success, message=message)

    # ---- 周期任务与发布 ----

    def _on_timer(self, unused_event):
        del unused_event
        with self._lock:
            if self.config is None:
                self._publish_all()
                return
            if self.machine.state in MissionState.PRE_START:
                ok, failed = self._evaluate_preconditions()
                reason = "；".join(failed) if failed else "前置条件满足"
                self.machine.handle_preconditions(ok, reason)
            self._publish_all()

    def _on_flight_timer(self, unused_event):
        """20Hz 飞行控制：飞行推进及可确认的 AUTO.LAND 握手。"""
        del unused_event
        with self._lock:
            if self.config is None:
                return
            if (self._manual_mode_active() and not
                    (self.driver is not None and
                     self.driver.awaiting_offboard_confirmation())):
                self.machine.confirm_manual_takeover()
                self.cancel_landing()
                self._publish_all()
                return
            if self.driver is not None and self.machine.state in MissionState.FLIGHT:
                self.driver.tick()
            if self.machine.state in (MissionState.LAND,
                                      MissionState.ABORT_LAND):
                self._drive_landing()
            self._publish_all()

    def _on_manual_takeover(self, unused_request):
        """人工接管确认：任一飞行态立即停止 mission 自动控制。"""
        del unused_request
        with self._lock:
            if self.machine.state not in (MissionState.FLIGHT +
                                          (MissionState.ABORT_LAND,)):
                return TriggerResponse(
                    success=False,
                    message="当前状态不可人工接管：" + self.machine.state)
            self.machine.confirm_manual_takeover()
            self.cancel_landing()
            self._publish_all()
        return TriggerResponse(success=True, message="已确认人工接管")

    # ---- AUTO.LAND 与落地确认 ----

    def begin_landing(self, target=None):
        """进入 LAND/ABORT_LAND 时保存当前点，直到 FCU 确认 AUTO.LAND 前持续保持。"""
        if self._landing_active:
            return
        if self.pose is not None:
            position = self.pose.pose.position
            target = ((position.x, position.y, position.z), self.coord.home_yaw)
        if target is None:
            target = ((0.0, 0.0, 0.0), self.coord.home_yaw)
        self._landing_active = True
        self._landing_target = target
        self._landing_started = rospy.Time.now()
        self._landing_last_request = None
        self._landing_requests = 0
        self._landing_mode_confirmed = False
        self._landing_confirmation_started = False
        self._landing_failure_reported = False
        if self._landing_confirmation is not None:
            self._landing_confirmation.reset()

    def cancel_landing(self):
        self._landing_active = False
        self._landing_target = None
        self._landing_started = None
        self._landing_last_request = None
        self._landing_requests = 0
        self._landing_mode_confirmed = False
        self._landing_confirmation_started = False
        self._landing_failure_reported = False
        if self._landing_confirmation is not None:
            self._landing_confirmation.reset()

    def _manual_mode_active(self):
        return (self.fcu_state is not None and
                guards.manual_mode_active(self.fcu_state.mode,
                                          self.fcu_state.armed) and
                self.machine.state in (MissionState.FLIGHT +
                                       (MissionState.ABORT_LAND,)))

    def _drive_landing(self):
        if not self._landing_active:
            self.begin_landing()
        landing = self.config["landing"]
        fcu_fresh = (self.fcu_state is not None and
                     self.topic_age("fcu_state") <=
                     self.config["timing"]["topic_timeout"]["fcu_state"])
        mode_confirmed = (fcu_fresh and self.fcu_state.mode == "AUTO.LAND")
        if mode_confirmed:
            self._landing_mode_confirmed = True
        if not self._landing_mode_confirmed:
            self.interfaces.send_position(
                self._landing_target[0], self._landing_target[1],
                self.config["frames"]["mission_frame"])
            now = rospy.Time.now()
            due = (self._landing_last_request is None or
                   (now - self._landing_last_request).to_sec() >=
                   landing["mode_retry_seconds"])
            if due and self._landing_requests < landing["mode_retry_count"]:
                if not self._landing_confirmation_started:
                    self._landing_confirmation.begin(self.extended_sequence)
                    self._landing_confirmation_started = True
                self._landing_last_request = now
                self._landing_requests += 1
                ok, reason = self.interfaces.set_mode("AUTO.LAND")
                if not ok:
                    rospy.logwarn("AUTO.LAND 请求 %d/%d 失败: %s",
                                  self._landing_requests,
                                  landing["mode_retry_count"], reason)
            self._landing_timeout_if_needed("AUTO.LAND 未获 FCU 确认")
            return
        if not self._landing_confirmation_started:
            self._landing_confirmation.begin(self.extended_sequence)
            self._landing_confirmation_started = True
        pose_z = None
        if self.pose is not None and self.coord.ready:
            pose_z = self.coord.map_to_field(
                (self.pose.pose.position.x, self.pose.pose.position.y,
                 self.pose.pose.position.z))[2]
        velocity_z = (None if self.velocity is None else
                      self.velocity.twist.linear.z)
        landed = self._landing_confirmation.observe(
            self.extended_sequence,
            self.extended_state is not None and
            self.extended_state.landed_state ==
            ExtendedState.LANDED_STATE_ON_GROUND,
            self.topic_age("extended_state"),
            self.config["timing"]["topic_timeout"]["extended_state"],
            self.fcu_state.armed, pose_z, self.topic_age("pose"),
            self.config["timing"]["pose_timeout"], velocity_z,
            self.topic_age("velocity"), landing["velocity_timeout"],
            landing["max_height"], landing["max_vertical_speed"])
        if landed:
            if self.machine.state == MissionState.LAND:
                self.machine.stage_done()
            else:
                self.machine.confirm_landed()
            self.cancel_landing()
            return
        self._landing_timeout_if_needed("落地确认超时")

    def _landing_timeout_if_needed(self, reason):
        if self._landing_started is None or self._landing_failure_reported:
            return
        elapsed = (rospy.Time.now() - self._landing_started).to_sec()
        if elapsed <= self.config["timing"]["land_confirm"]:
            return
        self._landing_failure_reported = True
        if self.machine.state == MissionState.LAND:
            self.machine.abort(reason + "，请立即人工接管")
        rospy.logerr("%s；保持当前位置 setpoint，等待人工接管", reason)

    def _capture_home(self):
        """起飞时刻 map 位姿 + 偏航（起飞点局部归零锚点）。"""
        position = self.pose.pose.position
        yaw = self._yaw_from_quaternion(self.pose.pose.orientation)
        return (position.x, position.y, position.z), yaw

    @staticmethod
    def _yaw_from_quaternion(orientation):
        return math.atan2(
            2.0 * (orientation.w * orientation.z +
                   orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2))

    def _publish_all(self):
        self.state_pub.publish(String(data=self.machine.state))
        self.reason_pub.publish(String(data=self.machine.reason))
        self.active_pub.publish(Bool(data=self.machine.active))
        self.result_pub.publish(String(data=self.machine.result))
        # 飞行态下目标预览由 FlightDriver 转发（当前实际 setpoint）；此处只发预启动名义目标
        if self.machine.state not in MissionState.FLIGHT:
            target = self._current_target()
            if target is not None:
                self.interfaces.publish_target(target)

    def publish_all(self):
        """FlightDriver 状态推进后调用的公共发布入口（委托内部实现）。"""
        self._publish_all()

    def _current_target(self):
        """骨架轮：发布起飞点作为名义目标；飞行轮改为当前计算航点。"""
        if self.config is None:
            return None
        takeoff = self.config.get("waypoints", {}).get("takeoff")
        if takeoff is None:
            return None
        pose = PoseStamped()
        pose.header.frame_id = self.config["frames"]["mission_frame"]
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = float(takeoff[0])
        pose.pose.position.y = float(takeoff[1])
        pose.pose.position.z = float(takeoff[2])
        pose.pose.orientation.w = 1.0
        return pose


def main():
    rospy.init_node("robotac_mission")
    node = MissionNode()
    rospy.spin()


if __name__ == "__main__":
    main()
