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
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import EstimatorStatus, ExtendedState, State, TimesyncStatus
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, TriggerResponse

from robotac_mission import guards
from robotac_mission.config import ConfigError, load_config
from robotac_mission.coordinates import Coordinates
from robotac_mission.flight_driver import FlightDriver
from robotac_mission.interfaces import MissionInterfaces
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
        self.pose = None
        self.pose_received = None
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
        self._window_started = None   # 6 分钟共享窗口起点（首次 start）
        self._abort_land_issued = False
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
        self._abort_land_issued = False
        try:
            self.config = load_config(self.config_path)
            self.interfaces.dry_run = bool(
                self.config.get("mission", {}).get("dry_run", True))
            self.flight_enabled = bool(
                self.config.get("mission", {}).get("flight_enabled", False))
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

    def _cb_pose(self, message):
        self.pose = message
        self.pose_received = rospy.Time.now()

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
                                  "timesync": "timesync_received",
                                  "vision_healthy": "vision_healthy_received",
                                  "tag": "tag_received"
                                  }.get(name, ""), None)
        return self._topic_age(received)

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
                # 前置通过才查窗口：失败尝试不得消耗 6 分钟共享窗口（规则硬性）
                if ok and self.flight_enabled:
                    ok, reason = self._check_window()
                    if not ok:
                        self._publish_all()
                        return TriggerResponse(success=False, message=reason)
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
            self._publish_all()
        return TriggerResponse(success=success, message=message)

    def _on_reset(self, unused_request):
        del unused_request
        with self._lock:
            success, message = self.machine.request_reset()
            if success:
                self.driver = None
                self._abort_land_issued = False
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
            elif self.machine.state == MissionState.ABORT_LAND:
                # 中止落地确认 -> COMPLETE（首个根因保留在 result）
                if (self.extended_state is not None and
                        self.extended_state.landed_state ==
                        ExtendedState.LANDED_STATE_ON_GROUND):
                    self.machine.confirm_landed()
            self._publish_all()

    def _on_flight_timer(self, unused_event):
        """20Hz 飞行控制：飞行态逐态推进；ABORT_LAND 触发放下 AUTO.LAND。"""
        del unused_event
        with self._lock:
            if self.config is None or self.driver is None:
                return
            if self.machine.state in MissionState.FLIGHT:
                self.driver.tick()
            elif (self.machine.state == MissionState.ABORT_LAND and
                    self.machine.active and not self._abort_land_issued):
                self._abort_land_issued = True
                ok, reason = self.interfaces.set_mode("AUTO.LAND")
                if not ok:
                    rospy.logwarn("中止降落模式请求失败: %s", reason)

    def _check_window(self):
        """6 分钟共享窗口：首次 start 记起点，此后校验剩余 ≥ flight_budget（M1）。"""
        timing = self.config["timing"]
        if self._window_started is None:
            self._window_started = rospy.Time.now()
            return True, ""
        used = (rospy.Time.now() - self._window_started).to_sec()
        return guards.window_ok(used, timing["total_window"],
                                timing["flight_budget"])

    def _on_manual_takeover(self, unused_request):
        """人工接管确认：仅 ABORT_LAND 态可用（空中断连/AUTO.LAND 失败时
        操作员接管后的唯一出口，M4）。"""
        del unused_request
        with self._lock:
            if self.machine.state != MissionState.ABORT_LAND:
                return TriggerResponse(
                    success=False,
                    message="当前状态不可人工接管：" + self.machine.state)
            self.machine.confirm_manual_takeover()
            self._publish_all()
        return TriggerResponse(success=True, message="已确认人工接管")

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
