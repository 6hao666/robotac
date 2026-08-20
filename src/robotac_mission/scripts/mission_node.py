#!/usr/bin/env python3
"""robotac_mission 状态机节点壳（骨架）。

启动后仅进入 WAIT_READY / WAIT_START，不产生任何飞行动作（dry_run）。
订阅传感器/飞控话题 -> 计算安全门 -> 驱动 state_machine -> 发布
state / state_reason / active / target / result，并暴露 start / stop / reset 服务。

BOOT 可重入：mission_reset（ERROR -> BOOT）会重新从磁盘读取 mission.yaml，
不得复用进程内存中缓存的旧参数。
"""

import threading

import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import EstimatorStatus, ExtendedState, State, TimesyncStatus
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, TriggerResponse

from robotac_mission import guards
from robotac_mission.config import ConfigError, load_config
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
        self._boot()
        rospy.Timer(rospy.Duration(0.2), self._on_timer)

    # ---- BOOT / 参数 ----

    def _boot(self):
        """读取并校验 mission.yaml。成功 -> WAIT_READY；失败 -> ERROR。"""
        self.config = None
        try:
            self.config = load_config(self.config_path)
            self.interfaces.dry_run = bool(
                self.config.get("mission", {}).get("dry_run", True))
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
            checks.append(guards.pose_valid(
                position, self._topic_age(self.pose_received),
                self.config["limits"]["field_min"],
                self.config["limits"]["field_max"],
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

    # ---- 服务 ----

    def _on_start(self, unused_request):
        del unused_request
        with self._lock:
            # start 的接受依据必须是最新前置条件，而非最近一次 0.2s 定时器的结果
            # （≤0.2s 陈旧）；飞行轮 start 触发 TAKEOFF 前必须消除（R5-3）。
            if self.config is not None:
                ok, failed = self._evaluate_preconditions()
                reason = "；".join(failed) if failed else "前置条件满足"
                self.machine.handle_preconditions(ok, reason)
            success, message = self.machine.request_start()
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
            if self.machine.state == MissionState.BOOT:
                # ERROR -> BOOT：从磁盘重读参数（BOOT 可重入）
                self._boot()
            self._publish_all()
        return TriggerResponse(success=success, message=message)

    # ---- 周期任务与发布 ----

    def _on_timer(self, unused_event):
        del unused_event
        with self._lock:
            if self.config is not None:
                ok, failed = self._evaluate_preconditions()
                reason = "；".join(failed) if failed else "前置条件满足"
                self.machine.handle_preconditions(ok, reason)
            self._publish_all()

    def _publish_all(self):
        self.state_pub.publish(String(data=self.machine.state))
        self.reason_pub.publish(String(data=self.machine.reason))
        self.active_pub.publish(Bool(data=self.machine.active))
        self.result_pub.publish(String(data=self.machine.result))
        target = self._current_target()
        if target is not None:
            self.interfaces.publish_target(target)

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
