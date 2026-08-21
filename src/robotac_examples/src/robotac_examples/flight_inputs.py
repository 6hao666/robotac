"""收集飞行示例需要的 MAVROS 状态并检查数据时效。"""

import time

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import EstimatorStatus, ExtendedState, State, TimesyncStatus
from std_msgs.msg import Bool


class FlightInputs(object):
    def __init__(self, pose_timeout, data_timeout):
        self.pose_timeout = pose_timeout
        self.data_timeout = data_timeout
        # 2026-08-21 修复：health 检查对"瞬态类"问题（数据过期 / timesync RTT）
        # 做去抖——连续 health_debounce 次才中止，避免握手期间单次尖峰误报。
        # 硬安全项（估计器无效 / OFFBOARD 丢失 / 飞控断开）仍立即中止。
        self.health_debounce = int(rospy.get_param("~health_debounce", 5))
        self.soft_bad_count = 0
        self.state = State()
        self.extended = ExtendedState()
        self.pose = None
        self.estimator = None
        self.timesync = None
        self.localization_ok = False
        self.received = {}
        rospy.Subscriber("/mavros/state", State, self._state_cb, queue_size=10)
        rospy.Subscriber("/mavros/extended_state", ExtendedState,
                         self._extended_cb, queue_size=10)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped,
                         self._pose_cb, queue_size=20)
        rospy.Subscriber("/mavros/estimator_status", EstimatorStatus,
                         self._estimator_cb, queue_size=10)
        rospy.Subscriber("/mavros/timesync_status", TimesyncStatus,
                         self._timesync_cb, queue_size=10)
        rospy.Subscriber("/vision_pose_bridge/healthy", Bool,
                         self._localization_cb, queue_size=5)

    def _mark(self, name):
        self.received[name] = time.monotonic()

    def _state_cb(self, message):
        self.state = message
        self._mark("state")

    def _extended_cb(self, message):
        self.extended = message
        self._mark("extended")

    def _pose_cb(self, message):
        self.pose = message
        self._mark("pose")

    def _estimator_cb(self, message):
        self.estimator = message
        self._mark("estimator")

    def _timesync_cb(self, message):
        self.timesync = message
        self._mark("timesync")

    def _localization_cb(self, message):
        self.localization_ok = bool(message.data)
        self._mark("localization")

    def fresh(self, name, timeout=None):
        if timeout is None:
            timeout = self.data_timeout
        received = self.received.get(name)
        return received is not None and time.monotonic() - received <= timeout

    def data_issue(self):
        checks = [("state", "飞控状态"), ("extended", "落地状态"),
                  ("estimator", "PX4 estimator 状态"),
                  ("timesync", "时间同步状态"),
                  ("localization", "外部视觉状态")]
        for name, title in checks:
            if not self.fresh(name):
                return title + "过期"
        if not self.fresh("pose", self.pose_timeout):
            return "本地位置过期"
        if not self.state.connected:
            return "飞控未连接"
        if not self.localization_ok:
            return "外部视觉未就绪"
        if not self.estimator.attitude_status_flag:
            return "PX4 estimator 姿态无效"
        if not self.estimator.pos_horiz_rel_status_flag:
            return "PX4 estimator 水平位置无效"
        if not (self.estimator.pos_vert_abs_status_flag or
                self.estimator.pos_vert_agl_status_flag):
            return "PX4 estimator 垂直位置无效"
        if self.timesync.round_trip_time_ms > 20.0:
            return "时间同步往返延迟过大"
        return None

    def readiness_issue(self, setpoint_connections):
        issue = self.data_issue()
        if issue is not None:
            return issue
        if self.state.armed:
            return "飞机已经解锁"
        if self.extended.landed_state != ExtendedState.LANDED_STATE_ON_GROUND:
            return "飞机不在地面"
        if setpoint_connections < 1:
            return "MAVROS 未订阅位置设定点"
        return None

    def _soft_issue(self, issue):
        return issue.endswith("过期") or issue == "时间同步往返延迟过大"

    def health_issue(self, stop_requested, setpoint_connections):
        if stop_requested:
            return "操作员停止"
        issue = self.data_issue()
        if issue is not None:
            if self._soft_issue(issue):
                # 瞬态类（数据过期 / timesync RTT）：连续 health_debounce 次
                # 才上报中止，避免握手期间的瞬时尖峰误报。
                self.soft_bad_count += 1
                if self.soft_bad_count < self.health_debounce:
                    return None
                return issue
            # 硬安全项（估计器无效 / OFFBOARD 丢失 / 飞控断开等）：立即中止。
            self.soft_bad_count = 0
            return issue
        self.soft_bad_count = 0
        if self.state.armed and self.state.mode != "OFFBOARD":
            return "OFFBOARD 模式丢失"
        if setpoint_connections < 1:
            return "MAVROS 位置设定点订阅中断"
        return None
