"""基础飞行示例共用的 MAVROS 接口和安全门禁。"""

import math
import time

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import ExtendedState
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, TriggerResponse

from robotac_examples.flight_inputs import FlightInputs
from robotac_examples.flight_safety import FlightSafety
from robotac_examples.geometry import distance3, yaw_from_quaternion
from robotac_examples.ros_messages import make_pose


class FlightController(FlightInputs):
    def __init__(self):
        self.start_requested = False
        self.stop_requested = False
        self.used = False
        self.active = False
        self.landing = False
        self.target = None
        self.origin = None
        self.origin_yaw = 0.0
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.rate_hz = float(rospy.get_param("~rate", 20.0))
        self.pose_timeout = float(rospy.get_param("~pose_timeout", 0.5))
        # 2026-08-21 修复：/mavros/state 与 estimator_status 按 1Hz 发布，
        # 默认 1.0s 等于消息间隔，任何抖动都误判"过期"而中止（06 第 2 次尝试踩到）。
        # 默认提到 3.0s；启动该参数的 launch 可覆盖。仅瞬态门受益，硬安全门不受影响。
        self.data_timeout = float(rospy.get_param("~data_timeout", 3.0))
        self.max_xy = float(rospy.get_param("~max_xy", 2.0))
        self.max_z = float(rospy.get_param("~max_z", 2.0))
        self.position_tolerance = float(rospy.get_param(
            "~position_tolerance", 0.15))
        # 事故后安全保险（防撞墙/防漂移/起飞前稳定性）拆到 flight_safety.py，
        # 避免本模块超过 check_source 250 行红线。
        self.safety = FlightSafety(self)
        FlightInputs.__init__(self, self.pose_timeout, self.data_timeout)

        self.setpoint_pub = rospy.Publisher(
            "/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.state_pub = rospy.Publisher("~state", String, queue_size=1,
                                         latch=True)
        self.active_pub = rospy.Publisher("~active", Bool, queue_size=1,
                                          latch=True)
        self.target_pub = rospy.Publisher("~target", PoseStamped, queue_size=5)
        self.mode_service = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.arm_service = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.start_service = rospy.Service("~start", Trigger, self._start_cb)
        self.stop_service = rospy.Service("~stop", Trigger, self._stop_cb)
        self._publish_state("IDLE", False)

    def readiness_issue(self):
        connections = self.setpoint_pub.get_num_connections()
        return FlightInputs.readiness_issue(self, connections)

    def health_issue(self):
        connections = self.setpoint_pub.get_num_connections()
        return FlightInputs.health_issue(
            self, self.stop_requested, connections)

    def _start_cb(self, unused_request):
        del unused_request
        if self.used or self.start_requested:
            return TriggerResponse(False, "本次节点运行已经使用")
        issue = self.readiness_issue()
        if issue is not None:
            return TriggerResponse(False, issue)
        self.start_requested = True
        return TriggerResponse(True, "已接受启动请求")

    def _stop_cb(self, unused_request):
        del unused_request
        if (not self.active and not self.start_requested) or self.landing:
            return TriggerResponse(False, "当前没有活动任务")
        self.stop_requested = True
        return TriggerResponse(True, "已接受停止请求")

    def _publish_state(self, state, active):
        self.state_pub.publish(String(data=state))
        self.active_pub.publish(Bool(data=active))

    def wait_for_start(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and not self.start_requested:
            rate.sleep()
        if rospy.is_shutdown():
            return False
        self.start_requested = False
        self.used = True
        self.active = True
        position = self.pose.pose.position
        self.origin = [position.x, position.y, position.z]
        self.origin_yaw = yaw_from_quaternion(self.pose.pose.orientation)
        self._publish_state("PRESTREAM", True)
        return True

    def validate_target(self, target):
        for value in target:
            if not math.isfinite(value):
                raise ValueError("目标包含非有限数值")
        dx = target[0] - self.origin[0]
        dy = target[1] - self.origin[1]
        dz = target[2] - self.origin[2]
        if math.hypot(dx, dy) > self.max_xy or abs(dz) > self.max_z:
            raise ValueError("目标超出示例软件边界")

    def publish_target(self, target):
        self.validate_target(target)
        # 保险：距起飞点过远立即中止（发布目标本身也可能带偏）。
        if self.safety.too_far():
            raise ValueError("超过最大飞行距离")
        self.target = target
        message = make_pose(self.frame_id, rospy.Time.now(), target)
        self.setpoint_pub.publish(message)
        self.target_pub.publish(message)

    def _publish_for(self, target, seconds):
        end_time = time.monotonic() + seconds
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown() and time.monotonic() < end_time:
            issue = self.health_issue()
            if issue is not None:
                return self.fail(issue)
            try:
                self.publish_target(target)
            except ValueError as error:
                return self.fail(str(error))
            rate.sleep()
        return not rospy.is_shutdown()

    def begin(self, height):
        # 保险：起飞前确认位置估计稳定，防止带漂移定位起飞（2026-08-23 事故）。
        if not self.safety.check_ground_stable():
            return self.fail("起飞前位置估计漂移，定位不稳定")
        target = [self.origin[0], self.origin[1],
                  self.origin[2] + height, self.origin_yaw]
        if not self._publish_for(target, 2.0):
            return False
        try:
            mode_ok = self.mode_service(
                base_mode=0, custom_mode="OFFBOARD").mode_sent
            if not mode_ok:
                return self.fail("OFFBOARD 请求失败")
            if not self.arm_service(value=True).success:
                return self.fail("解锁请求失败")
        except rospy.ServiceException as error:
            return self.fail("飞控服务调用失败: %s" % error)
        self._publish_state("TAKEOFF", True)
        return self.wait_target(target, 30.0, 2.0)

    def wait_target(self, target, timeout, hold_seconds):
        start = time.monotonic()
        reached_since = None
        previous = None
        previous_time = None
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            issue = self.health_issue()
            if issue is not None:
                return self.fail(issue)
            try:
                self.publish_target(target)
            except ValueError as error:
                return self.fail(str(error))
            # 保险：位置估计速度异常（定位跳变/漂移带飞）。
            now = time.monotonic()
            if previous is not None and previous_time is not None:
                if self.safety.too_fast(previous, now - previous_time):
                    return self.fail("位置估计异常快速移动")
            if self.pose is not None:
                previous = [self.pose.pose.position.x,
                            self.pose.pose.position.y]
            previous_time = now
            position = self.pose.pose.position
            current = [position.x, position.y, position.z]
            if distance3(current, target[:3]) <= self.position_tolerance:
                if reached_since is None:
                    reached_since = time.monotonic()
                if time.monotonic() - reached_since >= hold_seconds:
                    return True
            else:
                reached_since = None
            if time.monotonic() - start > timeout:
                return self.fail("目标到达超时")
            rate.sleep()
        return False

    def finish(self, final_state="COMPLETE"):
        if self.landing or not self.active:
            return False
        self.landing = True
        self._publish_state("LANDING", True)
        try:
            self.mode_service(base_mode=0, custom_mode="AUTO.LAND")
        except rospy.ServiceException:
            rospy.logerr("AUTO.LAND 服务调用失败")
        end_time = time.monotonic() + 30.0
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown() and time.monotonic() < end_time:
            if self.fresh("extended"):
                if self.extended.landed_state == ExtendedState.LANDED_STATE_ON_GROUND:
                    self.active = False
                    self._publish_state(final_state, False)
                    return True
            rate.sleep()
        self.active = False
        self._publish_state("ABORT", False)
        return False

    def fail(self, reason):
        rospy.logerr("示例中止: %s", reason)
        if self.active and not self.landing:
            self._publish_state("ABORT", True)
            self.finish("ABORT")
        return False
