#!/usr/bin/env python3
"""教学示例测试使用的简化假飞控，不连接任何硬件。"""

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import EstimatorStatus, ExtendedState, State, TimesyncStatus
from mavros_msgs.srv import CommandBool, CommandBoolResponse
from mavros_msgs.srv import SetMode, SetModeResponse
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolResponse


class FakeFcu(object):
    def __init__(self):
        self.state = State(connected=True, armed=False, mode="MANUAL")
        self.extended = ExtendedState(
            landed_state=ExtendedState.LANDED_STATE_ON_GROUND)
        self.pose = PoseStamped()
        self.pose.header.frame_id = "map"
        self.pose.pose.orientation.w = 1.0
        self.localization_ok = True
        self.estimator_ok = True
        self.pose_stream = True
        self.setpoint_count = 0
        self.state_pub = rospy.Publisher("/mavros/state", State, queue_size=5)
        self.extended_pub = rospy.Publisher("/mavros/extended_state",
                                            ExtendedState, queue_size=5)
        self.pose_pub = rospy.Publisher("/mavros/local_position/pose",
                                        PoseStamped, queue_size=10)
        self.estimator_pub = rospy.Publisher("/mavros/estimator_status",
                                             EstimatorStatus, queue_size=5)
        self.timesync_pub = rospy.Publisher("/mavros/timesync_status",
                                            TimesyncStatus, queue_size=5)
        self.health_pub = rospy.Publisher("/vision_pose_bridge/healthy",
                                          Bool, queue_size=1, latch=True)
        self.count_pub = rospy.Publisher("/robotac_test/setpoint_seen",
                                         Bool, queue_size=1, latch=True)
        rospy.Subscriber("/mavros/setpoint_position/local", PoseStamped,
                         self._setpoint_cb, queue_size=20)
        rospy.Service("/mavros/set_mode", SetMode, self._set_mode)
        rospy.Service("/mavros/cmd/arming", CommandBool, self._arm)
        rospy.Service("/robotac_test/set_on_ground", SetBool,
                      self._set_on_ground)
        rospy.Service("/robotac_test/set_localization", SetBool,
                      self._set_localization)
        rospy.Service("/robotac_test/set_estimator", SetBool,
                      self._set_estimator)
        rospy.Service("/robotac_test/set_pose_stream", SetBool,
                      self._set_pose_stream)
        rospy.Timer(rospy.Duration(0.05), self._publish)

    def _setpoint_cb(self, message):
        self.setpoint_count += 1
        self.pose.pose = message.pose
        self.count_pub.publish(Bool(data=True))

    def _set_mode(self, request):
        self.state.mode = request.custom_mode
        if request.custom_mode == "AUTO.LAND":
            self.state.armed = False
            self.pose.pose.position.z = 0.0
            self.extended.landed_state = ExtendedState.LANDED_STATE_ON_GROUND
        return SetModeResponse(mode_sent=True)

    def _arm(self, request):
        self.state.armed = bool(request.value)
        if request.value:
            self.extended.landed_state = ExtendedState.LANDED_STATE_IN_AIR
        return CommandBoolResponse(success=True, result=0)

    def _set_on_ground(self, request):
        if request.data:
            value = ExtendedState.LANDED_STATE_ON_GROUND
        else:
            value = ExtendedState.LANDED_STATE_IN_AIR
        self.extended.landed_state = value
        self.extended_pub.publish(self.extended)
        return SetBoolResponse(True, "已更新")

    def _set_localization(self, request):
        self.localization_ok = bool(request.data)
        return SetBoolResponse(True, "已更新")

    def _set_estimator(self, request):
        self.estimator_ok = bool(request.data)
        return SetBoolResponse(True, "已更新")

    def _set_pose_stream(self, request):
        self.pose_stream = bool(request.data)
        return SetBoolResponse(True, "已更新")

    def _publish(self, unused_event):
        del unused_event
        self.pose.header.stamp = rospy.Time.now()
        self.state_pub.publish(self.state)
        self.extended_pub.publish(self.extended)
        if self.pose_stream:
            self.pose_pub.publish(self.pose)
        estimator = EstimatorStatus()
        estimator.attitude_status_flag = self.estimator_ok
        estimator.pos_horiz_rel_status_flag = self.estimator_ok
        estimator.pos_vert_abs_status_flag = self.estimator_ok
        self.estimator_pub.publish(estimator)
        timesync = TimesyncStatus()
        timesync.round_trip_time_ms = 1.0
        self.timesync_pub.publish(timesync)
        self.health_pub.publish(Bool(data=self.localization_ok))
        if self.setpoint_count == 0:
            self.count_pub.publish(Bool(data=False))


def main():
    rospy.init_node("fake_fcu")
    FakeFcu()
    rospy.spin()


if __name__ == "__main__":
    main()
