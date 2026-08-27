#!/usr/bin/env python3
"""示例 02：只读显示定位链路和 MAVROS 本地位置。"""

import time

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import EstimatorStatus
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


class LocalPoseMonitor(object):
    def __init__(self):
        self.received = {}
        self.local_count = 0
        self.local_pose = None
        self.bridge_healthy = False
        self.estimator = None
        self.window_start = time.monotonic()
        rospy.Subscriber("/sunray/odometry", Odometry,
                         self._odom_cb, queue_size=20)
        rospy.Subscriber("/vision_pose_bridge/healthy", Bool,
                         self._bridge_cb, queue_size=10)
        rospy.Subscriber("/mavros/estimator_status", EstimatorStatus,
                         self._estimator_cb, queue_size=10)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped,
                         self._pose_cb, queue_size=20)
        self.report_timer = rospy.Timer(rospy.Duration(1.0), self._report)

    def _mark(self, name):
        self.received[name] = time.monotonic()

    def _odom_cb(self, unused_message):
        del unused_message
        self._mark("odometry")

    def _bridge_cb(self, message):
        self.bridge_healthy = bool(message.data)
        self._mark("bridge")

    def _estimator_cb(self, message):
        self.estimator = message
        self._mark("estimator")

    def _pose_cb(self, message):
        self.local_count += 1
        self.local_pose = message
        self._mark("local_pose")

    def _fresh(self, name, timeout=1.0):
        received = self.received.get(name)
        return received is not None and time.monotonic() - received <= timeout

    def _estimator_ready(self):
        if self.estimator is None or not self._fresh("estimator"):
            return False
        return bool(self.estimator.attitude_status_flag and
                    self.estimator.pos_horiz_rel_status_flag and
                    (self.estimator.pos_vert_abs_status_flag or
                     self.estimator.pos_vert_agl_status_flag))

    def _report(self, unused_event):
        del unused_event
        odometry_ok = self._fresh("odometry")
        bridge_ok = self._fresh("bridge") and self.bridge_healthy
        estimator_ok = self._estimator_ready()
        local_ok = self._fresh("local_pose")
        rospy.loginfo(
            "定位链路: 修正里程计=%s，外部视觉转发=%s，PX4 位置状态=%s，"
            "本地位姿=%s",
            self._word(odometry_ok), self._word(bridge_ok),
            self._word(estimator_ok), self._word(local_ok))
        if not local_ok or self.local_pose is None:
            self.local_count = 0
            self.window_start = time.monotonic()
            return
        elapsed = time.monotonic() - self.window_start
        message = self.local_pose
        position = message.pose.position
        orientation = message.pose.orientation
        rate = self.local_count / elapsed if elapsed > 0.0 else 0.0
        age = (rospy.Time.now() - message.header.stamp).to_sec()
        rospy.loginfo(
            "本地位姿: 位置(%.3f, %.3f, %.3f)，四元数(%.3f, %.3f, %.3f, %.3f)，"
            "频率 %.1f Hz，数据年龄 %.3f s",
            position.x, position.y, position.z,
            orientation.x, orientation.y, orientation.z, orientation.w,
            rate, age)
        self.local_count = 0
        self.window_start = time.monotonic()

    @staticmethod
    def _word(value):
        if value:
            return "正常"
        return "未就绪"


def main():
    rospy.init_node("local_pose_example")
    LocalPoseMonitor()
    rospy.spin()


if __name__ == "__main__":
    main()
