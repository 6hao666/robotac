#!/usr/bin/env python3
"""示例 02：只读显示 MAVROS 本地位置和接收频率。"""

import time

import rospy
from geometry_msgs.msg import PoseStamped


class LocalPoseMonitor(object):
    def __init__(self):
        self.count = 0
        self.window_start = time.monotonic()
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped,
                         self._pose_cb, queue_size=20)

    def _pose_cb(self, message):
        self.count += 1
        elapsed = time.monotonic() - self.window_start
        if elapsed < 1.0:
            return
        position = message.pose.position
        orientation = message.pose.orientation
        rate = self.count / elapsed
        age = (rospy.Time.now() - message.header.stamp).to_sec()
        rospy.loginfo(
            "本地位姿: 位置(%.3f, %.3f, %.3f)，四元数(%.3f, %.3f, %.3f, %.3f)，"
            "频率 %.1f Hz，数据年龄 %.3f s",
            position.x, position.y, position.z,
            orientation.x, orientation.y, orientation.z, orientation.w,
            rate, age)
        self.count = 0
        self.window_start = time.monotonic()


def main():
    rospy.init_node("local_pose_example")
    LocalPoseMonitor()
    rospy.spin()


if __name__ == "__main__":
    main()
