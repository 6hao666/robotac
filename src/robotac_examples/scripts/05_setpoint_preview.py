#!/usr/bin/env python3
"""示例 05：生成目标位姿预览，不向 MAVROS 发布。"""

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String

from robotac_examples.ros_messages import make_pose


def main():
    rospy.init_node("setpoint_preview_example")
    target = [float(rospy.get_param("~x", 0.5)),
              float(rospy.get_param("~y", 0.0)),
              float(rospy.get_param("~z", 0.6)),
              float(rospy.get_param("~yaw", 0.0))]
    publisher = rospy.Publisher("~target", PoseStamped, queue_size=1,
                                latch=True)
    state_pub = rospy.Publisher("~state", String, queue_size=1, latch=True)
    active_pub = rospy.Publisher("~active", Bool, queue_size=1, latch=True)
    state_pub.publish(String(data="PREVIEW"))
    active_pub.publish(Bool(data=False))
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        publisher.publish(make_pose("map", rospy.Time.now(), target))
        rospy.loginfo_throttle(5.0, "预览目标: x=%.2f y=%.2f z=%.2f",
                               target[0], target[1], target[2])
        rate.sleep()


if __name__ == "__main__":
    main()
