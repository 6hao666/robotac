#!/usr/bin/env python3
"""示例 09：计算 AprilTag 对准目标，但不控制飞机。"""

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String

from robotac_examples.geometry import yaw_from_quaternion
from robotac_examples.ros_messages import make_pose
from robotac_examples.tag import TagTracker


def main():
    rospy.init_node("tag_centering_preview_example")
    tracker = TagTracker(int(rospy.get_param("~tag_id", 0)), "map")
    publisher = rospy.Publisher("~target", PoseStamped, queue_size=5)
    state_pub = rospy.Publisher("~state", String, queue_size=1, latch=True)
    active_pub = rospy.Publisher("~active", Bool, queue_size=1, latch=True)
    state_pub.publish(String(data="WAITING_TAG"))
    active_pub.publish(Bool(data=False))
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if tracker.fresh() and tracker.vehicle_pose is not None:
            tag = tracker.pose.pose.position
            vehicle = tracker.vehicle_pose.pose
            yaw = yaw_from_quaternion(vehicle.orientation)
            target = [tag.x, tag.y, vehicle.position.z, yaw]
            publisher.publish(make_pose("map", rospy.Time.now(), target))
            state_pub.publish(String(data="PREVIEW"))
            rospy.loginfo_throttle(2.0, "对准目标: x=%.3f y=%.3f", tag.x, tag.y)
        else:
            state_pub.publish(String(data="WAITING_TAG"))
            rospy.loginfo_throttle(2.0, "等待稳定的 Tag 和本地位置")
        rate.sleep()


if __name__ == "__main__":
    main()
