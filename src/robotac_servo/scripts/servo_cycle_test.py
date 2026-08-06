#!/usr/bin/env python3
"""Publish the servo switch test sequence: 0 degrees for 1 s, open for 5 s."""

import rospy
from std_msgs.msg import Bool


def main() -> None:
    rospy.init_node("servo_cycle_test")
    topic = rospy.get_param("~topic", "/robotac/servo/open")
    closed_seconds = float(rospy.get_param("~closed_seconds", 1.0))
    open_seconds = float(rospy.get_param("~open_seconds", 5.0))
    publisher = rospy.Publisher(topic, Bool, queue_size=1, latch=True)

    while not rospy.is_shutdown() and publisher.get_num_connections() == 0:
        rospy.sleep(0.1)

    message = Bool()
    while not rospy.is_shutdown():
        message.data = False
        publisher.publish(message)
        rospy.loginfo("servo cycle: 0 degrees for %.1f s", closed_seconds)
        rospy.sleep(closed_seconds)

        message.data = True
        publisher.publish(message)
        rospy.loginfo("servo cycle: configured open angle for %.1f s", open_seconds)
        rospy.sleep(open_seconds)


if __name__ == "__main__":
    main()
