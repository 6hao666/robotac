#!/usr/bin/env python3
"""Publish a deterministic relative PoseArray for the waypoint API test.

This helper only sends the route description topic. It never calls a flight
service and never publishes a MAVROS setpoint.
"""

import sys
import time

import rospy
from geometry_msgs.msg import Pose, PoseArray
from tf.transformations import quaternion_from_euler


WAYPOINTS = (
    (0.50, 0.00, 1.00, 0.00),
    (0.50, -0.25, 1.00, 0.00),
    (0.00, 0.00, 1.00, 0.00),
    (-0.75, 0.50, 1.00, 0.00),
    (0.00, 0.00, 1.00, 0.00),
)


def main():
    rospy.init_node("robotac_dynamic_waypoint_publisher", anonymous=True)
    topic = str(rospy.get_param("~topic", "/robotac/flight/waypoints"))
    frame_id = str(rospy.get_param("~frame_id", "robotac_start_body"))
    wait_seconds = float(rospy.get_param("~wait_seconds", 5.0))
    publish_count = int(rospy.get_param("~publish_count", 5))
    publish_period = float(rospy.get_param("~publish_period", 0.10))
    if not topic or not frame_id or wait_seconds <= 0.0 or publish_count < 1 or publish_period <= 0.0:
        raise ValueError("invalid dynamic waypoint publisher parameters")

    publisher = rospy.Publisher(topic, PoseArray, queue_size=1, latch=True)
    deadline = time.monotonic() + wait_seconds
    while not rospy.is_shutdown() and publisher.get_num_connections() < 1:
        if time.monotonic() >= deadline:
            raise RuntimeError("no waypoint-controller subscriber on %s" % topic)
        rospy.sleep(0.05)

    message = PoseArray()
    message.header.frame_id = frame_id
    for x, y, z, yaw in WAYPOINTS:
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        quaternion = quaternion_from_euler(0.0, 0.0, yaw)
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]
        message.poses.append(pose)

    for _ in range(publish_count):
        message.header.stamp = rospy.Time.now()
        publisher.publish(message)
        rospy.sleep(publish_period)
    print("Published %d dynamic %s waypoints on %s" %
          (len(message.poses), frame_id, topic))


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        rospy.logerr("dynamic waypoint publisher failed: %s", exc)
        sys.exit(1)
