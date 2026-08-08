#!/usr/bin/env python3
"""Hardware-isolated regression for the FAST-LIO MAVROS vision bridge.

The test publishes valid FAST-LIO-shaped odometry, verifies that the bridge
emits a local pose, injects an invalid frame, then proves output stays blocked
until a new complete health window has been collected.
"""

import math
import sys
import time

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler


class VisionBridgeIntegration(object):
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/robotac/test/fastlio_odom")
        self.output_topic = rospy.get_param("~output_topic", "/robotac/test/vision_pose")
        self.min_samples = int(rospy.get_param("~min_samples", 3))
        self.outputs = []
        self.publisher = rospy.Publisher(self.input_topic, Odometry, queue_size=10)
        self.subscription = rospy.Subscriber(
            self.output_topic, PoseWithCovarianceStamped, self._output_cb, queue_size=10)

    def _output_cb(self, msg):
        self.outputs.append(msg)

    def _wait_for(self, predicate, timeout, description):
        deadline = time.monotonic() + timeout
        rate = rospy.Rate(100)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if predicate():
                return
            rate.sleep()
        raise RuntimeError("timed out waiting for %s" % description)

    def _publish(self, x, parent="camera_init", child="body"):
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = parent
        msg.child_frame_id = child
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = -0.2
        msg.pose.pose.position.z = 0.4
        quaternion = quaternion_from_euler(0.0, 0.0, 0.1)
        msg.pose.pose.orientation.x = quaternion[0]
        msg.pose.pose.orientation.y = quaternion[1]
        msg.pose.pose.orientation.z = quaternion[2]
        msg.pose.pose.orientation.w = quaternion[3]
        for index in (0, 7, 14, 21, 28, 35):
            msg.pose.covariance[index] = 0.01
        self.publisher.publish(msg)
        return msg

    def _publish_and_wait(self, x, parent="camera_init", child="body"):
        message = self._publish(x, parent=parent, child=child)
        rospy.sleep(0.06)
        return message

    def run(self):
        self._wait_for(lambda: self.publisher.get_num_connections() > 0, 5.0,
                       "vision bridge subscriber")

        last_source = None
        for index in range(self.min_samples + 2):
            last_source = self._publish_and_wait(index * 0.01)
        self._wait_for(
            lambda: self.outputs and self.outputs[-1].header.stamp == last_source.header.stamp,
            2.0, "initial vision output")
        initial = self.outputs[-1]
        if initial.header.frame_id != "odom":
            raise RuntimeError("unexpected output parent frame: %s" % initial.header.frame_id)
        if initial.header.stamp != last_source.header.stamp:
            raise RuntimeError("bridge did not preserve the FAST-LIO timestamp")
        if abs(initial.pose.pose.position.x - last_source.pose.pose.position.x) > 1.0e-6:
            raise RuntimeError("identity bridge changed the local x position")
        if abs(initial.pose.pose.position.y - last_source.pose.pose.position.y) > 1.0e-6:
            raise RuntimeError("identity bridge changed the local y position")

        # The bad frame clears the previous health window. A single later valid
        # sample must not reach the MAVROS output topic yet.
        self._publish_and_wait(0.10, parent="unexpected_frame")
        rospy.sleep(0.10)
        output_count = len(self.outputs)
        self._publish_and_wait(0.11)
        rospy.sleep(0.15)
        if len(self.outputs) != output_count:
            raise RuntimeError("vision output resumed before the new health window")

        for index in range(1, self.min_samples):
            self._publish_and_wait(0.11 + 0.01 * index)
        self._wait_for(lambda: len(self.outputs) > output_count, 2.0,
                       "vision output after recovered health window")
        recovered = self.outputs[-1]
        expected_x = 0.11 + 0.01 * (self.min_samples - 1)
        if not math.isclose(recovered.pose.pose.position.x, expected_x, abs_tol=1.0e-6):
            raise RuntimeError("recovered output does not contain the latest local pose")

        print("Vision bridge integration passed: %d output samples" % len(self.outputs))


if __name__ == "__main__":
    rospy.init_node("robotac_vision_bridge_integration")
    try:
        VisionBridgeIntegration().run()
    except Exception as exc:
        rospy.logerr("vision bridge integration failed: %s", exc)
        sys.exit(1)
