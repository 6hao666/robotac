#!/usr/bin/env python3
"""Hardware-isolated regression for the FAST-LIO MAVROS vision bridge.

The test publishes valid FAST-LIO-shaped odometry, verifies that the bridge
emits a local pose, injects an invalid frame, then proves output stays blocked
until a new complete health window has been collected.
"""

import math
import sys
import time

import numpy as np
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, quaternion_matrix, quaternion_multiply


class VisionBridgeIntegration(object):
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/robotac/test/fastlio_odom")
        self.output_topic = rospy.get_param("~output_topic", "/robotac/test/vision_pose")
        self.min_samples = int(rospy.get_param("~min_samples", 3))
        self.world_quaternion = self._normalize(np.asarray(rospy.get_param(
            "/fastlio_vision_bridge/input_world_to_output_quaternion"), dtype=float))
        self.world_translation = np.asarray(rospy.get_param(
            "/fastlio_vision_bridge/input_world_to_output_translation"), dtype=float)
        self.child_quaternion = self._normalize(np.asarray(rospy.get_param(
            "/fastlio_vision_bridge/input_child_to_output_child_quaternion"), dtype=float))
        self.child_translation = np.asarray(rospy.get_param(
            "/fastlio_vision_bridge/input_child_to_output_child_translation"), dtype=float)
        if np.allclose(quaternion_matrix(self.world_quaternion)[:3, :3], np.eye(3)):
            raise RuntimeError("simulation world transform must be non-identity")
        if np.allclose(quaternion_matrix(self.child_quaternion)[:3, :3], np.eye(3)):
            raise RuntimeError("simulation child transform must be non-identity")
        self.outputs = []
        self.publisher = rospy.Publisher(self.input_topic, Odometry, queue_size=10)
        self.subscription = rospy.Subscriber(
            self.output_topic, PoseWithCovarianceStamped, self._output_cb, queue_size=10)

    def _output_cb(self, msg):
        self.outputs.append(msg)

    @staticmethod
    def _normalize(quaternion):
        norm = float(np.linalg.norm(quaternion))
        if norm < 1.0e-6:
            raise RuntimeError("zero simulation quaternion")
        return quaternion / norm

    def _source_covariance(self):
        matrix = np.zeros((6, 6), dtype=float)
        for row in range(6):
            for column in range(6):
                if row == column:
                    matrix[row, column] = 0.01 * (row + 1)
                else:
                    matrix[row, column] = 0.0001 * (row + column + 2)
        return matrix

    def _expected_pose(self, source):
        world_rotation = quaternion_matrix(self.world_quaternion)[:3, :3]
        child_rotation = quaternion_matrix(self.child_quaternion)[:3, :3]
        source_quaternion = self._normalize(np.asarray([
            source.pose.pose.orientation.x,
            source.pose.pose.orientation.y,
            source.pose.pose.orientation.z,
            source.pose.pose.orientation.w,
        ], dtype=float))
        source_rotation = quaternion_matrix(source_quaternion)[:3, :3]
        source_position = np.asarray([
            source.pose.pose.position.x,
            source.pose.pose.position.y,
            source.pose.pose.position.z,
        ], dtype=float)
        expected_position = (world_rotation.dot(
            source_position + source_rotation.dot(self.child_translation)) +
                             self.world_translation)
        expected_quaternion = self._normalize(np.asarray(quaternion_multiply(
            self.world_quaternion,
            quaternion_multiply(source_quaternion, self.child_quaternion)), dtype=float))

        covariance = self._source_covariance()
        skew = np.array([
            [0.0, -self.child_translation[2], self.child_translation[1]],
            [self.child_translation[2], 0.0, -self.child_translation[0]],
            [-self.child_translation[1], self.child_translation[0], 0.0],
        ])
        child_jacobian = np.eye(6, dtype=float)
        child_jacobian[:3, 3:] = -source_rotation.dot(skew)
        world_jacobian = np.zeros((6, 6), dtype=float)
        world_jacobian[:3, :3] = world_rotation
        world_jacobian[3:, 3:] = world_rotation
        expected_covariance = (world_jacobian.dot(child_jacobian).dot(covariance)
                               .dot(child_jacobian.T).dot(world_jacobian.T))
        return expected_position, expected_quaternion, expected_covariance

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
        msg.pose.covariance = self._source_covariance().reshape(-1).tolist()
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
        expected_position, expected_quaternion, expected_covariance = self._expected_pose(last_source)
        actual_position = np.asarray([
            initial.pose.pose.position.x,
            initial.pose.pose.position.y,
            initial.pose.pose.position.z,
        ], dtype=float)
        actual_quaternion = np.asarray([
            initial.pose.pose.orientation.x,
            initial.pose.pose.orientation.y,
            initial.pose.pose.orientation.z,
            initial.pose.pose.orientation.w,
        ], dtype=float)
        if not np.allclose(actual_position, expected_position, atol=1.0e-6):
            raise RuntimeError("non-identity bridge position transform is incorrect")
        if abs(float(np.dot(actual_quaternion, expected_quaternion))) < 1.0 - 1.0e-6:
            raise RuntimeError("non-identity bridge orientation transform is incorrect")
        if not np.allclose(np.asarray(initial.pose.covariance).reshape((6, 6)),
                           expected_covariance, atol=1.0e-8):
            raise RuntimeError("bridge covariance transform is incorrect")

        # The bad frame clears the previous health window. A single later valid
        # sample must not reach the MAVROS output topic yet.
        self._publish_and_wait(0.10, parent="unexpected_frame")
        rospy.sleep(0.10)
        output_count = len(self.outputs)
        self._publish_and_wait(0.11)
        rospy.sleep(0.15)
        if len(self.outputs) != output_count:
            raise RuntimeError("vision output resumed before the new health window")

        recovered_source = None
        for index in range(1, self.min_samples):
            recovered_source = self._publish_and_wait(0.11 + 0.01 * index)
        self._wait_for(lambda: len(self.outputs) > output_count, 2.0,
                       "vision output after recovered health window")
        recovered = self.outputs[-1]
        expected_position, expected_quaternion, expected_covariance = self._expected_pose(recovered_source)
        recovered_position = np.asarray([
            recovered.pose.pose.position.x,
            recovered.pose.pose.position.y,
            recovered.pose.pose.position.z,
        ], dtype=float)
        recovered_quaternion = np.asarray([
            recovered.pose.pose.orientation.x,
            recovered.pose.pose.orientation.y,
            recovered.pose.pose.orientation.z,
            recovered.pose.pose.orientation.w,
        ], dtype=float)
        if recovered.header.stamp != recovered_source.header.stamp:
            raise RuntimeError("recovered output did not use the latest source timestamp")
        if not np.allclose(recovered_position, expected_position, atol=1.0e-6):
            raise RuntimeError("recovered output does not contain the latest transformed pose")
        if abs(float(np.dot(recovered_quaternion, expected_quaternion))) < 1.0 - 1.0e-6:
            raise RuntimeError("recovered output does not contain the latest transformed orientation")
        if not np.allclose(np.asarray(recovered.pose.covariance).reshape((6, 6)),
                           expected_covariance, atol=1.0e-8):
            raise RuntimeError("recovered output does not contain the latest transformed covariance")

        print("Vision bridge integration passed: %d output samples" % len(self.outputs))


if __name__ == "__main__":
    rospy.init_node("robotac_vision_bridge_integration")
    try:
        VisionBridgeIntegration().run()
    except Exception as exc:
        rospy.logerr("vision bridge integration failed: %s", exc)
        sys.exit(1)
