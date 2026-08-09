#!/usr/bin/env python3
"""Validate and adapt FAST-LIO pose output for MAVROS vision fusion.

The input is the upstream FAST-LIO Odometry message (local coordinates).  The
output is ROS ENU/FLU pose data; MAVROS performs the ENU/NED and FLU/FRD
conversion when it sends VISION_POSITION_ESTIMATE to PX4.
"""

import math
import time

import numpy as np
import rospy
import tf.transformations as tft
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String


def _finite(values):
    return all(math.isfinite(float(value)) for value in values)


def _as_bool(value):
    """Parse ROS XML/YAML booleans without accepting the string "false" as true."""
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in ("true", "1", "yes", "on"):
            return True
        if normalized in ("false", "0", "no", "off", ""):
            return False
        raise ValueError("invalid boolean value: %s" % value)
    return bool(value)


def _as_vector(value, length, name):
    value = list(value)
    if len(value) != length or not _finite(value):
        raise ValueError("%s must contain %d finite values" % (name, length))
    return np.asarray(value, dtype=float)


def _as_quaternion(value, name):
    quaternion = _as_vector(value, 4, name)
    norm = float(np.linalg.norm(quaternion))
    if norm < 1.0e-6:
        raise ValueError("%s must not be a zero quaternion" % name)
    return quaternion / norm


def _require_deployment_gates(names):
    """Do not emit external-vision data until aircraft checks are recorded."""
    deployment = rospy.get_param("/deployment", {})
    if not isinstance(deployment, dict):
        raise RuntimeError("/deployment must be a mapping loaded from deployment.yaml")
    missing = [name for name in names if deployment.get(name) is not True]
    if missing:
        raise RuntimeError("deployment gates not confirmed: %s" % ", ".join(missing))


VISION_DEPLOYMENT_GATES = (
    "lidar_network_configured",
    "lidar_imu_extrinsics_calibrated",
    "lidar_imu_time_checked",
    "stable_fcu_device_configured",
    "fastlio_airframe_extrinsics_validated",
    "fastlio_axes_validated",
    "px4_external_vision_configured",
)


class FastlioVisionBridge(object):
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/Odometry")
        self.output_topic = rospy.get_param("~output_topic", "/mavros/vision_pose/pose_cov")
        self.preview_topic = rospy.get_param("~preview_topic", "/robotac/fastlio_vision/pose_preview")
        requested_output = _as_bool(rospy.get_param("~enable_mavros_output", False))
        self.frame_alignment_approved = _as_bool(
            rospy.get_param("~frame_alignment_approved", False))
        if requested_output and not self.frame_alignment_approved:
            raise RuntimeError(
                "MAVROS vision output requires frame_alignment_approved=true")
        if requested_output:
            _require_deployment_gates(VISION_DEPLOYMENT_GATES)
        self.enable_mavros_output = requested_output and self.frame_alignment_approved
        self.health_topic = rospy.get_param("~health_topic", "/robotac/fastlio_vision/healthy")
        self.status_topic = rospy.get_param("~status_topic", "/robotac/fastlio_vision/status")
        self.expected_parent = rospy.get_param("~expected_input_parent", "camera_init")
        self.expected_child = rospy.get_param("~expected_input_child", "body")
        self.output_parent = rospy.get_param("~output_parent_frame", "odom")
        self.strict_frames = _as_bool(rospy.get_param("~strict_input_frames", True))
        self.max_age = float(rospy.get_param("~max_age", 0.30))
        self.max_future = float(rospy.get_param("~max_future", 0.10))
        self.max_speed = float(rospy.get_param("~max_position_speed", 8.0))
        self.max_yaw_rate = float(rospy.get_param("~max_yaw_rate", 6.0))
        self.zero_origin_on_start = _as_bool(rospy.get_param("~zero_origin_on_start", True))
        self.max_output_radius = float(rospy.get_param("~max_output_radius", 100.0))
        self.min_rate = float(rospy.get_param("~min_rate_hz", 5.0))
        self.min_samples = int(rospy.get_param("~min_samples", 5))
        self.health_timeout = float(rospy.get_param("~health_timeout", 0.50))
        self.min_covariance = float(rospy.get_param("~min_covariance", 1.0e-6))
        self.default_position_variance = float(rospy.get_param("~default_position_variance", 0.04))
        self.default_angle_variance = float(rospy.get_param("~default_angle_variance", 0.03))

        self.world_rotation = tft.quaternion_matrix(_as_quaternion(
            rospy.get_param("~input_world_to_output_quaternion", [0.0, 0.0, 0.0, 1.0]),
            "input_world_to_output_quaternion"))[:3, :3]
        self.world_translation = _as_vector(
            rospy.get_param("~input_world_to_output_translation", [0.0, 0.0, 0.0]),
            3, "input_world_to_output_translation")
        self.child_rotation = tft.quaternion_matrix(_as_quaternion(
            rospy.get_param("~input_child_to_output_child_quaternion", [0.0, 0.0, 0.0, 1.0]),
            "input_child_to_output_child_quaternion"))[:3, :3]
        self.child_translation = _as_vector(
            rospy.get_param("~input_child_to_output_child_translation", [0.0, 0.0, 0.0]),
            3, "input_child_to_output_child_translation")

        self.pose_pub = rospy.Publisher(self.output_topic, PoseWithCovarianceStamped, queue_size=10)
        self.preview_pub = rospy.Publisher(self.preview_topic, PoseWithCovarianceStamped, queue_size=10)
        self.output_enabled_pub = rospy.Publisher(
            "/robotac/fastlio_vision/output_enabled", Bool, queue_size=1, latch=True)
        self.health_pub = rospy.Publisher(self.health_topic, Bool, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=1, latch=True)
        self.sub = rospy.Subscriber(self.input_topic, Odometry, self._odom_cb, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.10), self._health_cb)

        self.last_stamp = None
        self.last_position = None
        self.last_quaternion = None
        self.origin_position = None
        self.last_receive = None
        self.last_rate_stamp = None
        self.rate_hz = 0.0
        self.valid_count = 0
        self.drop_count = 0
        self.healthy = False
        self.reason = "waiting_for_fastlio"
        self._set_status(self.reason)
        self.output_enabled_pub.publish(Bool(data=self.enable_mavros_output))
        if not self.enable_mavros_output:
            rospy.logwarn("MAVROS vision output disabled; publishing preview only")

    def _set_status(self, reason):
        self.reason = reason
        self.status_pub.publish(String(data=reason))

    def _reject(self, reason):
        self.drop_count += 1
        self._reset_health_window(reset_pose_baseline=True)
        self._set_status(reason)

    def _reset_health_window(self, reset_pose_baseline=True):
        """Require a new consecutive sample window after a fault or timeout."""
        self.healthy = False
        self.valid_count = 0
        self.rate_hz = 0.0
        self.last_rate_stamp = None
        if reset_pose_baseline:
            self.last_stamp = None
            self.last_position = None
            self.last_quaternion = None

    def _normal_covariance(self, values, quaternion):
        try:
            matrix = np.asarray(values, dtype=float).reshape((6, 6))
        except (ValueError, TypeError):
            matrix = np.zeros((6, 6), dtype=float)
        if not np.all(np.isfinite(matrix)):
            matrix = np.zeros((6, 6), dtype=float)
        matrix = 0.5 * (matrix + matrix.T)
        if float(np.max(np.abs(matrix))) < self.min_covariance:
            matrix = np.diag([
                self.default_position_variance,
                self.default_position_variance,
                self.default_position_variance,
                self.default_angle_variance,
                self.default_angle_variance,
                self.default_angle_variance,
            ])
        # Right-multiplying the pose by the fixed body->base transform adds a
        # lever-arm Jacobian: position errors contain orientation errors when
        # the IMU/body origin and airframe origin are not coincident.
        skew = np.array([[0.0, -self.child_translation[2], self.child_translation[1]],
                         [self.child_translation[2], 0.0, -self.child_translation[0]],
                         [-self.child_translation[1], self.child_translation[0], 0.0]])
        child_jacobian = np.eye(6, dtype=float)
        child_jacobian[:3, 3:] = -tft.quaternion_matrix(quaternion)[:3, :3].dot(skew)
        world_jacobian = np.zeros((6, 6), dtype=float)
        world_jacobian[:3, :3] = self.world_rotation
        world_jacobian[3:, 3:] = self.world_rotation
        matrix = world_jacobian.dot(child_jacobian).dot(matrix).dot(child_jacobian.T).dot(world_jacobian.T)
        for index in range(6):
            matrix[index, index] = max(float(matrix[index, index]), self.min_covariance)
        try:
            if float(np.min(np.linalg.eigvalsh(matrix))) < -1.0e-7:
                matrix = np.diag([
                    self.default_position_variance,
                    self.default_position_variance,
                    self.default_position_variance,
                    self.default_angle_variance,
                    self.default_angle_variance,
                    self.default_angle_variance,
                ])
        except np.linalg.LinAlgError:
            matrix = np.eye(6, dtype=float)
        return matrix

    def _odom_cb(self, msg):
        stamp = msg.header.stamp
        if stamp == rospy.Time(0):
            self._reject("zero_timestamp")
            return
        if self.strict_frames and (msg.header.frame_id != self.expected_parent or
                                   msg.child_frame_id != self.expected_child):
            self._reject("unexpected_frames:%s->%s" % (msg.header.frame_id, msg.child_frame_id))
            return
        pose = msg.pose.pose
        position = np.asarray([pose.position.x, pose.position.y, pose.position.z], dtype=float)
        quaternion = np.asarray([pose.orientation.x, pose.orientation.y,
                                 pose.orientation.z, pose.orientation.w], dtype=float)
        if not _finite(position) or not _finite(quaternion):
            self._reject("nonfinite_pose")
            return
        norm = float(np.linalg.norm(quaternion))
        if norm < 1.0e-6:
            self._reject("invalid_quaternion")
            return
        quaternion /= norm
        if self.last_stamp is not None:
            dt = (stamp - self.last_stamp).to_sec()
            if dt <= 0.0:
                self._reject("non_monotonic_timestamp")
                return
            if dt < 0.001:
                self._reject("timestamp_too_close")
                return
            if self.last_position is not None:
                speed = float(np.linalg.norm(position - self.last_position) / dt)
                if speed > self.max_speed:
                    self._reject("position_jump_speed:%.2f" % speed)
                    return
            if self.last_quaternion is not None:
                previous = self.last_quaternion
                dot = min(1.0, max(-1.0, abs(float(np.dot(previous, quaternion)))))
                angular_rate = (2.0 * math.acos(dot)) / dt
                if angular_rate > self.max_yaw_rate:
                    self._reject("orientation_jump_rate:%.2f" % angular_rate)
                    return

        now = rospy.Time.now()
        if now == rospy.Time(0):
            self._reject("ros_clock_unavailable")
            return
        age = (now - stamp).to_sec()
        if age > self.max_age or age < -self.max_future:
            self._reject("timestamp_age:%.3f" % age)
            return

        # T_output_parent_input_parent * T_input_parent_input_child *
        # T_input_child_base_link. PoseWithCovarianceStamped carries only the
        # parent header, so its pose always has implicit base_link/FLU meaning.
        parent_position = self.world_rotation.dot(position) + self.world_translation
        child_position = self.world_rotation.dot(
            tft.quaternion_matrix(quaternion)[:3, :3].dot(self.child_translation))
        output_position = parent_position + child_position
        if self.zero_origin_on_start:
            if self.origin_position is None:
                self.origin_position = output_position.copy()
                rospy.loginfo("FAST-LIO vision local origin captured at [%.3f, %.3f, %.3f]",
                              self.origin_position[0], self.origin_position[1], self.origin_position[2])
            output_position = output_position - self.origin_position
        if self.max_output_radius > 0.0 and float(np.linalg.norm(output_position)) > self.max_output_radius:
            self._reject("output_radius_exceeded:%.2f" % float(np.linalg.norm(output_position)))
            return
        output_quaternion = tft.quaternion_multiply(
            tft.quaternion_from_matrix(np.vstack((np.hstack((self.world_rotation, np.zeros((3, 1)))),
                                                   [0.0, 0.0, 0.0, 1.0]))),
            tft.quaternion_multiply(quaternion,
                                    tft.quaternion_from_matrix(np.vstack((np.hstack((self.child_rotation, np.zeros((3, 1)))),
                                                                           [0.0, 0.0, 0.0, 1.0])))))
        output_quaternion /= np.linalg.norm(output_quaternion)

        output = PoseWithCovarianceStamped()
        output.header = msg.header
        output.header.frame_id = self.output_parent
        output.pose.pose.position.x = float(output_position[0])
        output.pose.pose.position.y = float(output_position[1])
        output.pose.pose.position.z = float(output_position[2])
        output.pose.pose.orientation.x = float(output_quaternion[0])
        output.pose.pose.orientation.y = float(output_quaternion[1])
        output.pose.pose.orientation.z = float(output_quaternion[2])
        output.pose.pose.orientation.w = float(output_quaternion[3])
        output.pose.covariance = self._normal_covariance(
            msg.pose.covariance, quaternion).reshape(-1).tolist()
        next_rate = self.rate_hz
        if self.last_rate_stamp is not None:
            rate_dt = (stamp - self.last_rate_stamp).to_sec()
            if rate_dt > 0.0:
                instant = 1.0 / rate_dt
                next_rate = instant if self.rate_hz == 0.0 else 0.8 * self.rate_hz + 0.2 * instant
        self.last_stamp = stamp
        self.last_position = position
        self.last_quaternion = quaternion
        self.last_receive = time.monotonic()
        self.valid_count += 1
        self.rate_hz = next_rate
        self.last_rate_stamp = stamp
        self.healthy = self.valid_count >= self.min_samples and self.rate_hz >= self.min_rate
        self.preview_pub.publish(output)
        # Preview remains available during warm-up.  PX4 never receives a
        # sample until the complete consecutive health window has passed.
        if self.enable_mavros_output and self.healthy:
            self.pose_pub.publish(output)
        state = "ok" if self.healthy else "warming_up"
        self._set_status("%s rate_hz=%.2f valid=%d dropped=%d mavros_output=%s zero_origin=%s" %
                         (state, self.rate_hz, self.valid_count, self.drop_count,
                          self.enable_mavros_output, self.zero_origin_on_start))

    def _health_cb(self, _event):
        # This latched topic is also a live bridge heartbeat.  Re-publishing it
        # here lets the flight controller distinguish a running bridge from a
        # stale latched value after the process or ROS graph has failed.
        self.output_enabled_pub.publish(Bool(data=self.enable_mavros_output))
        if self.last_receive is None or time.monotonic() - self.last_receive > self.health_timeout:
            self._reset_health_window()
            if self.last_receive is None:
                self._set_status("waiting_for_fastlio")
            else:
                self._set_status("fastlio_timeout")
        elif self.valid_count < self.min_samples or self.rate_hz < self.min_rate:
            self.healthy = False
            self._set_status("fastlio_not_ready rate_hz=%.2f samples=%d" %
                             (self.rate_hz, self.valid_count))
        self.health_pub.publish(Bool(data=self.healthy))


if __name__ == "__main__":
    rospy.init_node("fastlio_vision_bridge")
    FastlioVisionBridge()
    rospy.spin()
