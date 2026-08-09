#!/usr/bin/env python3
"""Sunray Path A style local odometry to MAVROS vision-pose bridge.

Sunray's Path A is intentionally simple: take a local ``nav_msgs/Odometry``
estimate, copy its pose into ``geometry_msgs/PoseStamped``, and feed MAVROS's
``vision_pose_estimate`` plugin on ``/mavros/vision_pose/pose``.

This implementation keeps that data path, but wraps it with Robotac's safety
contract: MAVROS output is disabled by default, frame-alignment approval is
required before output can be enabled, and the node reports live health/status
for the flight controller.  Preview output is always local ROS-only.
"""

import math
import time
import traceback

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String


def _as_bool(value):
    """Parse ROS XML/YAML booleans without accepting the string false as true."""
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in ("true", "1", "yes", "on"):
            return True
        if normalized in ("false", "0", "no", "off", ""):
            return False
        raise ValueError("invalid boolean value: %s" % value)
    return bool(value)


def _finite(values):
    return all(math.isfinite(float(value)) for value in values)


def _require_deployment_gates(names):
    """Do not emit FCU-bound vision data until aircraft checks are recorded."""
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


class LocalOdomToVisionPose(object):
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/sunray/odometry")
        self.output_topic = rospy.get_param("~output_topic", "/mavros/vision_pose/pose")
        self.preview_topic = rospy.get_param(
            "~preview_topic", "/robotac/fastlio_vision/path_a_pose_preview")
        self.health_topic = rospy.get_param("~health_topic", "/robotac/fastlio_vision/healthy")
        self.status_topic = rospy.get_param("~status_topic", "/robotac/fastlio_vision/status")
        self.output_enabled_topic = rospy.get_param(
            "~output_enabled_topic", "/robotac/fastlio_vision/output_enabled")

        requested_output = _as_bool(rospy.get_param("~enable_mavros_output", False))
        self.frame_alignment_approved = _as_bool(
            rospy.get_param("~frame_alignment_approved", False))
        if requested_output and not self.frame_alignment_approved:
            raise RuntimeError(
                "MAVROS vision output requires frame_alignment_approved=true")
        if requested_output:
            _require_deployment_gates(VISION_DEPLOYMENT_GATES)
        self.enable_mavros_output = requested_output and self.frame_alignment_approved

        self.output_frame_id = str(rospy.get_param("~output_frame_id", "odom")).strip()
        self.preserve_input_frame = _as_bool(rospy.get_param("~preserve_input_frame", False))
        self.strict_frames = _as_bool(rospy.get_param("~strict_input_frames", False))
        self.expected_parent = str(rospy.get_param("~expected_input_parent", "world")).strip()
        self.expected_child = str(rospy.get_param("~expected_input_child", "")).strip()
        self.zero_origin_on_start = _as_bool(rospy.get_param("~zero_origin_on_start", False))
        self.max_age = float(rospy.get_param("~max_age", 0.30))
        self.max_future = float(rospy.get_param("~max_future", 0.10))
        self.max_speed = float(rospy.get_param("~max_position_speed", 8.0))
        self.max_yaw_rate = float(rospy.get_param("~max_yaw_rate", 6.0))
        self.max_output_radius = float(rospy.get_param("~max_output_radius", 50.0))
        self.min_rate = float(rospy.get_param("~min_rate_hz", 5.0))
        self.min_samples = int(rospy.get_param("~min_samples", 5))
        self.health_timeout = float(rospy.get_param("~health_timeout", 0.50))

        if not self.preserve_input_frame and not self.output_frame_id:
            raise ValueError("output_frame_id is required when preserve_input_frame=false")
        if self.strict_frames and not self.expected_parent:
            raise ValueError("expected_input_parent is required when strict_input_frames=true")
        positive_values = (
            self.max_age, self.max_future, self.max_speed, self.max_yaw_rate,
            self.min_rate, self.health_timeout,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in positive_values):
            raise ValueError("Path A bridge timing/rate limits must be positive finite values")
        if self.min_samples < 1:
            raise ValueError("min_samples must be >= 1")

        self.pose_pub = rospy.Publisher(self.output_topic, PoseStamped, queue_size=10)
        self.preview_pub = rospy.Publisher(self.preview_topic, PoseStamped, queue_size=10)
        self.output_enabled_pub = rospy.Publisher(
            self.output_enabled_topic, Bool, queue_size=1, latch=True)
        self.health_pub = rospy.Publisher(self.health_topic, Bool, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=1, latch=True)
        self.sub = rospy.Subscriber(self.input_topic, Odometry, self._odom_cb, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.10), self._health_cb)

        self.last_stamp = None
        self.last_position = None
        self.last_quaternion = None
        self.last_receive = None
        self.last_rate_stamp = None
        self.origin_position = None
        self.rate_hz = 0.0
        self.valid_count = 0
        self.drop_count = 0
        self.healthy = False
        self.reason = "waiting_for_local_odom"
        self._set_status(self.reason)
        self.output_enabled_pub.publish(Bool(data=self.enable_mavros_output))
        if not self.enable_mavros_output:
            rospy.logwarn("Path A MAVROS vision output disabled; publishing preview only")
        rospy.loginfo("Path A vision bridge: %s -> %s", self.input_topic, self.output_topic)

    def _set_status(self, reason):
        self.reason = reason
        self.status_pub.publish(String(data=reason))

    def _reject(self, reason):
        self.drop_count += 1
        self.healthy = False
        self.valid_count = 0
        self.rate_hz = 0.0
        self.last_rate_stamp = None
        self.last_stamp = None
        self.last_position = None
        self.last_quaternion = None
        self._set_status(reason)

    @staticmethod
    def _normalize_quaternion(quaternion):
        norm = math.sqrt(sum(value * value for value in quaternion))
        if norm < 1.0e-6:
            return None
        return [value / norm for value in quaternion]

    @staticmethod
    def _angular_rate(previous, current, dt):
        dot = abs(sum(previous[index] * current[index] for index in range(4)))
        dot = min(1.0, max(-1.0, dot))
        return (2.0 * math.acos(dot)) / dt

    def _odom_cb(self, msg):
        if self.strict_frames:
            if msg.header.frame_id != self.expected_parent:
                self._reject("unexpected_parent:%s" % msg.header.frame_id)
                return
            if self.expected_child and msg.child_frame_id != self.expected_child:
                self._reject("unexpected_child:%s" % msg.child_frame_id)
                return

        stamp = msg.header.stamp
        if stamp == rospy.Time(0):
            self._reject("zero_timestamp")
            return
        now = rospy.Time.now()
        if now == rospy.Time(0):
            self._reject("ros_clock_unavailable")
            return
        age = (now - stamp).to_sec()
        if age > self.max_age or age < -self.max_future:
            self._reject("timestamp_age:%.3f" % age)
            return

        pose = msg.pose.pose
        position = [pose.position.x, pose.position.y, pose.position.z]
        quaternion = [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
        if not _finite(position) or not _finite(quaternion):
            self._reject("nonfinite_pose")
            return
        quaternion = self._normalize_quaternion([float(value) for value in quaternion])
        if quaternion is None:
            self._reject("invalid_quaternion")
            return
        position = [float(value) for value in position]

        if self.last_stamp is not None:
            dt = (stamp - self.last_stamp).to_sec()
            if dt <= 0.0:
                self._reject("non_monotonic_timestamp")
                return
            if dt < 0.001:
                self._reject("timestamp_too_close")
                return
            if self.last_position is not None:
                speed = math.sqrt(sum(
                    (position[index] - self.last_position[index]) ** 2
                    for index in range(3))) / dt
                if speed > self.max_speed:
                    self._reject("position_jump_speed:%.2f" % speed)
                    return
            if self.last_quaternion is not None:
                rate = self._angular_rate(self.last_quaternion, quaternion, dt)
                if rate > self.max_yaw_rate:
                    self._reject("orientation_jump_rate:%.2f" % rate)
                    return

        output_position = list(position)
        if self.zero_origin_on_start:
            if self.origin_position is None:
                self.origin_position = list(output_position)
                rospy.loginfo("Path A local origin captured at [%.3f, %.3f, %.3f]",
                              self.origin_position[0], self.origin_position[1], self.origin_position[2])
            output_position = [output_position[index] - self.origin_position[index]
                               for index in range(3)]
        radius = math.sqrt(sum(value * value for value in output_position))
        if self.max_output_radius > 0.0 and radius > self.max_output_radius:
            self._reject("output_radius_exceeded:%.2f" % radius)
            return

        output = PoseStamped()
        output.header.stamp = stamp
        output.header.frame_id = msg.header.frame_id if self.preserve_input_frame else self.output_frame_id
        output.pose.position.x = output_position[0]
        output.pose.position.y = output_position[1]
        output.pose.position.z = output_position[2]
        output.pose.orientation.x = quaternion[0]
        output.pose.orientation.y = quaternion[1]
        output.pose.orientation.z = quaternion[2]
        output.pose.orientation.w = quaternion[3]

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
        self.last_rate_stamp = stamp
        self.valid_count += 1
        self.rate_hz = next_rate
        self.healthy = self.valid_count >= self.min_samples and self.rate_hz >= self.min_rate

        self.preview_pub.publish(output)
        if self.enable_mavros_output and self.healthy:
            self.pose_pub.publish(output)
        state = "ok" if self.healthy else "warming_up"
        self._set_status(
            "%s path_a rate_hz=%.2f valid=%d dropped=%d mavros_output=%s input=%s output=%s frame=%s" %
            (state, self.rate_hz, self.valid_count, self.drop_count,
             self.enable_mavros_output, self.input_topic, self.output_topic,
             output.header.frame_id))

    def _health_cb(self, _event):
        self.output_enabled_pub.publish(Bool(data=self.enable_mavros_output))
        if self.last_receive is None or time.monotonic() - self.last_receive > self.health_timeout:
            self.healthy = False
            self.valid_count = 0
            self.rate_hz = 0.0
            self.last_rate_stamp = None
            if self.last_receive is None:
                self._set_status("waiting_for_local_odom")
            else:
                self._set_status("local_odom_timeout")
        elif self.valid_count < self.min_samples or self.rate_hz < self.min_rate:
            self.healthy = False
            self._set_status("local_odom_not_ready rate_hz=%.2f samples=%d" %
                             (self.rate_hz, self.valid_count))
        self.health_pub.publish(Bool(data=self.healthy))


if __name__ == "__main__":
    rospy.init_node("path_a_vision_pose")
    try:
        LocalOdomToVisionPose()
    except Exception as exc:  # pylint: disable=broad-except
        rospy.logfatal("Path A vision bridge startup failed: %s", exc)
        traceback.print_exc()
        raise
    rospy.spin()
