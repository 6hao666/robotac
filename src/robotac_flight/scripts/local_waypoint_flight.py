#!/usr/bin/env python3
"""Safety-gated local ENU waypoint controller for PX4 through MAVROS.

No action is taken at startup.  Setting ``enable_control`` true only permits
MAVROS setpoint/service calls; the mission still requires an explicit
``/robotac/flight/start`` request.  ``auto_arm`` and ``auto_mode`` are false by
default and should remain false during bench validation.
"""

import math
import time

import rospy
from geometry_msgs.msg import PoseArray, PoseWithCovarianceStamped
from mavros_msgs.msg import EstimatorStatus, ExtendedState, PositionTarget, State, TimesyncStatus
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, TriggerResponse
from tf.transformations import euler_from_quaternion


def _as_bool(value):
    """Accept ROS XML/YAML booleans without treating the string "false" as true."""
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in ("true", "1", "yes", "on"):
            return True
        if normalized in ("false", "0", "no", "off", ""):
            return False
        raise ValueError("invalid boolean value: %s" % value)
    return bool(value)


def _finite_float(value, name):
    try:
        number = float(value)
    except (TypeError, ValueError):
        raise ValueError("%s must be numeric" % name)
    if not math.isfinite(number):
        raise ValueError("%s must be finite" % name)
    return number


def _require_deployment_gates(names):
    """Refuse active control unless the aircraft-level checks are explicit."""
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


CONTROL_DEPLOYMENT_GATES = VISION_DEPLOYMENT_GATES + (
    "px4_offboard_failsafe_configured",
    "local_flight_ground_tested",
)


class LocalWaypointFlight(object):
    IDLE = "IDLE"
    PAYLOAD_PREPARE = "PAYLOAD_PREPARE"
    PRESTREAM = "PRESTREAM"
    WAIT_OFFBOARD = "WAIT_OFFBOARD"
    WAIT_ARMED = "WAIT_ARMED"
    TAKEOFF = "TAKEOFF"
    WAYPOINTS = "WAYPOINTS"
    WAIT_LAND = "WAIT_LAND"
    LANDING = "LANDING"
    COMPLETE = "COMPLETE"
    ABORT = "ABORT"

    def __init__(self):
        self.enable_control = _as_bool(rospy.get_param("~enable_control", False))
        self.auto_mode = _as_bool(rospy.get_param("~auto_mode", False))
        self.auto_arm = _as_bool(rospy.get_param("~auto_arm", False))
        self.auto_land = _as_bool(rospy.get_param("~auto_land", False))
        self.enable_payload = _as_bool(rospy.get_param("~enable_payload", False))
        self.require_vision = _as_bool(rospy.get_param("~require_vision", True))
        self.require_vision_output = _as_bool(
            rospy.get_param("~require_vision_output", True))
        self.require_estimator = _as_bool(
            rospy.get_param("~require_estimator_status", True))
        self.require_horizontal_relative = _as_bool(
            rospy.get_param("~require_horizontal_relative", True))
        self.require_vertical_estimate = _as_bool(
            rospy.get_param("~require_vertical_estimate", True))
        self.prestream_seconds = float(rospy.get_param("~prestream_seconds", 5.0))
        self.control_rate = float(rospy.get_param("~control_rate_hz", 20.0))
        self.takeoff_height = float(rospy.get_param("~takeoff_height", 1.0))
        self.position_tolerance = float(rospy.get_param("~position_tolerance", 0.25))
        self.yaw_tolerance = math.radians(float(rospy.get_param("~yaw_tolerance_deg", 12.0)))
        self.hold_seconds = float(rospy.get_param("~waypoint_hold_seconds", 2.0))
        self.waypoint_timeout = float(rospy.get_param("~waypoint_timeout", 45.0))
        self.mission_timeout = float(rospy.get_param("~mission_timeout", 600.0))
        self.local_pose_timeout = float(rospy.get_param("~local_pose_timeout", 0.30))
        self.local_stamp_timeout = float(rospy.get_param("~local_stamp_timeout", 0.50))
        self.local_stamp_future_tolerance = float(
            rospy.get_param("~local_stamp_future_tolerance", 0.10))
        # MAVROS local_position publishes ROS ENU odometry in these configured
        # frames. Rejecting a mismatched producer prevents an unrelated odom
        # source from silently becoming the control reference.
        self.strict_local_frames = _as_bool(rospy.get_param("~strict_local_frames", True))
        self.expected_local_parent = str(rospy.get_param("~expected_local_parent", "map")).strip()
        self.expected_local_child = str(rospy.get_param("~expected_local_child", "base_link")).strip()
        self.max_local_position_speed = float(
            rospy.get_param("~max_local_position_speed", 6.0))
        self.max_local_yaw_rate = float(rospy.get_param("~max_local_yaw_rate", 6.0))
        self.state_timeout = float(rospy.get_param("~state_timeout", 1.0))
        self.extended_state_timeout = float(rospy.get_param("~extended_state_timeout", 1.0))
        self.vision_timeout = float(rospy.get_param("~vision_timeout", 0.50))
        self.vision_status_timeout = float(
            rospy.get_param("~vision_status_timeout", self.vision_timeout))
        self.vision_output_timeout = float(
            rospy.get_param("~vision_output_timeout", self.vision_timeout))
        self.vision_output_stamp_timeout = float(
            rospy.get_param("~vision_output_stamp_timeout", 0.30))
        self.vision_output_stamp_future_tolerance = float(
            rospy.get_param("~vision_output_stamp_future_tolerance", 0.10))
        self.vision_output_parent = str(
            rospy.get_param("~vision_output_parent", "odom")).strip()
        self.require_timesync = _as_bool(rospy.get_param("~require_timesync", True))
        self.timesync_timeout = float(rospy.get_param("~timesync_timeout", 1.0))
        self.max_timesync_rtt_ms = float(
            rospy.get_param("~max_timesync_rtt_ms", 20.0))
        self.estimator_timeout = float(rospy.get_param("~estimator_timeout", 1.0))
        self.offboard_timeout = float(rospy.get_param("~offboard_timeout", 15.0))
        self.arming_timeout = float(rospy.get_param("~arming_timeout", 15.0))
        self.takeoff_timeout = float(rospy.get_param("~takeoff_timeout", self.waypoint_timeout))
        self.max_xy = float(rospy.get_param("~max_waypoint_xy", 20.0))
        self.max_z = float(rospy.get_param("~max_waypoint_z", 5.0))
        # Health/communications failures must never keep an old OFFBOARD target
        # alive.  An operator-triggered abort may deliberately use a different
        # policy while the operator is observing the aircraft.
        self.critical_fault_action = rospy.get_param("~critical_fault_action", "release")
        self.operator_abort_action = rospy.get_param("~operator_abort_action", "release")
        self.abort_land_mode_timeout = float(
            rospy.get_param("~abort_land_mode_timeout", 8.0))
        self.auto_land_mode_timeout = float(
            rospy.get_param("~auto_land_mode_timeout", 8.0))
        self.landing_timeout = float(rospy.get_param("~landing_timeout", 60.0))
        self.require_auto_land = _as_bool(rospy.get_param("~require_auto_land", True))
        self.land_mode = rospy.get_param("~land_mode", "AUTO.LAND")
        self.land_descent_rate = float(rospy.get_param("~land_descent_rate", 0.30))
        self.land_switch_height = float(rospy.get_param("~land_switch_height", 0.50))
        self.input_frame = rospy.get_param("~waypoint_frame", "robotac_local_enu")
        self.waypoint_topic = rospy.get_param("~waypoint_topic", "/robotac/flight/waypoints")
        self.setpoint_topic = rospy.get_param("~setpoint_topic", "/mavros/setpoint_raw/local")
        self.require_setpoint_consumer = _as_bool(
            rospy.get_param("~require_setpoint_consumer", True))
        self.setpoint_consumer_node = str(rospy.get_param(
            "~setpoint_consumer_node", "/mavros")).strip()
        self.consumer_check_interval = float(
            rospy.get_param("~consumer_check_interval", 0.50))
        self.vision_status_topic = rospy.get_param(
            "~vision_status_topic", "/robotac/fastlio_vision/status")
        self.vision_output_topic = rospy.get_param(
            "~vision_output_topic", "/mavros/vision_pose/pose_cov")
        self.require_vision_output_consumer = _as_bool(
            rospy.get_param("~require_vision_output_consumer", True))
        self.vision_output_consumer_node = str(rospy.get_param(
            "~vision_output_consumer_node", "/mavros")).strip()
        self.payload_topic = rospy.get_param("~payload_topic", "/robotac/servo/open")
        self.payload_status_topic = rospy.get_param(
            "~payload_status_topic", "/robotac/servo/status")
        self.payload_required_connection = _as_bool(
            rospy.get_param("~payload_required_connection", True))
        self.payload_require_ack = _as_bool(
            rospy.get_param("~payload_require_ack", True))
        self.payload_ack_timeout = float(
            rospy.get_param("~payload_ack_timeout", 1.0))
        self.payload_preflight_close = _as_bool(
            rospy.get_param("~payload_preflight_close", True))
        self.payload_default_settle = float(
            rospy.get_param("~payload_default_settle_seconds", 2.0))
        if self.enable_payload and not self.enable_control:
            raise ValueError("enable_payload requires enable_control=true")
        if (self.auto_mode or self.auto_arm or self.auto_land) and not self.enable_control:
            raise ValueError("automatic flight actions require enable_control=true")
        if self.enable_control:
            required_gates = CONTROL_DEPLOYMENT_GATES
            if self.enable_payload:
                required_gates += ("stable_servo_device_configured",)
            _require_deployment_gates(required_gates)

        self.state = self.IDLE
        self.fcu = State()
        self.state_receive_time = None
        self.extended = ExtendedState()
        self.extended_receive_time = None
        self.local_odom = None
        self.local_receive_time = None
        self.last_local_stamp = None
        self.last_local_position = None
        self.last_local_yaw = None
        self.local_rejection_reason = None
        self.vision_healthy = False
        self.vision_receive_time = None
        self.vision_output_enabled = False
        self.vision_output_enabled_receive_time = None
        self.vision_status = "waiting_for_fastlio"
        self.vision_status_receive_time = None
        self.vision_output_receive_time = None
        self.vision_output_stamp = None
        self.vision_output_rejection_reason = None
        self.timesync = None
        self.timesync_receive_time = None
        self.timesync_issue = "waiting"
        self.estimator = None
        self.estimator_receive_time = None
        self.origin = None
        self.origin_yaw = 0.0
        self.waypoints = self._load_waypoints()
        self.index = 0
        self.target = None
        self.state_started = time.monotonic()
        self.mission_started = None
        self.reached_since = None
        self.last_setpoint = None
        self.last_setpoint_wall = None
        self.last_service_wall = {}
        self.landing_target = None
        self.landing_started = None
        self.auto_land_request_started = None
        self.abort_land_requested = False
        self.abort_land_deadline = None
        self.abort_action = "release"
        self.abort_setpoint_policy = "release"
        self.abort_reason = None
        # This second transmission gate is cleared by any critical abort.  It
        # prevents later code paths from accidentally resuming raw setpoints.
        self.control_tx_enabled = False
        self.payload_action_index = None
        self.payload_action_started = None
        self.payload_expected_state = None
        self.payload_command_time = None
        self.payload_command_sequence = None
        self.payload_command_boot_id = None
        self.payload_ack_state = None
        self.payload_ack_success = None
        self.payload_ack_sequence = 0
        self.payload_ack_boot_id = None
        self.payload_ack_time = None
        self.payload_state = "disabled" if not self.enable_payload else "uncommanded"
        self.last_error = "idle"
        self.last_consumer_check_wall = 0.0
        self.last_consumer_issue = None

        self.preview_pub = rospy.Publisher("/robotac/flight/setpoint_preview", PositionTarget, queue_size=10)
        # Do not even register a MAVROS setpoint publisher unless active
        # control is explicitly enabled. This keeps dry-run/preview launches
        # out of the FCU control topic and makes read-only evidence unambiguous.
        self.setpoint_pub = (rospy.Publisher(self.setpoint_topic, PositionTarget, queue_size=10)
                             if self.enable_control else None)
        self.payload_pub = (rospy.Publisher(self.payload_topic, Bool, queue_size=1)
                            if self.enable_payload else None)
        self.status_pub = rospy.Publisher("/robotac/flight/status", String, queue_size=1, latch=True)
        self.active_pub = rospy.Publisher("/robotac/flight/active", Bool, queue_size=1, latch=True)
        rospy.Subscriber("/mavros/state", State, self._state_cb, queue_size=10)
        rospy.Subscriber("/mavros/extended_state", ExtendedState, self._extended_cb, queue_size=10)
        rospy.Subscriber("/mavros/estimator_status", EstimatorStatus, self._estimator_cb, queue_size=10)
        rospy.Subscriber("/mavros/local_position/odom", Odometry, self._local_cb, queue_size=10)
        rospy.Subscriber("/robotac/fastlio_vision/healthy", Bool, self._vision_cb, queue_size=1)
        rospy.Subscriber("/robotac/fastlio_vision/output_enabled", Bool,
                         self._vision_output_cb, queue_size=1)
        rospy.Subscriber(self.vision_status_topic, String, self._vision_status_cb, queue_size=5)
        rospy.Subscriber("/mavros/timesync_status", TimesyncStatus,
                         self._timesync_cb, queue_size=10)
        if self.require_vision_output:
            rospy.Subscriber(self.vision_output_topic, PoseWithCovarianceStamped,
                             self._vision_pose_cb, queue_size=10)
        rospy.Subscriber(self.payload_status_topic, String, self._payload_status_cb, queue_size=5)
        rospy.Subscriber(self.waypoint_topic, PoseArray, self._waypoints_cb, queue_size=1)

        rospy.Service("/robotac/flight/start", Trigger, self._start_cb)
        rospy.Service("/robotac/flight/abort", Trigger, self._abort_cb)
        rospy.Service("/robotac/flight/land", Trigger, self._land_cb)
        rospy.Service("/robotac/flight/reset", Trigger, self._reset_cb)

        self.arm_srv = None
        self.mode_srv = None
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(1.0, self.control_rate)), self._tick)
        self._publish_status()
        if not self.enable_control:
            rospy.logwarn("robotac flight control disabled; only setpoint_preview is published")
        if self.auto_arm or self.auto_mode or self.auto_land:
            rospy.logwarn("automatic flight actions are configured; keep them false for bench testing")
        if self.enable_payload:
            rospy.logwarn("payload output is enabled; it is sent only after explicit mission start")

    def _load_waypoints(self):
        raw = rospy.get_param("~waypoints", [])
        if not isinstance(raw, list):
            raise ValueError("waypoints must be a list")
        result = []
        for index, item in enumerate(raw):
            if not isinstance(item, dict):
                raise ValueError("waypoints must be dictionaries")
            yaw_keys = [key for key in ("yaw", "yaw_deg") if key in item]
            if len(yaw_keys) > 1:
                raise ValueError("waypoint %d must use yaw or yaw_deg, not both" % index)
            if "yaw_deg" in item:
                yaw = math.radians(_finite_float(item["yaw_deg"], "waypoint %d yaw_deg" % index))
            else:
                yaw = _finite_float(item.get("yaw", 0.0), "waypoint %d yaw" % index)
            action = str(item.get("payload_action", "none")).strip().lower()
            result.append({
                "x": _finite_float(item.get("x", 0.0), "waypoint %d x" % index),
                "y": _finite_float(item.get("y", 0.0), "waypoint %d y" % index),
                "z": _finite_float(item.get("z", 0.0), "waypoint %d z" % index),
                "yaw": yaw,
                "hold": _finite_float(item.get("hold", self.hold_seconds),
                                      "waypoint %d hold" % index),
                "payload_action": action,
                "payload_settle": _finite_float(
                    item.get("payload_settle", self.payload_default_settle),
                    "waypoint %d payload_settle" % index),
            })
        return result

    @staticmethod
    def _angle_error(a, b):
        return math.atan2(math.sin(a - b), math.cos(a - b))

    @staticmethod
    def _finite_pose(pose):
        return LocalWaypointFlight._finite_geometry_pose(pose.pose.pose)

    @staticmethod
    def _finite_geometry_pose(pose):
        values = [pose.position.x, pose.position.y, pose.position.z,
                  pose.orientation.x, pose.orientation.y,
                  pose.orientation.z, pose.orientation.w]
        return all(math.isfinite(float(value)) for value in values)

    @staticmethod
    def _normalized_quaternion(quaternion):
        values = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        if not all(math.isfinite(float(value)) for value in values):
            return None
        norm = math.sqrt(sum(float(value) * float(value) for value in values))
        if norm < 1.0e-6:
            return None
        return [float(value) / norm for value in values]

    def _state_fresh(self):
        return (self.state_receive_time is not None and
                time.monotonic() - self.state_receive_time <= self.state_timeout)

    def _extended_state_fresh(self):
        return (self.extended_receive_time is not None and
                time.monotonic() - self.extended_receive_time <= self.extended_state_timeout)

    def _state_cb(self, msg):
        self.fcu = msg
        self.state_receive_time = time.monotonic()

    def _extended_cb(self, msg):
        self.extended = msg
        self.extended_receive_time = time.monotonic()

    def _estimator_cb(self, msg):
        self.estimator = msg
        self.estimator_receive_time = time.monotonic()

    def _reject_local_odom(self, reason):
        """Discard unsafe local odometry and abort an active mission immediately."""
        self.local_rejection_reason = reason
        self.local_odom = None
        self.local_receive_time = None
        self.last_local_stamp = None
        self.last_local_position = None
        self.last_local_yaw = None
        if self.state not in (self.IDLE, self.COMPLETE, self.ABORT):
            self._abort("local_position_%s" % reason)

    def _local_cb(self, msg):
        if self.strict_local_frames and (
                msg.header.frame_id != self.expected_local_parent or
                msg.child_frame_id != self.expected_local_child):
            self._reject_local_odom(
                "unexpected_frames:%s->%s" % (msg.header.frame_id, msg.child_frame_id))
            return
        if not self._finite_pose(msg):
            self._reject_local_odom("nonfinite_pose")
            return
        normalized = self._normalized_quaternion(msg.pose.pose.orientation)
        if normalized is None:
            self._reject_local_odom("invalid_quaternion")
            return
        stamp = msg.header.stamp
        if stamp == rospy.Time(0):
            self._reject_local_odom("zero_timestamp")
            return
        if self.last_local_stamp is not None and stamp <= self.last_local_stamp:
            self._reject_local_odom("non_monotonic_timestamp")
            return
        now = rospy.Time.now()
        if now == rospy.Time(0):
            self._reject_local_odom("ros_clock_unavailable")
            return
        age = (now - stamp).to_sec()
        if age > self.local_stamp_timeout or age < -self.local_stamp_future_tolerance:
            self._reject_local_odom("timestamp_age:%.3f" % age)
            return
        position = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y),
                    float(msg.pose.pose.position.z))
        _, _, yaw = euler_from_quaternion(normalized)
        if self.last_local_stamp is not None:
            dt = (stamp - self.last_local_stamp).to_sec()
            if dt <= 0.0:
                self._reject_local_odom("non_monotonic_timestamp")
                return
            if self.last_local_position is not None:
                speed = math.sqrt(sum(
                    (position[index] - self.last_local_position[index]) ** 2
                    for index in range(3))) / dt
                if speed > self.max_local_position_speed:
                    self._reject_local_odom("position_jump_speed:%.2f" % speed)
                    return
            if self.last_local_yaw is not None:
                yaw_rate = abs(self._angle_error(yaw, self.last_local_yaw)) / dt
                if yaw_rate > self.max_local_yaw_rate:
                    self._reject_local_odom("yaw_jump_rate:%.2f" % yaw_rate)
                    return
        msg.pose.pose.orientation.x = normalized[0]
        msg.pose.pose.orientation.y = normalized[1]
        msg.pose.pose.orientation.z = normalized[2]
        msg.pose.pose.orientation.w = normalized[3]
        self.local_odom = msg
        self.local_receive_time = time.monotonic()
        self.last_local_stamp = stamp
        self.last_local_position = position
        self.last_local_yaw = yaw
        self.local_rejection_reason = None

    def _vision_cb(self, msg):
        self.vision_healthy = bool(msg.data)
        self.vision_receive_time = time.monotonic()

    def _vision_output_cb(self, msg):
        self.vision_output_enabled = bool(msg.data)
        self.vision_output_enabled_receive_time = time.monotonic()

    def _vision_status_cb(self, msg):
        self.vision_status = str(msg.data).strip() or "empty"
        self.vision_status_receive_time = time.monotonic()

    def _timesync_cb(self, msg):
        self.timesync = msg
        self.timesync_receive_time = time.monotonic()
        rtt = float(msg.round_trip_time_ms)
        if not math.isfinite(rtt) or rtt < 0.0:
            self.timesync_issue = "invalid_rtt"
        elif rtt > self.max_timesync_rtt_ms:
            self.timesync_issue = "rtt_ms:%.2f" % rtt
        else:
            self.timesync_issue = "ok"

    def _reject_vision_output(self, reason):
        self.vision_output_receive_time = None
        self.vision_output_stamp = None
        self.vision_output_rejection_reason = reason

    def _vision_pose_cb(self, msg):
        if msg.header.frame_id != self.vision_output_parent:
            self._reject_vision_output("unexpected_parent:%s" % msg.header.frame_id)
            return
        if not self._finite_geometry_pose(msg.pose.pose):
            self._reject_vision_output("nonfinite_or_invalid_quaternion")
            return
        normalized = self._normalized_quaternion(msg.pose.pose.orientation)
        if normalized is None:
            self._reject_vision_output("invalid_quaternion")
            return
        stamp = msg.header.stamp
        if stamp == rospy.Time(0):
            self._reject_vision_output("zero_timestamp")
            return
        now = rospy.Time.now()
        if now == rospy.Time(0):
            self._reject_vision_output("ros_clock_unavailable")
            return
        age = (now - stamp).to_sec()
        if (age > self.vision_output_stamp_timeout or
                age < -self.vision_output_stamp_future_tolerance):
            self._reject_vision_output("timestamp_age:%.3f" % age)
            return
        self.vision_output_receive_time = time.monotonic()
        self.vision_output_stamp = stamp
        self.vision_output_rejection_reason = None

    def _payload_status_cb(self, msg):
        fields = {}
        for token in msg.data.replace(";", " ").split():
            if "=" in token:
                key, value = token.split("=", 1)
                fields[key.strip().lower()] = value.strip().lower()
        state = fields.get("state")
        success = fields.get("success")
        boot_id = fields.get("boot")
        try:
            sequence = int(fields.get("seq", ""))
        except ValueError:
            return
        if not boot_id:
            return
        if success == "true":
            succeeded = True
        elif success == "false":
            succeeded = False
        else:
            return
        if state in ("open", "closed"):
            if boot_id != self.payload_ack_boot_id:
                self.payload_ack_boot_id = boot_id
                self.payload_ack_sequence = 0
                self.payload_ack_state = None
                self.payload_ack_success = None
                self.payload_ack_time = None
            if sequence <= self.payload_ack_sequence:
                return
            self.payload_ack_state = state
            self.payload_ack_success = succeeded
            self.payload_ack_sequence = sequence
            self.payload_ack_time = time.monotonic()
            if self.payload_expected_state == state and succeeded:
                self.payload_state = state

    def _waypoints_cb(self, msg):
        if self.state != self.IDLE:
            self.last_error = "waypoints_locked_during_mission"
            return
        if msg.header.frame_id != self.input_frame:
            self.last_error = "waypoint_frame_must_be_%s" % self.input_frame
            self._publish_status()
            return
        converted = []
        for pose in msg.poses:
            values = [pose.position.x, pose.position.y, pose.position.z]
            normalized = self._normalized_quaternion(pose.orientation)
            if not all(math.isfinite(float(value)) for value in values) or normalized is None:
                self.last_error = "waypoint_pose_nonfinite_or_invalid_quaternion"
                self._publish_status()
                return
            _, _, yaw = euler_from_quaternion(normalized)
            converted.append({"x": float(pose.position.x), "y": float(pose.position.y),
                              "z": float(pose.position.z),
                              "yaw": yaw, "hold": self.hold_seconds,
                              "payload_action": "none", "payload_settle": 0.0})
        self.waypoints = converted
        self.last_error = "waypoints_loaded=%d" % len(converted)
        self._publish_status()

    def _start_cb(self, _request):
        if self.state not in (self.IDLE, self.COMPLETE, self.ABORT):
            return TriggerResponse(False, "mission_already_active:%s" % self.state)
        if not self._state_fresh():
            return TriggerResponse(False, "mavros_state_stale")
        if not self._extended_state_fresh():
            return TriggerResponse(False, "mavros_extended_state_stale")
        if not self.fcu.connected:
            return TriggerResponse(False, "mavros_not_connected")
        if self.fcu.armed:
            return TriggerResponse(False, "refuse_start_while_armed")
        if self.extended.landed_state != ExtendedState.LANDED_STATE_ON_GROUND:
            return TriggerResponse(False, "vehicle_not_reported_on_ground")
        if self.local_odom is None:
            reason = self.local_rejection_reason or "unavailable"
            return TriggerResponse(False, "local_position_%s" % reason)
        if time.monotonic() - self.local_receive_time > self.local_pose_timeout:
            return TriggerResponse(False, "local_position_stale")
        vision_issue = self._vision_flight_issue()
        if vision_issue is not None:
            return TriggerResponse(False, vision_issue)
        if (self.enable_control and self.require_vision_output and
                self.require_vision_output_consumer and
                not self._vision_output_consumer_present()):
            return TriggerResponse(False, "mavros_vision_pose_consumer_unavailable")
        if (self.enable_control and self.require_setpoint_consumer and
                not self._setpoint_consumer_present()):
            return TriggerResponse(False, "mavros_setpoint_raw_consumer_unavailable")
        if self.require_estimator and not self._estimator_ok():
            return TriggerResponse(False, "px4_estimator_unhealthy_or_stale")
        if self.enable_control and self.require_auto_land and not self.auto_land:
            return TriggerResponse(False, "auto_land_required_for_this_route")
        if not self._validate_waypoints():
            return TriggerResponse(False, self.last_error)
        if (self.enable_payload and self._has_payload_actions() and
                self.payload_required_connection and
                (self.payload_pub is None or self.payload_pub.get_num_connections() < 1)):
            return TriggerResponse(False, "payload_subscriber_unavailable")
        if (self.enable_payload and self.payload_require_ack and
                (self.payload_ack_time is None or self.payload_ack_boot_id is None)):
            return TriggerResponse(False, "payload_status_unavailable")
        self.payload_action_index = None
        self.payload_action_started = None
        self.payload_expected_state = None
        self.payload_command_time = None
        self.payload_command_sequence = None
        self.payload_command_boot_id = None
        self.payload_ack_state = None
        self.payload_ack_success = None
        self.payload_ack_time = None
        if self.enable_payload and self.payload_preflight_close:
            if not self._publish_payload(False):
                return TriggerResponse(False, "payload_preflight_close_failed")
        p = self.local_odom.pose.pose.position
        q = self.local_odom.pose.pose.orientation
        self.origin = (p.x, p.y, p.z)
        _, _, self.origin_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.index = 0
        self.target = (self.origin[0], self.origin[1], self.origin[2], self.origin_yaw)
        self.reached_since = None
        self.mission_started = time.monotonic()
        self.abort_land_requested = False
        self.abort_land_deadline = None
        self.abort_reason = None
        self.abort_action = "release"
        self.abort_setpoint_policy = "release"
        self.control_tx_enabled = self.enable_control
        self.last_consumer_check_wall = 0.0
        self.last_consumer_issue = None
        self._enter(self.PAYLOAD_PREPARE if
                    self.enable_payload and self.payload_preflight_close else self.PRESTREAM)
        return TriggerResponse(True, "mission_started_control_enabled=%s" % self.enable_control)

    def _abort_cb(self, _request):
        if self.state == self.IDLE:
            return TriggerResponse(False, "mission_idle")
        self._abort("operator_abort", capture_abort_position=True,
                    action=self.operator_abort_action)
        return TriggerResponse(True, "mission_aborted")

    def _land_cb(self, _request):
        if self.state not in (self.TAKEOFF, self.WAYPOINTS, self.WAIT_LAND):
            return TriggerResponse(False, "cannot_land_from_%s" % self.state)
        self._enter(self.LANDING)
        return TriggerResponse(True, "landing_requested")

    def _reset_cb(self, _request):
        if not self._state_fresh():
            return TriggerResponse(False, "mavros_state_stale")
        if self.fcu.armed:
            return TriggerResponse(False, "refuse_reset_while_armed")
        self.origin = None
        self.target = None
        self.index = 0
        self.abort_land_requested = False
        self.abort_land_deadline = None
        self.abort_reason = None
        self.abort_action = "release"
        self.abort_setpoint_policy = "release"
        self.control_tx_enabled = False
        self.last_consumer_check_wall = 0.0
        self.last_consumer_issue = None
        self.landing_started = None
        self.auto_land_request_started = None
        self.payload_action_index = None
        self.payload_action_started = None
        self.payload_expected_state = None
        self.payload_command_time = None
        self.payload_command_sequence = None
        self.payload_command_boot_id = None
        self.payload_ack_state = None
        self.payload_ack_success = None
        self.payload_ack_time = None
        self.payload_state = "disabled" if not self.enable_payload else "uncommanded"
        self._enter(self.IDLE)
        return TriggerResponse(True, "reset")

    def _validate_waypoints(self):
        if not self.waypoints:
            self.last_error = "no_waypoints"
            return False
        for i, waypoint in enumerate(self.waypoints):
            if not all(math.isfinite(waypoint[key]) for key in
                       ("x", "y", "z", "yaw", "hold", "payload_settle")):
                self.last_error = "waypoint_%d_nonfinite" % i
                return False
            if math.hypot(waypoint["x"], waypoint["y"]) > self.max_xy or \
               waypoint["z"] < -0.1 or waypoint["z"] > self.max_z:
                self.last_error = "waypoint_%d_out_of_bounds" % i
                return False
            if waypoint["hold"] < 0.0 or waypoint["hold"] > self.waypoint_timeout:
                self.last_error = "waypoint_%d_invalid_hold" % i
                return False
            if waypoint["payload_action"] not in ("none", "open", "close"):
                self.last_error = "waypoint_%d_invalid_payload_action" % i
                return False
            if waypoint["payload_settle"] < 0.0 or waypoint["payload_settle"] > 30.0:
                self.last_error = "waypoint_%d_invalid_payload_settle" % i
                return False
        if self.input_frame not in ("robotac_local_enu", "robotac_start_body"):
            self.last_error = "invalid_waypoint_frame"
            return False
        if self.require_vision_output and not self.require_vision:
            self.last_error = "require_vision_output_requires_vision"
            return False
        if not self.vision_output_parent:
            self.last_error = "invalid_vision_output_parent"
            return False
        if self.require_vision_output_consumer and not self.vision_output_consumer_node:
            self.last_error = "invalid_vision_output_consumer_node"
            return False
        if self.require_setpoint_consumer and (
                not self.setpoint_topic or not self.setpoint_consumer_node):
            self.last_error = "invalid_setpoint_consumer_config"
            return False
        if not math.isfinite(self.consumer_check_interval) or self.consumer_check_interval <= 0.0:
            self.last_error = "invalid_consumer_check_interval"
            return False
        if self.strict_local_frames and (
                not self.expected_local_parent or not self.expected_local_child):
            self.last_error = "invalid_expected_local_frames"
            return False
        positive_parameters = (
            self.prestream_seconds, self.control_rate, self.position_tolerance,
            self.yaw_tolerance, self.waypoint_timeout, self.mission_timeout,
            self.local_pose_timeout, self.local_stamp_timeout, self.state_timeout,
            self.extended_state_timeout, self.vision_timeout, self.vision_status_timeout,
            self.vision_output_timeout, self.vision_output_stamp_timeout,
            self.estimator_timeout, self.offboard_timeout, self.arming_timeout,
            self.max_xy, self.max_z, self.timesync_timeout,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in positive_parameters):
            self.last_error = "invalid_positive_flight_parameter"
            return False
        if (not math.isfinite(self.local_stamp_future_tolerance) or
                self.local_stamp_future_tolerance < 0.0 or
                not math.isfinite(self.vision_output_stamp_future_tolerance) or
                self.vision_output_stamp_future_tolerance < 0.0):
            self.last_error = "invalid_timestamp_future_tolerance"
            return False
        if (not math.isfinite(self.max_timesync_rtt_ms) or
                self.max_timesync_rtt_ms < 0.0):
            self.last_error = "invalid_timesync_rtt_limit"
            return False
        if (not math.isfinite(self.max_local_position_speed) or
                not math.isfinite(self.max_local_yaw_rate) or
                self.max_local_position_speed <= 0.0 or self.max_local_yaw_rate <= 0.0):
            self.last_error = "invalid_local_jump_limits"
            return False
        if self.takeoff_height <= 0.1 or self.takeoff_height > self.max_z:
            self.last_error = "invalid_takeoff_height"
            return False
        if self.takeoff_timeout <= 0.0:
            self.last_error = "invalid_takeoff_timeout"
            return False
        if self.land_descent_rate <= 0.0 or self.land_switch_height < 0.0:
            self.last_error = "invalid_landing_parameters"
            return False
        if self.critical_fault_action not in ("release", "land"):
            self.last_error = "invalid_critical_fault_action"
            return False
        if self.operator_abort_action not in ("hold", "release", "land"):
            self.last_error = "invalid_operator_abort_action"
            return False
        if self.abort_land_mode_timeout <= 0.0:
            self.last_error = "invalid_abort_land_mode_timeout"
            return False
        if self.auto_land_mode_timeout <= 0.0 or self.landing_timeout <= 0.0:
            self.last_error = "invalid_landing_timeout"
            return False
        if self.payload_default_settle < 0.0 or self.payload_default_settle > 30.0:
            self.last_error = "invalid_payload_default_settle"
            return False
        if self.payload_preflight_close and self.prestream_seconds < self.payload_default_settle:
            self.last_error = "prestream_shorter_than_payload_preflight_settle"
            return False
        if self.payload_require_ack and self.payload_ack_timeout <= 0.0:
            self.last_error = "invalid_payload_ack_timeout"
            return False
        return True

    def _enter(self, state, capture_abort_position=False):
        if state == self.ABORT:
            current = self._current_position()
            if (capture_abort_position and current is not None and
                    self.local_receive_time is not None and
                    time.monotonic() - self.local_receive_time <= self.local_pose_timeout):
                self.target = current
        elif state == self.LANDING:
            current = self._current_position()
            if current is not None:
                self.landing_target = current
            self.landing_started = time.monotonic()
            self.auto_land_request_started = None
        else:
            self.abort_land_requested = False
        self.state = state
        self.state_started = time.monotonic()
        self.reached_since = None
        self._publish_status()

    def _abort(self, reason, capture_abort_position=False, action=None):
        """Stop active control safely after a critical fault or operator request."""
        action = self.critical_fault_action if action is None else action
        if action not in ("hold", "release", "land"):
            rospy.logerr("invalid abort action %s; forcing release", action)
            action = "release"
        self.last_error = reason
        self.abort_reason = reason
        self.abort_action = action
        self.abort_setpoint_policy = "hold" if action == "hold" else "release"
        if action != "hold":
            self.control_tx_enabled = False
        if self.state != self.ABORT:
            self._enter(self.ABORT, capture_abort_position=capture_abort_position)

        if action == "land":
            self.abort_land_requested = True
            self.abort_land_deadline = time.monotonic() + self.abort_land_mode_timeout
            # This is a best-effort request only.  The transmission gate stays
            # closed even if MAVROS is disconnected, so PX4 offboard-loss
            # failsafe remains the authoritative communication-loss behavior.
            if self.enable_control and self.fcu.armed:
                self._request_mode(self.land_mode)
        return True

    def _request_mode(self, mode):
        if not self.enable_control or not self._state_fresh() or not self.fcu.connected:
            return False
        now = time.monotonic()
        if now - self.last_service_wall.get("mode", 0.0) < 2.0:
            return False
        try:
            if self.mode_srv is None:
                self.mode_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)
            result = self.mode_srv(0, mode)
            self.last_service_wall["mode"] = now
            return bool(result.mode_sent)
        except (rospy.ServiceException, rospy.ROSException) as exc:
            self.last_error = "set_mode_failed:%s" % exc
            return False

    def _request_arm(self):
        if not self.enable_control or not self._state_fresh() or not self.fcu.connected:
            return False
        now = time.monotonic()
        if now - self.last_service_wall.get("arm", 0.0) < 2.0:
            return False
        try:
            if self.arm_srv is None:
                self.arm_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
            result = self.arm_srv(True)
            self.last_service_wall["arm"] = now
            return bool(result.success)
        except (rospy.ServiceException, rospy.ROSException) as exc:
            self.last_error = "arming_failed:%s" % exc
            return False

    def _current_position(self):
        if self.local_odom is None:
            return None
        p = self.local_odom.pose.pose.position
        q = self.local_odom.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return (p.x, p.y, p.z, yaw)

    def _ground_confirmed(self):
        return (self._state_fresh() and self._extended_state_fresh() and
                self.extended.landed_state == ExtendedState.LANDED_STATE_ON_GROUND and
                not self.fcu.armed)

    def _estimator_ok(self):
        if self.estimator is None or self.estimator_receive_time is None:
            return False
        if time.monotonic() - self.estimator_receive_time > self.estimator_timeout:
            return False
        if not self.estimator.attitude_status_flag:
            return False
        if self.require_horizontal_relative and not self.estimator.pos_horiz_rel_status_flag:
            return False
        if self.require_vertical_estimate and not (
                self.estimator.pos_vert_abs_status_flag or
                self.estimator.pos_vert_agl_status_flag):
            return False
        return True

    @staticmethod
    def _fresh(receive_time, timeout):
        return (receive_time is not None and
                time.monotonic() - receive_time <= timeout)

    def _vision_flight_issue(self):
        """Return a precise reason when the vision chain is unsafe for flight."""
        if self.require_vision:
            if not self.vision_healthy or not self._fresh(
                    self.vision_receive_time, self.vision_timeout):
                return "fastlio_vision_lost"
            if not self._fresh(self.vision_status_receive_time, self.vision_status_timeout):
                return "fastlio_vision_status_stale"
            if not (self.vision_status == "ok" or self.vision_status.startswith("ok ")):
                return "fastlio_vision_status_unhealthy"
        if self.enable_control and self.require_vision_output:
            if not self.vision_output_enabled or not self._fresh(
                    self.vision_output_enabled_receive_time, self.vision_output_timeout):
                return "mavros_vision_output_disabled"
            if not self._fresh(self.vision_output_receive_time, self.vision_output_timeout):
                if self.vision_output_rejection_reason:
                    return "mavros_vision_pose_%s" % self.vision_output_rejection_reason
                return "mavros_vision_pose_timeout"
        if self.enable_control and self.require_timesync:
            if not self._fresh(self.timesync_receive_time, self.timesync_timeout):
                return "mavros_timesync_stale"
            if self.timesync_issue != "ok":
                return "mavros_timesync_unhealthy:%s" % self.timesync_issue
        return None

    def _topic_consumer_present(self, target_topic, node_name, description):
        try:
            code, _message, state = rospy.get_master().getSystemState()
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "unable to inspect ROS graph for %s: %s",
                                   description, exc)
            return False
        if code != 1:
            return False
        subscribers = state[1]
        for topic, nodes in subscribers:
            if topic == target_topic:
                return node_name in nodes
        return False

    def _vision_output_consumer_present(self):
        """Confirm the MAVROS vision plugin has subscribed before takeoff."""
        return self._topic_consumer_present(
            self.vision_output_topic, self.vision_output_consumer_node, "MAVROS vision")

    def _setpoint_consumer_present(self):
        """Confirm MAVROS setpoint_raw has subscribed before active control."""
        return self._topic_consumer_present(
            self.setpoint_topic, self.setpoint_consumer_node, "MAVROS setpoint_raw")

    def _active_consumer_issue(self):
        """Return a precise issue if a required MAVROS consumer disappeared."""
        if not self.enable_control:
            self.last_consumer_issue = None
            return None
        now = time.monotonic()
        if now - self.last_consumer_check_wall < self.consumer_check_interval:
            return self.last_consumer_issue
        self.last_consumer_check_wall = now
        issue = None
        if (self.require_vision_output and self.require_vision_output_consumer and
                not self._vision_output_consumer_present()):
            issue = "mavros_vision_pose_consumer_lost"
        elif self.require_setpoint_consumer and not self._setpoint_consumer_present():
            issue = "mavros_setpoint_raw_consumer_lost"
        self.last_consumer_issue = issue
        return issue

    def _has_payload_actions(self):
        return any(waypoint["payload_action"] != "none" for waypoint in self.waypoints)

    def _absolute_target(self, relative):
        if self.input_frame == "robotac_start_body":
            # FLU offsets at mission start become fixed ENU offsets for MAVROS.
            cos_yaw = math.cos(self.origin_yaw)
            sin_yaw = math.sin(self.origin_yaw)
            offset_x = cos_yaw * relative["x"] - sin_yaw * relative["y"]
            offset_y = sin_yaw * relative["x"] + cos_yaw * relative["y"]
        else:
            offset_x = relative["x"]
            offset_y = relative["y"]
        return (self.origin[0] + offset_x, self.origin[1] + offset_y,
                self.origin[2] + relative["z"],
                self._angle_error(self.origin_yaw + relative["yaw"], 0.0))

    def _publish_payload(self, is_open):
        if not self.enable_payload:
            return True
        if self.payload_pub is None:
            return False
        if self.payload_required_connection and self.payload_pub.get_num_connections() < 1:
            return False
        self.payload_expected_state = "open" if is_open else "closed"
        self.payload_command_sequence = self.payload_ack_sequence
        self.payload_command_boot_id = self.payload_ack_boot_id
        self.payload_command_time = time.monotonic()
        self.payload_state = "awaiting_%s" % self.payload_expected_state
        self.payload_pub.publish(Bool(data=is_open))
        return True

    def _payload_acknowledged(self):
        if not self.enable_payload or not self.payload_require_ack:
            return True
        return (self.payload_expected_state is not None and
                self.payload_ack_state == self.payload_expected_state and
                self.payload_ack_success is True and
                self.payload_command_sequence is not None and
                self.payload_command_boot_id is not None and
                self.payload_ack_boot_id == self.payload_command_boot_id and
                self.payload_ack_sequence > self.payload_command_sequence and
                self.payload_ack_time is not None and
                self.payload_command_time is not None and
                self.payload_ack_time >= self.payload_command_time)

    def _payload_ack_failed(self):
        return (self.enable_payload and self.payload_require_ack and
                self.payload_expected_state is not None and
                self.payload_command_sequence is not None and
                self.payload_command_boot_id is not None and
                self.payload_ack_time is not None and
                self.payload_command_time is not None and
                self.payload_ack_time >= self.payload_command_time and
                (self.payload_ack_boot_id != self.payload_command_boot_id or
                 (self.payload_ack_sequence > self.payload_command_sequence and
                  (self.payload_ack_success is not True or
                   self.payload_ack_state != self.payload_expected_state))))

    def _payload_ack_timed_out(self):
        return (self.enable_payload and self.payload_require_ack and
                self.payload_command_time is not None and
                time.monotonic() - self.payload_command_time > self.payload_ack_timeout)

    def _finish_waypoint(self):
        waypoint = self.waypoints[self.index]
        action = waypoint["payload_action"]
        if action != "none":
            now = time.monotonic()
            if self.payload_action_index != self.index:
                self.payload_action_index = self.index
                self.payload_action_started = now
                if self.enable_payload:
                    if not self._publish_payload(action == "open"):
                        self._abort("payload_subscriber_lost")
                        return
                else:
                    self.payload_state = "simulated_%s" % action
                return
            if not self._payload_acknowledged():
                if self._payload_ack_failed():
                    self._abort("payload_ack_failed")
                elif self._payload_ack_timed_out():
                    self._abort("payload_ack_timeout")
                return
            settle_started = (self.payload_ack_time if self.enable_payload and
                              self.payload_require_ack else self.payload_action_started)
            if now - settle_started < waypoint["payload_settle"]:
                return
        self.index += 1
        self._enter(self.WAYPOINTS)

    def _make_setpoint(self, target):
        msg = PositionTarget()
        msg.header.stamp = rospy.Time.now()
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask = (PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY |
                         PositionTarget.IGNORE_VZ | PositionTarget.IGNORE_AFX |
                         PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                         PositionTarget.IGNORE_YAW_RATE)
        msg.position.x, msg.position.y, msg.position.z = target[:3]
        msg.yaw = target[3]
        msg.yaw_rate = 0.0
        return msg

    def _publish_setpoint(self, target):
        msg = self._make_setpoint(target)
        self.preview_pub.publish(msg)
        if self.enable_control and self.control_tx_enabled and self.setpoint_pub is not None:
            self.setpoint_pub.publish(msg)
            self.last_setpoint_wall = time.monotonic()
        self.last_setpoint = msg

    def _publish_preview(self, target):
        msg = self._make_setpoint(target)
        self.preview_pub.publish(msg)
        self.last_setpoint = msg

    def _at_target(self, target):
        current = self._current_position()
        if current is None:
            return False
        distance = math.sqrt(sum((current[i] - target[i]) ** 2 for i in range(3)))
        yaw_error = abs(self._angle_error(current[3], target[3]))
        return distance <= self.position_tolerance and yaw_error <= self.yaw_tolerance

    def _tick(self, _event):
        if self.state == self.IDLE:
            self._publish_status()
            return
        mission_active = self.state not in (self.ABORT, self.COMPLETE)
        if mission_active:
            if self.mission_started and time.monotonic() - self.mission_started > self.mission_timeout:
                self._abort("mission_timeout")
            elif not self._state_fresh():
                self._abort("mavros_state_stale")
            elif not self.fcu.connected:
                self._abort("mavros_disconnected")
            elif not self._extended_state_fresh():
                self._abort("mavros_extended_state_stale")
            elif (self.local_receive_time is None or
                  time.monotonic() - self.local_receive_time > self.local_pose_timeout):
                self._abort("local_position_timeout")
            else:
                vision_issue = self._vision_flight_issue()
                if vision_issue is not None:
                    self._abort(vision_issue)
                elif self.require_estimator and not self._estimator_ok():
                    self._abort("px4_estimator_lost")
                else:
                    consumer_issue = self._active_consumer_issue()
                    if consumer_issue is not None:
                        self._abort(consumer_issue)

        if self.state in (self.PAYLOAD_PREPARE, self.PRESTREAM, self.WAIT_OFFBOARD) and self.fcu.armed:
            self._abort("unexpected_armed_before_offboard")
        elif self.state in (self.TAKEOFF, self.WAYPOINTS, self.WAIT_LAND):
            if self.fcu.mode != "OFFBOARD":
                self._abort("offboard_mode_lost")
        elif self.state == self.LANDING and self.fcu.mode not in ("OFFBOARD", self.land_mode):
            self._abort("landing_mode_lost")

        if self.state == self.PAYLOAD_PREPARE:
            self._publish_setpoint(self.target)
            if self._payload_acknowledged():
                self._enter(self.PRESTREAM)
            elif self._payload_ack_failed():
                self._abort("payload_preflight_ack_failed")
            elif self._payload_ack_timed_out():
                self._abort("payload_preflight_ack_timeout")
        elif self.state == self.PRESTREAM:
            self._publish_setpoint(self.target)
            if time.monotonic() - self.state_started >= self.prestream_seconds:
                self._enter(self.WAIT_OFFBOARD)
        elif self.state == self.WAIT_OFFBOARD:
            self._publish_setpoint(self.target)
            if self.fcu.mode == "OFFBOARD":
                self._enter(self.WAIT_ARMED)
            elif time.monotonic() - self.state_started > self.offboard_timeout:
                self._abort("offboard_timeout")
            elif self.auto_mode:
                self._request_mode("OFFBOARD")
        elif self.state == self.WAIT_ARMED:
            self._publish_setpoint(self.target)
            if self.fcu.mode != "OFFBOARD":
                self._abort("offboard_mode_lost_during_arming")
            elif self.fcu.armed:
                self._enter(self.TAKEOFF)
            elif time.monotonic() - self.state_started > self.arming_timeout:
                self._abort("arming_timeout")
            elif self.auto_arm:
                self._request_arm()
        elif self.state == self.TAKEOFF:
            self.target = (self.origin[0], self.origin[1], self.origin[2] + self.takeoff_height, self.origin_yaw)
            self._publish_setpoint(self.target)
            if self._at_target(self.target):
                if self.reached_since is None:
                    self.reached_since = time.monotonic()
                elif time.monotonic() - self.reached_since >= self.hold_seconds:
                    self._enter(self.WAYPOINTS if self.waypoints else self.WAIT_LAND)
            elif time.monotonic() - self.state_started > self.takeoff_timeout:
                self._abort("takeoff_timeout")
        elif self.state == self.WAYPOINTS:
            if self.index >= len(self.waypoints):
                self._enter(self.LANDING if self.auto_land else self.WAIT_LAND)
            else:
                self.target = self._absolute_target(self.waypoints[self.index])
                self._publish_setpoint(self.target)
                if self._at_target(self.target):
                    if self.reached_since is None:
                        self.reached_since = time.monotonic()
                    elif time.monotonic() - self.reached_since >= self.waypoints[self.index]["hold"]:
                        self._finish_waypoint()
                elif time.monotonic() - self.state_started > self.waypoint_timeout:
                    self._abort("waypoint_%d_timeout" % self.index)
        elif self.state == self.WAIT_LAND:
            self._publish_setpoint(self.target)
        elif self.state == self.LANDING:
            if (self.landing_started is not None and
                    time.monotonic() - self.landing_started > self.landing_timeout):
                self._abort("landing_timeout")
                self._publish_status()
                return
            if self.landing_target is None:
                self.landing_target = self._current_position()
            switch_z = self.origin[2] + self.land_switch_height
            if self.fcu.mode != self.land_mode and self.landing_target is not None:
                next_z = max(switch_z,
                             self.landing_target[2] - self.land_descent_rate / max(1.0, self.control_rate))
                self.landing_target = (self.landing_target[0], self.landing_target[1],
                                       next_z, self.landing_target[3])
                self.target = self.landing_target
                self._publish_setpoint(self.target)
                if next_z <= switch_z + 1.0e-3 and self.enable_control:
                    if self.auto_land_request_started is None:
                        self.auto_land_request_started = time.monotonic()
                    elif (time.monotonic() - self.auto_land_request_started >
                          self.auto_land_mode_timeout):
                        self._abort("auto_land_mode_timeout")
                        self._publish_status()
                        return
                    self._request_mode(self.land_mode)
            if self._ground_confirmed():
                self._enter(self.COMPLETE)
        elif self.state == self.ABORT:
            if self.abort_land_requested:
                if self._state_fresh() and self.fcu.connected and self.fcu.mode == self.land_mode:
                    self.abort_land_requested = False
                elif self.abort_land_deadline is not None and time.monotonic() > self.abort_land_deadline:
                    self.abort_land_requested = False
                    self.last_error = "%s:auto_land_unconfirmed" % self.abort_reason
                elif self._state_fresh() and self.fcu.connected and self.fcu.armed:
                    self._request_mode(self.land_mode)
            if (self.abort_setpoint_policy == "hold" and self.control_tx_enabled and
                    self.target is not None):
                self._publish_setpoint(self.target)
            elif self.target is not None:
                self._publish_preview(self.target)
        elif self.state == self.COMPLETE:
            self._publish_status()
        self._publish_status()

    def _publish_status(self):
        event_stamp = rospy.Time.now().to_sec()
        self.status_pub.publish(String(data="state=%s waypoint=%d/%d connected=%s armed=%s mode=%s vision=%s estimator=%s timesync=%s payload=%s abort_action=%s tx=%s error=%s stamp=%.9f" %
                                      (self.state, self.index, len(self.waypoints), self.fcu.connected,
                                       self.fcu.armed, self.fcu.mode, self.vision_healthy,
                                       self._estimator_ok(), self.timesync_issue, self.payload_state, self.abort_action,
                                       self.control_tx_enabled, self.last_error, event_stamp)))
        self.active_pub.publish(Bool(data=self.state not in (self.IDLE, self.COMPLETE, self.ABORT)))


if __name__ == "__main__":
    rospy.init_node("local_waypoint_flight")
    LocalWaypointFlight()
    rospy.spin()
