#!/usr/bin/env python3
"""Read-only observer for a real Robotac local waypoint flight.

This node is an evidence recorder, not a controller. It only subscribes to the
flight controller, MAVROS state, MAVROS local position, the actual MAVROS raw
setpoint topic, and optional servo status topics. It never publishes setpoints,
calls services, changes modes, arms, lands, or writes PX4 parameters.
"""

import json
import math
import pathlib
import sys
import time

import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from mavros_msgs.msg import ExtendedState, PositionTarget, State
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String


def _as_bool(value):
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in ("true", "1", "yes", "on"):
            return True
        if normalized in ("false", "0", "no", "off", ""):
            return False
        raise ValueError("invalid Boolean value: %s" % value)
    return bool(value)


def _parse_fields(text):
    fields = {}
    for token in str(text).replace(";", " ").split():
        if "=" in token:
            key, value = token.split("=", 1)
            fields[key.strip().lower()] = value.strip()
    return fields


def _finite(values):
    return all(math.isfinite(float(value)) for value in values)


def _yaw_from_quaternion(quaternion):
    x = float(quaternion.x)
    y = float(quaternion.y)
    z = float(quaternion.z)
    w = float(quaternion.w)
    return math.atan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z))


class ActiveFlightObserver(object):
    def __init__(self):
        self.observe_timeout = float(rospy.get_param("~observe_timeout", 900.0))
        self.stream_timeout = float(rospy.get_param("~stream_timeout", 1.0))
        self.min_setpoints = int(rospy.get_param("~min_setpoints", 20))
        self.require_raw_setpoints = _as_bool(rospy.get_param("~require_raw_setpoints", True))
        self.min_raw_setpoints = int(rospy.get_param("~min_raw_setpoints", 20))
        self.min_active_raw_setpoints = int(rospy.get_param("~min_active_raw_setpoints", 20))
        self.min_active_unique_raw_setpoints = int(rospy.get_param("~min_active_unique_raw_setpoints", 2))
        self.raw_setpoint_topic = rospy.get_param("~raw_setpoint_topic", "/mavros/setpoint_raw/local")
        self.require_raw_setpoint_publisher = _as_bool(
            rospy.get_param("~require_raw_setpoint_publisher", True))
        self.raw_setpoint_expected_publisher_node = str(rospy.get_param(
            "~raw_setpoint_expected_publisher_node", "/local_waypoint_flight")).strip()
        self.raw_setpoint_publisher_check_interval = float(rospy.get_param(
            "~raw_setpoint_publisher_check_interval", 0.50))
        self.min_airborne_altitude = float(rospy.get_param("~min_airborne_altitude", 0.50))
        self.waypoint_reach_tolerance = float(rospy.get_param("~waypoint_reach_tolerance", 0.35))
        self.min_target_dwell_s = float(rospy.get_param("~min_target_dwell_s", 0.25))
        self.require_active_vision_pose = _as_bool(rospy.get_param("~require_active_vision_pose", True))
        self.min_active_vision_pose_count = int(rospy.get_param("~min_active_vision_pose_count", 5))
        self.vision_pose_type = str(rospy.get_param("~vision_pose_type", "pose")).strip().lower()
        self.expected_vision_parent = str(rospy.get_param("~expected_vision_parent", "odom")).strip()
        self.require_active_vision_local_consistency = _as_bool(rospy.get_param(
            "~require_active_vision_local_consistency", True))
        self.min_active_vision_local_pairs = int(rospy.get_param(
            "~min_active_vision_local_pairs", 5))
        self.max_active_vision_local_delta_m = float(rospy.get_param(
            "~max_active_vision_local_delta_m", 0.75))
        self.vision_local_pair_timeout = float(rospy.get_param(
            "~vision_local_pair_timeout", 0.30))
        self.require_active_mavros_control = _as_bool(
            rospy.get_param("~require_active_mavros_control", True))
        self.require_takeoff_landing_states = _as_bool(
            rospy.get_param("~require_takeoff_landing_states", True))
        self.require_waypoints_complete = _as_bool(rospy.get_param("~require_waypoints_complete", True))
        self.require_route_manifest = _as_bool(rospy.get_param("~require_route_manifest", True))
        self.route_manifest_target_tolerance = float(
            rospy.get_param("~route_manifest_target_tolerance", 0.05))
        self.route_manifest_yaw_tolerance = math.radians(float(
            rospy.get_param("~route_manifest_yaw_tolerance_deg", 1.0)))
        self.require_final_disarmed = _as_bool(rospy.get_param("~require_final_disarmed", True))
        self.require_final_on_ground = _as_bool(rospy.get_param("~require_final_on_ground", True))
        self.require_payload_open = _as_bool(rospy.get_param("~require_payload_open", False))
        self.evidence_file = str(rospy.get_param("~evidence_file", "")).strip()
        self._validate_parameters()

        self.started = time.monotonic()
        self.exit_code = 1
        self.finished = False
        self.finish_reason = "running"

        self.status_count = 0
        self.status_receive = None
        self.last_status = {}
        self.state_history = []
        self.abort_reason = None
        self.max_waypoint_index = -1
        self.total_waypoints = None
        self.current_waypoint_index = None
        self.current_waypoint_total = None

        self.setpoint_count = 0
        self.setpoint_receive = None
        self.unique_setpoints = []
        self.raw_setpoint_count = 0
        self.raw_setpoint_receive = None
        self.unique_raw_setpoints = []
        self.active_raw_setpoint_count = 0
        self.active_raw_setpoint_receive = None
        self.active_unique_raw_setpoints = []
        self.raw_setpoint_frame_mismatch_count = 0
        self.raw_setpoint_expected_publisher_seen = False
        self.raw_setpoint_publishers_seen = []
        self.raw_setpoint_last_publisher_check_wall = 0.0
        self.target_records = []
        self.route_manifest = None
        self.route_manifest_history = []
        self.route_manifest_receive = None

        self.active_vision_pose_count = 0
        self.active_vision_pose_receive = None
        self.active_vision_pose_first_stamp = None
        self.active_vision_pose_last_stamp = None
        self.active_vision_pose_parent = None
        self.vision_output_enabled_latest = None
        self.vision_output_enabled_receive = None
        self.active_vision_output_enabled_seen = False
        self.active_vision_output_enabled_receive = None
        self.active_fastlio_vision_status_ok_seen = False
        self.active_fastlio_vision_status_receive = None
        self.last_fastlio_vision_status = None
        self.latest_vision_position = None
        self.latest_vision_receive = None
        self.vision_local_origin_local = None
        self.vision_local_origin_vision = None
        self.active_vision_local_pair_count = 0
        self.active_vision_local_max_delta_error_m = None
        self.active_vision_local_sum_sq_error = 0.0
        self.active_vision_local_max_motion_m = None
        self.active_vision_local_last_local_delta = None
        self.active_vision_local_last_vision_delta = None
        self.active_vision_local_last_pair = None

        self.local_count = 0
        self.local_receive = None
        self.latest_local_position = None
        self.initial_local_position = None
        self.initial_local_yaw = None
        self.initial_local_z = None
        self.max_local_z = None
        self.max_relative_local_z = None
        self.final_local_position = None
        self.final_local_yaw = None

        self.mavros_state = None
        self.mavros_state_receive = None
        self.active_mavros_state_count = 0
        self.active_mavros_connected_seen = False
        self.active_mavros_armed_seen = False
        self.active_mavros_offboard_seen = False
        self.active_mavros_modes = []
        self.extended_state = None
        self.extended_state_receive = None
        self.active_extended_state_count = 0
        self.active_landed_states = []
        self.active_in_air_seen = False
        self.active_landing_seen = False

        self.payload_open_seen = False
        self.payload_status_receive = None

        rospy.Subscriber(rospy.get_param("~flight_status_topic", "/robotac/flight/status"),
                         String, self._flight_status_cb, queue_size=20)
        rospy.Subscriber(rospy.get_param("~setpoint_preview_topic", "/robotac/flight/setpoint_preview"),
                         PositionTarget, self._setpoint_preview_cb, queue_size=50)
        rospy.Subscriber(self.raw_setpoint_topic, PositionTarget, self._raw_setpoint_cb, queue_size=50)
        rospy.Subscriber(rospy.get_param("~local_position_topic", "/mavros/local_position/odom"),
                         Odometry, self._local_position_cb, queue_size=50)
        rospy.Subscriber(rospy.get_param("~mavros_state_topic", "/mavros/state"),
                         State, self._mavros_state_cb, queue_size=20)
        rospy.Subscriber(rospy.get_param("~extended_state_topic", "/mavros/extended_state"),
                         ExtendedState, self._extended_state_cb, queue_size=20)
        rospy.Subscriber(rospy.get_param("~payload_status_topic", "/robotac_servo/status"),
                         String, self._payload_status_cb, queue_size=10)
        rospy.Subscriber(rospy.get_param("~route_manifest_topic", "/robotac/flight/route_manifest"),
                         String, self._route_manifest_cb, queue_size=10)
        vision_pose_topic = rospy.get_param("~vision_pose_topic", "/mavros/vision_pose/pose")
        if self.vision_pose_type in ("pose", "pose_stamped", "posestamped"):
            rospy.Subscriber(vision_pose_topic, PoseStamped, self._vision_pose_cb, queue_size=50)
        elif self.vision_pose_type in (
                "pose_cov", "pose_with_covariance", "pose_with_covariance_stamped",
                "posewithcovariancestamped"):
            rospy.Subscriber(vision_pose_topic, PoseWithCovarianceStamped,
                             self._vision_pose_cov_cb, queue_size=50)
        else:
            raise ValueError("unsupported vision_pose_type: %s" % self.vision_pose_type)
        rospy.Subscriber(rospy.get_param("~vision_output_enabled_topic", "/robotac/fastlio_vision/output_enabled"),
                         Bool, self._vision_output_enabled_cb, queue_size=10)
        rospy.Subscriber(rospy.get_param("~vision_status_topic", "/robotac/fastlio_vision/status"),
                         String, self._vision_status_cb, queue_size=20)
        self.timer = rospy.Timer(rospy.Duration(0.20), self._tick)

    def _validate_parameters(self):
        if not math.isfinite(self.observe_timeout) or self.observe_timeout <= 0.0:
            raise ValueError("observe_timeout must be finite and positive")
        if not math.isfinite(self.stream_timeout) or self.stream_timeout <= 0.0:
            raise ValueError("stream_timeout must be finite and positive")
        if self.min_setpoints < 1:
            raise ValueError("min_setpoints must be positive")
        if self.require_raw_setpoints and self.min_raw_setpoints < 1:
            raise ValueError("min_raw_setpoints must be positive when raw setpoints are required")
        if self.require_raw_setpoints and self.min_active_raw_setpoints < 1:
            raise ValueError("min_active_raw_setpoints must be positive when raw setpoints are required")
        if self.require_raw_setpoints and self.min_active_unique_raw_setpoints < 1:
            raise ValueError("min_active_unique_raw_setpoints must be positive when raw setpoints are required")
        if self.require_raw_setpoint_publisher:
            if not self.raw_setpoint_topic:
                raise ValueError("raw_setpoint_topic must be non-empty when raw setpoint publisher is required")
            if not self.raw_setpoint_expected_publisher_node:
                raise ValueError("raw_setpoint_expected_publisher_node must be non-empty when required")
            if (not math.isfinite(self.raw_setpoint_publisher_check_interval) or
                    self.raw_setpoint_publisher_check_interval <= 0.0):
                raise ValueError("raw_setpoint_publisher_check_interval must be finite and positive")
        if not math.isfinite(self.min_airborne_altitude) or self.min_airborne_altitude < 0.0:
            raise ValueError("min_airborne_altitude must be finite and non-negative")
        if not math.isfinite(self.waypoint_reach_tolerance) or self.waypoint_reach_tolerance <= 0.0:
            raise ValueError("waypoint_reach_tolerance must be finite and positive")
        if not math.isfinite(self.min_target_dwell_s) or self.min_target_dwell_s < 0.0:
            raise ValueError("min_target_dwell_s must be finite and non-negative")
        if self.min_active_vision_pose_count < 0:
            raise ValueError("min_active_vision_pose_count must be non-negative")
        allowed_pose_types = (
            "pose", "pose_stamped", "posestamped",
            "pose_cov", "pose_with_covariance", "pose_with_covariance_stamped",
            "posewithcovariancestamped",
        )
        if self.vision_pose_type not in allowed_pose_types:
            raise ValueError("vision_pose_type must be pose or pose_cov")
        if self.require_active_vision_pose and not self.expected_vision_parent:
            raise ValueError("expected_vision_parent must be non-empty")
        if self.min_active_vision_local_pairs < 0:
            raise ValueError("min_active_vision_local_pairs must be non-negative")
        if not math.isfinite(self.max_active_vision_local_delta_m) or self.max_active_vision_local_delta_m < 0.0:
            raise ValueError("max_active_vision_local_delta_m must be finite and non-negative")
        if not math.isfinite(self.vision_local_pair_timeout) or self.vision_local_pair_timeout <= 0.0:
            raise ValueError("vision_local_pair_timeout must be finite and positive")
        if not math.isfinite(self.route_manifest_target_tolerance) or self.route_manifest_target_tolerance < 0.0:
            raise ValueError("route_manifest_target_tolerance must be finite and non-negative")
        if not math.isfinite(self.route_manifest_yaw_tolerance) or self.route_manifest_yaw_tolerance < 0.0:
            raise ValueError("route_manifest_yaw_tolerance_deg must be finite and non-negative")

    @staticmethod
    def _fresh(receive_time, timeout):
        return receive_time is not None and time.monotonic() - receive_time <= timeout

    @staticmethod
    def _append_unique(values, item, tolerance=1.0e-3):
        if not values or any(abs(a - b) > tolerance for a, b in zip(values[-1], item)):
            values.append(item)
            return True
        return False

    @staticmethod
    def _distance3(a, b):
        return math.sqrt(sum((float(a[i]) - float(b[i])) ** 2 for i in range(3)))

    def _target_reached(self, record):
        distance = record.get("min_distance_m")
        try:
            distance = float(distance)
            dwell = float(record.get("max_continuous_reach_s") or 0.0)
        except (TypeError, ValueError):
            return False
        return (record.get("reached") is True and math.isfinite(distance) and
                distance <= self.waypoint_reach_tolerance and
                math.isfinite(dwell) and dwell >= self.min_target_dwell_s)

    @staticmethod
    def _same_target(a, b, tolerance=1.0e-3):
        return all(abs(float(a[i]) - float(b[i])) <= tolerance for i in range(4))

    @staticmethod
    def _angle_error(a, b):
        return math.atan2(math.sin(float(a) - float(b)), math.cos(float(a) - float(b)))

    @staticmethod
    def _target_key(record):
        if not isinstance(record, dict):
            return None
        if record.get("state") == "TAKEOFF":
            return ("TAKEOFF", 0)
        if record.get("state") == "WAYPOINTS":
            try:
                return ("WAYPOINTS", int(record.get("waypoint_index")))
            except (TypeError, ValueError):
                return None
        return None

    @staticmethod
    def _record_target(record):
        if not isinstance(record, dict):
            return None
        target = record.get("target")
        return ActiveFlightObserver._target_values(target)

    @staticmethod
    def _target_values(target):
        if not isinstance(target, (list, tuple)) or len(target) < 4:
            return None
        try:
            values = tuple(float(value) for value in target[:4])
        except (TypeError, ValueError):
            return None
        return values if _finite(values) else None

    def _target_map(self, records):
        targets = {}
        if not isinstance(records, list):
            return targets
        for record in records:
            key = self._target_key(record)
            target = self._record_target(record)
            if key is not None and target is not None:
                targets[key] = target
        return targets

    def _target_present_in_raw_setpoints(self, expected, raw_setpoints=None):
        source = self.unique_raw_setpoints if raw_setpoints is None else raw_setpoints
        for raw_target in source:
            actual = self._target_values(raw_target)
            if actual is None:
                continue
            position_delta = self._distance3(actual[:3], expected[:3])
            yaw_delta = abs(self._angle_error(actual[3], expected[3]))
            if (position_delta <= self.route_manifest_target_tolerance and
                    yaw_delta <= self.route_manifest_yaw_tolerance):
                return True
        return False

    def _targets_match(self, actual, expected):
        position_delta = self._distance3(actual[:3], expected[:3])
        yaw_delta = abs(self._angle_error(actual[3], expected[3]))
        return (position_delta <= self.route_manifest_target_tolerance and
                yaw_delta <= self.route_manifest_yaw_tolerance)

    def _raw_setpoints_follow_manifest_route(self, manifest_route, raw_setpoints=None):
        source = self.unique_raw_setpoints if raw_setpoints is None else raw_setpoints
        raw_targets = []
        for raw_target in source:
            actual = self._target_values(raw_target)
            if actual is not None:
                raw_targets.append(actual)
        raw_index = 0
        for item in manifest_route:
            expected = self._record_target(item)
            if expected is None:
                continue
            found = False
            while raw_index < len(raw_targets):
                if self._targets_match(raw_targets[raw_index], expected):
                    raw_index += 1
                    found = True
                    break
                raw_index += 1
            if not found:
                return False
        return True

    def _update_raw_setpoint_publishers(self):
        if not self.require_raw_setpoint_publisher:
            return
        now = time.monotonic()
        if now - self.raw_setpoint_last_publisher_check_wall < self.raw_setpoint_publisher_check_interval:
            return
        self.raw_setpoint_last_publisher_check_wall = now
        try:
            code, _message, state = rospy.get_master().getSystemState()
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "unable to inspect ROS graph for raw setpoint publisher: %s", exc)
            return
        if code != 1:
            return
        for topic, nodes in state[0]:
            if topic != self.raw_setpoint_topic:
                continue
            for node in sorted(nodes):
                if node not in self.raw_setpoint_publishers_seen:
                    self.raw_setpoint_publishers_seen.append(node)
            if self.raw_setpoint_expected_publisher_node in nodes:
                self.raw_setpoint_expected_publisher_seen = True
            return

    def _flight_status_cb(self, msg):
        fields = _parse_fields(msg.data)
        if not fields:
            return
        self.status_count += 1
        self.status_receive = time.monotonic()
        self.last_status = fields
        state = fields.get("state")
        if state and (not self.state_history or self.state_history[-1] != state):
            self.state_history.append(state)
        waypoint = fields.get("waypoint", "")
        if "/" in waypoint:
            current, total = waypoint.split("/", 1)
            try:
                self.current_waypoint_index = int(current)
                self.current_waypoint_total = int(total)
                self.max_waypoint_index = max(self.max_waypoint_index, self.current_waypoint_index)
                self.total_waypoints = self.current_waypoint_total
            except ValueError:
                pass
        if state == "ABORT":
            self.abort_reason = fields.get("error", "unknown")

    def _mission_active(self):
        return self.last_status.get("state") not in (None, "", "IDLE", "COMPLETE", "ABORT")

    def _active_control_window(self):
        if not self._mission_active() or self.mavros_state is None:
            return False
        return (bool(self.mavros_state.connected) and bool(self.mavros_state.armed) and
                str(self.mavros_state.mode).strip() == "OFFBOARD")

    def _append_target_record_if_needed(self, target):
        state = self.last_status.get("state", "unknown")
        if state not in ("TAKEOFF", "WAYPOINTS"):
            return
        waypoint_index = self.current_waypoint_index
        waypoint_total = self.current_waypoint_total
        if self.target_records:
            previous = self.target_records[-1]
            previous_target = previous.get("target")
            if (previous.get("state") == state and
                    previous.get("waypoint_index") == waypoint_index and
                    isinstance(previous_target, list) and len(previous_target) == 4 and
                    self._same_target(previous_target, target)):
                return
        self.target_records.append({
            "target": [target[0], target[1], target[2], target[3]],
            "state": state,
            "waypoint_index": waypoint_index,
            "waypoint_total": waypoint_total,
            "min_distance_m": None,
            "max_continuous_reach_s": 0.0,
            "_inside_since_wall": None,
            "reached": False,
        })
        if self.final_local_position is not None:
            self._update_target_hits(self.final_local_position)

    def _setpoint_preview_cb(self, msg):
        values = (msg.position.x, msg.position.y, msg.position.z, msg.yaw)
        if not _finite(values):
            return
        self.setpoint_count += 1
        self.setpoint_receive = time.monotonic()
        target = tuple(float(value) for value in values)
        self._append_unique(self.unique_setpoints, target)
        self._append_target_record_if_needed(target)

    def _raw_setpoint_cb(self, msg):
        values = (msg.position.x, msg.position.y, msg.position.z, msg.yaw)
        if not _finite(values):
            return
        self._update_raw_setpoint_publishers()
        self.raw_setpoint_count += 1
        self.raw_setpoint_receive = time.monotonic()
        target = tuple(float(value) for value in values)
        self._append_unique(self.unique_raw_setpoints, target)
        if self._active_control_window():
            self.active_raw_setpoint_count += 1
            self.active_raw_setpoint_receive = self.raw_setpoint_receive
            self._append_unique(self.active_unique_raw_setpoints, target)
        if msg.coordinate_frame != PositionTarget.FRAME_LOCAL_NED:
            self.raw_setpoint_frame_mismatch_count += 1

    def _update_target_hits(self, position, sample_time=None):
        for record in self.target_records:
            distance = self._distance3(position, record["target"])
            previous = record.get("min_distance_m")
            if previous is None or distance < previous:
                record["min_distance_m"] = distance
            if distance <= self.waypoint_reach_tolerance:
                record["reached"] = True
                if sample_time is not None:
                    inside_since = record.get("_inside_since_wall")
                    if inside_since is None:
                        record["_inside_since_wall"] = sample_time
                        inside_since = sample_time
                    continuous = max(0.0, sample_time - float(inside_since))
                    record["max_continuous_reach_s"] = max(
                        float(record.get("max_continuous_reach_s") or 0.0), continuous)
            else:
                record["_inside_since_wall"] = None

    def _local_position_cb(self, msg):
        values = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
        if not _finite(values):
            return
        self.local_count += 1
        self.local_receive = time.monotonic()
        self.final_local_position = tuple(float(value) for value in values)
        self.latest_local_position = self.final_local_position
        quaternion = msg.pose.pose.orientation
        self.final_local_yaw = (_yaw_from_quaternion(quaternion)
                                if _finite((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
                                else None)
        z = self.final_local_position[2]
        if self.initial_local_position is None:
            self.initial_local_position = self.final_local_position
            self.initial_local_yaw = self.final_local_yaw
        if self.initial_local_z is None:
            self.initial_local_z = z
        self.max_local_z = z if self.max_local_z is None else max(self.max_local_z, z)
        relative_z = z - self.initial_local_z
        self.max_relative_local_z = (relative_z if self.max_relative_local_z is None
                                     else max(self.max_relative_local_z, relative_z))
        self._update_target_hits(self.final_local_position, self.local_receive)
        self._update_vision_local_consistency()

    @staticmethod
    def _delta3(current, origin):
        return tuple(float(current[index]) - float(origin[index]) for index in range(3))

    def _update_vision_local_consistency(self):
        if not self._mission_active():
            return
        if self.latest_local_position is None or self.latest_vision_position is None:
            return
        if self.local_receive is None or self.latest_vision_receive is None:
            return
        if abs(self.local_receive - self.latest_vision_receive) > self.vision_local_pair_timeout:
            return
        pair_key = (self.local_receive, self.latest_vision_receive)
        if pair_key == self.active_vision_local_last_pair:
            return
        self.active_vision_local_last_pair = pair_key
        if self.vision_local_origin_local is None:
            self.vision_local_origin_local = self.latest_local_position
            self.vision_local_origin_vision = self.latest_vision_position
        local_delta = self._delta3(self.latest_local_position, self.vision_local_origin_local)
        vision_delta = self._delta3(self.latest_vision_position, self.vision_local_origin_vision)
        delta_error = self._distance3(local_delta, vision_delta)
        local_motion = self._distance3(local_delta, (0.0, 0.0, 0.0))
        vision_motion = self._distance3(vision_delta, (0.0, 0.0, 0.0))
        motion = max(local_motion, vision_motion)
        self.active_vision_local_pair_count += 1
        self.active_vision_local_sum_sq_error += delta_error * delta_error
        self.active_vision_local_max_delta_error_m = (
            delta_error if self.active_vision_local_max_delta_error_m is None
            else max(self.active_vision_local_max_delta_error_m, delta_error))
        self.active_vision_local_max_motion_m = (
            motion if self.active_vision_local_max_motion_m is None
            else max(self.active_vision_local_max_motion_m, motion))
        self.active_vision_local_last_local_delta = local_delta
        self.active_vision_local_last_vision_delta = vision_delta

    def _public_target_records(self):
        public_records = []
        for record in self.target_records:
            public_records.append({
                key: value for key, value in record.items()
                if not str(key).startswith("_")
            })
        return public_records

    def _mavros_state_cb(self, msg):
        self.mavros_state = msg
        self.mavros_state_receive = time.monotonic()
        if self._mission_active():
            self.active_mavros_state_count += 1
            if bool(msg.connected):
                self.active_mavros_connected_seen = True
            if bool(msg.armed):
                self.active_mavros_armed_seen = True
            mode = str(msg.mode).strip()
            if mode and (not self.active_mavros_modes or self.active_mavros_modes[-1] != mode):
                self.active_mavros_modes.append(mode)
                if len(self.active_mavros_modes) > 20:
                    self.active_mavros_modes = self.active_mavros_modes[-20:]
            if mode == "OFFBOARD":
                self.active_mavros_offboard_seen = True

    def _extended_state_cb(self, msg):
        self.extended_state = msg
        self.extended_state_receive = time.monotonic()
        if self._mission_active():
            self.active_extended_state_count += 1
            landed_state = int(msg.landed_state)
            if (not self.active_landed_states or
                    self.active_landed_states[-1] != landed_state):
                self.active_landed_states.append(landed_state)
                if len(self.active_landed_states) > 20:
                    self.active_landed_states = self.active_landed_states[-20:]
            if landed_state == ExtendedState.LANDED_STATE_IN_AIR:
                self.active_in_air_seen = True
            if landed_state == ExtendedState.LANDED_STATE_LANDING:
                self.active_landing_seen = True

    def _payload_status_cb(self, msg):
        fields = _parse_fields(msg.data)
        if fields.get("state", "").lower() == "open" and fields.get("success", "").lower() == "true":
            self.payload_open_seen = True
            self.payload_status_receive = time.monotonic()

    def _route_manifest_cb(self, msg):
        try:
            manifest = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if not isinstance(manifest, dict):
            return
        self.route_manifest = manifest
        self.route_manifest_receive = time.monotonic()
        self.route_manifest_history.append(manifest)
        if len(self.route_manifest_history) > 20:
            self.route_manifest_history = self.route_manifest_history[-20:]

    def _accept_vision_pose(self, header, pose):
        if not self._mission_active():
            return
        values = (pose.position.x, pose.position.y, pose.position.z,
                  pose.orientation.x, pose.orientation.y,
                  pose.orientation.z, pose.orientation.w)
        if not _finite(values):
            return
        if self.expected_vision_parent and header.frame_id != self.expected_vision_parent:
            return
        self.active_vision_pose_count += 1
        self.active_vision_pose_receive = time.monotonic()
        self.latest_vision_position = tuple(float(value) for value in (
            pose.position.x, pose.position.y, pose.position.z))
        self.latest_vision_receive = self.active_vision_pose_receive
        stamp = header.stamp.to_sec()
        self.active_vision_pose_parent = header.frame_id
        if stamp > 0.0:
            if self.active_vision_pose_first_stamp is None:
                self.active_vision_pose_first_stamp = stamp
            self.active_vision_pose_last_stamp = stamp
        self._update_vision_local_consistency()

    def _vision_pose_cb(self, msg):
        self._accept_vision_pose(msg.header, msg.pose)

    def _vision_pose_cov_cb(self, msg):
        self._accept_vision_pose(msg.header, msg.pose.pose)

    def _vision_output_enabled_cb(self, msg):
        self.vision_output_enabled_latest = bool(msg.data)
        self.vision_output_enabled_receive = time.monotonic()
        if self._mission_active() and self.vision_output_enabled_latest:
            self.active_vision_output_enabled_seen = True
            self.active_vision_output_enabled_receive = time.monotonic()

    def _vision_status_cb(self, msg):
        status = str(msg.data).strip()
        self.last_fastlio_vision_status = status
        if self._mission_active() and status.startswith("ok"):
            self.active_fastlio_vision_status_ok_seen = True
            self.active_fastlio_vision_status_receive = time.monotonic()

    def _target_reach_issue(self):
        flight_targets = [record for record in self.target_records
                          if record.get("state") in ("TAKEOFF", "WAYPOINTS")]
        if not flight_targets:
            return "target_records_missing"
        if not any(record.get("state") == "TAKEOFF" for record in flight_targets):
            return "takeoff_target_record_missing"
        if self.require_waypoints_complete and self.total_waypoints is not None:
            reached_indices = set()
            for record in flight_targets:
                if record.get("state") != "WAYPOINTS" or not self._target_reached(record):
                    continue
                try:
                    reached_indices.add(int(record.get("waypoint_index")))
                except (TypeError, ValueError):
                    pass
            missing_indices = [index for index in range(self.total_waypoints)
                               if index not in reached_indices]
            if missing_indices:
                return "waypoint_target_records_missing:%s" % ",".join(
                    str(index) for index in missing_indices)
        unreached = []
        for index, record in enumerate(flight_targets):
            distance = record.get("min_distance_m")
            if not self._target_reached(record):
                try:
                    distance_text = "%.3f" % float(distance)
                except (TypeError, ValueError):
                    distance_text = "unknown"
                unreached.append("%d:%s" % (
                    index, distance_text))
        if unreached:
            return "target_records_unreached:%s" % ";".join(unreached)
        return None

    def _route_manifest_issue(self):
        if not self.require_route_manifest:
            return None
        if not isinstance(self.route_manifest, dict):
            return "route_manifest_missing"
        if self.route_manifest.get("event") != "mission_started":
            return "route_manifest_not_started"
        if self.route_manifest.get("route_source") not in ("configured", "posearray"):
            return "route_manifest_source_invalid"
        if not self.route_manifest.get("route_fingerprint"):
            return "route_manifest_fingerprint_missing"
        status_revision = self.last_status.get("route_revision")
        if status_revision is None:
            return "route_status_revision_missing"
        if str(status_revision) != str(self.route_manifest.get("route_revision")):
            return "route_status_revision_mismatch"
        status_fingerprint = self.last_status.get("route_fingerprint")
        if not status_fingerprint:
            return "route_status_fingerprint_missing"
        if str(status_fingerprint) != str(self.route_manifest.get("route_fingerprint")):
            return "route_status_fingerprint_mismatch"
        if not isinstance(self.route_manifest.get("origin"), list):
            return "route_manifest_origin_missing"
        if self.route_manifest.get("origin_yaw") is None:
            return "route_manifest_origin_yaw_missing"
        target_route = self.route_manifest.get("target_route")
        if not isinstance(target_route, list) or not target_route:
            return "route_manifest_target_route_missing"
        waypoint_count = self.route_manifest.get("waypoint_count")
        try:
            waypoint_count_int = int(waypoint_count)
        except (TypeError, ValueError):
            return "route_manifest_waypoint_count_invalid"
        if self.total_waypoints is not None and waypoint_count_int != self.total_waypoints:
            return "route_manifest_waypoint_count_mismatch:%d/%d" % (
                waypoint_count_int, self.total_waypoints)
        waypoint_targets = [item for item in target_route
                            if isinstance(item, dict) and item.get("state") == "WAYPOINTS"]
        if len(waypoint_targets) != waypoint_count_int:
            return "route_manifest_target_count_mismatch:%d/%d" % (
                len(waypoint_targets), waypoint_count_int)
        manifest_targets = self._target_map(target_route)
        observed_targets = self._target_map(self.target_records)
        expected_target_count = waypoint_count_int + 1
        if len(manifest_targets) != expected_target_count:
            return "route_manifest_target_count_mismatch:%d/%d" % (
                len(manifest_targets), expected_target_count)
        if (self.require_raw_setpoints and
                not self._raw_setpoints_follow_manifest_route(target_route)):
            return "route_manifest_raw_setpoint_order_mismatch"
        if (self.require_raw_setpoints and
                not self._raw_setpoints_follow_manifest_route(target_route, self.active_unique_raw_setpoints)):
            return "route_manifest_active_raw_setpoint_order_mismatch"
        for key, expected in manifest_targets.items():
            actual = observed_targets.get(key)
            if actual is None:
                return "route_manifest_observed_target_missing:%s%d" % (key[0].lower(), key[1])
            position_delta = self._distance3(actual[:3], expected[:3])
            yaw_delta = abs(self._angle_error(actual[3], expected[3]))
            if (position_delta > self.route_manifest_target_tolerance or
                    yaw_delta > self.route_manifest_yaw_tolerance):
                return "route_manifest_observed_target_mismatch:%s%d:pos=%.3f:yaw_deg=%.2f" % (
                    key[0].lower(), key[1], position_delta, math.degrees(yaw_delta))
            if self.require_raw_setpoints and not self._target_present_in_raw_setpoints(expected):
                return "route_manifest_raw_setpoint_missing:%s%d" % (key[0].lower(), key[1])
            if (self.require_raw_setpoints and
                    not self._target_present_in_raw_setpoints(expected, self.active_unique_raw_setpoints)):
                return "route_manifest_active_raw_setpoint_missing:%s%d" % (key[0].lower(), key[1])
        return None

    def _failure_reason(self, final=False):
        if self.abort_reason:
            return "flight_aborted:%s" % self.abort_reason
        if self.status_count < 1 or (not final and not self._fresh(self.status_receive, self.stream_timeout)):
            return "flight_status_missing_or_stale"
        if self.setpoint_count < self.min_setpoints:
            return "setpoint_preview_missing_or_stale"
        if not final and not self._fresh(self.setpoint_receive, self.stream_timeout):
            return "setpoint_preview_missing_or_stale"
        if self.require_raw_setpoints:
            if self.raw_setpoint_count < self.min_raw_setpoints:
                return "raw_setpoint_count_below_%d" % self.min_raw_setpoints
            if self.active_raw_setpoint_count < self.min_active_raw_setpoints:
                return "active_raw_setpoint_count_below_%d" % self.min_active_raw_setpoints
            if len(self.active_unique_raw_setpoints) < self.min_active_unique_raw_setpoints:
                return "active_unique_raw_setpoints_below_%d" % self.min_active_unique_raw_setpoints
            if self.raw_setpoint_frame_mismatch_count > 0:
                return "raw_setpoint_frame_mismatch"
            if (self.require_raw_setpoint_publisher and
                    not self.raw_setpoint_expected_publisher_seen):
                return "raw_setpoint_expected_publisher_missing"
            if not final and not self._fresh(self.raw_setpoint_receive, self.stream_timeout):
                return "raw_setpoint_missing_or_stale"
        if self.local_count < 1:
            return "local_position_missing_or_stale"
        if not final and not self._fresh(self.local_receive, self.stream_timeout):
            return "local_position_missing_or_stale"
        if (self.max_relative_local_z is None or
                self.max_relative_local_z < self.min_airborne_altitude):
            return "airborne_altitude_not_observed"
        if self.require_takeoff_landing_states:
            if "TAKEOFF" not in self.state_history:
                return "takeoff_state_missing"
            if "LANDING" not in self.state_history:
                return "landing_state_missing"
        if self.require_waypoints_complete:
            if self.total_waypoints is None:
                return "waypoint_progress_unavailable"
            if self.max_waypoint_index < self.total_waypoints:
                return "waypoints_incomplete:%d/%d" % (self.max_waypoint_index, self.total_waypoints)
        route_manifest_issue = self._route_manifest_issue()
        if route_manifest_issue is not None:
            return route_manifest_issue
        if self.require_active_vision_pose:
            if self.active_vision_pose_count < self.min_active_vision_pose_count:
                return "active_vision_pose_count_below_%d" % self.min_active_vision_pose_count
            if not self.active_vision_output_enabled_seen:
                return "active_vision_output_enabled_missing"
            if not self.active_fastlio_vision_status_ok_seen:
                return "active_fastlio_vision_status_ok_missing"
            if (not final and
                    not self._fresh(self.active_vision_pose_receive, self.stream_timeout)):
                return "active_vision_pose_missing_or_stale"
            if self.require_active_vision_local_consistency:
                if self.active_vision_local_pair_count < self.min_active_vision_local_pairs:
                    return "active_vision_local_pairs_below_%d" % self.min_active_vision_local_pairs
                if (self.active_vision_local_max_delta_error_m is None or
                        self.active_vision_local_max_delta_error_m > self.max_active_vision_local_delta_m):
                    return "active_vision_local_delta_error:%.3f" % (
                        -1.0 if self.active_vision_local_max_delta_error_m is None
                        else self.active_vision_local_max_delta_error_m)
        if self.require_active_mavros_control:
            if self.active_mavros_state_count < 1:
                return "active_mavros_state_missing"
            if not self.active_mavros_connected_seen:
                return "active_mavros_connected_missing"
            if not self.active_mavros_armed_seen:
                return "active_mavros_armed_missing"
            if not self.active_mavros_offboard_seen:
                return "active_mavros_offboard_missing"
        target_issue = self._target_reach_issue()
        if target_issue is not None:
            return target_issue
        if self.require_final_disarmed:
            if (not self._fresh(self.mavros_state_receive, self.stream_timeout) or
                    self.mavros_state is None or self.mavros_state.armed):
                return "final_disarmed_not_confirmed"
        if self.require_final_on_ground:
            if (not self._fresh(self.extended_state_receive, self.stream_timeout) or
                    self.extended_state is None or
                    self.extended_state.landed_state != ExtendedState.LANDED_STATE_ON_GROUND):
                return "final_on_ground_not_confirmed"
        if self.require_payload_open and not self.payload_open_seen:
            return "payload_open_not_observed"
        return None

    def _summary(self):
        return {
            "status_count": self.status_count,
            "last_status": self.last_status,
            "state_history": self.state_history,
            "abort_reason": self.abort_reason,
            "max_waypoint_index": self.max_waypoint_index,
            "total_waypoints": self.total_waypoints,
            "current_waypoint_index": self.current_waypoint_index,
            "current_waypoint_total": self.current_waypoint_total,
            "setpoint_count": self.setpoint_count,
            "unique_setpoints": self.unique_setpoints,
            "raw_setpoint_count": self.raw_setpoint_count,
            "unique_raw_setpoints": self.unique_raw_setpoints,
            "active_raw_setpoint_count": self.active_raw_setpoint_count,
            "active_unique_raw_setpoints": self.active_unique_raw_setpoints,
            "raw_setpoint_frame_mismatch_count": self.raw_setpoint_frame_mismatch_count,
            "raw_setpoint_expected_publisher_seen": self.raw_setpoint_expected_publisher_seen,
            "raw_setpoint_publishers_seen": self.raw_setpoint_publishers_seen,
            "target_records": self._public_target_records(),
            "route_manifest": self.route_manifest,
            "route_manifest_history": self.route_manifest_history,
            "active_vision_pose_count": self.active_vision_pose_count,
            "active_vision_pose_first_stamp": self.active_vision_pose_first_stamp,
            "active_vision_pose_last_stamp": self.active_vision_pose_last_stamp,
            "active_vision_pose_parent": self.active_vision_pose_parent,
            "active_vision_local_pair_count": self.active_vision_local_pair_count,
            "active_vision_local_max_delta_error_m": self.active_vision_local_max_delta_error_m,
            "active_vision_local_rms_delta_error_m": (
                None if self.active_vision_local_pair_count < 1 else
                math.sqrt(self.active_vision_local_sum_sq_error / self.active_vision_local_pair_count)),
            "active_vision_local_max_motion_m": self.active_vision_local_max_motion_m,
            "active_vision_local_last_local_delta": self.active_vision_local_last_local_delta,
            "active_vision_local_last_vision_delta": self.active_vision_local_last_vision_delta,
            "vision_output_enabled_latest": self.vision_output_enabled_latest,
            "active_vision_output_enabled_seen": self.active_vision_output_enabled_seen,
            "active_fastlio_vision_status_ok_seen": self.active_fastlio_vision_status_ok_seen,
            "last_fastlio_vision_status": self.last_fastlio_vision_status,
            "local_count": self.local_count,
            "initial_local_position": self.initial_local_position,
            "initial_local_yaw": self.initial_local_yaw,
            "initial_local_z": self.initial_local_z,
            "max_local_z": self.max_local_z,
            "max_relative_local_z": self.max_relative_local_z,
            "final_local_position": self.final_local_position,
            "final_local_yaw": self.final_local_yaw,
            "active_mavros_state_count": self.active_mavros_state_count,
            "active_mavros_connected_seen": self.active_mavros_connected_seen,
            "active_mavros_armed_seen": self.active_mavros_armed_seen,
            "active_mavros_offboard_seen": self.active_mavros_offboard_seen,
            "active_mavros_modes": self.active_mavros_modes,
            "active_extended_state_count": self.active_extended_state_count,
            "active_landed_states": self.active_landed_states,
            "active_in_air_seen": self.active_in_air_seen,
            "active_landing_seen": self.active_landing_seen,
            "final_armed": None if self.mavros_state is None else bool(self.mavros_state.armed),
            "final_mode": None if self.mavros_state is None else self.mavros_state.mode,
            "final_landed_state": None if self.extended_state is None else self.extended_state.landed_state,
            "payload_open_seen": self.payload_open_seen,
            "parameters": {
                "observe_timeout": self.observe_timeout,
                "stream_timeout": self.stream_timeout,
                "min_setpoints": self.min_setpoints,
                "require_raw_setpoints": self.require_raw_setpoints,
                "min_raw_setpoints": self.min_raw_setpoints,
                "min_active_raw_setpoints": self.min_active_raw_setpoints,
                "min_active_unique_raw_setpoints": self.min_active_unique_raw_setpoints,
                "require_raw_setpoint_publisher": self.require_raw_setpoint_publisher,
                "raw_setpoint_expected_publisher_node": self.raw_setpoint_expected_publisher_node,
                "raw_setpoint_publisher_check_interval": self.raw_setpoint_publisher_check_interval,
                "min_airborne_altitude": self.min_airborne_altitude,
                "waypoint_reach_tolerance": self.waypoint_reach_tolerance,
                "min_target_dwell_s": self.min_target_dwell_s,
                "require_active_vision_pose": self.require_active_vision_pose,
                "min_active_vision_pose_count": self.min_active_vision_pose_count,
                "expected_vision_parent": self.expected_vision_parent,
                "require_active_vision_local_consistency": self.require_active_vision_local_consistency,
                "min_active_vision_local_pairs": self.min_active_vision_local_pairs,
                "max_active_vision_local_delta_m": self.max_active_vision_local_delta_m,
                "vision_local_pair_timeout": self.vision_local_pair_timeout,
                "require_active_mavros_control": self.require_active_mavros_control,
                "require_takeoff_landing_states": self.require_takeoff_landing_states,
                "require_waypoints_complete": self.require_waypoints_complete,
                "require_route_manifest": self.require_route_manifest,
                "route_manifest_target_tolerance": self.route_manifest_target_tolerance,
                "route_manifest_yaw_tolerance_deg": math.degrees(self.route_manifest_yaw_tolerance),
                "require_final_disarmed": self.require_final_disarmed,
                "require_final_on_ground": self.require_final_on_ground,
                "require_payload_open": self.require_payload_open,
            },
        }

    def _write_evidence(self, success, reason):
        if not self.evidence_file:
            return
        path = pathlib.Path(self.evidence_file).expanduser()
        if path.parent:
            path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "observer": "active_flight_observer",
            "success": bool(success),
            "reason": reason,
            "generated_at_wall_time": time.time(),
            "summary": self._summary(),
        }
        path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")

    def _finish(self, success, reason):
        if self.finished:
            return
        self.finished = True
        self.finish_reason = reason
        try:
            self._write_evidence(success, reason)
        except Exception as exc:
            success = False
            reason = "evidence_write_failed:%s" % exc
            rospy.logerr("failed to write active-flight evidence: %s", exc)
        self.exit_code = 0 if success else 2
        level = rospy.loginfo if success else rospy.logerr
        level("%s: %s summary=%s", "PASS" if success else "FAIL", reason, self._summary())
        rospy.signal_shutdown(reason)

    def _tick(self, _event):
        state = self.last_status.get("state")
        if state == "ABORT":
            self._finish(False, self._failure_reason() or "flight_aborted")
            return
        if state == "COMPLETE":
            issue = self._failure_reason(final=True)
            if issue is None:
                self._finish(True, "active_local_flight_passed")
            return
        if time.monotonic() - self.started > self.observe_timeout:
            self._finish(False, "observe_timeout:%s" % (self._failure_reason() or "mission_not_complete"))


if __name__ == "__main__":
    rospy.init_node("active_flight_observer", anonymous=True)
    try:
        observer = ActiveFlightObserver()
        rospy.loginfo("Read-only active-flight observer started")
        rospy.spin()
        sys.exit(observer.exit_code)
    except Exception as exc:
        rospy.logerr("FAIL: active-flight observer initialization: %s", exc)
        sys.exit(3)
