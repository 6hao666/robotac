#!/usr/bin/env python3
"""Read-only observer for a real Robotac local waypoint flight.

This node is an evidence recorder, not a controller. It only subscribes to the
flight controller, MAVROS state, MAVROS local position, and optional servo
status topics. It never publishes setpoints, calls services, changes modes,
arms, lands, or writes PX4 parameters.
"""

import json
import math
import pathlib
import sys
import time

import rospy
from mavros_msgs.msg import ExtendedState, PositionTarget, State
from nav_msgs.msg import Odometry
from std_msgs.msg import String


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
        self.min_airborne_altitude = float(rospy.get_param("~min_airborne_altitude", 0.50))
        self.waypoint_reach_tolerance = float(rospy.get_param("~waypoint_reach_tolerance", 0.35))
        self.min_target_dwell_s = float(rospy.get_param("~min_target_dwell_s", 0.25))
        self.require_waypoints_complete = _as_bool(rospy.get_param("~require_waypoints_complete", True))
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
        self.target_records = []

        self.local_count = 0
        self.local_receive = None
        self.initial_local_position = None
        self.initial_local_yaw = None
        self.initial_local_z = None
        self.max_local_z = None
        self.max_relative_local_z = None
        self.final_local_position = None
        self.final_local_yaw = None

        self.mavros_state = None
        self.mavros_state_receive = None
        self.extended_state = None
        self.extended_state_receive = None

        self.payload_open_seen = False
        self.payload_status_receive = None

        rospy.Subscriber(rospy.get_param("~flight_status_topic", "/robotac/flight/status"),
                         String, self._flight_status_cb, queue_size=20)
        rospy.Subscriber(rospy.get_param("~setpoint_preview_topic", "/robotac/flight/setpoint_preview"),
                         PositionTarget, self._setpoint_preview_cb, queue_size=50)
        rospy.Subscriber(rospy.get_param("~local_position_topic", "/mavros/local_position/odom"),
                         Odometry, self._local_position_cb, queue_size=50)
        rospy.Subscriber(rospy.get_param("~mavros_state_topic", "/mavros/state"),
                         State, self._mavros_state_cb, queue_size=20)
        rospy.Subscriber(rospy.get_param("~extended_state_topic", "/mavros/extended_state"),
                         ExtendedState, self._extended_state_cb, queue_size=20)
        rospy.Subscriber(rospy.get_param("~payload_status_topic", "/robotac/servo/status"),
                         String, self._payload_status_cb, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.20), self._tick)

    def _validate_parameters(self):
        if not math.isfinite(self.observe_timeout) or self.observe_timeout <= 0.0:
            raise ValueError("observe_timeout must be finite and positive")
        if not math.isfinite(self.stream_timeout) or self.stream_timeout <= 0.0:
            raise ValueError("stream_timeout must be finite and positive")
        if self.min_setpoints < 1:
            raise ValueError("min_setpoints must be positive")
        if not math.isfinite(self.min_airborne_altitude) or self.min_airborne_altitude < 0.0:
            raise ValueError("min_airborne_altitude must be finite and non-negative")
        if not math.isfinite(self.waypoint_reach_tolerance) or self.waypoint_reach_tolerance <= 0.0:
            raise ValueError("waypoint_reach_tolerance must be finite and positive")
        if not math.isfinite(self.min_target_dwell_s) or self.min_target_dwell_s < 0.0:
            raise ValueError("min_target_dwell_s must be finite and non-negative")

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

    def _extended_state_cb(self, msg):
        self.extended_state = msg
        self.extended_state_receive = time.monotonic()

    def _payload_status_cb(self, msg):
        fields = _parse_fields(msg.data)
        if fields.get("state", "").lower() == "open" and fields.get("success", "").lower() == "true":
            self.payload_open_seen = True
            self.payload_status_receive = time.monotonic()

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

    def _failure_reason(self, final=False):
        if self.abort_reason:
            return "flight_aborted:%s" % self.abort_reason
        if self.status_count < 1 or (not final and not self._fresh(self.status_receive, self.stream_timeout)):
            return "flight_status_missing_or_stale"
        if self.setpoint_count < self.min_setpoints:
            return "setpoint_preview_missing_or_stale"
        if not final and not self._fresh(self.setpoint_receive, self.stream_timeout):
            return "setpoint_preview_missing_or_stale"
        if self.local_count < 1:
            return "local_position_missing_or_stale"
        if not final and not self._fresh(self.local_receive, self.stream_timeout):
            return "local_position_missing_or_stale"
        if (self.max_relative_local_z is None or
                self.max_relative_local_z < self.min_airborne_altitude):
            return "airborne_altitude_not_observed"
        if self.require_waypoints_complete:
            if self.total_waypoints is None:
                return "waypoint_progress_unavailable"
            if self.max_waypoint_index < self.total_waypoints:
                return "waypoints_incomplete:%d/%d" % (self.max_waypoint_index, self.total_waypoints)
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
            "target_records": self._public_target_records(),
            "local_count": self.local_count,
            "initial_local_position": self.initial_local_position,
            "initial_local_yaw": self.initial_local_yaw,
            "initial_local_z": self.initial_local_z,
            "max_local_z": self.max_local_z,
            "max_relative_local_z": self.max_relative_local_z,
            "final_local_position": self.final_local_position,
            "final_local_yaw": self.final_local_yaw,
            "final_armed": None if self.mavros_state is None else bool(self.mavros_state.armed),
            "final_mode": None if self.mavros_state is None else self.mavros_state.mode,
            "final_landed_state": None if self.extended_state is None else self.extended_state.landed_state,
            "payload_open_seen": self.payload_open_seen,
            "parameters": {
                "observe_timeout": self.observe_timeout,
                "stream_timeout": self.stream_timeout,
                "min_setpoints": self.min_setpoints,
                "min_airborne_altitude": self.min_airborne_altitude,
                "waypoint_reach_tolerance": self.waypoint_reach_tolerance,
                "min_target_dwell_s": self.min_target_dwell_s,
                "require_waypoints_complete": self.require_waypoints_complete,
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
