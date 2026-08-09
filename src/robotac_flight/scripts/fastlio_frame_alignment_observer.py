#!/usr/bin/env python3
"""Read-only FAST-LIO preview frame-alignment observer.

This node subscribes to ``/robotac/fastlio_vision/pose_preview`` and optional
bridge health topics only. It never publishes, calls services, changes PX4
state, arms, switches modes, sends MAVROS setpoints, or writes deployment
configuration. Use it on the ground before enabling MAVROS vision output: move
the disarmed aircraft by a known translation or yaw direction and keep the JSON
evidence with the deployment notes before manually approving frame alignment.
"""

import json
import math
import pathlib
import sys
import time

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool, String
from tf.transformations import euler_from_quaternion


def _as_bool(value):
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in ("true", "1", "yes", "on"):
            return True
        if normalized in ("false", "0", "no", "off", ""):
            return False
        raise ValueError("invalid Boolean value: %s" % value)
    return bool(value)


def _finite_pose(pose):
    values = (
        pose.position.x, pose.position.y, pose.position.z,
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
    )
    if not all(math.isfinite(float(value)) for value in values):
        return False
    norm = math.sqrt(sum(float(value) * float(value) for value in values[3:]))
    return norm >= 1.0e-6


def _yaw_from_pose(pose):
    q = pose.orientation
    _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
    return yaw


def _angle_error(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def _norm3(vector):
    return math.sqrt(sum(float(item) * float(item) for item in vector))


class PoseStream(object):
    def __init__(self, window_samples):
        self.window_samples = window_samples
        self.samples = []
        self.first_stamp = None
        self.last_stamp = None
        self.last_receive = None
        self.last_issue = "waiting"
        self.rejects = 0

    def reject(self, reason):
        self.rejects += 1
        self.samples = []
        self.first_stamp = None
        self.last_stamp = None
        self.last_receive = None
        self.last_issue = reason

    def accept(self, stamp, pose):
        if self.last_stamp is not None and stamp <= self.last_stamp:
            self.reject("non_monotonic_timestamp")
            return False
        sample = (
            stamp,
            (float(pose.position.x), float(pose.position.y), float(pose.position.z)),
            _yaw_from_pose(pose),
        )
        if self.first_stamp is None:
            self.first_stamp = stamp
        self.samples.append(sample)
        if len(self.samples) > self.window_samples:
            del self.samples[:-self.window_samples]
        self.last_stamp = stamp
        self.last_receive = time.monotonic()
        self.last_issue = "ok"
        return True

    def count(self):
        return len(self.samples)

    def fresh(self, timeout):
        return self.last_receive is not None and time.monotonic() - self.last_receive <= timeout

    def rate_hz(self):
        if len(self.samples) < 2:
            return 0.0
        duration = (self.samples[-1][0] - self.samples[0][0]).to_sec()
        return 0.0 if duration <= 0.0 else float(len(self.samples) - 1) / duration

    def delta(self):
        if len(self.samples) < 2:
            return None
        first = self.samples[0]
        last = self.samples[-1]
        return (
            last[1][0] - first[1][0],
            last[1][1] - first[1][1],
            last[1][2] - first[1][2],
            _angle_error(last[2], first[2]),
        )


class FastlioFrameAlignmentObserver(object):
    def __init__(self):
        self.observe_seconds = float(rospy.get_param("~observe_seconds", 20.0))
        self.startup_timeout = float(rospy.get_param("~startup_timeout", 30.0))
        self.stream_timeout = float(rospy.get_param("~stream_timeout", 0.75))
        self.age_limit = float(rospy.get_param("~age_limit", 0.50))
        self.future_tolerance = float(rospy.get_param("~future_tolerance", 0.10))
        self.min_samples = int(rospy.get_param("~min_samples", 20))
        self.min_rate_hz = float(rospy.get_param("~min_rate_hz", 5.0))
        self.motion_type = str(rospy.get_param("~motion_type", "translation")).strip().lower()
        self.motion_name = str(rospy.get_param("~motion_name", "manual_axis_check")).strip()
        self.expected_x = float(rospy.get_param("~expected_x", 1.0))
        self.expected_y = float(rospy.get_param("~expected_y", 0.0))
        self.expected_z = float(rospy.get_param("~expected_z", 0.0))
        self.expected_distance_m = float(rospy.get_param("~expected_distance_m", 0.0))
        self.min_translation_m = float(rospy.get_param("~min_translation_m", 0.30))
        self.max_direction_error_deg = float(rospy.get_param("~max_direction_error_deg", 25.0))
        self.min_delta_scale = float(rospy.get_param("~min_delta_scale", 0.50))
        self.max_delta_scale = float(rospy.get_param("~max_delta_scale", 2.00))
        self.expected_yaw_deg = float(rospy.get_param("~expected_yaw_deg", 0.0))
        self.expected_yaw_sign = int(rospy.get_param("~expected_yaw_sign", 0))
        self.min_yaw_deg = float(rospy.get_param("~min_yaw_deg", 20.0))
        self.max_yaw_error_deg = float(rospy.get_param("~max_yaw_error_deg", 25.0))
        self.require_vision_status_ok = _as_bool(
            rospy.get_param("~require_vision_status_ok", True))
        self.require_mavros_output_disabled = _as_bool(
            rospy.get_param("~require_mavros_output_disabled", True))
        self.expected_parent = str(rospy.get_param("~expected_parent", "odom")).strip()
        self.pose_topic = rospy.get_param("~pose_topic", "/robotac/fastlio_vision/pose_preview")
        self.vision_status_topic = rospy.get_param(
            "~vision_status_topic", "/robotac/fastlio_vision/status")
        self.output_enabled_topic = rospy.get_param(
            "~vision_output_enabled_topic", "/robotac/fastlio_vision/output_enabled")
        self.evidence_file = str(rospy.get_param("~evidence_file", "")).strip()
        self._validate_parameters()

        window_samples = max(self.min_samples * 4, int(math.ceil(self.observe_seconds * self.min_rate_hz * 3.0)), 256)
        self.poses = PoseStream(window_samples)
        self.vision_status = "waiting"
        self.vision_status_receive = None
        self.output_enabled = None
        self.output_enabled_receive = None
        self.exit_code = 1
        self.started = time.monotonic()
        self.finished = False
        self.last_metrics = {}

        rospy.Subscriber(self.pose_topic, PoseWithCovarianceStamped, self._pose_cb, queue_size=20)
        if self.require_vision_status_ok:
            rospy.Subscriber(self.vision_status_topic, String, self._vision_status_cb, queue_size=10)
        if self.require_mavros_output_disabled:
            rospy.Subscriber(self.output_enabled_topic, Bool, self._output_enabled_cb, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.10), self._tick)

    def _validate_parameters(self):
        positive = (
            self.observe_seconds, self.startup_timeout, self.stream_timeout,
            self.age_limit, self.min_rate_hz,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError("observer timeouts and rates must be finite and positive")
        if self.min_samples < 2:
            raise ValueError("min_samples must be at least 2")
        if not math.isfinite(self.future_tolerance) or self.future_tolerance < 0.0:
            raise ValueError("future_tolerance must be finite and non-negative")
        if self.motion_type not in ("translation", "yaw"):
            raise ValueError("motion_type must be translation or yaw")
        if self.expected_yaw_sign not in (-1, 0, 1):
            raise ValueError("expected_yaw_sign must be -1, 0, or 1")
        if not self.motion_name:
            raise ValueError("motion_name must be non-empty")
        if not self.expected_parent:
            raise ValueError("expected_parent must be non-empty")
        if self.motion_type == "translation":
            expected = (self.expected_x, self.expected_y, self.expected_z)
            if not all(math.isfinite(value) for value in expected):
                raise ValueError("expected translation vector must be finite")
            if _norm3(expected) <= 1.0e-9:
                raise ValueError("expected translation vector must be non-zero")
            if (not math.isfinite(self.min_translation_m) or self.min_translation_m <= 0.0 or
                    not math.isfinite(self.max_direction_error_deg) or
                    self.max_direction_error_deg <= 0.0 or self.max_direction_error_deg >= 90.0):
                raise ValueError("translation thresholds must be finite and valid")
            if self.expected_distance_m < 0.0 or not math.isfinite(self.expected_distance_m):
                raise ValueError("expected_distance_m must be finite and non-negative")
            if self.min_delta_scale > self.max_delta_scale:
                raise ValueError("min_delta_scale must be <= max_delta_scale")
        if self.motion_type == "yaw":
            if (not math.isfinite(self.min_yaw_deg) or self.min_yaw_deg <= 0.0 or
                    not math.isfinite(self.max_yaw_error_deg) or self.max_yaw_error_deg <= 0.0):
                raise ValueError("yaw thresholds must be finite and positive")
            if not math.isfinite(self.expected_yaw_deg):
                raise ValueError("expected_yaw_deg must be finite")

    def _stamp_current(self, stamp):
        if stamp == rospy.Time(0):
            return False, "zero_timestamp"
        now = rospy.Time.now()
        if now == rospy.Time(0):
            return False, "ros_clock_unavailable"
        age = (now - stamp).to_sec()
        if age > self.age_limit or age < -self.future_tolerance:
            return False, "timestamp_age:%.3f" % age
        return True, "ok"

    def _pose_cb(self, msg):
        if msg.header.frame_id != self.expected_parent:
            self.poses.reject("unexpected_frame:%s" % msg.header.frame_id)
            return
        if not _finite_pose(msg.pose.pose):
            self.poses.reject("nonfinite_or_invalid_quaternion")
            return
        valid, reason = self._stamp_current(msg.header.stamp)
        if not valid:
            self.poses.reject(reason)
            return
        self.poses.accept(msg.header.stamp, msg.pose.pose)

    def _vision_status_cb(self, msg):
        self.vision_status = str(msg.data).strip() or "empty"
        self.vision_status_receive = time.monotonic()

    def _output_enabled_cb(self, msg):
        self.output_enabled = bool(msg.data)
        self.output_enabled_receive = time.monotonic()

    def _status_ok(self):
        if not self.require_vision_status_ok:
            return True, "not_required"
        if self.vision_status_receive is None:
            return False, "vision_status_waiting"
        if time.monotonic() - self.vision_status_receive > self.stream_timeout:
            return False, "vision_status_stale"
        if not (self.vision_status == "ok" or self.vision_status.startswith("ok ")):
            return False, "vision_status:%s" % self.vision_status
        return True, "ok"

    def _output_disabled_ok(self):
        if not self.require_mavros_output_disabled:
            return True, "not_required"
        if self.output_enabled_receive is None:
            return False, "output_enabled_waiting"
        if time.monotonic() - self.output_enabled_receive > self.stream_timeout:
            return False, "output_enabled_stale"
        if self.output_enabled is not False:
            return False, "mavros_output_enabled"
        return True, "ok"

    def _base_checks(self):
        missing = []
        notes = []
        if self.poses.count() < self.min_samples:
            missing.append("pose_samples:%d<%d" % (self.poses.count(), self.min_samples))
        if not self.poses.fresh(self.stream_timeout):
            missing.append("pose_stream_not_fresh:%s" % self.poses.last_issue)
        rate = self.poses.rate_hz()
        if rate < self.min_rate_hz:
            missing.append("pose_rate:%.3f<%.3f" % (rate, self.min_rate_hz))
        else:
            notes.append("pose_rate=%.3f" % rate)
        ok, reason = self._status_ok()
        if not ok:
            missing.append(reason)
        ok, reason = self._output_disabled_ok()
        if not ok:
            missing.append(reason)
        return missing, notes

    def _evaluate_translation(self, missing, notes):
        delta = self.poses.delta()
        if delta is None:
            missing.append("pose_delta_missing")
            return False, missing, notes
        observed = delta[:3]
        distance = _norm3(observed)
        expected = (self.expected_x, self.expected_y, self.expected_z)
        expected_norm = _norm3(expected)
        unit = tuple(value / expected_norm for value in expected)
        projection = sum(observed[i] * unit[i] for i in range(3))
        direction_cos = projection / distance if distance > 1.0e-9 else 0.0
        direction_cos = max(-1.0, min(1.0, direction_cos))
        direction_error_deg = math.degrees(math.acos(direction_cos))
        self.last_metrics = {
            "motion_type": "translation",
            "motion_name": self.motion_name,
            "observed_delta": list(observed),
            "observed_yaw_delta_deg": math.degrees(delta[3]),
            "translation_distance_m": distance,
            "projection_m": projection,
            "direction_cos": direction_cos,
            "direction_error_deg": direction_error_deg,
            "expected_unit": list(unit),
            "expected_distance_m": self.expected_distance_m,
            "sample_count": self.poses.count(),
            "pose_rate_hz": self.poses.rate_hz(),
        }
        if distance < self.min_translation_m:
            missing.append("translation_distance:%.3f<%.3f" % (distance, self.min_translation_m))
        if projection <= 0.0:
            missing.append("translation_wrong_direction:projection=%.3f" % projection)
        if direction_error_deg > self.max_direction_error_deg:
            missing.append("translation_direction_error_deg:%.2f>%.2f" % (
                direction_error_deg, self.max_direction_error_deg))
        if self.expected_distance_m > 0.0:
            scale = projection / self.expected_distance_m
            self.last_metrics["delta_scale"] = scale
            if scale < self.min_delta_scale or scale > self.max_delta_scale:
                missing.append("translation_scale:%.3f_not_in_%.3f..%.3f" % (
                    scale, self.min_delta_scale, self.max_delta_scale))
        if not missing:
            notes.append("delta=(%.3f,%.3f,%.3f) direction_error_deg=%.2f" % (
                observed[0], observed[1], observed[2], direction_error_deg))
        return not missing, missing, notes

    def _evaluate_yaw(self, missing, notes):
        delta = self.poses.delta()
        if delta is None:
            missing.append("pose_delta_missing")
            return False, missing, notes
        yaw_delta_deg = math.degrees(delta[3])
        expected_sign = self.expected_yaw_sign
        if expected_sign == 0 and abs(self.expected_yaw_deg) > 1.0e-9:
            expected_sign = 1 if self.expected_yaw_deg > 0.0 else -1
        self.last_metrics = {
            "motion_type": "yaw",
            "motion_name": self.motion_name,
            "observed_delta": list(delta[:3]),
            "observed_yaw_delta_deg": yaw_delta_deg,
            "expected_yaw_deg": self.expected_yaw_deg,
            "expected_yaw_sign": expected_sign,
            "sample_count": self.poses.count(),
            "pose_rate_hz": self.poses.rate_hz(),
        }
        if abs(yaw_delta_deg) < self.min_yaw_deg:
            missing.append("yaw_delta_deg:%.2f<%.2f" % (abs(yaw_delta_deg), self.min_yaw_deg))
        if expected_sign != 0 and yaw_delta_deg * expected_sign <= 0.0:
            missing.append("yaw_wrong_direction:%.2f_sign_%d" % (yaw_delta_deg, expected_sign))
        if abs(self.expected_yaw_deg) > 1.0e-9:
            error = abs(math.degrees(_angle_error(math.radians(yaw_delta_deg), math.radians(self.expected_yaw_deg))))
            self.last_metrics["yaw_error_deg"] = error
            if error > self.max_yaw_error_deg:
                missing.append("yaw_error_deg:%.2f>%.2f" % (error, self.max_yaw_error_deg))
        if not missing:
            notes.append("yaw_delta_deg=%.2f" % yaw_delta_deg)
        return not missing, missing, notes

    def _evaluate(self):
        missing, notes = self._base_checks()
        if self.motion_type == "translation":
            return self._evaluate_translation(missing, notes)
        return self._evaluate_yaw(missing, notes)

    def _finish(self, success, reason, missing=None, notes=None):
        if self.finished:
            return
        self.finished = True
        self.exit_code = 0 if success else 2
        result = {
            "observer": "fastlio_frame_alignment_observer",
            "success": bool(success),
            "reason": reason,
            "missing": list(missing or []),
            "notes": list(notes or []),
            "metrics": self.last_metrics,
            "parameters": {
                "pose_topic": self.pose_topic,
                "expected_parent": self.expected_parent,
                "motion_type": self.motion_type,
                "motion_name": self.motion_name,
                "expected_x": self.expected_x,
                "expected_y": self.expected_y,
                "expected_z": self.expected_z,
                "expected_distance_m": self.expected_distance_m,
                "expected_yaw_deg": self.expected_yaw_deg,
                "expected_yaw_sign": self.expected_yaw_sign,
                "require_vision_status_ok": self.require_vision_status_ok,
                "require_mavros_output_disabled": self.require_mavros_output_disabled,
            },
        }
        text = json.dumps(result, indent=2, sort_keys=True)
        if self.evidence_file:
            path = pathlib.Path(self.evidence_file).expanduser()
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(text + "\n", encoding="utf-8")
            rospy.loginfo("Wrote FAST-LIO frame-alignment evidence: %s", path)
        else:
            print(text)
        rospy.signal_shutdown(reason)

    def _tick(self, _event):
        if self.finished:
            return
        elapsed = time.monotonic() - self.started
        if elapsed >= self.observe_seconds:
            success, missing, notes = self._evaluate()
            if success:
                reason = "frame_alignment_preview_passed motion=%s" % self.motion_name
            else:
                reason = "frame_alignment_preview_failed:%s" % ",".join(missing)
            self._finish(success, reason, missing, notes)
            return
        if elapsed >= self.startup_timeout and self.poses.count() < self.min_samples:
            self.last_metrics = {
                "sample_count": self.poses.count(),
                "last_issue": self.poses.last_issue,
            }
            self._finish(False, "startup_timeout:%s" % self.poses.last_issue,
                         ["pose_samples:%d<%d" % (self.poses.count(), self.min_samples)], [])


def main():
    rospy.init_node("fastlio_frame_alignment_observer", anonymous=True)
    observer = FastlioFrameAlignmentObserver()
    rospy.spin()
    return observer.exit_code


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print("ERROR: %s" % exc, file=sys.stderr)
        sys.exit(3)
