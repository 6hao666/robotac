#!/usr/bin/env python3
"""Read-only external-vision acceptance observer for PX4/MAVROS.

This node subscribes only. It never publishes setpoints, calls MAVROS services,
changes parameters, arms, or requests modes. Use it on the ground after the
FAST-LIO bridge is publishing to MAVROS: slowly move the aircraft by a small
known amount and verify that PX4/MAVROS local_position moves in the same ENU
direction as the external-vision input.
"""

import json
import math
import pathlib
import sys
import time

import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from mavros_msgs.msg import ExtendedState, State
from nav_msgs.msg import Odometry
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


class PoseStream(object):
    def __init__(self, name, window_samples):
        self.name = name
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


class EvAcceptanceObserver(object):
    def __init__(self):
        self.observe_seconds = float(rospy.get_param("~observe_seconds", 20.0))
        self.startup_timeout = float(rospy.get_param("~startup_timeout", 60.0))
        self.stream_timeout = float(rospy.get_param("~stream_timeout", 0.75))
        self.age_limit = float(rospy.get_param("~age_limit", 0.50))
        self.future_tolerance = float(rospy.get_param("~future_tolerance", 0.10))
        self.min_samples = int(rospy.get_param("~min_samples", 20))
        self.min_rate_hz = float(rospy.get_param("~min_rate_hz", 5.0))
        self.min_motion_m = float(rospy.get_param("~min_motion_m", 0.30))
        self.max_direction_error_deg = float(rospy.get_param("~max_direction_error_deg", 25.0))
        self.min_delta_scale = float(rospy.get_param("~min_delta_scale", 0.50))
        self.max_delta_scale = float(rospy.get_param("~max_delta_scale", 2.00))
        self.require_connected = _as_bool(rospy.get_param("~require_connected", True))
        self.require_disarmed = _as_bool(rospy.get_param("~require_disarmed", True))
        self.require_on_ground = _as_bool(rospy.get_param("~require_on_ground", True))
        self.require_vision_output_enabled = _as_bool(
            rospy.get_param("~require_vision_output_enabled", True))
        self.require_vision_status_ok = _as_bool(
            rospy.get_param("~require_vision_status_ok", True))
        self.evidence_file = str(rospy.get_param("~evidence_file", "")).strip()
        self.local_parent = str(rospy.get_param("~local_parent", "map")).strip()
        self.local_child = str(rospy.get_param("~local_child", "base_link")).strip()
        self.vision_parent = str(rospy.get_param("~vision_parent", "odom")).strip()
        self.vision_type = str(rospy.get_param("~vision_type", "pose")).strip().lower()
        self._validate_parameters()

        # Keep the full ground-observation window even at camera/LiDAR-like
        # rates so the operator can move the aircraft once during the test
        # rather than exactly at the final evaluation instant.
        window_samples = max(self.min_samples * 4, 4096)
        self.local = PoseStream("mavros_local_position", window_samples)
        self.vision = PoseStream("mavros_vision_pose_input", window_samples)
        self.state = None
        self.state_receive = None
        self.extended = None
        self.extended_receive = None
        self.vision_output_enabled = None
        self.output_enabled_receive = None
        self.vision_status = "waiting"
        self.vision_status_receive = None
        self.exit_code = 1
        self.last_metrics = {}
        self.started = time.monotonic()
        self.ready_since = None

        rospy.Subscriber(rospy.get_param("~state_topic", "/mavros/state"),
                         State, self._state_cb, queue_size=10)
        rospy.Subscriber(rospy.get_param("~extended_state_topic", "/mavros/extended_state"),
                         ExtendedState, self._extended_cb, queue_size=10)
        rospy.Subscriber(rospy.get_param("~local_topic", "/mavros/local_position/odom"),
                         Odometry, self._local_cb, queue_size=20)
        vision_topic = rospy.get_param("~vision_topic", "/mavros/vision_pose/pose")
        if self.vision_type in ("pose", "pose_stamped", "posestamped"):
            rospy.Subscriber(vision_topic, PoseStamped, self._vision_cb, queue_size=20)
        elif self.vision_type in (
                "pose_cov", "pose_with_covariance", "pose_with_covariance_stamped",
                "posewithcovariancestamped"):
            rospy.Subscriber(vision_topic, PoseWithCovarianceStamped,
                             self._vision_cov_cb, queue_size=20)
        else:
            raise ValueError("unsupported vision_type: %s" % self.vision_type)
        rospy.Subscriber(rospy.get_param("~vision_output_enabled_topic",
                                         "/robotac/fastlio_vision/output_enabled"),
                         Bool, self._output_enabled_cb, queue_size=10)
        rospy.Subscriber(rospy.get_param("~vision_status_topic",
                                         "/robotac/fastlio_vision/status"),
                         String, self._vision_status_cb, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.10), self._tick)

    def _validate_parameters(self):
        positive = (
            self.observe_seconds, self.startup_timeout, self.stream_timeout,
            self.age_limit, self.min_rate_hz, self.min_motion_m,
            self.max_direction_error_deg, self.min_delta_scale, self.max_delta_scale,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError("observer timeouts, rates, motion and scale limits must be finite and positive")
        if self.min_samples < 2:
            raise ValueError("min_samples must be at least 2")
        if self.min_delta_scale > self.max_delta_scale:
            raise ValueError("min_delta_scale must be <= max_delta_scale")
        if not math.isfinite(self.future_tolerance) or self.future_tolerance < 0.0:
            raise ValueError("future_tolerance must be finite and non-negative")
        if self.max_direction_error_deg >= 90.0:
            raise ValueError("max_direction_error_deg must be below 90 degrees")
        if not self.local_parent or not self.local_child or not self.vision_parent:
            raise ValueError("expected frame names must be non-empty")
        allowed_pose_types = (
            "pose", "pose_stamped", "posestamped",
            "pose_cov", "pose_with_covariance", "pose_with_covariance_stamped",
            "posewithcovariancestamped",
        )
        if self.vision_type not in allowed_pose_types:
            raise ValueError("vision_type must be pose or pose_cov")

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

    def _state_cb(self, msg):
        self.state = msg
        self.state_receive = time.monotonic()

    def _extended_cb(self, msg):
        self.extended = msg
        self.extended_receive = time.monotonic()

    def _output_enabled_cb(self, msg):
        self.vision_output_enabled = bool(msg.data)
        self.output_enabled_receive = time.monotonic()

    def _vision_status_cb(self, msg):
        self.vision_status = str(msg.data).strip() or "empty"
        self.vision_status_receive = time.monotonic()

    def _local_cb(self, msg):
        if msg.header.frame_id != self.local_parent or msg.child_frame_id != self.local_child:
            self.local.reject("unexpected_frames:%s->%s" % (msg.header.frame_id, msg.child_frame_id))
            return
        if not _finite_pose(msg.pose.pose):
            self.local.reject("nonfinite_or_invalid_quaternion")
            return
        valid, reason = self._stamp_current(msg.header.stamp)
        if not valid:
            self.local.reject(reason)
            return
        self.local.accept(msg.header.stamp, msg.pose.pose)

    def _accept_vision_pose(self, header, pose):
        if header.frame_id != self.vision_parent:
            self.vision.reject("unexpected_parent:%s" % header.frame_id)
            return
        if not _finite_pose(pose):
            self.vision.reject("nonfinite_or_invalid_quaternion")
            return
        valid, reason = self._stamp_current(header.stamp)
        if not valid:
            self.vision.reject(reason)
            return
        self.vision.accept(header.stamp, pose)

    def _vision_cb(self, msg):
        self._accept_vision_pose(msg.header, msg.pose)

    def _vision_cov_cb(self, msg):
        self._accept_vision_pose(msg.header, msg.pose.pose)

    @staticmethod
    def _fresh(receive_time, timeout):
        return receive_time is not None and time.monotonic() - receive_time <= timeout

    def _basic_ready(self):
        if self.require_connected and (not self._fresh(self.state_receive, self.stream_timeout) or
                                       self.state is None or not self.state.connected):
            return False, "mavros_not_connected"
        if self.require_disarmed and self.state is not None and self.state.armed:
            return False, "vehicle_armed"
        if self.require_on_ground:
            if (not self._fresh(self.extended_receive, self.stream_timeout) or
                    self.extended is None or
                    self.extended.landed_state != ExtendedState.LANDED_STATE_ON_GROUND):
                return False, "vehicle_not_on_ground"
        if self.require_vision_output_enabled:
            if (self.vision_output_enabled is not True or
                    not self._fresh(self.output_enabled_receive, self.stream_timeout)):
                return False, "vision_output_not_enabled"
        if self.require_vision_status_ok:
            if (not self._fresh(self.vision_status_receive, self.stream_timeout) or
                    not (self.vision_status == "ok" or self.vision_status.startswith("ok "))):
                return False, "vision_status_not_ok:%s" % self.vision_status
        for stream in (self.local, self.vision):
            if stream.count() < self.min_samples:
                return False, "%s_samples:%d" % (stream.name, stream.count())
            if not stream.fresh(self.stream_timeout):
                return False, "%s_stale:%s" % (stream.name, stream.last_issue)
            if stream.rate_hz() < self.min_rate_hz:
                return False, "%s_rate:%.2f" % (stream.name, stream.rate_hz())
        return True, "ok"

    @staticmethod
    def _norm3(delta):
        return math.sqrt(delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2])

    @staticmethod
    def _direction_cosine(a, b):
        an = EvAcceptanceObserver._norm3(a)
        bn = EvAcceptanceObserver._norm3(b)
        if an <= 0.0 or bn <= 0.0:
            return -1.0
        return (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]) / (an * bn)

    def _evaluate(self):
        ready, reason = self._basic_ready()
        if not ready:
            self.last_metrics = self._metric_snapshot()
            return False, reason
        local_delta = self.local.delta()
        vision_delta = self.vision.delta()
        if local_delta is None or vision_delta is None:
            self.last_metrics = self._metric_snapshot()
            return False, "delta_unavailable"
        local_motion = self._norm3(local_delta)
        vision_motion = self._norm3(vision_delta)
        self.last_metrics = self._metric_snapshot(
            local_delta=local_delta,
            vision_delta=vision_delta,
            local_motion=local_motion,
            vision_motion=vision_motion)
        if vision_motion < self.min_motion_m or local_motion < self.min_motion_m:
            return False, "motion_too_small local=%.3f vision=%.3f required=%.3f" % (
                local_motion, vision_motion, self.min_motion_m)
        delta_direction_cos = self._direction_cosine(local_delta, vision_delta)
        min_cos = math.cos(math.radians(self.max_direction_error_deg))
        self.last_metrics.update({
            "delta_direction_cos": delta_direction_cos,
            "min_direction_cos": min_cos,
        })
        if delta_direction_cos < min_cos:
            return False, "delta_direction_mismatch cos=%.3f min=%.3f local_delta=(%.3f,%.3f,%.3f) vision_delta=(%.3f,%.3f,%.3f)" % (
                delta_direction_cos, min_cos,
                local_delta[0], local_delta[1], local_delta[2],
                vision_delta[0], vision_delta[1], vision_delta[2])
        delta_scale = local_motion / vision_motion
        self.last_metrics["delta_scale"] = delta_scale
        if delta_scale < self.min_delta_scale or delta_scale > self.max_delta_scale:
            return False, "delta_scale_mismatch scale=%.3f local_motion=%.3f vision_motion=%.3f" % (
                delta_scale, local_motion, vision_motion)
        return True, "ev_acceptance_passed local_delta=(%.3f,%.3f,%.3f) vision_delta=(%.3f,%.3f,%.3f) delta_direction_cos=%.3f delta_scale=%.3f" % (
            local_delta[0], local_delta[1], local_delta[2],
            vision_delta[0], vision_delta[1], vision_delta[2],
            delta_direction_cos, delta_scale)

    def _summary(self):
        return (
            "state_connected=%s armed=%s extended=%s vision_output_enabled=%s "
            "vision_status=%s local_count=%d local_rate=%.2f local_issue=%s "
            "vision_count=%d vision_rate=%.2f vision_issue=%s" % (
                "unknown" if self.state is None else self.state.connected,
                "unknown" if self.state is None else self.state.armed,
                "unknown" if self.extended is None else self.extended.landed_state,
                self.vision_output_enabled, self.vision_status,
                self.local.count(), self.local.rate_hz(), self.local.last_issue,
                self.vision.count(), self.vision.rate_hz(), self.vision.last_issue))

    def _stream_snapshot(self, stream):
        delta = stream.delta()
        return {
            "count": stream.count(),
            "rate_hz": stream.rate_hz(),
            "fresh": stream.fresh(self.stream_timeout),
            "last_issue": stream.last_issue,
            "rejects": stream.rejects,
            "delta": None if delta is None else [delta[0], delta[1], delta[2], delta[3]],
        }

    def _metric_snapshot(self, local_delta=None, vision_delta=None,
                         local_motion=None, vision_motion=None):
        return {
            "local": self._stream_snapshot(self.local),
            "vision": self._stream_snapshot(self.vision),
            "local_delta": None if local_delta is None else [local_delta[0], local_delta[1], local_delta[2], local_delta[3]],
            "vision_delta": None if vision_delta is None else [vision_delta[0], vision_delta[1], vision_delta[2], vision_delta[3]],
            "local_motion_m": local_motion,
            "vision_motion_m": vision_motion,
        }

    def _write_evidence(self, success, reason):
        if not self.evidence_file:
            return
        path = pathlib.Path(self.evidence_file).expanduser()
        if path.parent:
            path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "observer": "ev_acceptance_observer",
            "success": bool(success),
            "reason": reason,
            "summary": self._summary(),
            "generated_at_wall_time": time.time(),
            "parameters": {
                "observe_seconds": self.observe_seconds,
                "startup_timeout": self.startup_timeout,
                "stream_timeout": self.stream_timeout,
                "age_limit": self.age_limit,
                "future_tolerance": self.future_tolerance,
                "min_samples": self.min_samples,
                "min_rate_hz": self.min_rate_hz,
                "min_motion_m": self.min_motion_m,
                "max_direction_error_deg": self.max_direction_error_deg,
                "min_delta_scale": self.min_delta_scale,
                "max_delta_scale": self.max_delta_scale,
                "require_connected": self.require_connected,
                "require_disarmed": self.require_disarmed,
                "require_on_ground": self.require_on_ground,
                "require_vision_output_enabled": self.require_vision_output_enabled,
                "require_vision_status_ok": self.require_vision_status_ok,
                "local_parent": self.local_parent,
                "local_child": self.local_child,
                "vision_parent": self.vision_parent,
            },
            "metrics": self.last_metrics,
        }
        path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")

    def _finish(self, success, reason):
        try:
            self._write_evidence(success, reason)
        except Exception as exc:
            if success:
                success = False
                reason = "evidence_write_failed:%s" % exc
            rospy.logerr("FAIL: failed to write EV acceptance evidence: %s", exc)
        self.exit_code = 0 if success else 2
        level = rospy.loginfo if success else rospy.logerr
        level("%s: %s\n%s", "PASS" if success else "FAIL", reason, self._summary())
        rospy.signal_shutdown(reason)

    def _tick(self, _event):
        ready, _reason = self._basic_ready()
        if ready:
            if self.ready_since is None:
                self.ready_since = time.monotonic()
            elif time.monotonic() - self.ready_since >= self.observe_seconds:
                success, reason = self._evaluate()
                self._finish(success, reason)
                return
        else:
            self.ready_since = None
        if time.monotonic() - self.started >= self.startup_timeout:
            success, reason = self._evaluate()
            self._finish(success, "startup_timeout:%s" % reason)


if __name__ == "__main__":
    rospy.init_node("ev_acceptance_observer", anonymous=True)
    try:
        observer = EvAcceptanceObserver()
        rospy.loginfo("Read-only EV acceptance observer started; move the disarmed aircraft slowly on the ground")
        rospy.spin()
        sys.exit(observer.exit_code)
    except Exception as exc:
        rospy.logerr("FAIL: EV acceptance observer initialization: %s", exc)
        sys.exit(3)
