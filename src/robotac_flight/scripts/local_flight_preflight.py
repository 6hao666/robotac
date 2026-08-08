#!/usr/bin/env python3
"""Read-only readiness check for local MAVROS flight with FAST-LIO vision.

This node only subscribes to existing ROS topics by default. When
``check_px4_vision_params`` is enabled, it also calls MAVROS' read-only
``/mavros/param/get`` service. It never creates publishers, calls
flight-control services, changes PX4 parameters, or emits MAVROS setpoints.
"""

import json
import math
import pathlib
import sys
import time

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from mavros_msgs.msg import EstimatorStatus, ExtendedState, State, TimesyncStatus
from mavros_msgs.srv import ParamGet
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String


# PX4 v1.10/v1.11 EKF2_AID_MASK: bit 3 is vision position, bit 4 is vision yaw.
LEGACY_AID_MASK_VISION_POSITION = 1 << 3
LEGACY_AID_MASK_VISION_YAW = 1 << 4


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


class Stream(object):
    """Track one consecutive, timestamp-valid stream without publishing data."""

    def __init__(self, name):
        self.name = name
        self.count = 0
        self.first_stamp = None
        self.last_stamp = None
        self.recent_stamps = []
        self.last_receive = None
        self.last_issue = "waiting"

    def reject(self, reason):
        self.count = 0
        self.first_stamp = None
        self.last_stamp = None
        self.recent_stamps = []
        self.last_receive = None
        self.last_issue = reason

    def accept(self, stamp):
        now = time.monotonic()
        if self.last_stamp is not None and stamp <= self.last_stamp:
            self.reject("non_monotonic_timestamp")
            return False
        if self.count == 0:
            self.first_stamp = stamp
        self.count += 1
        self.last_stamp = stamp
        self.recent_stamps.append(stamp)
        if len(self.recent_stamps) > 64:
            del self.recent_stamps[:-64]
        self.last_receive = now
        self.last_issue = "ok"
        return True

    def rate_hz(self):
        if self.count < 2 or self.first_stamp is None or self.last_stamp is None:
            return 0.0
        duration = (self.last_stamp - self.first_stamp).to_sec()
        return 0.0 if duration <= 0.0 else float(self.count - 1) / duration

    def fresh(self, timeout):
        return self.last_receive is not None and time.monotonic() - self.last_receive <= timeout


class LocalFlightPreflight(object):
    def __init__(self):
        self.observe_seconds = float(rospy.get_param("~observe_seconds", 10.0))
        self.startup_timeout = float(rospy.get_param("~startup_timeout", 20.0))
        self.mavros_timeout = float(rospy.get_param("~mavros_timeout", 1.0))
        self.estimator_timeout = float(rospy.get_param("~estimator_timeout", 1.0))
        self.local_age_limit = float(rospy.get_param("~local_age_limit", 0.50))
        self.vision_age_limit = float(rospy.get_param("~vision_age_limit", 0.30))
        self.vision_status_timeout = float(
            rospy.get_param("~vision_status_timeout", 0.50))
        self.future_tolerance = float(rospy.get_param("~future_tolerance", 0.10))
        self.min_local_rate = float(rospy.get_param("~min_local_rate_hz", 10.0))
        self.min_fastlio_rate = float(rospy.get_param("~min_fastlio_rate_hz", 5.0))
        self.min_preview_rate = float(rospy.get_param("~min_preview_rate_hz", 5.0))
        self.min_output_rate = float(rospy.get_param("~min_vision_output_rate_hz", 5.0))
        self.min_local_samples = int(rospy.get_param("~min_local_samples", 10))
        self.min_fastlio_samples = int(rospy.get_param("~min_fastlio_samples", 5))
        self.min_preview_samples = int(rospy.get_param("~min_preview_samples", 5))
        self.min_output_samples = int(rospy.get_param("~min_vision_output_samples", 5))
        self.vision_health_window = float(
            rospy.get_param("~vision_health_window_seconds", 5.0))
        self.timesync_timeout = float(rospy.get_param("~timesync_timeout", 1.0))
        self.max_timesync_rtt_ms = float(
            rospy.get_param("~max_timesync_rtt_ms", 20.0))

        self.require_vision = _as_bool(rospy.get_param("~require_vision", True))
        self.require_vision_output = _as_bool(
            rospy.get_param("~require_vision_output", False))
        self.require_timesync = _as_bool(
            rospy.get_param("~require_timesync", self.require_vision_output))
        self.require_estimator = _as_bool(rospy.get_param("~require_estimator", True))
        self.require_disarmed = _as_bool(rospy.get_param("~require_disarmed", True))
        self.require_on_ground = _as_bool(rospy.get_param("~require_on_ground", True))
        self.check_px4_vision_params = _as_bool(
            rospy.get_param("~check_px4_vision_params", False))
        self.require_yaw_fusion = _as_bool(rospy.get_param("~require_yaw_fusion", False))
        self.require_ev_offsets_zero = _as_bool(
            rospy.get_param("~require_ev_offsets_zero", True))
        self.ev_offset_tolerance_m = float(rospy.get_param("~ev_offset_tolerance_m", 0.01))
        self.require_ev_delay = _as_bool(rospy.get_param("~require_ev_delay", False))
        self.expected_ev_delay_ms = float(rospy.get_param("~expected_ev_delay_ms", 0.0))
        self.ev_delay_tolerance_ms = float(rospy.get_param("~ev_delay_tolerance_ms", 20.0))
        self.require_vision_output_consumer = _as_bool(
            rospy.get_param("~require_vision_output_consumer", self.require_vision_output))
        self.vision_output_consumer_node = str(rospy.get_param(
            "~vision_output_consumer_node", "/mavros")).strip()
        self.require_setpoint_consumer = _as_bool(
            rospy.get_param("~require_setpoint_consumer", False))
        self.setpoint_consumer_node = str(rospy.get_param(
            "~setpoint_consumer_node", "/mavros")).strip()
        self.evidence_file = str(rospy.get_param("~evidence_file", "")).strip()

        self.local_parent = str(rospy.get_param("~local_parent", "map")).strip()
        self.local_child = str(rospy.get_param("~local_child", "base_link")).strip()
        self.fastlio_parent = str(rospy.get_param("~fastlio_parent", "camera_init")).strip()
        self.fastlio_child = str(rospy.get_param("~fastlio_child", "body")).strip()
        self.vision_parent = str(rospy.get_param("~vision_parent", "odom")).strip()
        self._validate_parameters()

        self.state_topic = rospy.get_param("~state_topic", "/mavros/state")
        self.extended_state_topic = rospy.get_param(
            "~extended_state_topic", "/mavros/extended_state")
        self.estimator_topic = rospy.get_param(
            "~estimator_topic", "/mavros/estimator_status")
        self.local_topic = rospy.get_param("~local_topic", "/mavros/local_position/odom")
        self.setpoint_topic = rospy.get_param("~setpoint_topic", "/mavros/setpoint_raw/local")
        self.fastlio_topic = rospy.get_param("~fastlio_topic", "/Odometry")
        self.vision_health_topic = rospy.get_param(
            "~vision_health_topic", "/robotac/fastlio_vision/healthy")
        self.vision_status_topic = rospy.get_param(
            "~vision_status_topic", "/robotac/fastlio_vision/status")
        self.preview_topic = rospy.get_param(
            "~preview_topic", "/robotac/fastlio_vision/pose_preview")
        self.vision_output_enabled_topic = rospy.get_param(
            "~vision_output_enabled_topic", "/robotac/fastlio_vision/output_enabled")
        self.vision_output_topic = rospy.get_param(
            "~vision_output_topic", "/mavros/vision_pose/pose_cov")
        self.param_get_service = rospy.get_param("~param_get_service", "/mavros/param/get")

        self.local_stream = Stream("mavros_local_odom")
        self.fastlio_stream = Stream("fastlio_odom")
        self.preview_stream = Stream("fastlio_preview")
        self.output_stream = Stream("mavros_vision_pose")
        self.state = None
        self.state_receive = None
        self.extended_state = None
        self.extended_receive = None
        self.estimator = None
        self.estimator_receive = None
        self.estimator_issue = "waiting"
        self.vision_healthy = False
        self.vision_health_receive = None
        self.vision_healthy_since = None
        self.vision_status = "waiting"
        self.vision_status_receive = None
        self.output_enabled = None
        self.output_enabled_receive = None
        self.vision_output_consumer_issue = "not_checked"
        self.setpoint_consumer_issue = "not_checked"
        self.timesync = None
        self.timesync_receive = None
        self.timesync_issue = "waiting"
        self.px4_params_checked = not self.check_px4_vision_params
        self.px4_params_issue = "not_requested"
        self.px4_param_values = {}
        self.exit_code = 1
        self.started = time.monotonic()
        self.observation_ready_since = None

        rospy.Subscriber(self.state_topic, State, self._state_cb, queue_size=10)
        rospy.Subscriber(self.extended_state_topic, ExtendedState, self._extended_cb, queue_size=10)
        rospy.Subscriber(self.estimator_topic, EstimatorStatus, self._estimator_cb, queue_size=10)
        if self.require_timesync:
            rospy.Subscriber("/mavros/timesync_status", TimesyncStatus,
                             self._timesync_cb, queue_size=10)
        rospy.Subscriber(self.local_topic, Odometry, self._local_cb, queue_size=20)
        if self.require_vision:
            rospy.Subscriber(self.fastlio_topic, Odometry, self._fastlio_cb, queue_size=20)
            rospy.Subscriber(self.vision_health_topic, Bool, self._vision_health_cb, queue_size=10)
            rospy.Subscriber(self.vision_status_topic, String, self._vision_status_cb, queue_size=10)
            rospy.Subscriber(self.preview_topic, PoseWithCovarianceStamped, self._preview_cb, queue_size=20)
        if self.require_vision_output:
            rospy.Subscriber(self.vision_output_enabled_topic, Bool,
                             self._output_enabled_cb, queue_size=10)
            rospy.Subscriber(self.vision_output_topic, PoseWithCovarianceStamped,
                             self._output_cb, queue_size=20)
        self.timer = rospy.Timer(rospy.Duration(0.10), self._tick)

    def _validate_parameters(self):
        positive = (
            self.observe_seconds, self.startup_timeout, self.mavros_timeout,
            self.estimator_timeout, self.local_age_limit, self.vision_age_limit,
            self.vision_status_timeout,
            self.min_local_rate, self.min_fastlio_rate, self.min_preview_rate,
            self.min_output_rate, self.vision_health_window, self.timesync_timeout,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError("all preflight timeouts and rates must be finite and positive")
        if not math.isfinite(self.future_tolerance) or self.future_tolerance < 0.0:
            raise ValueError("future_tolerance must be finite and non-negative")
        if min(self.min_local_samples, self.min_fastlio_samples,
               self.min_preview_samples, self.min_output_samples) < 1:
            raise ValueError("minimum sample counts must be positive")
        if not self.local_parent or not self.local_child or not self.fastlio_parent or \
                not self.fastlio_child or not self.vision_parent:
            raise ValueError("expected frame names must be non-empty")
        if self.require_vision_output and not self.require_vision:
            raise ValueError("require_vision_output requires require_vision=true")
        if self.require_vision_output_consumer and not self.require_vision_output:
            raise ValueError("require_vision_output_consumer requires require_vision_output=true")
        if self.require_vision_output_consumer and not self.vision_output_consumer_node:
            raise ValueError("vision_output_consumer_node must be non-empty")
        if self.require_ev_delay and not self.check_px4_vision_params:
            raise ValueError("require_ev_delay requires check_px4_vision_params=true")
        if self.require_setpoint_consumer and (
                not self.setpoint_topic or not self.setpoint_consumer_node):
            raise ValueError("setpoint_topic and setpoint_consumer_node must be non-empty")
        if not math.isfinite(self.max_timesync_rtt_ms) or self.max_timesync_rtt_ms < 0.0:
            raise ValueError("max_timesync_rtt_ms must be finite and non-negative")
        if not math.isfinite(self.ev_offset_tolerance_m) or self.ev_offset_tolerance_m < 0.0:
            raise ValueError("ev_offset_tolerance_m must be finite and non-negative")
        if (not math.isfinite(self.expected_ev_delay_ms) or
                not math.isfinite(self.ev_delay_tolerance_ms) or
                self.ev_delay_tolerance_ms < 0.0):
            raise ValueError("expected_ev_delay_ms and ev_delay_tolerance_ms must be finite; tolerance must be non-negative")

    def _stamp_is_current(self, stamp, age_limit):
        if stamp == rospy.Time(0):
            return False, "zero_timestamp"
        now = rospy.Time.now()
        if now == rospy.Time(0):
            return False, "ros_clock_unavailable"
        age = (now - stamp).to_sec()
        if age > age_limit or age < -self.future_tolerance:
            return False, "timestamp_age:%.3f" % age
        return True, "ok"

    def _accept_pose_stream(self, stream, msg, parent, child, age_limit):
        if msg.header.frame_id != parent:
            stream.reject("unexpected_parent:%s" % msg.header.frame_id)
            return
        if child is not None and msg.child_frame_id != child:
            stream.reject("unexpected_child:%s" % msg.child_frame_id)
            return
        if not _finite_pose(msg.pose.pose):
            stream.reject("nonfinite_or_invalid_quaternion")
            return
        valid, reason = self._stamp_is_current(msg.header.stamp, age_limit)
        if not valid:
            stream.reject(reason)
            return
        stream.accept(msg.header.stamp)

    def _state_cb(self, msg):
        self.state = msg
        self.state_receive = time.monotonic()

    def _extended_cb(self, msg):
        self.extended_state = msg
        self.extended_receive = time.monotonic()

    def _estimator_cb(self, msg):
        self.estimator = msg
        self.estimator_receive = time.monotonic()
        if not msg.attitude_status_flag:
            self.estimator_issue = "attitude_invalid"
        elif not msg.pos_horiz_rel_status_flag:
            self.estimator_issue = "horizontal_relative_invalid"
        elif not (msg.pos_vert_abs_status_flag or msg.pos_vert_agl_status_flag):
            self.estimator_issue = "vertical_invalid"
        else:
            self.estimator_issue = "ok"

    def _timesync_cb(self, msg):
        self.timesync = msg
        self.timesync_receive = time.monotonic()
        rtt = float(msg.round_trip_time_ms)
        if not math.isfinite(rtt) or rtt < 0.0:
            self.timesync_issue = "invalid_rtt"
        elif rtt > self.max_timesync_rtt_ms:
            self.timesync_issue = "rtt_ms:%.2f" % rtt
        else:
            self.timesync_issue = "ok"

    def _local_cb(self, msg):
        self._accept_pose_stream(
            self.local_stream, msg, self.local_parent, self.local_child, self.local_age_limit)

    def _fastlio_cb(self, msg):
        self._accept_pose_stream(
            self.fastlio_stream, msg, self.fastlio_parent, self.fastlio_child,
            self.vision_age_limit)

    def _preview_cb(self, msg):
        if msg.header.frame_id != self.vision_parent:
            self.preview_stream.reject("unexpected_parent:%s" % msg.header.frame_id)
            return
        if not _finite_pose(msg.pose.pose):
            self.preview_stream.reject("nonfinite_or_invalid_quaternion")
            return
        valid, reason = self._stamp_is_current(msg.header.stamp, self.vision_age_limit)
        if not valid:
            self.preview_stream.reject(reason)
            return
        self.preview_stream.accept(msg.header.stamp)

    def _output_cb(self, msg):
        if msg.header.frame_id != self.vision_parent:
            self.output_stream.reject("unexpected_parent:%s" % msg.header.frame_id)
            return
        if not _finite_pose(msg.pose.pose):
            self.output_stream.reject("nonfinite_or_invalid_quaternion")
            return
        valid, reason = self._stamp_is_current(msg.header.stamp, self.vision_age_limit)
        if not valid:
            self.output_stream.reject(reason)
            return
        self.output_stream.accept(msg.header.stamp)

    def _vision_health_cb(self, msg):
        now = time.monotonic()
        self.vision_healthy = bool(msg.data)
        self.vision_health_receive = now
        if self.vision_healthy:
            if self.vision_healthy_since is None:
                self.vision_healthy_since = now
        else:
            self.vision_healthy_since = None

    def _vision_status_cb(self, msg):
        self.vision_status = str(msg.data).strip() or "empty"
        self.vision_status_receive = time.monotonic()

    def _output_enabled_cb(self, msg):
        self.output_enabled = bool(msg.data)
        self.output_enabled_receive = time.monotonic()

    @staticmethod
    def _fresh(receive, timeout):
        return receive is not None and time.monotonic() - receive <= timeout

    def _stream_ready(self, stream, samples, rate, timeout):
        return (stream.count >= samples and stream.rate_hz() >= rate and stream.fresh(timeout))

    @staticmethod
    def _matching_stamp(first, second):
        # Callback scheduling can leave a derived preview one input frame
        # behind. Its timestamp must still be one of the recently accepted
        # source stamps, rather than merely be close in wall-clock time.
        return (first.last_stamp is not None and second.last_stamp is not None and
                any(second.last_stamp == stamp for stamp in first.recent_stamps))

    def _topic_consumer_present(self, target_topic, node_name):
        try:
            code, _message, state = rospy.get_master().getSystemState()
        except Exception as exc:
            return False, "master_error:%s" % exc
        if code != 1:
            return False, "master_state_unavailable"
        for topic, nodes in state[1]:
            if topic == target_topic:
                if node_name in nodes:
                    return True, "ok"
                return False, "missing:%s" % node_name
        return False, "topic_unsubscribed"

    def _vision_output_consumer_present(self):
        present, issue = self._topic_consumer_present(
            self.vision_output_topic, self.vision_output_consumer_node)
        self.vision_output_consumer_issue = issue
        return present

    def _setpoint_consumer_present(self):
        present, issue = self._topic_consumer_present(
            self.setpoint_topic, self.setpoint_consumer_node)
        self.setpoint_consumer_issue = issue
        return present

    def _mavros_ready(self):
        if not self._fresh(self.state_receive, self.mavros_timeout):
            return False
        if self.state is None or not self.state.connected:
            return False
        if self.require_disarmed and self.state.armed:
            return False
        if not self._fresh(self.extended_receive, self.mavros_timeout):
            return False
        if self.require_on_ground and (
                self.extended_state is None or
                self.extended_state.landed_state != ExtendedState.LANDED_STATE_ON_GROUND):
            return False
        if self.require_estimator and (
                not self._fresh(self.estimator_receive, self.estimator_timeout) or
                self.estimator_issue != "ok"):
            return False
        if self.require_timesync and (
                not self._fresh(self.timesync_receive, self.timesync_timeout) or
                self.timesync_issue != "ok"):
            return False
        if self.require_setpoint_consumer and not self._setpoint_consumer_present():
            return False
        return self._stream_ready(
            self.local_stream, self.min_local_samples, self.min_local_rate, self.mavros_timeout)

    def _vision_ready(self):
        if not self.require_vision:
            return True
        health_window_ok = (self.vision_healthy and self._fresh(
            self.vision_health_receive, self.vision_age_limit) and
            self.vision_healthy_since is not None and
            time.monotonic() - self.vision_healthy_since >= self.vision_health_window)
        status_ready = (self._fresh(self.vision_status_receive, self.vision_status_timeout) and
                        (self.vision_status == "ok" or
                         self.vision_status.startswith("ok ")))
        base_ready = (
            health_window_ok and status_ready and
            self._stream_ready(self.fastlio_stream, self.min_fastlio_samples,
                               self.min_fastlio_rate, self.vision_age_limit) and
            self._stream_ready(self.preview_stream, self.min_preview_samples,
                               self.min_preview_rate, self.vision_age_limit) and
            self._matching_stamp(self.fastlio_stream, self.preview_stream))
        if not self.require_vision_output:
            return base_ready
        output_ready = (base_ready and self.output_enabled is True and self._fresh(
            self.output_enabled_receive, self.startup_timeout) and
            self._stream_ready(self.output_stream, self.min_output_samples,
                               self.min_output_rate, self.vision_age_limit) and
            self._matching_stamp(self.preview_stream, self.output_stream))
        if output_ready and self.require_vision_output_consumer:
            output_ready = self._vision_output_consumer_present()
        return output_ready

    def _get_px4_param(self, proxy, name):
        response = proxy(param_id=name)
        if not response.success:
            return None
        value = int(response.value.integer) if response.value.integer != 0 else float(response.value.real)
        self.px4_param_values[name] = value
        return value

    def _check_px4_params(self):
        if self.px4_params_checked:
            return self.px4_params_issue == "ok"
        try:
            rospy.wait_for_service(self.param_get_service, timeout=2.0)
            get = rospy.ServiceProxy(self.param_get_service, ParamGet)
            ev_ctrl = self._get_px4_param(get, "EKF2_EV_CTRL")
            if ev_ctrl is not None:
                required = 0x03 | (0x08 if self.require_yaw_fusion else 0x00)
                if int(ev_ctrl) & required != required:
                    self.px4_params_issue = "EKF2_EV_CTRL_missing_mask:0x%02x" % required
                else:
                    self.px4_params_issue = "ok"
            else:
                aid_mask = self._get_px4_param(get, "EKF2_AID_MASK")
                required = (LEGACY_AID_MASK_VISION_POSITION |
                            (LEGACY_AID_MASK_VISION_YAW if self.require_yaw_fusion else 0))
                if aid_mask is None:
                    self.px4_params_issue = "vision_fusion_parameter_unavailable"
                elif int(aid_mask) & required != required:
                    self.px4_params_issue = "EKF2_AID_MASK_missing_mask:0x%02x" % required
                else:
                    self.px4_params_issue = "ok"
            if self.px4_params_issue == "ok" and self.require_ev_offsets_zero:
                offsets = []
                for name in ("EKF2_EV_POS_X", "EKF2_EV_POS_Y", "EKF2_EV_POS_Z"):
                    value = self._get_px4_param(get, name)
                    if value is None:
                        self.px4_params_issue = "%s_unavailable" % name
                        break
                    offsets.append(float(value))
                if self.px4_params_issue == "ok" and any(
                        not math.isfinite(value) for value in offsets):
                    self.px4_params_issue = "EV_POS_nonfinite:%s" % ",".join(
                        str(value) for value in offsets)
                if self.px4_params_issue == "ok" and any(
                        abs(value) > self.ev_offset_tolerance_m for value in offsets):
                    self.px4_params_issue = "EV_POS_nonzero:%s" % ",".join(
                        "%.4f" % value for value in offsets)
            if self.px4_params_issue == "ok" and self.require_ev_delay:
                value = self._get_px4_param(get, "EKF2_EV_DELAY")
                if value is None:
                    self.px4_params_issue = "EKF2_EV_DELAY_unavailable"
                elif not math.isfinite(float(value)):
                    self.px4_params_issue = "EKF2_EV_DELAY_nonfinite"
                elif abs(float(value) - self.expected_ev_delay_ms) > self.ev_delay_tolerance_ms:
                    self.px4_params_issue = "EKF2_EV_DELAY_mismatch:%.3f_expected_%.3f_tol_%.3f" % (
                        float(value), self.expected_ev_delay_ms, self.ev_delay_tolerance_ms)
        except (rospy.ROSException, rospy.ServiceException) as exc:
            self.px4_params_issue = "param_get_failed:%s" % exc
        self.px4_params_checked = True
        return self.px4_params_issue == "ok"

    def _summary(self):
        parts = [
            "state=%s connected=%s armed=%s" % (
                "fresh" if self._fresh(self.state_receive, self.mavros_timeout) else "stale",
                "unknown" if self.state is None else self.state.connected,
                "unknown" if self.state is None else self.state.armed),
            "extended=%s" % (
                "unknown" if self.extended_state is None else self.extended_state.landed_state),
            "estimator=%s" % self.estimator_issue,
            "local=count:%d rate:%.2f issue:%s" % (
                self.local_stream.count, self.local_stream.rate_hz(), self.local_stream.last_issue),
            "setpoint_consumer=%s issue:%s" % (
                self.setpoint_consumer_node, self.setpoint_consumer_issue),
        ]
        if self.require_vision:
            parts.extend((
                "fastlio=count:%d rate:%.2f issue:%s" % (
                    self.fastlio_stream.count, self.fastlio_stream.rate_hz(),
                    self.fastlio_stream.last_issue),
                "preview=count:%d rate:%.2f issue:%s" % (
                    self.preview_stream.count, self.preview_stream.rate_hz(),
                    self.preview_stream.last_issue),
                "vision_healthy=%s status_fresh=%s status=%s fastlio_preview_stamp_match=%s" % (
                    self.vision_healthy,
                    self._fresh(self.vision_status_receive, self.vision_status_timeout),
                    self.vision_status,
                    self._matching_stamp(self.fastlio_stream, self.preview_stream)),
            ))
        if self.require_vision_output:
            parts.extend((
                "vision_output_enabled=%s" % self.output_enabled,
                "vision_output=count:%d rate:%.2f issue:%s" % (
                    self.output_stream.count, self.output_stream.rate_hz(),
                    self.output_stream.last_issue),
                "vision_output_consumer=%s issue:%s" % (
                    self.vision_output_consumer_node,
                    self.vision_output_consumer_issue),
                "preview_output_stamp_match=%s" % (
                    self._matching_stamp(self.preview_stream, self.output_stream)),
            ))
        if self.check_px4_vision_params:
            parts.append("px4_vision_params=%s" % self.px4_params_issue)
        if self.require_timesync:
            parts.append("timesync=%s fresh=%s" % (
                self.timesync_issue,
                self._fresh(self.timesync_receive, self.timesync_timeout)))
        return " | ".join(parts)

    def _stream_evidence(self, stream):
        return {
            "count": stream.count,
            "rate_hz": stream.rate_hz(),
            "last_issue": stream.last_issue,
            "fresh": stream.fresh(self.vision_age_limit),
        }

    def _evidence_payload(self, success, reason):
        return {
            "observer": "local_flight_preflight",
            "success": bool(success),
            "reason": reason,
            "generated_at_wall_time": time.time(),
            "summary": {
                "mavros_connected": None if self.state is None else bool(self.state.connected),
                "mavros_armed": None if self.state is None else bool(self.state.armed),
                "landed_state": None if self.extended_state is None else int(self.extended_state.landed_state),
                "estimator_issue": self.estimator_issue,
                "local_stream": self._stream_evidence(self.local_stream),
                "fastlio_stream": self._stream_evidence(self.fastlio_stream),
                "preview_stream": self._stream_evidence(self.preview_stream),
                "vision_healthy": bool(self.vision_healthy),
                "vision_status": self.vision_status,
                "vision_output_enabled": self.output_enabled,
                "vision_output_stream": self._stream_evidence(self.output_stream),
                "vision_output_consumer_issue": self.vision_output_consumer_issue,
                "setpoint_consumer_issue": self.setpoint_consumer_issue,
                "timesync_issue": self.timesync_issue,
                "px4_params_checked": bool(self.px4_params_checked),
                "px4_params_issue": self.px4_params_issue,
                "px4_param_values": self.px4_param_values,
            },
            "parameters": {
                "require_vision": self.require_vision,
                "require_vision_output": self.require_vision_output,
                "require_timesync": self.require_timesync,
                "require_estimator": self.require_estimator,
                "require_disarmed": self.require_disarmed,
                "require_on_ground": self.require_on_ground,
                "check_px4_vision_params": self.check_px4_vision_params,
                "require_yaw_fusion": self.require_yaw_fusion,
                "require_ev_offsets_zero": self.require_ev_offsets_zero,
                "ev_offset_tolerance_m": self.ev_offset_tolerance_m,
                "require_ev_delay": self.require_ev_delay,
                "expected_ev_delay_ms": self.expected_ev_delay_ms,
                "ev_delay_tolerance_ms": self.ev_delay_tolerance_ms,
                "require_vision_output_consumer": self.require_vision_output_consumer,
                "require_setpoint_consumer": self.require_setpoint_consumer,
                "setpoint_consumer_node": self.setpoint_consumer_node,
                "vision_output_consumer_node": self.vision_output_consumer_node,
            },
        }

    def _write_evidence(self, success, reason):
        if not self.evidence_file:
            return
        path = pathlib.Path(self.evidence_file).expanduser()
        if path.parent:
            path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(self._evidence_payload(success, reason),
                                   indent=2, sort_keys=True),
                        encoding="utf-8")

    def _finish(self, success, reason):
        self._write_evidence(success, reason)
        self.exit_code = 0 if success else 2
        level = rospy.loginfo if success else rospy.logerr
        level("%s: %s\n%s", "PASS" if success else "FAIL", reason, self._summary())
        rospy.signal_shutdown(reason)

    def _tick(self, _event):
        mavros_ready = self._mavros_ready()
        vision_ready = self._vision_ready()
        core_ready = mavros_ready and vision_ready
        if core_ready and self.check_px4_vision_params:
            core_ready = self._check_px4_params()
        if core_ready:
            if self.observation_ready_since is None:
                self.observation_ready_since = time.monotonic()
            elif time.monotonic() - self.observation_ready_since >= self.observe_seconds:
                self._finish(True, "local_flight_preflight_passed")
                return
        else:
            self.observation_ready_since = None
        if time.monotonic() - self.started >= self.startup_timeout:
            self._finish(False, "local_flight_preflight_timeout")


if __name__ == "__main__":
    rospy.init_node("local_flight_preflight", anonymous=True)
    try:
        checker = LocalFlightPreflight()
        rospy.loginfo("Read-only local-flight preflight started; no MAVROS commands will be sent")
        rospy.spin()
        sys.exit(checker.exit_code)
    except Exception as exc:
        rospy.logerr("FAIL: local-flight preflight initialization: %s", exc)
        sys.exit(3)
