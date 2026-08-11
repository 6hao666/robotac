#!/usr/bin/env python3
"""AprilTag-guided payload mission for Robotac.

Mission contract, in the body frame captured at mission start:

* +x points toward the aircraft nose.
* +y points toward the aircraft right side.
* +z points up.

The node is deliberately service-started.  Launching it does not arm, change
mode, publish FCU setpoints, or command the servo until
``/robotac/tag_payload_mission/start`` succeeds.  ``enable_control`` gates all
FCU-bound setpoints; ``auto_mode``/``auto_arm``/``auto_land`` gate the active
PX4 service calls; and ``enable_payload`` gates the servo command.
"""

import collections
import hashlib
import json
import math
import time

import rospy
import tf
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import ExtendedState, PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, TriggerResponse
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class MissionState(object):
    IDLE = "IDLE"
    PRESTREAM = "PRESTREAM"
    WAIT_OFFBOARD = "WAIT_OFFBOARD"
    WAIT_ARMED = "WAIT_ARMED"
    TAKEOFF = "TAKEOFF"
    GOTO_FIRST = "GOTO_FIRST"
    GOTO_DROP_SCAN = "GOTO_DROP_SCAN"
    DROP_STABILIZE = "DROP_STABILIZE"
    DROP_TAG_SCAN = "DROP_TAG_SCAN"
    GOTO_DROP_TAG = "GOTO_DROP_TAG"
    PAYLOAD_WAIT = "PAYLOAD_WAIT"
    PAYLOAD_OPEN = "PAYLOAD_OPEN"
    RETURN_FIRST = "RETURN_FIRST"
    RETURN_HOME = "RETURN_HOME"
    LAND_TAG_SCAN = "LAND_TAG_SCAN"
    GOTO_LAND_TAG = "GOTO_LAND_TAG"
    LANDING = "LANDING"
    WAIT_LAND = "WAIT_LAND"
    COMPLETE = "COMPLETE"
    ABORT = "ABORT"


def _as_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def _finite_float(value, name):
    result = float(value)
    if not math.isfinite(result):
        raise ValueError("%s must be finite" % name)
    return result


def _angle_error(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


class TagStabilityTracker(object):
    def __init__(self, required_samples, jump_threshold, max_age):
        self.required_samples = int(required_samples)
        self.jump_threshold = float(jump_threshold)
        self.max_age = float(max_age)
        self.reset("not_started")

    def reset(self, reason):
        self.reason = reason
        self.target_id = None
        self.count = 0
        self.last_pose = None
        self.last_stamp = None
        self.samples = collections.deque(maxlen=max(1, self.required_samples))
        self.confirmed_pose = None
        self.confirmed_stamp = None
        self.last_jump = 0.0

    def start(self, tag_id):
        self.reset("waiting_for_tag_%d" % int(tag_id))
        self.target_id = int(tag_id)

    def update(self, pose_stamped):
        now = rospy.Time.now()
        stamp = pose_stamped.header.stamp
        if stamp.is_zero():
            stamp = now
        age = (now - stamp).to_sec()
        if age > self.max_age:
            self.reset("tag_pose_stale_age=%.3f" % age)
            return False

        p = pose_stamped.pose.position
        current = (float(p.x), float(p.y), float(p.z))
        if self.last_pose is None:
            self.count = 1
            self.samples.clear()
            self.samples.append(current)
            self.last_pose = current
            self.last_stamp = stamp
            self.reason = "collecting_1/%d" % self.required_samples
            return False

        jump = math.sqrt(sum((current[i] - self.last_pose[i]) ** 2 for i in range(3)))
        self.last_jump = jump
        if jump > self.jump_threshold:
            self.count = 1
            self.samples.clear()
            self.samples.append(current)
            self.last_pose = current
            self.last_stamp = stamp
            self.reason = "jump_reset_%.3f" % jump
            return False

        self.count += 1
        self.samples.append(current)
        self.last_pose = current
        self.last_stamp = stamp
        self.reason = "collecting_%d/%d_jump=%.3f" % (
            self.count, self.required_samples, jump)
        if self.count >= self.required_samples:
            n = float(len(self.samples))
            avg = [sum(sample[i] for sample in self.samples) / n for i in range(3)]
            confirmed = PoseStamped()
            confirmed.header = pose_stamped.header
            confirmed.header.stamp = stamp
            confirmed.pose.position.x = avg[0]
            confirmed.pose.position.y = avg[1]
            confirmed.pose.position.z = avg[2]
            confirmed.pose.orientation = pose_stamped.pose.orientation
            self.confirmed_pose = confirmed
            self.confirmed_stamp = stamp
            self.reason = "confirmed_%d_samples" % self.required_samples
            return True
        return False


class TagPayloadMission(object):
    def __init__(self):
        # ``tag_payload_mission.launch`` historically loaded
        # ``config/flight/tag_payload_mission.yaml`` at the global namespace,
        # creating ``/tag_payload_mission``.  A private ``~tag_payload_mission``
        # lookup resolves to ``/tag_payload_mission/tag_payload_mission`` and
        # therefore silently fell back to code defaults.  Read the private key
        # first, then the existing global key so field edits in the YAML really
        # drive the aircraft mission.
        if rospy.has_param("~tag_payload_mission"):
            self.config = rospy.get_param("~tag_payload_mission")
            self.config_source = "private"
        else:
            self.config = rospy.get_param("/tag_payload_mission", {})
            self.config_source = "global"
        if not isinstance(self.config, dict):
            raise ValueError("~tag_payload_mission must be a YAML dictionary")

        self.enable_control = _as_bool(rospy.get_param("~enable_control", False))
        self.auto_mode = _as_bool(rospy.get_param("~auto_mode", False))
        self.auto_arm = _as_bool(rospy.get_param("~auto_arm", False))
        self.auto_land = _as_bool(rospy.get_param("~auto_land", False))
        self.enable_payload = _as_bool(rospy.get_param("~enable_payload", False))
        if (self.auto_mode or self.auto_arm or self.auto_land) and not self.enable_control:
            raise ValueError("auto_mode/auto_arm/auto_land require enable_control=true")
        if self.enable_payload and not self.enable_control:
            raise ValueError("enable_payload requires enable_control=true")

        self.control_rate_hz = _finite_float(self._cfg("control_rate_hz", 20.0), "control_rate_hz")
        self.takeoff_height = _finite_float(self._cfg("takeoff_height", 0.3), "takeoff_height")
        self.default_yaw_deg = _finite_float(self._cfg("yaw_deg", 0.0), "yaw_deg")
        self.first_xy = self._cfg_point("first_point", {"x": 2.0, "y": 2.0, "z": 0.3})
        self.drop_scan_xy = self._cfg_point("drop_scan_point", {"x": 4.0, "y": 0.0, "z": 0.8})
        self.return_first_xy = self._cfg_point("return_first_point", {"x": 2.0, "y": 2.0, "z": 0.3})
        self.home_xy = self._cfg_point("home_point", {"x": 0.0, "y": 0.0, "z": 0.3})
        self.drop_stabilize_seconds = _finite_float(
            self._cfg("drop_stabilize_seconds", 2.0), "drop_stabilize_seconds")
        # User wrote “等待 1m”; this is treated as a configurable 1 second wait.
        self.payload_wait_seconds = _finite_float(
            self._cfg("payload_wait_seconds", 1.0), "payload_wait_seconds")
        self.payload_settle_seconds = _finite_float(
            self._cfg("payload_settle_seconds", 1.0), "payload_settle_seconds")
        self.tag_overfly_height = _finite_float(
            self._cfg("tag_overfly_height", 0.3), "tag_overfly_height")
        self.land_switch_height = _finite_float(
            self._cfg("land_switch_height", 0.3), "land_switch_height")
        self.land_descent_rate = _finite_float(
            self._cfg("land_descent_rate", 0.25), "land_descent_rate")
        self.auto_land_required = _as_bool(self._cfg("auto_land_required", True))
        self.skip_tag_alignment = _as_bool(self._cfg("skip_tag_alignment", False))

        self.position_tolerance = _finite_float(
            self._cfg("position_tolerance", 0.18), "position_tolerance")
        self.yaw_tolerance = math.radians(_finite_float(
            self._cfg("yaw_tolerance_deg", 12.0), "yaw_tolerance_deg"))
        self.hold_seconds = _finite_float(self._cfg("hold_seconds", 1.0), "hold_seconds")
        self.prestream_seconds = _finite_float(
            self._cfg("prestream_seconds", 5.0), "prestream_seconds")
        self.offboard_timeout = _finite_float(self._cfg("offboard_timeout", 15.0), "offboard_timeout")
        self.arming_timeout = _finite_float(self._cfg("arming_timeout", 15.0), "arming_timeout")
        self.takeoff_timeout = _finite_float(self._cfg("takeoff_timeout", 45.0), "takeoff_timeout")
        self.segment_timeout = _finite_float(self._cfg("segment_timeout", 60.0), "segment_timeout")
        self.tag_scan_timeout = _finite_float(self._cfg("tag_scan_timeout", 45.0), "tag_scan_timeout")
        self.mission_timeout = _finite_float(self._cfg("mission_timeout", 600.0), "mission_timeout")
        self.local_pose_timeout = _finite_float(
            self._cfg("local_pose_timeout", 0.5), "local_pose_timeout")
        self.state_timeout = _finite_float(self._cfg("state_timeout", 3.0), "state_timeout")
        self.extended_state_timeout = _finite_float(
            self._cfg("extended_state_timeout", 1.0), "extended_state_timeout")
        self.max_local_position_speed = _finite_float(
            self._cfg("max_local_position_speed", 8.0), "max_local_position_speed")
        self.landing_timeout = _finite_float(self._cfg("landing_timeout", 60.0), "landing_timeout")

        self.drop_tag_id = int(self._cfg("drop_tag_id", 1))
        self.land_tag_id = int(self._cfg("land_tag_id", 0))
        self.tag_stable_samples = int(self._cfg("tag_stable_samples", 15))
        self.tag_jump_threshold = _finite_float(
            self._cfg("tag_jump_threshold", 0.05), "tag_jump_threshold")
        self.tag_max_age = _finite_float(self._cfg("tag_max_age", 0.5), "tag_max_age")
        self.tag_target_frame = str(self._cfg("tag_target_frame", "")).strip()
        self.tf_timeout = _finite_float(self._cfg("tf_timeout", 0.15), "tf_timeout")

        self.land_mode = str(self._cfg("land_mode", "AUTO.LAND"))
        self.abort_action = str(self._cfg("abort_action", "release")).strip().lower()
        if self.abort_action not in ("release", "hold", "land"):
            raise ValueError("abort_action must be release, hold, or land")

        self.local_odom_topic = rospy.get_param("~local_odom_topic", "/mavros/local_position/odom")
        self.state_topic = rospy.get_param("~state_topic", "/mavros/state")
        self.extended_state_topic = rospy.get_param(
            "~extended_state_topic", "/mavros/extended_state")
        self.setpoint_topic = rospy.get_param("~setpoint_topic", "/mavros/setpoint_raw/local")
        self.tag_detections_topic = rospy.get_param("~tag_detections_topic", "/tag_detections")
        self.payload_topic = rospy.get_param("~payload_topic", "/robotac_servo/control")
        self.payload_status_topic = rospy.get_param("~payload_status_topic", "/robotac_servo/status")
        self.payload_required_connection = _as_bool(rospy.get_param("~payload_required_connection", True))
        self.payload_require_ack = _as_bool(rospy.get_param("~payload_require_ack", True))
        self.payload_ack_timeout = _finite_float(rospy.get_param("~payload_ack_timeout", 1.0),
                                                "payload_ack_timeout")
        self.require_setpoint_consumer = _as_bool(rospy.get_param("~require_setpoint_consumer", True))
        self.setpoint_consumer_node = rospy.get_param("~setpoint_consumer_node", "/mavros")
        self.consumer_check_interval = _finite_float(
            rospy.get_param("~consumer_check_interval", 0.5), "consumer_check_interval")

        self.state = MissionState.IDLE
        self.state_started = time.monotonic()
        self.mission_started = None
        self.last_error = "idle"
        self.origin = None
        self.origin_yaw = 0.0
        self.target = None
        self.reached_since = None
        self.drop_tag_pose = None
        self.land_tag_pose = None
        self.scan_kind = None
        self.landing_target = None
        self.auto_land_request_started = None
        self.last_setpoint = None
        self.last_setpoint_wall = None
        self.control_tx_enabled = False
        self.last_consumer_check_wall = 0.0
        self.last_consumer_issue = None
        self.payload_expected_state = None
        self.payload_command_time = None
        self.payload_ack_state = None
        self.payload_ack_success = None
        self.payload_ack_sequence = 0
        self.payload_command_sequence = None
        self.payload_ack_boot_id = None
        self.payload_command_boot_id = None
        self.payload_ack_time = None
        self.payload_state = "disabled" if not self.enable_payload else "uncommanded"
        self.local_odom = None
        self.local_receive_time = None
        self.previous_local_pose = None
        self.local_rejection_reason = None
        self.fcu = State()
        self.fcu_receive_time = None
        self.extended = ExtendedState()
        self.extended_receive_time = None
        self.tag_tracker = TagStabilityTracker(
            self.tag_stable_samples, self.tag_jump_threshold, self.tag_max_age)
        self.tf_listener = tf.TransformListener()

        self.mode_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.arm_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.setpoint_pub = (rospy.Publisher(self.setpoint_topic, PositionTarget, queue_size=20)
                             if self.enable_control else None)
        self.preview_pub = rospy.Publisher(
            "/robotac/tag_payload_mission/setpoint_preview", PositionTarget, queue_size=10)
        self.status_pub = rospy.Publisher(
            "/robotac/tag_payload_mission/status", String, queue_size=10)
        self.active_pub = rospy.Publisher(
            "/robotac/tag_payload_mission/active", Bool, queue_size=1, latch=True)
        self.route_pub = rospy.Publisher(
            "/robotac/tag_payload_mission/route_manifest", String, queue_size=1, latch=True)
        self.confirmed_tag_pub = rospy.Publisher(
            "/robotac/tag_payload_mission/confirmed_tag_pose", PoseStamped, queue_size=2, latch=True)
        self.payload_pub = (rospy.Publisher(self.payload_topic, Bool, queue_size=1)
                            if self.enable_payload else None)

        rospy.Subscriber(self.state_topic, State, self._state_cb, queue_size=10)
        rospy.Subscriber(self.extended_state_topic, ExtendedState, self._extended_state_cb, queue_size=10)
        rospy.Subscriber(self.local_odom_topic, Odometry, self._local_odom_cb, queue_size=20)
        rospy.Subscriber(self.tag_detections_topic, AprilTagDetectionArray,
                         self._tag_detections_cb, queue_size=5)
        rospy.Subscriber(self.payload_status_topic, String, self._payload_status_cb, queue_size=5)
        rospy.Service("/robotac/tag_payload_mission/start", Trigger, self._start_cb)
        rospy.Service("/robotac/tag_payload_mission/abort", Trigger, self._abort_cb)
        rospy.Service("/robotac/tag_payload_mission/reset", Trigger, self._reset_cb)
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(1.0, self.control_rate_hz)), self._tick)

        self._publish_route_manifest()
        rospy.logwarn("tag payload mission launched: enable_control=%s auto_mode=%s auto_arm=%s auto_land=%s enable_payload=%s",
                      self.enable_control, self.auto_mode, self.auto_arm,
                      self.auto_land, self.enable_payload)
        rospy.logwarn("tag payload mission config source=%s takeoff_height=%.2f first=(%.2f,%.2f,%.2f) drop=(%.2f,%.2f,%.2f) skip_tag_alignment=%s",
                      self.config_source, self.takeoff_height,
                      self.first_xy["x"], self.first_xy["y"], self.first_xy["z"],
                      self.drop_scan_xy["x"], self.drop_scan_xy["y"], self.drop_scan_xy["z"],
                      getattr(self, "skip_tag_alignment", False))

    def _cfg(self, key, default):
        return self.config.get(key, default)

    def _cfg_point(self, key, default):
        value = self._cfg(key, default)
        if not isinstance(value, dict):
            raise ValueError("%s must be a dictionary with x/y/z" % key)
        return {
            "x": _finite_float(value.get("x", default.get("x", 0.0)), "%s.x" % key),
            "y": _finite_float(value.get("y", default.get("y", 0.0)), "%s.y" % key),
            "z": _finite_float(value.get("z", default.get("z", 0.0)), "%s.z" % key),
            "yaw_deg": _finite_float(value.get("yaw_deg", self.default_yaw_deg), "%s.yaw_deg" % key),
        }

    def _state_cb(self, msg):
        self.fcu = msg
        self.fcu_receive_time = time.monotonic()

    def _extended_state_cb(self, msg):
        self.extended = msg
        self.extended_receive_time = time.monotonic()

    def _local_odom_cb(self, msg):
        now = time.monotonic()
        reason = self._local_rejection(msg, now)
        if reason is not None:
            self.local_rejection_reason = reason
            return
        self.local_odom = msg
        self.local_receive_time = now
        p = msg.pose.pose.position
        self.previous_local_pose = (p.x, p.y, p.z, now)
        self.local_rejection_reason = None

    def _local_rejection(self, msg, now):
        p = msg.pose.pose.position
        values = (p.x, p.y, p.z, msg.pose.pose.orientation.x,
                  msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
                  msg.pose.pose.orientation.w)
        if not all(math.isfinite(float(value)) for value in values):
            return "nonfinite"
        if self.previous_local_pose is not None:
            dt = now - self.previous_local_pose[3]
            if dt > 1.0e-3:
                distance = math.sqrt((p.x - self.previous_local_pose[0]) ** 2 +
                                     (p.y - self.previous_local_pose[1]) ** 2 +
                                     (p.z - self.previous_local_pose[2]) ** 2)
                speed = distance / dt
                if speed > self.max_local_position_speed:
                    return "position_jump_speed:%.2f" % speed
        return None

    def _payload_status_cb(self, msg):
        fields = {}
        for token in msg.data.split():
            if "=" in token:
                key, value = token.split("=", 1)
                fields[key] = value
        state = fields.get("state")
        if state not in ("open", "closed"):
            return
        try:
            sequence = int(fields.get("seq", self.payload_ack_sequence + 1))
        except ValueError:
            sequence = self.payload_ack_sequence + 1
        boot_id = fields.get("boot", "unknown")
        succeeded = fields.get("success", "true").lower() in ("1", "true", "yes", "ok")
        if boot_id != self.payload_ack_boot_id:
            self.payload_ack_boot_id = boot_id
            self.payload_ack_sequence = 0
        if sequence <= self.payload_ack_sequence:
            return
        self.payload_ack_state = state
        self.payload_ack_success = succeeded
        self.payload_ack_sequence = sequence
        self.payload_ack_time = time.monotonic()
        if self.payload_expected_state == state and succeeded:
            self.payload_state = state

    def _tag_detections_cb(self, msg):
        if self.state not in (MissionState.DROP_TAG_SCAN, MissionState.LAND_TAG_SCAN):
            return
        target_id = self.drop_tag_id if self.state == MissionState.DROP_TAG_SCAN else self.land_tag_id
        for detection in msg.detections:
            if target_id not in [int(item) for item in detection.id]:
                continue
            local_pose = self._detection_to_local_pose(detection, msg.header)
            if local_pose is None:
                return
            if self.tag_tracker.update(local_pose):
                self.confirmed_tag_pub.publish(self.tag_tracker.confirmed_pose)
            return

    def _detection_to_local_pose(self, detection, array_header):
        pose = PoseStamped()
        pose.header = detection.pose.header
        if not pose.header.frame_id:
            pose.header.frame_id = array_header.frame_id
        if pose.header.stamp.is_zero():
            pose.header.stamp = array_header.stamp
        pose.pose = detection.pose.pose.pose

        target_frame = self.tag_target_frame or self._local_frame()
        if not target_frame:
            self.tag_tracker.reason = "local_frame_unavailable"
            return None
        if not pose.header.frame_id:
            self.tag_tracker.reason = "tag_pose_frame_empty"
            return None
        if pose.header.frame_id == target_frame:
            return pose
        try:
            self.tf_listener.waitForTransform(
                target_frame, pose.header.frame_id, pose.header.stamp,
                rospy.Duration(self.tf_timeout))
            return self.tf_listener.transformPose(target_frame, pose)
        except Exception as first_error:
            try:
                latest = PoseStamped()
                latest.header = pose.header
                latest.header.stamp = rospy.Time(0)
                latest.pose = pose.pose
                return self.tf_listener.transformPose(target_frame, latest)
            except Exception as second_error:
                self.tag_tracker.reason = "tf_%s_to_%s_failed:%s/%s" % (
                    pose.header.frame_id, target_frame, first_error, second_error)
                return None

    def _local_frame(self):
        if self.local_odom is not None and self.local_odom.header.frame_id:
            return self.local_odom.header.frame_id
        return "map"

    def _state_fresh(self):
        return self.fcu_receive_time is not None and time.monotonic() - self.fcu_receive_time <= self.state_timeout

    def _extended_state_fresh(self):
        return (self.extended_receive_time is not None and
                time.monotonic() - self.extended_receive_time <= self.extended_state_timeout)

    def _local_fresh(self):
        return self.local_receive_time is not None and time.monotonic() - self.local_receive_time <= self.local_pose_timeout

    def _setpoint_consumer_present(self):
        now = time.monotonic()
        if now - self.last_consumer_check_wall < self.consumer_check_interval:
            return self.last_consumer_issue is None
        self.last_consumer_check_wall = now
        if self.setpoint_pub is None:
            self.last_consumer_issue = None
            return True
        if self.setpoint_pub.get_num_connections() < 1:
            self.last_consumer_issue = "setpoint_no_subscribers"
            return False
        try:
            info = rospy.get_published_topics()
        except Exception:
            info = []
        # The publisher connection count is the authoritative fast check.  The
        # optional graph scan simply preserves a useful status string.
        _ = info
        self.last_consumer_issue = None
        return True

    def _start_cb(self, _request):
        if self.state not in (MissionState.IDLE, MissionState.COMPLETE, MissionState.ABORT):
            return TriggerResponse(False, "mission_already_active:%s" % self.state)
        if not self._state_fresh():
            return TriggerResponse(False, "mavros_state_stale")
        if not self.fcu.connected:
            return TriggerResponse(False, "mavros_not_connected")
        if self.fcu.armed:
            return TriggerResponse(False, "refuse_start_while_armed")
        if not self._extended_state_fresh():
            return TriggerResponse(False, "mavros_extended_state_stale")
        if self.extended.landed_state == ExtendedState.LANDED_STATE_IN_AIR:
            return TriggerResponse(False, "vehicle_reports_in_air")
        if not self._local_fresh():
            return TriggerResponse(False, "local_position_%s" % (self.local_rejection_reason or "stale"))
        if self.enable_control and self.require_setpoint_consumer and not self._setpoint_consumer_present():
            return TriggerResponse(False, self.last_consumer_issue or "setpoint_consumer_unavailable")
        if self.enable_control and self.auto_land_required and not self.auto_land:
            return TriggerResponse(False, "auto_land_required_but_disabled")
        if (self.enable_payload and self.payload_required_connection and
                (self.payload_pub is None or self.payload_pub.get_num_connections() < 1)):
            return TriggerResponse(False, "payload_subscriber_unavailable")
        if self.enable_payload and self.payload_require_ack and self.payload_ack_time is None:
            return TriggerResponse(False, "payload_status_unavailable")

        p = self.local_odom.pose.pose.position
        q = self.local_odom.pose.pose.orientation
        self.origin = (float(p.x), float(p.y), float(p.z))
        _, _, self.origin_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.target = (self.origin[0], self.origin[1], self.origin[2], self.origin_yaw)
        self.reached_since = None
        self.drop_tag_pose = None
        self.land_tag_pose = None
        self.scan_kind = None
        self.landing_target = None
        self.auto_land_request_started = None
        self.mission_started = time.monotonic()
        self.control_tx_enabled = self.enable_control
        self.payload_state = "disabled" if not self.enable_payload else "ready"
        self.last_error = "idle"
        self._enter(MissionState.PRESTREAM)
        return TriggerResponse(True, "mission_started_control_enabled=%s" % self.enable_control)

    def _abort_cb(self, _request):
        if self.state == MissionState.IDLE:
            return TriggerResponse(False, "mission_idle")
        self._abort("operator_abort")
        return TriggerResponse(True, "mission_aborted")

    def _reset_cb(self, _request):
        if self._state_fresh() and self.fcu.armed:
            return TriggerResponse(False, "refuse_reset_while_armed")
        self.state = MissionState.IDLE
        self.state_started = time.monotonic()
        self.mission_started = None
        self.control_tx_enabled = False
        self.last_error = "reset"
        self._publish_status()
        return TriggerResponse(True, "mission_reset")

    def _abort(self, reason):
        if self.state == MissionState.ABORT:
            return
        self.last_error = reason
        self.control_tx_enabled = False
        self._enter(MissionState.ABORT)
        if self.abort_action == "land" and self.enable_control and self.auto_land:
            self._request_mode(self.land_mode)
        rospy.logerr("tag payload mission aborted: %s", reason)

    def _enter(self, state):
        self.state = state
        self.state_started = time.monotonic()
        self.reached_since = None
        if state == MissionState.DROP_TAG_SCAN:
            self.scan_kind = "drop"
            self.tag_tracker.start(self.drop_tag_id)
        elif state == MissionState.LAND_TAG_SCAN:
            self.scan_kind = "land"
            self.tag_tracker.start(self.land_tag_id)
        rospy.loginfo("tag payload mission state -> %s", state)

    def _body_right_target(self, point):
        if self.origin is None:
            return None
        yaw = self.origin_yaw
        forward = (math.cos(yaw), math.sin(yaw))
        right = (math.sin(yaw), -math.cos(yaw))
        x = self.origin[0] + point["x"] * forward[0] + point["y"] * right[0]
        y = self.origin[1] + point["x"] * forward[1] + point["y"] * right[1]
        z = self.origin[2] + point["z"]
        target_yaw = self.origin_yaw + math.radians(point.get("yaw_deg", self.default_yaw_deg))
        return (x, y, z, target_yaw)

    def _tag_overfly_target(self, pose_stamped):
        if pose_stamped is None or self.origin is None:
            return None
        p = pose_stamped.pose.position
        return (float(p.x), float(p.y), self.origin[2] + self.tag_overfly_height, self.origin_yaw)

    def _current_position(self):
        if not self._local_fresh():
            return None
        p = self.local_odom.pose.pose.position
        q = self.local_odom.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return (float(p.x), float(p.y), float(p.z), yaw)

    def _at_target(self, target):
        current = self._current_position()
        if current is None or target is None:
            return False
        distance = math.sqrt(sum((current[i] - target[i]) ** 2 for i in range(3)))
        yaw_error = abs(_angle_error(current[3], target[3]))
        return distance <= self.position_tolerance and yaw_error <= self.yaw_tolerance

    def _hold_reached(self, target, hold_seconds=None):
        if hold_seconds is None:
            hold_seconds = self.hold_seconds
        if self._at_target(target):
            if self.reached_since is None:
                self.reached_since = time.monotonic()
            return time.monotonic() - self.reached_since >= hold_seconds
        self.reached_since = None
        return False

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
        if target is None:
            return
        msg = self._make_setpoint(target)
        self.preview_pub.publish(msg)
        if self.enable_control and self.control_tx_enabled and self.setpoint_pub is not None:
            self.setpoint_pub.publish(msg)
            self.last_setpoint_wall = time.monotonic()
        self.last_setpoint = msg

    def _request_mode(self, mode):
        try:
            response = self.mode_srv(base_mode=0, custom_mode=mode)
            if not response.mode_sent:
                rospy.logwarn("PX4 rejected mode request: %s", mode)
            return bool(response.mode_sent)
        except rospy.ServiceException as exc:
            rospy.logerr("mode request failed: %s", exc)
            return False

    def _request_arm(self, value=True):
        try:
            response = self.arm_srv(value=bool(value))
            if not response.success:
                rospy.logwarn("PX4 rejected arm=%s result=%s", value, response.result)
            return bool(response.success)
        except rospy.ServiceException as exc:
            rospy.logerr("arm request failed: %s", exc)
            return False

    def _publish_payload(self, is_open):
        if not self.enable_payload:
            self.payload_state = "simulated_%s" % ("open" if is_open else "closed")
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
        self.payload_pub.publish(Bool(data=bool(is_open)))
        return True

    def _payload_acknowledged(self):
        if not self.enable_payload or not self.payload_require_ack:
            return True
        return (self.payload_expected_state is not None and
                self.payload_ack_state == self.payload_expected_state and
                self.payload_ack_success is True and
                self.payload_command_sequence is not None and
                self.payload_ack_sequence > self.payload_command_sequence and
                self.payload_ack_boot_id == self.payload_command_boot_id and
                self.payload_ack_time is not None and
                self.payload_command_time is not None and
                self.payload_ack_time >= self.payload_command_time)

    def _payload_timed_out(self):
        return (self.enable_payload and self.payload_require_ack and
                self.payload_command_time is not None and
                time.monotonic() - self.payload_command_time > self.payload_ack_timeout)

    def _ground_confirmed(self):
        return (self._extended_state_fresh() and
                self.extended.landed_state == ExtendedState.LANDED_STATE_ON_GROUND)

    def _active_health_issue(self):
        if self.state in (MissionState.IDLE, MissionState.COMPLETE, MissionState.ABORT):
            return None
        if self.mission_started and time.monotonic() - self.mission_started > self.mission_timeout:
            return "mission_timeout"
        if not self._state_fresh():
            return "mavros_state_stale"
        if not self.fcu.connected:
            return "mavros_disconnected"
        if not self._extended_state_fresh():
            return "mavros_extended_state_stale"
        if not self._local_fresh():
            return "local_position_%s" % (self.local_rejection_reason or "stale")
        if self.enable_control and self.control_tx_enabled and self.require_setpoint_consumer:
            if not self._setpoint_consumer_present():
                return self.last_consumer_issue or "setpoint_consumer_unavailable"
        return None

    def _tick(self, _event):
        issue = self._active_health_issue()
        if issue is not None:
            self._abort(issue)
            self._publish_status()
            return

        if self.state in (MissionState.TAKEOFF, MissionState.GOTO_FIRST,
                          MissionState.GOTO_DROP_SCAN, MissionState.GOTO_DROP_TAG,
                          MissionState.RETURN_FIRST, MissionState.RETURN_HOME,
                          MissionState.GOTO_LAND_TAG, MissionState.LANDING):
            if self.enable_control and self.fcu.armed and self.fcu.mode != "OFFBOARD":
                if self.state == MissionState.LANDING and self.fcu.mode == self.land_mode:
                    pass
                else:
                    self._abort("offboard_mode_lost")

        now = time.monotonic()
        if self.state == MissionState.IDLE:
            pass
        elif self.state == MissionState.PRESTREAM:
            self._publish_setpoint(self.target)
            if now - self.state_started >= self.prestream_seconds:
                self._enter(MissionState.WAIT_OFFBOARD)
        elif self.state == MissionState.WAIT_OFFBOARD:
            self._publish_setpoint(self.target)
            if self.fcu.mode == "OFFBOARD":
                self._enter(MissionState.WAIT_ARMED)
            elif now - self.state_started > self.offboard_timeout:
                self._abort("offboard_timeout")
            elif self.auto_mode:
                self._request_mode("OFFBOARD")
        elif self.state == MissionState.WAIT_ARMED:
            self._publish_setpoint(self.target)
            if self.fcu.armed:
                self._enter(MissionState.TAKEOFF)
            elif now - self.state_started > self.arming_timeout:
                self._abort("arming_timeout")
            elif self.auto_arm:
                self._request_arm(True)
        elif self.state == MissionState.TAKEOFF:
            self.target = (self.origin[0], self.origin[1], self.origin[2] + self.takeoff_height, self.origin_yaw)
            self._publish_setpoint(self.target)
            if self._hold_reached(self.target):
                self._enter(MissionState.GOTO_FIRST)
            elif now - self.state_started > self.takeoff_timeout:
                self._abort("takeoff_timeout")
        elif self.state == MissionState.GOTO_FIRST:
            self.target = self._body_right_target(self.first_xy)
            self._publish_setpoint(self.target)
            if self._hold_reached(self.target):
                self._enter(MissionState.GOTO_DROP_SCAN)
            elif now - self.state_started > self.segment_timeout:
                self._abort("goto_first_timeout")
        elif self.state == MissionState.GOTO_DROP_SCAN:
            self.target = self._body_right_target(self.drop_scan_xy)
            self._publish_setpoint(self.target)
            if self._hold_reached(self.target):
                self._enter(MissionState.DROP_STABILIZE)
            elif now - self.state_started > self.segment_timeout:
                self._abort("goto_drop_scan_timeout")
        elif self.state == MissionState.DROP_STABILIZE:
            self._publish_setpoint(self.target)
            if now - self.state_started >= self.drop_stabilize_seconds:
                if self.skip_tag_alignment:
                    self._enter(MissionState.PAYLOAD_WAIT)
                else:
                    self._enter(MissionState.DROP_TAG_SCAN)
        elif self.state == MissionState.DROP_TAG_SCAN:
            self._publish_setpoint(self.target)
            if self.tag_tracker.confirmed_pose is not None:
                self.drop_tag_pose = self.tag_tracker.confirmed_pose
                self.target = self._tag_overfly_target(self.drop_tag_pose)
                self._enter(MissionState.GOTO_DROP_TAG)
            elif now - self.state_started > self.tag_scan_timeout:
                self._abort("drop_tag_scan_timeout:%s" % self.tag_tracker.reason)
        elif self.state == MissionState.GOTO_DROP_TAG:
            self._publish_setpoint(self.target)
            if self._hold_reached(self.target):
                self._enter(MissionState.PAYLOAD_WAIT)
            elif now - self.state_started > self.segment_timeout:
                self._abort("goto_drop_tag_timeout")
        elif self.state == MissionState.PAYLOAD_WAIT:
            self._publish_setpoint(self.target)
            if now - self.state_started >= self.payload_wait_seconds:
                if not self._publish_payload(True):
                    self._abort("payload_open_failed")
                else:
                    self._enter(MissionState.PAYLOAD_OPEN)
        elif self.state == MissionState.PAYLOAD_OPEN:
            self._publish_setpoint(self.target)
            if self._payload_acknowledged() and now - self.state_started >= self.payload_settle_seconds:
                self._enter(MissionState.RETURN_FIRST)
            elif self._payload_timed_out():
                self._abort("payload_ack_timeout")
        elif self.state == MissionState.RETURN_FIRST:
            self.target = self._body_right_target(self.return_first_xy)
            self._publish_setpoint(self.target)
            if self._hold_reached(self.target):
                self._enter(MissionState.RETURN_HOME)
            elif now - self.state_started > self.segment_timeout:
                self._abort("return_first_timeout")
        elif self.state == MissionState.RETURN_HOME:
            self.target = self._body_right_target(self.home_xy)
            self._publish_setpoint(self.target)
            if self._hold_reached(self.target):
                if self.skip_tag_alignment:
                    self._enter(MissionState.LANDING if self.auto_land else MissionState.WAIT_LAND)
                else:
                    self._enter(MissionState.LAND_TAG_SCAN)
            elif now - self.state_started > self.segment_timeout:
                self._abort("return_home_timeout")
        elif self.state == MissionState.LAND_TAG_SCAN:
            self._publish_setpoint(self.target)
            if self.tag_tracker.confirmed_pose is not None:
                self.land_tag_pose = self.tag_tracker.confirmed_pose
                self.target = self._tag_overfly_target(self.land_tag_pose)
                self._enter(MissionState.GOTO_LAND_TAG)
            elif now - self.state_started > self.tag_scan_timeout:
                self._abort("land_tag_scan_timeout:%s" % self.tag_tracker.reason)
        elif self.state == MissionState.GOTO_LAND_TAG:
            self._publish_setpoint(self.target)
            if self._hold_reached(self.target):
                self._enter(MissionState.LANDING if self.auto_land else MissionState.WAIT_LAND)
            elif now - self.state_started > self.segment_timeout:
                self._abort("goto_land_tag_timeout")
        elif self.state == MissionState.WAIT_LAND:
            self._publish_setpoint(self.target)
        elif self.state == MissionState.LANDING:
            if now - self.state_started > self.landing_timeout:
                self._abort("landing_timeout")
            else:
                self._tick_landing()
        elif self.state == MissionState.ABORT:
            if self.abort_action == "hold" and self.target is not None:
                self._publish_setpoint(self.target)
        elif self.state == MissionState.COMPLETE:
            pass
        self._publish_status()

    def _tick_landing(self):
        if self._ground_confirmed():
            self.control_tx_enabled = False
            self._enter(MissionState.COMPLETE)
            return
        if self.landing_target is None:
            current = self._current_position()
            if current is None:
                return
            self.landing_target = current
        switch_z = self.origin[2] + self.land_switch_height
        if self.fcu.mode != self.land_mode:
            next_z = max(switch_z, self.landing_target[2] - self.land_descent_rate / max(1.0, self.control_rate_hz))
            self.landing_target = (self.landing_target[0], self.landing_target[1], next_z, self.landing_target[3])
            self.target = self.landing_target
            self._publish_setpoint(self.target)
            if next_z <= switch_z + 1.0e-3 and self.enable_control:
                if self.auto_land_request_started is None:
                    self.auto_land_request_started = time.monotonic()
                self._request_mode(self.land_mode)

    def _publish_route_manifest(self):
        manifest = {
            "frame": "robotac_start_body_fru",
            "axis_contract": "+x nose/front, +y aircraft right, +z up",
            "takeoff_height": self.takeoff_height,
            "first_point": self.first_xy,
            "drop_scan_point": self.drop_scan_xy,
            "drop_tag_id": self.drop_tag_id,
            "land_tag_id": self.land_tag_id,
            "tag_stable_samples": self.tag_stable_samples,
            "tag_jump_threshold": self.tag_jump_threshold,
            "tag_overfly_height": self.tag_overfly_height,
            "skip_tag_alignment": self.skip_tag_alignment,
            "return_first_point": self.return_first_xy,
            "home_point": self.home_xy,
            "payload_wait_seconds": self.payload_wait_seconds,
        }
        text = json.dumps(manifest, sort_keys=True)
        fingerprint = hashlib.sha256(text.encode("utf-8")).hexdigest()
        self.route_pub.publish(String(data="fingerprint=%s manifest=%s" % (fingerprint, text)))

    def _publish_status(self):
        active = self.state not in (MissionState.IDLE, MissionState.COMPLETE, MissionState.ABORT)
        self.active_pub.publish(Bool(data=active))
        current = self._current_position()
        current_text = "none" if current is None else "(%.3f,%.3f,%.3f,%.3f)" % current
        target_text = "none" if self.target is None else "(%.3f,%.3f,%.3f,%.3f)" % self.target
        self.status_pub.publish(String(data=(
            "state=%s connected=%s armed=%s mode=%s active=%s tx=%s payload=%s "
            "scan=%s tag_status=%s tag_count=%d/%d current=%s target=%s error=%s stamp=%.9f" %
            (self.state, self.fcu.connected, self.fcu.armed, self.fcu.mode, active,
             self.control_tx_enabled, self.payload_state, self.scan_kind or "none",
             self.tag_tracker.reason, self.tag_tracker.count, self.tag_stable_samples,
             current_text, target_text, self.last_error, rospy.Time.now().to_sec()))))


def main():
    rospy.init_node("tag_payload_mission")
    try:
        TagPayloadMission()
    except Exception as exc:
        rospy.logfatal("tag payload mission startup failed: %s", exc)
        raise
    rospy.spin()


if __name__ == "__main__":
    main()
