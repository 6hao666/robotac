#!/usr/bin/env python3
"""Offline MAVROS/PX4 surrogate for the local waypoint controller.

This node never opens a serial device or starts MAVROS.  It provides the
minimal topic/service contract expected by ``local_waypoint_flight.py`` and
moves a simulated vehicle toward received local setpoints.  It is intended for
the integration test that exercises takeoff, a sequence of local ENU waypoints,
and the AUTO.LAND hand-off.
"""

import math
import time

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from mavros_msgs.msg import EstimatorStatus, ExtendedState, PositionTarget, State, TimesyncStatus
from mavros_msgs.srv import CommandBool, CommandBoolResponse, SetMode, SetModeResponse
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from tf.transformations import quaternion_from_euler


class ClosedLoopMavrosSim(object):
    def __init__(self):
        self.rate_hz = float(rospy.get_param("~rate_hz", 40.0))
        self.horizontal_speed = float(rospy.get_param("~horizontal_speed", 1.0))
        self.vertical_speed = float(rospy.get_param("~vertical_speed", 0.8))
        self.yaw_rate = float(rospy.get_param("~yaw_rate", math.radians(120.0)))
        self.ground_z = float(rospy.get_param("~ground_z", 0.0))
        self.initial_x = float(rospy.get_param("~initial_x", 0.0))
        self.initial_y = float(rospy.get_param("~initial_y", 0.0))
        self.initial_yaw = math.radians(float(rospy.get_param("~initial_yaw_deg", 0.0)))
        self.subscribe_setpoint = bool(rospy.get_param("~subscribe_setpoint", True))
        self.fault = str(rospy.get_param("~fault", "")).strip().lower()
        self.fault_delay = float(rospy.get_param("~fault_delay", 0.8))
        if self.fault not in ("", "vision_loss", "vision_output_loss",
                              "vision_consumer_loss", "setpoint_consumer_loss"):
            raise ValueError("unsupported simulated fault: %s" % self.fault)

        self.mode = "STABILIZED"
        self.armed = False
        self.landed = True
        # The simulated vehicle state is ROS local ENU/base_link. Incoming raw
        # setpoints take the same route as MAVROS: ROS ENU/FLU -> MAVLink
        # LOCAL_NED/FRD -> PX4 local NED -> MAVROS local ENU/base_link.
        self.position = [self.initial_x, self.initial_y, self.ground_z]
        self.yaw = self.initial_yaw
        self.target = None
        self.mode_requests = []
        self.arm_requests = []
        self.payload_commands = []
        self.payload_command_targets = []
        self.payload_sequence = 0
        self.setpoint_targets = []
        self.mavlink_ned_targets = []
        self.setpoint_count = 0
        self.last_tick = time.monotonic()
        self.completed = False
        self.flight_started = None
        self.fault_active = False
        self.abort_seen = False
        self.abort_reason = ""
        self.abort_seen_time = None
        self.abort_source_stamp = None
        self.setpoint_source_stamps = []
        self.fault_summary_published = False

        self.state_pub = rospy.Publisher("/mavros/state", State, queue_size=5)
        self.extended_pub = rospy.Publisher("/mavros/extended_state", ExtendedState, queue_size=5)
        self.estimator_pub = rospy.Publisher("/mavros/estimator_status", EstimatorStatus, queue_size=5)
        self.local_pub = rospy.Publisher("/mavros/local_position/odom", Odometry, queue_size=5)
        self.timesync_pub = rospy.Publisher(
            "/mavros/timesync_status", TimesyncStatus, queue_size=5)
        self.vision_pub = rospy.Publisher("/robotac/fastlio_vision/healthy", Bool, queue_size=1, latch=True)
        self.vision_status_pub = rospy.Publisher(
            "/robotac/fastlio_vision/status", String, queue_size=5, latch=True)
        self.output_pub = rospy.Publisher(
            "/robotac/fastlio_vision/output_enabled", Bool, queue_size=1, latch=True)
        self.vision_pose_pub = rospy.Publisher(
            "/mavros/vision_pose/pose_cov", PoseWithCovarianceStamped, queue_size=5)
        self.payload_status_pub = rospy.Publisher(
            "/robotac/servo/status", String, queue_size=5, latch=True)
        # Mirror the servo node's startup closed command so the controller can
        # establish its required status sequence baseline before mission start.
        self.payload_sequence = 1
        self.payload_status_pub.publish(
            String(data="state=closed success=true seq=1 boot=sim"))
        self.summary_pub = rospy.Publisher("/robotac/test/flight_summary", String, queue_size=1, latch=True)
        self.fault_summary_pub = rospy.Publisher(
            "/robotac/test/flight_fault_summary", String, queue_size=1, latch=True)

        self.setpoint_sub = None
        if self.subscribe_setpoint:
            self.setpoint_sub = rospy.Subscriber("/mavros/setpoint_raw/local", PositionTarget,
                                                 self._setpoint_cb, queue_size=20)
        # Stand in for MAVROS's vision_pose_estimate plugin so the controller's
        # graph-level consumer check exercises the same ROS topic contract.
        self.vision_pose_sub = rospy.Subscriber(
            "/mavros/vision_pose/pose_cov", PoseWithCovarianceStamped,
            lambda _msg: None, queue_size=10)
        rospy.Subscriber("/robotac/flight/status", String, self._status_cb, queue_size=10)
        rospy.Subscriber("/robotac/servo/open", Bool, self._payload_cb, queue_size=5)
        rospy.Service("/mavros/set_mode", SetMode, self._set_mode_cb)
        rospy.Service("/mavros/cmd/arming", CommandBool, self._arming_cb)
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(1.0, self.rate_hz)), self._tick)

    @staticmethod
    def _move_toward(current, target, limit):
        error = target - current
        if abs(error) <= limit:
            return target
        return current + math.copysign(limit, error)

    @staticmethod
    def _wrap_angle(value):
        return math.atan2(math.sin(value), math.cos(value))

    @classmethod
    def _enu_to_ned_target(cls, target):
        """Mirror MAVROS SetpointRawPlugin::local_cb for FRAME_LOCAL_NED."""
        east, north, up, yaw_enu = target
        return (north, east, -up, cls._wrap_angle(math.pi / 2.0 - yaw_enu))

    @classmethod
    def _ned_to_enu_target(cls, target):
        """Mirror MAVROS local-position output back into ROS ENU/base_link."""
        north, east, down, yaw_ned = target
        return (east, north, -down, cls._wrap_angle(math.pi / 2.0 - yaw_ned))

    @staticmethod
    def _append_unique(targets, target):
        if not targets or any(
                abs(current - previous) > 1.0e-3
                for current, previous in zip(target, targets[-1])):
            targets.append(target)

    def _setpoint_cb(self, msg):
        if msg.coordinate_frame != PositionTarget.FRAME_LOCAL_NED:
            rospy.logerr("unexpected coordinate frame: %d", msg.coordinate_frame)
            return
        ros_enu_target = (msg.position.x, msg.position.y, msg.position.z, msg.yaw)
        mavlink_ned_target = self._enu_to_ned_target(ros_enu_target)
        self.target = self._ned_to_enu_target(mavlink_ned_target)
        self._append_unique(self.setpoint_targets, self.target)
        self._append_unique(self.mavlink_ned_targets, mavlink_ned_target)
        self.setpoint_count += 1
        self.setpoint_source_stamps.append(msg.header.stamp.to_sec())

    def _set_mode_cb(self, request):
        self.mode_requests.append(request.custom_mode)
        if request.custom_mode in ("OFFBOARD", "AUTO.LAND"):
            self.mode = request.custom_mode
            if request.custom_mode == "AUTO.LAND":
                self.landed = False
        return SetModeResponse(mode_sent=True)

    def _arming_cb(self, request):
        self.arm_requests.append(bool(request.value))
        self.armed = bool(request.value)
        self.landed = not self.armed
        return CommandBoolResponse(success=True, result=0)

    def _payload_cb(self, msg):
        is_open = bool(msg.data)
        self.payload_commands.append(is_open)
        self.payload_command_targets.append(self.target)
        self.payload_sequence += 1
        self.payload_status_pub.publish(String(data=(
            "state=%s success=true seq=%d boot=sim" %
            ("open" if is_open else "closed", self.payload_sequence))))

    def _route_summary(self):
        airborne = [target for target in self.setpoint_targets
                    if target[2] > self.ground_z + 0.1]
        return "->".join("(%.3f,%.3f,%.3f)" % target[:3] for target in airborne)

    def _mavlink_ned_route_summary(self):
        airborne = [target for target in self.mavlink_ned_targets
                    if target[2] < -self.ground_z - 0.1]
        return "->".join("(%.3f,%.3f,%.3f,%.3f)" % target for target in airborne)

    def _status_cb(self, msg):
        if "state=ABORT" in msg.data and not self.abort_seen:
            self.abort_seen = True
            self.abort_seen_time = time.monotonic()
            # ``PositionTarget`` carries the controller's source timestamp.
            # Count messages generated after this status callback, rather than
            # discarding an arbitrary receive-time window that could hide a
            # continuing setpoint stream.
            self.abort_source_stamp = None
            for field in msg.data.split():
                if field.startswith("error="):
                    self.abort_reason = field.split("=", 1)[1]
                elif field.startswith("stamp="):
                    try:
                        self.abort_source_stamp = float(field.split("=", 1)[1])
                    except ValueError:
                        pass
            if self.abort_source_stamp is None:
                self.abort_source_stamp = rospy.Time.now().to_sec()
            return
        if "state=COMPLETE" not in msg.data or self.completed:
            return
        self.completed = True
        open_target = next((target for command, target in zip(
            self.payload_commands, self.payload_command_targets) if command), None)
        open_target_text = ("none" if open_target is None else
                            "(%.3f,%.3f,%.3f)" % open_target[:3])
        self.summary_pub.publish(String(data=(
            "complete mode_requests=%s arm_requests=%s payload_commands=%s route=%s mavlink_ned_route=%s payload_open_at=%s setpoints=%d final=(%.3f,%.3f,%.3f,%.3f)" %
            (",".join(self.mode_requests), ",".join(str(value) for value in self.arm_requests),
             ",".join("open" if value else "closed" for value in self.payload_commands),
             self._route_summary(), self._mavlink_ned_route_summary(), open_target_text, self.setpoint_count,
             self.position[0], self.position[1], self.position[2], self.yaw))))

    def _advance_vehicle(self, dt):
        if self.mode == "AUTO.LAND":
            self.position[2] = max(self.ground_z, self.position[2] - self.vertical_speed * dt)
            if self.position[2] <= self.ground_z + 1.0e-3:
                self.position[2] = self.ground_z
                self.armed = False
                self.landed = True
            return
        if self.mode != "OFFBOARD" or not self.armed or self.target is None:
            return
        self.position[0] = self._move_toward(
            self.position[0], self.target[0], self.horizontal_speed * dt)
        self.position[1] = self._move_toward(
            self.position[1], self.target[1], self.horizontal_speed * dt)
        self.position[2] = self._move_toward(
            self.position[2], self.target[2], self.vertical_speed * dt)
        yaw_error = math.atan2(math.sin(self.target[3] - self.yaw),
                               math.cos(self.target[3] - self.yaw))
        self.yaw += math.copysign(min(abs(yaw_error), self.yaw_rate * dt), yaw_error)
        self.landed = False

    def _fault_due(self):
        if not self.fault:
            return False
        if self.flight_started is None:
            if self.mode == "OFFBOARD" and self.armed:
                self.flight_started = time.monotonic()
            return False
        return time.monotonic() - self.flight_started >= self.fault_delay

    def _publish_fault_summary(self):
        if not self.abort_seen or self.fault_summary_published:
            return
        now = time.monotonic()
        # Observe long enough for a faulty 20 Hz controller to emit several
        # messages. Source stamps distinguish a pre-ABORT transport backlog
        # from a message generated after the controller reported ABORT.
        if now - self.abort_seen_time < 0.8:
            return
        post_abort_setpoints = sum(
            1 for stamp in self.setpoint_source_stamps
            if stamp > self.abort_source_stamp)
        self.fault_summary_pub.publish(String(data=(
            "abort fault=%s error=%s post_abort_setpoints=%d mode=%s armed=%s" %
            (self.fault, self.abort_reason, post_abort_setpoints, self.mode, self.armed))))
        self.fault_summary_published = True

    def _tick(self, _event):
        now_wall = time.monotonic()
        dt = min(0.2, max(0.0, now_wall - self.last_tick))
        self.last_tick = now_wall
        self._advance_vehicle(dt)
        if self._fault_due():
            self.fault_active = True
            if self.fault == "vision_consumer_loss" and self.vision_pose_sub is not None:
                self.vision_pose_sub.unregister()
                self.vision_pose_sub = None
            if self.fault == "setpoint_consumer_loss" and self.setpoint_sub is not None:
                self.setpoint_sub.unregister()
                self.setpoint_sub = None

        now = rospy.Time.now()
        state = State(connected=True, armed=self.armed, mode=self.mode)
        self.state_pub.publish(state)
        self.extended_pub.publish(ExtendedState(
            landed_state=(ExtendedState.LANDED_STATE_ON_GROUND if self.landed
                          else ExtendedState.LANDED_STATE_IN_AIR)))
        self.estimator_pub.publish(EstimatorStatus(
            attitude_status_flag=True,
            pos_horiz_rel_status_flag=True,
            pos_vert_abs_status_flag=True,
            pos_vert_agl_status_flag=True))
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.position[0]
        odom.pose.pose.position.y = self.position[1]
        odom.pose.pose.position.z = self.position[2]
        quaternion = quaternion_from_euler(0.0, 0.0, self.yaw)
        odom.pose.pose.orientation.x = quaternion[0]
        odom.pose.pose.orientation.y = quaternion[1]
        odom.pose.pose.orientation.z = quaternion[2]
        odom.pose.pose.orientation.w = quaternion[3]
        self.local_pub.publish(odom)
        timesync = TimesyncStatus()
        timesync.header.stamp = now
        timesync.remote_timestamp_ns = now.to_nsec()
        timesync.observed_offset_ns = 0
        timesync.estimated_offset_ns = 0
        timesync.round_trip_time_ms = 1.0
        self.timesync_pub.publish(timesync)
        vision_healthy = not (self.fault_active and self.fault == "vision_loss")
        if vision_healthy:
            self.vision_pub.publish(Bool(data=True))
            self.vision_status_pub.publish(
                String(data="ok rate_hz=%.2f valid=20 dropped=0 mavros_output=true" % self.rate_hz))
        else:
            self.vision_pub.publish(Bool(data=False))
            self.vision_status_pub.publish(String(data="fastlio_timeout"))
        self.output_pub.publish(Bool(data=True))
        if not (self.fault_active and self.fault in ("vision_loss", "vision_output_loss")):
            vision_pose = PoseWithCovarianceStamped()
            vision_pose.header.stamp = now
            vision_pose.header.frame_id = "odom"
            vision_pose.pose.pose.position.x = self.position[0]
            vision_pose.pose.pose.position.y = self.position[1]
            vision_pose.pose.pose.position.z = self.position[2]
            vision_pose.pose.pose.orientation.x = quaternion[0]
            vision_pose.pose.pose.orientation.y = quaternion[1]
            vision_pose.pose.pose.orientation.z = quaternion[2]
            vision_pose.pose.pose.orientation.w = quaternion[3]
            for index in (0, 7, 14, 21, 28, 35):
                vision_pose.pose.covariance[index] = 0.01
            self.vision_pose_pub.publish(vision_pose)
        self._publish_fault_summary()
        if not self.completed:
            self.summary_pub.publish(String(data=(
                "running mode=%s armed=%s setpoints=%d position=(%.3f,%.3f,%.3f)" %
                (self.mode, self.armed, self.setpoint_count,
                 self.position[0], self.position[1], self.position[2]))))


if __name__ == "__main__":
    rospy.init_node("robotac_flight_closed_loop_sim")
    ClosedLoopMavrosSim()
    rospy.spin()
