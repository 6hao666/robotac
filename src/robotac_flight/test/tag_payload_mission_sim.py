#!/usr/bin/env python3
"""Hardware-isolated simulator for ``tag_payload_mission.py``.

The simulator provides the small MAVROS, local-odometry, AprilTag, and payload
contracts needed by the AprilTag payload mission.  It never opens a serial
device and never talks to PX4; it is intended for the local/Ubuntu integration
test script next to this file.
"""

import math
import time

import rospy
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from mavros_msgs.msg import ExtendedState, PositionTarget, State
from mavros_msgs.srv import CommandBool, CommandBoolResponse, SetMode, SetModeResponse
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from tf.transformations import quaternion_from_euler


class TagPayloadMissionSim(object):
    def __init__(self):
        self.rate_hz = float(rospy.get_param("~rate_hz", 40.0))
        self.horizontal_speed = float(rospy.get_param("~horizontal_speed", 3.0))
        self.vertical_speed = float(rospy.get_param("~vertical_speed", 1.0))
        self.yaw_rate = float(rospy.get_param("~yaw_rate", math.radians(180.0)))
        self.ground_z = float(rospy.get_param("~ground_z", 0.0))
        self.initial_x = float(rospy.get_param("~initial_x", 0.0))
        self.initial_y = float(rospy.get_param("~initial_y", 0.0))
        self.initial_yaw = math.radians(float(rospy.get_param("~initial_yaw_deg", 0.0)))
        self.drop_tag_id = int(rospy.get_param("~drop_tag_id", 1))
        self.land_tag_id = int(rospy.get_param("~land_tag_id", 0))
        self.tag_size = float(rospy.get_param("~tag_size", 0.15))
        self.drop_tag_xyz = self._param_xyz("~drop_tag_xyz", (4.0, 0.0, self.ground_z))
        self.land_tag_xyz = self._param_xyz("~land_tag_xyz", (0.0, 0.0, self.ground_z))

        self.mode = "STABILIZED"
        self.armed = False
        self.landed = True
        self.position = [self.initial_x, self.initial_y, self.ground_z]
        self.yaw = self.initial_yaw
        self.target = None
        self.last_tick = time.monotonic()
        self.mission_state = "IDLE"
        self.completed = False

        self.mode_requests = []
        self.arm_requests = []
        self.payload_commands = []
        self.payload_command_targets = []
        self.setpoint_targets = []
        self.setpoint_count = 0
        self.payload_sequence = 1

        self.state_pub = rospy.Publisher("/mavros/state", State, queue_size=5)
        self.extended_pub = rospy.Publisher("/mavros/extended_state", ExtendedState, queue_size=5)
        self.local_pub = rospy.Publisher("/mavros/local_position/odom", Odometry, queue_size=5)
        self.tag_pub = rospy.Publisher("/tag_detections", AprilTagDetectionArray, queue_size=5)
        self.payload_status_pub = rospy.Publisher(
            "/robotac_servo/status", String, queue_size=5, latch=True)
        self.summary_pub = rospy.Publisher(
            "/robotac/test/tag_payload_mission_summary", String, queue_size=1, latch=True)

        rospy.Subscriber("/mavros/setpoint_raw/local", PositionTarget,
                         self._setpoint_cb, queue_size=20)
        rospy.Subscriber("/robotac_servo/control", Bool, self._payload_cb, queue_size=5)
        rospy.Subscriber("/robotac/tag_payload_mission/status", String,
                         self._mission_status_cb, queue_size=10)
        rospy.Service("/mavros/set_mode", SetMode, self._set_mode_cb)
        rospy.Service("/mavros/cmd/arming", CommandBool, self._arming_cb)

        self.payload_status_pub.publish(String(data="state=closed success=true seq=1 boot=tag_sim"))
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(1.0, self.rate_hz)), self._tick)

    @staticmethod
    def _param_xyz(name, default):
        value = rospy.get_param(name, default)
        if isinstance(value, str):
            parts = [float(item.strip()) for item in value.split(",")]
        else:
            parts = [float(item) for item in value]
        if len(parts) != 3:
            raise ValueError("%s must contain exactly x,y,z" % name)
        return tuple(parts)

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
        east, north, up, yaw_enu = target
        return (north, east, -up, cls._wrap_angle(math.pi / 2.0 - yaw_enu))

    @classmethod
    def _ned_to_enu_target(cls, target):
        north, east, down, yaw_ned = target
        return (east, north, -down, cls._wrap_angle(math.pi / 2.0 - yaw_ned))

    @staticmethod
    def _append_unique(targets, target):
        if not targets or any(abs(a - b) > 1.0e-3 for a, b in zip(target, targets[-1])):
            targets.append(target)

    def _setpoint_cb(self, msg):
        if msg.coordinate_frame != PositionTarget.FRAME_LOCAL_NED:
            rospy.logerr("unexpected coordinate frame: %d", msg.coordinate_frame)
            return
        ros_enu_target = (msg.position.x, msg.position.y, msg.position.z, msg.yaw)
        self.target = self._ned_to_enu_target(self._enu_to_ned_target(ros_enu_target))
        self._append_unique(self.setpoint_targets, self.target)
        self.setpoint_count += 1

    def _set_mode_cb(self, request):
        self.mode_requests.append(request.custom_mode)
        if request.custom_mode in ("OFFBOARD", "AUTO.LAND"):
            self.mode = request.custom_mode
            if request.custom_mode == "AUTO.LAND":
                self.landed = False
        return SetModeResponse(mode_sent=True)

    def _arming_cb(self, request):
        value = bool(request.value)
        self.arm_requests.append(value)
        self.armed = value
        self.landed = not value
        return CommandBoolResponse(success=True, result=0)

    def _payload_cb(self, msg):
        is_open = bool(msg.data)
        self.payload_commands.append(is_open)
        self.payload_command_targets.append(tuple(self.position))
        self.payload_sequence += 1
        self.payload_status_pub.publish(String(data=(
            "state=%s success=true seq=%d boot=tag_sim" %
            ("open" if is_open else "closed", self.payload_sequence))))

    def _mission_status_cb(self, msg):
        for field in msg.data.split():
            if field.startswith("state="):
                self.mission_state = field.split("=", 1)[1]
                break
        if self.mission_state == "COMPLETE" and not self.completed:
            self.completed = True
            self._publish_summary("complete")
        elif self.mission_state == "ABORT" and not self.completed:
            self.completed = True
            self._publish_summary("abort")

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
        self.position[0] = self._move_toward(self.position[0], self.target[0], self.horizontal_speed * dt)
        self.position[1] = self._move_toward(self.position[1], self.target[1], self.horizontal_speed * dt)
        self.position[2] = self._move_toward(self.position[2], self.target[2], self.vertical_speed * dt)
        yaw_error = self._wrap_angle(self.target[3] - self.yaw)
        self.yaw += math.copysign(min(abs(yaw_error), self.yaw_rate * dt), yaw_error)
        self.yaw = self._wrap_angle(self.yaw)
        self.landed = False

    def _publish_odom(self, now):
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.position[0]
        odom.pose.pose.position.y = self.position[1]
        odom.pose.pose.position.z = self.position[2]
        q = quaternion_from_euler(0.0, 0.0, self.yaw)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        self.local_pub.publish(odom)

    def _publish_tag_if_needed(self, now):
        if self.mission_state == "DROP_TAG_SCAN":
            tag_id = self.drop_tag_id
            xyz = self.drop_tag_xyz
        elif self.mission_state == "LAND_TAG_SCAN":
            tag_id = self.land_tag_id
            xyz = self.land_tag_xyz
        else:
            return
        detection = AprilTagDetection()
        detection.id = [tag_id]
        detection.size = [self.tag_size]
        detection.pose.header.stamp = now
        detection.pose.header.frame_id = "map"
        detection.pose.pose.pose.position.x = xyz[0]
        detection.pose.pose.pose.position.y = xyz[1]
        detection.pose.pose.pose.position.z = xyz[2]
        detection.pose.pose.pose.orientation.w = 1.0
        msg = AprilTagDetectionArray()
        msg.header.stamp = now
        msg.header.frame_id = "map"
        msg.detections.append(detection)
        self.tag_pub.publish(msg)

    def _route_summary(self):
        airborne = [target for target in self.setpoint_targets
                    if target[2] > self.ground_z + 0.05]
        return "->".join("(%.3f,%.3f,%.3f)" % target[:3] for target in airborne)

    def _publish_summary(self, outcome):
        open_target = next((target for command, target in zip(
            self.payload_commands, self.payload_command_targets) if command), None)
        open_target_text = "none" if open_target is None else "(%.3f,%.3f,%.3f)" % open_target[:3]
        self.summary_pub.publish(String(data=(
            "%s state=%s mode_requests=%s arm_requests=%s payload_commands=%s "
            "route=%s payload_open_at=%s setpoints=%d final=(%.3f,%.3f,%.3f,%.3f)" %
            (outcome, self.mission_state, ",".join(self.mode_requests),
             ",".join(str(value) for value in self.arm_requests),
             ",".join("open" if value else "closed" for value in self.payload_commands),
             self._route_summary(), open_target_text, self.setpoint_count,
             self.position[0], self.position[1], self.position[2], self.yaw))))

    def _tick(self, _event):
        now_wall = time.monotonic()
        dt = min(0.2, max(0.0, now_wall - self.last_tick))
        self.last_tick = now_wall
        self._advance_vehicle(dt)

        now = rospy.Time.now()
        self.state_pub.publish(State(connected=True, armed=self.armed, mode=self.mode))
        self.extended_pub.publish(ExtendedState(
            landed_state=(ExtendedState.LANDED_STATE_ON_GROUND if self.landed
                          else ExtendedState.LANDED_STATE_IN_AIR)))
        self._publish_odom(now)
        self._publish_tag_if_needed(now)
        if not self.completed:
            self.summary_pub.publish(String(data=(
                "running mission_state=%s mode=%s armed=%s setpoints=%d "
                "position=(%.3f,%.3f,%.3f)" %
                (self.mission_state, self.mode, self.armed, self.setpoint_count,
                 self.position[0], self.position[1], self.position[2]))))


if __name__ == "__main__":
    rospy.init_node("robotac_tag_payload_mission_sim")
    TagPayloadMissionSim()
    rospy.spin()
