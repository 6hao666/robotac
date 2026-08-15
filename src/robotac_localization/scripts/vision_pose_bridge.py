#!/usr/bin/env python3
"""检查本地里程计，并按需发送到 MAVROS 外部视觉接口。"""

import math
import time

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String

from robotac_localization.validation import distance3
from robotac_localization.validation import pose_values
from robotac_localization.validation import quaternion_norm
from robotac_localization.validation import values_are_finite


class VisionPoseBridge(object):
    def __init__(self):
        self.publish_to_px4 = bool(rospy.get_param("~publish_to_px4", False))
        self.input_topic = rospy.get_param("~input_topic", "/sunray/odometry")
        self.preview_topic = rospy.get_param(
            "~preview_topic", "/robotac_localization/vision_pose_preview")
        self.px4_topic = rospy.get_param("~px4_topic", "/mavros/vision_pose/pose")
        self.output_frame = rospy.get_param("~output_frame", "odom")
        self.expected_frame = rospy.get_param("~expected_input_frame", "world")
        self.strict_frame = bool(rospy.get_param("~strict_input_frame", False))
        self.max_age = float(rospy.get_param("~max_age", 0.30))
        self.max_future = float(rospy.get_param("~max_future", 0.10))
        self.timeout = float(rospy.get_param("~timeout", 0.50))
        self.max_speed = float(rospy.get_param("~max_speed", 8.0))
        self.max_radius = float(rospy.get_param("~max_radius", 50.0))

        self.last_position = None
        self.last_stamp = None
        self.last_receive = None
        self.preview_pub = rospy.Publisher(self.preview_topic, PoseStamped,
                                           queue_size=10)
        self.px4_pub = None
        if self.publish_to_px4:
            self.px4_pub = rospy.Publisher(self.px4_topic, PoseStamped,
                                           queue_size=10)
        self.healthy_pub = rospy.Publisher("~healthy", Bool, queue_size=1,
                                           latch=True)
        self.state_pub = rospy.Publisher("~state", String, queue_size=1,
                                         latch=True)
        self.subscriber = rospy.Subscriber(self.input_topic, Odometry,
                                           self._odom_cb, queue_size=20)
        self.timer = rospy.Timer(rospy.Duration(0.1), self._timeout_check)
        self._publish_health(False, "WAITING")

    def _publish_health(self, healthy, state):
        self.healthy_pub.publish(Bool(data=healthy))
        self.state_pub.publish(String(data=state))

    def _reject(self, reason):
        self._publish_health(False, reason)
        rospy.logwarn_throttle(2.0, "外部视觉数据被拒绝: %s", reason)

    def _valid_message(self, message):
        now = rospy.Time.now()
        stamp = message.header.stamp
        age = (now - stamp).to_sec()
        if stamp.is_zero() or age > self.max_age or age < -self.max_future:
            return False, "STAMP"
        if self.strict_frame and message.header.frame_id != self.expected_frame:
            return False, "FRAME"
        pose = message.pose.pose
        if not values_are_finite(pose_values(pose)):
            return False, "NONFINITE"
        if abs(quaternion_norm(pose) - 1.0) > 0.05:
            return False, "QUATERNION"
        position = [pose.position.x, pose.position.y, pose.position.z]
        if distance3(position, [0.0, 0.0, 0.0]) > self.max_radius:
            return False, "RADIUS"
        if self.last_position is not None and self.last_stamp is not None:
            elapsed = (stamp - self.last_stamp).to_sec()
            if elapsed <= 0.0:
                return False, "ORDER"
            speed = distance3(position, self.last_position) / elapsed
            if not math.isfinite(speed) or speed > self.max_speed:
                return False, "SPEED"
        return True, "OK"

    def _odom_cb(self, message):
        valid, reason = self._valid_message(message)
        if not valid:
            self._reject(reason)
            return
        output = PoseStamped()
        output.header.stamp = message.header.stamp
        output.header.frame_id = self.output_frame
        output.pose = message.pose.pose
        self.preview_pub.publish(output)
        if self.px4_pub is not None:
            self.px4_pub.publish(output)
        position = output.pose.position
        self.last_position = [position.x, position.y, position.z]
        self.last_stamp = output.header.stamp
        self.last_receive = time.monotonic()
        self._publish_health(True, "OK")

    def _timeout_check(self, unused_event):
        del unused_event
        if self.last_receive is None:
            return
        if time.monotonic() - self.last_receive > self.timeout:
            self._publish_health(False, "TIMEOUT")


def main():
    rospy.init_node("vision_pose_bridge")
    VisionPoseBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
