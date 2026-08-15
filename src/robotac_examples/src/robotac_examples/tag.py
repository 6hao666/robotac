"""将 AprilTag 检测转换为本地坐标并发布偏差。"""

import time

import rospy
import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import PoseStamped, Vector3Stamped

from robotac_examples.tag_filter import find_detection, StableTag


class TagTracker(object):
    def __init__(self, tag_id=0, target_frame="map"):
        self.tag_id = tag_id
        self.target_frame = target_frame
        self.pose = None
        self.receive_time = None
        self.last_sample_time = None
        self.vehicle_pose = None
        self.tag_timeout = float(rospy.get_param("~tag_timeout", 0.8))
        if self.tag_timeout <= 0.0:
            raise ValueError("tag_timeout 必须大于 0")
        self.stability = StableTag(
            int(rospy.get_param("~stable_samples", 5)),
            float(rospy.get_param("~jump_limit", 0.15)))
        self.buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.pose_pub = rospy.Publisher("/robotac_examples/tag/pose",
                                        PoseStamped, queue_size=5)
        self.error_pub = rospy.Publisher("/robotac_examples/tag/error",
                                         Vector3Stamped, queue_size=5)
        from apriltag_ros.msg import AprilTagDetectionArray
        self.tag_sub = rospy.Subscriber("/tag_detections",
                                        AprilTagDetectionArray,
                                        self._tag_cb, queue_size=10)
        self.pose_sub = rospy.Subscriber("/mavros/local_position/pose",
                                         PoseStamped, self._vehicle_cb,
                                         queue_size=10)

    def _vehicle_cb(self, message):
        self.vehicle_pose = message

    def _tag_cb(self, message):
        detection = find_detection(message.detections, self.tag_id)
        if detection is None:
            return
        now = time.monotonic()
        source = PoseStamped()
        source.header = detection.pose.header
        if not source.header.frame_id:
            source.header = message.header
        source.pose = detection.pose.pose.pose
        try:
            transform = self.buffer.lookup_transform(
                self.target_frame, source.header.frame_id,
                source.header.stamp, rospy.Duration(0.15))
            local_pose = tf2_geometry_msgs.do_transform_pose(source, transform)
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException):
            rospy.logwarn_throttle(2.0, "无法将 AprilTag 转换到 %s",
                                   self.target_frame)
            return
        if (self.last_sample_time is not None and
                now - self.last_sample_time > self.tag_timeout):
            self.stability.reset()
        self.last_sample_time = now
        point = [local_pose.pose.position.x, local_pose.pose.position.y,
                 local_pose.pose.position.z]
        average = self.stability.add(point)
        if average is None:
            return
        local_pose.pose.position.x = average[0]
        local_pose.pose.position.y = average[1]
        local_pose.pose.position.z = average[2]
        self.pose = local_pose
        self.receive_time = now
        self.pose_pub.publish(local_pose)
        self._publish_error(local_pose)

    def _publish_error(self, tag_pose):
        if self.vehicle_pose is None:
            return
        error = Vector3Stamped()
        error.header = tag_pose.header
        error.vector.x = tag_pose.pose.position.x - self.vehicle_pose.pose.position.x
        error.vector.y = tag_pose.pose.position.y - self.vehicle_pose.pose.position.y
        error.vector.z = tag_pose.pose.position.z - self.vehicle_pose.pose.position.z
        self.error_pub.publish(error)

    def fresh(self, timeout=None):
        if timeout is None:
            timeout = self.tag_timeout
        return self.receive_time is not None and (
            time.monotonic() - self.receive_time <= timeout)
