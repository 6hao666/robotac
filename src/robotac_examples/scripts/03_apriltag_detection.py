#!/usr/bin/env python3
"""示例 03：显示 AprilTag ID 和相机坐标系位置。"""

import rospy
from apriltag_ros.msg import AprilTagDetectionArray


def detection_cb(message):
    if not message.detections:
        rospy.loginfo_throttle(2.0, "当前未检测到 AprilTag")
        return
    for detection in message.detections:
        position = detection.pose.pose.pose.position
        ids = ",".join(str(int(tag_id)) for tag_id in detection.id)
        age = (rospy.Time.now() - detection.pose.header.stamp).to_sec()
        rospy.loginfo(
            "Tag ID %s，相机坐标系 %s，位置(%.3f, %.3f, %.3f)，数据年龄 %.3f s",
            ids, detection.pose.header.frame_id,
            position.x, position.y, position.z, age)


def main():
    rospy.init_node("apriltag_detection_example")
    rospy.Subscriber("/tag_detections", AprilTagDetectionArray,
                     detection_cb, queue_size=10)
    rospy.spin()


if __name__ == "__main__":
    main()
