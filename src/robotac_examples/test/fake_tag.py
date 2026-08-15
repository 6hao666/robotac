#!/usr/bin/env python3
"""按服务开关发布固定 ID 0 检测，仅用于离线仿真。"""

import rospy
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
from std_srvs.srv import SetBool, SetBoolResponse


class FakeTag(object):
    def __init__(self):
        self.enabled = True
        self.publisher = rospy.Publisher("/tag_detections",
                                         AprilTagDetectionArray, queue_size=5)
        rospy.Service("/robotac_test/set_tag", SetBool, self._set_enabled)
        rospy.Timer(rospy.Duration(0.05), self._publish)

    def _set_enabled(self, request):
        self.enabled = bool(request.data)
        return SetBoolResponse(True, "已更新")

    def _publish(self, unused_event):
        del unused_event
        if not self.enabled:
            return
        message = AprilTagDetectionArray()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "map"
        detection = AprilTagDetection()
        detection.id = [0]
        detection.size = [0.15]
        detection.pose.header = message.header
        detection.pose.pose.pose.position.x = 0.2
        detection.pose.pose.pose.orientation.w = 1.0
        message.detections = [detection]
        self.publisher.publish(message)


def main():
    rospy.init_node("fake_tag")
    FakeTag()
    rospy.spin()


if __name__ == "__main__":
    main()
