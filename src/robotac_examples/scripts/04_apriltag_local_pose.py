#!/usr/bin/env python3
"""示例 04：将 ID 0 AprilTag 转换到本地 map 坐标系。"""

import rospy

from robotac_examples.tag import TagTracker


def main():
    rospy.init_node("apriltag_local_pose_example")
    tag_id = int(rospy.get_param("~tag_id", 0))
    target_frame = rospy.get_param("~target_frame", "map")
    TagTracker(tag_id, target_frame)
    rospy.loginfo("等待 Tag ID %d，并转换到 %s", tag_id, target_frame)
    rospy.spin()


if __name__ == "__main__":
    main()
