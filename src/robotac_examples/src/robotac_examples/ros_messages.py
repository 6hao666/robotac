"""将示例中的普通数值转换为 ROS 消息。"""

from geometry_msgs.msg import PoseStamped

from robotac_examples.geometry import quaternion_from_yaw


def make_pose(frame_id, stamp, target):
    message = PoseStamped()
    message.header.frame_id = frame_id
    message.header.stamp = stamp
    message.pose.position.x = target[0]
    message.pose.position.y = target[1]
    message.pose.position.z = target[2]
    quaternion = quaternion_from_yaw(target[3])
    message.pose.orientation.x = quaternion[0]
    message.pose.orientation.y = quaternion[1]
    message.pose.orientation.z = quaternion[2]
    message.pose.orientation.w = quaternion[3]
    return message
