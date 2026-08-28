#!/usr/bin/env python3
"""位置日志记录器：每0.5秒打印当前位置（map坐标 + field坐标）。"""
import rospy
from geometry_msgs.msg import PoseStamped
import math

class PoseLogger:
    def __init__(self):
        self.last_pose = None
        self.field_yaw = None
        
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, 
                         self.pose_callback, queue_size=10)
        
        self.field_yaw_offset = rospy.get_param('/robotac_mission/field_yaw_offset', -math.pi/2)
        
        rospy.Timer(rospy.Duration(0.5), self.timer_callback)
        
        rospy.loginfo("[POSE_LOGGER] 位置日志记录器已启动，0.5秒打印一次")
    
    def pose_callback(self, msg):
        self.last_pose = msg
    
    def timer_callback(self, event):
        if self.last_pose is None:
            rospy.logwarn_throttle(5.0, "[POSE_LOGGER] 等待位姿数据...")
            return
        
        p = self.last_pose.pose.position
        q = self.last_pose.pose.orientation
        
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        if self.field_yaw is None:
            self.field_yaw = yaw + self.field_yaw_offset
        
        cos_f = math.cos(self.field_yaw)
        sin_f = math.sin(self.field_yaw)
        field_x = cos_f * p.x + sin_f * p.y
        field_y = -sin_f * p.x + cos_f * p.y
        
        rospy.loginfo(
            "[POSE] t=%.2fs | map=(%.2f, %.2f, %.2f) | field=(%.2f, %.2f, %.2f) | yaw=%.2frad",
            rospy.get_time() % 1000,
            p.x, p.y, p.z,
            field_x, field_y, p.z,
            yaw
        )

def main():
    rospy.init_node('pose_logger', anonymous=False)
    logger = PoseLogger()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
