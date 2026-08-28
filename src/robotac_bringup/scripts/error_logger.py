#!/usr/bin/env python3
"""错误日志过滤器：监听/rosout_agg，筛选ERROR和关键WARN打印。"""
import rospy
from rosgraph_msgs.msg import Log

WARN_KEYWORDS = ['timeout', 'lost', 'failed', 'invalid', 'exceed', 'error', 'fault']

def log_callback(msg):
    # 排除error_logger自己的日志，避免无限循环
    if msg.name == '/error_logger':
        return

    if msg.level == Log.ERROR:
        rospy.logwarn("[ERROR_FILTER] node=%s | msg=%s", msg.name, msg.msg)
        return
    if msg.level == Log.WARN:
        msg_lower = msg.msg.lower()
        if any(kw in msg_lower for kw in WARN_KEYWORDS):
            rospy.logwarn("[WARN_FILTER] node=%s | msg=%s", msg.name, msg.msg)

def main():
    rospy.init_node('error_logger', anonymous=False)
    rospy.loginfo("[ERROR_LOGGER] 错误日志过滤器已启动")
    rospy.Subscriber('/rosout_agg', Log, log_callback, queue_size=100)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
