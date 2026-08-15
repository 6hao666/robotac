#!/usr/bin/env python3
"""示例 01：只读显示飞控连接、模式、解锁和落地状态。"""

import rospy
from mavros_msgs.msg import ExtendedState, State


class FcuStateMonitor(object):
    def __init__(self):
        self.state = None
        self.landed_state = None
        rospy.Subscriber("/mavros/state", State, self._state_cb, queue_size=10)
        rospy.Subscriber("/mavros/extended_state", ExtendedState,
                         self._extended_cb, queue_size=10)

    def _state_cb(self, message):
        current = (message.connected, message.armed, message.mode)
        if current != self.state:
            self.state = current
            rospy.loginfo("飞控连接 %s，解锁 %s，当前模式 %s",
                          message.connected, message.armed, message.mode)

    def _extended_cb(self, message):
        if message.landed_state != self.landed_state:
            self.landed_state = message.landed_state
            rospy.loginfo("落地状态编号: %d", message.landed_state)


def main():
    rospy.init_node("fcu_state_example")
    FcuStateMonitor()
    rospy.spin()


if __name__ == "__main__":
    main()
