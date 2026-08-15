#!/usr/bin/env python3
"""示例 11：显式启动后调用一次投放机构服务。"""

import rospy
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger, TriggerResponse


class PayloadReleaseExample(object):
    def __init__(self):
        self.used = False
        self.released = bool(rospy.get_param("~released", True))
        self.state_pub = rospy.Publisher("~state", String, queue_size=1,
                                         latch=True)
        self.active_pub = rospy.Publisher("~active", Bool, queue_size=1,
                                          latch=True)
        self.servo = rospy.ServiceProxy("/robotac_servo/set_released", SetBool)
        self.start = rospy.Service("~start", Trigger, self._start_cb)
        self.state_pub.publish(String(data="IDLE"))
        self.active_pub.publish(Bool(data=False))

    def _start_cb(self, unused_request):
        del unused_request
        if self.used:
            return TriggerResponse(False, "本次节点运行已经使用")
        self.used = True
        self.state_pub.publish(String(data="RUNNING"))
        self.active_pub.publish(Bool(data=True))
        try:
            response = self.servo(self.released)
        except rospy.ServiceException as error:
            self.state_pub.publish(String(data="ABORT"))
            self.active_pub.publish(Bool(data=False))
            return TriggerResponse(False, "舵机服务调用失败: %s" % error)
        state = "COMPLETE" if response.success else "ABORT"
        self.state_pub.publish(String(data=state))
        self.active_pub.publish(Bool(data=False))
        return TriggerResponse(response.success, response.message)


def main():
    rospy.init_node("payload_release_example")
    PayloadReleaseExample()
    rospy.spin()


if __name__ == "__main__":
    main()
