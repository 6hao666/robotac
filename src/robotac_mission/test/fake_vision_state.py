#!/usr/bin/env python3
"""发布 /vision_pose_bridge/state，并暴露故障注入服务。

fake_fcu 未提供该话题（方案 §9.3 fake 话题覆盖审计）；本节点在 G1 仿真中补发，
支持把 state 在 OK / UNHEALTHY 之间切换以测试 vision_healthy 安全门。
"""

import rospy
from std_msgs.msg import String
from std_srvs.srv import SetBool, SetBoolResponse


class FakeVisionState(object):
    def __init__(self):
        self.state = "OK"
        self.publisher = rospy.Publisher("/vision_pose_bridge/state", String,
                                         queue_size=1, latch=True)
        rospy.Service("/robotac_test/set_vision_state", SetBool, self._set)
        # R3-1 起启动门对 state 也做数据年龄门控：latch 只发一次会让年龄在阈值
        # 后超龄导致 G1 卡在 WAIT_READY，故按 0.5s 周期重发当前值。
        rospy.Timer(rospy.Duration(0.5), self._tick)
        self.publisher.publish(String(data=self.state))

    def _tick(self, unused_event):
        del unused_event
        self.publisher.publish(String(data=self.state))

    def _set(self, request):
        self.state = "OK" if request.data else "UNHEALTHY"
        self.publisher.publish(String(data=self.state))
        return SetBoolResponse(True, "已更新")


def main():
    rospy.init_node("fake_vision_state")
    FakeVisionState()
    rospy.spin()


if __name__ == "__main__":
    main()
