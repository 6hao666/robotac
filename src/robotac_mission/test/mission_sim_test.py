#!/usr/bin/env python3
"""robotac_mission G1 仿真测试驱动（配合 launch/mission_sim.test）。

覆盖方案 §9.2/§9.3：正常流程（启动后停在 WAIT_START、无控制输出）、start 骨架
语义、启动保护（不在地面 / 已解锁 / 位姿过期 / 估计器异常 / 视觉不健康）、
操作员 stop、幂等性、reset 幂等、可观测性。ABORT_LAND 触达测试为飞行轮用例，
在本测试中不触发。
"""

import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import EstimatorStatus, ExtendedState
from mavros_msgs.srv import CommandBool
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger

# G1 用到（含 fake 注入）的全部服务；setUp 逐个 wait_for_service，
# 避免 WSL 冷启动慢机上服务未注册就调用而抛 ServiceException。
REQUIRED_SERVICES = [
    "/robotac_mission/start",
    "/robotac_mission/stop",
    "/robotac_mission/reset",
    "/robotac_test/set_on_ground",
    "/robotac_test/set_pose_stream",
    "/robotac_test/set_estimator",
    "/robotac_test/set_localization",
    "/robotac_test/set_vision_state",
    "/mavros/cmd/arming",
]


class MissionSimulationTest(unittest.TestCase):
    def setUp(self):
        self.state = None
        self.reason = None
        self.active = None
        self.result = None
        self.target = None

        self.landed_state = None
        self.extended_count = 0
        self.pose_count = 0
        self.last_pose_time = None
        self.estimator_ok = None
        self.estimator_count = 0
        self.localization_ok = None
        self.localization_count = 0
        self.vision_state = None
        self.vision_count = 0
        self.setpoint_seen = None

        rospy.Subscriber("/robotac_mission/state", String, self._state_cb,
                         queue_size=10)
        rospy.Subscriber("/robotac_mission/state_reason", String,
                         self._reason_cb, queue_size=10)
        rospy.Subscriber("/robotac_mission/active", Bool, self._active_cb,
                         queue_size=10)
        rospy.Subscriber("/robotac_mission/result", String, self._result_cb,
                         queue_size=10)
        rospy.Subscriber("/robotac_mission/target", PoseStamped,
                         self._target_cb, queue_size=10)
        rospy.Subscriber("/mavros/extended_state", ExtendedState,
                         self._extended_cb, queue_size=10)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped,
                         self._pose_cb, queue_size=10)
        rospy.Subscriber("/mavros/estimator_status", EstimatorStatus,
                         self._estimator_cb, queue_size=10)
        rospy.Subscriber("/vision_pose_bridge/healthy", Bool,
                         self._localization_cb, queue_size=10)
        rospy.Subscriber("/vision_pose_bridge/state", String,
                         self._vision_cb, queue_size=10)
        rospy.Subscriber("/robotac_test/setpoint_seen", Bool,
                         self._setpoint_cb, queue_size=10)

        self.start = rospy.ServiceProxy("/robotac_mission/start", Trigger)
        self.stop = rospy.ServiceProxy("/robotac_mission/stop", Trigger)
        self.reset = rospy.ServiceProxy("/robotac_mission/reset", Trigger)
        self.set_ground = rospy.ServiceProxy("/robotac_test/set_on_ground",
                                             SetBool)
        self.set_pose = rospy.ServiceProxy("/robotac_test/set_pose_stream",
                                           SetBool)
        self.set_estimator = rospy.ServiceProxy("/robotac_test/set_estimator",
                                                SetBool)
        self.set_localization = rospy.ServiceProxy(
            "/robotac_test/set_localization", SetBool)
        self.set_vision_state = rospy.ServiceProxy(
            "/robotac_test/set_vision_state", SetBool)
        self.arm = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)

        for service in REQUIRED_SERVICES:
            try:
                rospy.wait_for_service(service, timeout=30.0)
            except rospy.ROSException as exc:
                self.fail("服务未就绪：%s（%s）" % (service, exc))

    # ---- 订阅回调 ----

    def _state_cb(self, message):
        self.state = message.data

    def _reason_cb(self, message):
        self.reason = message.data

    def _active_cb(self, message):
        self.active = bool(message.data)

    def _result_cb(self, message):
        self.result = message.data

    def _target_cb(self, message):
        self.target = message

    def _extended_cb(self, message):
        self.landed_state = message.landed_state
        self.extended_count += 1

    def _pose_cb(self, unused_message):
        del unused_message
        self.pose_count += 1
        self.last_pose_time = time.monotonic()

    def _estimator_cb(self, message):
        self.estimator_ok = bool(
            message.attitude_status_flag and
            message.pos_horiz_rel_status_flag and
            message.pos_vert_abs_status_flag)
        self.estimator_count += 1

    def _localization_cb(self, message):
        self.localization_ok = bool(message.data)
        self.localization_count += 1

    def _vision_cb(self, message):
        self.vision_state = message.data
        self.vision_count += 1

    def _setpoint_cb(self, message):
        self.setpoint_seen = bool(message.data)

    # ---- 等待工具 ----

    def _wait_for(self, condition, timeout=15.0):
        end_time = time.monotonic() + timeout
        while time.monotonic() < end_time and not rospy.is_shutdown():
            if condition():
                return True
            rospy.sleep(0.05)
        return False

    def _wait_state(self, state, timeout=15.0):
        return self._wait_for(lambda: self.state == state, timeout)

    def _wait_reason_contains(self, text, timeout=15.0):
        return self._wait_for(
            lambda: self.reason is not None and text in self.reason, timeout)

    # ---- 故障注入 ----

    def _set_ground(self, on_ground):
        count = self.extended_count
        response = self.set_ground(on_ground)
        expected = (ExtendedState.LANDED_STATE_ON_GROUND if on_ground
                    else ExtendedState.LANDED_STATE_IN_AIR)
        return response.success and self._wait_for(
            lambda: self.landed_state == expected and
            self.extended_count >= count + 2)

    def _set_estimator(self, enabled):
        count = self.estimator_count
        response = self.set_estimator(enabled)
        return response.success and self._wait_for(
            lambda: self.estimator_ok == enabled and
            self.estimator_count >= count + 2)

    def _set_localization(self, enabled):
        count = self.localization_count
        response = self.set_localization(enabled)
        return response.success and self._wait_for(
            lambda: self.localization_ok == enabled and
            self.localization_count >= count + 2)

    def _set_vision_state(self, ok):
        count = self.vision_count
        response = self.set_vision_state(ok)
        expected = "OK" if ok else "UNHEALTHY"
        return response.success and self._wait_for(
            lambda: self.vision_state == expected and
            self.vision_count >= count + 2)

    def _set_pose(self, enabled):
        count = self.pose_count
        response = self.set_pose(enabled)
        if not response.success:
            return False
        if enabled:
            return self._wait_for(lambda: self.pose_count >= count + 2)
        return self._wait_for(
            lambda: self.last_pose_time is not None and
            time.monotonic() - self.last_pose_time > 0.7)

    def _expect_start_rejected(self, reason_hint=None):
        response = self.start()
        self.assertFalse(response.success, response.message)
        if reason_hint:
            self.assertIn(reason_hint, response.message)

    # ---- 测试 ----

    def test_skeleton_normal_flow_and_safety_failures(self):
        # 正常流程：启动后停在 WAIT_START，active 始终 False，无控制输出
        self.assertTrue(self._wait_state("WAIT_START", 25.0))
        self.assertFalse(self.active)
        self.assertIsNotNone(self.target)
        self.assertEqual(self.result, "")
        # G1 判据（方案 §9.3）：通过 fake_fcu setpoint 计数器确认未发 setpoint
        self.assertTrue(self._wait_for(lambda: self.setpoint_seen is not None))
        self.assertFalse(self.setpoint_seen)

        # start 骨架语义：接受但停留 WAIT_START
        response = self.start()
        self.assertTrue(response.success, response.message)
        self.assertIn("飞行态未启用", response.message)
        self.assertTrue(self._wait_state("WAIT_START"))
        self.assertFalse(self.active)

        # stop：预启动态回 WAIT_READY；前置恢复后再进 WAIT_START
        response = self.stop()
        self.assertTrue(response.success, response.message)
        self.assertTrue(self._wait_state("WAIT_READY", 10.0))
        # stop 幂等（WAIT_READY 时再次 stop 无效果）
        response = self.stop()
        self.assertTrue(response.success, response.message)
        self.assertTrue(self._wait_state("WAIT_READY"))
        self.assertTrue(self._wait_state("WAIT_START", 10.0))

        # reset：预启动态幂等
        response = self.reset()
        self.assertTrue(response.success, response.message)
        self.assertTrue(self._wait_state("WAIT_START", 10.0))

        # 启动保护：不在地面拒 start
        self.assertTrue(self._set_ground(False))
        self.assertTrue(self._wait_state("WAIT_READY", 10.0))
        self.assertTrue(self._wait_reason_contains("不在地面"))
        self._expect_start_rejected()
        self.assertTrue(self._set_ground(True))
        self.assertTrue(self._wait_state("WAIT_START", 10.0))

        # 启动保护：已解锁拒 start（fake arming）
        self.assertTrue(self.arm(True).success)
        self.assertTrue(self._wait_state("WAIT_READY", 10.0))
        self._expect_start_rejected()
        self.assertTrue(self.arm(False).success)
        self.assertTrue(self._set_ground(True))
        self.assertTrue(self._wait_state("WAIT_START", 10.0))

        # 定位异常：位姿流停止 -> 过期 -> 回 WAIT_READY
        self.assertTrue(self._set_pose(False))
        self.assertTrue(self._wait_state("WAIT_READY", 10.0))
        self.assertTrue(self._wait_reason_contains("过期"))
        self._expect_start_rejected()
        self.assertTrue(self._set_pose(True))
        self.assertTrue(self._wait_state("WAIT_START", 10.0))

        # 估计器异常
        self.assertTrue(self._set_estimator(False))
        self.assertTrue(self._wait_state("WAIT_READY", 10.0))
        self._expect_start_rejected()
        self.assertTrue(self._set_estimator(True))
        self.assertTrue(self._wait_state("WAIT_START", 10.0))

        # 外部视觉 healthy 丢失
        self.assertTrue(self._set_localization(False))
        self.assertTrue(self._wait_state("WAIT_READY", 10.0))
        self._expect_start_rejected()
        self.assertTrue(self._set_localization(True))
        self.assertTrue(self._wait_state("WAIT_START", 10.0))

        # 外部视觉 state 非 OK
        self.assertTrue(self._set_vision_state(False))
        self.assertTrue(self._wait_state("WAIT_READY", 10.0))
        self._expect_start_rejected()
        self.assertTrue(self._set_vision_state(True))
        self.assertTrue(self._wait_state("WAIT_START", 10.0))

        # 骨架轮无飞行终态：result 保持空，active 保持 False，全程无控制输出
        self.assertEqual(self.result, "")
        self.assertFalse(self.active)
        self.assertFalse(self.setpoint_seen)


if __name__ == "__main__":
    rospy.init_node("mission_sim_test")
    rostest.rosrun("robotac_mission", "mission_sim_test",
                   MissionSimulationTest)
