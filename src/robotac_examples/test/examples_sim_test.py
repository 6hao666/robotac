#!/usr/bin/env python3

import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import EstimatorStatus, ExtendedState
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger


class ExamplesSimulationTest(unittest.TestCase):
    def setUp(self):
        self.states = {}
        self.setpoint_seen = False
        self.landed_state = None
        self.extended_count = 0
        self.pose_count = 0
        self.last_pose_time = None
        self.estimator_ok = None
        self.estimator_count = 0
        self.localization_ok = None
        self.localization_count = 0
        names = ["hover_default", "hover_stop", "hover_fault", "hover_stale",
                 "hover_estimator", "move_relative", "move_bounds",
                 "waypoints", "tag_success", "tag_loss"]
        for name in names:
            topic = "/robotac_examples/%s/state" % name
            rospy.Subscriber(topic, String, self._state_cb,
                             callback_args=name, queue_size=10)
        rospy.Subscriber("/robotac_test/setpoint_seen", Bool,
                         self._setpoint_cb, queue_size=10)
        rospy.Subscriber("/mavros/extended_state", ExtendedState,
                         self._extended_cb, queue_size=10)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped,
                         self._pose_cb, queue_size=10)
        rospy.Subscriber("/mavros/estimator_status", EstimatorStatus,
                         self._estimator_cb, queue_size=10)
        rospy.Subscriber("/vision_pose_bridge/healthy", Bool,
                         self._localization_cb, queue_size=10)

    def _state_cb(self, message, name):
        self.states[name] = message.data

    def _setpoint_cb(self, message):
        self.setpoint_seen = bool(message.data)

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

    def _assert_landed(self):
        self.assertEqual(self.landed_state,
                         ExtendedState.LANDED_STATE_ON_GROUND)

    def _wait_for(self, condition, timeout=15.0):
        end_time = time.monotonic() + timeout
        while time.monotonic() < end_time and not rospy.is_shutdown():
            if condition():
                return True
            rospy.sleep(0.05)
        return False

    def _wait_state(self, name, state, timeout=15.0):
        return self._wait_for(lambda: self.states.get(name) == state, timeout)

    def _set_ground(self, service, on_ground):
        count = self.extended_count
        response = service(on_ground)
        if on_ground:
            expected = ExtendedState.LANDED_STATE_ON_GROUND
        else:
            expected = ExtendedState.LANDED_STATE_IN_AIR
        return (response.success and self._wait_for(
            lambda: self.landed_state == expected and
            self.extended_count >= count + 2))

    def _set_estimator(self, service, enabled):
        count = self.estimator_count
        response = service(enabled)
        return (response.success and self._wait_for(
            lambda: self.estimator_ok == enabled and
            self.estimator_count >= count + 2))

    def _set_localization(self, service, enabled):
        count = self.localization_count
        response = service(enabled)
        return (response.success and self._wait_for(
            lambda: self.localization_ok == enabled and
            self.localization_count >= count + 2))

    def _set_pose_stream(self, service, enabled):
        count = self.pose_count
        response = service(enabled)
        if not response.success:
            return False
        if enabled:
            return self._wait_for(lambda: self.pose_count >= count + 2)
        return self._wait_for(
            lambda: self.last_pose_time is not None and
            time.monotonic() - self.last_pose_time > 0.7)

    def _start(self, name):
        path = "/robotac_examples/%s/start" % name
        rospy.wait_for_service(path, 5.0)
        return rospy.ServiceProxy(path, Trigger)()

    def _wait_start(self, name, expected_success, expected_message=None,
                    timeout=5.0):
        end_time = time.monotonic() + timeout
        response = None
        while time.monotonic() < end_time and not rospy.is_shutdown():
            response = self._start(name)
            success_matches = response.success == expected_success
            message_matches = (expected_message is None or
                               response.message == expected_message)
            if success_matches and message_matches:
                return response
            if response.success:
                return response
            rospy.sleep(0.05)
        return response

    def _stop(self, name):
        path = "/robotac_examples/%s/stop" % name
        return rospy.ServiceProxy(path, Trigger)()

    def test_examples_and_safety_failures(self):
        self.assertTrue(self._wait_state("hover_default", "IDLE"))
        rospy.sleep(0.5)
        self.assertFalse(self.setpoint_seen)

        servo_path = "/robotac_test/servo_missing/set_released"
        rospy.wait_for_service(servo_path, 5.0)
        servo_response = rospy.ServiceProxy(servo_path, SetBool)(True)
        self.assertFalse(servo_response.success)
        self.assertIn("串口不可用", servo_response.message)

        set_ground = rospy.ServiceProxy("/robotac_test/set_on_ground", SetBool)
        set_pose = rospy.ServiceProxy("/robotac_test/set_pose_stream", SetBool)
        set_localization = rospy.ServiceProxy(
            "/robotac_test/set_localization", SetBool)
        set_estimator = rospy.ServiceProxy(
            "/robotac_test/set_estimator", SetBool)
        set_tag = rospy.ServiceProxy("/robotac_test/set_tag", SetBool)

        self.assertTrue(self._set_ground(set_ground, False))
        response = self._wait_start(
            "hover_default", False, "飞机不在地面")
        self.assertFalse(response.success, response.message)
        self.assertEqual(response.message, "飞机不在地面")
        self.assertTrue(self._set_ground(set_ground, True))
        response = self._wait_start("hover_default", True)
        self.assertTrue(response.success, response.message)
        self.assertTrue(self._wait_state("hover_default", "COMPLETE"))
        self.assertTrue(self.setpoint_seen)
        # 2026-08-23 修复：原断言"hover_default/start 服务不存在"与上文
        # _wait_start("hover_default", True) 成功调用同一服务自相矛盾，删除。
        # (服务该存在，assertRaises(wait_for_service .5s) 反因服务存在而不抛异常)

        self.assertTrue(self._set_pose_stream(set_pose, False))
        response = self._wait_start(
            "hover_stale", False, "本地位置过期")
        self.assertFalse(response.success, response.message)
        self.assertEqual(response.message, "本地位置过期")
        self.assertTrue(self._set_pose_stream(set_pose, True))

        self.assertTrue(self._set_estimator(set_estimator, False))
        response = self._wait_start(
            "hover_estimator", False, "PX4 estimator 姿态无效")
        self.assertFalse(response.success, response.message)
        self.assertEqual(response.message, "PX4 estimator 姿态无效")
        self.assertTrue(self._set_estimator(set_estimator, True))

        response = self._wait_start("move_relative", True)
        self.assertTrue(response.success, response.message)
        self.assertTrue(self._wait_state("move_relative", "COMPLETE", 20.0))
        response = self._wait_start("waypoints", True)
        self.assertTrue(response.success, response.message)
        self.assertTrue(self._wait_state("waypoints", "COMPLETE", 20.0))

        response = self._wait_start("move_bounds", True)
        self.assertTrue(response.success, response.message)
        self.assertTrue(self._wait_state("move_bounds", "ABORT", 15.0))
        self._assert_landed()

        response = self._wait_start("hover_stop", True)
        self.assertTrue(response.success, response.message)
        self.assertTrue(self._wait_state("hover_stop", "TAKEOFF", 10.0))
        self.assertTrue(self._stop("hover_stop").success)
        self.assertTrue(self._wait_state("hover_stop", "ABORT"))
        self._assert_landed()

        response = self._wait_start("hover_fault", True)
        self.assertTrue(response.success, response.message)
        self.assertTrue(self._wait_state("hover_fault", "TAKEOFF", 10.0))
        self.assertTrue(self._set_localization(set_localization, False))
        self.assertTrue(self._wait_state("hover_fault", "ABORT"))
        self._assert_landed()
        self.assertTrue(self._set_localization(set_localization, True))

        response = self._wait_start("tag_success", True)
        self.assertTrue(response.success, response.message)
        self.assertTrue(self._wait_state("tag_success", "COMPLETE", 20.0))

        response = self._wait_start("tag_loss", True)
        self.assertTrue(response.success, response.message)
        self.assertTrue(self._wait_state("tag_loss", "TAG_CENTERING", 15.0))
        set_tag(False)
        self.assertTrue(self._wait_state("tag_loss", "ABORT", 10.0))
        self._assert_landed()


if __name__ == "__main__":
    rospy.init_node("examples_sim_test")
    rostest.rosrun("robotac_examples", "examples_sim_test",
                   ExamplesSimulationTest)
