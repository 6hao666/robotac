#!/usr/bin/env python3
"""起飞 OFFBOARD/解锁确认握手的无 ROS 回归测试。"""

import sys
import types
import unittest
from types import SimpleNamespace


if "mavros_msgs.msg" not in sys.modules:
    mavros = types.ModuleType("mavros_msgs")
    messages = types.ModuleType("mavros_msgs.msg")
    messages.ExtendedState = object
    sys.modules["mavros_msgs"] = mavros
    sys.modules["mavros_msgs.msg"] = messages

from robotac_mission._flight_stages import FlightStageMixin


class _Driver(FlightStageMixin):
    def __init__(self):
        self.now = 0.0
        self.config = {
            "control": {"prestream_seconds": 2.0},
            "timing": {"takeoff_hold": 1.0,
                       "stage_timeout": {"takeoff": 20.0}},
        }
        self.takeoff_target = [2.0, 0.8, 0.75]
        self.ctx = SimpleNamespace(
            pose=SimpleNamespace(pose=SimpleNamespace(
                position=SimpleNamespace(x=10.0, y=20.0, z=30.0))),
            fcu_state=SimpleNamespace(mode="POSCTL", armed=False))
        self.coord = SimpleNamespace(home_map=(10.0, 20.0, 30.0), ready=True)
        self.interfaces = SimpleNamespace(
            modes=[], arms=[],
            set_mode=lambda mode: (self.interfaces.modes.append(mode) or (True, "")),
            arm=lambda value: (self.interfaces.arms.append(value) or (True, "")))
        self._takeoff_armed = False
        self._takeoff_offboard_requested = False
        self._takeoff_offboard_confirmed = False
        self._takeoff_arm_requested = False
        self._takeoff_map = None
        self._stage_start = 0.0
        self.sent = []
        self.abort_reason = None

    def _now(self):
        return self.now

    def _send(self, target, is_map=False):
        self.sent.append((tuple(target), is_map))

    def _arrived_map(self, target, hold):
        return False

    def _stage_timeout(self, seconds):
        return self.now - self._stage_start > seconds

    def _advance(self):
        raise AssertionError("not expected")

    def _abort(self, reason):
        self.abort_reason = reason


class TakeoffHandshakeTest(unittest.TestCase):
    def test_waits_for_offboard_then_armed_feedback(self):
        driver = _Driver()
        driver._stage_takeoff()
        self.assertEqual(driver.interfaces.modes, [])
        self.assertEqual(driver.interfaces.arms, [])

        driver.now = 2.0
        driver._stage_takeoff()
        self.assertEqual(driver.interfaces.modes, ["OFFBOARD"])
        self.assertEqual(driver.interfaces.arms, [])

        driver._stage_takeoff()
        self.assertEqual(driver.interfaces.arms, [])

        driver.ctx.fcu_state.mode = "OFFBOARD"
        driver._stage_takeoff()
        self.assertEqual(driver.interfaces.arms, [])

        driver._stage_takeoff()
        self.assertEqual(driver.interfaces.arms, [True])
        self.assertFalse(driver._takeoff_armed)

        driver.ctx.fcu_state.armed = True
        driver._stage_takeoff()
        self.assertTrue(driver._takeoff_armed)
        self.assertIsNone(driver.abort_reason)


class RouteOnlyStageTest(unittest.TestCase):
    def test_reaches_delivery_point_without_tag_before_return(self):
        driver = _Driver()
        driver.config.update({
            "mission": {"route_only": True},
            "timing": {"waypoint_hold": 1.0,
                       "stage_timeout": {"search": 20.0}},
        })
        driver.mission_point = [2.0, 4.2, 0.8]
        driver.tag = SimpleNamespace(update=lambda *unused: (_ for _ in ()).throw(
            AssertionError("route_only must not read Tag")))
        driver._arrived = lambda target, hold: True
        advanced = []
        driver._advance = lambda: advanced.append(True)

        driver._stage_search()

        self.assertEqual(driver.sent[-1], ((2.0, 4.2, 0.8), False))
        self.assertEqual(advanced, [True])

    def test_nearby_route_only_mission_points_each_hold_independently(self):
        driver = _Driver()
        driver.config.update({
            "mission": {"route_only": True},
            "timing": {"waypoint_hold": 1.0,
                       "stage_timeout": {"search": 20.0}},
        })
        driver.mission_point = [2.0, 4.2, 0.8]
        driver.mission_points = ([2.0, 4.20, 0.8],
                                 [2.0, 4.22, 0.8],
                                 [2.0, 4.24, 0.8])
        driver._mission_index = 0
        driver.tag = SimpleNamespace(update=lambda *unused: (_ for _ in ()).throw(
            AssertionError("route_only must not read Tag")))
        driver._reached_since = None

        def arrived(unused_target, hold):
            if driver._reached_since is None:
                driver._reached_since = driver.now
            return driver.now - driver._reached_since >= hold

        driver._arrived = arrived
        advanced_at = []
        driver._advance = lambda: advanced_at.append(driver.now)

        # 按实际 20Hz 推进。三个点即使都在 0.15m 到达容差内，也必须各保持 1 秒。
        for step in range(80):
            driver.now = step * 0.05
            driver._stage_search()
            if advanced_at:
                break

        self.assertEqual(driver._mission_index, 2)
        self.assertEqual(len(advanced_at), 1)
        self.assertGreaterEqual(advanced_at[0], 3.0)
        self.assertLess(advanced_at[0], 3.3)
        sent_targets = [entry[0] for entry in driver.sent]
        for target in driver.mission_points:
            self.assertIn(tuple(target), sent_targets)


class FixedPointDropStageTest(unittest.TestCase):
    def test_reaches_fixed_endpoint_without_reading_tag_then_advances(self):
        driver = _Driver()
        driver.config.update({
            "mission": {"fixed_point_drop": True},
            "timing": {"waypoint_hold": 1.0,
                       "stage_timeout": {"search": 20.0}},
        })
        driver.mission_point = [2.0, 4.20, 0.55]
        driver.mission_points = ([2.0, 4.20, 0.55],)
        driver._mission_index = 0
        driver.tag = SimpleNamespace(update=lambda *unused: (_ for _ in ()).throw(
            AssertionError("fixed_point_drop must not read Tag")))
        driver._arrived = lambda target, hold: True
        advanced = []
        driver._advance = lambda: advanced.append(True)

        driver._stage_search()
        driver._stage_align()

        self.assertEqual(driver.sent[-1], ((2.0, 4.20, 0.55), False))
        self.assertEqual(advanced, [True, True])

    def test_release_target_uses_last_endpoint_with_servo_offset(self):
        driver = _Driver()
        driver.drop_point = [2.0, 4.24, 0.55]
        driver.config["payload"] = {"offset": [-0.04, 0.0, -0.09]}

        self.assertEqual(driver._release_target(), [2.04, 4.24, 0.55])


if __name__ == "__main__":
    unittest.main()
