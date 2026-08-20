#!/usr/bin/env python3
"""guards 单元测试（纯 Python）。"""

import unittest

from robotac_mission import guards

FIELD_MIN = (0.0, 0.0, 0.0)
FIELD_MAX = (4.0, 5.0, 3.0)


class GuardTest(unittest.TestCase):
    def test_fcu_connected(self):
        self.assertTrue(guards.fcu_connected(True)[0])
        ok, reason = guards.fcu_connected(False)
        self.assertFalse(ok)
        self.assertIn("飞控", reason)

    def test_not_armed(self):
        self.assertTrue(guards.not_armed(False)[0])
        ok, reason = guards.not_armed(True)
        self.assertFalse(ok)
        self.assertIn("解锁", reason)

    def test_on_ground(self):
        self.assertTrue(guards.on_ground(1)[0])
        ok, reason = guards.on_ground(3)
        self.assertFalse(ok)
        self.assertIn("地面", reason)

    def test_pose_valid_ok(self):
        ok, _ = guards.pose_valid((1.0, 1.0, 0.5), 0.1,
                                  FIELD_MIN, FIELD_MAX, 0.5)
        self.assertTrue(ok)

    def test_pose_valid_stale(self):
        ok, reason = guards.pose_valid((1.0, 1.0, 0.5), 1.0,
                                       FIELD_MIN, FIELD_MAX, 0.5)
        self.assertFalse(ok)
        self.assertIn("过期", reason)

    def test_pose_valid_future_stamp(self):
        ok, _ = guards.pose_valid((1.0, 1.0, 0.5), -0.1,
                                  FIELD_MIN, FIELD_MAX, 0.5)
        self.assertFalse(ok)

    def test_pose_valid_out_of_field(self):
        ok, reason = guards.pose_valid((9.0, 1.0, 0.5), 0.1,
                                       FIELD_MIN, FIELD_MAX, 0.5)
        self.assertFalse(ok)
        self.assertIn("边界", reason)

    def test_pose_valid_nan(self):
        ok, reason = guards.pose_valid((float("nan"), 1.0, 0.5), 0.1,
                                       FIELD_MIN, FIELD_MAX, 0.5)
        self.assertFalse(ok)
        self.assertIn("非有限", reason)

    def test_estimator_ok(self):
        flags = {"attitude": True, "pos_horiz_rel": True, "pos_vert_abs": True}
        self.assertTrue(guards.estimator_ok(flags)[0])
        flags["pos_vert_abs"] = False
        ok, reason = guards.estimator_ok(flags)
        self.assertFalse(ok)
        self.assertIn("pos_vert_abs", reason)
        self.assertFalse(guards.estimator_ok({})[0])

    def test_timesync_ok(self):
        self.assertTrue(guards.timesync_ok(1.0, 50.0)[0])
        self.assertFalse(guards.timesync_ok(200.0, 50.0)[0])
        self.assertFalse(guards.timesync_ok(-1.0, 50.0)[0])
        self.assertFalse(guards.timesync_ok(None, 50.0)[0])

    def test_topic_fresh(self):
        self.assertTrue(guards.topic_fresh(0.1, 1.0)[0])
        # 边界：age == max_age 允许（等号含在内）
        self.assertTrue(guards.topic_fresh(1.0, 1.0)[0])
        ok, reason = guards.topic_fresh(1.5, 1.0, label="估计器状态")
        self.assertFalse(ok)
        self.assertIn("估计器状态", reason)
        self.assertFalse(guards.topic_fresh(None, 1.0)[0])
        self.assertFalse(guards.topic_fresh(-0.1, 1.0)[0])

    def test_vision_healthy(self):
        self.assertTrue(guards.vision_healthy(True, "OK")[0])
        self.assertFalse(guards.vision_healthy(False, "OK")[0])
        self.assertFalse(guards.vision_healthy(True, "UNHEALTHY")[0])
        self.assertFalse(guards.vision_healthy(None, None)[0])

    def test_readiness_aggregates(self):
        checks = [(True, ""), (False, "飞机不在地面"), (False, "本地位置过期")]
        ok, failed = guards.readiness(checks)
        self.assertFalse(ok)
        self.assertEqual(failed, ["飞机不在地面", "本地位置过期"])


if __name__ == "__main__":
    unittest.main()
