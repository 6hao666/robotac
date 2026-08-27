#!/usr/bin/env python3
"""LandingConfirmation 单元测试：拒绝请求之前的缓存落地状态。"""

import unittest

from robotac_mission.landing import LandingConfirmation


class LandingConfirmationTest(unittest.TestCase):
    def setUp(self):
        self.gate = LandingConfirmation(required_samples=3)
        self.gate.begin(extended_sequence=10)

    def observe(self, sequence, **changes):
        values = {
            "on_ground": True,
            "extended_age": 0.1,
            "extended_timeout": 1.0,
            "armed": False,
            "pose_z": 0.0,
            "pose_age": 0.1,
            "pose_timeout": 0.5,
            "vertical_speed": 0.02,
            "velocity_age": 0.1,
            "velocity_timeout": 0.5,
            "max_height": 0.25,
            "max_vertical_speed": 0.1,
        }
        values.update(changes)
        return self.gate.observe(sequence, **values)

    def test_requires_new_samples_after_auto_land_request(self):
        self.assertFalse(self.observe(10))
        self.assertFalse(self.observe(11))
        self.assertFalse(self.observe(12))
        self.assertTrue(self.observe(13))

    def test_stale_or_reused_sample_cannot_confirm(self):
        self.assertFalse(self.observe(11, extended_age=2.0))
        self.assertFalse(self.observe(11))
        self.assertFalse(self.observe(12))
        self.assertFalse(self.observe(13))
        self.assertTrue(self.observe(14))

    def test_airborne_or_armed_sample_resets_consecutive_count(self):
        self.assertFalse(self.observe(11))
        self.assertFalse(self.observe(12, armed=True))
        self.assertFalse(self.observe(13))
        self.assertFalse(self.observe(14))
        self.assertTrue(self.observe(15))


if __name__ == "__main__":
    unittest.main()
