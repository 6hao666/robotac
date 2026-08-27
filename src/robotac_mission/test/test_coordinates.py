#!/usr/bin/env python3
"""coordinates 单元测试：起飞点局部归零（方案 §16.2/§16.10）。"""

import math
import unittest

from robotac_mission.coordinates import Coordinates, limit_step


class CoordinatesTest(unittest.TestCase):
    def test_not_ready_before_capture(self):
        coord = Coordinates()
        with self.assertRaises(ValueError):
            coord.map_to_field((1.0, 1.0, 1.0))
        with self.assertRaises(ValueError):
            coord.field_to_map((1.0, 1.0, 1.0))

    def test_home_is_takeoff_field_xy(self):
        coord = Coordinates()
        # map 原点漂移到 (10,20,30)，home 应映射到起降台圆心 (2.0,0.8,0 地面)
        coord.capture_home((10.0, 20.0, 30.0), (2.0, 0.8))
        self.assertEqual(coord.map_to_field((10.0, 20.0, 30.0)),
                         (2.0, 0.8, 0.0))
        self.assertEqual(coord.field_to_map((2.0, 0.8, 0.0)),
                         (10.0, 20.0, 30.0))

    def test_round_trip(self):
        coord = Coordinates()
        coord.capture_home((-5.23, -8.06, 0.1), (2.0, 0.8), yaw=0.3)
        point = (1.0, 2.0, 3.0)
        restored = coord.field_to_map(coord.map_to_field(point))
        for index in range(3):
            self.assertAlmostEqual(restored[index], point[index])
        self.assertAlmostEqual(coord.home_yaw, 0.3)

    def test_relative_offsets(self):
        coord = Coordinates()
        coord.capture_home((10.0, 20.0, 30.0), (2.0, 0.8))
        # 场地内 1.0m 高度 = map 内 home.z + 1.0（示例 origin.z + height 语义）
        self.assertEqual(coord.field_to_map((2.0, 0.8, 1.0)),
                         (10.0, 20.0, 31.0))
        # 场地移动 (0, 1.0) = map 移动同量（平移不变）
        self.assertEqual(coord.map_to_field((10.0, 21.0, 30.0)),
                         (2.0, 1.8, 0.0))

    def test_field_yaw_rotation(self):
        # 2026-08-27：机头朝场地+y（field_yaw=π/2），起飞时 map 系 x 恰为机头方向
        # （home_yaw=0）。map 偏移 (+1,0)（沿机头方向）应映射到场地 +y。
        coord = Coordinates(field_yaw=math.pi / 2.0)
        coord.capture_home((10.0, 20.0, 30.0), (2.0, 0.8), yaw=0.0)
        # map 偏移 (+1, 0) -> 场地偏移 (0, +1)
        self.assertAlmostEqual(coord.map_to_field((11.0, 20.0, 30.0))[0], 2.0)
        self.assertAlmostEqual(coord.map_to_field((11.0, 20.0, 30.0))[1], 1.8)
        # 场地偏移 (0, +1) -> map 偏移 (+1, 0)
        self.assertAlmostEqual(coord.field_to_map((2.0, 1.8, 30.0))[0], 11.0)
        self.assertAlmostEqual(coord.field_to_map((2.0, 1.8, 30.0))[1], 20.0)
        # 高度（z）不受旋转影响（平移不变：Δz 守恒）
        self.assertAlmostEqual(coord.field_to_map((2.0, 1.8, 31.0))[2], 61.0)
        self.assertAlmostEqual(coord.field_to_map((2.0, 1.8, 30.0))[2], 60.0)

    def test_field_yaw_round_trip(self):
        coord = Coordinates(field_yaw=math.pi / 2.0)
        coord.capture_home((-5.23, -8.06, 0.1), (2.0, 0.8), yaw=0.7)
        point = (1.0, 2.0, 3.0)
        restored = coord.field_to_map(coord.map_to_field(point))
        for index in range(3):
            self.assertAlmostEqual(restored[index], point[index])

    def test_capture_validates_length(self):
        coord = Coordinates()
        with self.assertRaises(ValueError):
            coord.capture_home((1.0, 2.0), (2.0, 0.8))
        with self.assertRaises(ValueError):
            coord.capture_home((1.0, 2.0, 3.0), (2.0,))


class LimitStepTest(unittest.TestCase):
    def test_within_limit_unchanged(self):
        target = (1.0, 0.0, 0.0)
        result = limit_step((0.0, 0.0, 0.0), target, 2.0)
        self.assertEqual(result, target)

    def test_clamped_to_max_step(self):
        # M5：单 tick 位移 ≤ max_step；0.5/20=0.025 限幅
        result = limit_step((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 0.025)
        self.assertAlmostEqual(result[0], 0.025)
        self.assertEqual(result[1], 0.0)

    def test_zero_distance(self):
        self.assertEqual(limit_step((1.0, 1.0, 1.0), (1.0, 1.0, 1.0), 0.1),
                         (1.0, 1.0, 1.0))


if __name__ == "__main__":
    unittest.main()
