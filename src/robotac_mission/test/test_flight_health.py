#!/usr/bin/env python3
"""flight_health 单元测试：飞行态 20Hz 健康门（纯逻辑，fake ctx）。

flight_health.check 只读 ctx 属性 + coord/config，不依赖 rospy，可独立单测。
"""

import unittest
from types import SimpleNamespace

from robotac_mission.coordinates import Coordinates
from robotac_mission import flight_health


def _config(debounce=3):
    return {
        "timing": {
            "pose_timeout": 0.5,
            "topic_timeout": {"timesync_status": 2.0,
                              "vision_healthy": 1.0},
            "max_rtt_ms": 50.0,
        },
        "control": {"health_debounce": debounce, "pose_jump_limit": 1.0},
        "limits": {"field_min": [0.0, 0.0, 0.0],
                   "field_max": [4.0, 5.0, 3.0]},
    }


def _pose(x=0.0, y=0.0, z=0.0):
    return SimpleNamespace(pose=SimpleNamespace(
        position=SimpleNamespace(x=x, y=y, z=z)))


class FakeCtx(object):
    def __init__(self):
        self.fcu_state = SimpleNamespace(
            connected=True, armed=True, mode="OFFBOARD")
        self.pose = _pose()
        self.estimator = SimpleNamespace(attitude_status_flag=True,
                                         pos_horiz_rel_status_flag=True,
                                         pos_vert_abs_status_flag=True)
        self.timesync = SimpleNamespace(round_trip_time_ms=1.0)
        self.vision_healthy = True
        self.vision_state = "OK"
        self.ages = {"pose": 0.1, "timesync": 0.1, "vision_healthy": 0.1}

    def topic_age(self, name):
        return self.ages.get(name, 0.1)


class FlightHealthTest(unittest.TestCase):
    def setUp(self):
        self.ctx = FakeCtx()
        self.coord = Coordinates()
        self.coord.capture_home((0.0, 0.0, 0.0), (2.0, 0.8))
        self.state = {"soft_bad": 0, "prev_pose": None}

    def check(self):
        return flight_health.check(self.ctx, self.coord,
                                   _config(), self.state)

    def test_healthy_returns_none(self):
        self.assertIsNone(self.check())

    def test_fcu_disconnected(self):
        self.ctx.fcu_state.connected = False
        self.assertEqual(self.check(), "飞控未连接")

    def test_pose_stale(self):
        self.ctx.ages["pose"] = 1.0
        self.assertEqual(self.check(), "本地位置过期")

    def test_estimator_invalid(self):
        self.ctx.estimator.pos_horiz_rel_status_flag = False
        self.assertIn("estimator", self.check())

    def test_offboard_lost(self):
        self.ctx.fcu_state.mode = "POSCTL"
        self.assertEqual(self.check(), "OFFBOARD 模式丢失")

    def test_timesync_debounced(self):
        self.ctx.timesync.round_trip_time_ms = 200.0
        self.assertIsNone(self.check())   # 1/3 次：去抖未到阈值
        self.assertIsNone(self.check())   # 2/3
        self.assertEqual(self.check(), "时间同步")

    def test_out_of_field(self):
        self.ctx.pose = _pose(x=10.0)   # field x=12 > 4
        self.assertEqual(self.check(), "超出场地边界")

    def test_pose_jump(self):
        self.state["prev_pose"] = (0.0, 0.0, 0.0)
        self.ctx.pose = _pose(x=5.0)    # 跳变 > 1.0
        self.assertIn("跳变", self.check())

    def test_vision_state_unhealthy(self):
        self.ctx.vision_state = "UNHEALTHY"
        self.assertEqual(self.check(), "外部视觉状态非 OK")


if __name__ == "__main__":
    unittest.main()
