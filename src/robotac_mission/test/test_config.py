#!/usr/bin/env python3
"""config 加载/校验单元测试（纯 Python）。"""

import os
import tempfile
import unittest

from robotac_mission.config import ConfigError, load_config


def _write(text):
    handle, path = tempfile.mkstemp(suffix=".yaml")
    with os.fdopen(handle, "w", encoding="utf-8") as stream:
        stream.write(text)
    return path


def _minimal():
    return """\
frames:
  mission_frame: map
  body_frame: base_link
limits:
  field_min: [0, 0, 0]
  field_max: [4, 5, 3]
  max_speed: 0.5
obstacle:
  size: [2.10, 0.15, 2.00]
  cross_gap: 0.95
  no_overfly: true
tables:
  takeoff_center: [2.00, 0.80]
  delivery_center: [2.00, 4.20]
  height: 0.75
  search_radius: 1.0
timing:
  pose_timeout: 0.5
  tag_timeout: 1.0
  topic_timeout: {fcu_state: 2.0, extended_state: 2.0, estimator_status: 3.0, timesync_status: 2.0, vision_healthy: 1.0, vision_state: 2.0}
  max_rtt_ms: 50.0
  total_window: 360.0
  flight_budget: 150.0
  takeoff_hold: 3.0
  align_hold: 3.0
  stage_timeout: {takeoff: 20, transit: 30, search: 30, align: 20, release: 10, return: 30, land: 20}
tag:
  family: tag36h11
  id: 0
  black_size_m: 0.15
  stable_time: 1.0
waypoints:
  takeoff: [2.0, 0.80, 1.0]
  obstacle_routing:
    approach: [2.0, 1.5, 0.6]
    gap_enter: [1.0, 1.5, 0.8]
    gap_cross: [1.0, 3.5, 0.8]
    resume: [2.0, 3.5, 0.6]
  mission: [[2.0, 4.20, 1.0]]
  return_routing:
    approach: [2.0, 3.5, 0.6]
    gap_enter: [3.0, 3.5, 0.8]
    gap_cross: [3.0, 1.5, 0.8]
    resume: [2.0, 1.5, 0.6]
  return: [[2.0, 0.80, 1.0]]
payload:
  enable: false
  retry_count: 0
mission:
  dry_run: true
"""


class ConfigTest(unittest.TestCase):
    def _load(self, text):
        path = _write(text)
        self.addCleanup(os.unlink, path)
        return load_config(path)

    def test_valid_minimal(self):
        config = self._load(_minimal())
        self.assertEqual(config["tag"]["id"], 0)
        self.assertTrue(config["mission"]["dry_run"])
        self.assertEqual(config["timing"]["total_window"], 360.0)

    def test_missing_section(self):
        text = _minimal().replace("limits:\n", "")
        with self.assertRaises(ConfigError):
            self._load(text)

    def test_field_bounds_must_increase(self):
        text = _minimal().replace("field_max: [4, 5, 3]",
                                  "field_max: [0, 5, 3]")
        with self.assertRaises(ConfigError):
            self._load(text)

    def test_missing_stage_timeout_key(self):
        text = _minimal().replace(", land: 20", "")
        with self.assertRaises(ConfigError):
            self._load(text)

    def test_missing_topic_timeout_key(self):
        text = _minimal().replace("vision_healthy: 1.0, vision_state: 2.0}",
                                  "vision_healthy: 1.0}")
        with self.assertRaises(ConfigError):
            self._load(text)

    def test_default_yaml_estimator_margin(self):
        # 交接显示 /mavros/estimator_status 实测 ~1Hz；topic_timeout 须 ≥2-3× 周期，
        # 防真实发布抖动误报"估计器数据过期"（R4-1）。加载随包发布的默认配置断言。
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "..", "config", "mission.yaml")
        config = load_config(path)
        self.assertGreaterEqual(
            config["timing"]["topic_timeout"]["estimator_status"], 2.0)

    def test_waypoint_out_of_field(self):
        text = _minimal().replace("takeoff: [2.0, 0.80, 1.0]",
                                  "takeoff: [2.0, 9.0, 1.0]")
        with self.assertRaises(ConfigError):
            self._load(text)

    def test_tabletop_waypoint_below_table_rejected(self):
        # R5-2：桌上方航点 z 低于桌面高（0.75）→ 撞桌，须拒绝
        text = _minimal().replace("takeoff: [2.0, 0.80, 1.0]",
                                  "takeoff: [2.0, 0.80, 0.5]")
        with self.assertRaises(ConfigError):
            self._load(text)

    def test_negative_timeout_rejected(self):
        text = _minimal().replace("pose_timeout: 0.5",
                                  "pose_timeout: -1")
        with self.assertRaises(ConfigError):
            self._load(text)

    def test_missing_file(self):
        with self.assertRaises(ConfigError):
            load_config("/nonexistent/mission.yaml")

    def test_invalid_yaml(self):
        with self.assertRaises(ConfigError):
            self._load("frames: [unclosed")


if __name__ == "__main__":
    unittest.main()
