#!/usr/bin/env python3
"""config 加载/校验单元测试（纯 Python）。"""

import math
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
  field_yaw_offset: -1.5707963267948966
limits:
  field_min: [0, 0, 0]
  field_max: [4, 5, 3]
  max_speed: 0.5
obstacle:
  center: [2.00, 2.50]
  size: [2.10, 0.15, 2.00]
  cross_gap: 0.95
  route_clearance: 0.20
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
  waypoint_hold: 1.0
  release_hold: 1.0
  land_confirm: 30.0
  stage_timeout: {takeoff: 20, transit: 30, search: 30, align: 20, release: 10, return: 30, land: 20}
landing:
  mode_retry_seconds: 1.0
  mode_retry_count: 3
  confirm_samples: 3
  max_height: 0.25
  max_vertical_speed: 0.10
  velocity_timeout: 0.5
control:
  rate_hz: 20.0
  position_tolerance: 0.15
  max_step: 0.20
  prestream_seconds: 2.0
  health_debounce: 5
  tag_jump_limit: 0.15
  pose_jump_limit: 1.0
tag:
  family: tag36h11
  id: 0
  black_size_m: 0.15
  stable_time: 1.0
  stable_samples: 5
waypoints:
  takeoff: [2.0, 0.80, 1.30]
  obstacle_routing:
    approach: [0.475, 1.75, 1.30]
    gap_enter: [0.475, 2.30, 1.30]
    gap_cross: [0.475, 2.70, 1.30]
    resume: [0.475, 3.25, 1.30]
  mission: [[2.0, 4.20, 1.30]]
  return_routing:
    approach: [0.475, 3.25, 1.30]
    gap_enter: [0.475, 2.70, 1.30]
    gap_cross: [0.475, 2.30, 1.30]
    resume: [0.475, 1.75, 1.30]
  return: [[2.0, 0.80, 1.30]]
payload:
  enable: false
  retry_count: 0
mission:
  dry_run: true
  flight_enabled: false
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

    def test_field_yaw_offset_required(self):
        text = _minimal().replace(
            "  field_yaw_offset: -1.5707963267948966\n", "")
        with self.assertRaises(ConfigError):
            self._load(text)

    def test_field_yaw_offset_must_be_finite_number(self):
        text = _minimal().replace(
            "field_yaw_offset: -1.5707963267948966",
            "field_yaw_offset: north")
        with self.assertRaises(ConfigError):
            self._load(text)

    def test_legacy_field_yaw_is_normalized_as_offset(self):
        text = _minimal().replace("field_yaw_offset:", "field_yaw:")
        config = self._load(text)
        self.assertAlmostEqual(config["frames"]["field_yaw_offset"],
                               -math.pi / 2)

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
        text = _minimal().replace("takeoff: [2.0, 0.80, 1.30]",
                                  "takeoff: [2.0, 9.0, 1.30]")
        with self.assertRaises(ConfigError):
            self._load(text)

    def test_tabletop_waypoint_below_table_rejected(self):
        # 飞行航点必须保留桌面上方 0.55m 的固定净空。
        text = _minimal().replace("takeoff: [2.0, 0.80, 1.30]",
                                  "takeoff: [2.0, 0.80, 1.29]")
        with self.assertRaises(ConfigError):
            self._load(text)

    def test_fixed_point_drop_must_be_bool(self):
        text = _minimal().replace("  flight_enabled: false\n",
                                  "  flight_enabled: false\n  fixed_point_drop: enabled\n")
        with self.assertRaises(ConfigError):
            self._load(text)

    def test_fixed_point_drop_and_route_only_are_exclusive(self):
        text = _minimal().replace(
            "  flight_enabled: false\n",
            "  flight_enabled: false\n  route_only: true\n  fixed_point_drop: true\n")
        with self.assertRaises(ConfigError):
            self._load(text)

    def test_fixed_point_drop_requires_payload_target(self):
        text = _minimal().replace("  flight_enabled: false\n",
                                  "  flight_enabled: false\n  fixed_point_drop: true\n")
        with self.assertRaises(ConfigError):
            self._load(text)

    def test_routing_through_obstacle_clearance_rejected(self):
        text = _minimal().replace(
            "gap_cross: [0.475, 2.70, 1.30]",
            "gap_cross: [2.0, 2.70, 1.30]", 1)
        with self.assertRaises(ConfigError):
            self._load(text)

    def test_missing_landing_confirmation_settings_rejected(self):
        text = _minimal().replace("landing:\n", "")
        with self.assertRaises(ConfigError):
            self._load(text)

    def test_negative_timeout_rejected(self):
        text = _minimal().replace("pose_timeout: 0.5",
                                  "pose_timeout: -1")
        with self.assertRaises(ConfigError):
            self._load(text)

    def test_flight_enabled_required(self):
        text = _minimal().replace("  flight_enabled: false\n", "")
        with self.assertRaises(ConfigError):
            self._load(text)

    def test_control_section_required(self):
        text = _minimal().replace("control:\n", "")
        with self.assertRaises(ConfigError):
            self._load(text)

    def test_control_health_debounce_must_be_int(self):
        text = _minimal().replace("health_debounce: 5",
                                  "health_debounce: 5.0")
        with self.assertRaises(ConfigError):
            self._load(text)

    def test_missing_waypoint_hold_rejected(self):
        text = _minimal().replace("  waypoint_hold: 1.0\n", "")
        with self.assertRaises(ConfigError):
            self._load(text)

    def test_stable_samples_required(self):
        text = _minimal().replace("  stable_samples: 5\n", "")
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
