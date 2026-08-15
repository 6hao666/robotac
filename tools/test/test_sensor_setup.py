#!/usr/bin/env python3
"""sensor_setup.py 的纯本地单元测试。"""

import importlib.util
import json
import pathlib
import tempfile
import unittest
from unittest import mock


SCRIPT = pathlib.Path(__file__).resolve().parents[1] / "sensor_setup.py"
SPEC = importlib.util.spec_from_file_location("sensor_setup", str(SCRIPT))
sensor_setup = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(sensor_setup)


class CommandResult:
    def __init__(self, returncode=0, stdout="", stderr=""):
        self.returncode = returncode
        self.stdout = stdout
        self.stderr = stderr


class RecordingRunner:
    def __init__(self):
        self.commands = []

    def __call__(self, command, **kwargs):
        self.commands.append(command)
        return CommandResult()


class SensorSetupTest(unittest.TestCase):
    def test_atomic_config_update_changes_only_addresses(self):
        source = {
            "Mid360s": {"host_net_info": [{"host_ip": "<机载网卡地址>",
                                             "point_data_port": 56301}]},
            "lidar_configs": [{"ip": "<雷达地址>", "pcl_data_type": 1}],
        }
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "mid360s.json"
            path.write_text(json.dumps(source, ensure_ascii=False), encoding="utf-8")
            sensor_setup.atomic_write_lidar_config(
                path, "198.51.100.5", "198.51.100.12")
            updated = json.loads(path.read_text(encoding="utf-8"))
        self.assertEqual(updated["Mid360s"]["host_net_info"][0]["host_ip"],
                         "198.51.100.5")
        self.assertEqual(updated["lidar_configs"][0]["ip"], "198.51.100.12")
        self.assertEqual(updated["lidar_configs"][0]["pcl_data_type"], 1)

    def test_non_interactive_confirmation_is_always_false(self):
        self.assertFalse(sensor_setup.ask_yes_no("write", non_interactive=True))

    def test_interface_selection_requires_unambiguous_carrier(self):
        interfaces = [
            {"name": "eth0", "carrier": True, "ipv4": []},
            {"name": "eth1", "carrier": True, "ipv4": []},
        ]
        with self.assertRaises(sensor_setup.SetupError) as context:
            sensor_setup.select_interface(interfaces, None)
        self.assertEqual(context.exception.exit_code, sensor_setup.EXIT_AMBIGUOUS)
        self.assertEqual(
            sensor_setup.select_interface(interfaces, "eth1")["name"], "eth1")

    def test_configured_host_is_preferred_for_temporary_discovery(self):
        candidate = sensor_setup.discovery_candidate(
            "198.51.100.5", {"192.0.2.10"})
        self.assertEqual(candidate, "198.51.100.5")
        self.assertIsNone(sensor_setup.discovery_candidate(
            "198.51.100.5", {"198.51.100.5"}))

    def test_temporary_address_is_removed_after_context(self):
        runner = RecordingRunner()
        with mock.patch.object(sensor_setup, "require_commands"):
            with sensor_setup.TemporaryAddress(
                    "eth1", "198.51.100.5", runner=runner):
                self.assertEqual(runner.commands[1][3], "add")
        self.assertEqual(runner.commands[-1][3], "del")

    def test_helper_json_ignores_sdk_noise(self):
        data = sensor_setup.parse_helper_json(
            "SDK startup\n[{\"lidar_ip\":\"198.51.100.12\"}]\n")
        self.assertEqual(data[0]["lidar_ip"], "198.51.100.12")

    def test_camera_mode_requires_mjpeg_size_and_rate(self):
        supported = """
            [0]: 'MJPG' (Motion-JPEG, compressed)
                Size: Discrete 1920x1080
                    Interval: Discrete 0.033s (30.000 fps)
        """
        self.assertTrue(sensor_setup.camera_mode_supported(supported))
        self.assertFalse(sensor_setup.camera_mode_supported(
            supported.replace("30.000 fps", "20.000 fps")))

    def test_udev_capture_property_distinguishes_metadata(self):
        capture = sensor_setup.parse_udev_properties(
            "ID_V4L_CAPABILITIES=:capture:\nID_VENDOR_ID=0bda\n")
        metadata = sensor_setup.parse_udev_properties(
            "ID_V4L_CAPABILITIES=:\nID_VENDOR_ID=0bda\n")
        self.assertIn(":capture:", capture["ID_V4L_CAPABILITIES"])
        self.assertNotIn(":capture:", metadata["ID_V4L_CAPABILITIES"])


if __name__ == "__main__":
    unittest.main()
