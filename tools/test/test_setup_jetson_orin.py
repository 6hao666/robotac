#!/usr/bin/env python3

import os
import pathlib
import shlex
import shutil
import subprocess
import tempfile
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[2]
SETUP_SCRIPT = ROOT / "tools/setup_jetson_orin.sh"
LEGACY_SETUP_SCRIPT = ROOT / "tools/setup_orin_nano_super.sh"
INSTALL_SCRIPT = ROOT / "tools/install_ubuntu20.sh"


def run_bash(script, command, env=None):
    shell = "source %s; %s" % (shlex.quote(str(script)), command)
    return subprocess.run(
        ["bash", "-c", shell],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        errors="replace",
        env=env,
        check=False,
    )


class JetsonOrinEnvironmentTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = pathlib.Path(self.temporary.name)
        self.ros_setup = self.root / "setup.bash"
        self.ros_setup.touch()
        self.sudo = self.root / "sudo"
        self.sudo.write_text("#!/usr/bin/env bash\nexit 0\n", encoding="utf-8")
        self.sudo.chmod(0o755)

    def tearDown(self):
        self.temporary.cleanup()

    def validate(self, architecture="aarch64", model="NVIDIA Jetson Orin Nano",
                 os_id="ubuntu", version="20.04", ros_setup=None,
                 uid="1000", owner="1000", sudo_path=None):
        values = [
            architecture, model, os_id, version,
            str(ros_setup or self.ros_setup), uid, owner,
            str(sudo_path or self.sudo),
        ]
        command = "validate_environment %s" % " ".join(
            shlex.quote(value) for value in values)
        return run_bash(SETUP_SCRIPT, command)

    def test_supported_models_pass(self):
        models = [
            "NVIDIA Jetson Orin Nano",
            "NVIDIA Jetson Orin Nano Engineering Reference Developer Kit Super",
            "NVIDIA Jetson Orin NX",
            "NVIDIA Jetson Orin NX Engineering Reference Developer Kit",
        ]
        for model in models:
            with self.subTest(model=model):
                self.assertEqual(self.validate(model=model).returncode, 0)

    def test_invalid_platform_conditions_fail(self):
        cases = [
            {"architecture": "x86_64"},
            {"model": "NVIDIA Jetson AGX Orin"},
            {"model": "NVIDIA Jetson Xavier NX"},
            {"model": "Generic ARM64 Computer"},
            {"os_id": "debian"},
            {"version": "22.04"},
            {"uid": "0", "owner": "0"},
            {"owner": "1001"},
            {"sudo_path": self.root / "missing-sudo"},
        ]
        for values in cases:
            with self.subTest(values=values):
                self.assertNotEqual(self.validate(**values).returncode, 0)

    def test_missing_noetic_fails(self):
        result = self.validate(ros_setup=self.root / "missing-setup.bash")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("ROS Noetic", result.stderr)

    def test_command_line_arguments_are_rejected(self):
        result = run_bash(SETUP_SCRIPT, "main unexpected-argument")
        self.assertEqual(result.returncode, 64)
        self.assertIn("用法", result.stderr)

    def test_legacy_entry_preserves_argument_rejection(self):
        result = subprocess.run(
            [str(LEGACY_SETUP_SCRIPT), "unexpected-argument"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            errors="replace",
            check=False,
        )
        self.assertEqual(result.returncode, 64)
        self.assertIn("用法", result.stderr)

    def test_legacy_entry_forwards_build_jobs(self):
        workspace = self.root / "workspace"
        tools = workspace / "tools"
        tools.mkdir(parents=True)
        legacy = tools / LEGACY_SETUP_SCRIPT.name
        generic = tools / SETUP_SCRIPT.name
        shutil.copy2(LEGACY_SETUP_SCRIPT, legacy)
        generic.write_text(
            "#!/usr/bin/env bash\n"
            "printf '%s\\n' \"${ROBOTAC_BUILD_JOBS:-missing}\"\n",
            encoding="utf-8",
        )
        generic.chmod(0o755)
        env = os.environ.copy()
        env["ROBOTAC_BUILD_JOBS"] = "7"
        result = subprocess.run(
            [str(legacy)],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            errors="replace",
            env=env,
            check=False,
        )
        self.assertEqual(result.returncode, 0)
        self.assertEqual(result.stdout.strip(), "7")


class LivoxInstallationTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = pathlib.Path(self.temporary.name)
        self.header = self.root / "livox_lidar_api.h"
        self.library = self.root / "liblivox_lidar_sdk_shared.so"
        self.library.touch()
        self.bin_dir = self.root / "bin"
        self.bin_dir.mkdir()
        self.nm = self.bin_dir / "nm"
        self.env = os.environ.copy()
        self.env["PATH"] = "%s:%s" % (self.bin_dir, self.env["PATH"])

    def tearDown(self):
        self.temporary.cleanup()

    def verify(self):
        command = "verify_livox_installation %s %s" % (
            shlex.quote(str(self.header)), shlex.quote(str(self.library)))
        return run_bash(SETUP_SCRIPT, command, self.env)

    def write_nm(self, symbol):
        self.nm.write_text(
            "#!/usr/bin/env bash\nprintf '00000000 T %s\\n'\n" % symbol,
            encoding="utf-8",
        )
        self.nm.chmod(0o755)

    def test_matching_header_and_library_pass(self):
        self.header.write_text(
            "void EnableLivoxLidarDiscoveryOnly();\n", encoding="utf-8")
        self.write_nm("EnableLivoxLidarDiscoveryOnly")
        self.assertEqual(self.verify().returncode, 0)

    def test_missing_header_symbol_fails(self):
        self.header.write_text("void OtherSymbol();\n", encoding="utf-8")
        self.write_nm("EnableLivoxLidarDiscoveryOnly")
        self.assertNotEqual(self.verify().returncode, 0)

    def test_missing_header_file_fails(self):
        self.write_nm("EnableLivoxLidarDiscoveryOnly")
        self.assertNotEqual(self.verify().returncode, 0)

    def test_missing_library_symbol_fails(self):
        self.header.write_text(
            "void EnableLivoxLidarDiscoveryOnly();\n", encoding="utf-8")
        self.write_nm("OtherSymbol")
        self.assertNotEqual(self.verify().returncode, 0)

    def test_missing_library_file_fails(self):
        self.header.write_text(
            "void EnableLivoxLidarDiscoveryOnly();\n", encoding="utf-8")
        self.library.unlink()
        self.write_nm("EnableLivoxLidarDiscoveryOnly")
        self.assertNotEqual(self.verify().returncode, 0)


class RosdepInitializationTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = pathlib.Path(self.temporary.name)
        self.bin_dir = self.root / "bin"
        self.bin_dir.mkdir()
        self.marker = self.root / "rosdep-called"
        rosdep = self.bin_dir / "rosdep"
        rosdep.write_text(
            "#!/usr/bin/env bash\n"
            "touch \"$ROSDEP_MARKER\"\n"
            "exit \"${ROSDEP_EXIT:-0}\"\n",
            encoding="utf-8",
        )
        rosdep.chmod(0o755)
        self.env = os.environ.copy()
        self.env["PATH"] = "%s:%s" % (self.bin_dir, self.env["PATH"])
        self.env["ROSDEP_MARKER"] = str(self.marker)

    def tearDown(self):
        self.temporary.cleanup()

    def initialize(self, sources_file, env=None):
        command = "ensure_rosdep_initialized %s" % shlex.quote(str(sources_file))
        return run_bash(INSTALL_SCRIPT, command, env or self.env)

    def test_uninitialized_rosdep_runs_init(self):
        result = self.initialize(self.root / "missing-sources")
        self.assertEqual(result.returncode, 0)
        self.assertTrue(self.marker.exists())

    def test_initialized_rosdep_skips_init(self):
        sources = self.root / "20-default.list"
        sources.touch()
        result = self.initialize(sources)
        self.assertEqual(result.returncode, 0)
        self.assertFalse(self.marker.exists())

    def test_rosdep_init_failure_is_preserved(self):
        env = self.env.copy()
        env["ROSDEP_EXIT"] = "42"
        result = self.initialize(self.root / "missing-sources", env)
        self.assertEqual(result.returncode, 42)


if __name__ == "__main__":
    unittest.main()
