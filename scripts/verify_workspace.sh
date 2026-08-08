#!/usr/bin/env bash
set -euo pipefail

workspace_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
required_paths=(
  src/CMakeLists.txt
  src/livox_ros_driver2
  src/Livox-SDK2
  src/fast_lio
  src/web_cam
  src/mavros/mavros
  src/mavros/mavros_msgs
  src/apriltag
  src/apriltag/CATKIN_IGNORE
  src/apriltag_ros/apriltag_ros
  src/robotac_bringup
  src/robotac_flight
  src/robotac_servo
  config/lidar/mid360s.json
  config/camera/rgb.yaml
  config/fastlio/mid360s.yaml
  config/fastlio/vision_bridge.yaml
  config/flight/local_waypoints.yaml
  config/mavros/px4.yaml
  config/apriltag/settings.yaml
  config/apriltag/tags.yaml
  config/deployment.yaml
  config/deployment_sim.yaml
  config/udev/99-robotac-servo.rules.template
  config/udev/99-robotac-rgb-camera.rules.template
)

for path in "${required_paths[@]}"; do
  if [[ ! -e "${workspace_dir}/${path}" ]]; then
    echo "Missing required path: ${path}"
    exit 1
  fi
done

python3 - "${workspace_dir}/config/lidar/mid360s.json" <<'PY'
import json
import pathlib
import sys

path = pathlib.Path(sys.argv[1])
with path.open() as stream:
    json.load(stream)
print(f"Validated JSON: {path}")
PY

python3 - "${workspace_dir}/src" <<'PY'
import pathlib
import sys
import xml.etree.ElementTree as etree

root = pathlib.Path(sys.argv[1])
for path in sorted(root.rglob("*.launch")):
    etree.parse(path)
    print(f"Validated XML: {path.relative_to(root)}")
PY

echo "Workspace layout validation passed."

hardware_check="${workspace_dir}/src/robotac_bringup/scripts/check_hardware_config.sh"
if ! bash "${hardware_check}" "${workspace_dir}/config" false false false false false false; then
  echo "Passive full-system hardware check unexpectedly failed." >&2
  exit 1
fi
if ! bash "${hardware_check}" "${workspace_dir}/config" false false false false true false; then
  echo "Read-only MAVROS hardware check unexpectedly required flight calibration." >&2
  exit 1
fi
if bash "${hardware_check}" "${workspace_dir}/config" true true false false true true >/dev/null 2>&1; then
  echo "Active full-system hardware check unexpectedly bypassed deployment gates." >&2
  exit 1
fi
hardware_test_dir=$(mktemp -d "${TMPDIR:-/tmp}/robotac-hardware-check.XXXXXX")
mkdir -p "${hardware_test_dir}/lidar"
cp "${workspace_dir}/config/deployment_sim.yaml" "${hardware_test_dir}/deployment.yaml"
cp "${workspace_dir}/config/lidar/mid360s.json" "${hardware_test_dir}/lidar/mid360s.json"
if bash "${hardware_check}" "${hardware_test_dir}" false false false false false true >/dev/null 2>&1; then
  rm -rf "${hardware_test_dir}"
  echo "Active hardware check unexpectedly accepted the MID360s sample host address." >&2
  exit 1
fi
rm -rf "${hardware_test_dir}"
echo "Validated passive/active hardware-gate separation."

python3 - "${workspace_dir}/src/robotac_servo/scripts" <<'PY'
import pathlib
import py_compile
import sys

for path in sorted(pathlib.Path(sys.argv[1]).glob("*.py")):
    py_compile.compile(str(path), doraise=True)
    print(f"Validated Python syntax: {path.name}")
PY

python3 - "${workspace_dir}/src/robotac_servo/scripts/servo_node.py" <<'PY'
import pathlib
import sys

source = pathlib.Path(sys.argv[1]).read_text()
for expected in (
    "DEFAULT_OPEN_ANGLE = 45",
    "MIN_DUTY = 3",
    "MAX_DUTY = 12",
    '"/dev/robotac_servo"',
):
    if expected not in source:
        raise SystemExit(f"Servo protocol check failed: missing {expected}")
print("Validated servo defaults and PWM protocol constants.")
PY

python3 - "${workspace_dir}/src/robotac_flight/scripts" "${workspace_dir}/src/robotac_flight/test" <<'PY'
import pathlib
import py_compile
import sys

for directory in (pathlib.Path(value) for value in sys.argv[1:]):
    for path in sorted(directory.glob("*.py")):
        py_compile.compile(str(path), doraise=True)
        print(f"Validated flight Python syntax: {path.name}")
PY

bash -n "${workspace_dir}/src/robotac_flight/test/run_closed_loop_sim.sh"
echo "Validated closed-loop flight simulation shell syntax."

python3 - "${workspace_dir}/config/flight/local_waypoints.yaml" <<'PY'
import pathlib
import sys

source = pathlib.Path(sys.argv[1]).read_text()
for expected in (
    "waypoint_frame: robotac_start_body",
    "payload_action: open",
    "payload_topic: /robotac/servo/open",
    "payload_status_topic: /robotac/servo/status",
    "payload_required_connection: true",
    "payload_require_ack: true",
    "payload_ack_timeout: 1.0",
    "critical_fault_action: release",
    "operator_abort_action: release",
    "require_auto_land: true",
):
    if expected not in source:
        raise SystemExit(f"Flight route check failed: missing {expected}")
print("Validated start-body payload route configuration.")
PY

python3 - "${workspace_dir}/config/apriltag/tags.yaml" <<'PY'
import pathlib
import sys

source = pathlib.Path(sys.argv[1]).read_text()
for expected in (
    "total_size_m: 0.25",
    "tag_size_m: 0.15",
    "{id: 0, size: 0.15, name: tag_0}",
    "{id: 1, size: 0.15, name: tag_1}",
):
    if expected not in source:
        raise SystemExit(f"AprilTag config check failed: missing {expected}")
print("Validated AprilTag IDs 0/1 and 0.15 m pose-estimation size.")
PY

for script in "${workspace_dir}"/scripts/*.sh "${workspace_dir}"/src/robotac_bringup/scripts/*.sh; do
  bash -n "${script}"
  echo "Validated shell syntax: ${script##*/}"
done
