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
  src/robotac_servo
  config/lidar/mid360s.json
  config/camera/rgb.yaml
  config/fastlio/mid360s.yaml
  config/mavros/px4.yaml
  config/apriltag/settings.yaml
  config/apriltag/tags.yaml
  config/deployment.yaml
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

python3 - "${workspace_dir}/config/apriltag/tags.yaml" <<'PY'
import pathlib
import sys

source = pathlib.Path(sys.argv[1]).read_text()
for expected in (
    "total_size_m: 0.25",
    "tag_size_m: 0.20",
    "{id: 0, size: 0.20, name: tag_0}",
    "{id: 1, size: 0.20, name: tag_1}",
):
    if expected not in source:
        raise SystemExit(f"AprilTag config check failed: missing {expected}")
print("Validated AprilTag IDs 0/1 and 0.20 m pose-estimation size.")
PY

for script in "${workspace_dir}"/scripts/*.sh "${workspace_dir}"/src/robotac_bringup/scripts/*.sh; do
  bash -n "${script}"
  echo "Validated shell syntax: ${script##*/}"
done
