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
  src/robotac_flight/launch/local_flight_preflight.launch
  src/robotac_servo
  config/lidar/mid360s.json
  config/camera/rgb.yaml
  config/fastlio/mid360s.yaml
  config/fastlio/vision_bridge.yaml
  config/fastlio/vision_bridge_sim.yaml
  config/flight/local_waypoints.yaml
  config/flight/posearray_waypoints_example.yaml
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

for script in \
  "${workspace_dir}/src/robotac_flight/test/run_closed_loop_sim.sh" \
  "${workspace_dir}/src/robotac_flight/test/run_dynamic_waypoints_sim.sh" \
  "${workspace_dir}/src/robotac_flight/test/run_flight_fault_sim.sh" \
  "${workspace_dir}/src/robotac_flight/test/run_flight_preflight_sim.sh" \
  "${workspace_dir}/src/robotac_flight/test/run_vision_bridge_sim.sh"; do
  bash -n "${script}"
  echo "Validated simulation shell syntax: ${script##*/}"
done

python3 "${workspace_dir}/src/robotac_flight/scripts/publish_waypoints.py" \
  --file "${workspace_dir}/config/flight/posearray_waypoints_example.yaml" \
  --dry-run >/dev/null
echo "Validated position-only waypoint YAML publisher dry run."
if python3 "${workspace_dir}/src/robotac_flight/scripts/publish_waypoints.py" \
  --file "${workspace_dir}/config/flight/local_waypoints.yaml" \
  --dry-run >/dev/null 2>&1; then
  echo "Waypoint publisher unexpectedly accepted full mission metadata." >&2
  exit 1
fi
echo "Validated PoseArray publisher rejects payload/hold mission metadata by default."

python3 - "${workspace_dir}/config/flight/local_waypoints.yaml" \
  "${workspace_dir}/src/mavros/mavros/src/plugins/setpoint_raw.cpp" \
  "${workspace_dir}/src/fast_lio/src/laserMapping.cpp" \
  "${workspace_dir}/src/robotac_flight/test/flight_closed_loop_sim.py" \
  "${workspace_dir}/config/fastlio/vision_bridge.yaml" \
  "${workspace_dir}/src/robotac_flight/scripts/local_flight_preflight.py" \
  "${workspace_dir}/src/robotac_flight/scripts/local_waypoint_flight.py" \
  "${workspace_dir}/src/robotac_flight/scripts/fastlio_vision_bridge.py" \
  "${workspace_dir}/src/robotac_flight/test/run_dynamic_waypoints_sim.sh" \
  "${workspace_dir}/src/robotac_flight/test/run_flight_preflight_sim.sh" <<'PY'
import pathlib
import sys

flight_config = pathlib.Path(sys.argv[1]).read_text()
mavros_source = pathlib.Path(sys.argv[2]).read_text()
fastlio_source = pathlib.Path(sys.argv[3]).read_text()
sim_source = pathlib.Path(sys.argv[4]).read_text()
vision_config = pathlib.Path(sys.argv[5]).read_text()
preflight_source = pathlib.Path(sys.argv[6]).read_text()
flight_source = pathlib.Path(sys.argv[7]).read_text()
bridge_source = pathlib.Path(sys.argv[8]).read_text()
dynamic_waypoint_test = pathlib.Path(sys.argv[9]).read_text()
preflight_test = pathlib.Path(sys.argv[10]).read_text()
for expected in (
    "waypoint_frame: robotac_start_body",
    "strict_local_frames: true",
    "expected_local_parent: map",
    "expected_local_child: base_link",
    "payload_action: open",
    "payload_topic: /robotac/servo/open",
    "payload_status_topic: /robotac/servo/status",
    "payload_required_connection: true",
    "payload_require_ack: true",
    "payload_ack_timeout: 1.0",
    "critical_fault_action: release",
    "operator_abort_action: release",
    "require_auto_land: true",
    "vision_status_timeout: 0.50",
    "vision_output_timeout: 0.50",
    "vision_output_parent: odom",
    "vision_output_consumer_node: /mavros",
    "require_timesync: true",
):
    if expected not in flight_config:
        raise SystemExit(f"Flight route check failed: missing {expected}")
for expected in (
    "position = ftf::transform_frame_enu_ned(position);",
    "transform_orientation_ned_enu(",
    "transform_orientation_aircraft_baselink(",
):
    if expected not in mavros_source:
        raise SystemExit(f"MAVROS local-setpoint conversion check failed: missing {expected}")
for expected in (
    "def _enu_to_ned_target",
    "def _ned_to_enu_target",
    "mavlink_ned_route=",
):
    if expected not in sim_source:
        raise SystemExit(f"Flight simulation conversion check failed: missing {expected}")
if "output_child_frame" in vision_config:
    raise SystemExit("Vision bridge must keep fixed implicit base_link pose semantics")
for expected in (
    "for (int j = 0; j < 6; j ++)",
    "odomAftMapped.pose.covariance[i * 6 + j] = P(i, j);",
):
    if expected not in fastlio_source:
        raise SystemExit(f"FAST-LIO covariance layout check failed: missing {expected}")
if "int k = i < 3 ? i + 3 : i - 3;" in fastlio_source:
    raise SystemExit("FAST-LIO covariance layout must not swap position/rotation blocks")
for expected in (
    "vision_status_receive",
    'self.vision_status.startswith("ok ")',
    "ros_clock_unavailable",
    "_matching_stamp",
    "TimesyncStatus",
    "timesync_issue",
):
    if expected not in preflight_source:
        raise SystemExit(f"Flight preflight check failed: missing {expected}")
if "rospy.Publisher(" in preflight_source:
    raise SystemExit("Flight preflight must remain subscriber-only")
for source, name in ((flight_source, "flight controller"),
                     (bridge_source, "vision bridge")):
    if "ros_clock_unavailable" not in source:
        raise SystemExit(f"{name} must reject an unavailable ROS clock")
if "msg.header.frame_id != self.input_frame" not in flight_source:
    raise SystemExit("Dynamic waypoint messages must require their declared frame")
for expected in (
    "PoseWithCovarianceStamped",
    "vision_status_receive_time",
    "vision_output_receive_time",
    "mavros_vision_pose_timeout",
    "TimesyncStatus",
    "mavros_timesync_stale",
    "mavros_vision_pose_consumer_unavailable",
):
    if expected not in flight_source:
        raise SystemExit(f"Flight vision-output gate check failed: missing {expected}")
for expected in (
    "scripts/publish_waypoints.py",
    "config/flight/posearray_waypoints_example.yaml",
    "waypoints_loaded=6",
):
    if expected not in dynamic_waypoint_test:
        raise SystemExit(f"Dynamic waypoint regression check failed: missing {expected}")
for expected in (
    "local_flight_preflight.launch",
    "require_vision_output:=true",
    "vision_output_topic:=/robotac/test/vision_pose",
):
    if expected not in preflight_test:
        raise SystemExit(f"Preflight regression check failed: missing {expected}")
print("Validated local ENU/MAVROS-NED route and vision-pose semantics.")
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
