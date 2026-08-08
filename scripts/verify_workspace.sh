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
  src/robotac_flight/launch/ev_acceptance_observer.launch
  src/robotac_flight/launch/local_flight_preflight.launch
  src/robotac_flight/scripts/audit_local_mission.py
  src/robotac_flight/scripts/ev_acceptance_observer.py
  src/robotac_flight/scripts/local_flight_readiness.py
  src/robotac_flight/scripts/preview_local_route.py
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
  scripts/analyze_readonly_flight_evidence.py
  scripts/collect_readonly_flight_evidence.sh
  scripts/flight_test_ladder.sh
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

python3 - "${workspace_dir}/config/mavros/px4_pluginlists.yaml" \
  "${workspace_dir}/src/robotac_bringup/launch/mavros_px4.launch" <<'PY'
import pathlib
import sys

import yaml

plugin_path = pathlib.Path(sys.argv[1])
launch_path = pathlib.Path(sys.argv[2])
plugins = yaml.safe_load(plugin_path.read_text()) or {}
whitelist = set(plugins.get("plugin_whitelist") or [])
blacklist = set(plugins.get("plugin_blacklist") or [])
required = {
    "command",
    "imu",
    "local_position",
    "param",
    "setpoint_raw",
    "sys_status",
    "sys_time",
    "vision_pose_estimate",
}
for name in sorted(required):
    if name not in whitelist:
        raise SystemExit(f"MAVROS local-only plugin check failed: missing {name}")
for name in ("global_position", "gps_status", "waypoint"):
    if name in whitelist:
        raise SystemExit(f"MAVROS local-only plugin check failed: {name} is whitelisted")
    if name not in blacklist:
        raise SystemExit(f"MAVROS local-only plugin check failed: {name} is not blacklisted")
launch_source = launch_path.read_text()
if '<arg name="check_geographiclib" default="false" />' not in launch_source:
    raise SystemExit("MAVROS launch must not require GeographicLib for local-only plugin list")
print("Validated MAVROS local-only plugin surface and optional GeographicLib check.")
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

python3 - "${workspace_dir}/scripts" <<'PY'
import pathlib
import py_compile
import sys

for path in sorted(pathlib.Path(sys.argv[1]).glob("*.py")):
    py_compile.compile(str(path), doraise=True)
    print(f"Validated top-level Python syntax: {path.name}")
PY

for script in \
  "${workspace_dir}/src/robotac_flight/test/run_closed_loop_sim.sh" \
  "${workspace_dir}/src/robotac_flight/test/run_dynamic_waypoints_sim.sh" \
  "${workspace_dir}/src/robotac_flight/test/run_flight_fault_sim.sh" \
  "${workspace_dir}/src/robotac_flight/test/run_flight_preflight_sim.sh" \
  "${workspace_dir}/src/robotac_flight/test/run_setpoint_consumer_gate_sim.sh" \
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

route_preview=$(python3 "${workspace_dir}/src/robotac_flight/scripts/preview_local_route.py" \
  --file "${workspace_dir}/config/flight/local_waypoints.yaml" \
  --origin-x 3.0 --origin-y -2.0 --origin-yaw-deg 90.0)
printf '%s\n' "${route_preview}"
[[ "${route_preview}" == *"target_enu_route=(3.000,-2.000,1.000)->(3.000,-1.000,1.000)->(3.000,-2.000,1.000)->(2.000,-2.000,1.000)->(3.000,-2.000,1.000)->(4.000,-2.000,1.000)->(3.000,-2.000,1.000)->(3.000,-3.000,1.000)->(3.000,-2.000,1.000)"* ]]
[[ "${route_preview}" == *"mavlink_ned_route=(-2.000,3.000,-1.000)->(-1.000,3.000,-1.000)->(-2.000,3.000,-1.000)->(-2.000,2.000,-1.000)->(-2.000,3.000,-1.000)->(-2.000,4.000,-1.000)->(-2.000,3.000,-1.000)->(-3.000,3.000,-1.000)->(-2.000,3.000,-1.000)"* ]]
[[ "${route_preview}" == *"payload_events=wp6:open@(3.000,-3.000,1.000)"* ]]
echo "Validated offline local route preview."

mission_audit=$(python3 "${workspace_dir}/src/robotac_flight/scripts/audit_local_mission.py" \
  --file "${workspace_dir}/config/flight/local_waypoints.yaml" \
  --origin-x 3.0 --origin-y -2.0 --origin-yaw-deg 90.0 \
  --require-payload-open)
printf '%s\n' "${mission_audit}"
[[ "${mission_audit}" == *"MISSION_AUDIT_PASS"* ]]
[[ "${mission_audit}" == *"frame=robotac_start_body waypoints=8 targets_with_takeoff=9"* ]]
[[ "${mission_audit}" == *"takeoff_height_m=1.000 auto_land_required=True land_mode=AUTO.LAND"* ]]
[[ "${mission_audit}" == *"payload_events=wp6:open@(3.000,-3.000,1.000)"* ]]
[[ "${mission_audit}" == *"global_coordinates=forbidden"* ]]
echo "Validated offline local mission audit."

python3 - "${workspace_dir}/src/robotac_flight/scripts/audit_local_mission.py" <<'PY'
import pathlib
import subprocess
import sys
import tempfile

with tempfile.TemporaryDirectory(prefix="robotac-mission-audit.") as directory:
    route = pathlib.Path(directory) / "global.yaml"
    route.write_text("""local_waypoint_flight:
  waypoint_frame: robotac_start_body
  require_auto_land: true
  waypoints:
    - {x: 0.0, y: 0.0, z: 1.0, latitude: 30.0}
""", encoding="utf-8")
    result = subprocess.run(
        [sys.executable, sys.argv[1], "--file", str(route)],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False)
    if result.returncode == 0:
        raise SystemExit("Mission audit unexpectedly accepted a global/GPS waypoint key.")
PY
echo "Validated local mission audit rejects global/GPS keys."

readiness_report=$(python3 "${workspace_dir}/src/robotac_flight/scripts/local_flight_readiness.py" \
  --config-root "${workspace_dir}/config" \
  --origin-x 3.0 --origin-y -2.0 --origin-yaw-deg 90.0)
printf '%s\n' "${readiness_report}"
[[ "${readiness_report}" == *"LOCAL_FLIGHT_READINESS"* ]]
[[ "${readiness_report}" == *"local_mission_file=READY"* ]]
[[ "${readiness_report}" == *"mavros_local_only=READY"* ]]
[[ "${readiness_report}" == *"fastlio_vision_bridge_config=BLOCKED"* ]]
[[ "${readiness_report}" == *"vision_output=BLOCKED"* ]]
[[ "${readiness_report}" == *"active_local_flight=BLOCKED"* ]]
[[ "${readiness_report}" == *"payload_local_flight=BLOCKED"* ]]
if python3 "${workspace_dir}/src/robotac_flight/scripts/local_flight_readiness.py" \
  --config-root "${workspace_dir}/config" --require-phase active_local_flight >/dev/null 2>&1; then
  echo "Readiness report unexpectedly marked active local flight ready." >&2
  exit 1
fi
echo "Validated offline local flight readiness report."

python3 - "${workspace_dir}/config/flight/local_waypoints.yaml" \
  "${workspace_dir}/src/mavros/mavros/src/plugins/setpoint_raw.cpp" \
  "${workspace_dir}/src/fast_lio/src/laserMapping.cpp" \
  "${workspace_dir}/src/robotac_flight/test/flight_closed_loop_sim.py" \
  "${workspace_dir}/config/fastlio/vision_bridge.yaml" \
  "${workspace_dir}/src/robotac_flight/scripts/local_flight_preflight.py" \
  "${workspace_dir}/src/robotac_flight/scripts/local_waypoint_flight.py" \
  "${workspace_dir}/src/robotac_flight/scripts/fastlio_vision_bridge.py" \
  "${workspace_dir}/src/robotac_flight/test/run_dynamic_waypoints_sim.sh" \
  "${workspace_dir}/src/robotac_flight/test/run_flight_preflight_sim.sh" \
  "${workspace_dir}/src/robotac_flight/scripts/check_px4_vision_config.py" \
  "${workspace_dir}/src/robotac_flight/test/run_setpoint_consumer_gate_sim.sh" \
  "${workspace_dir}/src/robotac_flight/scripts/ev_acceptance_observer.py" \
  "${workspace_dir}/src/robotac_flight/test/run_flight_fault_sim.sh" <<'PY'
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
px4_check_source = pathlib.Path(sys.argv[11]).read_text()
setpoint_gate_test = pathlib.Path(sys.argv[12]).read_text()
ev_acceptance_source = pathlib.Path(sys.argv[13]).read_text()
flight_fault_test = pathlib.Path(sys.argv[14]).read_text()
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
    "setpoint_topic: /mavros/setpoint_raw/local",
    "require_setpoint_consumer: true",
    "setpoint_consumer_node: /mavros",
    "consumer_check_interval: 0.50",
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
    "vision_consumer_loss",
    "self.vision_pose_sub.unregister()",
    "setpoint_consumer_loss",
    "self.setpoint_sub.unregister()",
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
    "_vision_output_consumer_present",
    "getSystemState",
    "require_vision_output_consumer",
    "vision_output_consumer_node",
    "vision_output_consumer_issue",
    "require_setpoint_consumer",
    "setpoint_consumer_node",
    "setpoint_consumer_issue",
    "setpoint_topic",
    "require_ev_offsets_zero",
    "ev_offset_tolerance_m",
    "require_ev_delay",
    "expected_ev_delay_ms",
    "ev_delay_tolerance_ms",
    "require_ev_delay requires check_px4_vision_params=true",
    "EKF2_EV_POS_X",
    "EKF2_EV_DELAY",
    "EV_POS_nonfinite",
    "EV_POS_nonzero",
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
    "waypoint %d must use yaw or yaw_deg, not both",
    '"yaw": yaw',
    '"waypoint %d yaw_deg"',
    '"waypoint %d yaw"',
):
    if expected not in flight_source:
        raise SystemExit(f"Configured waypoint yaw parsing check failed: missing {expected}")
if "math.radians(float(item.get(\"yaw_deg\", item.get(\"yaw\", 0.0))))" in flight_source:
    raise SystemExit("Configured waypoint yaw parser must not treat yaw radians as degrees")
for expected in (
    "PoseWithCovarianceStamped",
    "vision_status_receive_time",
    "vision_output_receive_time",
    "mavros_vision_pose_timeout",
    "TimesyncStatus",
    "mavros_timesync_stale",
    "mavros_vision_pose_consumer_unavailable",
    "mavros_setpoint_raw_consumer_unavailable",
    "mavros_vision_pose_consumer_lost",
    "mavros_setpoint_raw_consumer_lost",
    "consumer_check_interval",
    "invalid_consumer_check_interval",
    "_active_consumer_issue",
    "_setpoint_consumer_present",
    "setpoint_topic",
):
    if expected not in flight_source:
        raise SystemExit(f"Flight vision-output gate check failed: missing {expected}")
for expected in (
    "if self.enable_control else None",
    "if self.enable_payload else None",
    "self.setpoint_pub is not None",
):
    if expected not in flight_source:
        raise SystemExit(f"Flight dry-run publisher isolation check failed: missing {expected}")
if "self.setpoint_pub = rospy.Publisher(self.setpoint_topic" in flight_source:
    raise SystemExit("Dry-run flight node must not unconditionally register MAVROS setpoint publisher")
if "self.payload_pub = rospy.Publisher(self.payload_topic" in flight_source:
    raise SystemExit("Dry-run flight node must not unconditionally register payload publisher")
for name, source in (("flight controller", flight_source),
                     ("vision bridge", bridge_source),
                     ("read-only preflight", preflight_source)):
    for forbidden in (
        "CommandTOL",
        "GlobalPositionTarget",
        "NavSatFix",
        "GeoPose",
        "FRAME_GLOBAL",
        "WaypointPush",
        "WaypointPull",
        "WaypointSetCurrent",
        "/mavros/global_position",
        "/mavros/mission",
        "/mavros/setpoint_position",
        "/mavros/setpoint_velocity",
        "/mavros/setpoint_attitude",
        "latitude",
        "longitude",
    ):
        if forbidden in source:
            raise SystemExit(
                f"{name} must stay local-only; found forbidden token {forbidden}")
if "from mavros_msgs.srv import CommandBool, SetMode" not in flight_source:
    raise SystemExit("Flight controller should expose only MAVROS arming/mode services")
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
    "require_setpoint_consumer:=true",
    "setpoint_consumer_node:=/robotac_flight_dryrun_inputs",
    "vision_output_consumer_node:=/robotac_flight_dryrun_inputs",
    "vision_output_topic:=/robotac/test/vision_pose",
):
    if expected not in preflight_test:
        raise SystemExit(f"Preflight regression check failed: missing {expected}")
for expected in (
    "require_ev_offsets_zero",
    "ev_offset_tolerance_m",
    "require_ev_delay",
    "expected_ev_delay_ms",
    "ev_delay_tolerance_ms",
    "EKF2_EV_POS_X",
    "EKF2_EV_DELAY",
    "EV_POS offsets must be finite",
    "EV_POS offsets must be zero",
):
    if expected not in px4_check_source:
        raise SystemExit(f"PX4 vision config check failed: missing {expected}")
for expected in (
    "Read-only external-vision acceptance observer",
    "rospy.Subscriber(",
    "mavros_local_position",
    "mavros_vision_pose_input",
    "local_delta",
    "vision_delta",
    "delta_direction_cos",
    "delta_scale",
    "min_motion_m",
    "require_disarmed",
    "require_on_ground",
):
    if expected not in ev_acceptance_source:
        raise SystemExit(f"EV acceptance observer check failed: missing {expected}")
for forbidden in (
    "rospy.Publisher(",
    "ServiceProxy",
    "SetMode",
    "CommandBool",
    "/mavros/setpoint_raw/local",
):
    if forbidden in ev_acceptance_source:
        raise SystemExit(f"EV acceptance observer must remain read-only; found {forbidden}")
for expected in (
    "_subscribe_setpoint:=false",
    "success: False",
    "mavros_setpoint_raw_consumer_unavailable",
    "local_waypoint_flight.launch",
    "enable_control:=true",
):
    if expected not in setpoint_gate_test:
        raise SystemExit(f"Setpoint consumer gate regression check failed: missing {expected}")
for expected in (
    "vision_consumer_loss",
    "mavros_vision_pose_consumer_lost",
    "setpoint_consumer_loss",
    "mavros_setpoint_raw_consumer_lost",
    "post_abort_setpoints=0",
):
    if expected not in flight_fault_test:
        raise SystemExit(f"Flight fault regression check failed: missing {expected}")
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

python3 - "${workspace_dir}/scripts/flight_test_ladder.sh" <<'PY'
import pathlib
import re
import sys

path = pathlib.Path(sys.argv[1])
source = path.read_text()
for expected in (
    "This script never starts ROS nodes",
    "--show-active",
    "--evidence-dir",
    "Deployment gates are not all true",
    "Readiness report did not pass active_local_flight",
    "Read-only evidence did not pass active_preflight_evidence",
    "Evidence directory is required for --show-active",
    "Payload readiness did not pass payload_local_flight",
    "rosservice call /robotac/flight/start",
    "enable_flight_controller:=false",
    "flight_auto_arm:=true",
):
    if expected not in source:
        raise SystemExit(f"Flight test ladder check failed: missing {expected}")

in_heredoc = False
delimiter = None
for number, line in enumerate(source.splitlines(), start=1):
    stripped = line.strip()
    if in_heredoc:
        if stripped == delimiter:
            in_heredoc = False
            delimiter = None
        continue
    match = re.search(r"<<'?([A-Za-z_][A-Za-z0-9_]*)'?", line)
    if match:
        in_heredoc = True
        delimiter = match.group(1)
        continue
    if re.match(r"^(roslaunch|rosservice|rostopic|rosrun)\b", stripped):
        raise SystemExit(
            f"Flight test ladder must only print ROS commands; line {number} executes: {stripped}")
print("Validated flight test ladder remains offline/read-only by default.")
PY

python3 - "${workspace_dir}" <<'PY'
import pathlib
import shutil
import subprocess
import sys
import tempfile

workspace = pathlib.Path(sys.argv[1])
with tempfile.TemporaryDirectory(prefix="robotac-ladder-readiness.") as directory:
    config_root = pathlib.Path(directory) / "config"
    shutil.copytree(str(workspace / "config"), str(config_root))
    shutil.copyfile(str(workspace / "config" / "deployment_sim.yaml"),
                    str(config_root / "deployment.yaml"))
    result = subprocess.run(
        [str(workspace / "scripts" / "flight_test_ladder.sh"),
         "--config-root", str(config_root), "--skip-verify", "--show-active"],
        text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    if result.returncode != 2:
        raise SystemExit("Flight ladder should fail closed when readiness is blocked despite deployment gates")
    if "Readiness report did not pass active_local_flight" not in result.stderr:
        raise SystemExit("Flight ladder did not report active readiness failure")
    if "active flight commands (not executed by this script)" in result.stdout:
        raise SystemExit("Flight ladder printed active commands before readiness passed")
print("Validated flight ladder requires readiness before active command printing.")
PY

python3 - "${workspace_dir}" <<'PY'
import pathlib
import shutil
import subprocess
import sys
import tempfile

workspace = pathlib.Path(sys.argv[1])
expected_types = {
    "mavros_state": "mavros_msgs/State",
    "mavros_extended_state": "mavros_msgs/ExtendedState",
    "mavros_local_position_odom": "nav_msgs/Odometry",
    "mavros_estimator_status": "mavros_msgs/EstimatorStatus",
    "mavros_timesync_status": "mavros_msgs/TimesyncStatus",
    "mavros_vision_pose_pose_cov": "geometry_msgs/PoseWithCovarianceStamped",
    "mavros_setpoint_raw_local": "mavros_msgs/PositionTarget",
    "robotac_fastlio_vision_healthy": "std_msgs/Bool",
    "robotac_fastlio_vision_status": "std_msgs/String",
    "robotac_fastlio_vision_output_enabled": "std_msgs/Bool",
    "robotac_fastlio_vision_pose_preview": "geometry_msgs/PoseWithCovarianceStamped",
    "Odometry": "nav_msgs/Odometry",
}

def write(path, text):
    path.write_text(text, encoding="utf-8")

def write_valid_evidence(root):
    for safe, message_type in expected_types.items():
        write(root / f"topic_type_{safe}.txt", f"### type\n{message_type}\n")
        write(root / f"topic_hz_{safe}.txt", "average rate: 10.000\n")
        write(root / f"topic_info_{safe}.txt",
              "Type: %s\nPublishers:\n * /producer\nSubscribers:\n * /listener\n" % message_type)
        write(root / f"topic_echo_{safe}.txt", "---\n")
    write(root / "topic_echo_mavros_state.txt", "connected: True\narmed: False\nmode: MANUAL\n")
    write(root / "topic_echo_mavros_extended_state.txt", "landed_state: 1\n")
    write(root / "topic_echo_robotac_fastlio_vision_healthy.txt", "data: True\n")
    write(root / "topic_echo_robotac_fastlio_vision_status.txt",
          "data: ok rate_hz=10.0 valid=20 dropped=0\n")
    write(root / "topic_echo_robotac_fastlio_vision_output_enabled.txt", "data: True\n")
    write(root / "topic_info_mavros_vision_pose_pose_cov.txt",
          "Type: geometry_msgs/PoseWithCovarianceStamped\nPublishers:\n * /fastlio_vision_bridge\nSubscribers:\n * /mavros\n")
    write(root / "topic_info_mavros_setpoint_raw_local.txt",
          "Type: mavros_msgs/PositionTarget\nPublishers: None\nSubscribers:\n * /mavros\n")

with tempfile.TemporaryDirectory(prefix="robotac-ladder-evidence.") as directory:
    root = pathlib.Path(directory)
    config_root = root / "config"
    evidence = root / "evidence"
    evidence.mkdir()
    shutil.copytree(str(workspace / "config"), str(config_root))
    shutil.copyfile(str(workspace / "config" / "deployment_sim.yaml"),
                    str(config_root / "deployment.yaml"))
    shutil.copyfile(str(workspace / "config" / "fastlio" / "vision_bridge_sim.yaml"),
                    str(config_root / "fastlio" / "vision_bridge.yaml"))
    missing_evidence = subprocess.run(
        [str(workspace / "scripts" / "flight_test_ladder.sh"),
         "--config-root", str(config_root), "--skip-verify", "--show-active"],
        text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    if missing_evidence.returncode != 2:
        raise SystemExit("Flight ladder should require --evidence-dir when active readiness passes")
    if "Evidence directory is required for --show-active" not in missing_evidence.stderr:
        raise SystemExit("Flight ladder did not report missing evidence directory")

    write_valid_evidence(evidence)
    ready = subprocess.run(
        [str(workspace / "scripts" / "flight_test_ladder.sh"),
         "--config-root", str(config_root), "--skip-verify", "--show-active",
         "--evidence-dir", str(evidence)],
        text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    if ready.returncode != 0:
        raise SystemExit("Flight ladder rejected valid synthetic readiness/evidence:\n%s\n%s" %
                         (ready.stdout, ready.stderr))
    if "active flight commands (not executed by this script)" not in ready.stdout:
        raise SystemExit("Flight ladder did not print active command block after valid evidence")
    if "payload flight commands (not executed by this script)" not in ready.stdout:
        raise SystemExit("Flight ladder did not print payload command block after valid payload readiness")
print("Validated flight ladder requires and accepts read-only evidence before active command printing.")
PY

python3 - "${workspace_dir}/scripts/collect_readonly_flight_evidence.sh" <<'PY'
import pathlib
import re
import sys

source = pathlib.Path(sys.argv[1]).read_text()
for expected in (
    "Read-only evidence collector",
    "rostopic info",
    "rostopic hz",
    "rostopic echo -n 1",
    "rosbag record",
    "safety=no_roslaunch_no_rosservice_no_rostopic_pub_no_setpoints_no_arming_no_mode_change",
):
    if expected not in source:
        raise SystemExit(f"Read-only evidence collector check failed: missing {expected}")
for number, line in enumerate(source.splitlines(), start=1):
    stripped = line.strip()
    if re.match(r"^(roslaunch|rosservice)\b", stripped):
        raise SystemExit(
            f"Read-only evidence collector must not execute {stripped} on line {number}")
    if re.match(r"^rostopic\s+pub\b", stripped) or " rostopic pub " in line:
        raise SystemExit("Read-only evidence collector must not publish topics")
for forbidden in ("/mavros/cmd/arming", "/mavros/set_mode", "/robotac/flight/start"):
    if forbidden in source:
        raise SystemExit(f"Read-only evidence collector must not reference control path {forbidden}")
print("Validated read-only flight evidence collector safety surface.")
PY

python3 - "${workspace_dir}/scripts/analyze_readonly_flight_evidence.py" <<'PY'
import pathlib
import re
import sys

source = pathlib.Path(sys.argv[1]).read_text()
for expected in (
    "READ_ONLY_EVIDENCE_ANALYSIS",
    "active_preflight_evidence",
    "mavros_vision_pose_consumer",
    "mavros_setpoint_raw_consumer",
    "read_only_no_setpoint_publishers",
    "connected/disarmed/on-ground",
):
    if expected not in source:
        raise SystemExit(f"Read-only evidence analyzer check failed: missing {expected}")
for forbidden in (
    "rospy.Publisher",
    "ServiceProxy",
    "rosservice",
    "rostopic pub",
    "/mavros/cmd/arming",
    "/mavros/set_mode",
    "/robotac/flight/start",
):
    if forbidden in source:
        raise SystemExit(f"Read-only evidence analyzer must not reference control path {forbidden}")
if re.search(r"subprocess\.run|subprocess\.Popen|os\.system", source):
    raise SystemExit("Read-only evidence analyzer must only read files, not execute commands")
print("Validated read-only flight evidence analyzer safety surface.")
PY

python3 - "${workspace_dir}/scripts/analyze_readonly_flight_evidence.py" <<'PY'
import pathlib
import subprocess
import sys
import tempfile

script = sys.argv[1]
expected_types = {
    "mavros_state": "mavros_msgs/State",
    "mavros_extended_state": "mavros_msgs/ExtendedState",
    "mavros_local_position_odom": "nav_msgs/Odometry",
    "mavros_estimator_status": "mavros_msgs/EstimatorStatus",
    "mavros_timesync_status": "mavros_msgs/TimesyncStatus",
    "mavros_vision_pose_pose_cov": "geometry_msgs/PoseWithCovarianceStamped",
    "mavros_setpoint_raw_local": "mavros_msgs/PositionTarget",
    "robotac_fastlio_vision_healthy": "std_msgs/Bool",
    "robotac_fastlio_vision_status": "std_msgs/String",
    "robotac_fastlio_vision_output_enabled": "std_msgs/Bool",
    "robotac_fastlio_vision_pose_preview": "geometry_msgs/PoseWithCovarianceStamped",
    "Odometry": "nav_msgs/Odometry",
}

def write(path, text):
    path.write_text(text, encoding="utf-8")

with tempfile.TemporaryDirectory(prefix="robotac-evidence-analysis.") as directory:
    root = pathlib.Path(directory)
    for safe, message_type in expected_types.items():
        write(root / f"topic_type_{safe}.txt", f"### type\n{message_type}\n")
        write(root / f"topic_hz_{safe}.txt", "average rate: 10.000\n")
        write(root / f"topic_info_{safe}.txt", "Type: %s\nPublishers:\n * /producer\nSubscribers:\n * /listener\n" % message_type)
        write(root / f"topic_echo_{safe}.txt", "---\n")
    write(root / "topic_echo_mavros_state.txt", "connected: True\narmed: False\nmode: MANUAL\n")
    write(root / "topic_echo_mavros_extended_state.txt", "landed_state: 1\n")
    write(root / "topic_echo_robotac_fastlio_vision_healthy.txt", "data: True\n")
    write(root / "topic_echo_robotac_fastlio_vision_status.txt", "data: ok rate_hz=10.0 valid=20 dropped=0\n")
    write(root / "topic_echo_robotac_fastlio_vision_output_enabled.txt", "data: True\n")
    write(root / "topic_info_mavros_vision_pose_pose_cov.txt",
          "Type: geometry_msgs/PoseWithCovarianceStamped\nPublishers:\n * /fastlio_vision_bridge\nSubscribers:\n * /mavros\n")
    write(root / "topic_info_mavros_setpoint_raw_local.txt",
          "Type: mavros_msgs/PositionTarget\nPublishers: None\nSubscribers:\n * /mavros\n")
    ok = subprocess.run([sys.executable, script, str(root)], text=True,
                        stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    if ok.returncode != 0 or "active_preflight_evidence=READY" not in ok.stdout:
        raise SystemExit("Read-only evidence analyzer unexpectedly failed valid synthetic evidence:\n%s\n%s" % (ok.stdout, ok.stderr))
    write(root / "topic_info_mavros_vision_pose_pose_cov.txt",
          "Type: geometry_msgs/PoseWithCovarianceStamped\nPublishers:\n * /fastlio_vision_bridge\nSubscribers:\n * /rviz\n")
    bad = subprocess.run([sys.executable, script, str(root), "--require-phase", "vision_to_mavros"],
                         text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    if bad.returncode == 0 or "mavros_vision_pose_consumer=BLOCKED" not in bad.stdout:
        raise SystemExit("Read-only evidence analyzer unexpectedly accepted missing MAVROS vision consumer")
    write(root / "topic_info_mavros_vision_pose_pose_cov.txt",
          "Type: geometry_msgs/PoseWithCovarianceStamped\nPublishers:\n * /fastlio_vision_bridge\nSubscribers:\n * /mavros\n")
    write(root / "topic_info_mavros_setpoint_raw_local.txt",
          "Type: mavros_msgs/PositionTarget\nPublishers:\n * /local_waypoint_flight\nSubscribers:\n * /mavros\n")
    unsafe = subprocess.run([sys.executable, script, str(root)], text=True,
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    if unsafe.returncode == 0 or "read_only_no_setpoint_publishers=BLOCKED" not in unsafe.stdout:
        raise SystemExit("Read-only evidence analyzer unexpectedly accepted a setpoint publisher")
print("Validated read-only flight evidence analyzer with synthetic evidence.")
PY

for script in "${workspace_dir}"/scripts/*.sh "${workspace_dir}"/src/robotac_bringup/scripts/*.sh; do
  bash -n "${script}"
  echo "Validated shell syntax: ${script##*/}"
done
