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
  src/robotac_flight/launch/active_flight_observer.launch
  src/robotac_flight/launch/ev_acceptance_observer.launch
  src/robotac_flight/launch/local_flight_preflight.launch
  src/robotac_flight/scripts/active_flight_observer.py
  src/robotac_flight/scripts/audit_local_mission.py
  src/robotac_flight/scripts/create_route_file.py
  src/robotac_flight/scripts/ev_acceptance_observer.py
  src/robotac_flight/scripts/local_flight_readiness.py
  src/robotac_flight/scripts/preview_local_route.py
  src/robotac_flight/test/run_route_file_sim.sh
  src/robotac_servo
  config/lidar/mid360s.json
  config/camera/rgb.yaml
  config/fastlio/mid360s.yaml
  config/fastlio/vision_bridge.yaml
  config/fastlio/vision_bridge_sim.yaml
  config/flight/local_waypoints.yaml
  config/flight/local_waypoints_simple_box.yaml
  config/flight/posearray_waypoints_example.yaml
  config/mavros/px4.yaml
  config/apriltag/settings.yaml
  config/apriltag/tags.yaml
  config/deployment.yaml
  config/deployment_sim.yaml
  config/udev/99-robotac-servo.rules.template
  config/udev/99-robotac-rgb-camera.rules.template
  scripts/analyze_active_flight_evidence.py
  scripts/analyze_readonly_flight_evidence.py
  scripts/check_flight_contract.py
  scripts/collect_readonly_flight_evidence.sh
  scripts/flight_goal_audit.py
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
  "${workspace_dir}/src/robotac_flight/test/run_route_file_sim.sh" \
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

generated_route_dir=$(mktemp -d "${TMPDIR:-/tmp}/robotac-route-generator.XXXXXX")
generated_route="${generated_route_dir}/generated_payload_route.yaml"
python3 "${workspace_dir}/src/robotac_flight/scripts/create_route_file.py" \
  --output "${generated_route}" \
  --point 1,0,1,0 \
  --point 0,0,1,0 \
  --point 0,1,1,0 \
  --point 0,0,1,0 \
  --point 0,-1,1,0 \
  --point 0,0,1,0 \
  --point=-1,0,1,0 \
  --payload-open-index 6 \
  --append-return-home >/dev/null
generated_route_preview=$(python3 "${workspace_dir}/src/robotac_flight/scripts/preview_local_route.py" \
  --file "${generated_route}" \
  --origin-x 3.0 --origin-y -2.0 --origin-yaw-deg 90.0)
printf '%s\n' "${generated_route_preview}"
[[ "${generated_route_preview}" == *"payload_events=wp6:open@(3.000,-3.000,1.000)"* ]]
generated_route_audit=$(python3 "${workspace_dir}/src/robotac_flight/scripts/audit_local_mission.py" \
  --file "${generated_route}" \
  --origin-x 3.0 --origin-y -2.0 --origin-yaw-deg 90.0 \
  --require-payload-open)
printf '%s\n' "${generated_route_audit}"
[[ "${generated_route_audit}" == *"MISSION_AUDIT_PASS"* ]]
[[ "${generated_route_audit}" == *"frame=robotac_start_body waypoints=8 targets_with_takeoff=9"* ]]
generated_readiness=$(python3 "${workspace_dir}/src/robotac_flight/scripts/local_flight_readiness.py" \
  --config-root "${workspace_dir}/config" \
  --route-file "${generated_route}")
printf '%s\n' "${generated_readiness}"
[[ "${generated_readiness}" == *"local_mission_file=READY"* ]]
python3 - "${workspace_dir}/src/robotac_flight/scripts/create_route_file.py" <<'PY'
import pathlib
import subprocess
import sys
import tempfile

with tempfile.TemporaryDirectory(prefix="robotac-route-generator-bad.") as directory:
    waypoint_file = pathlib.Path(directory) / "bad.yaml"
    waypoint_file.write_text("""waypoints:
  - {x: 0.0, y: 0.0, z: 1.0, latitude: 30.0}
""", encoding="utf-8")
    result = subprocess.run(
        [sys.executable, sys.argv[1], "--input", str(waypoint_file), "--dry-run"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False)
    if result.returncode == 0:
        raise SystemExit("Route generator unexpectedly accepted a global/GPS key.")
PY
rm -rf "${generated_route_dir}"
echo "Validated local route-file generator."

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

simple_mission_audit=$(python3 "${workspace_dir}/src/robotac_flight/scripts/audit_local_mission.py" \
  --file "${workspace_dir}/config/flight/local_waypoints_simple_box.yaml" \
  --require-auto-land)
printf '%s\n' "${simple_mission_audit}"
[[ "${simple_mission_audit}" == *"MISSION_AUDIT_PASS"* ]]
[[ "${simple_mission_audit}" == *"global_coordinates=forbidden"* ]]
echo "Validated simple route-file mission template."

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

contract_report=$(python3 "${workspace_dir}/scripts/check_flight_contract.py" \
  --workspace "${workspace_dir}")
printf '%s\n' "${contract_report}"
[[ "${contract_report}" == *"mavros_local_only_contract=READY"* ]]
[[ "${contract_report}" == *"local_relative_route_contract=READY"* ]]
[[ "${contract_report}" == *"waypoint_controller_contract=READY"* ]]
[[ "${contract_report}" == *"local_takeoff_landing_contract=READY"* ]]
[[ "${contract_report}" == *"route_generator_contract=READY"* ]]
[[ "${contract_report}" == *"fastlio_vision_pose_contract=READY"* ]]
[[ "${contract_report}" == *"evidence_gate_contract=READY"* ]]
echo "Validated offline local flight goal contract."
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
  "${workspace_dir}/src/robotac_flight/test/run_route_file_sim.sh" \
  "${workspace_dir}/src/robotac_flight/scripts/check_px4_vision_config.py" \
  "${workspace_dir}/src/robotac_flight/test/run_setpoint_consumer_gate_sim.sh" \
  "${workspace_dir}/src/robotac_flight/scripts/ev_acceptance_observer.py" \
  "${workspace_dir}/src/robotac_flight/scripts/active_flight_observer.py" \
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
route_file_test = pathlib.Path(sys.argv[11]).read_text()
px4_check_source = pathlib.Path(sys.argv[12]).read_text()
setpoint_gate_test = pathlib.Path(sys.argv[13]).read_text()
ev_acceptance_source = pathlib.Path(sys.argv[14]).read_text()
active_observer_source = pathlib.Path(sys.argv[15]).read_text()
flight_fault_test = pathlib.Path(sys.argv[16]).read_text()
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
    "check_px4_offboard_failsafe_params",
    "COM_OF_LOSS_T",
    "COM_OBL_RC_ACT",
    "COM_OBL_ACT",
    "allowed_offboard_loss_actions",
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
    "waypoints_empty",
    "waypoints_loaded=6",
):
    if expected not in dynamic_waypoint_test:
        raise SystemExit(f"Dynamic waypoint regression check failed: missing {expected}")
for expected in (
    "local_waypoints_simple_box.yaml",
    "route_file:=",
    "enable_payload:=false",
    "payload_open_at=none",
    "Route-file closed-loop flight simulation passed.",
):
    if expected not in route_file_test:
        raise SystemExit(f"Route-file regression check failed: missing {expected}")
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
    "check_px4_offboard_failsafe_params",
    "COM_OF_LOSS_T",
    "COM_OBL_RC_ACT",
    "COM_OBL_ACT",
    "allowed_offboard_loss_actions",
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
    "evidence_file",
    "ev_acceptance_observer",
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
    "Read-only observer for a real Robotac local waypoint flight",
    "active_flight_observer",
    "flight_status_topic",
    "setpoint_preview_topic",
    "raw_setpoint_topic",
    "raw_setpoint_count",
    "unique_raw_setpoints",
    "active_raw_setpoint_count",
    "active_unique_raw_setpoints",
    "min_active_raw_setpoints",
    "min_active_unique_raw_setpoints",
    "raw_setpoint_frame_mismatch_count",
    "raw_setpoint_expected_publisher_seen",
    "raw_setpoint_expected_publisher_missing",
    "raw_setpoint_publishers_seen",
    "local_position_topic",
    "active_vision_local_pair_count",
    "active_vision_local_max_delta_error_m",
    "require_active_vision_local_consistency",
    "require_route_manifest",
    "route_manifest_missing",
    "route_manifest_target_route_missing",
    "route_manifest_target_tolerance",
    "route_manifest_observed_target_mismatch",
    "route_manifest_raw_setpoint_missing",
    "route_manifest_raw_setpoint_order_mismatch",
    "route_manifest_active_raw_setpoint_missing",
    "route_manifest_active_raw_setpoint_order_mismatch",
    "route_status_fingerprint_mismatch",
    "active_local_flight_passed",
    "target_records",
    "target_records_unreached",
    "waypoint_reach_tolerance",
    "min_target_dwell_s",
    "max_continuous_reach_s",
    "waypoint_index",
    "waypoints_incomplete",
    "takeoff_target_record_missing",
    "waypoint_target_records_missing",
    "final_disarmed_not_confirmed",
    "final_on_ground_not_confirmed",
    "payload_open_not_observed",
):
    if expected not in active_observer_source:
        raise SystemExit(f"Active flight observer check failed: missing {expected}")
for forbidden in (
    "rospy.Publisher(",
    "ServiceProxy",
    "SetMode",
    "CommandBool",
    "/mavros/set_mode",
    "/mavros/cmd/arming",
    "/robotac/flight/start",
):
    if forbidden in active_observer_source:
        raise SystemExit(f"Active flight observer must remain read-only; found {forbidden}")
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
    "--route-file",
    "--deploy-workspace",
    "--deploy-route-file",
    "--evidence-dir",
    "deploy_workspace",
    "deploy_route_file",
    "flight_route_file",
    "ev_acceptance_observer.json",
    "check_px4_offboard_failsafe_params:=true",
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
import subprocess
import sys

workspace = pathlib.Path(sys.argv[1])
result = subprocess.run(
    [str(workspace / "scripts" / "flight_test_ladder.sh"), "--skip-verify"],
    text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
if result.returncode != 0:
    raise SystemExit("Flight ladder default print failed:\n%s\n%s" % (result.stdout, result.stderr))
for expected in (
        'deploy_workspace="${HOME}/robotac_ws"',
        'route_file="${HOME}/robotac_ws/config/flight/local_waypoints.yaml"',
        'evidence_dir="${deploy_workspace}/logs/read_only_evidence/$(date +%Y%m%d_%H%M%S)"'):
    if expected not in result.stdout:
        raise SystemExit("Flight ladder deploy-path print check failed: missing %s" % expected)
print("Validated flight ladder prints aircraft deploy paths for copy/paste commands.")
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
    write(root / "ev_acceptance_observer.json", """{
  "observer": "ev_acceptance_observer",
  "success": true,
  "reason": "ev_acceptance_passed local_delta=(0.400,0.000,0.000) vision_delta=(0.390,0.000,0.000) delta_direction_cos=1.000 delta_scale=1.026",
  "parameters": {
    "require_connected": true,
    "require_disarmed": true,
    "require_on_ground": true,
    "require_vision_output_enabled": true,
    "require_vision_status_ok": true
  },
  "metrics": {}
}
""")
    write(root / "local_flight_preflight.json", """{
  "observer": "local_flight_preflight",
  "success": true,
  "reason": "local_flight_preflight_passed",
  "summary": {
    "px4_params_checked": true,
    "px4_params_issue": "ok",
    "vision_output_consumer_issue": "ok",
    "setpoint_consumer_issue": "ok",
    "px4_param_values": {
      "EKF2_EV_CTRL": 3,
      "EKF2_EV_POS_X": 0.0,
      "EKF2_EV_POS_Y": 0.0,
      "EKF2_EV_POS_Z": 0.0,
      "COM_OF_LOSS_T": 1.0,
      "COM_OBL_RC_ACT": 4
    }
  },
  "parameters": {
    "require_vision": true,
    "require_vision_output": true,
    "require_timesync": true,
    "require_disarmed": true,
    "require_on_ground": true,
    "check_px4_vision_params": true,
    "check_px4_offboard_failsafe_params": true,
    "require_vision_output_consumer": true,
    "require_setpoint_consumer": true
  }
}
""")

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
    "--output-dir",
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
    "local_flight_preflight",
    "px4_vision_params_not_checked",
    "check_px4_vision_params",
    "check_px4_offboard_failsafe_params",
    "px4_offboard_failsafe:COM_OF_LOSS_T",
    "ev_acceptance_observer",
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

python3 - "${workspace_dir}/scripts/analyze_active_flight_evidence.py" <<'PY'
import pathlib
import re
import sys

source = pathlib.Path(sys.argv[1]).read_text()
for expected in (
    "ACTIVE_FLIGHT_EVIDENCE_ANALYSIS",
    "active_local_flight",
    "payload_local_flight",
    "active_flight_observer.json",
    "active_local_flight_passed",
    "target_records_unreached",
    "min-target-dwell-s",
    "expected_waypoints_mismatch",
    "waypoint-reach-tolerance",
    "active_vision_local_delta_error",
    "min-active-vision-local-pairs",
    "max-active-vision-local-delta-m",
    "active_route_manifest",
    "route_manifest_missing",
    "route_manifest_target_route_missing",
    "route_manifest_observed_target_mismatch",
    "route_manifest_raw_setpoint_missing",
    "route_manifest_raw_setpoint_order_mismatch",
    "route_status_fingerprint_mismatch",
    "raw_setpoint_count_below",
    "unique_raw_setpoints_below",
    "active_raw_setpoint_count_below",
    "active_unique_raw_setpoints_below",
    "raw_setpoint_frame_mismatch_count",
    "raw_setpoint_expected_publisher_missing",
    "min-active-raw-setpoints",
    "min-active-unique-raw-setpoints",
    "route-manifest-target-tolerance",
    "waypoints_incomplete",
    "final_disarmed",
    "final_on_ground",
):
    if expected not in source:
        raise SystemExit(f"Active flight evidence analyzer check failed: missing {expected}")
for forbidden in (
    "rospy",
    "rosservice",
    "roslaunch",
    "rostopic pub",
    "ServiceProxy",
    "Publisher",
    "/mavros/cmd/arming",
    "/mavros/set_mode",
    "/robotac/flight/start",
):
    if forbidden in source:
        raise SystemExit(f"Active flight evidence analyzer must not reference control path {forbidden}")
if re.search(r"subprocess\.run|subprocess\.Popen|os\.system", source):
    raise SystemExit("Active flight evidence analyzer must only read files, not execute commands")
print("Validated active flight evidence analyzer safety surface.")
PY

python3 - "${workspace_dir}/scripts/flight_goal_audit.py" <<'PY'
import pathlib
import re
import sys

source = pathlib.Path(sys.argv[1]).read_text()
for expected in (
    "ROBOTAC_FLIGHT_GOAL_AUDIT",
    "config_active_local_flight",
    "readonly_active_preflight_evidence",
    "active_local_flight_evidence",
    "payload_local_flight_evidence",
    "active_local_flight",
    "payload_local_flight",
    "preflight_evidence_file",
    "route_file",
):
    if expected not in source:
        raise SystemExit(f"Flight goal audit check failed: missing {expected}")
for forbidden in (
    "rospy",
    "rosservice",
    "roslaunch",
    "rostopic pub",
    "ServiceProxy",
    "Publisher",
    "/mavros/cmd/arming",
    "/mavros/set_mode",
    "/robotac/flight/start",
):
    if forbidden in source:
        raise SystemExit(f"Flight goal audit must not reference control path {forbidden}")
if re.search(r"subprocess\.run|subprocess\.Popen|os\.system", source):
    raise SystemExit("Flight goal audit must only read files, not execute commands")
print("Validated flight goal audit safety surface.")
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
    missing_ev = subprocess.run([sys.executable, script, str(root)], text=True,
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    if missing_ev.returncode == 0 or "ev_acceptance_observer=BLOCKED" not in missing_ev.stdout:
        raise SystemExit("Read-only evidence analyzer unexpectedly accepted missing EV acceptance evidence")
    write(root / "ev_acceptance_observer.json", """{
  "observer": "ev_acceptance_observer",
  "success": true,
  "reason": "ev_acceptance_passed local_delta=(0.400,0.000,0.000) vision_delta=(0.390,0.000,0.000) delta_direction_cos=1.000 delta_scale=1.026",
  "parameters": {
    "require_connected": true,
    "require_disarmed": true,
    "require_on_ground": true,
    "require_vision_output_enabled": true,
    "require_vision_status_ok": true
  },
  "metrics": {}
}
""")
    missing_preflight = subprocess.run([sys.executable, script, str(root)], text=True,
                                       stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    if missing_preflight.returncode == 0 or "local_flight_preflight=BLOCKED" not in missing_preflight.stdout:
        raise SystemExit("Read-only evidence analyzer unexpectedly accepted missing local preflight evidence")
    write(root / "local_flight_preflight.json", """{
  "observer": "local_flight_preflight",
  "success": true,
  "reason": "local_flight_preflight_passed",
  "summary": {
    "px4_params_checked": true,
    "px4_params_issue": "ok",
    "vision_output_consumer_issue": "ok",
    "setpoint_consumer_issue": "ok",
    "px4_param_values": {
      "EKF2_EV_CTRL": 3,
      "EKF2_EV_POS_X": 0.0,
      "EKF2_EV_POS_Y": 0.0,
      "EKF2_EV_POS_Z": 0.0,
      "COM_OF_LOSS_T": 1.0,
      "COM_OBL_RC_ACT": 4
    }
  },
  "parameters": {
    "require_vision": true,
    "require_vision_output": true,
    "require_timesync": true,
    "require_disarmed": true,
    "require_on_ground": true,
    "check_px4_vision_params": true,
    "check_px4_offboard_failsafe_params": true,
    "require_vision_output_consumer": true,
    "require_setpoint_consumer": true
  }
}
""")
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

python3 - "${workspace_dir}/scripts/analyze_active_flight_evidence.py" <<'PY'
import json
import pathlib
import subprocess
import sys
import tempfile

script = sys.argv[1]

def write_evidence(path, *, success=True, reason="active_local_flight_passed",
                   payload_open=False, state="COMPLETE", abort_reason=None,
                   target_reached=True, missing_waypoint_index=None,
                   target_dwell_s=1.0, active_vision=True,
                   active_vision_local_error=0.05,
                   active_mavros_control=True, include_landing_state=True,
                   raw_setpoints=True, raw_frame_mismatch=False,
                   raw_expected_publisher=True,
                   active_raw_setpoints=True,
                   raw_setpoint_order_mismatch=False,
                   corrupt_raw_setpoint_target=False,
                   active_raw_setpoint_order_mismatch=False,
                   corrupt_active_raw_setpoint_target=False,
                   omit_route_manifest=False, corrupt_route_manifest_target=False,
                   corrupt_route_status_fingerprint=False):
    target_records = [
        {"target": [0, 0, 1, 0], "state": "TAKEOFF", "waypoint_index": 0,
         "waypoint_total": 8, "min_distance_m": 0.08,
         "max_continuous_reach_s": target_dwell_s, "reached": True},
    ]
    route = [
        [1, 0, 1, 0],
        [0, 0, 1, 0],
        [0, 1, 1, 0],
        [0, 0, 1, 0],
        [0, -1, 1, 0],
        [0, 0, 1, 0],
        [-1, 0, 1, 0],
        [0, 0, 1, 0],
    ]
    for index, target in enumerate(route):
        if missing_waypoint_index == index:
            continue
        reached = target_reached or index != 1
        target_records.append({
            "target": target,
            "state": "WAYPOINTS",
            "waypoint_index": index,
            "waypoint_total": 8,
            "min_distance_m": 0.12 if reached else 0.80,
            "max_continuous_reach_s": target_dwell_s if reached else 0.0,
            "reached": reached,
        })
    route_manifest = None
    if not omit_route_manifest:
        manifest_route = []
        for record in target_records:
            manifest_target = list(record["target"])
            if corrupt_route_manifest_target and record["state"] == "WAYPOINTS" and record["waypoint_index"] == 2:
                manifest_target[0] += 0.50
            manifest_route.append({
                "state": record["state"],
                "waypoint_index": record["waypoint_index"] if record["state"] == "WAYPOINTS" else None,
                "target": manifest_target,
            })
        route_manifest = {
            "event": "mission_started",
            "route_source": "configured",
            "route_revision": 0,
            "route_fingerprint": "synthetic",
            "waypoint_frame": "robotac_start_body",
            "waypoint_count": 8,
            "takeoff_height": 1.0,
            "require_auto_land": True,
            "auto_land": True,
            "origin": [0.0, 0.0, 0.0],
            "origin_yaw": 0.0,
            "target_route": manifest_route,
        }
    raw_targets = [[0, 0, 0, 0]] + [list(record["target"]) for record in target_records]
    if corrupt_raw_setpoint_target and len(raw_targets) > 4:
        raw_targets[4][0] += 0.50
    if raw_setpoint_order_mismatch and len(raw_targets) > 4:
        raw_targets[2], raw_targets[3] = raw_targets[3], raw_targets[2]
    active_raw_targets = [list(record["target"]) for record in target_records]
    if corrupt_active_raw_setpoint_target and len(active_raw_targets) > 3:
        active_raw_targets[3][0] += 0.50
    if active_raw_setpoint_order_mismatch and len(active_raw_targets) > 3:
        active_raw_targets[2], active_raw_targets[3] = active_raw_targets[3], active_raw_targets[2]
    status_fingerprint = "wrong" if corrupt_route_status_fingerprint else "synthetic"
    path.write_text(json.dumps({
        "observer": "active_flight_observer",
        "success": success,
        "reason": reason,
        "summary": {
            "last_status": {"state": state, "waypoint": "8/8",
                            "route_revision": "0", "route_fingerprint": status_fingerprint},
            "state_history": (["IDLE", "PRESTREAM", "WAIT_OFFBOARD", "WAIT_ARMED", "TAKEOFF", "WAYPOINTS", "LANDING", "COMPLETE"]
                              if include_landing_state else
                              ["IDLE", "PRESTREAM", "WAIT_OFFBOARD", "WAIT_ARMED", "TAKEOFF", "WAYPOINTS", "COMPLETE"]),
            "abort_reason": abort_reason,
            "max_waypoint_index": 8,
            "total_waypoints": 8,
            "setpoint_count": 120,
            "unique_setpoints": [[0, 0, 0, 0], [0, 0, 1, 0], [1, 0, 1, 0], [0, 0, 1, 0]],
            "raw_setpoint_count": 120 if raw_setpoints else 0,
            "unique_raw_setpoints": raw_targets if raw_setpoints else [],
            "active_raw_setpoint_count": 120 if raw_setpoints and active_raw_setpoints else 0,
            "active_unique_raw_setpoints": active_raw_targets if raw_setpoints and active_raw_setpoints else [],
            "raw_setpoint_frame_mismatch_count": 1 if raw_frame_mismatch else 0,
            "raw_setpoint_expected_publisher_seen": bool(raw_expected_publisher),
            "raw_setpoint_publishers_seen": (["/local_waypoint_flight"] if raw_expected_publisher else ["/other_controller"]),
            "target_records": target_records,
            "route_manifest": route_manifest,
            "active_vision_pose_count": 24 if active_vision else 0,
            "active_vision_pose_parent": "odom" if active_vision else None,
            "active_vision_pose_first_stamp": 10.0 if active_vision else None,
            "active_vision_pose_last_stamp": 13.0 if active_vision else None,
            "active_vision_local_pair_count": 24 if active_vision else 0,
            "active_vision_local_max_delta_error_m": active_vision_local_error if active_vision else None,
            "active_vision_local_rms_delta_error_m": active_vision_local_error / 2.0 if active_vision else None,
            "active_vision_local_max_motion_m": 1.0 if active_vision else None,
            "active_vision_local_last_local_delta": [1.0, 0.0, 0.0] if active_vision else None,
            "active_vision_local_last_vision_delta": [1.0 - active_vision_local_error, 0.0, 0.0] if active_vision else None,
            "vision_output_enabled_latest": bool(active_vision),
            "active_vision_output_enabled_seen": bool(active_vision),
            "active_fastlio_vision_status_ok_seen": bool(active_vision),
            "last_fastlio_vision_status": "ok rate_hz=10.0 valid=24 dropped=0" if active_vision else "timeout",
            "local_count": 240,
            "initial_local_position": [0, 0, 0],
            "initial_local_yaw": 0.0,
            "initial_local_z": 0.0,
            "max_relative_local_z": 1.02,
            "active_mavros_state_count": 12 if active_mavros_control else 0,
            "active_mavros_connected_seen": bool(active_mavros_control),
            "active_mavros_armed_seen": bool(active_mavros_control),
            "active_mavros_offboard_seen": bool(active_mavros_control),
            "active_mavros_modes": ["OFFBOARD"] if active_mavros_control else [],
            "active_extended_state_count": 12 if active_mavros_control else 0,
            "active_landed_states": [2, 4] if include_landing_state else [2],
            "active_in_air_seen": True,
            "active_landing_seen": bool(include_landing_state),
            "final_armed": False,
            "final_landed_state": 1,
            "payload_open_seen": payload_open,
        },
    }, indent=2), encoding="utf-8")

with tempfile.TemporaryDirectory(prefix="robotac-active-flight-evidence.") as directory:
    root = pathlib.Path(directory)
    evidence = root / "active_flight_observer.json"
    write_evidence(evidence)
    ok = subprocess.run([sys.executable, script, str(root)], text=True,
                        stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    if ok.returncode != 0 or "active_local_flight=READY" not in ok.stdout:
        raise SystemExit("Active flight evidence analyzer rejected valid evidence:\n%s\n%s" % (ok.stdout, ok.stderr))
    if "active_route_manifest=READY" not in ok.stdout:
        raise SystemExit("Active flight evidence analyzer did not mark route manifest evidence ready")
    write_evidence(evidence, omit_route_manifest=True)
    missing_manifest = subprocess.run([sys.executable, script, str(root)], text=True,
                                      stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    if missing_manifest.returncode == 0 or "route_manifest_missing" not in missing_manifest.stdout:
        raise SystemExit("Active flight evidence analyzer accepted missing route manifest evidence")
    write_evidence(evidence, corrupt_route_manifest_target=True)
    bad_manifest_target = subprocess.run([sys.executable, script, str(root)], text=True,
                                         stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    if (bad_manifest_target.returncode == 0 or
            "route_manifest_observed_target_mismatch:waypoints2" not in bad_manifest_target.stdout):
        raise SystemExit("Active flight evidence analyzer accepted route manifest target mismatch")
    write_evidence(evidence, corrupt_route_status_fingerprint=True)
    bad_status_fingerprint = subprocess.run([sys.executable, script, str(root)], text=True,
                                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    if bad_status_fingerprint.returncode == 0 or "route_status_fingerprint_mismatch" not in bad_status_fingerprint.stdout:
        raise SystemExit("Active flight evidence analyzer accepted status/manifest fingerprint mismatch")
    write_evidence(evidence, raw_setpoints=False)
    missing_raw = subprocess.run([sys.executable, script, str(root)], text=True,
                                 stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    if missing_raw.returncode == 0 or "raw_setpoint_count_below_20" not in missing_raw.stdout:
        raise SystemExit("Active flight evidence analyzer accepted missing actual MAVROS raw setpoint evidence")
    write_evidence(evidence, active_raw_setpoints=False)
    missing_active_raw = subprocess.run([sys.executable, script, str(root)], text=True,
                                        stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    if missing_active_raw.returncode == 0 or "active_raw_setpoint_count_below_20" not in missing_active_raw.stdout:
        raise SystemExit("Active flight evidence analyzer accepted missing active-window MAVROS raw setpoint evidence")
    write_evidence(evidence, raw_frame_mismatch=True)
    bad_raw_frame = subprocess.run([sys.executable, script, str(root)], text=True,
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    if bad_raw_frame.returncode == 0 or "raw_setpoint_frame_mismatch_count" not in bad_raw_frame.stdout:
        raise SystemExit("Active flight evidence analyzer accepted a MAVROS raw setpoint frame mismatch")
    write_evidence(evidence, raw_expected_publisher=False)
    missing_raw_publisher = subprocess.run([sys.executable, script, str(root)], text=True,
                                           stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    if missing_raw_publisher.returncode == 0 or "raw_setpoint_expected_publisher_missing" not in missing_raw_publisher.stdout:
        raise SystemExit("Active flight evidence analyzer accepted a missing local_waypoint_flight raw setpoint publisher")
    write_evidence(evidence, corrupt_raw_setpoint_target=True)
    bad_raw_target = subprocess.run([sys.executable, script, str(root)], text=True,
                                    stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    if bad_raw_target.returncode == 0 or "route_manifest_raw_setpoint_missing:waypoints2" not in bad_raw_target.stdout:
        raise SystemExit("Active flight evidence analyzer accepted a raw setpoint route/manifest mismatch")
    write_evidence(evidence, corrupt_active_raw_setpoint_target=True)
    bad_active_raw_target = subprocess.run([sys.executable, script, str(root)], text=True,
                                           stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    if (bad_active_raw_target.returncode == 0 or
            "route_manifest_active_raw_setpoint_missing:waypoints2" not in bad_active_raw_target.stdout):
        raise SystemExit("Active flight evidence analyzer accepted an active raw setpoint route/manifest mismatch")
    write_evidence(evidence, raw_setpoint_order_mismatch=True)
    bad_raw_order = subprocess.run([sys.executable, script, str(root)], text=True,
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    if bad_raw_order.returncode == 0 or "route_manifest_raw_setpoint_order_mismatch" not in bad_raw_order.stdout:
        raise SystemExit("Active flight evidence analyzer accepted out-of-order MAVROS raw setpoints")
    write_evidence(evidence, active_raw_setpoint_order_mismatch=True)
    bad_active_raw_order = subprocess.run([sys.executable, script, str(root)], text=True,
                                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    if (bad_active_raw_order.returncode == 0 or
            "route_manifest_active_raw_setpoint_order_mismatch" not in bad_active_raw_order.stdout):
        raise SystemExit("Active flight evidence analyzer accepted out-of-order active MAVROS raw setpoints")
    write_evidence(evidence)
    payload_missing = subprocess.run([sys.executable, script, str(root), "--require-phase", "payload_local_flight"],
                                     text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    if payload_missing.returncode == 0 or "payload_local_flight=BLOCKED" not in payload_missing.stdout:
        raise SystemExit("Active flight evidence analyzer accepted missing payload evidence")
    write_evidence(evidence, payload_open=True)
    payload_ok = subprocess.run([sys.executable, script, str(root), "--require-phase", "payload_local_flight"],
                                text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    if payload_ok.returncode != 0 or "payload_local_flight=READY" not in payload_ok.stdout:
        raise SystemExit("Active flight evidence analyzer rejected valid payload evidence")
    write_evidence(evidence, active_vision=False)
    no_vision = subprocess.run([sys.executable, script, str(root)], text=True,
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    if no_vision.returncode == 0 or "active_vision_pose_count_below_5" not in no_vision.stdout:
        raise SystemExit("Active flight evidence analyzer accepted missing active vision pose evidence")
    write_evidence(evidence, active_vision_local_error=1.50)
    bad_ev_local = subprocess.run([sys.executable, script, str(root)], text=True,
                                  stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    if bad_ev_local.returncode == 0 or "active_vision_local_delta_error" not in bad_ev_local.stdout:
        raise SystemExit("Active flight evidence analyzer accepted inconsistent active vision/local motion evidence")
    write_evidence(evidence, active_mavros_control=False)
    no_mavros_control = subprocess.run([sys.executable, script, str(root)], text=True,
                                       stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    if no_mavros_control.returncode == 0 or "active_mavros_offboard_seen" not in no_mavros_control.stdout:
        raise SystemExit("Active flight evidence analyzer accepted missing active MAVROS OFFBOARD evidence")
    write_evidence(evidence, include_landing_state=False)
    no_landing = subprocess.run([sys.executable, script, str(root)], text=True,
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    if no_landing.returncode == 0 or "landing_state_missing" not in no_landing.stdout:
        raise SystemExit("Active flight evidence analyzer accepted evidence without a LANDING state")
    write_evidence(evidence, target_reached=False)
    unreached = subprocess.run([sys.executable, script, str(root)], text=True,
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    if unreached.returncode == 0 or "target_records_unreached" not in unreached.stdout:
        raise SystemExit("Active flight evidence analyzer accepted an unreached waypoint target")
    write_evidence(evidence, target_dwell_s=0.0)
    low_dwell = subprocess.run([sys.executable, script, str(root)], text=True,
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    if low_dwell.returncode == 0 or "target_records_unreached" not in low_dwell.stdout:
        raise SystemExit("Active flight evidence analyzer accepted a no-dwell waypoint target")
    write_evidence(evidence, missing_waypoint_index=3)
    missing_waypoint = subprocess.run([sys.executable, script, str(root)], text=True,
                                      stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    if missing_waypoint.returncode == 0 or "waypoint_target_records_missing:3" not in missing_waypoint.stdout:
        raise SystemExit("Active flight evidence analyzer accepted missing per-waypoint target evidence")
    write_evidence(evidence, success=False, reason="flight_aborted:test", abort_reason="test", state="ABORT")
    failed = subprocess.run([sys.executable, script, str(root)], text=True,
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    if failed.returncode == 0 or "active_local_flight=BLOCKED" not in failed.stdout:
        raise SystemExit("Active flight evidence analyzer accepted failed/aborted evidence")
print("Validated active flight evidence analyzer with synthetic evidence.")
PY

python3 - "${workspace_dir}" <<'PY'
import json
import pathlib
import shutil
import subprocess
import sys
import tempfile

workspace = pathlib.Path(sys.argv[1])
script = workspace / "scripts" / "flight_goal_audit.py"
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

def write_readonly_evidence(root):
    root.mkdir(parents=True, exist_ok=True)
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
    write(root / "ev_acceptance_observer.json", json.dumps({
        "observer": "ev_acceptance_observer",
        "success": True,
        "reason": "ev_acceptance_passed local_delta=(0.400,0.000,0.000) vision_delta=(0.390,0.000,0.000) delta_direction_cos=1.000 delta_scale=1.026",
        "parameters": {
            "require_connected": True,
            "require_disarmed": True,
            "require_on_ground": True,
            "require_vision_output_enabled": True,
            "require_vision_status_ok": True,
        },
        "metrics": {},
    }, indent=2))
    write(root / "local_flight_preflight.json", json.dumps({
        "observer": "local_flight_preflight",
        "success": True,
        "reason": "local_flight_preflight_passed",
        "summary": {
            "px4_params_checked": True,
            "px4_params_issue": "ok",
            "vision_output_consumer_issue": "ok",
            "setpoint_consumer_issue": "ok",
            "px4_param_values": {
                "EKF2_EV_CTRL": 3,
                "EKF2_EV_POS_X": 0.0,
                "EKF2_EV_POS_Y": 0.0,
                "EKF2_EV_POS_Z": 0.0,
                "COM_OF_LOSS_T": 1.0,
                "COM_OBL_RC_ACT": 4,
            },
        },
        "parameters": {
            "require_vision": True,
            "require_vision_output": True,
            "require_timesync": True,
            "require_disarmed": True,
            "require_on_ground": True,
            "check_px4_vision_params": True,
            "check_px4_offboard_failsafe_params": True,
            "require_vision_output_consumer": True,
            "require_setpoint_consumer": True,
        },
    }, indent=2))

def write_active_evidence(root, payload_open=False, success=True, waypoint_count=8,
                          corrupt_waypoint_index=None, corrupt_initial_origin=False,
                          dynamic_manifest=False, corrupt_dynamic_manifest=False,
                          omit_route_manifest=False):
    root.mkdir(parents=True, exist_ok=True)
    target_records = [
        {"target": [0, 0, 1, 0], "state": "TAKEOFF", "waypoint_index": 0,
         "waypoint_total": waypoint_count, "min_distance_m": 0.08,
         "max_continuous_reach_s": 1.0, "reached": True},
    ]
    route = (
        [1, 0, 1, 0],
        [0, 0, 1, 0],
        [0, 1, 1, 0],
        [0, 0, 1, 0],
        [0, -1, 1, 0],
        [0, 0, 1, 0],
        [-1, 0, 1, 0],
        [0, 0, 1, 0],
    )[:waypoint_count]
    for index, target in enumerate(route):
        target = list(target)
        if corrupt_waypoint_index == index:
            target[0] += 0.50
        target_records.append({
            "target": target,
            "state": "WAYPOINTS",
            "waypoint_index": index,
            "waypoint_total": waypoint_count,
            "min_distance_m": 0.12,
            "max_continuous_reach_s": 1.0,
            "reached": True,
        })
    raw_targets = [[0, 0, 0, 0]] + [list(record["target"]) for record in target_records]
    summary = {
        "last_status": {"state": "COMPLETE" if success else "ABORT",
                        "waypoint": "%d/%d" % (waypoint_count, waypoint_count),
                        "route_revision": "1" if dynamic_manifest else "0",
                        "route_fingerprint": "synthetic"},
        "state_history": ["IDLE", "PRESTREAM", "WAIT_OFFBOARD", "WAIT_ARMED", "TAKEOFF", "WAYPOINTS", "LANDING", "COMPLETE"],
        "abort_reason": None if success else "test",
        "max_waypoint_index": waypoint_count,
        "total_waypoints": waypoint_count,
        "setpoint_count": 120,
        "unique_setpoints": [[0, 0, 0, 0], [0, 0, 1, 0], [1, 0, 1, 0]],
        "raw_setpoint_count": 120,
        "unique_raw_setpoints": raw_targets,
        "active_raw_setpoint_count": 120,
        "active_unique_raw_setpoints": raw_targets,
        "raw_setpoint_frame_mismatch_count": 0,
        "raw_setpoint_expected_publisher_seen": True,
        "raw_setpoint_publishers_seen": ["/local_waypoint_flight"],
        "target_records": target_records,
        "active_vision_pose_count": 24,
        "active_vision_pose_parent": "odom",
        "active_vision_pose_first_stamp": 10.0,
        "active_vision_pose_last_stamp": 13.0,
        "active_vision_local_pair_count": 24,
        "active_vision_local_max_delta_error_m": 0.05,
        "active_vision_local_rms_delta_error_m": 0.025,
        "active_vision_local_max_motion_m": 1.0,
        "active_vision_local_last_local_delta": [1.0, 0.0, 0.0],
        "active_vision_local_last_vision_delta": [0.95, 0.0, 0.0],
        "vision_output_enabled_latest": True,
        "active_vision_output_enabled_seen": True,
        "active_fastlio_vision_status_ok_seen": True,
        "last_fastlio_vision_status": "ok rate_hz=10.0 valid=24 dropped=0",
        "local_count": 240,
        "initial_local_position": [0.50 if corrupt_initial_origin else 0.0, 0.0, 0.0],
        "initial_local_yaw": 0.0,
        "max_relative_local_z": 1.02,
        "active_mavros_state_count": 12,
        "active_mavros_connected_seen": True,
        "active_mavros_armed_seen": True,
        "active_mavros_offboard_seen": True,
        "active_mavros_modes": ["OFFBOARD"],
        "active_extended_state_count": 12,
        "active_landed_states": [2, 4],
        "active_in_air_seen": True,
        "active_landing_seen": True,
        "final_armed": False,
        "final_landed_state": 1,
        "payload_open_seen": payload_open,
    }
    manifest_route = []
    for record in target_records:
        item = {
            "state": record["state"],
            "waypoint_index": record["waypoint_index"] if record["state"] == "WAYPOINTS" else None,
            "target": list(record["target"]),
        }
        manifest_route.append(item)
    if corrupt_dynamic_manifest and len(manifest_route) > 1:
        manifest_route[1]["target"][0] += 0.50
    if not omit_route_manifest:
        route_source = "posearray" if dynamic_manifest else "configured"
        route_revision = 1 if dynamic_manifest else 0
        summary["route_manifest"] = {
            "event": "mission_started",
            "route_source": route_source,
            "route_revision": route_revision,
            "route_fingerprint": "synthetic",
            "waypoint_frame": "robotac_start_body",
            "waypoint_count": waypoint_count,
            "takeoff_height": 1.0,
            "require_auto_land": True,
            "auto_land": True,
            "origin": [0.0, 0.0, 0.0],
            "origin_yaw": 0.0,
            "target_route": manifest_route,
        }
    write(root / "active_flight_observer.json", json.dumps({
        "observer": "active_flight_observer",
        "success": success,
        "reason": "active_local_flight_passed" if success else "flight_aborted:test",
        "summary": summary,
    }, indent=2))

with tempfile.TemporaryDirectory(prefix="robotac-goal-audit.") as directory:
    root = pathlib.Path(directory)
    config_root = root / "config"
    readonly = root / "readonly"
    active = root / "active"
    shutil.copytree(str(workspace / "config"), str(config_root))
    shutil.copyfile(str(workspace / "config" / "deployment_sim.yaml"),
                    str(config_root / "deployment.yaml"))
    shutil.copyfile(str(workspace / "config" / "fastlio" / "vision_bridge_sim.yaml"),
                    str(config_root / "fastlio" / "vision_bridge.yaml"))
    write_readonly_evidence(readonly)
    write_active_evidence(active, payload_open=False)
    missing_active = subprocess.run(
        [sys.executable, str(script), "--config-root", str(config_root),
         "--readonly-evidence", str(readonly)],
        text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    if missing_active.returncode == 0 or "active_local_flight_evidence=BLOCKED" not in missing_active.stdout:
        raise SystemExit("Goal audit accepted missing active-flight evidence")
    ready = subprocess.run(
        [sys.executable, str(script), "--config-root", str(config_root),
         "--readonly-evidence", str(readonly), "--active-evidence", str(active)],
        text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    if ready.returncode != 0 or "active_local_flight=READY" not in ready.stdout:
        raise SystemExit("Goal audit rejected valid synthetic active-flight evidence:\n%s\n%s" % (ready.stdout, ready.stderr))
    write_active_evidence(active, payload_open=False, omit_route_manifest=True)
    missing_configured_manifest = subprocess.run(
        [sys.executable, str(script), "--config-root", str(config_root),
         "--readonly-evidence", str(readonly), "--active-evidence", str(active)],
        text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    if (missing_configured_manifest.returncode == 0 or
            "configured_route_manifest_missing" not in missing_configured_manifest.stdout):
        raise SystemExit("Goal audit accepted configured active-flight evidence without route manifest")
    custom_route = root / "custom_route.yaml"
    custom_route.write_text("""local_waypoint_flight:
  waypoint_frame: robotac_start_body
  takeoff_height: 1.0
  require_auto_land: true
  waypoints:
    - {x: 1.5, y: 0.0, z: 1.0, yaw_deg: 0.0, hold: 1.0}
  require_vision: true
  require_vision_output: true
  require_estimator_status: true
  require_horizontal_relative: true
  require_vertical_estimate: true
  require_vision_output_consumer: true
  require_setpoint_consumer: true
  require_timesync: true
  land_mode: AUTO.LAND
""", encoding="utf-8")
    write_active_evidence(active, payload_open=False, waypoint_count=1)
    custom_without_route_file = subprocess.run(
        [sys.executable, str(script), "--config-root", str(config_root),
         "--readonly-evidence", str(readonly), "--active-evidence", str(active)],
        text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    if custom_without_route_file.returncode == 0 or "expected_waypoints_mismatch:1!=8" not in custom_without_route_file.stdout:
        raise SystemExit("Goal audit accepted custom-route evidence without --route-file")
    custom_wrong_target = subprocess.run(
        [sys.executable, str(script), "--config-root", str(config_root),
         "--route-file", str(custom_route),
         "--readonly-evidence", str(readonly), "--active-evidence", str(active)],
        text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    if custom_wrong_target.returncode == 0 or "route_target_mismatch:waypoints0" not in custom_wrong_target.stdout:
        raise SystemExit("Goal audit accepted custom route-file evidence with the wrong target")
    write_active_evidence(active, payload_open=False, waypoint_count=1, corrupt_waypoint_index=0)
    custom_ready = subprocess.run(
        [sys.executable, str(script), "--config-root", str(config_root),
         "--route-file", str(custom_route),
         "--readonly-evidence", str(readonly), "--active-evidence", str(active)],
        text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    if custom_ready.returncode != 0 or "active_local_flight=READY" not in custom_ready.stdout:
        raise SystemExit("Goal audit rejected matching custom route-file evidence:\n%s\n%s" %
                         (custom_ready.stdout, custom_ready.stderr))
    write_active_evidence(active, payload_open=False)
    write_active_evidence(active, payload_open=False, waypoint_count=1)
    short_route = subprocess.run(
        [sys.executable, str(script), "--config-root", str(config_root),
         "--readonly-evidence", str(readonly), "--active-evidence", str(active)],
        text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    if short_route.returncode == 0 or "expected_waypoints_mismatch:1!=8" not in short_route.stdout:
        raise SystemExit("Goal audit accepted active-flight evidence from the wrong route length")
    write_active_evidence(active, payload_open=False)
    write_active_evidence(active, payload_open=False, corrupt_waypoint_index=4)
    wrong_target = subprocess.run(
        [sys.executable, str(script), "--config-root", str(config_root),
         "--readonly-evidence", str(readonly), "--active-evidence", str(active)],
        text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    if wrong_target.returncode == 0 or "route_target_mismatch:waypoints4" not in wrong_target.stdout:
        raise SystemExit("Goal audit accepted active-flight evidence for the wrong configured route target")
    write_active_evidence(active, payload_open=False)
    write_active_evidence(active, payload_open=False, corrupt_initial_origin=True)
    wrong_origin = subprocess.run(
        [sys.executable, str(script), "--config-root", str(config_root),
         "--readonly-evidence", str(readonly), "--active-evidence", str(active)],
        text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    if wrong_origin.returncode == 0 or "route_origin_mismatch" not in wrong_origin.stdout:
        raise SystemExit("Goal audit accepted active-flight evidence with the wrong local route origin")
    write_active_evidence(active, payload_open=False, waypoint_count=2, omit_route_manifest=True)
    dynamic_missing_manifest = subprocess.run(
        [sys.executable, str(script), "--config-root", str(config_root),
         "--readonly-evidence", str(readonly), "--active-evidence", str(active),
         "--allow-dynamic-active-route"],
        text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    if dynamic_missing_manifest.returncode == 0 or "dynamic_route_manifest_missing" not in dynamic_missing_manifest.stdout:
        raise SystemExit("Goal audit accepted dynamic active route evidence without a route manifest")
    write_active_evidence(active, payload_open=False, waypoint_count=2, dynamic_manifest=True)
    dynamic_ready = subprocess.run(
        [sys.executable, str(script), "--config-root", str(config_root),
         "--readonly-evidence", str(readonly), "--active-evidence", str(active),
         "--allow-dynamic-active-route"],
        text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    if dynamic_ready.returncode != 0 or "active_local_flight=READY" not in dynamic_ready.stdout:
        raise SystemExit("Goal audit rejected valid dynamic route manifest evidence:\n%s\n%s" % (dynamic_ready.stdout, dynamic_ready.stderr))
    write_active_evidence(active, payload_open=False, waypoint_count=2, dynamic_manifest=True,
                          corrupt_dynamic_manifest=True)
    dynamic_wrong_target = subprocess.run(
        [sys.executable, str(script), "--config-root", str(config_root),
         "--readonly-evidence", str(readonly), "--active-evidence", str(active),
         "--allow-dynamic-active-route"],
        text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    if dynamic_wrong_target.returncode == 0 or "dynamic_route_target_mismatch:waypoints0" not in dynamic_wrong_target.stdout:
        raise SystemExit("Goal audit accepted dynamic active route evidence with a wrong manifest target")
    write_active_evidence(active, payload_open=False)
    payload_missing = subprocess.run(
        [sys.executable, str(script), "--config-root", str(config_root),
         "--readonly-evidence", str(readonly), "--active-evidence", str(active),
         "--require-phase", "payload_local_flight"],
        text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    if payload_missing.returncode == 0 or "payload_local_flight=BLOCKED" not in payload_missing.stdout:
        raise SystemExit("Goal audit accepted missing payload evidence")
    write_active_evidence(active, payload_open=True)
    payload_ready = subprocess.run(
        [sys.executable, str(script), "--config-root", str(config_root),
         "--readonly-evidence", str(readonly), "--active-evidence", str(active),
         "--require-phase", "payload_local_flight"],
        text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    if payload_ready.returncode != 0 or "payload_local_flight=READY" not in payload_ready.stdout:
        raise SystemExit("Goal audit rejected valid synthetic payload evidence:\n%s\n%s" % (payload_ready.stdout, payload_ready.stderr))
print("Validated top-level flight goal audit with synthetic evidence.")
PY

for script in "${workspace_dir}"/scripts/*.sh "${workspace_dir}"/src/robotac_bringup/scripts/*.sh; do
  bash -n "${script}"
  echo "Validated shell syntax: ${script##*/}"
done
