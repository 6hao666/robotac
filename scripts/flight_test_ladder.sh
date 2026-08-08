#!/usr/bin/env bash
# Print and validate the staged path from offline checks to read-only FCU/EV
# acceptance and, only when explicitly requested, active local-flight commands.
# This script never starts ROS nodes, opens serial devices, sends setpoints,
# arms, changes modes, or calls MAVROS services.
set -euo pipefail

workspace_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
config_root="${workspace_dir}/config"
origin_x=0.0
origin_y=0.0
origin_z=0.0
origin_yaw_deg=0.0
expected_ev_delay_ms=""
ev_delay_tolerance_ms=20.0
evidence_dir=""
show_active=false
skip_verify=false

usage() {
  cat <<'EOF'
usage: scripts/flight_test_ladder.sh [options]

Offline by default: verifies the workspace, previews the configured route, and
prints the next read-only aircraft commands. It never starts ROS or touches FCU
hardware.

Options:
  --config-root PATH              Config directory, default: ./config
  --origin-x M                    Preview start ENU x, default: 0.0
  --origin-y M                    Preview start ENU y, default: 0.0
  --origin-z M                    Preview start ENU z, default: 0.0
  --origin-yaw-deg DEG            Preview captured start yaw, default: 0.0
  --expected-ev-delay-ms MS       Include require_ev_delay command argument
  --ev-delay-tolerance-ms MS      EV delay tolerance, default: 20.0
  --evidence-dir PATH             Read-only topic + EV-acceptance evidence directory before --show-active
  --skip-verify                   Skip scripts/verify_workspace.sh
  --show-active                   Print active commands only after readiness gates pass
  -h, --help                      Show this help
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --config-root)
      config_root=${2:?--config-root requires a value}
      shift 2
      ;;
    --origin-x)
      origin_x=${2:?--origin-x requires a value}
      shift 2
      ;;
    --origin-y)
      origin_y=${2:?--origin-y requires a value}
      shift 2
      ;;
    --origin-z)
      origin_z=${2:?--origin-z requires a value}
      shift 2
      ;;
    --origin-yaw-deg)
      origin_yaw_deg=${2:?--origin-yaw-deg requires a value}
      shift 2
      ;;
    --expected-ev-delay-ms)
      expected_ev_delay_ms=${2:?--expected-ev-delay-ms requires a value}
      shift 2
      ;;
    --ev-delay-tolerance-ms)
      ev_delay_tolerance_ms=${2:?--ev-delay-tolerance-ms requires a value}
      shift 2
      ;;
    --evidence-dir)
      evidence_dir=${2:?--evidence-dir requires a value}
      shift 2
      ;;
    --skip-verify)
      skip_verify=true
      shift
      ;;
    --show-active)
      show_active=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 64
      ;;
  esac
done

deployment_file="${config_root}/deployment.yaml"
route_file="${config_root}/flight/local_waypoints.yaml"
if [[ ! -f "${deployment_file}" ]]; then
  echo "Missing deployment file: ${deployment_file}" >&2
  exit 1
fi
if [[ ! -f "${route_file}" ]]; then
  echo "Missing route file: ${route_file}" >&2
  exit 1
fi

print_section() {
  printf '\n=== %s ===\n' "$1"
}

gate_report=$(python3 - "${deployment_file}" <<'PY'
import sys
try:
    import yaml
except ImportError as exc:
    raise SystemExit(f"PyYAML is required: {exc}")

path = sys.argv[1]
with open(path, "r", encoding="utf-8") as stream:
    data = yaml.safe_load(stream) or {}
deployment = data.get("deployment", {})
if not isinstance(deployment, dict):
    raise SystemExit("deployment.yaml must contain a deployment mapping")

vision_keys = [
    "lidar_network_configured",
    "lidar_imu_extrinsics_calibrated",
    "lidar_imu_time_checked",
    "stable_fcu_device_configured",
    "fastlio_airframe_extrinsics_validated",
    "fastlio_axes_validated",
    "px4_external_vision_configured",
]
flight_keys = vision_keys + [
    "px4_offboard_failsafe_configured",
    "local_flight_ground_tested",
]
payload_keys = flight_keys + ["stable_servo_device_configured"]

def status(keys):
    missing = [key for key in keys if deployment.get(key) is not True]
    return missing

for title, keys in (("vision_output", vision_keys),
                    ("active_flight", flight_keys),
                    ("payload_flight", payload_keys)):
    missing = status(keys)
    print("%s=%s" % (title, "READY" if not missing else "BLOCKED:" + ",".join(missing)))
PY
)

print_section "offline validation"
if [[ "${skip_verify}" == false ]]; then
  "${workspace_dir}/scripts/verify_workspace.sh"
else
  echo "Skipped scripts/verify_workspace.sh by request."
fi

print_section "mission audit"
python3 "${workspace_dir}/src/robotac_flight/scripts/audit_local_mission.py" \
  --file "${route_file}" \
  --origin-x "${origin_x}" --origin-y "${origin_y}" --origin-z "${origin_z}" \
  --origin-yaw-deg "${origin_yaw_deg}" \
  --require-payload-open

print_section "readiness report"
readiness_report=$(python3 "${workspace_dir}/src/robotac_flight/scripts/local_flight_readiness.py" \
  --config-root "${config_root}" \
  --origin-x "${origin_x}" --origin-y "${origin_y}" --origin-z "${origin_z}" \
  --origin-yaw-deg "${origin_yaw_deg}")
printf '%s\n' "${readiness_report}"

print_section "route preview"
python3 "${workspace_dir}/src/robotac_flight/scripts/preview_local_route.py" \
  --file "${route_file}" \
  --origin-x "${origin_x}" --origin-y "${origin_y}" --origin-z "${origin_z}" \
  --origin-yaw-deg "${origin_yaw_deg}"

print_section "deployment gates"
printf '%s\n' "${gate_report}"

print_section "next read-only aircraft commands"
cat <<'EOF'
# 1) Read-only FCU/sensor bringup. Opens MAVROS telemetry only if enable_mavros is true;
#    no vision output, flight controller, setpoints, mode changes, or arming.
roslaunch robotac_bringup full_system.launch \
  enable_mavros:=true \
  enable_vision_bridge:=true vision_enable_output:=false \
  enable_flight_controller:=false enable_servo:=false

# 2) Read-only preflight before external-vision output is enabled.
roslaunch robotac_flight local_flight_preflight.launch \
  observe_seconds:=30 require_vision_output:=false

# 3) After transforms/PX4 EV parameters/deployment gates are approved, enable
#    MAVROS vision input and run the strict read-only preflight.
roslaunch robotac_bringup full_system.launch \
  enable_mavros:=true \
  enable_vision_bridge:=true vision_enable_output:=true \
  enable_flight_controller:=false enable_servo:=false

# Use one evidence directory for strict preflight, EV acceptance, topic capture,
# and offline analyzers.
evidence_dir=~/robotac_ws/logs/read_only_evidence/$(date +%Y%m%d_%H%M%S)
mkdir -p "${evidence_dir}"
EOF

if [[ -n "${expected_ev_delay_ms}" ]]; then
  cat <<EOF
roslaunch robotac_flight local_flight_preflight.launch \\
  observe_seconds:=30 require_vision_output:=true require_timesync:=true \\
  check_px4_vision_params:=true require_setpoint_consumer:=true \\
  require_ev_delay:=true expected_ev_delay_ms:=${expected_ev_delay_ms} \\
  ev_delay_tolerance_ms:=${ev_delay_tolerance_ms} \\
  evidence_file:="\${evidence_dir}/local_flight_preflight.json"
EOF
else
  cat <<'EOF'
roslaunch robotac_flight local_flight_preflight.launch \
  observe_seconds:=30 require_vision_output:=true require_timesync:=true \
  check_px4_vision_params:=true require_setpoint_consumer:=true \
  evidence_file:="${evidence_dir}/local_flight_preflight.json"
# Add require_ev_delay:=true expected_ev_delay_ms:=... after measuring FAST-LIO->FCU delay.
EOF
fi

cat <<'EOF'

# 4) Read-only EV acceptance: keep vehicle disarmed/on-ground and move it slowly.
roslaunch robotac_flight ev_acceptance_observer.launch \
  observe_seconds:=20 min_motion_m:=0.30 \
  evidence_file:="${evidence_dir}/ev_acceptance_observer.json"

# 5) Subscriber-only evidence capture after the read-only graph is running.
./scripts/collect_readonly_flight_evidence.sh \
  --duration 8 --bag-seconds 0 --output-dir "${evidence_dir}"

# 6) Analyze the same directory. This requires preflight and EV acceptance JSON.
./scripts/analyze_readonly_flight_evidence.py "${evidence_dir}"

# 7) Top-level goal audit stays BLOCKED until active-flight evidence is added.
./scripts/flight_goal_audit.py --readonly-evidence "${evidence_dir}"
EOF

if [[ "${show_active}" == true ]]; then
  if ! grep -q '^active_flight=READY$' <<<"${gate_report}"; then
    print_section "active flight commands blocked"
    echo "Deployment gates are not all true for active flight; refusing to print active commands." >&2
    printf '%s\n' "${gate_report}" >&2
    exit 2
  fi
  if ! python3 "${workspace_dir}/src/robotac_flight/scripts/local_flight_readiness.py" \
      --config-root "${config_root}" \
      --origin-x "${origin_x}" --origin-y "${origin_y}" --origin-z "${origin_z}" \
      --origin-yaw-deg "${origin_yaw_deg}" \
      --require-phase active_local_flight >/tmp/robotac-active-readiness.$$ 2>&1; then
    print_section "active flight commands blocked"
    echo "Readiness report did not pass active_local_flight; refusing to print active commands." >&2
    cat /tmp/robotac-active-readiness.$$ >&2
    rm -f /tmp/robotac-active-readiness.$$
    exit 2
  fi
  rm -f /tmp/robotac-active-readiness.$$
  if [[ -z "${evidence_dir}" ]]; then
    print_section "active flight commands blocked"
    echo "Evidence directory is required for --show-active; run ev_acceptance_observer plus collect_readonly_flight_evidence.sh and pass --evidence-dir." >&2
    exit 2
  fi
  if [[ ! -d "${evidence_dir}" ]]; then
    print_section "active flight commands blocked"
    echo "Evidence directory does not exist: ${evidence_dir}" >&2
    exit 2
  fi
  if ! python3 "${workspace_dir}/scripts/analyze_readonly_flight_evidence.py" \
      "${evidence_dir}" --require-phase active_preflight_evidence \
      >/tmp/robotac-active-evidence.$$ 2>&1; then
    print_section "active flight commands blocked"
    echo "Read-only evidence did not pass active_preflight_evidence; refusing to print active commands." >&2
    cat /tmp/robotac-active-evidence.$$ >&2
    rm -f /tmp/robotac-active-evidence.$$
    exit 2
  fi
  rm -f /tmp/robotac-active-evidence.$$
  print_section "active flight commands (not executed by this script)"
  printf 'readonly_evidence_dir=%q\n' "${evidence_dir}"
  cat <<'EOF'
# First active connected test: controller can stream only after /robotac/flight/start;
# mode/arming are still manual, auto_land is allowed because the route requires landing.
flight_evidence_dir=~/robotac_ws/logs/active_flight_evidence/$(date +%Y%m%d_%H%M%S)
mkdir -p "${flight_evidence_dir}"
roslaunch robotac_bringup full_system.launch \
  enable_mavros:=true \
  enable_vision_bridge:=true vision_enable_output:=true \
  enable_flight_controller:=true flight_enable_control:=true \
  flight_auto_mode:=false flight_auto_arm:=false flight_auto_land:=true \
  flight_enable_payload:=false enable_servo:=false

# In another terminal before the start request, record read-only completion evidence:
roslaunch robotac_flight active_flight_observer.launch \
  evidence_file:="${flight_evidence_dir}/active_flight_observer.json"

# Explicit operator start after preflight/visual checks pass:
rosservice call /robotac/flight/start

# After the observer exits, verify the captured completion evidence:
./scripts/analyze_active_flight_evidence.py "${flight_evidence_dir}"
./scripts/flight_goal_audit.py \
  --readonly-evidence "${readonly_evidence_dir}" \
  --active-evidence "${flight_evidence_dir}"
EOF

  if ! python3 "${workspace_dir}/src/robotac_flight/scripts/local_flight_readiness.py" \
      --config-root "${config_root}" \
      --origin-x "${origin_x}" --origin-y "${origin_y}" --origin-z "${origin_z}" \
      --origin-yaw-deg "${origin_yaw_deg}" \
      --require-phase payload_local_flight >/tmp/robotac-payload-readiness.$$ 2>&1; then
    print_section "payload flight commands blocked"
    echo "Payload readiness did not pass payload_local_flight; final payload mission hidden." >&2
    cat /tmp/robotac-payload-readiness.$$ >&2
    rm -f /tmp/robotac-payload-readiness.$$
    exit 0
  fi
  rm -f /tmp/robotac-payload-readiness.$$

  print_section "payload flight commands (not executed by this script)"
  cat <<'EOF'

# Final automatic mission only after manual active test succeeds and the test area is clear:
payload_flight_evidence_dir=~/robotac_ws/logs/active_flight_evidence/$(date +%Y%m%d_%H%M%S)_payload
mkdir -p "${payload_flight_evidence_dir}"
roslaunch robotac_bringup full_system.launch \
  enable_mavros:=true \
  enable_vision_bridge:=true vision_enable_output:=true \
  enable_flight_controller:=true flight_enable_control:=true \
  flight_auto_mode:=true flight_auto_arm:=true flight_auto_land:=true \
  enable_servo:=true flight_enable_payload:=true
roslaunch robotac_flight active_flight_observer.launch \
  require_payload_open:=true \
  evidence_file:="${payload_flight_evidence_dir}/active_flight_observer.json"
rosservice call /robotac/flight/start
./scripts/analyze_active_flight_evidence.py \
  "${payload_flight_evidence_dir}" --require-phase payload_local_flight
./scripts/flight_goal_audit.py \
  --readonly-evidence "${readonly_evidence_dir}" \
  --active-evidence "${payload_flight_evidence_dir}" \
  --require-phase payload_local_flight
EOF
else
  print_section "active flight commands hidden"
  echo "Run this script with --show-active after deployment and readiness gates are true to print active flight commands."
fi
