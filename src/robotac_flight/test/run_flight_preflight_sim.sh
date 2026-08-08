#!/usr/bin/env bash
# Hardware-isolated read-only flight-preflight regression.
set -euo pipefail

workspace_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
master_port=${ROBOTAC_PREFLIGHT_SIM_MASTER_PORT:-11324}
log_dir=$(mktemp -d "${TMPDIR:-/tmp}/robotac-flight-preflight-sim.XXXXXX")
roscore_pid=
inputs_pid=
bridge_pid=

cleanup() {
  status=$?
  for pid in "${bridge_pid}" "${inputs_pid}" "${roscore_pid}"; do
    if [[ -n "${pid}" ]]; then
      kill -INT "${pid}" 2>/dev/null || true
    fi
  done
  for pid in "${bridge_pid}" "${inputs_pid}" "${roscore_pid}"; do
    if [[ -n "${pid}" ]]; then
      wait "${pid}" 2>/dev/null || true
    fi
  done
  if [[ ${status} -ne 0 ]]; then
    for log in "${log_dir}"/*.log; do
      [[ -e "${log}" ]] && cat "${log}" >&2
    done
  fi
  rm -rf "${log_dir}"
  exit "${status}"
}
trap cleanup EXIT INT TERM

if python3 - "${master_port}" <<'PY'
import socket
import sys

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.settimeout(0.2)
try:
    occupied = sock.connect_ex(("127.0.0.1", int(sys.argv[1]))) == 0
finally:
    sock.close()
sys.exit(0 if occupied else 1)
PY
then
  echo "Refusing to use occupied ROS test port: ${master_port}" >&2
  exit 1
fi

source /opt/ros/noetic/setup.bash
source "${workspace_dir}/devel/setup.bash"
export ROS_MASTER_URI="http://127.0.0.1:${master_port}"
export ROS_IP=127.0.0.1
export ROS_HOSTNAME=localhost

roscore -p "${master_port}" >"${log_dir}/roscore.log" 2>&1 &
roscore_pid=$!
for _ in $(seq 1 100); do
  if rosparam list >/dev/null 2>&1; then
    break
  fi
  sleep 0.1
done
rosparam list >/dev/null

python3 "${workspace_dir}/src/robotac_flight/test/flight_dryrun_inputs.py" \
  >"${log_dir}/inputs.log" 2>&1 &
inputs_pid=$!
roslaunch robotac_flight fastlio_vision_bridge.launch \
  config_root:="${workspace_dir}/config" \
  deployment_file:="${workspace_dir}/config/deployment_sim.yaml" \
  vision_config:="${workspace_dir}/config/fastlio/vision_bridge_sim.yaml" \
  input_topic:=/Odometry output_topic:=/robotac/test/vision_pose \
  enable_mavros_output:=true >"${log_dir}/bridge.log" 2>&1 &
bridge_pid=$!

roslaunch robotac_flight local_flight_preflight.launch \
  observe_seconds:=1.0 startup_timeout:=12.0 \
  vision_health_window_seconds:=0.5 \
  require_vision_output:=true \
  require_setpoint_consumer:=true \
  setpoint_consumer_node:=/robotac_flight_dryrun_inputs \
  vision_output_consumer_node:=/robotac_flight_dryrun_inputs \
  vision_output_topic:=/robotac/test/vision_pose \
  >"${log_dir}/preflight.log" 2>&1
cat "${log_dir}/preflight.log"
grep -q 'PASS: local_flight_preflight_passed' "${log_dir}/preflight.log"

# The bridge is the sole health/status publisher.  Once it stops, valid fake
# MAVROS and FAST-LIO input alone must not satisfy the read-only preflight.
kill -INT "${bridge_pid}" 2>/dev/null || true
wait "${bridge_pid}" 2>/dev/null || true
bridge_pid=
set +e
roslaunch robotac_flight local_flight_preflight.launch \
  observe_seconds:=0.5 startup_timeout:=2.0 \
  vision_health_window_seconds:=0.5 \
  require_vision_output:=true \
  require_setpoint_consumer:=true \
  setpoint_consumer_node:=/robotac_flight_dryrun_inputs \
  vision_output_consumer_node:=/robotac_flight_dryrun_inputs \
  vision_output_topic:=/robotac/test/vision_pose \
  >"${log_dir}/preflight-bridge-loss.log" 2>&1
bridge_loss_status=$?
set -e
cat "${log_dir}/preflight-bridge-loss.log"
if [[ ${bridge_loss_status} -eq 0 ]]; then
  echo "Preflight unexpectedly passed after vision bridge loss." >&2
  exit 1
fi
grep -q 'FAIL: local_flight_preflight_timeout' "${log_dir}/preflight-bridge-loss.log"
echo "Read-only local-flight preflight regression passed."
