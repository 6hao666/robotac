#!/usr/bin/env bash
# Hardware-isolated FAST-LIO-to-MAVROS vision-pose regression.
set -euo pipefail

workspace_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
master_port=${ROBOTAC_VISION_SIM_MASTER_PORT:-11322}
log_dir=$(mktemp -d "${TMPDIR:-/tmp}/robotac-vision-sim.XXXXXX")
roscore_pid=
bridge_pid=

cleanup() {
  status=$?
  for pid in "${bridge_pid}" "${roscore_pid}"; do
    if [[ -n "${pid}" ]]; then
      kill -INT "${pid}" 2>/dev/null || true
    fi
  done
  for pid in "${bridge_pid}" "${roscore_pid}"; do
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

roslaunch robotac_flight fastlio_vision_bridge.launch \
  config_root:="${workspace_dir}/config" \
  deployment_file:="${workspace_dir}/config/deployment_sim.yaml" \
  vision_config:="${workspace_dir}/config/fastlio/vision_bridge_sim.yaml" \
  input_topic:=/robotac/test/fastlio_odom \
  output_topic:=/robotac/test/vision_pose \
  enable_mavros_output:=true \
  >"${log_dir}/bridge.log" 2>&1 &
bridge_pid=$!

python3 "${workspace_dir}/src/robotac_flight/test/vision_bridge_integration.py" \
  _input_topic:=/robotac/test/fastlio_odom \
  _output_topic:=/robotac/test/vision_pose \
  _min_samples:=3 \
  >"${log_dir}/tester.log" 2>&1
cat "${log_dir}/tester.log"
