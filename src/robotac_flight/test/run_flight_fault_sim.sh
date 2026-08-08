#!/usr/bin/env bash
# Hardware-isolated controller failure regression. It verifies that loss of
# FAST-LIO vision or required MAVROS consumers stops raw setpoints instead of
# holding an old target.
set -euo pipefail

workspace_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
master_port=${ROBOTAC_FAULT_SIM_MASTER_PORT:-11323}
fault=${ROBOTAC_FLIGHT_FAULT:-vision_loss}
log_dir=$(mktemp -d "${TMPDIR:-/tmp}/robotac-flight-fault-sim.XXXXXX")
roscore_pid=
sim_pid=
controller_pid=

cleanup() {
  status=$?
  for pid in "${controller_pid}" "${sim_pid}" "${roscore_pid}"; do
    if [[ -n "${pid}" ]]; then
      kill -INT "${pid}" 2>/dev/null || true
    fi
  done
  for pid in "${controller_pid}" "${sim_pid}" "${roscore_pid}"; do
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

case "${fault}" in
  vision_loss)
    expected_error=fastlio_vision_lost
    ;;
  vision_output_loss)
    expected_error=mavros_vision_pose_timeout
    ;;
  vision_consumer_loss)
    expected_error=mavros_vision_pose_consumer_lost
    ;;
  setpoint_consumer_loss)
    expected_error=mavros_setpoint_raw_consumer_lost
    ;;
  *)
    echo "Unsupported ROBOTAC_FLIGHT_FAULT: ${fault}" >&2
    exit 64
    ;;
esac

python3 "${workspace_dir}/src/robotac_flight/test/flight_closed_loop_sim.py" \
  _fault:="${fault}" _fault_delay:=0.8 \
  >"${log_dir}/sim.log" 2>&1 &
sim_pid=$!
roslaunch robotac_flight local_waypoint_flight.launch \
  config_root:="${workspace_dir}/config" \
  deployment_file:="${workspace_dir}/config/deployment_sim.yaml" \
  enable_control:=true auto_mode:=true auto_arm:=true auto_land:=true \
  vision_output_consumer_node:=/robotac_flight_closed_loop_sim \
  setpoint_consumer_node:=/robotac_flight_closed_loop_sim \
  enable_payload:=false \
  >"${log_dir}/controller.log" 2>&1 &
controller_pid=$!

for _ in $(seq 1 100); do
  if rosservice list 2>/dev/null | grep -qx '/robotac/flight/start'; then
    break
  fi
  sleep 0.1
done
rosservice list | grep -qx '/robotac/flight/start'
sleep 0.5
start_result=$(rosservice call /robotac/flight/start)
printf '%s\n' "${start_result}"
[[ "${start_result}" == *"success: True"* ]]

summary=
for _ in $(seq 1 300); do
  summary=$(timeout 1 rostopic echo -n 1 /robotac/test/flight_fault_summary 2>/dev/null || true)
  if [[ "${summary}" == *"abort fault=${fault}"* ]]; then
    break
  fi
  sleep 0.1
done

printf '%s\n' "${summary}"
[[ "${summary}" == *"abort fault=${fault}"* ]]
[[ "${summary}" == *"error=${expected_error}"* ]]
[[ "${summary}" == *"post_abort_setpoints=0"* ]]
echo "Flight ${fault} safety regression passed."
