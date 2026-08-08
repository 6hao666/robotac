#!/usr/bin/env bash
# Hardware-isolated regression for route_file based local waypoint missions.
set -euo pipefail

workspace_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
master_port=${ROBOTAC_ROUTE_FILE_SIM_MASTER_PORT:-11329}
log_dir=$(mktemp -d "${TMPDIR:-/tmp}/robotac-route-file-sim.XXXXXX")
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

python3 "${workspace_dir}/src/robotac_flight/test/flight_closed_loop_sim.py" \
  _initial_x:=3.0 _initial_y:=-2.0 _initial_yaw_deg:=90.0 \
  >"${log_dir}/sim.log" 2>&1 &
sim_pid=$!
roslaunch robotac_flight local_waypoint_flight.launch \
  config_root:="${workspace_dir}/config" \
  deployment_file:="${workspace_dir}/config/deployment_sim.yaml" \
  route_file:="${workspace_dir}/config/flight/local_waypoints_simple_box.yaml" \
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
rosservice list | grep -qx '/mavros/set_mode'
rosservice list | grep -qx '/mavros/cmd/arming'
rosservice list | grep -qx '/robotac/flight/start'
sleep 0.5

start_result=$(rosservice call /robotac/flight/start)
printf '%s\n' "${start_result}"
[[ "${start_result}" == *"success: True"* ]]

summary=
for _ in $(seq 1 700); do
  summary=$(timeout 1 rostopic echo -n 1 /robotac/test/flight_summary 2>/dev/null || true)
  if [[ "${summary}" == *"complete mode_requests=OFFBOARD,AUTO.LAND"* ]]; then
    break
  fi
  sleep 0.1
done

printf '%s\n' "${summary}"
[[ "${summary}" == *"complete mode_requests=OFFBOARD,AUTO.LAND"* ]]
[[ "${summary}" == *"arm_requests=True"* ]]
[[ "${summary}" == *"payload_commands= route="* ]]
[[ "${summary}" == *"route=(3.000,-2.000,1.000)->(3.000,-1.000,1.000)->(3.000,-2.000,1.000)->(2.000,-2.000,1.000)->(3.000,-2.000,1.000)->(4.000,-2.000,1.000)->(3.000,-2.000,1.000)->(3.000,-3.000,1.000)->(3.000,-2.000,1.000)"* ]]
[[ "${summary}" == *"mavlink_ned_route=(-2.000,3.000,-1.000,0.000)->(-1.000,3.000,-1.000,0.000)->(-2.000,3.000,-1.000,0.000)->(-2.000,2.000,-1.000,0.000)->(-2.000,3.000,-1.000,0.000)->(-2.000,4.000,-1.000,0.000)->(-2.000,3.000,-1.000,0.000)->(-3.000,3.000,-1.000,0.000)->(-2.000,3.000,-1.000,0.000)"* ]]
[[ "${summary}" == *"payload_open_at=none"* ]]
[[ "${summary}" == *"final=(3.000,-2.000,0.000,1.571)"* ]]
echo "Route-file closed-loop flight simulation passed."
