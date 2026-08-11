#!/usr/bin/env bash
# Hardware-isolated integration test for tag_payload_mission.py.
set -euo pipefail

workspace_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
master_port=${ROBOTAC_TAG_SIM_MASTER_PORT:-11326}
log_dir=$(mktemp -d "${TMPDIR:-/tmp}/robotac-tag-payload-sim.XXXXXX")
roscore_pid=
sim_pid=
mission_pid=

cleanup() {
  status=$?
  for pid in "${mission_pid}" "${sim_pid}" "${roscore_pid}"; do
    if [[ -n "${pid}" ]]; then
      kill -INT "${pid}" 2>/dev/null || true
    fi
  done
  for pid in "${mission_pid}" "${sim_pid}" "${roscore_pid}"; do
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

set +u
source /opt/ros/noetic/setup.bash
source "${workspace_dir}/devel/setup.bash"
set -u
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

python3 "${workspace_dir}/src/robotac_flight/test/tag_payload_mission_sim.py" \
  _horizontal_speed:=3.0 _vertical_speed:=1.0 _initial_yaw_deg:=0.0 \
  >"${log_dir}/sim.log" 2>&1 &
sim_pid=$!

roslaunch robotac_flight tag_payload_mission.launch \
  config_root:="${workspace_dir}/config" \
  deployment_file:="${workspace_dir}/config/deployment_sim.yaml" \
  enable_control:=true auto_mode:=true auto_arm:=true auto_land:=true \
  enable_payload:=true \
  setpoint_consumer_node:=/robotac_tag_payload_mission_sim \
  >"${log_dir}/mission.log" 2>&1 &
mission_pid=$!

for _ in $(seq 1 100); do
  if rosservice list 2>/dev/null | grep -qx '/robotac/tag_payload_mission/start'; then
    break
  fi
  sleep 0.1
done
rosservice list | grep -qx '/mavros/set_mode'
rosservice list | grep -qx '/mavros/cmd/arming'
rosservice list | grep -qx '/robotac/tag_payload_mission/start'

# Let latched payload status and TCPROS setpoint/payload connections settle.
sleep 0.8

start_result=$(rosservice call /robotac/tag_payload_mission/start)
printf '%s\n' "${start_result}"
[[ "${start_result}" == *"success: True"* ]]

summary=
for _ in $(seq 1 900); do
  summary=$(timeout 1 rostopic echo -n 1 /robotac/test/tag_payload_mission_summary 2>/dev/null || true)
  if [[ "${summary}" == *"complete state=COMPLETE"* ]]; then
    break
  fi
  sleep 0.1
done

printf '%s\n' "${summary}"
[[ "${summary}" == *"complete state=COMPLETE"* ]]
[[ "${summary}" == *"mode_requests=OFFBOARD,AUTO.LAND"* ]]
[[ "${summary}" == *"arm_requests=True"* ]]
[[ "${summary}" == *"payload_commands=open"* ]]
[[ "${summary}" == *"route=(0.000,0.000,0.300)->(2.000,-2.000,0.300)->(4.000,0.000,0.800)->(4.000,0.000,0.300)->(2.000,-2.000,0.300)->(0.000,0.000,0.300)"* ]]
[[ "${summary}" == *"payload_open_at=(4.000,0.000,0.300)"* ]]
[[ "${summary}" == *"final=(0.000,0.000,0.000,0.000)"* ]]
echo "AprilTag payload mission simulation passed."
