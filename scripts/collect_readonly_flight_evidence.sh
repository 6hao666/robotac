#!/usr/bin/env bash
# Read-only evidence collector for MAVROS + FAST-LIO local flight readiness.
# It never starts ROS nodes, publishes topics, calls ROS services, changes PX4
# modes, arms, sends setpoints, or opens serial devices. Run it after the
# read-only bringup/preflight launches are already running.
set -euo pipefail

workspace_dir=${ROBOTAC_WS:-/home/yundrone/robotac_ws}
duration=8
bag_seconds=0
output_root="${workspace_dir}/logs/read_only_evidence"
output_dir=""

usage() {
  cat <<'EOF'
usage: scripts/collect_readonly_flight_evidence.sh [options]

Collect subscriber-only evidence from an already running ROS graph. This script
does not launch nodes, publish setpoints, call services, arm, or change modes.

Options:
  --workspace PATH       Robotac workspace, default: /home/yundrone/robotac_ws
  --duration SEC         Seconds for rostopic hz windows, default: 8
  --bag-seconds SEC      Optional rosbag record duration, default: 0/off
  --output-root PATH     Evidence directory root, default: WORKSPACE/logs/read_only_evidence
  --output-dir PATH      Use/append to an explicit evidence directory
  -h, --help             Show this help
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --workspace)
      workspace_dir=${2:?--workspace requires a value}
      output_root="${workspace_dir}/logs/read_only_evidence"
      shift 2
      ;;
    --duration)
      duration=${2:?--duration requires a value}
      shift 2
      ;;
    --bag-seconds)
      bag_seconds=${2:?--bag-seconds requires a value}
      shift 2
      ;;
    --output-root)
      output_root=${2:?--output-root requires a value}
      shift 2
      ;;
    --output-dir)
      output_dir=${2:?--output-dir requires a value}
      shift 2
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

case "${duration}" in
  ''|*[!0-9]*) echo "--duration must be a positive integer number of seconds" >&2; exit 64 ;;
esac
case "${bag_seconds}" in
  ''|*[!0-9]*) echo "--bag-seconds must be a non-negative integer number of seconds" >&2; exit 64 ;;
esac
if (( duration < 1 )); then
  echo "--duration must be at least 1" >&2
  exit 64
fi

source /opt/ros/noetic/setup.bash
if [[ -f "${workspace_dir}/devel/setup.bash" ]]; then
  source "${workspace_dir}/devel/setup.bash"
fi

stamp=$(date +%Y%m%d_%H%M%S)
if [[ -n "${output_dir}" ]]; then
  out_dir="${output_dir}"
else
  out_dir="${output_root}/${stamp}"
fi
mkdir -p "${out_dir}"
summary="${out_dir}/summary.txt"

run_capture() {
  local name=$1
  shift
  {
    printf '### %s\n' "${name}"
    printf '$'
    printf ' %q' "$@"
    printf '\n'
    "$@"
  } >"${out_dir}/${name}.txt" 2>&1 || true
}

run_timeout_capture() {
  local name=$1
  local seconds=$2
  shift 2
  if command -v timeout >/dev/null 2>&1; then
    run_capture "${name}" timeout "${seconds}" "$@"
  else
    run_capture "${name}" "$@"
  fi
}

topics=(
  /mavros/state
  /mavros/extended_state
  /mavros/local_position/odom
  /mavros/estimator_status
  /mavros/timesync_status
  /mavros/vision_pose/pose
  /mavros/setpoint_raw/local
  /robotac/fastlio_vision/healthy
  /robotac/fastlio_vision/status
  /robotac/fastlio_vision/output_enabled
  /robotac/fastlio_vision/path_a_pose_preview
  /Odometry
  /livox/lidar
  /livox/imu
)

run_capture rosnode_list rosnode list
run_capture rostopic_list rostopic list
run_capture roswtf roswtf
run_capture process_snapshot ps -eo pid,ppid,user,etimes,cmd

for topic in "${topics[@]}"; do
  safe_name=$(echo "${topic#/}" | tr '/:' '__')
  run_capture "topic_info_${safe_name}" rostopic info "${topic}"
  run_capture "topic_type_${safe_name}" rostopic type "${topic}"
  run_timeout_capture "topic_echo_${safe_name}" 5 rostopic echo -n 1 "${topic}"
  run_timeout_capture "topic_hz_${safe_name}" "$(( duration + 3 ))" rostopic hz -w 20 "${topic}"
done

if command -v rosbag >/dev/null 2>&1 && (( bag_seconds > 0 )); then
  bag_path="${out_dir}/readonly_flight_evidence.bag"
  run_timeout_capture rosbag_record "$(( bag_seconds + 5 ))" \
    rosbag record -O "${bag_path}" --duration="${bag_seconds}" \
      /mavros/state \
      /mavros/extended_state \
      /mavros/local_position/odom \
      /mavros/estimator_status \
      /mavros/timesync_status \
      /mavros/vision_pose/pose \
      /robotac/fastlio_vision/healthy \
      /robotac/fastlio_vision/status \
      /robotac/fastlio_vision/output_enabled \
      /robotac/fastlio_vision/path_a_pose_preview \
      /Odometry \
      /livox/lidar \
      /livox/imu
fi

{
  echo "READ_ONLY_FLIGHT_EVIDENCE=${out_dir}"
  echo "workspace_dir=${workspace_dir}"
  echo "duration=${duration}"
  echo "bag_seconds=${bag_seconds}"
  echo "generated_at=${stamp}"
  echo "output_dir=${out_dir}"
  echo "safety=no_roslaunch_no_rosservice_no_rostopic_pub_no_setpoints_no_arming_no_mode_change"
  echo
  echo "Key checks to inspect:"
  echo "- topic_hz_mavros_local_position_odom.txt"
  echo "- topic_hz_mavros_vision_pose_pose.txt"
  echo "- topic_info_mavros_vision_pose_pose.txt"
  echo "- topic_info_mavros_setpoint_raw_local.txt"
  echo "- topic_echo_mavros_state.txt"
  echo "- topic_echo_robotac_fastlio_vision_status.txt"
} | tee "${summary}"
