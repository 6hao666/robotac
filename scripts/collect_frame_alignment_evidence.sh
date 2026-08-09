#!/usr/bin/env bash
# Collect preview-only FAST-LIO frame-alignment evidence. This script only
# starts fastlio_frame_alignment_observer.py via roslaunch; it never starts
# MAVROS, LiDAR, FAST-LIO, the flight controller, publishes setpoints, calls
# services, changes modes, arms, or edits deployment files. Run it after the
# preview-only graph is already running with vision_enable_output:=false.
set -euo pipefail

workspace_dir=${ROBOTAC_WS:-/home/yundrone/robotac_ws}
output_dir=""
observe_seconds=20.0
min_translation_m=0.30
include_yaw=false
assume_ready=false

usage() {
  cat <<'EOF'
usage: scripts/collect_frame_alignment_evidence.sh [options]

Run preview-only FAST-LIO frame-alignment observers for +X/+Y/+Z. The operator
manually moves the disarmed/on-ground aircraft for each prompted motion. The
script writes JSON evidence and then runs analyze_frame_alignment_evidence.py.

Options:
  --workspace PATH        Robotac workspace, default: /home/yundrone/robotac_ws
  --output-dir PATH       Evidence output dir, default: WORKSPACE/logs/frame_alignment/TIMESTAMP
  --observe-seconds SEC   Observer window per motion, default: 20.0
  --min-translation M     Minimum translation per axis, default: 0.30
  --include-yaw           Also collect positive yaw preview evidence
  --assume-ready          Do not pause for Enter before each motion
  -h, --help              Show this help
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --workspace)
      workspace_dir=${2:?--workspace requires a value}
      shift 2
      ;;
    --output-dir)
      output_dir=${2:?--output-dir requires a value}
      shift 2
      ;;
    --observe-seconds)
      observe_seconds=${2:?--observe-seconds requires a value}
      shift 2
      ;;
    --min-translation)
      min_translation_m=${2:?--min-translation requires a value}
      shift 2
      ;;
    --include-yaw)
      include_yaw=true
      shift
      ;;
    --assume-ready)
      assume_ready=true
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

python3 - "${observe_seconds}" "${min_translation_m}" <<'PY'
import math
import sys

for label, raw in (("--observe-seconds", sys.argv[1]),
                   ("--min-translation", sys.argv[2])):
    try:
        value = float(raw)
    except ValueError:
        raise SystemExit(f"{label} must be numeric")
    if not math.isfinite(value) or value <= 0.0:
        raise SystemExit(f"{label} must be finite and positive")
PY

source /opt/ros/noetic/setup.bash
if [[ -f "${workspace_dir}/devel/setup.bash" ]]; then
  source "${workspace_dir}/devel/setup.bash"
fi

if [[ -z "${output_dir}" ]]; then
  output_dir="${workspace_dir}/logs/frame_alignment/$(date +%Y%m%d_%H%M%S)"
fi
mkdir -p "${output_dir}"

run_motion() {
  local name=$1
  local expected_x=$2
  local expected_y=$3
  local expected_z=$4
  local evidence_file="${output_dir}/${name}.json"
  printf '\n=== %s ===\n' "${name}"
  printf 'Move the disarmed/on-ground aircraft in preview direction (%s, %s, %s) for %.2f+ m during the %.1f s window.\n' \
    "${expected_x}" "${expected_y}" "${expected_z}" "${min_translation_m}" "${observe_seconds}"
  if [[ "${assume_ready}" == false ]]; then
    read -r -p "Press Enter when you are ready to move ${name}... " _
  fi
  roslaunch robotac_flight fastlio_frame_alignment_observer.launch \
    observe_seconds:="${observe_seconds}" \
    motion_name:="${name}" \
    expected_x:="${expected_x}" expected_y:="${expected_y}" expected_z:="${expected_z}" \
    min_translation_m:="${min_translation_m}" \
    evidence_file:="${evidence_file}"
}

run_yaw() {
  local name=preview_positive_yaw
  local evidence_file="${output_dir}/${name}.json"
  printf '\n=== %s ===\n' "${name}"
  printf 'Yaw the disarmed/on-ground aircraft in the positive preview yaw direction during the %.1f s window.\n' \
    "${observe_seconds}"
  if [[ "${assume_ready}" == false ]]; then
    read -r -p "Press Enter when you are ready to yaw ${name}... " _
  fi
  roslaunch robotac_flight fastlio_frame_alignment_observer.launch \
    observe_seconds:="${observe_seconds}" \
    motion_type:=yaw motion_name:="${name}" expected_yaw_sign:=1 \
    evidence_file:="${evidence_file}"
}

printf 'FRAME_ALIGNMENT_EVIDENCE_DIR=%s\n' "${output_dir}"
printf 'Safety: observer-only, no MAVROS control, no arming, no modes, no setpoints.\n'

run_motion preview_positive_x 1 0 0
run_motion preview_positive_y 0 1 0
run_motion preview_positive_z 0 0 1
if [[ "${include_yaw}" == true ]]; then
  run_yaw
fi

if [[ "${include_yaw}" == true ]]; then
  "${workspace_dir}/scripts/analyze_frame_alignment_evidence.py" \
    "${output_dir}" --require-yaw
else
  "${workspace_dir}/scripts/analyze_frame_alignment_evidence.py" \
    "${output_dir}"
fi
