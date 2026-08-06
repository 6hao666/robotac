#!/usr/bin/env bash
# Static deployment validation only: this script never starts a ROS node.
set -eo pipefail
source /opt/ros/noetic/setup.bash
source /home/yundrone/robotac_ws/devel/setup.bash
set -u

workspace_dir=/home/yundrone/robotac_ws
cd "${workspace_dir}"

./scripts/verify_workspace.sh

required_packages=(
  mavros
  mavros_msgs
  livox_ros_driver2
  fast_lio
  web_cam
  apriltag_ros
  robotac_bringup
)

for package in "${required_packages[@]}"; do
  printf '%s: ' "${package}"
  rospack find "${package}"
done

for binary in devel/lib/fast_lio/fastlio_mapping devel/lib/mavros/mavros_node; do
  if ldd "${binary}" | grep -q 'not found'; then
    echo "Missing shared-library dependency: ${binary}" >&2
    ldd "${binary}" | grep 'not found' >&2
    exit 1
  fi
done

echo 'No-launch validation passed.'
