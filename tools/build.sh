#!/usr/bin/env bash
set -euo pipefail

workspace=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
build_jobs=${ROBOTAC_BUILD_JOBS:-2}
if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
  echo "未找到 ROS Noetic。构建环境必须为 Ubuntu 20.04 与 ROS Noetic。" >&2
  exit 1
fi
set +u
source /opt/ros/noetic/setup.bash
set -u
cp "${workspace}/src/livox_ros_driver2/package_ROS1.xml" \
  "${workspace}/src/livox_ros_driver2/package.xml"
cd "${workspace}"
catkin_make -j"${build_jobs}" -l"${build_jobs}" \
  -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS1
