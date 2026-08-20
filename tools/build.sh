#!/usr/bin/env bash
set -euo pipefail

workspace=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
mode=${1:-competition}
if [[ "$mode" != competition && "$mode" != --full ]]; then
  echo "用法：$0 [--full]" >&2
  exit 64
fi
cpu_count=$(nproc)
default_build_jobs=$((cpu_count > 1 ? cpu_count - 1 : 1))
build_jobs=${ROBOTAC_BUILD_JOBS:-${default_build_jobs}}
if [[ ! "$build_jobs" =~ ^[1-9][0-9]*$ ]]; then
  echo "ROBOTAC_BUILD_JOBS 必须是正整数。" >&2
  exit 64
fi
if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
  echo "未找到 ROS Noetic。构建环境必须为 Ubuntu 20.04 与 ROS Noetic。" >&2
  exit 1
fi
set +u
source /opt/ros/noetic/setup.bash
set -u
cp "${workspace}/src/livox_ros_driver2/package_ROS1.xml" \
  "${workspace}/src/livox_ros_driver2/package.xml"

if [[ "$mode" == competition ]]; then
  system_packages=(libmavconn mavros_msgs mavros mavros_extras apriltag_ros)
  for package in "${system_packages[@]}"; do
    path=$(rospack find "$package" 2>/dev/null || true)
    if [[ "$path" != /opt/ros/noetic/* ]]; then
      echo "比赛构建需要系统 ROS 包：$package" >&2
      echo "请先执行 tools/install_ubuntu20.sh。" >&2
      exit 69
    fi
  done
  source_space="${workspace}/build/competition_source"
  build_space="${workspace}/build/competition"
  devel_space="${workspace}/devel"
  packages=(livox_ros_driver2 fast_lio web_cam robotac_bringup
    robotac_examples robotac_localization robotac_servo robotac_mission)
  rm -rf -- "$source_space"
  mkdir -p "$source_space"
  ln -s "${workspace}/src/CMakeLists.txt" "$source_space/CMakeLists.txt"
  for package in "${packages[@]}"; do
    ln -s "${workspace}/src/${package}" "$source_space/${package}"
  done
  echo "执行比赛构建：本地 8 个包，MAVROS 与 AprilTag 使用系统二进制包。"
else
  source_space="${workspace}/src"
  build_space="${workspace}/build/full"
  devel_space="${workspace}/devel_full"
  echo "执行全源码构建：包含 MAVROS、AprilTag ROS 和上游测试包。"
fi

cd "${workspace}"
catkin_make --source "$source_space" --build "$build_space" \
  -j"${build_jobs}" -l"${build_jobs}" \
  -DCATKIN_DEVEL_PREFIX="$devel_space" \
  -DCATKIN_WHITELIST_PACKAGES= -DCATKIN_BLACKLIST_PACKAGES= \
  -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS1
