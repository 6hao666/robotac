#!/usr/bin/env bash
set -euo pipefail

workspace=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
package_files=(
  "${workspace}/tools/ubuntu20_packages.txt"
  "${workspace}/tools/ubuntu20_direct_ros_packages.txt"
)
cpu_count=$(nproc)
default_build_jobs=$((cpu_count > 1 ? cpu_count - 1 : 1))
build_jobs=${ROBOTAC_BUILD_JOBS:-${default_build_jobs}}
if [[ ! "$build_jobs" =~ ^[1-9][0-9]*$ ]]; then
  echo "ROBOTAC_BUILD_JOBS 必须是正整数。" >&2
  exit 64
fi
if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
  echo "请在 Ubuntu 20.04 与 ROS Noetic 环境中执行。" >&2
  exit 1
fi
if [[ ${EUID} -eq 0 ]]; then
  sudo_cmd=()
elif command -v sudo >/dev/null 2>&1; then
  sudo_cmd=(sudo)
else
  echo "安装依赖需要 root 权限或 sudo。" >&2
  exit 1
fi

set +u
source /opt/ros/noetic/setup.bash
set -u
cp "${workspace}/src/livox_ros_driver2/package_ROS1.xml" \
  "${workspace}/src/livox_ros_driver2/package.xml"
if [[ ${ROBOTAC_SKIP_SYSTEM_PACKAGES:-0} != 1 ]]; then
  packages=()
  for package_file in "${package_files[@]}"; do
    mapfile -t current_packages < "${package_file}"
    packages+=("${current_packages[@]}")
  done
  "${sudo_cmd[@]}" apt-get update
  "${sudo_cmd[@]}" apt-get install -y "${packages[@]}"
  rosdep_sources=/etc/ros/rosdep/sources.list.d/20-default.list
  if [[ ! -f "$rosdep_sources" ]]; then
    "${sudo_cmd[@]}" rosdep init
  fi
  rosdep update
  rosdep install --from-paths "${workspace}/src" --ignore-src -r -y \
    --skip-keys=apriltag
fi

cmake -S "${workspace}/src/Livox-SDK2" \
  -B "${workspace}/build/livox-sdk2" -DCMAKE_BUILD_TYPE=Release
cmake --build "${workspace}/build/livox-sdk2" --parallel "${build_jobs}"
"${sudo_cmd[@]}" cmake --install "${workspace}/build/livox-sdk2"

cmake -S "${workspace}/src/apriltag" \
  -B "${workspace}/build/apriltag" -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_EXAMPLES=OFF
cmake --build "${workspace}/build/apriltag" --parallel "${build_jobs}"
"${sudo_cmd[@]}" cmake --install "${workspace}/build/apriltag"
"${sudo_cmd[@]}" ldconfig
echo "依赖安装完成。下一步执行 tools/test_02_build.sh。"
