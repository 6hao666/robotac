#!/usr/bin/env bash
set -euo pipefail

workspace_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)

if [[ ${EUID} -eq 0 ]]; then
  sudo_cmd=()
elif command -v sudo >/dev/null 2>&1; then
  sudo_cmd=(sudo)
else
  echo "This script needs root privileges or sudo."
  exit 1
fi

if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
  echo "ROS Noetic was not found at /opt/ros/noetic. Use Ubuntu 20.04 with ROS Noetic."
  exit 1
fi

source /opt/ros/noetic/setup.bash

# AprilTag is built and installed as a native CMake dependency below. Catkin
# must not attempt to build the same source tree as a second ROS package.
touch "${workspace_dir}/src/apriltag/CATKIN_IGNORE"

"${sudo_cmd[@]}" apt-get update
"${sudo_cmd[@]}" apt-get install -y \
  build-essential cmake geographiclib-tools git python3-rosdep python3-catkin-tools python3-serial \
  python3-numpy \
  libapr1-dev libeigen3-dev libopencv-dev libpcl-dev libv4l-dev \
  ros-noetic-camera-info-manager ros-noetic-cmake-modules \
  ros-noetic-cv-bridge \
  ros-noetic-geographic-msgs ros-noetic-image-geometry ros-noetic-image-proc \
  ros-noetic-image-transport ros-noetic-pcl-ros ros-noetic-tf

if ! rosdep db >/dev/null 2>&1; then
  "${sudo_cmd[@]}" rosdep init
fi
rosdep update

cmake -S "${workspace_dir}/src/Livox-SDK2" -B "${workspace_dir}/build/livox-sdk2" -DCMAKE_BUILD_TYPE=Release
cmake --build "${workspace_dir}/build/livox-sdk2" --parallel
"${sudo_cmd[@]}" cmake --install "${workspace_dir}/build/livox-sdk2"

cmake -S "${workspace_dir}/src/apriltag" -B "${workspace_dir}/build/apriltag" \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=OFF
cmake --build "${workspace_dir}/build/apriltag" --parallel
"${sudo_cmd[@]}" cmake --install "${workspace_dir}/build/apriltag"
"${sudo_cmd[@]}" ldconfig

rosdep install --from-paths "${workspace_dir}/src" --ignore-src -r -y --skip-keys=apriltag
"${sudo_cmd[@]}" "${workspace_dir}/src/mavros/mavros/scripts/install_geographiclib_datasets.sh"
if ! compgen -G "/usr/share/GeographicLib/geoids/*.pgm" >/dev/null && \
   ! compgen -G "/usr/local/share/GeographicLib/geoids/*.pgm" >/dev/null; then
  echo "GeographicLib geoid datasets were not installed." >&2
  exit 1
fi

cd "${workspace_dir}"
catkin_make -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS1

echo "Build completed. Run: source ${workspace_dir}/devel/setup.bash"
