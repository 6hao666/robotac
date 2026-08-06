#!/usr/bin/env bash
# ROS Noetic's setup scripts read optional variables before defining them, so
# load that environment before enabling nounset in a pristine remote shell.
set -eo pipefail
source /opt/ros/noetic/setup.bash
set -u
workspace_dir=/home/yundrone/robotac_ws

# Native AprilTag is installed below and consumed by apriltag_ros. Do not let
# catkin also interpret this upstream source checkout as a ROS package.
touch "${workspace_dir}/src/apriltag/CATKIN_IGNORE"

printf '%s\n' '=== build native Livox SDK ==='
cmake -S "${workspace_dir}/src/Livox-SDK2" -B "${workspace_dir}/build/livox-sdk2" -DCMAKE_BUILD_TYPE=Release
cmake --build "${workspace_dir}/build/livox-sdk2" --parallel 4

printf '%s\n' '=== install native Livox SDK ==='
sudo -S cmake --install "${workspace_dir}/build/livox-sdk2"

printf '%s\n' '=== build native AprilTag ==='
cmake -S "${workspace_dir}/src/apriltag" -B "${workspace_dir}/build/apriltag" -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=OFF
cmake --build "${workspace_dir}/build/apriltag" --parallel 4

printf '%s\n' '=== install native AprilTag ==='
sudo -S cmake --install "${workspace_dir}/build/apriltag"
sudo -S ldconfig
printf '%s\n' '=== install GeographicLib datasets ==='
sudo -S "${workspace_dir}/src/mavros/mavros/scripts/install_geographiclib_datasets.sh"
if ! compgen -G "/usr/share/GeographicLib/geoids/*.pgm" >/dev/null && \
   ! compgen -G "/usr/local/share/GeographicLib/geoids/*.pgm" >/dev/null; then
  echo 'GeographicLib geoid datasets are missing after build.' >&2
  exit 1
fi

printf '%s\n' '=== catkin build ==='
cd "${workspace_dir}"
catkin_make -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS1 -j4
