#!/usr/bin/env bash
set -euo pipefail

workspace=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
"${workspace}/tools/build.sh"
set +u
source "${workspace}/devel/setup.bash"
set -u
for package in robotac_bringup robotac_examples robotac_localization robotac_servo robotac_mission; do
  rospack find "${package}" >/dev/null
done
for package in libmavconn mavros_msgs mavros mavros_extras apriltag_ros; do
  path=$(rospack find "${package}")
  if [[ "$path" != /opt/ros/noetic/* ]]; then
    echo "系统 ROS 包被工作空间源码遮蔽：$package -> $path" >&2
    exit 1
  fi
done
echo "比赛构建和项目包发现检查通过。"
