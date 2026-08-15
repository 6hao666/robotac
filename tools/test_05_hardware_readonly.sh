#!/usr/bin/env bash
set -euo pipefail

if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
  echo "未找到 ROS Noetic。" >&2
  exit 1
fi
set +u
source /opt/ros/noetic/setup.bash
set -u

echo "本检查只读取设备文件和 ROS 话题，不发布消息，不调用飞控服务。"
for device in /dev/robotac_px4 /dev/robotac_rgb_camera /dev/robotac_servo; do
  if [[ ! -e "${device}" ]]; then
    echo "缺少设备：${device}" >&2
    exit 1
  fi
done

topics=(
  /mavros/state
  /mavros/extended_state
  /mavros/local_position/pose
  /mavros/estimator_status
  /mavros/timesync_status
  /sunray/odometry
  /vision_pose_bridge/healthy
  /camera/rgb/camera_info
)
for topic in "${topics[@]}"; do
  if ! timeout 8 rostopic echo -n 1 "${topic}" >/dev/null; then
    echo "未在规定时间内读到话题：${topic}" >&2
    exit 1
  fi
  echo "已读取：${topic}"
done
