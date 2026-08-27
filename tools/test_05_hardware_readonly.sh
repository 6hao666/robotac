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
for device in /dev/ttyACM0 /dev/robotac_rgb_camera /dev/robotac_servo; do
  if [[ ! -e "${device}" ]]; then
    echo "缺少设备：${device}" >&2
    exit 1
  fi
done

topics=(
  /livox/lidar
  /livox/imu
  /camera/rgb/image_raw
  /camera/rgb/camera_info
  /mavros/state
  /mavros/extended_state
  /mavros/local_position/pose
  /mavros/estimator_status
  /mavros/timesync_status
  /sunray/odometry
  /vision_pose_bridge/healthy
)

expect_topic_type() {
  local topic=$1 expected=$2 actual
  actual=$(rostopic type "${topic}" 2>/dev/null || true)
  if [[ "${actual}" != "${expected}" ]]; then
    echo "话题类型错误：${topic} -> ${actual:-未发布}，预期 ${expected}" >&2
    exit 1
  fi
  echo "类型正确：${topic} -> ${expected}"
}

expect_topic_type /livox/lidar livox_ros_driver2/CustomMsg
expect_topic_type /livox/imu sensor_msgs/Imu
expect_topic_type /camera/rgb/image_raw sensor_msgs/Image
expect_topic_type /camera/rgb/camera_info sensor_msgs/CameraInfo

for topic in "${topics[@]}"; do
  if ! timeout 8 rostopic echo -n 1 "${topic}" >/dev/null; then
    echo "未在规定时间内读到话题：${topic}" >&2
    exit 1
  fi
  echo "已读取：${topic}"
done

camera_info=$(timeout 8 rostopic echo -n 1 /camera/rgb/camera_info)
CAMERA_INFO="${camera_info}" python3 - <<'PY'
import os
import sys

import yaml

messages = [item for item in yaml.safe_load_all(os.environ["CAMERA_INFO"])
            if isinstance(item, dict)]
if not messages:
    print("CameraInfo 没有可解析的消息。", file=sys.stderr)
    raise SystemExit(1)
message = messages[0]
if message.get("width") != 1920 or message.get("height") != 1080:
    print("CameraInfo 分辨率不是 1920x1080。", file=sys.stderr)
    raise SystemExit(1)
if message.get("header", {}).get("frame_id") != "camera_rgb_optical_frame":
    print("CameraInfo frame_id 不符合配置。", file=sys.stderr)
    raise SystemExit(1)
PY
echo "CameraInfo 分辨率和 frame_id 正确。"
