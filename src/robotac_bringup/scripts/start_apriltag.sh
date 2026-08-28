#!/bin/bash
# AprilTag启动脚本（绕过find_node bug）
source /opt/ros/noetic/setup.bash
source /home/yundrone/robotac_ws/devel/setup.bash
BRINGUP_PATH=$(rospack find robotac_bringup)
# 等系统时钟同步（Jetson 开机 1970，组件时间戳会错）
bash ${BRINGUP_PATH}/scripts/wait_clock.sh
# 先加载配置到参数服务器
rosparam load ${BRINGUP_PATH}/config/apriltag/settings.yaml /apriltag_detector
rosparam load ${BRINGUP_PATH}/config/apriltag/tags.yaml /apriltag_detector
# 再直起二进制
exec /opt/ros/noetic/lib/apriltag_ros/apriltag_ros_continuous_node \
  __name:=apriltag_detector \
  image_rect:=/camera/rgb/image_raw \
  camera_info:=/camera/rgb/camera_info \
  _tag_family:=tag36h11 \
  _publish_tf:=true \
  _transport_hint:=raw \
  __log:=/home/yundrone/robotac_logs/apriltag_$(date +%Y%m%d_%H%M%S).log
