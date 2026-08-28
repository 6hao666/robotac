#!/bin/bash
# MAVROS启动脚本（绕过find_node bug）
source /opt/ros/noetic/setup.bash
source /home/yundrone/robotac_ws/devel/setup.bash
BRINGUP_PATH=$(rospack find robotac_bringup)
exec /opt/ros/noetic/lib/mavros/mavros_node \
  _fcu_url:=serial:///dev/robotac_px4:921600 \
  _gcs_url:=udp://0.0.0.0:14555@192.168.1.100:14550 \
  _fcu_protocol:=v2.0 \
  _pluginlists_yaml:=${BRINGUP_PATH}/config/mavros/px4_pluginlists.yaml \
  _config_yaml:=${BRINGUP_PATH}/config/mavros/px4.yaml \
  __name:=mavros \
  __log:=/home/yundrone/robotac_logs/mavros_$(date +%Y%m%d_%H%M%S).log
