#!/bin/bash
# ROBOTAC 全栈重建（飞机重启后）
# 顺序: 时钟校验 -> sensors(含TF链) -> perception -> mavros(带gcs_url) -> vision_to_px4
# 用法: bash tools/relaunch_stack.sh  （PC_IP 变了用 PC_IP=x.x.x.x 覆盖）
set -e
cd "$(dirname "${BASH_SOURCE[0]}")/.."
source /opt/ros/noetic/setup.bash
source devel/setup.bash

# PC 的 IP 是 DHCP 会变；gcs_url 转发给 PC 上的 QGC
PC_IP=${PC_IP:-10.201.166.4}

# 0. 等时钟同步（Jetson 无 RTC 电池，开机是 1970；组件用 1970 时钟会算错时间戳）
echo "[0] 等时钟同步 (NTP)"
for i in $(seq 1 12); do
  if timedatectl show -p NTPSynchronized --value | grep -q yes; then
    echo "   时钟已同步: $(date '+%H:%M:%S')"
    break
  fi
  sleep 5
done

echo "[1/4] sensors (含 TF 链: 相机外参 + map->camera_init 桥接)"
setsid nohup roslaunch robotac_bringup sensors.launch > /tmp/sensors.log 2>&1 < /dev/null &
sleep 5

echo "[2/4] perception (FAST-LIO + AprilTag)"
setsid nohup roslaunch robotac_bringup perception.launch > /tmp/perception.log 2>&1 < /dev/null &
sleep 5

echo "[3/4] mavros (fcu=/dev/robotac_px4, gcs=$PC_IP:14550)"
setsid nohup roslaunch robotac_bringup mavros_px4.launch fcu_url:=serial:///dev/robotac_px4:921600 "gcs_url:=udp://0.0.0.0:14555@${PC_IP}:14550" > /tmp/mavros.log 2>&1 < /dev/null &
sleep 5

echo "[4/4] vision_to_px4"
setsid nohup roslaunch robotac_localization vision_to_px4.launch > /tmp/vision.log 2>&1 < /dev/null &
sleep 3

echo "ALL DISPATCHED"
