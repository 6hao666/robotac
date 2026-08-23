#!/bin/bash
# 录制 mission 运行的关键话题到 ~/robotac_bags/，跑 mission 前先起这个。
# 用法: bash tools/record_mission.sh [备注名]
# 产物: ~/robotac_bags/<时间>_<备注>.bag （rosbag 可用 rosbag info/play 回放分析）
mkdir -p ~/robotac_bags
tag="${1:-default}"
stamp=$(date +%Y%m%d_%H%M%S)
out=~/robotac_bags/${stamp}_${tag}.bag

source /opt/ros/noetic/setup.bash
source ~/robotac_ws/devel/setup.bash 2>/dev/null

echo "录制到: $out"
setsid nohup rosbag record -O "$out" \
  /mavros/state \
  /mavros/local_position/pose \
  /mavros/estimator_status \
  /mavros/extended_state \
  /mavros/timesync_status \
  /mavros/rc/in \
  /mavros/setpoint_position/local \
  /vision_pose_bridge/healthy \
  /vision_pose_bridge/state \
  /tag_detections \
  /robotac_mission/state \
  /robotac_mission/state_reason \
  /robotac_mission/result \
  > /tmp/rosbag.log 2>&1 < /dev/null &

sleep 2
echo "ROSbag 录制中 (PID 见 /tmp/rosbag.log)。跑完 mission 后 Ctrl-C 或:"
echo "  pkill -f \"rosbag recor[d]\"   # 停止录制"
