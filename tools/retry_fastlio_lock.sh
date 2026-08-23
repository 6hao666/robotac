#!/bin/bash
# 反复重启 FAST-LIO 直到锁正。用法: bash tools/retry_fastlio_lock.sh
# 成功条件: |roll|<3 且 |pitch|<3 且 |yaw|<10（单位度）
# 最多尝试 max_attempts 次。
cd ~/robotac_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash

max_attempts=${MAX_ATTEMPTS:-10}

pkill -f "robotac_bringup/launch/perceptio[n]" 2>/dev/null
pkill -f "fastlio_mappin[g]" 2>/dev/null
pkill -f "apriltag_ros_continuou[s]" 2>/dev/null
pkill -f "transform_odom_pointClou[d]" 2>/dev/null
sleep 2

for attempt in $(seq 1 "$max_attempts"); do
  echo "=== attempt $attempt/$max_attempts ==="
  setsid nohup roslaunch robotac_bringup perception.launch > /tmp/perception.log 2>&1 < /dev/null &
  sleep 22

  line=$(grep -E "yaw :" /tmp/perception.log 2>/dev/null | tail -1)
  if [ -z "$line" ]; then
    echo "    无姿态输出"
    pkill -f "fastlio_mappin[g]" 2>/dev/null
    pkill -f "transform_odom_pointClou[d]" 2>/dev/null
    pkill -f "apriltag_ros_continuou[s]" 2>/dev/null
    sleep 2
    continue
  fi

  yaw=$(echo "$line" | grep -oE "yaw : [+-]?[0-9.]+" | grep -oE "[+-]?[0-9.]+$")
  pitch=$(echo "$line" | grep -oE "pitch: [+-]?[0-9.]+" | grep -oE "[+-]?[0-9.]+$")
  roll=$(echo "$line" | grep -oE "roll: [+-]?[0-9.]+" | grep -oE "[+-]?[0-9.]+$")
  echo "    yaw=$yaw pitch=$pitch roll=$roll"

  ok=$(awk -v r="$roll" -v p="$pitch" -v y="$yaw" \
    'BEGIN{ ok=(r<3&&r>-3&&p<3&&p>-3&&y<10&&y>-10); print (ok?"YES":"NO") }')
  if [ "$ok" = "YES" ]; then
    echo "LOCKED: yaw=$yaw pitch=$pitch roll=$roll (attempt $attempt)"
    exit 0
  fi

  pkill -f "fastlio_mappin[g]" 2>/dev/null
  pkill -f "transform_odom_pointClou[d]" 2>/dev/null
  pkill -f "apriltag_ros_continuou[s]" 2>/dev/null
  sleep 2
done

echo "FAILED after $max_attempts attempts"
exit 1
