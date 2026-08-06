#!/usr/bin/env bash
# Runtime observation only. This script never publishes commands or starts nodes.
set -eo pipefail
source /opt/ros/noetic/setup.bash
source /home/yundrone/robotac_ws/devel/setup.bash
set -u

echo '=== nodes ==='
rosnode list | grep -E '/livox_lidar_publisher|/fastlio_mapping' || true

for topic in /Odometry /PointCloud /cloud_registered; do
  echo "=== ${topic} type ==="
  rostopic type "${topic}" || true
  echo "=== ${topic} rate ==="
  timeout 8 rostopic hz -w 20 "${topic}" || true
done

echo '=== odometry sample ==='
timeout 5 rostopic echo -n 1 /Odometry || true

echo '=== tf frames ==='
timeout 5 rosrun tf tf_echo camera_init body || true
