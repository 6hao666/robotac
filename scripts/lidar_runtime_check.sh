#!/usr/bin/env bash
# Runtime observation only. This script never publishes commands or starts nodes.
set -eo pipefail
source /opt/ros/noetic/setup.bash
source /home/yundrone/robotac_ws/devel/setup.bash
set -u

echo '=== nodes ==='
rosnode list | grep -E '/livox_lidar_publisher|/fastlio_mapping' || true

echo '=== topic types ==='
rostopic type /livox/lidar
rostopic type /livox/imu

echo '=== lidar rate ==='
timeout 10 rostopic hz -w 30 /livox/lidar || true

echo '=== imu rate ==='
timeout 6 rostopic hz -w 100 /livox/imu || true

echo '=== lidar header ==='
timeout 5 rostopic echo -n 1 /livox/lidar/header || true

echo '=== imu header ==='
timeout 5 rostopic echo -n 1 /livox/imu/header || true
