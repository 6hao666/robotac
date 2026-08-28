#!/bin/bash
# Mission启动脚本（处理PYTHONPATH）
source /opt/ros/noetic/setup.bash
source /home/yundrone/robotac_ws/devel/setup.bash
# 关键：追加PYTHONPATH，不能覆盖
export PYTHONPATH=/home/yundrone/robotac_ws/src/robotac_mission/src:${PYTHONPATH}
exec python3 /home/yundrone/robotac_ws/src/robotac_mission/scripts/mission_node.py \
  _mission_yaml:=/home/yundrone/robotac_route_only.yaml \
  __name:=robotac_mission \
  __log:=/home/yundrone/robotac_logs/mission_$(date +%Y%m%d_%H%M%S).log
