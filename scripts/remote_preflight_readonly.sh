#!/usr/bin/env bash
printf '%s\n' '=== process ancestry ==='
ps -fp 3064,3430,3431,3472,3515,3520 || true
pstree -aps 3064 2>/dev/null || true

printf '%s\n' '=== active ROS graph ==='
source /opt/ros/noetic/setup.bash
rosnode list 2>&1 || true
rostopic list 2>&1 | sed -n '1,200p' || true

printf '%s\n' '=== session/autostart candidates ==='
ps -eo pid,ppid,user,cmd | grep -E '[t]mux|[s]creen|[g]nome-session|[w]eston|[a]utostart|[t]erminator' || true
for item in /home/yundrone/.bashrc /home/yundrone/.profile /home/yundrone/.xsessionrc; do
  if [[ -f "$item" ]]; then
    printf '\n--- %s ---\n' "$item"
    grep -nEi 'roscore|roslaunch|sunray|mavros|livox|fastlio|camera|apriltag' "$item" || true
  fi
done
find /home/yundrone/.config/autostart /home/yundrone/.config/systemd -maxdepth 3 -type f -print 2>/dev/null | while read -r item; do
  printf '\n--- %s ---\n' "$item"
  grep -nEi 'roscore|roslaunch|sunray|mavros|livox|fastlio|camera|apriltag' "$item" 2>/dev/null || true
done

printf '%s\n' '=== build and disk preflight ==='
df -h /home/yundrone | sed -n '1,2p'
command -v cmake || true
cmake --version 2>/dev/null | head -n1 || true
command -v catkin_make || true
command -v rosdep || true
for pkg in libapr1-dev libeigen3-dev libopencv-dev libpcl-dev libv4l-dev ros-noetic-image-proc; do
  status=$(dpkg-query -W -f='${db:Status-Status}' "$pkg" 2>/dev/null || true)
  printf '%s: %s\n' "$pkg" "${status:-not-installed}"
done

printf '%s\n' '=== Sunray source inventory ==='
find /home/yundrone/Sunray -maxdepth 3 -type d \( -name 'livox_ros_driver2' -o -name 'fast_lio' -o -name 'web_cam' -o -name 'sunray_communication_bridge' \) -print 2>/dev/null
