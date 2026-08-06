#!/usr/bin/env bash
set -u

printf '%s\n' '=== running ROS-related processes ==='
ps -eo pid,ppid,user,etimes,cmd | grep -E '[r]oscore|[r]oslaunch|[r]osmaster|[m]avros|[l]ivox|[f]astlio|[w]eb_cam|[a]priltag' || true

printf '%s\n' '=== enabled relevant systemd services ==='
systemctl list-unit-files --type=service --state=enabled --no-pager | grep -Ei 'ros|uav|drone|sunray|livox|mavros|fast|camera|jetson' || true

printf '%s\n' '=== active relevant systemd services ==='
systemctl list-units --type=service --state=running --no-pager | grep -Ei 'ros|uav|drone|sunray|livox|mavros|fast|camera|jetson' || true

printf '%s\n' '=== relevant service definitions ==='
for unit in $(systemctl list-unit-files --type=service --state=enabled --no-legend --no-pager | awk '{print $1}' | grep -Ei 'ros|uav|drone|sunray|livox|mavros|fast|camera|jetson'); do
  printf '\n--- %s ---\n' "$unit"
  systemctl cat "$unit" --no-pager 2>/dev/null | grep -E '^\[|^ExecStart=|^WorkingDirectory=|^Environment=|^User=' || true
done

printf '%s\n' '=== user systemd units ==='
systemctl --user list-unit-files --type=service --state=enabled --no-pager 2>/dev/null | grep -Ei 'ros|uav|drone|sunray|livox|mavros|fast|camera|jetson' || true

printf '%s\n' '=== cron and rc.local ==='
crontab -l 2>/dev/null || true
sudo -n crontab -l 2>/dev/null || true
sudo grep -RInE 'roslaunch|roscore|mavros|livox|fastlio|web_cam|apriltag|Sunray|uav' /etc/rc.local /etc/crontab /etc/cron.* 2>/dev/null || true

printf '%s\n' '=== existing workspaces ==='
find /home/yundrone -maxdepth 3 -type d \( -name '*ws*' -o -name '*Sunray*' -o -name '*robotac*' \) -print 2>/dev/null | sort
