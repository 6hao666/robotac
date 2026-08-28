#!/bin/bash
# 等系统时钟同步（Jetson 无 RTC 电池，开机是 1970；组件用 1970 时钟会算错时间戳）。
# 用法: 在直起二进制的 start_*.sh 的 exec 之前调用；也可作为 launch 的前置校验节点。
# 最多等 60s，超时打印告警并继续（不阻塞后续启动）。
set +e
for i in $(seq 1 12); do
  if timedatectl show -p NTPSynchronized --value | grep -q yes; then
    echo "[wait_clock] 时钟已同步: $(date '+%H:%M:%S')"
    return 0 2>/dev/null || exit 0
  fi
  sleep 5
done
echo "[wait_clock] WARN: 60s 内时钟未同步，继续（当前 $(date '+%H:%M:%S')）"
return 0 2>/dev/null || exit 0
