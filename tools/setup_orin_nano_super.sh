#!/usr/bin/env bash
set -euo pipefail

fail() {
  echo "Orin 软件部署失败：$1" >&2
  return 1
}

validate_environment() {
  local architecture=$1 model=$2 os_id=$3 os_version=$4 ros_setup=$5
  local current_uid=$6 workspace_owner=$7 sudo_path=$8

  [[ "$current_uid" != 0 ]] || fail "请使用普通用户运行，不要使用 sudo 执行整个脚本。"
  [[ "$workspace_owner" == "$current_uid" ]] || \
    fail "工作空间不属于当前用户，请先修正目录所有权。"
  [[ "$architecture" == aarch64 ]] || fail "目标架构必须为 aarch64，当前为 ${architecture}。"
  [[ "$model" == *"Jetson Orin Nano"* ]] || \
    fail "目标必须为 NVIDIA Jetson Orin Nano，当前型号为 ${model:-未知}。"
  [[ "$os_id" == ubuntu && "$os_version" == 20.04 ]] || \
    fail "目标系统必须为 Ubuntu 20.04，当前为 ${os_id:-未知} ${os_version:-未知}。"
  [[ -f "$ros_setup" ]] || fail "未找到 ROS Noetic：${ros_setup}。"
  [[ -n "$sudo_path" && -x "$sudo_path" ]] || fail "未找到可执行的 sudo。"
}

verify_livox_installation() {
  local header=$1 library=$2
  local symbol=EnableLivoxLidarDiscoveryOnly
  local exported_symbols

  [[ -f "$header" ]] || fail "未找到已安装的 Livox SDK 头文件：${header}。"
  grep -Fq "void ${symbol}();" "$header" || \
    fail "Livox SDK 头文件版本过旧，缺少 ${symbol}。"
  [[ -n "$library" && -f "$library" ]] || \
    fail "未找到已安装的 Livox SDK 动态库：${library}。"
  exported_symbols=$(nm -D --defined-only "$library" | awk '{print $3}')
  grep -Fxq "$symbol" <<<"$exported_symbols" || \
    fail "Livox SDK 动态库版本过旧，缺少 ${symbol}。"
}

main() {
  local workspace architecture model os_id os_version sudo_path workspace_owner
  local ros_setup=/opt/ros/noetic/setup.bash
  local livox_header=/usr/local/include/livox_lidar_api.h
  local livox_library=/usr/local/lib/liblivox_lidar_sdk_shared.so

  if [[ $# -ne 0 ]]; then
    echo "用法：$0" >&2
    return 64
  fi
  workspace=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
  [[ -r /proc/device-tree/model ]] || fail "无法读取 /proc/device-tree/model。"
  [[ -r /etc/os-release ]] || fail "无法读取 /etc/os-release。"

  architecture=$(uname -m)
  model=$(tr -d '\0' </proc/device-tree/model)
  # shellcheck disable=SC1091
  source /etc/os-release
  os_id=${ID:-}
  os_version=${VERSION_ID:-}
  sudo_path=$(command -v sudo || true)
  workspace_owner=$(stat -c '%u' "$workspace")
  validate_environment "$architecture" "$model" "$os_id" "$os_version" \
    "$ros_setup" "$EUID" "$workspace_owner" "$sudo_path"

  echo "[1/6] 环境预检通过，验证 sudo 权限。"
  sudo -v
  echo "[2/6] 安装项目依赖和定制 SDK。"
  ROBOTAC_SKIP_SYSTEM_PACKAGES=0 "${workspace}/tools/install_ubuntu20.sh"
  verify_livox_installation "$livox_header" "$livox_library"
  echo "[3/6] 检查源码、配置和文档。"
  "${workspace}/tools/test_01_source.sh"
  echo "[4/6] 执行默认比赛构建。"
  "${workspace}/tools/test_02_build.sh"
  echo "[5/6] 运行离线单元测试。"
  "${workspace}/tools/test_03_unit.sh"
  echo "[6/6] 运行简化假飞控仿真。"
  "${workspace}/tools/test_04_simulation.sh"

  printf '\nOrin Nano Super 软件部署完成，当前状态：软件就绪。\n'
  printf '加载环境：source %q\n' "${workspace}/devel/setup.bash"
  printf '雷达配置：%q lidar\n' "${workspace}/tools/sensor_setup.py"
  printf '相机检查：%q camera\n' "${workspace}/tools/sensor_setup.py"
  printf '硬件配置文档：%q\n' "${workspace}/docs/03-hardware-and-configuration.md"
  printf '舵机标定文档：%q\n' "${workspace}/docs/10-servo-release-calibration.md"
  echo "PX4、舵机 udev、雷达持久网络和机械标定仍须按现场文档人工确认。"
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  main "$@"
fi
