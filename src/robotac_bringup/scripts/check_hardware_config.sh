#!/usr/bin/env bash
set -euo pipefail

config_root=${1:?usage: check_hardware_config.sh CONFIG_ROOT [REQUIRE_FCU]}
require_fcu=${2:-true}
deployment_file="${config_root}/deployment.yaml"
lidar_file="${config_root}/lidar/mid360s.json"

if [[ ! -f "${deployment_file}" || ! -f "${lidar_file}" ]]; then
  echo "Missing deployment or MID360s configuration under ${config_root}." >&2
  exit 1
fi

required_keys=(
  lidar_network_configured
  lidar_imu_extrinsics_calibrated
  lidar_imu_time_checked
  camera_extrinsics_measured
  stable_camera_device_configured
)
case "${require_fcu}" in
  true|1|yes|on)
    required_keys+=(stable_fcu_device_configured)
    ;;
  false|0|no|off)
    ;;
  *)
    echo "require_fcu must be a Boolean value, got: ${require_fcu}" >&2
    exit 64
    ;;
esac
for key in "${required_keys[@]}"; do
  if ! grep -Eq "^[[:space:]]*${key}:[[:space:]]*true[[:space:]]*$" "${deployment_file}"; then
    echo "Deployment gate is not confirmed: ${key}" >&2
    exit 1
  fi
done

python3 - "${lidar_file}" <<'PY'
import json
import sys

with open(sys.argv[1]) as stream:
    config = json.load(stream)
host_ip = config["Mid360s"]["host_net_info"][0]["host_ip"]
lidar_ip = config["lidar_configs"][0]["ip"]
if (host_ip, lidar_ip) == ("192.168.1.5", "192.168.1.12"):
    raise SystemExit("MID360s network configuration still contains Sunray sample IP addresses.")
print(f"MID360s network configuration accepted: host={host_ip}, lidar={lidar_ip}")
PY
