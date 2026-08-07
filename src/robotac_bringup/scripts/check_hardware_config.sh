#!/usr/bin/env bash
set -euo pipefail

config_root=${1:?usage: check_hardware_config.sh CONFIG_ROOT [REQUIRE_FCU] [REQUIRE_FLIGHT] [REQUIRE_VISION_OUTPUT] [REQUIRE_PAYLOAD] [ENABLE_MAVROS]}
require_fcu=${2:-true}
require_flight=${3:-false}
require_vision_output=${4:-false}
require_payload=${5:-false}
enable_mavros=${6:-true}
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
  true|True|TRUE|1|yes|YES|on|ON)
    fcu_required=true
    required_keys+=(stable_fcu_device_configured)
    ;;
  false|False|FALSE|0|no|NO|off|OFF)
    fcu_required=false
    ;;
  *)
    echo "require_fcu must be a Boolean value, got: ${require_fcu}" >&2
    exit 64
    ;;
esac
case "${enable_mavros}" in
  true|True|TRUE|1|yes|YES|on|ON)
    mavros_enabled=true
    ;;
  false|False|FALSE|0|no|NO|off|OFF)
    mavros_enabled=false
    ;;
  *)
    echo "enable_mavros must be a Boolean value, got: ${enable_mavros}" >&2
    exit 64
    ;;
esac
if [[ "${fcu_required}" == true && "${mavros_enabled}" != true ]]; then
  echo "MAVROS must be explicitly enabled when vision output or flight control is requested." >&2
  exit 64
fi
case "${require_flight}" in
  true|True|TRUE|1|yes|YES|on|ON)
    required_keys+=(
      fastlio_airframe_extrinsics_validated
      fastlio_axes_validated
      px4_external_vision_configured
      px4_offboard_failsafe_configured
      local_flight_ground_tested
    )
    ;;
  false|False|FALSE|0|no|NO|off|OFF)
    ;;
  *)
    echo "require_flight must be a Boolean value, got: ${require_flight}" >&2
    exit 64
    ;;
esac
case "${require_vision_output}" in
  true|True|TRUE|1|yes|YES|on|ON)
    required_keys+=(
      stable_fcu_device_configured
      fastlio_airframe_extrinsics_validated
      fastlio_axes_validated
      px4_external_vision_configured
    )
    ;;
  false|False|FALSE|0|no|NO|off|OFF)
    ;;
  *)
    echo "require_vision_output must be a Boolean value, got: ${require_vision_output}" >&2
    exit 64
    ;;
esac
case "${require_payload}" in
  true|True|TRUE|1|yes|YES|on|ON)
    required_keys+=(stable_servo_device_configured)
    ;;
  false|False|FALSE|0|no|NO|off|OFF)
    ;;
  *)
    echo "require_payload must be a Boolean value, got: ${require_payload}" >&2
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
