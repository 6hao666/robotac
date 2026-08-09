#!/usr/bin/env bash
set -euo pipefail

hold=false
if [[ "${1:-}" == "--hold" ]]; then
  hold=true
  shift
fi

config_root=${1:?usage: check_hardware_config.sh [--hold] CONFIG_ROOT [REQUIRE_FCU] [REQUIRE_FLIGHT] [REQUIRE_VISION_OUTPUT] [REQUIRE_PAYLOAD] [ENABLE_MAVROS] [REQUIRE_SENSOR_CALIBRATION]}
require_fcu=${2:-true}
require_flight=${3:-false}
require_vision_output=${4:-false}
require_payload=${5:-false}
enable_mavros=${6:-true}
# Preserve the conservative behavior for direct script use. full_system passes
# this explicitly so passive observation can start before flight calibration.
require_sensor_calibration=${7:-true}
deployment_file="${config_root}/deployment.yaml"
lidar_file="${config_root}/lidar/mid360s.json"

if [[ ! -f "${deployment_file}" || ! -f "${lidar_file}" ]]; then
  echo "Missing deployment or MID360s configuration under ${config_root}." >&2
  exit 1
fi

# Bash 3.2 treats an empty array as unset under `set -u` unless it is declared.
declare -a required_keys=()
case "${require_sensor_calibration}" in
  true|True|TRUE|1|yes|YES|on|ON)
    sensor_calibration_required=true
    required_keys+=(
      lidar_network_configured
      lidar_imu_extrinsics_calibrated
      lidar_imu_time_checked
      camera_extrinsics_measured
      stable_camera_device_configured
    )
    ;;
  false|False|FALSE|0|no|NO|off|OFF)
    sensor_calibration_required=false
    ;;
  *)
    echo "require_sensor_calibration must be a Boolean value, got: ${require_sensor_calibration}" >&2
    exit 64
    ;;
esac
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
if (( ${#required_keys[@]} )); then
  for key in "${required_keys[@]}"; do
    if ! grep -Eq "^[[:space:]]*${key}:[[:space:]]*true[[:space:]]*$" "${deployment_file}"; then
      echo "Deployment gate is not confirmed: ${key}" >&2
      exit 1
    fi
  done
fi

python3 - "${lidar_file}" "${sensor_calibration_required}" <<'PY'
import ipaddress
import json
import sys

with open(sys.argv[1]) as stream:
    config = json.load(stream)
try:
    host_ip = config["Mid360s"]["host_net_info"][0]["host_ip"]
    lidar_ip = config["lidar_configs"][0]["ip"]
    ipaddress.ip_address(host_ip)
    ipaddress.ip_address(lidar_ip)
except (IndexError, KeyError, ValueError) as exc:
    raise SystemExit(f"Invalid MID360s network configuration: {exc}")

if sys.argv[2] == "true" and host_ip == "192.168.1.5":
    raise SystemExit(
        "MID360s host_ip is still the Sunray sample address 192.168.1.5. "
        "Set the aircraft LiDAR NIC address before enabling vision or flight output.")

if host_ip == "192.168.1.5":
    print(
        f"MID360s passive observation: host={host_ip}, lidar={lidar_ip}. "
        "The sample host address is allowed only while all PX4 outputs are disabled.")
else:
    print(f"MID360s network configuration accepted: host={host_ip}, lidar={lidar_ip}")
PY

if [[ "${hold}" == true ]]; then
  echo "Hardware configuration check passed; holding required roslaunch guard alive."
  while true; do
    sleep 3600
  done
fi
