#!/usr/bin/env bash
# Deploy the current Robotac checkout to an Ubuntu/ROS Noetic aircraft, build it,
# and optionally run read-only MAVROS + MID360/FAST-LIO smoke tests.
#
# Safety contract: this script never calls MAVROS services, never publishes
# setpoints, never arms, never changes flight mode, and never starts the
# Robotac flight state machine. Runtime checks are subscriber-only after starting
# the relevant sensor/telemetry nodes.
set -euo pipefail

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
workspace_dir=$(cd "${script_dir}/.." && pwd)

remote_user="yundrone"
remote_workspace="/home/yundrone/robotac_ws"
remote_password="${ROBOTAC_SSH_PASSWORD:-}"
baudrate="921600"
jobs="4"
lidar_ip=""
run_build=true
run_native_install=true
run_readonly_tests=true
mavros_ports=("/dev/ttyTHS0" "/dev/ttyTHS1")
mavros_ports_overridden=false
target_host=""

usage() {
  cat <<'EOF'
Usage:
  ROBOTAC_SSH_PASSWORD='<ssh-password>' ./scripts/deploy_aircraft.sh <aircraft-ip> [options]

Options:
  --user USER              SSH user. Default: yundrone
  --workspace PATH         Remote workspace. Default: /home/yundrone/robotac_ws
  --password PASSWORD      SSH/sudo password. Prefer ROBOTAC_SSH_PASSWORD.
  --lidar-ip IP            Patch remote config/lidar/mid360s.json before build/test.
  --mavros-port PATH       MAVROS serial port to test. Repeatable. Default: THS0 then THS1
  --baudrate RATE          MAVROS serial baudrate. Default: 921600
  --jobs N                 Build parallelism. Default: 4
  --skip-native-install    Do not rebuild/install native Livox-SDK2 and AprilTag.
  --skip-build             Sync only; skip remote catkin build and static checks.
  --skip-readonly-tests    Skip hardware runtime smoke tests.
  -h, --help               Show this help.

Examples:
  ROBOTAC_SSH_PASSWORD='<ssh-password>' ./scripts/deploy_aircraft.sh 192.168.10.66 \
    --lidar-ip 192.168.1.171 --mavros-port /dev/ttyTHS0

  ./scripts/deploy_aircraft.sh 192.168.10.66 --skip-readonly-tests

Safety:
  The read-only tests start MAVROS, Livox MID360, and FAST-LIO only long enough
  to subscribe to /mavros/state, /mavros/imu/data, /livox/*, /Odometry, and
  /cloud_registered. The script does not arm, set mode, publish setpoints, or
  call /robotac/flight/start.
EOF
}

die() {
  echo "ERROR: $*" >&2
  exit 1
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --user)
      remote_user=${2:?"--user requires a value"}
      shift 2
      ;;
    --workspace)
      remote_workspace=${2:?"--workspace requires a value"}
      shift 2
      ;;
    --password)
      remote_password=${2:?"--password requires a value"}
      shift 2
      ;;
    --lidar-ip)
      lidar_ip=${2:?"--lidar-ip requires a value"}
      shift 2
      ;;
    --mavros-port)
      if [[ "${mavros_ports_overridden}" == false ]]; then
        mavros_ports=()
        mavros_ports_overridden=true
      fi
      mavros_ports+=("${2:?"--mavros-port requires a value"}")
      shift 2
      ;;
    --baudrate)
      baudrate=${2:?"--baudrate requires a value"}
      shift 2
      ;;
    --jobs)
      jobs=${2:?"--jobs requires a value"}
      shift 2
      ;;
    --skip-native-install)
      run_native_install=false
      shift
      ;;
    --skip-build)
      run_build=false
      shift
      ;;
    --skip-readonly-tests)
      run_readonly_tests=false
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --*)
      die "Unknown option: $1"
      ;;
    *)
      if [[ -n "${target_host}" ]]; then
        die "Only one aircraft host may be supplied. Already have ${target_host}, got $1."
      fi
      target_host=$1
      shift
      ;;
  esac
done

[[ -n "${target_host}" ]] || { usage; die "Missing aircraft IP/host."; }
[[ "${jobs}" =~ ^[0-9]+$ ]] || die "--jobs must be an integer."
[[ "${baudrate}" =~ ^[0-9]+$ ]] || die "--baudrate must be an integer."

remote="${remote_user}@${target_host}"
ssh_base=(ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null)
rsync_ssh="ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null"

need_expect_for_password() {
  [[ -n "${remote_password}" ]] && ! command -v sshpass >/dev/null 2>&1
}

run_ssh_script() {
  local timeout=$1
  local body=$2
  local payload
  payload=$(printf '%s' "${body}" | base64 | tr -d '\n')

  if [[ -n "${remote_password}" ]]; then
    if command -v sshpass >/dev/null 2>&1; then
      SSHPASS=${remote_password} sshpass -e "${ssh_base[@]}" "${remote}" \
        "echo ${payload} | base64 -d | bash"
    else
      command -v expect >/dev/null 2>&1 || die "Password auth needs sshpass or expect. Install one, or use SSH keys."
      ROBOTAC_SSH_PASSWORD=${remote_password} expect <<EOF
set timeout ${timeout}
spawn ${ssh_base[*]} ${remote} "echo ${payload} | base64 -d | bash"
expect {
  -re "(P|p)assword:" { send "\$env(ROBOTAC_SSH_PASSWORD)\r"; exp_continue }
  eof
}
set wait_result [wait]
exit [lindex \$wait_result 3]
EOF
    fi
  else
    "${ssh_base[@]}" "${remote}" "echo ${payload} | base64 -d | bash"
  fi
}

run_rsync() {
  local destination="${remote}:${remote_workspace}/"
  local common_args=(
    -a --human-readable --progress
    --exclude=.git --exclude=build --exclude=devel --exclude=install --exclude=log
    --exclude='*.bag' --exclude='*.pcd' --exclude='*.pyc' --exclude=__pycache__
    -e "${rsync_ssh}"
    "${workspace_dir}/" "${destination}"
  )

  if [[ -n "${remote_password}" ]]; then
    if command -v sshpass >/dev/null 2>&1; then
      SSHPASS=${remote_password} sshpass -e rsync "${common_args[@]}"
    else
      command -v expect >/dev/null 2>&1 || die "Password auth needs sshpass or expect. Install one, or use SSH keys."
      ROBOTAC_SSH_PASSWORD=${remote_password} expect <<EOF
set timeout -1
spawn rsync -a --human-readable --progress --exclude=.git --exclude=build --exclude=devel --exclude=install --exclude=log --exclude=*.bag --exclude=*.pcd --exclude=*.pyc --exclude=__pycache__ -e "${rsync_ssh}" "${workspace_dir}/" "${destination}"
expect {
  -re "(P|p)assword:" { send "\$env(ROBOTAC_SSH_PASSWORD)\r"; exp_continue }
  eof
}
set wait_result [wait]
exit [lindex \$wait_result 3]
EOF
    fi
  else
    rsync "${common_args[@]}"
  fi
}

password_b64=$(printf '%s' "${remote_password}" | base64 | tr -d '\n')
mavros_ports_text=${mavros_ports[*]}

echo "=== Robotac aircraft deployment ==="
echo "Local:  ${workspace_dir}"
echo "Remote: ${remote}:${remote_workspace}"
echo "Build:  ${run_build}; native install: ${run_native_install}; read-only tests: ${run_readonly_tests}"
echo "MAVROS ports: ${mavros_ports_text} @ ${baudrate}"
if [[ -n "${lidar_ip}" ]]; then
  echo "Remote MID360 IP override: ${lidar_ip}"
fi
if need_expect_for_password; then
  echo "Using expect for password-based SSH. Prefer SSH keys or sshpass for unattended CI."
fi

echo "=== create remote workspace ==="
run_ssh_script 60 "set -euo pipefail
mkdir -p '${remote_workspace}'
"

echo "=== sync source ==="
run_rsync

if [[ "${run_build}" == true ]]; then
  echo "=== remote build and static validation ==="
  run_ssh_script 1800 "set -euo pipefail
workspace_dir='${remote_workspace}'
deploy_user='${remote_user}'
password_b64='${password_b64}'
lidar_ip='${lidar_ip}'
jobs='${jobs}'
run_native_install='${run_native_install}'

deploy_password=''
if [[ -n \"\${password_b64}\" ]]; then
  deploy_password=\$(printf '%s' \"\${password_b64}\" | base64 -d)
fi

sudo_run() {
  if [[ \${EUID} -eq 0 ]]; then
    \"\$@\"
  elif [[ -n \"\${deploy_password}\" ]]; then
    printf '%s\n' \"\${deploy_password}\" | sudo -S \"\$@\"
  else
    sudo -n \"\$@\"
  fi
}

cd \"\${workspace_dir}\"
if getent group dialout >/dev/null 2>&1; then
  sudo_run usermod -aG dialout \"\${deploy_user}\" || echo 'WARN: could not add user to dialout; serial access may fail.' >&2
fi
if getent group video >/dev/null 2>&1; then
  sudo_run usermod -aG video \"\${deploy_user}\" || echo 'WARN: could not add user to video; camera access may fail.' >&2
fi

touch \"\${workspace_dir}/src/apriltag/CATKIN_IGNORE\"

if [[ -n \"\${lidar_ip}\" ]]; then
  python3 - \"\${workspace_dir}/config/lidar/mid360s.json\" \"\${lidar_ip}\" <<'PY'
import json
import sys
path, ip = sys.argv[1], sys.argv[2]
with open(path) as stream:
    config = json.load(stream)
config['lidar_configs'][0]['ip'] = ip
with open(path, 'w') as stream:
    json.dump(config, stream, indent=2)
    stream.write('\n')
PY
fi

source /opt/ros/noetic/setup.bash

if [[ \"\${run_native_install}\" == true ]]; then
  echo '--- native Livox-SDK2 install ---'
  cmake -S \"\${workspace_dir}/src/Livox-SDK2\" -B \"\${workspace_dir}/build/livox-sdk2\" -DCMAKE_BUILD_TYPE=Release
  cmake --build \"\${workspace_dir}/build/livox-sdk2\" --parallel \"\${jobs}\"
  sudo_run cmake --install \"\${workspace_dir}/build/livox-sdk2\"

  echo '--- native AprilTag install ---'
  cmake -S \"\${workspace_dir}/src/apriltag\" -B \"\${workspace_dir}/build/apriltag\" -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=OFF
  cmake --build \"\${workspace_dir}/build/apriltag\" --parallel \"\${jobs}\"
  sudo_run cmake --install \"\${workspace_dir}/build/apriltag\"
  sudo_run ldconfig
fi

echo '--- catkin_make ---'
catkin_make -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS1 -j\"\${jobs}\"
source \"\${workspace_dir}/devel/setup.bash\"

echo '--- package presence ---'
for package in fast_lio livox_ros_driver2 mavros mavros_msgs robotac_bringup robotac_flight robotac_servo web_cam apriltag_ros; do
  printf '%s: ' \"\${package}\"
  rospack find \"\${package}\"
done

echo '--- no-launch/static checks ---'
if [[ -x \"\${workspace_dir}/scripts/verify_workspace.sh\" ]]; then
  \"\${workspace_dir}/scripts/verify_workspace.sh\"
fi
roslaunch --nodes robotac_bringup lidar_mid360s.launch
roslaunch --nodes robotac_bringup fastlio_mid360s.launch
roslaunch --nodes robotac_bringup mavros_px4.launch fcu_url:=serial:///dev/ttyTHS0:921600
"
fi

if [[ "${run_readonly_tests}" == true ]]; then
  echo "=== remote read-only hardware smoke tests ==="
  run_ssh_script 360 "set -uo pipefail
workspace_dir='${remote_workspace}'
baudrate='${baudrate}'
mavros_ports='${mavros_ports_text}'

cd \"\${workspace_dir}\" || exit 1
source /opt/ros/noetic/setup.bash
source \"\${workspace_dir}/devel/setup.bash\"

echo '--- serial devices ---'
ls -l /dev/ttyTHS* /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || true

mavros_ok=false
for port in \${mavros_ports}; do
  echo "--- MAVROS read-only test: \${port} ---"
  if [[ ! -e \"\${port}\" ]]; then
    echo "skip: \${port} does not exist"
    continue
  fi
  if fuser -v \"\${port}\" >/tmp/robotac_port_users.txt 2>&1; then
    cat /tmp/robotac_port_users.txt
    echo "skip: \${port} is already in use"
    continue
  fi

  state_file=/tmp/robotac_mavros_state.txt
  imu_file=/tmp/robotac_mavros_imu.txt
  log_file=/tmp/robotac_mavros_test.log
  : >\"\${state_file}\"
  : >\"\${imu_file}\"
  roslaunch robotac_bringup mavros_px4.launch fcu_url:=serial://\"\${port}\":\"\${baudrate}\" >\"\${log_file}\" 2>&1 &
  launch_pid=\$!
  for _ in \$(seq 1 25); do
    rosnode list 2>/dev/null | grep -qx /mavros && break
    sleep 1
  done
  timeout 8 rostopic echo -n 1 /mavros/state >\"\${state_file}\" 2>&1 || true
  timeout 8 rostopic echo -n 1 /mavros/imu/data >\"\${imu_file}\" 2>&1 || true
  echo 'state:'
  cat \"\${state_file}\"
  echo 'imu:'
  sed -n '1,80p' \"\${imu_file}\"
  echo 'mavros log key lines:'
  grep -E 'GeographicLib|FCU|serial|connected|Got HEARTBEAT|WARN|ERROR|FATAL|Permission|DeviceError' \"\${log_file}\" | tail -n 120 || true
  kill \"\${launch_pid}\" 2>/dev/null || true
  sleep 2
  pkill -P \"\${launch_pid}\" 2>/dev/null || true
  if grep -q 'connected: True' \"\${state_file}\" && grep -q 'linear_acceleration:' \"\${imu_file}\"; then
    mavros_ok=true
    break
  fi
done

if [[ \"\${mavros_ok}\" != true ]]; then
  echo 'ERROR: MAVROS read-only test did not confirm connected state plus IMU data.' >&2
  exit 2
fi

echo '--- network and LiDAR config ---'
ip -4 addr show | sed -n '1,140p'
cat \"\${workspace_dir}/config/lidar/mid360s.json\"

lidar_log=/tmp/robotac_lidar_test.log
fastlio_log=/tmp/robotac_fastlio_test.log
odom_file=/tmp/robotac_fastlio_odom.txt
: >\"\${odom_file}\"

roslaunch robotac_bringup lidar_mid360s.launch >\"\${lidar_log}\" 2>&1 &
lidar_pid=\$!
for _ in \$(seq 1 25); do
  rostopic list 2>/dev/null | grep -qx /livox/lidar && break
  sleep 1
done

roslaunch robotac_bringup fastlio_mid360s.launch >\"\${fastlio_log}\" 2>&1 &
fastlio_pid=\$!
for _ in \$(seq 1 35); do
  timeout 4 rostopic echo -n 1 /Odometry >\"\${odom_file}\" 2>/tmp/robotac_fastlio_odom.err && break
  sleep 1
done

echo 'topics:'
rostopic list 2>/dev/null | grep -E '/livox|/Odometry|/PointCloud|/cloud_registered|/sunray|/path' || true
echo 'lidar hz:'
timeout 8 rostopic hz /livox/lidar 2>&1 || true
echo 'imu hz:'
timeout 8 rostopic hz /livox/imu 2>&1 || true
echo 'odometry sample:'
cat \"\${odom_file}\" 2>/dev/null || true
echo 'odometry hz:'
timeout 8 rostopic hz /Odometry 2>&1 || true
echo 'cloud hz:'
timeout 8 rostopic hz /cloud_registered 2>&1 || true
echo 'lidar log key lines:'
grep -E 'succ|success|not defined|Storage point|ip:|ERROR|WARN|Support only one topic' \"\${lidar_log}\" | tail -n 140 || true
echo 'fastlio log key lines:'
grep -E 'ERROR|WARN|No point|IMU|Lidar|lidar|odom|cloud|initialize|Tilt' \"\${fastlio_log}\" | tail -n 120 || true

kill \"\${fastlio_pid}\" 2>/dev/null || true
kill \"\${lidar_pid}\" 2>/dev/null || true
sleep 2
pkill -P \"\${fastlio_pid}\" 2>/dev/null || true
pkill -P \"\${lidar_pid}\" 2>/dev/null || true

if ! grep -q 'header:' \"\${odom_file}\"; then
  echo 'ERROR: FAST-LIO read-only test did not receive /Odometry.' >&2
  exit 3
fi

echo 'Read-only MAVROS and FAST-LIO smoke tests passed.'
"
else
  echo "Skipped read-only hardware tests."
fi

echo "Deployment completed for ${remote}:${remote_workspace}."
