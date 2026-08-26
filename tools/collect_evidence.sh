#!/usr/bin/env bash
# =============================================================================
# C 组赛前线上初审 · 终端取证采集器（快速换机版 2026-08-23）
# 用法:  bash tools/collect_evidence.sh <阶段>
#   阶段:  set | E01 | E02 | E03 | E04 | E05 | E06 | E07 | E08 | E09 | E10
#          | E11 | E12 | pack   （不传则打印本帮助）
# 特性:
#   - 首次跑 <set> 建立证据目录 $HOME/robotac_precheck_<时间戳>，并把目录打到
#     $HOME/robotac_evidence_env.sh 让其它终端引用。
#   - 每个 E 阶段把输出同时 tee 到证据目录对应日志；涉及 launch 的用后台起、
#     采完精确停掉（pkill 用带括号防自匹配）。
#   - 采完打印「请截屏」提示，方便核对截图与日志对应。
#
# 安全红线（取自取证文档 §2）：全程要求拆桨、上锁、不请求解锁、不切 OFFBOARD、
# 不调实际飞行 start、不挂货物；舵机仅空载测试。任何人不得以本脚本代替飞行。
# =============================================================================

# 不能 set -u：ROS 的 setup.bash 会引用未定义变量（ROBOTAC 队已知坑），统一不加。

# 无阶段参数 → 打印帮助
if [ $# -lt 1 ]; then
    grep -E '^# ' "$0" | sed 's/^# //'
    exit 0
fi
STEP="$1"

# --- 环境准备（全阶段通用）--------------------------------------------------
source /opt/ros/noetic/setup.bash
source ~/robotac_ws/devel/setup.bash 2>/dev/null || true
export ROS_MASTER_URI=http://localhost:11311

# E 目录来自 env 文件（先跑 set 写入）；若缺失且不是 set，则先自动建一个
if [ -f "$HOME/robotac_evidence_env.sh" ]; then
    # shellcheck disable=SC1090
    . "$HOME/robotac_evidence_env.sh"
fi
if [ -z "${E:-}" ] || [ ! -d "${E:-}" ]; then
    E="$HOME/robotac_precheck_$(date +%Y%m%d_%H%M%S)"
fi

log() { echo "[$(date +%H:%M:%S)] $*"; }

# 后台启动 roslaunch：name 用于日志/停止，其余参数传给 roslaunch
_start() {
    local name="$1"; shift
    mkdir -p "${E:-$HOME}"
    setsid nohup roslaunch "$@" > "${E}/launch_${name}.log" 2>&1 < /dev/null &
    echo $! > "${E}/launch_${name}.pid"
    log "已后台启动 ${name} -> ${E}/launch_${name}.log (pid $(cat "${E}/launch_${name}.pid"))"
    sleep 6
}
# 精确停止某 launch（用带括号正则防 bash 自身命令行误匹配）
_stop() {
    local name="$1"
    pkill -f "roslaunch .*${name}" 2>/dev/null
    local pidfile="${E}/launch_${name}.pid"
    if [ -f "$pidfile" ]; then
        kill "$(cat "$pidfile")" 2>/dev/null
        rm -f "$pidfile"
    fi
    log "已停止 ${name}（残余检查见下）"
    sleep 2
}

# 打一条「请截屏」的醒目提示
snap() {
    echo ""
    echo "############################################################"
    echo "#  请对当前终端屏幕截图：$1"
    echo "#  保存为证据目录内的 $2（或用系统截图工具）"
    echo "############################################################"
    echo ""
}

echo "===== 取证阶段: ${STEP} ====="
echo "证据目录: ${E}"

# --- set: 建立证据目录 + env 文件 + 身份表 --------------------------------
do_set() {
    mkdir -p "$E"
    printf 'export E=%q\n' "$E" > "$HOME/robotac_evidence_env.sh"
    log "证据目录: ${E}"
    echo "已在 ${HOME}/robotac_evidence_env.sh 写入 E。其它终端执行："
    echo '  source ~/robotac_evidence_env.sh'
    snap "set 阶段（证据目录已建）" "00_set.png"
}

# --- E01 系统与设备身份 -----------------------------------------------------
do_E01() {
    {
        echo '===== E01 系统与设备身份 ====='
        date; echo
        hostname; hostnamectl; echo
        uname -a; echo
        lsb_release -a 2>/dev/null || true; echo
        id; echo
        ip -4 -br addr; echo
        ip -br link; echo
        free -h; echo
        df -h /
    } | tee "$E/01_identity.log"
    echo
    echo "请在文本编辑器中补全:"
    printf '飞机编号：\n代码来源：\n测试人员：\n安全负责人：\n是否拆桨：是\n' | tee "$E/01_identity.txt"
    echo
    snap "E01 身份（含补全后的 01_identity.txt）" "01_identity.png"
}

# --- E02 源码/构建/单测/仿真 -------------------------------------------------
do_E02() {
    cd ~/robotac_ws
    {
        echo '===== E02 源码检查 ====='
        ./tools/test_01_source.sh 2>&1
        echo
        echo '===== 比赛构建 ====='
        ./tools/test_02_build.sh 2>&1
        echo
        source devel/setup.bash
        echo '===== ROS 包发现 ====='
        rospack find robotac_bringup
        rospack find robotac_localization
        rospack find robotac_examples
        rospack find robotac_servo
        echo
        echo '===== 单元测试 ====='
        ./tools/test_03_unit.sh 2>&1
        echo
        echo '===== 简化仿真与安全故障路径 ====='
        ./tools/test_04_simulation.sh 2>&1
    } | tee "$E/02_software_tests.log"
    echo
    echo "最低要求：RESULT: SUCCESS / ERRORS: 0 / FAILURES: 0"
    snap "E02 软件测试结尾（SUCCESS / 0 errors / 0 failures）" "02_software_tests.png"
}

# --- E03 硬件只读盘点 --------------------------------------------------------
do_E03() {
    {
        echo '===== E03 硬件只读盘点 ====='
        id; echo
        lsusb; echo
        ls -l /dev/ttyTHS* /dev/ttyUSB* /dev/ttyACM* /dev/video* /dev/robotac_* 2>/dev/null; echo
        ip -4 -br addr; echo
        ip -br link; echo
        echo '--- PX4/相机遇占用（若打印则说明有进程占用）---'
        fuser /dev/ttyTHS0 /dev/ttyTHS3 /dev/ttyTHS4 2>/dev/null || echo '无占用'
    } | tee "$E/03_hardware_inventory.log"
    echo
    echo "须确认: PX4实际串口 / 相机video节点 / MID360网卡地址 / 舵机串口 / 三个robotac_别名都在"
    snap "E03 硬件盘点（确认设备别名齐全）" "03_hardware_inventory.png"
}

# --- E04 RGB 相机 ------------------------------------------------------------
do_E04() {
    cd ~/robotac_ws
    _start camera "robotac_bringup" "camera_rgb.launch" "video_device:=/dev/robotac_rgb_camera"
    {
        echo '===== E04 RGB相机 ====='
        timeout 12 rostopic echo -n 1 /camera/rgb/camera_info 2>&1
        echo
        timeout 12 rostopic echo -n 1 /camera/rgb/image_raw/header 2>&1
        echo
        echo '--- 图像频率 ---'
        timeout 12 rostopic hz /camera/rgb/image_raw 2>&1
    } | tee "$E/04_camera_topics.log"
    _stop camera
    echo "记录要求：分辨率 / frame_id / 时间戳 / 频率均合理。"
    snap "E04 相机（分辨率/frame_id/帧率合理）" "04_camera.png"
}

# --- E05 AprilTag 目标识别 ----------------------------------------------------
do_E05() {
    cd ~/robotac_ws
    echo ">>> 请将 Tag36h11 ID 0 标签放在相机正前方，光照充足、标签完整可见。"
    read -r -p "放好 tag 后按回车继续..." _
    _start camera "robotac_bringup" "camera_rgb.launch" "video_device:=/dev/robotac_rgb_camera"
    _start apriltag "robotac_bringup" "apriltag_rgb.launch" "publish_debug_image:=true"
    {
        echo '===== E05 AprilTag目标识别 ====='
        rostopic type /tag_detections 2>&1
        echo
        timeout 20 rostopic echo -n 1 /tag_detections 2>&1
    } | tee "$E/05_tag_topics.log"
    _stop apriltag
    _stop camera
    echo "须证明：Tag36h11 / ID 0 / 尺寸0.15m / 时间戳正确 / 标签移动时位置连续变化。"
    snap "E05 AprilTag（标签族/ID/尺寸/时间戳可见；建议标签移动时再截一张）" "05_tag.png"
}

# --- E06 MID360 点云 + IMU ----------------------------------------------------
do_E06() {
    cd ~/robotac_ws
    echo ">>> 确认 MID360 网线链路正常（carrier=1）。"
    _start lidar "robotac_bringup" "lidar_mid360s.launch"
    {
        echo '===== E06 MID360点云和IMU ====='
        rostopic type /livox/lidar 2>&1
        rostopic type /livox/imu 2>&1
        echo
        echo '--- 点云频率 ---'
        timeout 12 rostopic hz /livox/lidar 2>&1
        echo
        echo '--- IMU 频率 ---'
        timeout 12 rostopic hz /livox/imu 2>&1
        echo
        echo '--- IMU 采样 ---'
        timeout 10 rostopic echo -n 1 /livox/imu 2>&1
    } | tee "$E/06_lidar_topics.log"
    _stop lidar
    echo "点云类型须为 livox_ros_driver2/CustomMsg，IMU 持续输出。"
    snap "E06 MID360（点云类型 + IMU 频率）" "06_lidar.png"
}

# --- E07 FAST-LIO 里程计 -------------------------------------------------------
do_E07() {
    cd ~/robotac_ws
    echo ">>> 需要 MID360 点云/IMU 在跑。若未起，先起雷达："
    echo "    bash tools/collect_evidence.sh E06"
    echo ">>> 准备两人，能稳定抬起整机做小范围平移。"
    read -r -p "雷达就绪并确认抬机人手后，按回车继续启动 FAST-LIO..." _
    _start lidar "robotac_bringup" "lidar_mid360s.launch"
    _start fastlio "robotac_bringup" "fastlio_mid360s.launch"
    {
        echo '===== E07 FAST-LIO定位 ====='
        rostopic type /sunray/odometry 2>&1
        echo
        echo '--- 里程计频率 ---'
        timeout 15 rostopic hz /sunray/odometry 2>&1
        echo
        echo '--- 当前重采样（此刻保持飞机静止）---'
        timeout 10 rostopic echo -n 1 /sunray/odometry 2>&1
        echo
        echo '>>> 现在请两人抬起飞机原地缓慢平移（前/左/上），观察 odom 是否跟随：'
        read -r -p "抬机平移后，按回车采样一次…" _
        echo '--- 抬机平移后采样 ---'
        timeout 10 rostopic echo -n 1 /sunray/odometry 2>&1
    } | tee "$E/07_odometry.log"
    _stop fastlio
    _stop lidar
    echo "观察：前/左/上方向符合预期。若快速漂移/姿态突跳/数据中断则停止记录并说明。"
    snap "E07 FAST-LIO（静止 + 抬机微移两次数值可证明跟随）" "07_odometry.png"
}

# --- E08 MAVROS 只读状态 ------------------------------------------------------
do_E08() {
    cd ~/robotac_ws
    _start mavros "robotac_bringup" "mavros_px4.launch" "fcu_url:=serial:///dev/robotac_px4:921600"
    {
        echo '===== E08 MAVROS只读状态 ====='
        timeout 12 rostopic echo -n 1 /mavros/state 2>&1
        echo
        echo '--- extended_state ---'
        timeout 12 rostopic echo -n 1 /mavros/extended_state 2>&1
    } | tee "$E/08_mavros_state.log"
    _stop mavros
    echo "最低要求：connected: True / armed: False / landed_state: 1"
    snap "E08 MAVROS（armed:False + connected:True + landed_state:1）" "08_mavros.png"
}

# --- E09 真机联合链路 --------------------------------------------------------
do_E09() {
    cd ~/robotac_ws
    _start sensors "robotac_bringup" "sensors.launch"
    _start perception "robotac_bringup" "perception.launch"
    _start flight_base "robotac_bringup" "flight_base.launch" "fcu_url:=serial:///dev/robotac_px4:921600"
    sleep 5
    {
        echo '===== E09 真机联合链路 ====='
        timeout 12 rostopic echo -n 1 /vision_pose_bridge/state 2>&1
        echo
        timeout 12 rostopic echo -n 1 /vision_pose_bridge/healthy 2>&1
        echo
        echo '--- estimator_status ---'
        timeout 12 rostopic echo -n 1 /mavros/estimator_status 2>&1
        echo
        echo '--- timesync_status ---'
        timeout 12 rostopic echo -n 1 /mavros/timesync_status 2>&1
        echo
        echo '--- 本地位置频率 ---'
        timeout 12 rostopic hz /mavros/local_position/pose 2>&1
        echo
        echo '--- mavros state ---'
        timeout 12 rostopic echo -n 1 /mavros/state 2>&1
    } | tee "$E/09_integration.log"
    echo ""
    echo ">>> 保持三个 launch 运行，另一终端核对更多话题："
    echo "    source ~/robotac_evidence_env.sh && cd ~/robotac_ws && source devel/setup.bash"
    echo "    需要确认：外部视觉健康 + MAVROS连接 + 飞机未解锁(armed:False) + 本地位置持续输出"
    snap "E09 联合链路（一张图同时见 视觉healthy + mavros + armed:False + local_position）" "09_integration.png"
    read -r -p "核对完成后按回车关闭联合链路…" _
    _stop flight_base
    _stop perception
    _stop sensors
}

# --- E10 不飞行示例 ----------------------------------------------------------
do_E10() {
    cd ~/robotac_ws
    {
        echo '===== E10 示例：01_fcu_state ====='
        timeout 12 roslaunch robotac_examples 01_fcu_state.launch 2>&1 || true
        echo
        echo '===== 示例：02_local_pose ====='
        timeout 12 roslaunch robotac_examples 02_local_pose.launch 2>&1 || true
        echo
        echo '===== 示例：03_apriltag_detection ====='
        timeout 12 roslaunch robotac_examples 03_apriltag_detection.launch 2>&1 || true
        echo
        echo '===== 示例：05_setpoint_preview（纯预览，不发布）====='
        timeout 14 roslaunch robotac_examples 05_setpoint_preview.launch x:=0.5 y:=0.0 z:=0.6 2>&1 || true
        echo
        echo '--- 05 预览目标话题 ---'
        timeout 10 rostopic echo -n 1 /robotac_examples/setpoint_preview/target 2>&1 || true
        echo
        echo '--- setpoint_position/local 是否被占用（预览应不发布）---'
        rostopic info /mavros/setpoint_position/local 2>&1 || true
    } | tee "$E/10_examples.log"
    echo "05 必须证明：预览节点未向 /mavros/setpoint_position/local 真实发布。"
    snap "E10 示例（01/02/03/05，重点05预览不发布）" "10_examples.png"
}

# --- E11 空载投放机构 --------------------------------------------------------
do_E11() {
    cd ~/robotac_ws
    echo ">>> 前提检查（任一不满足则只记录设备存在，不强行动作）："
    echo "    全部桨叶已拆除 / 不挂货物 / 舵机供电与机械净空确认（当前为ttyUSB0）"
    read -r -p "已拆桨且确认安全后，按回车继续…" _
    _start servo "robotac_servo" "servo.launch" "port:=/dev/robotac_servo"
    {
        echo '===== E11 投放机构状态 ====='
        timeout 10 rostopic echo -n 1 /robotac_servo/connected 2>&1
        echo
        timeout 10 rostopic echo -n 1 /robotac_servo/state 2>&1
        echo
        echo '--- 阻挡(卡货位) ---'
        rosservice call /robotac_servo/set_released "data: false" 2>&1
        timeout 10 rostopic echo -n 1 /robotac_servo/state 2>&1
        echo
        timeout 10 rostopic echo -n 1 /robotac_servo/command_ok 2>&1
        echo
        echo '--- 释放(50° 放货位) ---'
        read -r -p "确认现场负责人同意后，按回车调用一次释放…" _
        rosservice call /robotac_servo/set_released "data: true" 2>&1
        timeout 10 rostopic echo -n 1 /robotac_servo/state 2>&1
        echo
        timeout 10 rostopic echo -n 1 /robotac_servo/command_ok 2>&1
    } | tee "$E/11_servo_test.log"
    _stop servo
    echo "串口成功但机构未完整动作时立即断电检查，不得连续强制调用。"
    snap "E11 舵机（阻挡/释放各一次 + command_ok）" "11_servo.png"
}

# --- E12 安全停止接口 --------------------------------------------------------
do_E12() {
    {
        echo '===== E12 安全停止接口 ====='
        echo '--- /stop 服务 ---'
        rosservice list 2>/dev/null | grep '/stop$' || true
        echo
        echo '--- 当前节点 ---'
        rosnode list 2>&1 || true
        echo
        echo '--- mavros state（若 mavros 未起则提示）---'
        timeout 8 rostopic echo -n 1 /mavros/state 2>&1
    } | tee "$E/12_safety.log"
    echo
    echo "PDF 须写明：安全操作员姓名 / 遥控器接管方式 / Kill开关位置 / 失联保护方式 / 终端停止服务名 / 全程拆桨未解锁未自主飞行"
    snap "E12 安全停止（stop服务 + 节点 + 未解锁状态）" "12_safety.png"
}

# --- pack: 打包证据目录 ------------------------------------------------------
do_pack() {
    cd "$E" || exit 1
    find . -maxdepth 1 -type f -printf '%f\t%s bytes\n' | sort | tee evidence_index.txt
    cd "$(dirname "$E")"
    tar -czf "$(basename "$E").tar.gz" "$(basename "$E")"
    echo
    ls -lh "$E" "$(basename "$E").tar.gz"
    echo
    echo "已打包: $(dirname "$E")/$(basename "$E").tar.gz （队内原始证据，勿原样公开提交）"
}

# --- 分发 ---
case "$STEP" in
    set|E01|E02|E03|E04|E05|E06|E07|E08|E09|E10|E11|E12|pack)
        "do_${STEP}"
        ;;
    *)
        echo "未知阶段: $STEP"
        grep -E '^# ' "$0" | sed 's/^# //'
        exit 1
        ;;
esac
