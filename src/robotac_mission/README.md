# robotac_mission

ROBOTAC C 组（无人机赛道）决赛任务状态机包。**M2 完整 C1-C5 状态机**：
预启动（`WAIT_READY` / `WAIT_START`）+ 飞行七态（TAKEOFF / TRANSIT /
SEARCH_TAG / ALIGN_TAG / RELEASE / RETURN / LAND）+ 终态（ABORT_LAND /
COMPLETE / ERROR）。`mission.flight_enabled=false`（默认）时 start 保持骨架占位
语义——**不进入飞行态**；`true` 后 start 进入 TAKEOFF，由 20Hz 飞行驱动逐态推进，
真实控制仍受 `mission.dry_run` 门控（G1 / 拆桨联调安全网）。

## 1. 设计要点

- **纯 Python 状态机核**：`src/robotac_mission/state_machine.py` 不依赖 rospy
  时间，时钟由调用方注入，便于单元测试。
- **转移优先级**：操作员 `stop` / 飞控断连 > 安全门 > 任务推进。
- **任务推进只前向**：`stage_done` 按 NEXT 表推进；阶段失败一律 `abort` ->
  ABORT_LAND（安全优先；"回退 SEARCH"等复杂重试留待实机数据再定，方案 §16.10）。
- **只记录第一个根因**作为 mission result，后续派生错误作为附加事件。
- **所有阈值来自 YAML**，代码不写现场常量；YAML 加载/校验失败进入 `ERROR`。
- **BOOT 可重入**：`mission_reset`（ERROR -> BOOT）会重新从磁盘读取
  `mission.yaml`，不复用进程内存缓存。
- **坐标系**：起飞点局部归零（`coordinates.py`，方案 §16.2）——start 捕获 home，
  `field_to_map` 发 setpoint、`map_to_field` 做边界/桌区判定；任务启动时用
  `home_yaw + frames.field_yaw_offset` 计算本轮场地轴，使场地 +y 跟随机头前方。

## 2. 输入话题（骨架轮启用安全门）

| 话题 | 类型 | 用途 |
|---|---|---|
| `/mavros/state` | mavros_msgs/State | `connected` 必须 True；`armed` 必须 False |
| `/mavros/extended_state` | mavros_msgs/ExtendedState | 仅 `landed_state=1` 允许启动 |
| `/mavros/local_position/pose` | geometry_msgs/PoseStamped | 有限数、数据年龄 ≤ `pose_timeout`（预启动不验场地边界，见 §8） |
| `/mavros/estimator_status` | mavros_msgs/EstimatorStatus | 关键 flag（attitude/pos_horiz_rel/pos_vert_abs）全 True |
| `/mavros/timesync_status` | mavros_msgs/TimesyncStatus | 往返时延 ≤ `max_rtt_ms` |
| `/vision_pose_bridge/healthy` | std_msgs/Bool | 必须 True |
| `/vision_pose_bridge/state` | std_msgs/String | 必须为 `OK` |
| `/tag_detections` | apriltag_ros/AprilTagDetectionArray | 仅订阅记录（骨架轮不参与启动门） |

> **数据年龄门控**：除 pose 用 `pose_timeout` 外，其余启动门话题（含无 header 的
> Bool/String）统一按**节点收到时刻**判龄，阈值见 `timing.topic_timeout`；某话题
> 停止发布后守卫变红，避免"最后一条消息"长期放行。

飞行态安全门（20Hz，`flight_health.py`）：`mode_ok`（OFFBOARD 丢失）、
`window_ok`（6 分钟共享窗口，start 时校验）、`pose_jump`（位姿跳变）、场地边界
（`map_to_field` 后）、软项去抖（`health_debounce` 次）。`tf_chain_ok`（定位→相机
坐标系链）为 ALIGN_TAG 前置（硬依赖②），TF 不可用时 Tag 恒 None → SEARCH 超时中止。

## 3. 输出话题与服务

| 接口 | 类型 | 语义 |
|---|---|---|
| `/robotac_mission/start` | std_srvs/Trigger | 仅 `WAIT_START` 且安全门通过时接受；`flight_enabled=false` 停留 `WAIT_START`，`true` 进入 TAKEOFF |
| `/robotac_mission/stop` | std_srvs/Trigger | 预启动态回 `WAIT_READY`；飞行活动态 → `ABORT_LAND`（AUTO.LAND 放下） |
| `/robotac_mission/reset` | std_srvs/Trigger | `COMPLETE`/`ABORT_LAND` → `WAIT_READY`；`ERROR` → `BOOT` 重读参数；飞行中拒绝 |
| `/robotac_mission/manual_takeover` | std_srvs/Trigger | 仅 `ABORT_LAND` 态可用：操作员接管后确认 → `COMPLETE`（空中断连唯一出口，M4） |
| `/robotac_mission/state` | std_msgs/String | 状态枚举（预启动/飞行七态/ABORT_LAND/COMPLETE/ERROR） |
| `/robotac_mission/state_reason` | std_msgs/String | 人类可读原因（含不就绪原因） |
| `/robotac_mission/active` | std_msgs/Bool | 任务执行中为 True（TAKEOFF→LAND；预启动恒 False） |
| `/robotac_mission/target` | geometry_msgs/PoseStamped | 当前名义目标（预启动发布起飞点；飞行态由 FlightDriver 转发实际 setpoint） |
| `/robotac_mission/result` | std_msgs/String | 成功/中止/首个故障原因（飞行中止记首个根因，如"任务中止：绕障超时"） |

## 4. mission.yaml 字段

`config/mission.yaml` 为参数模板，全部数值为**占位值**：

- `frames`：坐标系（`mission_frame` / `body_frame`）及机头到场地轴的偏置
  `field_yaw_offset`（弧度）。机头对准场地 +y 时取 `-π/2`；有效旋转为
  `home_yaw + field_yaw_offset`，因此场地整体换朝向无需改航点。
- `limits`：场地边界 `field_min/max`、最大速度。
- `obstacle`：固定障碍几何（规则参考）与 `no_overfly`。
- `tables`：两桌圆心（两桌同 Tag ID 0 的判别依据）、桌面高、搜索半径。
- `timing`：数据年龄阈值（`pose_timeout` / `tag_timeout` / 各启动门话题
  `topic_timeout`）、时延阈值 `max_rtt_ms`、6 分钟共享窗口 `total_window`、单次
  预算 `flight_budget`、C1/C3 稳定保持（`takeoff_hold`/`align_hold`）、航点/释放
  保持（`waypoint_hold`/`release_hold`）、`land_confirm`、各阶段超时 `stage_timeout`。
- `control`：飞行控制策略占位（`rate_hz`/`position_tolerance`/`max_step`/
  `prestream_seconds`/`health_debounce`/`tag_jump_limit`/`pose_jump_limit`，
  随 07/09/10 回填）。
- `tag`：Tag 族 / ID / 边长 / 稳定时间；`stable_samples` 多样本均值窗口（默认 5）。
- `waypoints`：起飞点、去程/返程绕障路径、任务点（名义骨架，map 原点标定后校准）。
- `payload`：控制终点投放开关与重试上限。正式固定点投放使用
  `/robotac_servo/set_released(true)`；舵机的 50 度释放角由
  `robotac_servo/config/servo.yaml` 标定。
- `mission.dry_run`：`true` 时 interfaces 不发送任何控制（G1/拆桨联调）；真飞须 `false`。
- `mission.flight_enabled`：`false`（默认）时 start 为骨架占位，不进入飞行态；
  `true` 后 start 进入 TAKEOFF（真飞/拆桨 dry_run 联调）。

**红线**：不写入借用新机 IP、序列号、设备别名、外参；现场值由实机负责人经地面
与飞行验证后回填。

## 5. 启动步骤（Ubuntu 20.04 + ROS Noetic）

```bash
cd ~/robotac_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch robotac_mission competition_mission.launch
# 观测 state / state_reason；ready 后 state 进入 WAIT_START
```

## 6. 测试

> **状态**：纯单测在 Windows 本地（装有 PyYAML 的 Python，如 Anaconda 3.13）跑绿
> 68 个（state_machine / guards / config / coordinates）；**若无 PyYAML，
> `test_config` 因 `config.py` 顶层 `import yaml` 会报 ModuleNotFoundError**
> （Linux/机载的 `python3-yaml` 由 `install_ubuntu20.sh` 覆盖，无此问题）。
> `test_03_unit.sh` / `rostest` 仍须以 Linux（机载或 WSL Ubuntu 20.04）验证。
> **`test_03_unit.sh` 含仓库历史红基线**（方案 §9.1：先复跑确认是否既有失败，
> 再叠加本包测试），当前不代表绿色。`flight_driver` / `tag_tracker` /
> `interfaces` / `mission_node` 为 rospy 控制层，不单测——飞行态集成靠 G1（预启动
> 范围）与真机 E 阶段。

```bash
# G0 单元测试（纯 Python，无 ROS；Linux 待跑，含历史红基线）
./tools/test_03_unit.sh          # 含 robotac_mission/test/test_*.py
# G1 仿真 rostest（Linux 待跑）
rostest robotac_mission mission_sim.test
```

**G1 覆盖缺口（已知）**：故障注入覆盖解锁 / 离地 / 位姿丢失 / 估计器丢失 /
视觉不健康五类；**"飞控断连"仅在单测层覆盖**（`test_guards` 的 `fcu_connected`）。
集成层缺断连用例，原因是 `fake_fcu` 未提供切换 `connected` 的服务；给 `fake` 加
`set_connected` 会改动 `robotac_examples`（方案 §1.2 不修改第三方），故当前以单测
覆盖 + 记录缺口，待组内定夺是否扩展 fake。同理，除 pose 外各话题的**数据年龄门控**
（R3-1）在 G1 只能通过 `set_pose_stream` 停机端到端验证机制，其余话题的静默死亡由
`topic_fresh` 单测覆盖（fake 无停发服务）。

## 7. 安全限制

- `flight_enabled=false`（默认）时 start 为占位，不触发飞行；`dry_run=true` 时
  interfaces 不发送任何控制。**真飞必须 `dry_run=false` 且 `flight_enabled=true`**，
  并经离线 → 地面 → 低风险飞行三级审核（方案 §16.9 顺序）。
- 不得为让测试通过而禁用 PX4 解锁检查、伪造定位健康、无限延长数据超时，
  或在无操作员确认时自动调用 `start`。
- 飞行中止（安全门/阶段超时）一律 → `ABORT_LAND`：节点放下 AUTO.LAND（离地时），
  落地确认 → `COMPLETE`（首个根因保留在 result）。
- 舵机释放单次调用（`payload.retry_count` 内），禁无限重试；`/dev/robotac_servo`
  缺失时 RELEASE 必中止——C4 硬阻塞，真机 E 段前须解锁。

## 8. 实机阶段核对（设计注意项）

- **预启动不验场地边界（M2 修订，§16.10）**：FAST-LIO map 原点随启动漂移
  （实测 -5.23,-8.06），原始 map 坐标对 `field_min/max` 校验必越界 → 真机卡
  WAIT_READY（G1 fake 位姿≈0 未暴露）。预启动 `pose_fresh` 只验新鲜度+有限数；
  场地边界移到飞行中经 `map_to_field` 后校验（flight_health），越界→ABORT。
- **C3 硬依赖（TF 链/外参）**：`tag_tracker` 依赖 map→camera 帧链（示例 04 曾
  失败）；链不可用时 Tag 恒 None → SEARCH 超时→ABORT。结构在、行为待 09/10 解锁验证。
- **C4 硬依赖（舵机设备）**：`/dev/robotac_servo` 缺失（`/dev/robotac_px4`、
  `robotac_rgb_camera` 已建）；RELEASE 服务调用将失败→ABORT。真机 E 段前须解锁。
- **视觉 state 字符串**：`vision_healthy` 精确匹配 `state == "OK"`（与交接证据
  `vision_pose_bridge state=OK` 一致）；若真实桥大小写/内容变化需同步 YAML/代码。
- **`python3-yaml` 运行时硬依赖**：`config.py` 顶层 import yaml，机载须已安装
  （`install_ubuntu20.sh` 的 `ubuntu20_packages.txt` 已含 `python3-yaml`，
  package.xml 亦声明 exec_depend）。
- **`timing.topic_timeout` 待实机标定**：规则为**阈值 ≥ 2-3× 实测发布周期**。已知
  频率（交接 `rosbag_info.txt`）：estimator ~1Hz（故默认 3.0s，R4-1）、pose ~30Hz、
  timesync ~10Hz、tag ~5.9Hz。`/mavros/state`、`/mavros/extended_state` 真实频率未在
  交接列出，须实测后按规则回填，防抖动误报。
- **`/vision_pose_bridge/state` 重发频率待实测（R4-2）**：本门要求 state 在
  `vision_state`(2.0s) 内持续到达；G1 的 `fake_vision_state` 已改 0.5s 周期重发，但
  真实桥若只在启动时 latch 发一次，2s 后门即红、真机进不了 WAIT_START。须实测
  state 话题真实重发频率；若一次性发布，须调高 `vision_state` 阈值或仅对 healthy
  判龄（state 只验值）。
- **全部飞行航点 z ≥ 相对起飞桌面 0.55 m**（config 校验）：坐标变换会把航点 z
  加到起飞桌面处的 map z；当前桌高 `0.75 m` 时，`z=0.55 m` 对应物理离地约
  `1.30 m`。固定点投放不读取 Tag，
  `payload.drop_target` 是载荷目标，最后一个飞机航点先按舵机偏移补偿，到点后才释放，
  再沿原返航/降落链路继续。
  绕障航点同样采用这个相对起飞桌面的 z 口径。
