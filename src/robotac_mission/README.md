# robotac_mission

ROBOTAC C 组（无人机赛道）决赛任务状态机包。**当前为 M1 骨架轮**：启动后仅进入
`WAIT_READY` / `WAIT_START`，**不产生任何飞行动作**（不请求 OFFBOARD / 解锁 /
降落 / 舵机）。飞行状态（TAKEOFF / TRANSIT / SEARCH_TAG / ALIGN_TAG / RELEASE /
RETURN / LAND）留待下一轮。

## 1. 设计要点

- **纯 Python 状态机核**：`src/robotac_mission/state_machine.py` 不依赖 rospy
  时间，时钟由调用方注入，便于单元测试。
- **转移优先级**：操作员 `stop` / 飞控断连 > 安全门 > 任务推进。
- **只记录第一个根因**作为 mission result，后续派生错误作为附加事件。
- **所有阈值来自 YAML**，代码不写现场常量；YAML 加载/校验失败进入 `ERROR`。
- **BOOT 可重入**：`mission_reset`（ERROR -> BOOT）会重新从磁盘读取
  `mission.yaml`，不复用进程内存缓存。

## 2. 输入话题（骨架轮启用安全门）

| 话题 | 类型 | 用途 |
|---|---|---|
| `/mavros/state` | mavros_msgs/State | `connected` 必须 True；`armed` 必须 False |
| `/mavros/extended_state` | mavros_msgs/ExtendedState | 仅 `landed_state=1` 允许启动 |
| `/mavros/local_position/pose` | geometry_msgs/PoseStamped | 有限数、数据年龄 ≤ `pose_timeout`、场地边界内 |
| `/mavros/estimator_status` | mavros_msgs/EstimatorStatus | 关键 flag（attitude/pos_horiz_rel/pos_vert_abs）全 True |
| `/mavros/timesync_status` | mavros_msgs/TimesyncStatus | 往返时延 ≤ `max_rtt_ms` |
| `/vision_pose_bridge/healthy` | std_msgs/Bool | 必须 True |
| `/vision_pose_bridge/state` | std_msgs/String | 必须为 `OK` |
| `/tag_detections` | apriltag_ros/AprilTagDetectionArray | 仅订阅记录（骨架轮不参与启动门） |

> **数据年龄门控**：除 pose 用 `pose_timeout` 外，其余启动门话题（含无 header 的
> Bool/String）统一按**节点收到时刻**判龄，阈值见 `timing.topic_timeout`；某话题
> 停止发布后守卫变红，避免"最后一条消息"长期放行。

预留（飞行轮启用）：`mode_ok`（模式丢失）、`window_ok`（6 分钟窗口）、
`tf_chain_ok`（map→base_link→camera 链）。

## 3. 输出话题与服务

| 接口 | 类型 | 语义 |
|---|---|---|
| `/robotac_mission/start` | std_srvs/Trigger | 仅 `WAIT_START` 且安全门通过时接受；骨架轮接受后停留 `WAIT_START`，不产生控制 |
| `/robotac_mission/stop` | std_srvs/Trigger | 预启动态回 `WAIT_READY`；飞行活动态 → `ABORT_LAND`（飞行轮） |
| `/robotac_mission/reset` | std_srvs/Trigger | `COMPLETE`/`ABORT_LAND` → `WAIT_READY`；`ERROR` → `BOOT` 重读参数 |
| `/robotac_mission/state` | std_msgs/String | 状态枚举（BOOT/WAIT_READY/WAIT_START/ABORT_LAND/COMPLETE/ERROR） |
| `/robotac_mission/state_reason` | std_msgs/String | 人类可读原因（含不就绪原因） |
| `/robotac_mission/active` | std_msgs/Bool | 任务执行中为 True（骨架轮恒 False） |
| `/robotac_mission/target` | geometry_msgs/PoseStamped | 当前名义目标（骨架轮固定发布起飞点，飞行轮改为当前航点） |
| `/robotac_mission/result` | std_msgs/String | 成功/中止/首个故障原因（骨架轮恒空） |

## 4. mission.yaml 字段

`config/mission.yaml` 为参数模板，全部数值为**占位值**：

- `frames`：坐标系（mission_frame / body_frame）。
- `limits`：场地边界 `field_min/max`、最大速度。
- `obstacle`：固定障碍几何（规则参考）与 `no_overfly`。
- `tables`：两桌圆心（两桌同 Tag ID 0 的判别依据）、桌面高、搜索半径。
- `timing`：数据年龄阈值（`pose_timeout` / `tag_timeout` / 各启动门话题
  `topic_timeout`）、时延阈值 `max_rtt_ms`、6 分钟共享窗口 `total_window`、单次
  预算 `flight_budget`、C1/C3 稳定保持、各阶段超时。
- `tag`：Tag 族 / ID / 边长 / 稳定时间。
- `waypoints`：起飞点、去程/返程绕障路径、任务点（名义骨架，map 原点标定后校准）。
- `payload`：本轮固定 `enable: false`。
- `mission.dry_run`：本轮固定 `true`，interfaces 不发送任何控制。

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
> 38 个；**若无 PyYAML，`test_config` 因 `config.py` 顶层 `import yaml` 会报
> ModuleNotFoundError**（Linux/机载的 `python3-yaml` 由 `install_ubuntu20.sh` 覆盖，
> 无此问题）。`test_03_unit.sh` / `rostest` 仍须以 Linux（机载或 WSL Ubuntu 20.04）
> 验证。**`test_03_unit.sh` 含仓库历史红基线**（方案 §9.1：先复跑确认是否既有失败，
> 再叠加本包测试），当前不代表绿色。

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

- 骨架轮不发送任何控制；`start` 仅占位接受，不触发飞行。
- 任何会请求 OFFBOARD / 解锁 / 降落 / 舵机动作的代码，必须通过离线 → 地面 →
  低风险飞行三级审核（飞行轮实现时才涉及）。
- 不得为让测试通过而禁用 PX4 解锁检查、伪造定位健康、无限延长数据超时，
  或在无操作员确认时自动调用 `start`。
- 本包为骨架实现，`ABORT_LAND` / `COMPLETE` 经 ROS 流程不可达，仅定义接口
  （由飞行轮接入）。

## 8. 实机阶段核对（设计注意项）

- **场地 z 边界**：`pose_valid` 严格要求坐标在 `field_min/max` 内（含 z_min=0）。
  FAST-LIO 原点未标定时若 z 轻微为负会卡在 WAIT_READY；map 原点标定后给 z 留裕量
  或加迟滞（方案 §7 已标注"待 map 原点标定"）。
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
- **桌上方航点 z ≥ 桌面高 0.75**（config 校验，R5-2）：takeoff / mission / return
  在桌面上方稳定，z 低于桌面即撞桌；C1 起飞稳定保持须 1.0-2.0m，占位取 1.0。
  绕障航点位于场地中段侧隙、不在桌上，不受此约束。填正式值在桌面高之上再加裕量。
