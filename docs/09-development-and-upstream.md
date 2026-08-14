# 开发与上游

## 开发范围

参赛开发范围原则上限定于：

- `src/robotac_bringup`：项目组合入口和硬件配置检查。
- `src/robotac_flight`：视觉适配、检查器、任务状态机和仿真。
- `src/robotac_servo`：投放机构接口。
- `config`：经过测量和审核的项目参数。
- `scripts`：构建、验证、部署和证据工具。
- `README.md` 与 `docs`：项目自有文档。

第三方源码必须保持可追踪。发现问题时须先判定问题属于项目配置、集成层或上游缺陷，不得
在第三方目录内提交无法说明依据的临时修改。

## 根目录 18 个脚本

### 离线检查与总审计

| 脚本 | 用途 | 边界 |
| --- | --- | --- |
| `check_flight_contract.py` | 检查本地 MAVROS/FAST-LIO 飞行接口、默认值和门禁契约 | 纯离线 |
| `verify_workspace.sh` | 验证目录、插件、配置、路线和可用仿真回归 | 不连接设备；依环境运行离线 ROS 测试 |
| `flight_goal_audit.py` | 汇总离线、只读、坐标对齐和实飞证据，判断总目标是否仍被阻塞 | 只读分析已有文件 |
| `flight_test_ladder.sh` | 执行离线检查并打印分级验证路径 | 默认不启动 ROS 或设备，不发送命令 |

### 构建、同步与部署

| 脚本 | 用途 | 边界 |
| --- | --- | --- |
| `bootstrap_ubuntu20.sh` | 在 Ubuntu 20.04 安装依赖并完成首次构建 | 安装软件并构建，不启动实机任务 |
| `sync_to_ubuntu.sh` | 用 rsync 同步源码到指定 Ubuntu 工作空间 | 排除构建产物、日志、bag 和 PCD |
| `deploy_aircraft.sh` | 同步、准备权限、构建、静态验证并可做被动冒烟检查 | 不调用 MAVROS 服务，不发布控制 |
| `remote_build_only.sh` | 构建 Livox SDK、AprilTag 和 catkin 工作空间 | 远端构建，不启动节点 |
| `remote_validate_no_launch.sh` | 检查远端包、launch、共享库和静态配置 | 明确不启动节点 |
| `remote_preflight_readonly.sh` | 收集远端构建、磁盘和飞行前只读状态 | 不发送飞行控制 |
| `remote_readonly_audit.sh` | 审计远端设备、网络、进程和只读 ROS 状态 | 只读诊断 |

### 运行时被动观察

| 脚本 | 用途 | 边界 |
| --- | --- | --- |
| `lidar_runtime_check.sh` | 检查现有 Livox 节点、类型、频率和样本 | 不启动节点，不发布消息 |
| `fastlio_runtime_check.sh` | 检查现有 FAST-LIO 节点、里程计、点云和 TF | 不启动节点，不发布消息 |
| `collect_readonly_flight_evidence.sh` | 采集现有 MAVROS、雷达、FAST-LIO 和视觉只读证据包 | 不启动设备节点，不调用服务 |
| `collect_frame_alignment_evidence.sh` | 启动只订阅的坐标对齐观察器并保存证据 | 要求视觉实机输出保持关闭 |

### 证据分析

| 脚本 | 用途 | 输入 |
| --- | --- | --- |
| `analyze_readonly_flight_evidence.py` | 判断只读证据中的连接、频率、时间同步和前置检查 | 只读证据目录 |
| `analyze_frame_alignment_evidence.py` | 判断 `+X/+Y/+Z` 与航向运动的方向、尺度和帧 | 一个或多个坐标对齐 JSON |
| `analyze_active_flight_evidence.py` | 审计航点、原始设定点、视觉、本地位置、起降和终态 | 受控实飞观察 JSON |

脚本名称中的 `active` 仅表示证据阶段，分析器不执行飞行控制。受控实飞必须按
[安全与验证](06-safety-and-validation.md) 的规定组织。

## 测试入口

`src/robotac_flight/test` 中的测试使用独立 ROS master 和模拟 MAVROS/传感器输入：

| 入口 | 覆盖内容 |
| --- | --- |
| `run_vision_bridge_sim.sh` | 位姿桥正常、异常、预览与输出隔离 |
| `run_flight_preflight_sim.sh` | 飞行前只读检查及拒绝条件 |
| `run_closed_loop_sim.sh` | 通用航点闭环与 ENU/NED 转换 |
| `run_dynamic_waypoints_sim.sh` | `PoseArray` 动态航点更新 |
| `run_route_file_sim.sh` | YAML 路线、停留和任务完成 |
| `run_setpoint_consumer_gate_sim.sh` | MAVROS 设定点消费者门禁 |
| `run_flight_fault_sim.sh` | 数据断流、消费者丢失和异常中止 |
| `run_tag_payload_mission_sim.sh` | C 组航线、Tag、投放、返航和降落状态流 |

上述测试均为硬件隔离软件测试，不包含真实飞行动力学、光照、雷达退化、网络抖动、机械投放或
PX4 实际融合误差。

`src/robotac_servo/scripts/servo_cycle_test.py` 将操作实际舵机接口，不属于普通离线测试。
该脚本仅限在独立地面工位、机构净空和人员监督条件下使用，不得纳入日常自动测试。

## 版本锁定

`versions.lock` 记录来源仓库、精确提交和导入路径；`.repos` 为支持 `vcs import` 的上游
仓库描述。更新第三方组件时：

1. 记录上游仓库、旧/新提交和更新原因。
2. 在隔离分支中更新，不混入任务参数修改。
3. 检查许可证、构建方法、消息和 launch 兼容性。
4. 运行契约、工作空间和全部硬件隔离测试。
5. 对涉及驱动、坐标或 MAVROS 的变化重新执行分级实机验证。
6. 更新 `versions.lock`、`.repos` 和必要的中文用途说明。

## 上游来源

| 组件 | 来源说明 |
| --- | --- |
| Livox 驱动、SDK、FAST-LIO、相机驱动 | 从已验证的 Sunray 源码修订导入，精确提交见 `versions.lock` |
| MAVROS | 官方 `mavlink/mavros`，当前锁定版本见 `versions.lock` |
| AprilTag | 官方 `AprilRobotics/apriltag` |
| AprilTag ROS | 官方 `AprilRobotics/apriltag_ros` |

“从已验证修订导入”仅表示版本来源可追踪，不代表当前飞机已经通过完整验证。

## 项目修改边界

当前已知集成需求可能涉及第三方目录，例如构建依赖顺序或 MAVROS 本地插件场景适配。
维护这类差异时应：

- 将修改限制在具有明确依据的最小范围。
- 保留上游提交基线和清晰 diff。
- 不得删除许可证、作者和上游文档。
- 不得将本项目特定默认值表述为上游通用行为。
- 评估能否提交上游或移到 `robotac_*` 集成层。

## 项目许可证

Robotac 项目自有代码与文档采用根目录 [MIT License](../LICENSE)。该许可证允许个人、教学、
竞赛及商业用途，也允许修改、分发和再许可；复制或分发时必须保留版权声明和许可声明。
许可证同时包含免责声明，不对适销性、特定用途适用性或非侵权性提供保证。

根许可证仅适用于 YunDrone-Team 有权授权的项目自有内容。第三方源码、资源及其本地修改仍受
对应上游许可证约束，根许可证不替代、不放宽第三方条款。

## 第三方许可证入口

- Livox SDK：`src/Livox-SDK2/LICENSE.txt`
- Livox ROS 驱动：`src/livox_ros_driver2/LICENSE.txt`
- FAST-LIO：`src/fast_lio/LICENSE`
- MAVROS：`src/mavros/LICENSE.md` 及同目录各许可证文件
- AprilTag：`src/apriltag/LICENSE.md`
- AprilTag ROS：`src/apriltag_ros/LICENSE`

组件内还可能包含第三方子依赖许可证，发布二进制或重新分发源码前必须一并检查。

## 提交前检查

```bash
./scripts/check_flight_contract.py
./scripts/verify_workspace.sh
git diff --check
```

人工核对项：

- 接口、launch、配置和文档是否同步。
- 默认值是否仍保持控制关闭。
- 是否误提交真实地址、UID、凭据、个人路径、bag、PCD 或构建产物。
- 对验证状态的表述是否严格对应实际证据。
- 第三方修改是否更新来源、许可证检查和差异说明。

[返回文档索引](README.md)
