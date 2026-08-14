# Robotac 无人机竞赛工作空间

Robotac 是面向 ROBOTAC AIROBOTIC C 组无人机任务的 ROS Noetic 工作空间。
该工作空间集成 Livox MID360、FAST-LIO、RGB 相机、AprilTag、MAVROS/PX4 和投放舵机，
构成可分级验证的实机任务系统。

本仓库供参赛选手和指导老师进行部署、验证与二次开发。使用者应具备 Linux 命令行和
ROS 基础，无须预先掌握本项目的节点关系。

> **安全提示：** 本项目包含可能影响真实无人机的控制程序。任何实机操作都必须
> 在合规封闭场地内，由指导老师或现场安全负责人组织，并完成标定、地面检查、
> PX4 失控保护确认和人员隔离。编译通过或话题正常出数均不构成实机起飞条件。

## 本 Repo 作用

- 接收 MID360 点云和 IMU 数据，运行 FAST-LIO 本地定位。
- 将经过坐标系校准和健康检查的位姿送入 MAVROS 外部视觉接口。
- 采集 RGB 图像并识别 AprilTag。
- 以本地相对坐标执行航点、目标对准、投放、返航和降落任务。
- 控制 USB PWM 舵机，并检查串口写入状态。
- 通过离线、仿真、只读观察和实机证据工具逐级验证系统。

本 Repo 不构成可直接投入飞行的成品飞控，也不替代 PX4 参数配置、传感器标定、现场安全制度
或组委会规则。仓库中的航线、Tag 编号、等待时间和超时参数都是历史开发示例，
不得直接作为当前比赛参数使用。

## 系统组成

| 模块 | 主要职责 |
| --- | --- |
| `src/robotac_bringup` | 组合相机、雷达、FAST-LIO、MAVROS、AprilTag 和任务节点 |
| `src/robotac_flight` | 外部视觉转换、飞行前检查、航点与竞赛任务状态机 |
| `src/robotac_servo` | USB PWM 舵机开关控制与写入状态发布 |
| `src/livox_ros_driver2`、`src/Livox-SDK2` | Livox 驱动与 SDK |
| `src/fast_lio` | 激光雷达惯性里程计 |
| `src/mavros` | PX4 与 ROS 之间的 MAVLink 接口 |
| `src/apriltag`、`src/apriltag_ros` | AprilTag 检测与 ROS 封装 |
| `src/web_cam` | V4L2 RGB 相机驱动 |
| `config` | 项目级硬件、标定、任务和安全门禁配置 |
| `scripts` | 构建、同步、只读检查、证据采集和离线审计工具 |

第三方目录保留上游原文、许可证和变更记录。项目自有中文说明统一放在本页和
[`docs/`](docs/README.md) 中。

## 支持环境

- 机载计算机：能够运行 Ubuntu 20.04 的 x86_64 或 aarch64 设备。
- 操作系统：Ubuntu 20.04。
- ROS：ROS Noetic。
- 飞控：PX4，通过 MAVROS 串口连接。
- 雷达：Livox MID360 系列。
- 相机：V4L2 兼容 RGB 相机。
- 投放机构：兼容当前串口协议的 USB PWM 控制器。

macOS 仅用于编辑、版本管理和源码同步，不属于本仓库支持的 ROS 构建与运行平台。
精确上游版本见 [`versions.lock`](versions.lock)，源码来源见 [`.repos`](.repos)。

## 离线验证入口

以下步骤仅执行源码和构建检查，不连接设备、不启动 ROS、不发送任何飞行命令。

### 1. 前置阅读

前置阅读顺序如下：

1. [项目总览](docs/01-project-overview.md)
2. [环境与构建](docs/02-environment-and-build.md)
3. [硬件与配置](docs/03-hardware-and-configuration.md)
4. [安全与验证](docs/06-safety-and-validation.md)

### 2. Ubuntu 工作空间准备

仓库应位于 Ubuntu 20.04 主机的工作目录中，例如：

```bash
cd <工作空间目录>
./scripts/bootstrap_ubuntu20.sh
```

该脚本负责安装本仓库所需的本机依赖、解析 ROS 依赖并执行 catkin 构建。
执行前须审阅脚本，并确认软件源、权限和磁盘空间符合所在实验室要求。

### 3. 环境加载与离线检查

```bash
cd <工作空间目录>
source /opt/ros/noetic/setup.bash
source devel/setup.bash
./scripts/check_flight_contract.py
./scripts/verify_workspace.sh
```

上述检查用于验证文件、参数契约、包发现和测试入口。检查通过不足以证明实机标定完成，
也不足以证明无人机具备安全起飞条件。

### 4. 分级验证流程

```bash
./scripts/flight_test_ladder.sh
```

该脚本默认仅执行离线检查并打印后续阶段说明，不会启动 ROS、打开串口、改变模式、
解锁飞控或调用任务启动服务。进入实机阶段前，须依据
[安全与验证](docs/06-safety-and-validation.md) 完成逐级审核。

## 配置前置要求

项目配置不得跨飞机直接复用。至少须现场确认：

- MID360 与机载计算机的网络地址、网卡和数据端口。
- PX4 串口设备、波特率、权限和稳定设备别名。
- RGB 相机设备、内参、畸变参数和图像方向。
- 雷达、IMU、机体、相机之间的外参和坐标轴方向。
- PX4 外部视觉融合、时间同步和 Offboard 失联保护。
- 舵机设备、关舱角度、开舱角度和机械限位。
- 最新正式规则中的场地尺寸、Tag 编号、任务时间和判分条件。

对应文件与门禁说明见 [硬件与配置](docs/03-hardware-and-configuration.md)。禁止将
`camera_extrinsics.launch` 的零值、FAST-LIO 的单位外参或示例航线当作实机标定值。

## 运行边界

本仓库对“数据可观测”和“飞行输出”实行隔离：

- FAST-LIO 和外部视觉节点默认只发布预览话题。
- `full_system.launch` 默认不启用 MAVROS。
- 航点和竞赛任务 launch 默认 `enable_control:=false`。
- 自动切换模式、自动解锁、自动降落和投放均有独立开关，默认关闭。
- 外部视觉输出和飞行控制还受 `config/deployment.yaml` 门禁约束。
- 飞行状态机启动前将检查数据新鲜度、坐标系、消费者、时间同步和飞控状态。

上述门禁属于强制安全约束，用于阻止不完整配置进入控制路径，不得绕过。本文档不提供
绕过标定、伪造检查结果或无人监管起飞的方法。

## 当前验证状态

| 层级 | 当前证据 | 结论边界 |
| --- | --- | --- |
| 源码与构建 | 工作空间构建、静态契约和离线测试已有记录 | 说明代码可构建，不说明硬件适配完成 |
| 被动实机链路 | MAVROS 被动连接、MID360 数据、FAST-LIO 输出已有记录 | 说明通信和定位链路曾工作，不说明可控飞行 |
| 仿真 | 任务状态流转、航点、投放指令和异常门禁有覆盖 | 说明软件闭环可测试，不等同真实动力学和环境 |
| 完整实飞 | 尚无足够公开证据 | 不声明完成赛道自主飞行或重复投放精度 |
| 规则适配 | 参数来自历史开发阶段 | 不声明与当前最终规则完全一致 |

详细验证矩阵和证据要求见 [安全与验证](docs/06-safety-and-validation.md)。正式比赛始终以
组委会最新规则、技术通知、现场裁判解释和安全要求为准。

## 文档导航

| 文档 | 内容 |
| --- | --- |
| [文档索引](docs/README.md) | 文档目录与阅读顺序 |
| [项目总览](docs/01-project-overview.md) | Repo 定位、系统组成与数据流 |
| [环境与构建](docs/02-environment-and-build.md) | Ubuntu 20.04 环境准备与构建流程 |
| [硬件与配置](docs/03-hardware-and-configuration.md) | 单机设备、标定与部署门禁要求 |
| [架构与接口](docs/04-architecture-and-interfaces.md) | launch、节点、话题、服务与坐标系定义 |
| [竞赛任务](docs/05-competition-mission.md) | C 组任务状态机与参数复核要求 |
| [安全与验证](docs/06-safety-and-validation.md) | 离线检查至受控实飞的分级验证要求 |
| [部署与运维](docs/07-deployment-and-operations.md) | 同步、构建、取证与进程清理流程 |
| [故障排查](docs/08-troubleshooting.md) | 常见故障的安全定位方法 |
| [开发与上游](docs/09-development-and-upstream.md) | 脚本、测试、版本和第三方边界 |

## 开发规范

- 修改任务逻辑前，应复现离线检查和仿真结果。
- 参数变更必须记录依据、单位、坐标系、测量方法和回退值。
- 实机日志须标注软件版本、配置版本、验证级别和现场条件。
- 仓库中不得提交密码、真实设备地址、个人绝对路径、飞控 UID 或现场人员信息。
- 不得通过修改第三方源码掩盖项目配置问题；确需修改时，须记录原因和上游差异。
- 构建成功、话题存在、数据有频率、任务仿真通过和实机飞行成功是五种不同证据。

## 许可证与贡献

各第三方组件按其目录中的许可证使用。本仓库的来源锁定和本地修改边界见
[开发与上游](docs/09-development-and-upstream.md)。提交变更前至少执行：

```bash
./scripts/check_flight_contract.py
./scripts/verify_workspace.sh
git diff --check
```

涉及飞行接口、坐标系、门禁或任务参数的变更，还应补充相应仿真和分级验证证据。
