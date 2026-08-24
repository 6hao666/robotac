# Robotac 无人机教学示例工作空间

本 Repo 面向 ROBOTAC AIROBOTIC C 组参赛选手，运行环境为 Ubuntu 20.04、ROS Noetic
和 Python 3.8。仓库提供定位接入、基础飞行、AprilTag 识别与对准、投放机构控制等独立
示例，参赛选手可据此安排自己的比赛任务流程。

## 本 Repo 作用

- 启动 MID360、RGB 相机、FAST-LIO、AprilTag 和 MAVROS 等基础组件。
- 将本地 `Odometry` 转换为 PX4 外部视觉使用的 `PoseStamped`。
- 通过 11 个编号示例说明飞控状态检查、位置设定点、航点、视觉对准和投放机构接口。
- 提供纯单元测试、简化假飞控仿真和只读硬件检查。

参赛选手可根据正式规则安排固定障碍绕行、比赛航线、任务流程、总计时和异常处理。

## 安全要求

`01` 至 `05`、`09` 为只读或预览示例。`06`、`07`、`08`、`10` 会在手动启动后请求
OFFBOARD、解锁和降落；`11` 会操作投放机构。

会控制飞机的示例遵守以下要求：

- launch 启动后不产生飞行动作，必须另行调用节点的 `~start` 服务。
- `~stop` 请求中止并降落。
- 每次节点运行只执行一次；再次测试须重新启动节点。
- 启动前检查 FCU、落地状态、本地位置、外部视觉、estimator、timesync 和设定点订阅。
- 运行中发生数据中断、OFFBOARD 丢失或 Tag 超时后，程序不再更新飞行目标，并请求降落。
- 程序检查不能替代遥控器、急停措施、安全操作员和现场清场。

地面调试时必须拆除桨叶。未完成标定、无人监管或无法人工接管时，不得启动会控制飞机的
示例。

## 工作空间组成

| 路径 | 内容 |
| --- | --- |
| `src/robotac_bringup` | 硬件、感知与基础飞行组件启动 |
| `src/robotac_localization` | 外部视觉预览与 PX4 输出 |
| `src/robotac_examples` | 11 个编号教学示例及测试 |
| `src/robotac_servo` | 投放机构串口驱动与地面标定 |
| `tools` | 安装、构建、测试和同步工具 |
| `docs` | 中文说明、教程和迁移资料 |

`mavros`、`apriltag`、`apriltag_ros`、`fast_lio`、`livox_ros_driver2`、`Livox-SDK2`
和 `web_cam` 为固定版本的第三方源码，参赛开发通常不需要修改。

## 获取源码与安装

目标机须预先安装 Ubuntu 20.04 和 ROS Noetic。先安装 Git：

```bash
# 更新软件包索引。
sudo apt update
# 安装 Git。
sudo apt install -y git
# 返回当前用户主目录，仓库将保存为 ~/robotac。
cd ~
```

GitHub 和 Gitee 选择一种即可。网络能够稳定访问 GitHub 时使用：

```bash
# 从 GitHub 获取 Robotac 源码。
git clone https://github.com/YunDrone-Team/robotac.git
```

网络条件不佳时使用 Gitee 镜像：

```bash
# 从 Gitee 镜像获取 Robotac 源码。
git clone https://gitee.com/yundrone_sunray2023/robotac.git
```

仓库根目录 `~/robotac` 本身就是 catkin 工作空间，不需要另外创建工作空间目录，也不要将
本仓库放入另一个 catkin 工作空间。完成 clone 后执行：

```bash
# 进入 Robotac 仓库根目录。
cd ~/robotac
# 在已安装 ROS Noetic 的 Jetson Orin Nano、Nano Super 或 NX 上完成软件部署。
./tools/setup_jetson_orin.sh
# 加载当前工作空间的 ROS 环境。
source devel/setup.bash
```

部署脚本不会配置真实硬件，也不会启动 ROS 节点。脚本完成后，表示依赖已经安装、工作空间
已经构建并完成离线检查；这不表示雷达网络、
稳定设备别名、飞控连接或投放机构标定已经完成。上述项目仍须按现场文档逐项确认。
旧入口 `./tools/setup_orin_nano_super.sh` 继续保留，并转发到相同的部署流程。

日常增量构建使用：

```bash
# 执行一次增量构建。
./tools/build.sh
```

需要验证仓库中包含的全部第三方源码时执行 `./tools/build_full.sh`，并按需加载
`devel_full/setup.bash`。日常比赛运行继续加载 `devel/setup.bash`。

需要分步排障时，仍可单独执行 `install_ubuntu20.sh`、`test_02_build.sh` 和其他测试脚本。
详细条件见[环境与构建](docs/02-environment-and-build.md)。

## 位置控制所需条件

`06`、`07`、`08` 和 `10` 使用 PX4 输出的 `/mavros/local_position/pose` 计算当前位置，
并向 `/mavros/setpoint_position/local` 发送目标。它们不会自行启动雷达、FAST-LIO、MAVROS
或外部视觉接口，不能只启动示例 launch 后直接执行。

本 Repo 的位置来源依次经过：MID360 点云与 IMU、FAST-LIO `/Odometry`、修正后的
`/sunray/odometry`、MAVROS 外部视觉输入、PX4 estimator 融合，最后由 MAVROS 输出本地
位姿。FAST-LIO 有里程计数据，只能说明定位计算已经开始；还需确认外部视觉持续送入 PX4、
PX4 的位置状态有效，并检查最终本地位姿没有明显漂移、跳变或方向错误。

标准启动和只读检查步骤见[部署与运行](docs/07-deployment-and-operations.md)，定位原理见
[架构与接口](docs/04-architecture-and-interfaces.md)。

## 示例顺序

| 编号 | launch | 默认行为 |
| --- | --- | --- |
| 01 | `01_fcu_state.launch` | 只读显示 FCU 状态 |
| 02 | `02_local_pose.launch` | 只读显示定位链路和本地位姿 |
| 03 | `03_apriltag_detection.launch` | 显示相机坐标系 Tag 检测 |
| 04 | `04_apriltag_local_pose.launch` | 发布本地 Tag 位姿和偏差 |
| 05 | `05_setpoint_preview.launch` | 仅生成目标位姿 |
| 06 | `06_hover.launch` | 起飞、悬停、降落 |
| 07 | `07_move_relative.launch` | 单次相对位移、返回、降落 |
| 08 | `08_waypoints.launch` | 执行简短 YAML 航点 |
| 09 | `09_tag_centering_preview.launch` | 仅计算视觉对准目标 |
| 10 | `10_tag_centering_flight.launch` | 起飞、对准 ID 0、稳定后降落 |
| 11 | `11_payload_release.launch` | 单独控制阻挡或释放位置 |

各示例的前置条件、命令和失败判断见[教程索引](docs/tutorials/README.md)。

## 离线验证

```bash
# 检查源码、接口和文档链接。
./tools/test_01_source.sh
# 运行不连接硬件的单元测试。
./tools/test_03_unit.sh
# 运行简化假飞控仿真。
./tools/test_04_simulation.sh
```

`test_04_simulation.sh` 依赖已经完成的 catkin 构建。真实设备只读检查须在机载 Ubuntu 主机
单独执行 `tools/test_05_hardware_readonly.sh`。自动验证不连接设备，不发送飞行设定点，不调用
飞控服务。

## 规则依据

示例依据 2026 年 8 月 12 日《ROBOTAC AIROBOTIC 总决赛 C 组无人机竞赛规则》整理。
AprilTag 默认采用 `Tag36h11 ID 0`，黑色编码区域边长为 `0.15 m`；视觉输出包含目标位置或
水平偏差；示例 10 默认稳定对准 3 秒。

规则摘要只用于说明示例对应关系。场地、设备、任务、计时和判分以组委会最新正式规则、
修订通知和技术公报为准。对应关系见[规则与示例的对应关系](docs/05-competition-examples.md)。

## 文档

- [文档索引](docs/README.md)
- [本 Repo 结构](docs/01-project-overview.md)
- [环境与构建](docs/02-environment-and-build.md)
- [硬件与配置](docs/03-hardware-and-configuration.md)
- [架构与接口](docs/04-architecture-and-interfaces.md)
- [规则与示例的对应关系](docs/05-competition-examples.md)
- [安全与验证](docs/06-safety-and-validation.md)
- [部署与运行](docs/07-deployment-and-operations.md)
- [故障排查](docs/08-troubleshooting.md)
- [开发与上游](docs/09-development-and-upstream.md)
- [舵机投放机构标定](docs/10-servo-release-calibration.md)

## 许可证

项目自有代码和文档采用 [MIT License](LICENSE)。第三方目录继续适用各自的许可证。
