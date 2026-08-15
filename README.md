# Robotac 无人机教学示例工作空间

本 Repo 面向 ROBOTAC AIROBOTIC C 组参赛选手，运行环境为 Ubuntu 20.04、ROS Noetic
和 Python 3.8。仓库提供定位接入、基础飞行、AprilTag 识别与对准、投放机构控制等独立
示例，不提供完整比赛任务程序。

## 本 Repo 作用

- 启动 MID360、RGB 相机、FAST-LIO、AprilTag 和 MAVROS 等基础组件。
- 将本地 `Odometry` 转换为 PX4 外部视觉使用的 `PoseStamped`。
- 通过 11 个编号示例说明飞控状态检查、位置设定点、航点、视觉对准和投放机构接口。
- 提供纯单元测试、简化假飞控仿真和只读硬件检查。

参赛选手须自行实现固定障碍绕行、比赛航线、完整任务流程、总计时和异常处理。
本 Repo 不包含可以直接用于比赛的 C1 至 C5 完整流程。

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

## 安装与构建

```bash
cd ~/robotac_ws
./tools/install_ubuntu20.sh
./tools/test_02_build.sh
source devel/setup.bash
```

增量构建执行：

```bash
./tools/build.sh
```

依赖安装和构建的详细条件见[环境与构建](docs/02-environment-and-build.md)。

## 示例顺序

| 编号 | launch | 默认行为 |
| --- | --- | --- |
| 01 | `01_fcu_state.launch` | 只读显示 FCU 状态 |
| 02 | `02_local_pose.launch` | 只读显示本地位姿 |
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
./tools/test_01_source.sh
./tools/test_03_unit.sh
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
- [旧版迁移说明](docs/11-migration.md)

## 许可证

项目自有代码和文档采用 [MIT License](LICENSE)。第三方目录继续适用各自的许可证。
