# 硬件与配置

## 基本原则

每台飞机必须有独立的设备清单、网络记录、串口记录、标定结果和 PX4 参数快照。
本仓库配置中的地址、设备名和单位外参仅为开发基线，不属于跨机通用值。

所有示例中的占位符均须现场核对：

- `<飞机地址>`：开发机访问机载计算机的地址。
- `<机载网卡地址>`：连接 MID360 的机载网卡地址。
- `<雷达地址>`：MID360 当前地址。
- `<飞控串口设备>`：稳定的 PX4 设备路径。
- `<舵机串口设备>`：稳定的 PWM 控制器设备路径。
- `<相机设备>`：稳定的 V4L2 捕获设备路径。

## 配置文件总表

| 类别 | 文件 | 用途 |
| --- | --- | --- |
| AprilTag | `config/apriltag/settings.yaml` | 检测器参数 |
| AprilTag | `config/apriltag/tags.yaml` | Tag 家族、编号、尺寸和名称 |
| 相机 | `config/camera/rgb.yaml` | RGB 相机内参和畸变 |
| 相机 | `config/camera/rgb_640x480.yaml` | 640x480 标定示例 |
| 部署 | `config/deployment.yaml` | 实机部署与控制门禁 |
| 部署 | `config/deployment_sim.yaml` | 隔离仿真的门禁配置 |
| 定位 | `config/fastlio/mid360s.yaml` | FAST-LIO 话题、外参和滤波参数 |
| 定位 | `config/fastlio/path_a_vision_pose.yaml` | Path A 位姿转换和健康阈值 |
| 定位 | `config/fastlio/vision_bridge.yaml` | `/Odometry` 协方差桥配置 |
| 定位 | `config/fastlio/vision_bridge_sim.yaml` | 协方差桥仿真配置 |
| 飞行 | `config/flight/local_waypoints.yaml` | 通用航点与控制限制 |
| 飞行 | `config/flight/local_waypoints_simple_box.yaml` | 简单箱形航线示例 |
| 飞行 | `config/flight/payload_drop_box_test.yaml` | 投放箱测试航线示例 |
| 飞行 | `config/flight/posearray_waypoints_example.yaml` | 动态 `PoseArray` 航点示例 |
| 飞行 | `config/flight/tag_payload_mission.yaml` | C 组任务航线、Tag 和时序 |
| 雷达 | `config/lidar/mid360s.json` | MID360 网络和数据格式 |
| MAVROS | `config/mavros/px4.yaml` | MAVROS 与 PX4 参数 |
| MAVROS | `config/mavros/px4_pluginlists.yaml` | 本地定位所需插件白名单 |
| udev | `config/udev/99-robotac-px4.rules.template` | PX4 稳定设备名模板 |
| udev | `config/udev/99-robotac-rgb-camera.rules.template` | RGB 相机稳定设备名模板 |
| udev | `config/udev/99-robotac-servo.rules.template` | 舵机控制器稳定设备名模板 |

## MID360 网络

在 `config/lidar/mid360s.json` 中设置 `<机载网卡地址>` 和 `<雷达地址>`。两者应位于现场
设计的同一子网且不冲突。配置完成后须先执行链路和只读数据检查，不得同时进入飞行控制阶段。

检查内容包括：

- 网卡链路、地址和路由是否正确。
- 雷达地址是否与设备当前设置一致。
- 防火墙是否允许 Livox 数据端口。
- `/livox/lidar` 与 `/livox/imu` 的类型、频率和时间戳是否稳定。

公开文档、提交信息和截图不得包含真实飞机网络布局。

## FAST-LIO 与外参

`config/fastlio/mid360s.yaml` 中的雷达到 IMU 外参、时间同步和滤波参数必须根据实际安装
测量。FAST-LIO 上游输出使用：

- 世界帧：`camera_init`
- 估计器机体帧：`body`

上述帧不等同于 `map` 和 `base_link`。只有测量并记录机体安装关系后，方可配置外部
视觉转换。单位旋转和零平移仅限台架检查，不得作为实飞标定结论。

## PX4 与 MAVROS

飞控串口通常使用 `921600` 波特率，但设备路径必须现场确认。应基于已核验的厂商、产品和
序列属性生成 udev 稳定别名，再将 `<飞控串口设备>` 传给 launch。

`config/mavros/px4_pluginlists.yaml` 默认只启用本地任务需要的插件：命令、IMU、本地位置、
参数、原始设定点、系统状态、时间同步和外部视觉。全局定位、航点上传和 GPS 相关插件
默认关闭，因此本系统不声明支持全局 GPS 任务。

实机前必须由飞控负责人确认：

- 串口稳定且 `/mavros/state.connected` 可被动观察为真。
- PX4 外部视觉位置和航向融合来源正确。
- MAVLink 时间同步稳定，往返延迟在项目阈值内。
- Offboard 失联超时和动作经过地面验证并留存记录。
- 遥控器接管、急停、地理围栏和电池保护符合现场要求。

## RGB 相机与 AprilTag

相机设备应使用稳定别名 `<相机设备>`。`config/camera/*.yaml` 的内参只对对应分辨率、镜头、
对焦和安装状态有效。更换相机、镜头、分辨率或固定方式后必须重新标定。

`camera_extrinsics.launch` 提供 `base_link -> camera_rgb_optical_frame` 静态变换入口，默认值
为零占位。必须测量平移、旋转和光学帧方向后再用于实机。

`config/apriltag/tags.yaml` 中的边长参与位姿估计，应对应检测算法使用的黑色编码区域定义，
并与实际打印、缩放和安装结果核对。任务 Tag 编号还须符合最新规则。

## 舵机与投放机构

当前控制器协议为 `115200` 波特率、通道 1、50 Hz，默认开舱角度为 45 度。USB 控制器
常见为 CH340/HL-340，但必须用本机设备信息确认。舵机回执仅证明串口写入成功，不是机械
角度传感器反馈。

地面检查必须覆盖：

- 设备稳定别名和 `dialout` 权限。
- 开关方向是否正确，关舱是否可靠。
- 机械限位、卡滞、电流和负载情况。
- 断电、节点退出和异常中止后的安全位置。

## udev 与权限

3 个 `.template` 文件均为模板。安装前应读取目标设备属性，替换 PX4 模板中的厂商、产品和
序列字段，并确认相机、舵机模板确实匹配本队硬件。安装后重新加载规则并重新插拔设备。

- 串口用户加入 `dialout`。
- 相机用户加入 `video`。
- 重新登录后再验证组权限。
- 不得使用全局可写权限代替设备规则。

## 坐标系与标定记录

至少建立以下关系：

```text
camera_init -> body                   FAST-LIO 输出
odom -> base_link                     外部视觉适配后的约定
map -> base_link                      MAVROS 本地位置输出
base_link -> camera_rgb_optical_frame 相机外参
robotac_start_body                    任务启动瞬间建立的本地任务帧
```

每次标定应记录设备、安装状态、方法、原始数据、结果、单位、坐标轴图、操作者和对应 Git SHA。
禁止记录无法追溯来源的孤立数值。

## 部署门禁

`config/deployment.yaml` 分别记录传感器、飞控、PX4、外部视觉、地面测试和投放机构的完成状态。
门禁值只能在对应证据完成并由负责人确认后更新。`deployment_sim.yaml` 仅用于
硬件隔离仿真，不得复制到实机运行。

当前提交中 `deployment.yaml` 的勾选状态来自历史飞机验证记录，仅能说明对应设备曾完成过
上述检查。换机、拆装传感器、升级 PX4 或修改坐标转换后，必须重新核验并生成当前飞机的
证据。文件中的 `true` 状态不得作为直接开放输出的依据。

出现以下任一情况时，不得开放控制输出：

- 使用未确认的网络地址或串口设备。
- 外参仍是占位值，坐标方向未验证。
- PX4 外部视觉或 Offboard 保护未确认。
- 数据断流、时间戳异常或 MAVROS 消费者缺失。
- 投放机构未完成地面闭环检查。

后续验证须依据 [安全与验证](06-safety-and-validation.md) 逐级执行，禁止跨级。

[返回文档索引](README.md)
