# 架构与接口

## 文档范围

本章列出 Robotac 自有的 19 个 launch 入口和主要公开 ROS 接口。第三方包包含独立 launch，
不属于项目入口。所有实际名称来自当前源码；修改接口时必须同步更新契约检查和本文档。

## `robotac_bringup` 启动矩阵

| launch | 默认风险级别 | 用途 |
| --- | --- | --- |
| `apriltag_rgb.launch` | 被动 | 连接 RGB 图像和相机信息，启动 AprilTag 检测 |
| `camera_extrinsics.launch` | 配置敏感 | 发布相机静态外参；默认零值为占位 |
| `camera_rgb.launch` | 被动 | 启动 V4L2 RGB 相机及相机标定参数 |
| `fastlio_mid360s.launch` | 被动 | 运行 MID360 配置的 FAST-LIO，可选 RViz |
| `full_system.launch` | 被动默认 | 组合雷达、相机、FAST-LIO、Tag；MAVROS、视觉输出、控制器和舵机均需单独启用 |
| `lidar_mid360s.launch` | 被动 | 启动 Livox MID360 驱动 |
| `mapping_demo.launch` | 被动 | 组合雷达和 FAST-LIO，用于定位/建图观察 |
| `mavros_px4.launch` | 遥测 | 使用项目插件和参数启动 MAVROS；打开 FCU 链路但不自动飞行 |
| `payload_drop_box_test.launch` | 预览默认 | 组合通用航点投放测试；`live_flight` 默认关闭 |
| `tag_payload_mission_full.launch` | 预览默认 | 组合完整 C 组任务栈；`live_flight` 默认关闭，外参发布也默认关闭 |

`full_system.launch` 是通用组合入口，不构成自动飞行指令。其重要开关均默认为 `false`：
`enable_mavros`、`enable_vision_bridge`、`vision_enable_output`、
`enable_flight_controller`、`flight_enable_control`、`flight_auto_mode`、
`flight_auto_arm`、`flight_auto_land`、`flight_enable_payload` 和 `enable_servo`。

## `robotac_flight` 启动矩阵

| launch | 用途 | 是否产生实机输出 |
| --- | --- | --- |
| `active_flight_observer.launch` | 采集受控实飞的航线、设定点、视觉和飞控证据 | 仅订阅 |
| `ev_acceptance_observer.launch` | 比较外部视觉与 MAVROS 本地位置的方向、尺度和状态 | 仅订阅 |
| `fastlio_frame_alignment_observer.launch` | 在输出关闭时检查 FAST-LIO 机体轴和手动运动方向 | 仅订阅 |
| `fastlio_vision_bridge.launch` | 将 `/Odometry` 转为带协方差的外部视觉候选 | 默认只预览 |
| `local_flight_preflight.launch` | 检查 MAVROS、估计器、视觉、时间同步、帧和消费者 | 仅订阅并读取参数 |
| `local_waypoint_flight.launch` | 通用本地相对航点状态机 | `enable_control` 默认关闭 |
| `path_a_vision_pose.launch` | 将 `/sunray/odometry` 转为 `PoseStamped` 外部视觉候选 | `enable_mavros_output` 默认关闭 |
| `tag_payload_mission.launch` | C 组 Tag 投放任务状态机 | 控制、自动模式、解锁、降落、投放均默认关闭 |

两条外部视觉路径用途不同：

- Path A：`/sunray/odometry` -> `PoseStamped`，正式候选输出为 `/mavros/vision_pose/pose`。
- 协方差桥：`/Odometry` -> `PoseWithCovarianceStamped`，默认候选输出为
  `/mavros/vision_pose/pose_cov`，主要用于比较或回退。

两条路径不得同时作为未经核验的飞控输入。

## `robotac_servo` 启动矩阵

| launch | 用途 | 默认行为 |
| --- | --- | --- |
| `servo.launch` | 打开 USB PWM 控制器并提供布尔开关接口 | 115200 波特率、启动关闭、退出关闭 |

## 核心节点

| 节点/脚本 | 输入 | 输出或职责 |
| --- | --- | --- |
| `local_odom_to_vision_pose.py` | `/sunray/odometry` | 位姿预览、健康状态、可选 MAVROS 视觉输出 |
| `fastlio_vision_bridge.py` | `/Odometry` | 带协方差预览、健康状态、可选 MAVROS 视觉输出 |
| `local_flight_preflight.py` | MAVROS、FAST-LIO、视觉状态 | 飞行前只读验收结果与证据文件 |
| `local_waypoint_flight.py` | 本地位置、视觉、飞控、航点、舵机状态 | 航点预览、状态和受门禁控制的设定点 |
| `tag_payload_mission.py` | 本地位置、飞控、Tag、舵机状态 | 竞赛任务预览、状态和受门禁控制的设定点 |
| `servo_node.py` | 布尔开关 | 串口 PWM 帧和写入状态 |

## C 组任务服务

以下服务类型均为 `std_srvs/Trigger`：

| 服务 | 作用 | 前置条件 |
| --- | --- | --- |
| `/robotac/tag_payload_mission/start` | 在当前位置和航向建立任务帧并申请启动 | 状态为空闲、全部实时检查通过、现场授权 |
| `/robotac/tag_payload_mission/abort` | 请求任务中止 | 操作员持续监督，并按现场预案接管 |
| `/robotac/tag_payload_mission/reset` | 在安全终态后恢复为空闲 | 已停止任务、飞机状态适合复位 |

`start` 服务可调用不代表启动条件成立。节点将拒绝缺少控制开关、飞控状态、位置、消费者、
舵机回执或部署门禁的请求。服务调用应由现场操作流程或受审计的上位机完成，禁止将调用
命令设计为跳过检查的快捷步骤。

## 通用航点服务

以下服务类型均为 `std_srvs/Trigger`：

| 服务 | 作用 |
| --- | --- |
| `/robotac/flight/start` | 建立本地任务帧并启动已加载航线 |
| `/robotac/flight/abort` | 中止当前航点任务 |
| `/robotac/flight/land` | 请求进入已配置的降落流程 |
| `/robotac/flight/reset` | 从终态复位到空闲 |

## C 组任务话题

| 话题 | 类型 | 方向 | 含义 |
| --- | --- | --- | --- |
| `/robotac/tag_payload_mission/status` | `std_msgs/String` | 节点 -> 观察者 | 状态机状态、原因和检查信息 |
| `/robotac/tag_payload_mission/active` | `std_msgs/Bool` | 节点 -> 观察者 | 任务是否活动，锁存 |
| `/robotac/tag_payload_mission/route_manifest` | `std_msgs/String` | 节点 -> 观察者 | 当前配置航线清单，锁存 |
| `/robotac/tag_payload_mission/confirmed_tag_pose` | `geometry_msgs/PoseStamped` | 节点 -> 观察者 | 稳定确认并变换后的 Tag 位姿 |
| `/robotac/tag_payload_mission/setpoint_preview` | `mavros_msgs/PositionTarget` | 节点 -> 观察者 | 目标预览，不是 MAVROS 控制话题 |

## 通用航点话题

| 话题 | 类型 | 方向 | 含义 |
| --- | --- | --- | --- |
| `/robotac/flight/status` | `std_msgs/String` | 节点 -> 观察者 | 状态和门禁原因 |
| `/robotac/flight/active` | `std_msgs/Bool` | 节点 -> 观察者 | 航点任务是否活动，锁存 |
| `/robotac/flight/route_manifest` | `std_msgs/String` | 节点 -> 观察者 | 实际加载的航线清单，锁存 |
| `/robotac/flight/setpoint_preview` | `mavros_msgs/PositionTarget` | 节点 -> 观察者 | 目标预览 |
| `/robotac/flight/waypoints` | `geometry_msgs/PoseArray` | 路线发布者 -> 节点 | 运行时位置与航向航点 |

`PoseArray` 仅承载位置与航向。停留时间、投放动作等任务元数据应写入 YAML 路线文件。

## 定位与视觉话题

| 话题 | 类型 | 作用 |
| --- | --- | --- |
| `/livox/lidar` | Livox 自定义点云 | MID360 点云输入 |
| `/livox/imu` | `sensor_msgs/Imu` | MID360 IMU 输入 |
| `/Odometry` | `nav_msgs/Odometry` | FAST-LIO 里程计 |
| `/mavros/vision_pose/pose` | `geometry_msgs/PoseStamped` | Path A 实际外部视觉输出 |
| `/robotac/fastlio_vision/path_a_pose_preview` | `geometry_msgs/PoseStamped` | Path A 预览 |
| `/robotac/fastlio_vision/healthy` | `std_msgs/Bool` | 位姿输入是否满足健康阈值 |
| `/robotac/fastlio_vision/status` | `std_msgs/String` | 健康、帧、频率和拒绝原因 |
| `/robotac/fastlio_vision/output_enabled` | `std_msgs/Bool` | 是否实际向 MAVROS 输出，锁存 |
| `/mavros/local_position/odom` | `nav_msgs/Odometry` | PX4/MAVROS 本地位置反馈 |
| `/tag_detections` | `apriltag_ros/AprilTagDetectionArray` | AprilTag 检测结果 |

## 舵机话题

| 话题 | 类型 | 含义 |
| --- | --- | --- |
| `/robotac_servo/control` | `std_msgs/Bool` | `false` 为关舱角，`true` 为配置的开舱角 |
| `/robotac_servo/status` | `std_msgs/String` | 串口写入状态，不是机械角度测量 |

## 预览与实机输出

| 功能 | 预览/观察 | 实机输出 |
| --- | --- | --- |
| 外部视觉 | `/robotac/fastlio_vision/path_a_pose_preview` | `/mavros/vision_pose/pose` |
| 通用航点 | `/robotac/flight/setpoint_preview` | `/mavros/setpoint_raw/local` |
| C 组任务 | `/robotac/tag_payload_mission/setpoint_preview` | `/mavros/setpoint_raw/local` |
| 投放 | 状态清单和仿真记录 | `/robotac_servo/control` |

存在预览数据不代表对应实机输出已经启用；`output_enabled=true` 亦不足以证明 PX4
已接受并正确融合数据。验收必须同时检查消费者、时间同步、估计器和本地位置响应。

## 坐标系

- FAST-LIO：`camera_init -> body`。
- 外部视觉候选：通常为 `odom -> base_link`。
- MAVROS 本地位置：通常为 `map -> base_link`。
- 通用航点任务：`robotac_start_body`，`x` 向前、`y` 向左、`z` 向上。
- C 组任务：启动时建立任务帧，`x` 向机头、`y` 向机体右侧、`z` 向上。

上述两类任务帧的横向正方向不同，航线文件不得混用。任何帧转换都必须由测量和运动方向
检查证明，禁止仅凭名称添加静态 TF。

[返回文档索引](README.md)
