# 架构与接口

## bringup 入口

| launch | 作用 | 输出 |
| --- | --- | --- |
| `sensors.launch` | 启动 MID360 与 RGB 相机 | 只产生传感器数据 |
| `perception.launch` | 启动 FAST-LIO 与 AprilTag | 只产生定位和检测结果 |
| `flight_base.launch` | 启动 MAVROS 与外部视觉 PX4 输出 | 不解锁、不发布飞行设定点 |

底层 launch 包括 `lidar_mid360s.launch`、`camera_rgb.launch`、`fastlio_mid360s.launch`、
`apriltag_rgb.launch`、`camera_extrinsics.launch` 和 `mavros_px4.launch`。需要单独检查某个
组件时，可直接启动相应的底层 launch。

## 定位入口

| launch | 输入 | 输出 |
| --- | --- | --- |
| `vision_preview.launch` | `/sunray/odometry` | `/robotac_localization/vision_pose_preview` |
| `vision_to_px4.launch` | `/sunray/odometry` | 预览话题和 `/mavros/vision_pose/pose` |

两者使用同一实现。输入时间戳、四元数、速度跳变和空间范围检查通过后才发布。

## 编号示例

| 编号 | 节点脚本 | 主要输入 | 主要输出或动作 |
| --- | --- | --- | --- |
| 01 | `01_fcu_state.py` | FCU 与落地状态 | 只读日志 |
| 02 | `02_local_pose.py` | 本地位姿 | 位置、姿态、频率、数据年龄 |
| 03 | `03_apriltag_detection.py` | `/tag_detections` | Tag ID、相机坐标位置、检测时间 |
| 04 | `04_apriltag_local_pose.py` | Tag、TF、本地位姿 | Tag 本地位置和水平偏差 |
| 05 | `05_setpoint_preview.py` | 参数 | `~target`，不发往 MAVROS |
| 06 | `06_hover.py` | 飞控与定位状态 | 起飞、悬停、降落 |
| 07 | `07_move_relative.py` | 飞控与定位状态 | 相对位移、返回、降落 |
| 08 | `08_waypoints.py` | YAML 航点 | 顺序航点和降落 |
| 09 | `09_tag_centering_preview.py` | Tag 与本地位姿 | 对准 `~target`，不控制飞机 |
| 10 | `10_tag_centering_flight.py` | Tag、飞控与定位状态 | 对准、稳定 3 秒、降落 |
| 11 | `11_payload_release.py` | 舵机服务 | 单次阻挡或释放动作 |

## 飞行服务与公共输出

会控制飞机的示例提供：

| 接口 | 类型 | 含义 |
| --- | --- | --- |
| `~start` | `std_srvs/Trigger` | 完成启动前检查后接受一次启动请求 |
| `~stop` | `std_srvs/Trigger` | 中止当前执行并请求降落 |
| `~state` | `std_msgs/String` | 当前状态名称，不使用 `key=value` 字符串 |
| `~active` | `std_msgs/Bool` | 当前是否正在执行 |
| `~target` | `geometry_msgs/PoseStamped` | 当前或预览目标位姿 |

所有飞行示例只向 `/mavros/setpoint_position/local` 发布 `PoseStamped`。示例接口不使用原始
位置目标位掩码。

## Tag 接口

| 接口 | 类型 | 方向 |
| --- | --- | --- |
| `/robotac_examples/tag/pose` | `geometry_msgs/PoseStamped` | TagTracker 输出 |
| `/robotac_examples/tag/error` | `geometry_msgs/Vector3Stamped` | Tag 相对飞机的本地坐标偏差 |

TagTracker 只选择配置的 ID。连续检测结果跳变过大时，需要重新积累稳定样本。示例 10 在
飞行中长时间收不到 Tag 时会中止并请求降落。

## 舵机接口

| 接口 | 类型 | 含义 |
| --- | --- | --- |
| `/robotac_servo/set_released` | `std_srvs/SetBool` | `true` 释放，`false` 阻挡 |
| `/robotac_servo/connected` | `std_msgs/Bool` | 串口当前可访问 |
| `/robotac_servo/state` | `std_msgs/String` | `unknown`、`blocked`、`released` 或 `error` |
| `/robotac_servo/command_ok` | `std_msgs/Bool` | 最近一次命令是否成功 |

节点断线重连后不会重放上一条动作。

## 坐标系

- FAST-LIO 输入与输出坐标以实际配置和安装标定为准。
- MAVROS 本地位置按 ROS ENU 表示，PX4 内部转换由 MAVROS 处理。
- 相对位移示例的 `x` 为起始航向前方，`y` 为左方，`z` 为上方。
- Tag 本地位置依赖 `map` 到相机光学坐标系的有效 TF。

TF 名称、方向和数值必须分别验证。
