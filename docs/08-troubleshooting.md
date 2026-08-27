# 故障排查

![PlantUML：Robotac 分层故障诊断决策树](./assets/plantuml/troubleshooting.svg "PlantUML：分层故障诊断决策树")

## 串口权限或设备名变化

现象：PX4 或舵机节点无法打开串口设备。

1. 使用 `ls -l <串口设备>` 检查设备是否存在。
2. 使用 `udevadm info --query=all --name=<串口设备>` 核对设备属性。
3. 检查当前用户所属设备访问组和 udev 规则。
4. 确认没有其他进程占用串口。

不得通过长期使用 root 运行整套工作空间解决权限问题。

## MID360 无数据

先执行 `./tools/sensor_setup.py lidar --interface <雷达网卡>`，再按结果处理：

- 网口没有物理链路：检查供电、线缆、交换机和所选接口；
- 网口有链路但没有 IPv4：按提示临时配置后发现，确认无地址冲突再考虑持久化；
- 多个接口或多个雷达：使用 `--interface` 缩小范围，按序列号核对目标设备；
- SDK 没有发现设备：检查同一二层网络、UDP 56000、防火墙和是否已有驱动占用端口；
- 已发现但配置不一致：停止驱动后重新运行工具，经确认写入正确字段；
- SDK 或发现器缺失：重新执行依赖安装和比赛运行构建。

检查 `mid360s.json` 中的主机和雷达地址、网卡状态、专用子网、驱动日志以及 Livox 数据端口。
公开排障记录应使用 `<机载网卡地址>` 和 `<雷达地址>`，不记录实际值。ping 或 ARP 扫描结果
不能证明目标是 Livox 雷达。

只有点云没有 IMU 时，FAST-LIO 仍不能正常工作。分别检查点云和 IMU 频率、时间戳和 frame。

## MAVROS 未连接

检查：

- `<飞控设备>` 是否存在且未被占用；
- `fcu_url` 的设备名和波特率；
- PX4 是否正常启动；
- MAVROS 心跳超时日志；
- udev 稳定别名是否匹配目标飞控。

MAVROS 显示连接成功，只表示已经收到飞控心跳。还需要单独检查 estimator 和 OFFBOARD
所需的状态。

## FAST-LIO 无里程计

先区分哪个话题没有数据：

1. Livox 点云与 IMU 均有数据；
2. 话题名和消息类型符合 `mid360s.yaml`；
3. 雷达类型、外参和时间同步配置正确；
4. FAST-LIO 日志没有初始化失败或时间回退；
5. FAST-LIO 原始 `/Odometry` 的时间戳、姿态和频率连续。

`/Odometry` 有数据而 `/sunray/odometry` 没有数据时，检查 `/livox/imu` 是否持续输入、
`transform_odom_pointCloud` 是否运行，以及启动后的 200 帧 IMU 初始化是否完成。重新初始化时
飞机必须上锁并保持静止。

## 修正里程计异常

`/sunray/odometry` 以第一帧 `/Odometry` 为原点，并使用启动时采集的 IMU 平均值修正倾角。
依次检查：

1. 时间戳持续递增，消息没有间歇停止；
2. 四元数有效，静止时位置和姿态没有明显跳变；
3. 向机体前、左、上方向移动时，位置增量方向符合约定；
4. 改变航向时，姿态变化方向正确；
5. 初始化期间飞机没有移动或受到振动。

frame 名正确不代表位姿数值已经转换到该坐标系。发现方向错误时应检查外参和修正计算，不能
只修改 `frame_id`。

## 外部视觉被拒绝

读取：

```bash
# 读取外部视觉桥接的当前状态。
rostopic echo -n 1 /vision_pose_bridge/state
# 读取外部视觉是否健康。
rostopic echo -n 1 /vision_pose_bridge/healthy
```

状态含义：

| 状态 | 检查项 |
| --- | --- |
| `WAITING` | 尚未收到里程计 |
| `STAMP` | 时间戳为空、过旧或超前 |
| `FRAME` | 输入 frame 与配置要求不一致 |
| `NONFINITE` | 位姿含 NaN 或无穷值 |
| `QUATERNION` | 四元数未归一化 |
| `RADIUS` | 位置超出配置范围 |
| `ORDER` | 时间戳未递增 |
| `SPEED` | 相邻位姿推算速度过大 |
| `TIMEOUT` | 输入数据中断 |

先修正数据源或标定，不应扩大阈值掩盖问题。

## PX4 未融合外部视觉

先确认 `/mavros/vision_pose/pose` 持续有数据，再读取 `/mavros/estimator_status`。没有外部
视觉输入时检查桥接状态；有输入但水平或垂直位置状态无效时，检查实际 PX4 固件版本及对应的
外部视觉融合参数。

新版 PX4 通常使用 `EKF2_EV_CTRL`，旧版固件可能使用 `EKF2_AID_MASK`、`EKF2_HGT_MODE`
等参数。不要把其他版本的参数值直接写入当前飞控。还应检查外部视觉时间戳、坐标方向、
高度来源和 estimator 创新量。

## MAVROS 本地位置异常

飞行示例读取 `/mavros/local_position/pose`，而不是直接读取 FAST-LIO 里程计。该话题无数据、
过期、跳变或方向错误时，按以下顺序检查：

1. `/sunray/odometry` 是否正常；
2. `/vision_pose_bridge/healthy` 是否为 `true`；
3. `/mavros/vision_pose/pose` 是否持续发布；
4. `/mavros/estimator_status` 的位置状态是否有效；
5. PX4 与 MAVROS 的时间同步是否稳定。

上游里程计有数据而本地位置无效，通常说明问题仍在外部视觉转发、PX4 融合或 MAVROS 输出
阶段。不要通过放宽飞行示例的数据时限绕过该问题。

## TF 缺失或方向错误

AprilTag 本地坐标需要 `map` 到相机光学坐标系的完整 TF。检查 frame 名、父子关系、时间戳
和外参数值。零外参只可用于明确的测试模型，不得直接用于真实飞机。

## AprilTag 无检测

检查：

- 图像与 `camera_info` 的分辨率和时间戳；
- Tag 家族是否为 `tag36h11`；
- 实体是否为 ID 0，黑色编码区域边长是否按 `0.15 m` 配置；
- 曝光、焦距、运动模糊、打印质量和遮挡；
- `/tag_detections` 是否有数据。

有检测但位置跳变时，检查相机内参、Tag 尺寸和光学坐标方向。

## RGB 相机无图像

执行 `./tools/sensor_setup.py camera` 并检查：

- 选择的节点必须具有 `Video Capture`，不能是同一 USB 设备的 metadata 节点；
- 设备必须支持 launch 使用的 `MJPEG 1920x1080@30`；
- `/dev/robotac_rgb_camera` 必须指向识别出的采集节点；
- `fuser <相机设备>` 不应显示另一个采集进程占用设备；
- `camera_info` 的宽高必须与采集分辨率和当前标定文件一致。

稳定别名缺失时，先核对工具输出的 USB ID，再运行 `camera --install-udev`。不得为了绕过
权限长期以 root 运行相机节点。

## 飞行示例拒绝启动

`~start` 会说明拒绝启动的原因。常见原因包括未落地、位置过期、外部视觉异常、estimator
无效、timesync 延迟过大或 MAVROS 没有订阅位置设定点。

逐项排除问题后再调用。启动失败不会跳过后续检查；节点已经执行过一次时必须重新启动。

## 运行中进入 `ABORT`

读取示例日志、`~state` 和 FCU 状态。确认是否由 `~stop`、数据断流、模式丢失、Tag 超时或
目标越界触发。软件已经请求降落时仍须观察实际飞机并准备人工接管。

## 舵机无响应

检查 `<舵机设备>`、供电、串口占用、波特率、控制器协议、软件限位和机构卡滞。状态话题只能
确认驱动程序的状态，不能确认货物是否已经脱离。禁止连续调用服务，强行驱动已经卡住的机构。

标定流程见[舵机投放机构标定](10-servo-release-calibration.md)。
