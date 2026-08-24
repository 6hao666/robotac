# 部署与运行

![PlantUML：从开发机同步到运行记录的部署时序](./assets/plantuml/deployment-operations.svg "PlantUML：部署与运行时序")

## 同步

从开发机同步到机载 Ubuntu 主机：

```bash
# 将当前 Robotac 仓库同步到机载主机的 /home/<用户>/robotac。
./tools/sync_workspace.sh <用户>@<飞机地址>:/home/<用户>/robotac
```

同步脚本排除 `.git`、`build`、`devel`、`install` 和 `log`，也不会删除远端已有的其他文件。
同步前应确认目标目录，避免覆盖其他队伍的文件或系统服务使用的配置。

需要用本地仓库完整替换已确认的机载工作空间时，先预演再执行镜像同步：

```bash
# 预览文件更新、旧源码删除、构建目录清理和环境文件初始化。
./tools/sync_workspace.sh --mirror --dry-run \
  <用户>@<飞机地址>:/home/<用户>/robotac
# 执行完整镜像；该操作会同步 .git 并删除远端多余源码。
./tools/sync_workspace.sh --mirror \
  <用户>@<飞机地址>:/home/<用户>/robotac
```

镜像模式不会上传或删除已有的 `.env`、`.env.local`。仓库只同步 `.env.template`；使用
`--init-env` 或 `--mirror` 时，仅在同目录 `.env` 不存在的情况下从模板初始化，并设置为
`0600`。初始化后的占位值必须在目标主机上核对，不得将真实凭据回传仓库。

## 机载构建

```bash
# 进入机载 Robotac 仓库根目录。
cd /home/<用户>/robotac
# 检查源码和文档链接。
./tools/test_01_source.sh
# 执行比赛运行构建并检查包发现情况。
./tools/test_02_build.sh
# 加载构建后的 ROS 环境。
source devel/setup.bash
# 运行纯单元测试。
./tools/test_03_unit.sh
# 运行简化仿真。
./tools/test_04_simulation.sh
```

构建和仿真阶段不连接设备，不启动真实硬件 launch。
比赛运行构建使用系统 MAVROS 和 AprilTag ROS。只有验证第三方源码时才执行
`./tools/build_full.sh`；其环境位于 `devel_full/setup.bash`，不得替代日常比赛环境。

## 单独验证传感器

首次配置或排障时，先分别启动需要检查的组件。雷达终端执行：

```bash
# 发现并比较现场地址；需要时按提示确认写入配置。
./tools/sensor_setup.py lidar --interface <雷达网卡>
# 只启动 Livox 驱动，不启动相机或 FAST-LIO。
roslaunch robotac_bringup lidar_mid360s.launch
```

在另一个已加载工作空间环境的终端检查：

```bash
# 确认 FAST-LIO 所需的自定义点云类型。
rostopic type /livox/lidar
# 观察点云接收频率。
rostopic hz /livox/lidar
# 观察 IMU 接收频率。
rostopic hz /livox/imu
# 读取一帧 IMU，检查时间戳和 frame。
rostopic echo -n 1 /livox/imu
```

`/livox/lidar` 必须为 `livox_ros_driver2/CustomMsg`，`/livox/imu` 必须为
`sensor_msgs/Imu`。两者稳定后才能启动 FAST-LIO。

相机终端执行：

```bash
# 确认采集节点、目标模式和稳定别名。
./tools/sensor_setup.py camera
# 只启动 RGB 相机。
roslaunch robotac_bringup camera_rgb.launch
```

另一个终端检查 `/camera/rgb/image_raw` 和 `/camera/rgb/camera_info` 的类型、频率、
`frame_id`、宽高以及标定尺寸。不要把相机 metadata 节点传给 `video_device`。

## 建立位置来源

`06`、`07`、`08` 和 `10` 需要 PX4 提供连续的本地位置。三个基础 launch 互不包含，飞行
示例也不会自动启动它们。以下步骤分别在已加载工作空间环境的终端执行。

### 1. 启动传感器

```bash
# 启动 MID360 和 RGB 相机，只产生传感器数据。
roslaunch robotac_bringup sensors.launch
```

在另一个终端确认 FAST-LIO 的两项输入：

```bash
# 观察 MID360 点云频率，不发布任何消息。
rostopic hz /livox/lidar
# 观察 MID360 内置 IMU 频率，不发布任何消息。
rostopic hz /livox/imu
```

点云和 IMU 都应连续输出。仅有点云时不能继续启动 FAST-LIO。

### 2. 启动 FAST-LIO 和里程计修正

将飞机放在水平、稳固的位置并保持上锁。启动后的前 200 帧 IMU 用于计算初始倾角，随后
第一帧 FAST-LIO 里程计会成为本次启动的原点；初始化期间不要移动或触碰飞机。

```bash
# 启动 FAST-LIO、里程计修正和 AprilTag，不向 PX4 发送位姿。
roslaunch robotac_bringup perception.launch
```

待输出稳定后检查 FAST-LIO 原始里程计和修正后的里程计：

```bash
# 观察 FAST-LIO 原始里程计频率。
rostopic hz /Odometry
# 读取一帧原始里程计，检查时间戳、frame 和位姿。
rostopic echo -n 1 /Odometry
# 观察修正后的里程计频率。
rostopic hz /sunray/odometry
# 读取一帧修正后的里程计，检查时间戳、frame 和位姿。
rostopic echo -n 1 /sunray/odometry
```

静止时数据应连续且没有明显跳变。拆除桨叶后，沿机体前、左、上方向缓慢移动飞机，并缓慢
改变航向，核对 `/sunray/odometry` 的位置和姿态变化。方向错误或漂移明显时不要进入下一步。

### 3. 启动 MAVROS 并将位姿送入 PX4

```bash
# 启动 MAVROS 和外部视觉转发，不解锁飞机，也不发送位置目标。
roslaunch robotac_bringup flight_base.launch
```

`flight_base.launch` 将 `/sunray/odometry` 检查后发布到 `/mavros/vision_pose/pose`。PX4
是否采用该数据，取决于目标飞控的固件版本和 estimator 参数。

### 4. 检查 PX4 融合结果

```bash
# 确认外部视觉位姿持续送到 MAVROS。
rostopic hz /mavros/vision_pose/pose
# 读取外部视觉桥接状态。
rostopic echo -n 1 /vision_pose_bridge/state
# 读取外部视觉桥接是否正常。
rostopic echo -n 1 /vision_pose_bridge/healthy
# 读取 PX4 estimator 状态。
rostopic echo -n 1 /mavros/estimator_status
# 观察 PX4 融合后的 MAVROS 本地位姿频率。
rostopic hz /mavros/local_position/pose
# 读取一帧最终本地位姿，检查位置、姿态和时间戳。
rostopic echo -n 1 /mavros/local_position/pose
```

`/mavros/vision_pose/pose` 有数据不表示 PX4 已经完成融合。必须同时确认 estimator 的姿态、
水平相对位置和垂直位置状态有效，并再次完成静止漂移、手动平移、轴向和航向核对。

### 5. 运行定位链路观察器

```bash
# 同时观察修正里程计、外部视觉转发、PX4 位置状态和最终本地位姿。
roslaunch robotac_examples 02_local_pose.launch
```

四项状态都正常，且最终本地位姿的方向、连续性和漂移满足现场要求后，才进入位置控制示例。
需要读取全部真实设备和 ROS 话题时，可以执行：

```bash
# 读取真实设备和 ROS 数据，不发送控制命令。
./tools/test_05_hardware_readonly.sh
```

以上检查均为只读操作，不得在该阶段调用飞控模式或解锁服务。排障时只启动对应组件，不要在
多个终端重复启动同一设备节点。

## 示例运行

以悬停示例为例：

```bash
# 启动定高悬停示例；此时不会自动起飞。
roslaunch robotac_examples 06_hover.launch
```

launch 启动后保持 `IDLE`。现场检查完成并获得开始指令后，另行调用：

```bash
# 请求悬停示例开始执行。
rosservice call /robotac_examples/hover/start "{}"
```

需要中止时：

```bash
# 请求当前示例中止并降落。
rosservice call /robotac_examples/hover/stop "{}"
```

每次节点只执行一次。结束后重新启动 launch 才能进行下一次测试。

## 运行记录

每次会控制飞机或投放机构的测试至少记录：

- Git 提交 SHA 和使用的配置文件；
- 飞机编号、测试日期、场地和安全负责人；
- launch 参数、`~state`、`~active` 和 `~target`；
- FCU 模式、解锁、落地、estimator 和 timesync 状态；
- 中止、人工接管和异常原因；
- 本次结果，以及下一次准备增加的测试内容。

公开材料应删除机器地址、个人路径和凭据。

## 比赛前检查

- 代码与配置和预审提交版本一致，变更已有记录。
- 指定飞机、投放机构、模拟货物和电池通过检录。
- 传感器、外部视觉和本地位置稳定。
- Tag ID、尺寸、贴纸方向和相机外参已复核。
- 障碍路线、边界、计时和中止条件使用当前规则。
- 遥控器接管和急停方式已经确认，安全操作员已经确定。
- 两次飞行共用计时的操作流程已经演练。

## 结束检查

1. 确认飞机落地、上锁、停桨。
2. 断开动力电池，再处理 USB 和载荷。
3. 正常结束示例、bringup 和记录进程。
4. 用 `rosnode list` 核对没有重复或残留节点。
5. 保存日志和配置快照，记录异常。

不得依据旧进程编号直接终止进程，也不得用宽泛进程名清理无关 ROS 节点。
