# 部署与运行

![PlantUML：从开发机同步到运行记录的部署时序](./assets/plantuml/deployment-operations.svg "PlantUML：部署与运行时序")

## 同步

从开发机同步到机载 Ubuntu 主机：

```bash
# 将工作空间同步到机载 Ubuntu 主机。
./tools/sync_workspace.sh <用户>@<飞机地址>:<工作空间目录>
```

同步脚本排除 `.git`、`build`、`devel`、`install` 和 `log`，也不会删除远端已有的其他文件。
同步前应确认目标目录，避免覆盖其他队伍的文件或系统服务使用的配置。

## 机载构建

```bash
# 进入机载工作空间。
cd <工作空间目录>
# 检查源码和文档链接。
./tools/test_01_source.sh
# 构建项目并检查包发现情况。
./tools/test_02_build.sh
# 加载构建后的 ROS 环境。
source devel/setup.bash
# 运行纯单元测试。
./tools/test_03_unit.sh
# 运行简化仿真。
./tools/test_04_simulation.sh
```

构建和仿真阶段不连接设备，不启动真实硬件 launch。

## 启动基础组件

分别在已加载工作空间环境的终端启动：

```bash
# 启动 MID360 和 RGB 相机。
roslaunch robotac_bringup sensors.launch
# 启动 FAST-LIO 和 AprilTag。
roslaunch robotac_bringup perception.launch
# 启动 MAVROS 和外部视觉输出。
roslaunch robotac_bringup flight_base.launch
```

排障时只启动对应的低层 launch。不要用多个终端重复启动同一设备节点。

## 只读检查

基础组件全部启动后执行：

```bash
# 读取真实设备和 ROS 数据，不发送控制命令。
./tools/test_05_hardware_readonly.sh
```

随后人工检查：

```bash
# 查看 FAST-LIO 里程计频率。
rostopic hz /sunray/odometry
# 查看 MAVROS 本地位姿频率。
rostopic hz /mavros/local_position/pose
# 读取外部视觉桥接状态。
rostopic echo -n 1 /vision_pose_bridge/state
# 读取飞控连接状态。
rostopic echo -n 1 /mavros/state
```

只读检查不得调用飞控服务。

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
