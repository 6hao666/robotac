# 示例 02：定位链路与本地位姿

![PlantUML：定位链路与本地位姿只读检查](../assets/plantuml/tutorial-02-local-pose.svg "PlantUML：定位链路与本地位姿只读检查")

## 本教程的作用

只读检查修正后的激光里程计、外部视觉转发、PX4 位置状态和 MAVROS 本地位姿。本示例不发布
位置目标，也不调用飞控服务。

飞行示例不直接读取 FAST-LIO。位置依次经过 `/sunray/odometry`、
`/mavros/vision_pose/pose` 和 PX4 estimator，最后以 `/mavros/local_position/pose` 提供给
示例。因此 FAST-LIO 有输出只是第一步，四级状态都正常后才具备位置控制所需的数据来源。

## 开始前确认

- 示例 01 已确认 FCU 连接正常。
- `sensors.launch`、`perception.launch` 和 `flight_base.launch` 已按顺序启动。
- FAST-LIO 初始化期间飞机保持静止，`/sunray/odometry` 已稳定输出。
- 飞机保持上锁并固定。

## 启动命令

```bash
# 启动定位链路与本地位姿只读示例。
roslaunch robotac_examples 02_local_pose.launch
```

## 正常结果

终端每秒显示以下四项状态：

- 修正里程计：最近收到 `/sunray/odometry`；
- 外部视觉转发：`/vision_pose_bridge/healthy` 最近更新且为 `true`；
- PX4 位置状态：estimator 的姿态、水平相对位置和垂直位置有效；
- 本地位姿：最近收到 `/mavros/local_position/pose`。

收到本地位姿后，还会显示 `x`、`y`、`z`、姿态、接收频率和消息年龄。四项显示正常并不
代替人工核对；静止时应无明显漂移或跳变，手动平移和转动时方向应符合本机标定。

## 需要停止并检查的情况

- 修正里程计未就绪：检查 MID360、FAST-LIO 和静止初始化。
- 外部视觉转发未就绪：读取 `/vision_pose_bridge/state` 判断时间戳、数值或数据中断问题。
- PX4 位置状态未就绪：检查实际固件版本对应的外部视觉融合参数和 estimator 状态。
- 本地位姿未就绪：PX4 尚未产生可由 MAVROS 输出的本地位置。
- 数据年龄持续增加：上游时间戳或通信异常。
- 静止漂移、跳变或轴向相反：不得进入飞行示例，先检查定位和坐标系。

## 下一步

本地位姿稳定后进入[示例 03](03-apriltag-detection.md)。
