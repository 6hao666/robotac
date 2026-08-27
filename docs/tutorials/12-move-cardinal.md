# 示例 12：前后左右水平移动排查

## 本教程的作用

起飞到固定高度后，按**机头朝向**依次完成前、后、左、右四个方向的水平位移（相对起点），
每个方向到达后打印期望位置与实际位置的偏差，全部走完返回起点并降落。用于排查水平移动
方向是否正确、各轴响应与坐标转换问题（如：期望向北，飞机实际向东/向西等）。

## 开始前确认

- 示例 07 已在同一飞机和场地通过，机头航向定义与 `relative_point` 约定一致。
- 定位链路已重新检查，当前原点、航向和各轴方向正确；`map` 系与场地系的旋转标定
  （`frames.field_yaw`）已核对。
- 前后左右四个方向 `step` 距离（默认 0.5 m）均在场内、无碰撞风险，满足人工接管条件。

## 定位启动顺序

本示例以接受 `~start` 时的 PX4 本地位置和航向作为起点，目标由该航向计算。分别执行：

```bash
# 启动 MID360 和 RGB 相机，只产生传感器数据。
roslaunch robotac_bringup sensors.launch
# 保持飞机静止，启动 FAST-LIO 和里程计修正。
roslaunch robotac_bringup perception.launch
# 启动 MAVROS，并将修正后的里程计送入 PX4。
roslaunch robotac_bringup flight_base.launch
```

```bash
# 只读检查完整定位链路和最终本地位姿。
roslaunch robotac_examples 02_local_pose.launch
```

四项状态均正常后，拆除桨叶检查前、左、上方向和航向变化。出现漂移、跳变、方向相反或消息
过期时不得继续。本示例 launch 不会启动 FAST-LIO、MAVROS 或外部视觉转发。

## 启动命令

```bash
# 启动水平移动排查示例；此时不会自动起飞。
roslaunch robotac_examples 12_move_cardinal.launch height:=0.3 step:=0.5
# 请求示例开始执行。
rosservice call /robotac_examples/move_cardinal/start "{}"
```

中止服务为 `/robotac_examples/move_cardinal/stop`。

## 正常结果

起飞后依次向 前 → 后 → 左 → 右 各移动 `step` 距离，`~state` 依次经过
`MOVE_前`、`MOVE_后`、`MOVE_左`、`MOVE_右`、`RETURN`，每个方向打印
「期望 vs 实际 偏差」，最后返回起点并降落。各方向实际位置应与期望偏差在定位/控制裕量内。

## 需要停止并检查的情况

- 某方向移动方向与机头标称方向不一致：停止测试，检查航向与坐标系（`field_yaw` 标定）。
- 某方向移动超时或无位移：检查该轴响应、setpoint 发布与定位跟踪。
- 本地位姿消息过期或跳变：中止测试，逐级检查里程计、外部视觉转发和 PX4 融合。

## 下一步

四个方向移动与返回均稳定后，进入完整[任务状态机](../robotac_mission/README.md)。
