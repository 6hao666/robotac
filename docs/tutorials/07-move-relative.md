# 示例 07：相对位移

![PlantUML：相对位移状态机与中止降落路径](../assets/plantuml/tutorial-07-move-relative.svg "PlantUML：相对位移状态机")

## 本教程的作用

起飞后沿起始航向前方完成一次相对位移，返回起点上方并降落。

## 开始前确认

- 示例 06 已在同一飞机和场地通过。
- 定位链路已重新检查，当前原点、航向和各轴方向正确。
- 前向距离、高度和移动范围已复核。
- 航线及周围净空满足人工接管条件。

## 定位启动顺序

本示例以接受 `~start` 时的 PX4 本地位置和航向作为起点。目标中的“前方”由该航向计算，
定位漂移会直接形成位移和返航误差。分别在三个终端执行：

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
# 启动相对位移示例；此时不会自动起飞。
roslaunch robotac_examples 07_move_relative.launch height:=0.6 forward:=0.5
# 请求示例开始执行相对位移。
rosservice call /robotac_examples/move_relative/start "{}"
```

中止服务为 `/robotac_examples/move_relative/stop`。

## 正常结果

状态经过 `TAKEOFF`、`MOVE`、`RETURN`、`LANDING` 和 `COMPLETE`。目标位置基于节点接受
启动请求时的 `/mavros/local_position/pose` 和航向计算，不是直接使用 FAST-LIO 坐标。

## 需要停止并检查的情况

- 目标超出 `max_xy` 或 `max_z`：节点进入 `ABORT` 并请求降落。
- 运动方向与机头前方不一致：停止测试，检查航向和坐标系。
- 返回误差持续超限：检查定位漂移和控制响应，不进入航点示例。
- 本地位姿消息过期或跳变：中止测试，逐级检查里程计、外部视觉转发和 PX4 融合。

## 下一步

单次位移方向和返回稳定后进入[示例 08](08-waypoints.md)。
