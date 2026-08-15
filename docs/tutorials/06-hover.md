# 示例 06：定高悬停

## 目的

从地面请求 OFFBOARD、解锁、定高悬停，并在保持结束后请求降落。

## 前置条件

- 已完成安全检查的前四个步骤。
- `sensors.launch`、`perception.launch` 和 `flight_base.launch` 正常。
- 飞机上锁、落地，场地封闭，安全操作员可接管。
- 高度和水平移动范围已经按本次场地复核。

## 启动命令

```bash
# 启动定高悬停示例；此时不会自动起飞。
roslaunch robotac_examples 06_hover.launch height:=0.6 hold:=3.0
```

launch 启动后应保持 `IDLE`。现场负责人批准后执行：

```bash
# 请求示例开始起飞和悬停。
rosservice call /robotac_examples/hover/start "{}"
```

中止：

```bash
# 请求示例中止并降落。
rosservice call /robotac_examples/hover/stop "{}"
```

## 预期输出

`~state` 依次进入 `PRESTREAM`、`TAKEOFF`、`HOVER`、`LANDING`、`COMPLETE`。`~active`
仅在任务执行期间为真。

## 失败判断

- `~start` 拒绝：按返回原因检查落地、位置、外部视觉、estimator、timesync 或订阅关系。
- 状态进入 `ABORT`：确认实际飞机是否执行降落，准备人工接管。
- 高度、姿态或水平漂移异常：立即中止或接管，不得修改阈值后直接重试。

## 下一步

悬停可重复验证后进入[示例 07](07-move-relative.md)。
