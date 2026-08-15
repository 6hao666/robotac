# 示例 08：简短航点

## 目的

读取 YAML 中的相对航点，按顺序到达并保持，最后请求降落。

## 前置条件

- 示例 07 已通过。
- 航点文件只包含 `x`、`y`、`z`、`hold`。
- 每个航点均在本次场地和程序允许的移动范围内。
- 航点文件用于短距离教学，不涉及场地路线设计。

## 启动命令

```bash
# 启动简短航点示例并指定航点文件。
roslaunch robotac_examples 08_waypoints.launch \
  route_file:=$(rospack find robotac_examples)/config/waypoints.yaml
# 请求示例开始执行航点。
rosservice call /robotac_examples/waypoints/start "{}"
```

中止服务为 `/robotac_examples/waypoints/stop`。

## 预期输出

状态按 `WAYPOINT_1`、`WAYPOINT_2` 等顺序变化，全部完成后进入 `LANDING` 和 `COMPLETE`。
`~target` 可用于记录每个实际发送的目标。

## 失败判断

- YAML 字段缺失、增加未知字段或 `hold` 超限：节点启动失败。
- 目标越界或超时：任务进入 `ABORT` 并请求降落。
- 实际轨迹侵入障碍或边界：立即人工接管，不能依赖软件目标继续飞行。

## 下一步

航点示例只说明顺序执行。比赛路线和异常策略由参赛选手另行设计。视觉部分从
[示例 09](09-tag-centering-preview.md)继续。
