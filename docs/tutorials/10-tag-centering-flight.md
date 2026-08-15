# 示例 10：Tag 对准飞行

## 目的

从地面起飞，对准 ID 0，在默认 3 秒稳定时间后请求降落。

## 前置条件

- 示例 06 和 09 均已通过。
- 起飞位置能够稳定检测目标，TF 和相机外参已经现场确认。
- `max_step`、高度、总超时和移动范围采用保守值。
- 场地封闭，安全操作员可随时接管。

## 启动命令

```bash
# 启动 Tag 对准飞行示例；此时不会自动起飞。
roslaunch robotac_examples 10_tag_centering_flight.launch \
  tag_id:=0 height:=0.6 hold:=3.0
# 请求示例开始起飞和 Tag 对准。
rosservice call /robotac_examples/tag_centering_flight/start "{}"
```

中止服务为 `/robotac_examples/tag_centering_flight/stop`。

## 预期输出

启动时必须已有稳定 Tag。状态经过 `PRESTREAM`、`TAKEOFF`、`TAG_CENTERING`、`LANDING`
和 `COMPLETE`。对准期间 `~target` 按 `max_step` 限制逐步修正。

## 失败判断

- 启动时无稳定 Tag：拒绝继续或进入中止降落。
- 飞行中长时间收不到 Tag：停止更新目标并请求降落。
- 修正震荡、方向错误或接近边界：立即调用 `~stop` 或人工接管。
- 进入 `COMPLETE` 只表示示例流程结束，不表示比赛 C3 已经得分。

## 下一步

保存识别状态、目标位置或中心偏差记录。后续任务流程由参赛选手根据规则安排。投放机构
单独按[示例 11](11-payload-release.md)检查。
