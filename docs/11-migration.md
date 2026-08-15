# 旧版迁移说明

本次重整删除完整比赛任务实现和旧接口，不提供兼容层。旧代码仍可从 Git 历史查阅，但不应
复制回当前教学工作空间继续使用。

## 包与目录

| 旧路径 | 当前替代 |
| --- | --- |
| `src/robotac_flight` | `robotac_localization` 与 `robotac_examples` |
| 顶层 `config/` | 配置随所属 ROS 包保存 |
| 根目录 `scripts/` | 按用途拆分的 `tools/` |

## launch

| 旧入口 | 当前替代 |
| --- | --- |
| `full_system.launch` | `sensors.launch`、`perception.launch`、`flight_base.launch` |
| `mapping_demo.launch` | 分别启动 `sensors.launch` 和 `perception.launch` |
| `fastlio_vision_bridge.launch`、`path_a_vision_pose.launch` | `vision_preview.launch` 或 `vision_to_px4.launch` |
| `local_flight_preflight.launch`、`local_waypoint_flight.launch` | 01、02、05、07、08 独立示例 |
| `tag_payload_mission.launch`、`tag_payload_mission_full.launch` | 03、04、09、10、11 独立示例 |
| `payload_drop_box_test.launch` | `11_payload_release.launch`，与飞行分开 |

完整任务 launch、历史比赛航线、观察器和证据分析器已经删除。

## ROS 接口

| 旧接口类别 | 当前接口 |
| --- | --- |
| `/robotac/flight/*` | 已删除；改用各飞行示例自己的 `~start`、`~stop`、`~state`、`~active`、`~target` |
| `/robotac/tag_payload_mission/*` | 已删除；分别接入 Tag、飞行和舵机接口 |
| 原始本地设定点接口 | `/mavros/setpoint_position/local` 的 `PoseStamped` |
| 旧 Tag 任务输出 | `/robotac_examples/tag/pose` 与 `/robotac_examples/tag/error` |
| 旧舵机布尔话题 | `/robotac_servo/set_released` 的 `std_srvs/SetBool` |

舵机服务中 `true` 表示释放，`false` 表示阻挡。状态话题为 `connected`、`state` 和
`command_ok`。

## 配置迁移

- MID360、FAST-LIO、相机、AprilTag、MAVROS、udev 和部署记录放在 `robotac_bringup`。
- 外部视觉参数放在 `robotac_localization`。
- 教学航点放在 `robotac_examples`。
- 舵机参数放在 `robotac_servo`。
- 历史比赛航线和完整任务参数不迁移。

## 行为变化

- 飞行示例启动 launch 后保持无动作，必须手动调用 `~start`。
- 每次节点只执行一次，不支持运行中复位或循环飞行。
- 位置控制只使用 `PoseStamped`。
- 飞行与舵机不自动串联。
- 定位预览和 PX4 输出使用两个明确 launch。

依赖旧接口的队伍代码应按功能拆分后重新接入。不要创建同名兼容服务，以免跳过当前的
启动检查。
