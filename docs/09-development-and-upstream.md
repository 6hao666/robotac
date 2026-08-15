# 开发与上游

## 修改范围

参赛开发优先修改：

- `src/robotac_examples` 中队伍自行新增的示例或算法；
- 各项目自有包的配置；
- `docs` 中与本队程序对应的运行说明；
- 独立的新 ROS 包。

不建议直接修改第三方目录。确需修改时，应单独记录上游版本、修改原因和差异。

## 代码约定

- 基线为 Python 3.8，不使用更新版本专有语法。
- 教学脚本不超过 200 行，公共模块不超过 250 行。
- 使用普通函数和小型类；状态变化使用明确常量和 `if/elif`。
- 不使用 `dataclass`、future annotations、异步框架、装饰器框架、海象运算符或模式匹配。
- 项目自有注释、日志和文档使用正式中文。
- ROS 接口、参数和程序标识符保持原文。

## tools 目录

| 工具 | 用途 |
| --- | --- |
| `install_ubuntu20.sh` | 安装 Ubuntu 20.04 与 ROS Noetic 依赖 |
| `build.sh` | 执行 catkin 增量构建 |
| `test_01_source.sh` | 静态结构和文档检查 |
| `test_02_build.sh` | 构建与包发现检查 |
| `test_03_unit.sh` | 纯单元测试 |
| `test_04_simulation.sh` | 简化假飞控 rostest |
| `test_05_hardware_readonly.sh` | 真实设备只读检查 |
| `sync_workspace.sh` | 不删除远端文件的工作空间同步 |

工具之间不自动串联硬件或实飞步骤。

## 测试位置

| 包 | 测试内容 |
| --- | --- |
| `robotac_examples/test` | 坐标、Tag 筛选、航点解析、假飞控与假 Tag |
| `robotac_localization/test` | 数值有限性和距离计算 |
| `robotac_servo/test` | 控制帧、角度换算、脉宽和软件限位 |

新增飞行逻辑应先补纯函数测试，再补不连接硬件的 rostest。受控实飞结果单独记录，不能写入
自动测试结论。

## 上游版本

`.repos` 和 `versions.lock` 记录固定来源与提交。主要第三方目录如下：

| 目录 | 上游 |
| --- | --- |
| `src/mavros` | MAVLink MAVROS |
| `src/apriltag` | AprilRobotics apriltag |
| `src/apriltag_ros` | AprilRobotics apriltag_ros |
| `src/Livox-SDK2` | Livox SDK |
| `src/livox_ros_driver2` | Livox ROS Driver 2 |
| `src/fast_lio` | FAST-LIO |
| `src/web_cam` | Sunray 相机驱动来源 |

第三方目录中的 README、许可证和变更记录保持原文。项目根目录的 MIT License 不替代第三方
许可证。

## 提交前检查

```bash
# 检查源码、接口和文档链接。
./tools/test_01_source.sh
# 运行纯单元测试。
./tools/test_03_unit.sh
```

在 Ubuntu 20.04 上继续执行构建和仿真。修改接口、坐标、启动前检查或实机参数后，应在
变更说明中写明已经完成哪些测试，还有哪些项目需要在实机上确认。
