# Robotac 文档索引

本文档适用于参赛选手、指导老师和现场安全负责人。建议按“只读检查、预览、仿真、地面联调、
受控实飞”的顺序阅读和操作；前一步未确认时，不要直接进入下一步。

## 推荐阅读顺序

1. [本 Repo 结构](01-project-overview.md)
2. [环境与构建](02-environment-and-build.md)
3. [硬件与配置](03-hardware-and-configuration.md)
4. [架构与接口](04-architecture-and-interfaces.md)
5. [教程索引](tutorials/README.md)
6. [安全与验证](06-safety-and-validation.md)
7. [规则与示例的对应关系](05-competition-examples.md)

## 文档目录

| 文档 | 内容 |
| --- | --- |
| [01 本 Repo 结构](01-project-overview.md) | Repo 用途、包含的内容、目录和数据流向 |
| [02 环境与构建](02-environment-and-build.md) | Ubuntu 20.04、ROS Noetic、依赖和 catkin |
| [03 硬件与配置](03-hardware-and-configuration.md) | MID360、PX4、相机、舵机、标定和部署前检查 |
| [04 架构与接口](04-architecture-and-interfaces.md) | launch、节点、话题、服务和坐标系 |
| [05 规则与示例的对应关系](05-competition-examples.md) | C1 至 C5 与示例的对应关系 |
| [06 安全与验证](06-safety-and-validation.md) | 从源码检查到受控实飞的步骤、接管要求和检查结果 |
| [07 部署与运行](07-deployment-and-operations.md) | 同步、构建、启动、记录和结束检查 |
| [08 故障排查](08-troubleshooting.md) | 常见故障和拒绝启动的原因 |
| [09 开发与上游](09-development-and-upstream.md) | 可修改的代码、工具、测试和第三方版本 |
| [10 舵机标定](10-servo-release-calibration.md) | 投放机构地面点动和参数记录 |

## 教程

`docs/tutorials/` 包含 11 篇编号教程。每篇均列明目的、前置条件、启动命令、预期输出、
失败判断和下一步。会控制飞机的示例不会自动复位或循环执行。

## 文档范围

文档不记录具体飞机地址、内部网络结构、个人本地路径、凭据或一次性进程编号。硬件相关值
统一使用 `<飞机地址>`、`<雷达地址>`、`<串口设备>` 等占位符，并要求在目标设备上核对。

第三方 README、许可证和变更记录保持原文。项目自有说明使用中文；命令、路径、ROS 接口、
参数名和软件名称保留原文。
