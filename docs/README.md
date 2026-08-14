# Robotac 中文文档

本目录收录 Robotac 项目自有文档，内容按照系统认知、构建、配置、验证、部署与故障处置的
顺序组织。第三方组件的原始 README、许可证和变更记录保留原文。

## 阅读顺序

### 基础阅读

1. [项目总览](01-project-overview.md)
2. [环境与构建](02-environment-and-build.md)
3. [硬件与配置](03-hardware-and-configuration.md)
4. [安全与验证](06-safety-and-validation.md)

### 竞赛任务准备

1. [架构与接口](04-architecture-and-interfaces.md)
2. [竞赛任务](05-competition-mission.md)
3. [安全与验证](06-safety-and-validation.md)
4. [部署与运维](07-deployment-and-operations.md)

### 故障处置

故障处置以 [故障排查](08-troubleshooting.md) 为入口，并按故障所属链路查阅硬件、接口或验证章节。

## 按职责索引

| 职责 | 对应文档 |
| --- | --- |
| 参赛选手 | 项目总览、环境与构建、竞赛任务 |
| 感知与定位 | 硬件与配置、架构与接口、故障排查 |
| 飞控与控制 | 架构与接口、安全与验证、竞赛任务 |
| 机务与现场保障 | 硬件与配置、部署与运维、故障排查 |
| 指导老师或安全负责人 | 项目总览、安全与验证、部署与运维 |
| 仓库维护者 | 开发与上游、架构与接口、验证矩阵 |

## 文档目录

- [01 项目总览](01-project-overview.md)
- [02 环境与构建](02-environment-and-build.md)
- [03 硬件与配置](03-hardware-and-configuration.md)
- [04 架构与接口](04-architecture-and-interfaces.md)
- [05 竞赛任务](05-competition-mission.md)
- [06 安全与验证](06-safety-and-validation.md)
- [07 部署与运维](07-deployment-and-operations.md)
- [08 故障排查](08-troubleshooting.md)
- [09 开发与上游](09-development-and-upstream.md)

## 文档边界

- 命令、路径、ROS 接口、参数名和软件名称保持原文，说明文字使用中文。
- 示例中的 `<飞机地址>`、`<雷达地址>`、`<串口设备>` 必须按现场记录替换。
- 规则内容仅摘要完成任务所需的约束，不复制组委会完整文件。
- 文档不提供绕过安全门禁、跳过标定或无人监管起飞的方法。
- 仓库内容与正式规则冲突时，以组委会最新文件和技术通知为准。

[返回项目首页](../README.md)
