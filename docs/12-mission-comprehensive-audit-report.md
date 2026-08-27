# `robotac_mission` 全面问题审计报告

> 审计日期：2026-08-27  
> 排查人：fable5  
> 代码基线：`main`，提交 `7757fb6`  
> 审计对象：`src/robotac_mission` 的状态机、ROS 节点、飞行阶段、坐标、Tag、配置、接口、launch、测试，以及直接支撑 mission 的仿真和取证脚本。  
> 规则基线：[C 组比赛背景与 mission 代码现状基线](11-competition-background-and-mission-status.md)。  
> 结论性质：这是完整静态审计与离线复现报告，不等于 ROS、SITL、HITL 或真机安全认证。

## 1. 总结结论

当前 `robotac_mission` **不具备直接进行带桨比赛飞行的条件**。

本次共记录 **41 项问题**：

| 严重度 | 数量 | 含义 |
| --- | ---: | --- |
| P0 | 7 | 可能直接导致碰撞、空中失去受控降落链、与人工接管冲突，或确定无法满足核心比赛规则；解除前禁止带桨正式飞行 |
| P1 | 21 | 高概率造成任务中止、错误控制、规则得分失败或使安全机制失效；进入受控实飞前必须关闭 |
| P2 | 13 | 可观测性、配置语义、测试、文档和维护性缺口；会放大现场误操作和故障定位风险 |

最危险的不是单一阶段算法，而是以下几条链路同时存在：

1. 默认去程和返程航点沿场地中线穿过障碍物；
2. `ABORT_LAND` 可被 `reset` 直接退出，形成 `WAIT_READY + active=True + driver=None`；
3. 正常降落和中止降落都可能使用起飞前缓存的 `ON_GROUND`，在空中误报完成；
4. `AUTO.LAND` 只请求一次、不确认实际模式，正常 LAND 还会立即停止 setpoint；
5. 遥控器接管造成 OFFBOARD 丢失后，程序会再次请求 `AUTO.LAND`，与人工控制竞争；
6. 当前 `payload.enable=false`，但任务仍能以“安全降落”结束，掩盖 C4 未执行；
7. 代码未执行最多两次、飞行中 6 分钟截止和裁判计时起点。

在 P0 全部关闭、P1 中安全相关项完成 ROS/SITL 故障注入前，建议只允许：源码检查、无 ROS 纯测试、拆桨 dry-run、服务和话题只读检查。

## 2. 审计方法与判定口径

### 2.1 检查范围

本次逐文件检查了：

- 状态与服务：`state_machine.py`、`mission_node.py`；
- 飞行行为：`flight_driver.py`、`_flight_stages.py`、`flight_health.py`；
- 数据与坐标：`coordinates.py`、`tag_tracker.py`、`guards.py`；
- 外部副作用：`interfaces.py`；
- 配置与运行入口：`config.py`、`mission.yaml`、两个 mission launch；
- 单元/ROS 仿真：`src/robotac_mission/test`、`fake_fcu.py`、`fake_tag.py`；
- 工具与证据：`test_03_unit.sh`、`test_04_simulation.sh`、`record_mission.sh`、`collect_evidence.sh`；
- README、package 描述和比赛规则映射。

### 2.2 问题类型

报告区分三种结论：

- **已确认缺陷**：可由当前代码、配置或最小复现直接证明；
- **系统性风险**：代码未建立必要保证，实际后果还依赖 PX4、ROS、TF、机械或现场配置；
- **比赛功能缺口**：程序可能安全运行，但不能证明满足 C1-C5、计时或证据要求。

“全面”指本次范围内逐状态、逐服务、逐配置消费、逐故障路径的静态排查；不表示已经穷尽 ROS 调度、PX4 固件、真实传感器、机械投放和空气动力学中的所有问题。

## 3. P0：禁止带桨飞行的问题

### M-P0-01 默认路线穿过固定障碍物

**类型：已确认缺陷 / C2、C5 阻断。**

当前去程和返程路由的四个中间点全部为 `x=2.0`，并在 `y=2.0` 到 `3.0` 间穿越场地中部；固定障碍横跨场地中央、顶部越障又被规则禁止，因此当前默认路线不是侧隙绕障路线。

证据：

- [mission.yaml:77-87](../src/robotac_mission/config/mission.yaml#L77-L87)；
- [flight_driver.py:49-70](../src/robotac_mission/src/robotac_mission/flight_driver.py#L49-L70) 按原样依次执行这些点；
- [config.py:117-137](../src/robotac_mission/src/robotac_mission/config.py#L117-L137) 只检查端点在场地长方体内，不检查航段与障碍相交。

**后果：** 按当前配置起飞会直接飞向障碍投影，存在碰撞、违规顶部越障和本次飞行立即终止风险。

**关闭条件：** 明确障碍在场地坐标中的包围盒，生成左/右侧合规航段；按机体外形、定位误差、控制超调留净空；对每条线段做障碍膨胀后的相交检查；去程和返程都通过离线几何测试及低速分段实测。

### M-P0-02 `ABORT_LAND` 可被 reset 退出并破坏降落链

**类型：已确认缺陷。**

`ABORT_LAND` 被放入 `TERMINAL`，`request_reset()` 又允许它直接进入 `WAIT_READY`，但没有先确认落地，也没有清除 `active`。节点收到成功 reset 后会把 `driver` 清空并清除降落请求标记。

证据：

- [state_machine.py:38-40](../src/robotac_mission/src/robotac_mission/state_machine.py#L38-L40)、[state_machine.py:126-145](../src/robotac_mission/src/robotac_mission/state_machine.py#L126-L145)；
- [mission_node.py:302-313](../src/robotac_mission/scripts/mission_node.py#L302-L313)；
- 本次最小复现得到：`ABORT_LAND, active=True -> reset -> WAIT_READY, active=True, result=''`。

**后果：** 空中 stop 后如果紧接 reset，可在 `AUTO.LAND` 发出前或执行中取消任务驱动；此后没有 driver、没有继续降落、也不能调用仅对 `ABORT_LAND` 开放的人工接管服务。

**关闭条件：** `ABORT_LAND` 不得视为可 reset 终态；只有“新鲜落地 + 已上锁”或明确人工接管确认后才能进入 `COMPLETE`，再允许 reset。为 `stop/reset/timer` 的所有交错顺序增加测试。

### M-P0-03 缓存的旧 `ON_GROUND` 可在空中误判降落完成

**类型：已确认缺陷。**

正常 LAND 和 ABORT_LAND 都只看缓存的 `extended_state.landed_state`，不检查消息年龄、当前高度、armed 状态或 AUTO.LAND 是否已经生效。

证据：

- 正常 LAND：[\_flight_stages.py:134-147](../src/robotac_mission/src/robotac_mission/_flight_stages.py#L134-L147)；
- 中止 LAND：[mission_node.py:317-333](../src/robotac_mission/scripts/mission_node.py#L317-L333)；
- 飞行健康门没有 `extended_state` 新鲜度检查：[flight_health.py:14-68](../src/robotac_mission/src/robotac_mission/flight_health.py#L14-L68)。

**后果：** 起飞前最后一条 `ON_GROUND` 若在空中仍被缓存，程序可立即进入 `COMPLETE`、清除 `active`，并把空中飞机当作已经安全落地。

**关闭条件：** 落地确认必须同时满足新鲜 `extended_state`、`armed=false`、连续多帧确认和合理高度/垂直速度；确认窗口应从 LAND/AUTO.LAND 请求之后开始，不接受之前的缓存样本。

### M-P0-04 AUTO.LAND 未确认、只请求一次，正常 LAND 会停止 setpoint

**类型：已确认缺陷 / 系统性高危。**

正常 LAND 把 `_land_requested` 置真后仅检查 `SetMode.mode_sent`，不等待 `/mavros/state.mode == AUTO.LAND`；后续也不再发送位置 setpoint。中止降落则在调用服务前把 `_abort_land_issued` 置真，调用失败后只打印 warning，不重试、不升级。

证据：

- [\_flight_stages.py:134-147](../src/robotac_mission/src/robotac_mission/_flight_stages.py#L134-L147)；
- [mission_node.py:335-348](../src/robotac_mission/scripts/mission_node.py#L335-L348)；
- [interfaces.py:36-47](../src/robotac_mission/src/robotac_mission/interfaces.py#L36-L47) 只把 `mode_sent` 当成功。

**后果：** 服务已应答但模式尚未切换、模式随后掉回、或服务第一次失败时，程序可能在仍处于 OFFBOARD 的情况下停止 setpoint，或永久卡在 ABORT_LAND，实际安全行为完全依赖 PX4 未在本仓库验证的 failsafe。

**关闭条件：** 建立“持续发安全 setpoint → 请求 AUTO.LAND → 新鲜状态确认实际进入 → 持续监控落地”的握手；带超时、有限重试、人工接管优先级和清晰升级策略。

### M-P0-05 遥控器接管会触发程序重新请求 AUTO.LAND

**类型：已确认控制冲突。**

飞行中只要 armed 且模式不是 `OFFBOARD`/`AUTO.LAND`，健康门就以“OFFBOARD 模式丢失”中止。操作员把模式切到 POSCTL 等人工模式时，节点下一轮进入 ABORT_LAND 并主动请求 AUTO.LAND；人工接管确认服务只有进入 ABORT_LAND 后才可调用。

证据：

- [guards.py:102-110](../src/robotac_mission/src/robotac_mission/guards.py#L102-L110)、[flight_health.py:28-30](../src/robotac_mission/src/robotac_mission/flight_health.py#L28-L30)；
- [mission_node.py:341-370](../src/robotac_mission/scripts/mission_node.py#L341-L370)。

**后果：** 安全操作员刚接管，软件可能马上再次改变模式，形成控制权争夺；这与规则要求“人工接管后本次飞行停止计分并由操作员控制”冲突。

**关闭条件：** 独立识别 RC override/人工模式/kill switch；人工接管优先于任何自动模式请求；一旦确认接管，mission 永不再请求 OFFBOARD 或 AUTO.LAND。必须用真实遥控器做模式切换和摇杆接管测试。

### M-P0-06 当前不会释放货物，却可以报告任务安全完成

**类型：已确认比赛阻断。**

生产配置为 `payload.enable=false`。RELEASE 状态只记录 `release_skipped`，随后继续 RETURN/LAND；最终结果仍可成为“安全降落”。即使将开关改为 true，也只确认 ROS 服务返回成功，没有机械完全脱离反馈。

证据：

- [mission.yaml:90-96](../src/robotac_mission/config/mission.yaml#L90-L96)；
- [\_flight_stages.py:110-132](../src/robotac_mission/src/robotac_mission/_flight_stages.py#L110-L132)；
- [state_machine.py:146-166](../src/robotac_mission/src/robotac_mission/state_machine.py#L146-L166)。

**后果：** 当前程序确定无法完成 C4；任务结果又不能区分“完成投放”与“跳过投放”，会形成错误成绩和错误取证。

**关闭条件：** 比赛配置禁止在 payload disabled 时进入正式 start；RELEASE 必须输出“命令已发、机构已到位、货物已脱离”的分层结果，至少通过机械/视觉/人工可核验信号确认完全脱离；未完成不得报告完整任务成功。

### M-P0-07 两次飞行和连续 6 分钟没有被完整执行

**类型：已确认规则缺口。**

节点只有首次 start 时间，没有 `flight_count`；只在 start 时看“剩余是否大于预算”，飞行过程中不执行 6 分钟截止；节点重启会重置窗口。首次 start 还无条件建立内部窗口并返回成功，计时起点不是裁判开始指令。

证据：[mission_node.py:108-111](../src/robotac_mission/scripts/mission_node.py#L108-L111)、[mission_node.py:262-292](../src/robotac_mission/scripts/mission_node.py#L262-L292)、[mission_node.py:350-358](../src/robotac_mission/scripts/mission_node.py#L350-L358)。

**后果：** 可以启动第三次飞行、飞过 6 分钟、通过重启清空计时，也会漏计裁判指令后至第一次 start 前的准备时间。

**关闭条件：** 单独建立裁判窗口启动事件、单调时钟、最多两次的持久计数、飞行中硬截止和重启恢复策略；明确截止时是受控降落还是人工接管，并为所有边界时刻写测试。

## 4. P1：进入受控实飞前必须关闭的问题

| ID | 问题与证据 | 触发与后果 | 建议关闭标准 |
| --- | --- | --- | --- |
| M-P1-01 | 坐标变换只有平移，`home_yaw` 不参与 XY 旋转；同时把实际起飞位置无条件映射为配置中的桌心。[coordinates.py:42-67](../src/robotac_mission/src/robotac_mission/coordinates.py#L42-L67)、[flight_driver.py:29-34](../src/robotac_mission/src/robotac_mission/flight_driver.py#L29-L34) | FAST-LIO map 轴若未与场地对齐，或飞机未精确放在桌心，全部航点、边界和 Tag 区域都会整体旋转/平移错误 | 用测量得到的 SE(2)/SE(3) 场地标定；验证双向变换、桌心误差、90°/180°朝向和重启漂移 |
| M-P1-02 | 高度语义互相矛盾：配置把 waypoint z 当场地绝对高度并与桌高比较，起飞却把 `takeoff.z` 当“当前 FC z 的相对爬升”；field z=0 又映射到起飞时 FC z。[config.py:139-150](../src/robotac_mission/src/robotac_mission/config.py#L139-L150)、[\_flight_stages.py:47-59](../src/robotac_mission/src/robotac_mission/_flight_stages.py#L47-L59)、[coordinates.py:42-59](../src/robotac_mission/src/robotac_mission/coordinates.py#L42-L59) | C1 的几何中心离场地地面 1-2 m 无法从当前 z 证明；可能飞得过高、过低或回程下降到错误高度 | 统一 ground/table/FC origin/geometric center 四个高度口径，配置只保留一种语义并做 C1 范围校验 |
| M-P1-03 | 没有障碍位置/朝向、线段碰撞、侧隙、机体半径或停止距离校验；边界只查中心点，还额外允许 `0.1 m` 越界。[config.py:63-74](../src/robotac_mission/src/robotac_mission/config.py#L63-L74)、[flight_health.py:58-67](../src/robotac_mission/src/robotac_mission/flight_health.py#L58-L67) | 即使端点合法，航段也可撞障；机体外轮廓可在中心仍在界内时越线 | 建立机体膨胀几何、航段采样/解析相交、定位误差和控制超调裕量；边界按外轮廓而非中心判定 |
| M-P1-04 | 20 Hz timer 持有全局 `RLock` 时同步调用 set_mode、arm、servo；ServiceProxy 无超时。[mission_node.py:335-348](../src/robotac_mission/scripts/mission_node.py#L335-L348)、[interfaces.py:36-59](../src/robotac_mission/src/robotac_mission/interfaces.py#L36-L59)、[interfaces.py:85-95](../src/robotac_mission/src/robotac_mission/interfaces.py#L85-L95) | 任一 ROS 服务卡住会同时阻塞 setpoint 和 stop/manual_takeover 回调，紧急操作失去响应 | 外部调用移出状态锁，使用异步请求或有界超时；stop/接管使用独立最高优先级路径 |
| M-P1-05 | start、两个 timer 和 FlightDriver 没有统一异常边界；状态先进入 TAKEOFF 后再构造/启动 driver。[mission_node.py:262-293](../src/robotac_mission/scripts/mission_node.py#L262-L293)、[mission_node.py:317-348](../src/robotac_mission/scripts/mission_node.py#L317-L348) | 配置类型、TF、Tag ID、消息结构或发布异常可让当前控制周期无输出，且不保证转入 ABORT；start 中异常可留下 `active=True` 的半初始化任务 | 所有控制入口捕获预期和未知异常，原子创建 driver 后再提交状态，异常统一进入可执行的安全状态并记录 traceback/root cause |
| M-P1-06 | 订阅回调未使用文件注释声称的 `_lock`，消息对象和 `*_received` 分两次赋值。[mission_node.py:40-64](../src/robotac_mission/scripts/mission_node.py#L40-L64)、[mission_node.py:141-171](../src/robotac_mission/scripts/mission_node.py#L141-L171) | timer 可观察到“新消息 + 旧时间”或反向组合，也可同时读取多个时刻不同的 FCU/pose/estimator 快照 | 回调在锁内生成不可变快照，控制周期一次性读取同一版本；并发测试覆盖 stop/start/tag/health 交错 |
| M-P1-07 | 飞行中不检查 FCU、estimator、vision_state、extended_state 的数据年龄；pose/timesync 也缺少负值和非有限时间/RTT的完整处理。[flight_health.py:14-49](../src/robotac_mission/src/robotac_mission/flight_health.py#L14-L49) | 话题静默后最后一个 healthy 值可长期放行；ROS 时间异常或 NaN 可能绕过阈值 | 对每个控制依赖使用到达时刻单调时钟、新鲜度、有限数和来源一致性检查；落地状态单独处理 |
| M-P1-08 | OFFBOARD 和 arm 只看服务响应，不等待实际 FCU 状态；`_takeoff_armed` 一旦置真不再重试。[\_flight_stages.py:15-45](../src/robotac_mission/src/robotac_mission/_flight_stages.py#L15-L45)、[interfaces.py:36-59](../src/robotac_mission/src/robotac_mission/interfaces.py#L36-L59) | 异步状态更新可能造成刚解锁就被“模式丢失”误杀，或服务成功但模式/解锁没有实际生效 | 把 mode/arm 设计为有确认和超时的子状态；基于新鲜 `/mavros/state` 确认后再推进 |
| M-P1-09 | ALIGN 中 Tag 不 fresh 时没有清空 `_reached_since`。[\_flight_stages.py:75-108](../src/robotac_mission/src/robotac_mission/_flight_stages.py#L75-L108) | 已进入容差后 Tag 丢失，3 秒计时仍继续；重获一帧后可立即完成“连续稳定” | 任一 Tag 丢失、TF 失败或越出容差立即重置连续保持计时，并测试丢失 1 帧/多帧/临界 3 秒 |
| M-P1-10 | Tag 无检测时不清稳定样本；超时只清 samples，不清 `pose_map`、last stamp；重放同时间戳消息可恢复旧 pose 为 fresh。[tag_tracker.py:30-67](../src/robotac_mission/src/robotac_mission/tag_tracker.py#L30-L67) | 间歇检测会被累计为连续稳定，旧消息重放可能绕过重新稳定 | 明确定义连续窗口；空帧/超时/时间倒退全部失效并清理旧 pose；只接受严格递增时间戳 |
| M-P1-11 | Tag ID `int()` 转换和 TF 后位置没有类型、有限数及协方差检查。[tag_tracker.py:69-92](../src/robotac_mission/src/robotac_mission/tag_tracker.py#L69-L92) | 畸形 ID 可抛异常；NaN/Inf 可传播为 setpoint | 消息解析完全防御式；候选 pose 必须有限、时间合法、在物理可达范围内并满足质量阈值 |
| M-P1-12 | SEARCH 只在首次进入 ALIGN 前校验投放台区域，ALIGN 不再校验；两张桌 Tag 都是 ID 0，Tracker 取第一个匹配检测。[\_flight_stages.py:61-73](../src/robotac_mission/src/robotac_mission/_flight_stages.py#L61-L73)、[\_flight_stages.py:75-108](../src/robotac_mission/src/robotac_mission/_flight_stages.py#L75-L108)、[tag_tracker.py:69-93](../src/robotac_mission/src/robotac_mission/tag_tracker.py#L69-L93) | 检测切换到另一桌或错误目标后，飞机仍可能追踪它 | ALIGN 每帧验证目标仍在投放桌可信区域，结合任务阶段、预测轨迹和数据关联；错误桌面立即退回 SEARCH/悬停 |
| M-P1-13 | 动态 ALIGN 目标只做每步限速，不做最终场地/桌面/障碍约束；健康门是在飞机已经移动后检查中心点。[flight_driver.py:93-108](../src/robotac_mission/src/robotac_mission/flight_driver.py#L93-L108)、[\_flight_stages.py:87-106](../src/robotac_mission/src/robotac_mission/_flight_stages.py#L87-L106) | 错 TF/外参/Tag 可持续把飞机引向墙或障碍，直到事后越界中止 | 所有输出先经过有限数、可达域、障碍、机体边界、速度和加速度统一约束，再发布 |
| M-P1-14 | payload 开启时没有起飞前检查舵机服务；重试在 20 Hz 锁内无等待连续执行，也没有调用级超时。[\_flight_stages.py:110-125](../src/robotac_mission/src/robotac_mission/_flight_stages.py#L110-L125)、[interfaces.py:85-95](../src/robotac_mission/src/robotac_mission/interfaces.py#L85-L95) | 飞到投放台才发现服务缺失；服务卡住会阻塞飞控；快速重试不能区分暂态和机械故障 | payload enabled 时将舵机健康、标定、服务可达和初始位置纳入 start 门；有界异步重试并记录每次结果 |
| M-P1-15 | start 前不检查 Tag TF 链、相机/检测、舵机、MAVROS setpoint consumer；已有 `tf_chain_ok` 和 `camera_frame` 没接入运行。[guards.py:127-131](../src/robotac_mission/src/robotac_mission/guards.py#L127-L131)、[mission_node.py:100-108](../src/robotac_mission/scripts/mission_node.py#L100-L108) | 无法完成任务的飞机仍会起飞，只能在空中等待超时后降落 | 按配置功能开关建立依赖清单和 start readiness；依赖丢失时在地面拒绝，不把系统集成问题留到空中发现 |
| M-P1-16 | 配置校验存在多处逃逸：缺 `stage_timeout` 抛未包装 KeyError；NaN、负障碍尺寸、字符串 tolerance、0 秒规则保持、budget 大于窗口均可被接受。[config.py:50-170](../src/robotac_mission/src/robotac_mission/config.py#L50-L170)、[config.py:173-209](../src/robotac_mission/src/robotac_mission/config.py#L173-L209) | 节点可在 BOOT 崩溃，或把非法值带入 timer、边界和控制计算 | 所有错误统一为 `ConfigError`；数值必须 finite；增加范围、关系、未知键和布尔/整数严格校验；见第 7.3 节变异复现 |
| M-P1-17 | `flight_budget=150 s`，但启用阶段的最大超时和为约 `170 s`，另有 2 秒预流；预算只在 start 用，首次 start 还无条件通过。[mission.yaml:41-55](../src/robotac_mission/config/mission.yaml#L41-L55)、[mission_node.py:350-358](../src/robotac_mission/scripts/mission_node.py#L350-L358) | “剩余 150 秒可飞”不能证明这一轮能在窗口内收尾 | 校验 `budget >=` 任务最坏时间或重新定义硬截止；在飞行中按剩余时间选择安全收尾 |
| M-P1-18 | mission 节点没有 shutdown/进程异常后的显式降落策略，安全完全依赖未在本仓库验证的 PX4 Offboard loss/failsafe 参数 | 节点崩溃、被杀或 Python timer 失效时，仓库内没有继续 setpoint 或模式仲裁的执行者 | 读取并实测 PX4 failsafe；建立独立于 mission 主锁/主进程的 watchdog，明确 crash 后控制权归属 |
| M-P1-19 | mission ROS 仿真自身不成立：加载生产 `flight_enabled=true/dry_run=false` 配置，却断言骨架不飞；fake FCU 在没有 setpoint 时不会发布 estimator/timesync/healthy；fake Tag 位置也不对应投放台。[mission_sim.test:10-17](../src/robotac_mission/launch/mission_sim.test#L10-L17)、[mission_sim_test.py:213-227](../src/robotac_mission/test/mission_sim_test.py#L213-L227)、[fake_fcu.py:108-142](../src/robotac_examples/test/fake_fcu.py#L108-L142)、[fake_tag.py:21-35](../src/robotac_examples/test/fake_tag.py#L21-L35) | 测试会卡在 WAIT_READY，或一旦就绪就触发真实飞行语义，不能验证任何完整 mission | 使用独立测试 YAML 和 mission 专用 fake，完成 TAKEOFF→LAND、ABORT、Tag、servo、计时全流程 |
| M-P1-20 | 没有 FlightDriver、FlightStageMixin、TagTracker、MissionInterfaces、MissionNode 服务/并发的直接测试；现有状态机测试未覆盖空中 ABORT reset | 本次 P0 中多项缺陷可长期存在而 65 个纯测试仍全绿 | 建立组件级和 ROS 级故障矩阵，加入服务卡死、模式异步、消息冻结、Tag 丢失、时钟边界和并发交错 |
| M-P1-21 | C5 仅返回固定坐标后调用 AUTO.LAND，不重新识别起降台 Tag，也不估计落点得分区。[flight_driver.py:69-70](../src/robotac_mission/src/robotac_mission/flight_driver.py#L69-L70)、[\_flight_stages.py:134-147](../src/robotac_mission/src/robotac_mission/_flight_stages.py#L134-L147) | 坐标漂移或起飞锚点误差会直接变成降落偏差，无法证明落入 0.8 m 有效区 | 增加起降台视觉复核/精确降落或至少给出落点误差估计、容差和实飞统计 |

## 5. P2：工程、可观测性与维护问题

| ID | 问题 | 证据与影响 |
| --- | --- | --- |
| M-P2-01 | 人工接管结果会被原始中止原因遮蔽 | `result` 只保留首个根因，真实飞行先 abort 再 manual takeover 后仍显示“任务中止”，只在 `reason` 中显示人工接管。[state_machine.py:169-195](../src/robotac_mission/src/robotac_mission/state_machine.py#L169-L195)、[state_machine.py:216-218](../src/robotac_mission/src/robotac_mission/state_machine.py#L216-L218) |
| M-P2-02 | ERROR reset 后新 `rate_hz` 不会重建 Timer | 控制定时器只在构造函数创建一次；配置重载只替换 dict。[mission_node.py:112-117](../src/robotac_mission/scripts/mission_node.py#L112-L117)、[mission_node.py:302-312](../src/robotac_mission/scripts/mission_node.py#L302-L312) |
| M-P2-03 | 非飞行态 target 把场地航点直接标成 `map` | 未经 `field_to_map` 的 `[2,0.8,1]` 被发布为 `map`，ABORT/COMPLETE 还会突然显示该名义点，误导 RViz 和 rosbag。[mission_node.py:386-415](../src/robotac_mission/scripts/mission_node.py#L386-L415) |
| M-P2-04 | 多个配置项只被校验/注释、不参与运行 | `obstacle.no_overfly/cross_gap/size`、`tables.takeoff_center`、`tag.family/black_size_m`、`frames.body_frame`、`camera_frame` 不形成控制或安全判定，造成“写了配置即已生效”的假象 |
| M-P2-05 | `mission`/`return` 允许列表，但 driver 只取第 0 项 | 配置中后续点静默忽略。[flight_driver.py:22-24](../src/robotac_mission/src/robotac_mission/flight_driver.py#L22-L24)、[config.py:130-137](../src/robotac_mission/src/robotac_mission/config.py#L130-L137) |
| M-P2-06 | `stage_timeout.land` 被要求和校验但从不使用 | LAND 使用 `land_confirm`，现场修改 land stage timeout 不会生效。[config.py:22-24](../src/robotac_mission/src/robotac_mission/config.py#L22-L24)、[\_flight_stages.py:134-147](../src/robotac_mission/src/robotac_mission/_flight_stages.py#L134-L147) |
| M-P2-07 | `health_debounce` 以 tick 次数而非秒定义，timesync 与 vision 共用一个计数器 | 改 `rate_hz` 会改变实际去抖时间；故障类别交替可累计或互相清除。[flight_health.py:34-49](../src/robotac_mission/src/robotac_mission/flight_health.py#L34-L49) |
| M-P2-08 | Tag 稳定均值不是固定窗口 | `_samples` 达标后继续无限累积并平均全部历史，响应变慢；长时间运行还会增长内存。[tag_tracker.py:55-65](../src/robotac_mission/src/robotac_mission/tag_tracker.py#L55-L65) |
| M-P2-09 | mission 取证话题不足 | `record_mission.sh` 未录 `/robotac_mission/active`、`target`、舵机状态/命令、Tag 选择和对准偏差；难以证明 C3/C4 和故障链。[record_mission.sh:14-28](../tools/record_mission.sh#L14-L28) |
| M-P2-10 | README、launch、package 与当前生产配置漂移 | YAML 当前会真发控制，但注释仍写 dry-run/骨架；launch 描述“只进入等待态”，package 仍称仅有预启动状态。[mission.yaml:1-7](../src/robotac_mission/config/mission.yaml#L1-L7)、[competition_mission.launch:1-9](../src/robotac_mission/launch/competition_mission.launch#L1-L9)、[package.xml:3-6](../src/robotac_mission/package.xml#L3-L6) |
| M-P2-11 | ROS 话题和服务全部使用绝对名称 | 难以 namespace、多机测试或启动两个隔离实例，也提高测试误连生产节点的风险。[mission_node.py:66-98](../src/robotac_mission/scripts/mission_node.py#L66-L98)、[interfaces.py:24-29](../src/robotac_mission/src/robotac_mission/interfaces.py#L24-L29) |
| M-P2-12 | `pending_actions` 无界增长 | dry-run 每个 20 Hz setpoint 都追加字符串，长时间地面联调会持续占用内存。[interfaces.py:20-34](../src/robotac_mission/src/robotac_mission/interfaces.py#L20-L34)、[interfaces.py:72-79](../src/robotac_mission/src/robotac_mission/interfaces.py#L72-L79) |
| M-P2-13 | 内部转移历史和动作日志没有对外发布 | 状态机保留 200 条 transitions、interfaces 保留 pending actions，但 rosbag 无法直接取得，现场只能看到当前 state/reason/result，难以还原竞态和模式请求顺序。[state_machine.py:65-70](../src/robotac_mission/src/robotac_mission/state_machine.py#L65-L70)、[state_machine.py:202-214](../src/robotac_mission/src/robotac_mission/state_machine.py#L202-L214) |

## 6. 状态不变量检查结果

| 状态/事件 | 应满足的不变量 | 当前结果 |
| --- | --- | --- |
| WAIT_READY / WAIT_START | `active=false`、无飞行 driver、不会发送控制 | 正常预启动成立；从 ABORT reset 可破坏，出现 `active=true` |
| TAKEOFF | 已建立坐标锚点；持续 setpoint；OFFBOARD 和 armed 均被新鲜状态确认 | 坐标只平移；服务响应即视为完成；缺状态确认 |
| TRANSIT / RETURN | 航段避障、机体不越界、持续健康检查 | 只校验静态端点和实际中心点；默认路线穿障碍 |
| SEARCH_TAG | 稳定、属于投放台的 Tag 才推进 | 首次区域过滤存在；连续样本与旧消息处理不严格 |
| ALIGN_TAG | 连续 3 秒有效视觉对准，目标始终可达且属于投放台 | Tag 丢失不清保持计时；目标无几何约束；区域不再验证 |
| RELEASE | 对准仍有效、机构可用、货物已完全脱离 | 当前直接跳过；服务成功也没有完全脱离证据 |
| LAND | 已确认 AUTO.LAND；继续监控；新鲜、多条件落地确认 | 模式不确认、setpoint 停止、旧 ON_GROUND 可直接完成 |
| ABORT_LAND | 不可 reset；自动降落与人工接管互斥；失败可升级 | 当前三个条件均不成立 |
| COMPLETE | 飞机真实落地或人工接管，`active=false`，结果能区分任务完成程度 | 可能空中误入；result 不能表达 payload skipped/人工接管终态 |
| ERROR | 不发送控制；修复配置后完整重建运行参数 | 不发送控制成立；rate Timer 不随重载更新，部分配置异常会直接逃逸成未捕获异常 |

## 7. 离线验证证据

### 7.1 语法检查

执行：

```bash
python3 -m compileall -q src/robotac_mission/src \
  src/robotac_mission/scripts src/robotac_mission/test
```

结果：**PASS**。这只证明 Python 文件可编译，不证明 ROS 依赖或飞行行为正确。

### 7.2 现有纯 Python 测试

- 5 个测试文件共声明 81 个 `test_*`；
- 本地 macOS 使用正确 `PYTHONPATH` 后，状态机、坐标、guards、flight_health 共 **65 项通过**；
- 16 个 config 测试因本机缺少 PyYAML 而没有加载；unittest 报告为 65 个测试通过、1 个测试模块导入错误；
- 本次没有 ROS Noetic 环境，未执行 catkin、rostest、SITL/HITL 或真机测试。

### 7.3 配置变异复现

以当前 `mission.yaml` 为基线直接调用 `validate()`：

```text
missing_stage_timeout: KeyError: 'stage_timeout'
nan_transit_timeout: ACCEPTED
budget_larger_than_window: ACCEPTED
zero_rule_holds: ACCEPTED
negative_obstacle_size: ACCEPTED
string_field_tolerance: ACCEPTED
```

这证明配置问题不只是“建议加强”：其中既有非法值被放行，也有异常类型逃出 `ConfigError` 的实际缺陷。

### 7.4 状态机最小复现

执行纯 Python 流程 `BOOT -> WAIT_READY -> WAIT_START -> TAKEOFF -> stop -> reset`：

```text
after_stop  ABORT_LAND True '任务中止：操作员 stop'
after_reset WAIT_READY  True ''
```

另一个流程 `abort('root cause') -> confirm_manual_takeover()` 得到：

```text
state=COMPLETE active=False result='任务中止：root cause' reason='人工接管'
```

第一项验证 M-P0-02；第二项验证 M-P2-01。

### 7.5 当前未形成有效验证的项目

- `tools/test_04_simulation.sh` 只运行 `robotac_examples examples_sim.test`，不运行 mission rostest；
- mission 自己的 `mission_sim.test` 与生产配置、fake 节点互相矛盾；
- 没有完整 TAKEOFF→TRANSIT→SEARCH→ALIGN→RELEASE→RETURN→LAND 的自动测试记录；
- 没有 AUTO.LAND 服务超时、模式不切换、RC takeover、节点退出、TF 错误、Tag NaN、舵机卡死的故障注入；
- 没有场地标定、障碍净空、机体外轮廓、带载投放和降落精度的实测数据。

## 8. 建议修复顺序与验收门

### 阶段 A：先恢复可证明的安全终止

1. 修复 ABORT reset、落地新鲜度、多条件落地确认；
2. 建立 AUTO.LAND 模式确认、持续 setpoint、有界重试和失败升级；
3. 明确 RC 接管最高优先级，接管后软件不得再抢模式；
4. 外部服务移出状态锁，给 stop/人工接管独立响应路径；
5. 为异常、节点 shutdown 和 PX4 failsafe 建立可重复故障测试。

**验收门：** 拆桨 ROS 测试中，任意状态下 stop 都能在限定时间内响应；reset 无法退出空中 ABORT；旧 landed 消息不能完成任务；人工模式切换后不再出现自动模式请求。

### 阶段 B：建立正确的场地与任务几何

1. 统一场地坐标标定、yaw 旋转和高度口径；
2. 定义障碍包围盒、机体半径、边界/侧隙净空；
3. 重做去返程航点并自动检查每个航段；
4. 对所有静态和动态 setpoint 做统一安全约束；
5. 增加起降台视觉复核或量化落点误差。

**验收门：** 离线几何测试证明整条航迹与膨胀障碍不相交、机体外轮廓不越界；旋转/偏置场景的坐标单测通过；分段低速实测与日志一致。

### 阶段 C：完成 C3/C4 与比赛控制面

1. 修复 Tag 连续稳定、丢失重置、时间戳、有限数和桌面数据关联；
2. payload enabled 前置检查、机械反馈和投放结果建模；
3. 实现裁判窗口、最多两次、飞行中截止和重启策略；
4. 发布 Tag 状态/偏差、flight count、window remaining、payload result 和事件历史；
5. 修复 mission 专用仿真并纳入标准测试脚本。

**验收门：** 完整 ROS/SITL 流程可重复通过；每个失败注入都进入预期安全状态；rosbag 能独立证明 C1-C5 的输入、决策、控制和结果。

### 阶段 D：受控真机递进

按“拆桨地面 → 空机低高度悬停 → 单侧绕障 → Tag 对准 → 空载舵机 → 带载投放 → 返航降落 → 两次/6 分钟全流程”递进。任何阶段的未解释异常都应退回上一阶段，不应直接用完整比赛飞行验证代码修复。

## 9. 最终判定

当前实现具备 C1-C5 的状态名称、基本前向流程和部分启动/飞行健康门，但它尚未形成一个闭合的安全状态机：

- 安全终止态可以被 reset 破坏；
- 落地事实不可信；
- 自动降落与人工接管没有完成仲裁；
- 航迹和坐标没有证明满足场地几何；
- C4 与比赛计时确定未完成；
- 自动测试没有覆盖真实运行配置和完整飞行链。

因此本报告的发布判定为：**NO-GO（禁止带桨正式飞行）**。建议先按阶段 A 关闭安全终止链，再处理几何和比赛功能；不建议先通过调大 timeout、tolerance 或关闭健康门来绕过问题。
