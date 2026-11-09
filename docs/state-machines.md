# Runtime state inventory / 运行状态清单

The console, simulator and ROS adapters are separate processes. Their states are
related but have different owners; they are not one global state machine. This
inventory covers the interactive runtime and the optional jobs started by SpRobo.
States calculated from a timestamp or a live connection remain derived rather
than being copied into another variable.

操作台、仿真器和 ROS 适配器是独立进程。各进程负责自己的状态，它们不能合并成一个
全局状态机。本清单覆盖交互运行时和 SpRobo 启动的可选任务。能从时间戳或实时连接
计算的状态保持为派生值，不再保存一份副本。

## User-facing flows / 用户操作流程

| Flow / 流程 | States and transitions / 状态与转换 | Owner / 实现 |
| --- | --- | --- |
| Session / 会话 | `config` → `running` ↔ `paused`; any open state can close. Stop clears motion and returns ownership to the GUI without pausing physics; mount edits require `config`. Before the first start, pause keeps `config`.<br>配置后进入运行，可暂停和恢复；任何未关闭的阶段都可以关闭。停车清零并交回 GUI 控制权，但不暂停物理；外参只能在配置阶段修改。首次启动前暂停仍处于配置阶段。 | [Session observer](../src/session_control.cpp) |
| Process and connection / 进程与连接 | A newer restart supersedes an older one. Exit or connection loss clears stale UI state and manual heartbeat; the console marks any retained view as the last frame.<br>新重启会替代旧重启。核心退出或连接断开时，界面清除过期状态和手动心跳，并将保留的画面标为上一帧。 | [Manager](../console/session/worker.go), [console](../console/app_frames.go), [viewport](../console/ui_media.go) |
| Page and overlays / 页面与浮层 | Launch, robot and scene are configuration pages; starting enters the operating page. Tutorials and diagnostics are separate overlays. Loading, checks and preparation can overlap.<br>启动、机器人和场景属于配置页面；开始后进入运行页。教程、日志浮层及加载、检查、准备过程相互独立。 | [Navigation](../console/ui_pages.go), [layout](../console/ui_layout.go), [preflight](../console/app_preflight.go) |
| Viewport input / 视口手势 | Idle, orbit, pan and transform are exclusive. Release, selection or history restore ends the relevant gesture; Shift can temporarily pan during an orbit.<br>空闲、旋转视角、平移视角和变换物体互斥。松开、选择或恢复历史会结束相应手势；旋转时可临时按 Shift 平移。 | [Viewport](../console/viewport_input.go), [history](../console/scene_history.go) |
| Scene and preset drafts / 场景与预设草稿 | Editing changes a draft; save updates its preset. Undo/redo restore snapshots, and one continuous drag is one edit.<br>修改先进入草稿；保存后更新预设。撤销／重做恢复快照，一次连续拖动视为一次编辑。 | [Workspace](../console/workspace.go), [history](../console/scene_history.go) |
| Sensor mount edit / 传感器外参 | Edits require a connected core in `config` and are pending until matching metadata acknowledges them. Rejection or disconnection restores acknowledged mounts. MID360's IMU remains coupled to its LiDAR.<br>外参编辑需要核心已连接且处于配置阶段。修改在核心确认前处于待确认状态；请求失败或断连时恢复已确认外参。MID360 内部 IMU 与雷达保持刚性连接。 | [Edit and acknowledgement](../console/app.go), [error handling](../console/app_frames.go) |
| Manual input / 手动控制 | Keyboard and joystick require manual ownership and a running session. Release, focus loss or Space clears manual motion; program motion is independent. The console sends a manual heartbeat.<br>键盘和摇杆只在运行中的手动模式生效。松开、失焦或空格清除手动运动，不影响程序控制；操作台定期发送手动心跳。 | [Keyboard](../console/control_keyboard.go), [controls](../console/ui_controls.go), [ownership](../console/ui_sources.go) |
| Dependency and adapter preparation / 依赖与适配器准备 | A newer dependency check supersedes an older one. Each ROS image may be ready, preparing or failed; adapters start and stop independently.<br>新依赖检查替代旧检查。各 ROS 镜像分别处于就绪、准备中或失败状态；适配器独立启动和停止。 | [Preflight](../console/app_preflight.go), [adapters](../console/session/adapters.go) |
| Mapping tutorial / 建图教程 | The manager prepares, starts and stops the frontend or reports failure. The frontend reports `initializing`, `running`, `stale`, `invalid_pose` or `storage_error`; these do not change command ownership.<br>管理器准备、启动和停止前端，或报告失败。前端报告初始化、运行、延迟、位姿无效或存储错误；这些状态不改变指令控制权。 | [Manager](../console/session/tutorial.go), [frontend observer](../tutorials/observer/main.cpp) |

## Simulation and transport / 仿真与通信

| Flow / 流程 | States and transitions / 状态与转换 | Owner / 实现 |
| --- | --- | --- |
| Command source and freshness / 指令来源与时效 | Source is GUI, program, ROS 1, ROS 2 or none. Status is derived as `paused`, `unowned`, `waiting`, `receiving` or `timed_out`; owner change, disconnect or timeout clears the relevant command.<br>来源可以是 GUI、程序、ROS 1、ROS 2 或无。状态由暂停、所有权和指令时间派生；切换所有者、断开或超时会清除相应指令。 | [Command status](../ipc/server_transport.cpp), [protocol](../ipc/server_protocol.cpp) |
| IPC pairing / IPC 配对 | Control and data connections use the same role and token. Control hello authorizes data pairing; closing either channel disconnects the client. Frame queues are bounded.<br>控制与数据连接使用相同角色和令牌。控制连接先完成握手，数据连接才能配对；任一通道关闭会断开客户端。帧队列有容量限制。 | [Server protocol](../ipc/server_protocol.cpp), [client](../console/session/ipc.go), [socket queues](../ipc/socket.cpp) |
| Quadrotor / 四旋翼 | `grounded`, `disarmed`, `takeoff_blocked`, `takeoff`, `hover`, `velocity`, `landing`, `fault`. Commands, clearance, tilt and touchdown drive transitions; each transition resets the integral.<br>状态包括落地、停桨、起飞受阻、起飞、悬停、速度控制、降落和故障；指令及环境测量触发转换，转换时重置积分。 | [Flight controller](../src/controllers/flight.cpp) |
| Go2 startup / Go2 起身 | Optional upstream get-up advances from `pre_getup` to `getup`, then policy control. Other startup configurations enter policy control directly.<br>可选上游起身流程先预备、再起身，之后进入策略控制；其他启动配置直接进入策略控制。 | [Go2 controller](../src/controllers/go2.cpp) |
| Wheel and tracked control / 轮式控制 | Ranger Mini holds steering targets while braking and waits for alignment before driving. The optional velocity tracker keeps filter, heading and integral history; zero-command braking resets correction.<br>Ranger Mini 制动时保持转向目标，车轮对齐后才驱动。可选速度跟踪器保留滤波、航向和积分历史；零指令制动会重置修正量。 | [Ranger Mini](../src/controllers/ranger_mini.cpp), [tracker](../src/tracking.cpp) |
| Course assessment / 地形验收 | Phase follows position (`approach`, `ascent`, `top`, `descent`, `traversal`, `exit`, `complete`) and can move backwards. Success and failure are separate.<br>阶段由位置计算，可以随倒退回退；成功和失败单独记录。 | [Course monitor](../src/course.cpp) |
| Sensor production / 传感器生产 | LiDAR, IMU and camera schedules use simulation timestamps. The camera worker is idle, rendering, failed or stopping, and publishes each completed snapshot with its original timestamp.<br>雷达、IMU 和相机按仿真时间采样。相机工作线程处于空闲、渲染、失败或停止中，完成的画面保留原采样时间。 | [LiDAR](../src/runtime_lidar.cpp), [IMU](../src/runtime_imu.cpp), [camera worker](../src/runtime_camera_worker.cpp) |
| Diagnostics / 日志 | Events enter a bounded queue, flush in batches and rotate storage. Drops and failures are counted; shutdown attempts a final flush. Tracing is independent of session state.<br>事件进入有界队列、批量写入并轮转；丢失和写入失败有计数，关闭时尝试排空。追踪开关独立于会话状态。 | [C++ logger](../diagnostics/event_log.cpp), [console logger](../console/diagnostics/log.go) |
| Optional batch physics / 可选批量物理 | A validated job exports its contract, optionally runs a separate backend, then completes or reports failure/timeout. It is separate from the interactive session.<br>任务先校验并导出接口，再按需启动独立后端，最后完成或报告失败／超时；它不属于交互仿真会话。 | [Job launcher](../console/physics_job.go) |

The explicit enums are used where values are mutually exclusive and owned by one
module. Independent facts such as sensor enablement, ROS connections, current
input, and pending mount acknowledgements remain separate. The protocol strings
and existing ROS topics are unchanged.

只有同一模块中互斥的阶段使用枚举。传感器开关、ROS 连接、当前输入和待确认外参等
独立事实仍分别保存。协议字符串和现有 ROS 话题保持不变。

## Lifecycle validation / 生命周期验证

The lifecycle E2E uses a real core and IPC connection. It checks initial and
automatic start, pause, resume, stop, mount edits and shutdown. The console
integration test also checks a lost connection, early core exit and an adapter
preparation that completes after disconnect, including interrupted mount edits.

生命周期端到端测试使用真实核心和 IPC，检查配置、自动启动、暂停、恢复、停车、
外参修改与关闭。操作台集成测试还检查连接断开、核心启动失败，以及断开后才完成
的适配器准备与中断的外参修改。

```bash
python3 tools/test_session_lifecycle.py --core /path/to/simple-rbt-sim
```

On 2026-11-09, the local build passed 19 CTest cases, the Go race suite, seven
scene-editor checks and 40 native-window checks. Direct ROS tests passed with
CPU physics: Go2W with RL-SAR on Noetic and Scout Mini on Noetic and Humble.
Those ROS tests disabled RGB and depth; they do not assess SuperMimic lateral
tracking.

2026-11-09 的本机构建通过 19 项 CTest、Go 竞态测试、7 项场景编辑检查和 40 项
原生窗口检查。直接 ROS 测试使用 CPU 物理：Go2W 使用 RL-SAR 和 Noetic，Scout
Mini 使用 Noetic 和 Humble。这些 ROS 测试关闭了 RGB 和深度，不用于评价
SuperMimic 侧移跟踪。
