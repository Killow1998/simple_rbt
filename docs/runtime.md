# Runtime and module boundaries / 运行与模块边界

## Processes / 进程

`simple-rbt-sim` owns the `simple_rbt` C++ framework: MuJoCo physics, control, sensors, rendering and session state. `sprobo` starts the **SpRobo** native Go/MyGo UI. ROS 1 and ROS 2 use separate C++ adapters, normally in separate public-image containers. Core and console do not link ROS or start Python.

`simple-rbt-sim` 运行 `simple_rbt` C++ 框架，负责物理、控制、传感器时序、离屏渲染和会话状态；`sprobo` 启动 **SpRobo** 原生 Go／MyGo 界面。ROS 1、ROS 2 使用独立 C++ 适配器，通常各自运行在公开镜像容器内。核心与操作台不链接 ROS，也不启动 Python。

```text
MyGo console ── control / live views ── C++ headless core
                                          │
                                neutral local session protocol
                                   ┌──────┴──────┐
                              C++ ROS 1      C++ ROS 2
                              container      container
```

The protocol uses separate control and data Unix sockets, fixed little-endian packet fields and bounded queues. The GUI consumes the latest observation frames; ROS adapters receive sensor samples for publication. A command owner is selected explicitly. Disconnect, malformed commands and the 0.5 s deadman clear motion commands. Stop clears the owner and commands zero while physics remains active; Pause freezes simulation time. Returning to configuration restarts the owned session.

协议使用分离的控制／数据 Unix socket、固定的小端字段和有界队列。GUI 消费最新观察画面；ROS 适配器接收传感器样本并发布消息。指令来源显式选择；断开、非法指令和 0.5 秒超时会清零。停车清除指令来源并给出零指令，物理仍运行；暂停冻结仿真时间。返回配置会重启操作台拥有的会话。

## Software camera rendering / 软件相机渲染

Perspective RGB/depth remains 320×240, panorama RGB/radial depth remains
720×360 from six 256×256 faces. Desktop sessions retain 10 Hz cameras and
MID360 scans (20,000 rays), with both enabled IMUs at 200 Hz. These are rates
in simulation time; a slow software renderer also slows the simulation.

Software llvmpipe/softpipe keeps shadows with a 2048-pixel shadow-map cap;
smaller model settings are preserved. Output resolution, depth arithmetic,
multisampling, physics and sensor sampling do not change. Hardware
rendering retains the model's shadow setting. Perspective and panorama views
share one lazily created render context and one physical scene snapshot.

For MuJoCo 3.15.0 software rendering, `src/camera_render.cpp` uses a private,
[versioned classic drawing adaptation](../third_party/mujoco_classic/README.md).
It reuses one-light shadow maps within a snapshot and rejects bounded geometry
outside the view, including the near clipping plane, before drawing. Every scene
update invalidates cached shadows; multiple shadow lights render normally, and
reflection draws retain native geometry.
Other MuJoCo versions/renderers use public `mjr_render` directly. This fallback
preserves functionality, not the measured speedup. No physics library or public
MuJoCo symbol is replaced.

Finite planes reuse indexed copies of the original quad grid and omit only
strips wholly outside the view. Primitive order, UVs and depth stay unchanged;
shadow casting uses the complete plane, and reflections/infinite planes retain
native drawing. Before initializing EGL, the core defaults Mesa's software
worker pool to at most four threads (`LP_NUM_THREADS`); an explicit environment
setting takes precedence. Hardware GL drivers do not use this Mesa setting.
CPU LiDAR uses a static AABB tree to skip distant geometry, then calls MuJoCo
ray functions for exact primitive/mesh intersections. Large scans use at most
four workers and join before assembling returns, preserving sample order,
timestamps and values. Small scans remain serial. Curved structured scenes use
CPU ray casting; explicit CUDA rejects unsupported geometry.

Sensor camera rendering runs on one dedicated C++ worker with a private model
and a copied `mjData` snapshot. Physics and IMU/LiDAR scheduling can advance while
that frozen sample renders. There is at most one sample in flight: the next
capture waits if necessary, without overwriting or dropping frames. Publication
stays on the simulation thread and retains the capture timestamp; arrival can
lag the current simulation clock by rendering time. Pause polling publishes
finished samples, and shutdown drains the final captured sample before reporting
counts and timing. This overlaps work rather than changing sensor rates; it adds
one model/data copy in memory. `CameraRig` itself remains synchronous.

The private camera mesh/lighting modules prepare context-owned buffers once.
RGB uses distance-dependent indexed LOD, selecting against a projected half-pixel
**error estimate** with a safety margin; this is not a certified geometric bound
or a bound on shadow/color differences. Original normal/UV attributes are retained.
Shadow casters use a separate 1 mm error estimate. Sensor depth keeps every original
triangle. Physics, LiDAR, model assets and policy inputs never use these buffers.
Opaque, untextured meshes under the supported two-directional-light setup combine
base and shadowed illumination in one draw. Other lighting, materials, transparency
and reflections retain native shading. Small meshes keep native display lists.
See [dependency provenance](../third_party/meshoptimizer/README.md).

透视 RGB／深度保持 320×240，全景 RGB／径向深度保持 720×360，由六个
256×256 视面生成。桌面相机和 MID360 保持仿真时间 10 Hz（每次 20,000
条射线），两组启用的 IMU 均为 200 Hz。软件绘制跟不上时，整个仿真会慢于墙钟。
软件模式保留阴影，将阴影贴图上限设为 2048；输出分辨率、深度精度、多重采样、
物理和传感器时序均不降低。RGB 按距离选择简化网格，使用带余量的半像素投影
误差估计；这不是严格的几何上界，也不保证阴影或颜色差异小于半像素。阴影使用
独立的 1 mm 简化误差估计；深度保留全部原始三角形，物理、雷达和模型资产不变。
常用双方向光下的不透明、无纹理网格合并两次光照绘制，其余情况保留原生着色。
有限地板保留原始四边形、绘制顺序与 UV，只跳过完全不在视野中的条带；阴影仍
使用完整地板。核心在初始化 EGL 前默认将 Mesa 软件渲染线程限制为最多 4 个，
用户指定的 `LP_NUM_THREADS` 优先。大量 CPU 雷达射线也分配到最多 4 个线程，
全部计算完成后按原采样顺序合并，不改变点云、时间戳或扫描频率。
传感器相机使用独立 C++ 线程和私有模型／状态快照，绘制期间允许物理与其他
传感器继续计算。最多保留一帧任务，处理不过来时等待，不覆盖或丢弃采样。
消息仍由主线程发布，保留采集时间戳；到达时间会晚于采集时间，不能当成零延迟
相机。暂停时继续收取已完成的画面，结束前等待最后一帧，代价是多一份模型内存。
透视与全景复用同一渲染资源和物理快照，每次更新
快照都重新计算阴影。固定版本优化只用于 MuJoCo 3.15.0 软件渲染；其他版本或
硬件渲染走原生接口。真实性能和完整 E2E 记录见
[阶段验证](desktop-console-plan.md#cpu-hills-with-noetic--2026-10-22--cpu-起伏地形与-noetic).

The native CI oracle `camera_render_consistency` compares complete RGB and raw
depth buffers against unmodified MuJoCo at the same shadow precision. It covers
changing geometry, lights, camera poses, transparency, reflections, multiple
shadow lights, and tilted/textured/infinite planes. Dense textured/untextured meshes additionally check RGB quality
(mean channel difference ≤1/255 and ≤2% of pixels differing by >10/255) and exact
segmentation/depth output. Runtime artifact parity separately covers robot physics, both
IMUs, rolling/snapshot LiDAR and full 10 Hz D435i output. Performance runs must
pass `--camera-hz 10` explicitly: the standalone CLI's default remains 1 Hz.
`camera_snapshot` compares every captured image/depth frame to synchronous
rendering while live physics advances, and checks worker shutdown and errors.

原生 CI 用原 MuJoCo 在相同阴影精度下逐像素检查 RGB 和深度，包括物体／光源
移动、视角改变、透明、反射和多个投影光源；高密度网格另外检查 RGB 差异限额与
原始深度一致性。运行时对照另外验证机器人、双 IMU、
滚动／快照扫描和 10 Hz D435i。性能测量必须显式指定 `--camera-hz 10`；
独立命令行默认仍为 1 Hz，不能将其结果当作 10 Hz 实时证明。

For an authorized RGB-quality change, artifact comparison can opt into explicit
limits; all non-RGB artifacts still require byte parity. Without these flags RGB
also remains strict:

```bash
.venv/bin/python tools/validate_runtime_parity.py \
  --before /path/to/baseline --after build/software-panorama/simple-rbt-sim \
  --rgb-mae-limit 1 --rgb-outlier-limit .02 --output output/render-parity
```

## Configuration / 配置

The robot registry comes from `simple-rbt-sim --list-robots` without initializing MuJoCo or graphics. The asset catalog supplies model/controller combinations and terrain presets. Wheel chassis allow `flat`, `ramp`, `hills`; Go2/Go2W additionally allow `stairs`, `rough`. Terrain levels are `standard`, `moderate`, `hard`, `extreme`.

机器人能力通过 `simple-rbt-sim --list-robots` 从 C++ 登记表读取，不初始化 MuJoCo 或图形。资源 catalog 提供模型／控制器组合和地形预设。轮式底盘使用 `flat`、`ramp`、`hills`；Go2／Go2W 另可使用 `stairs`、`rough`。难度为 `standard`、`moderate`、`hard`、`extreme`。

The `tracked` mode adds simulator velocity/heading feedback. `native` retains the chassis controller alone. Command limits are conservative simulator limits; speed on terrain is measured rather than guaranteed. Sensor selections independently enable MID360, RGB, depth and panorama.

`tracked` 模式增加模拟器速度／航向反馈；`native` 只使用底盘自身控制器。指令上限是保守的仿真上限，不保证所有地形上的实际速度。MID360、RGB、深度和全景输出可以分别选择。

A chassis sensor mount pose is relative to the robot base, with quaternion order `[w,x,y,z]`. Numeric Euler controls and projected 3D handles write the same pose. MID360's internal IMU is fixed at `[0.011,0.02329,-0.04412]` m in the LiDAR housing frame, with identical orientation. Moving the device preserves that relationship. The controller's body IMU is a different site and is never moved by sensor configuration. Mount editing is available before the first Start.

底盘传感器外参相对机器人 base，四元数顺序为 `[w,x,y,z]`。数值欧拉角与三维手柄修改同一 pose。MID360 内部 IMU 在雷达坐标中固定为 `[0.011,0.02329,-0.04412]` m，方向相同。调整设备时保持这一关系。控制器自身的机身 IMU 是另一感测点，不会被传感器配置移动。外参在首次开始之前编辑。

## Fixed arm and two cameras / 固定机械臂与双相机

Select **Panda · fixed arm** in Robot presets. SpRobo enables a wrist D435 RGB-D
camera and a world-fixed D435 RGB-D camera. Each camera produces 320×240 RGB and
metric Z depth at `camera_hz` (10 Hz by default), with separate optical origins and
CameraInfo. Both render from the same immutable physics snapshot and carry the
capture timestamp. The third-person sensor is distinct from **Observer**, the
free inspection view; moving Observer does not change a sensor's extrinsics.

在机器人预设中选择 **Panda · 固定机械臂**，默认启用腕部和第三人称 D435。
两台相机均输出 320×240 RGB、以米为单位的 Z 深度及各自的 CameraInfo，
默认 10 Hz。同一采样时刻的两路图像使用同一份物理快照。
“第三人称相机”是实际传感器；“自由观察”是操作台视图，移动后者不会修改外参。

The model is pinned to MuJoCo Menagerie's
[Franka Emika Panda](https://github.com/google-deepmind/mujoco_menagerie/tree/cda5236f98383f6fbaa4820c6b18dba892cd087f/franka_emika_panda).
Original fixed-base geometry, inertias, contacts, seven arm servos and the
coupled finger tendon are retained. The C++ `joint_position` controller ramps
arm targets at up to 0.5 rad/s and each finger at 0.04 m/s. It uses CPU physics
and position servos, without an ONNX policy or CUDA dependency.
The model's 2 m visualization extent sets a 1 cm near clip in the generated
scene and keeps shadow precision around the arm workspace. Its centre follows
a fixed-base scene spawn; model export preserves the same placement.

模型、关节限位、惯量、碰撞和夹爪联动来自固定版本的上游 Panda。
C++ 控制器使用位置伺服，关节目标变化限速为 0.5 rad/s，单侧夹指为 0.04 m/s。
该控制器使用 CPU，不需要 GPU。操作台显示角度和夹爪总开口毫米数；
下面的接口使用弧度和单侧夹指位移米数。当前接口执行关节位置目标。

| ROS topic / 话题 | Message / 消息 | Contract / 约定 |
| --- | --- | --- |
| `/panda/joint_position_command` | `sensor_msgs/JointState` | `name` + `position`; partial targets allowed / 可只更新部分关节 |
| `/panda/joint_states` | `sensor_msgs/JointState` | 7 arm joints + both fingers, 50 Hz / 七关节与两侧夹指的实际状态 |
| `/panda/wrist_camera/image_raw` | `sensor_msgs/Image` | `rgb8`, `camera_optical_frame` |
| `/panda/wrist_camera/depth/image_raw` | `sensor_msgs/Image` | `32FC1`, `camera_depth_optical_frame` |
| `/panda/external_camera/image_raw` | `sensor_msgs/Image` | `rgb8`, `external_camera_optical_frame` |
| `/panda/external_camera/depth/image_raw` | `sensor_msgs/Image` | `32FC1`, `external_depth_optical_frame` |

Each camera also publishes `camera_info` and `depth/camera_info` under its prefix.
Joint commands accept `joint1` through `joint7` and `finger_joint1` (0–0.04 m;
total opening 0–80 mm). The second finger follows its physical tendon constraint.
Send targets at about 20 Hz. Unknown, duplicated, nonfinite or out-of-range
targets are rejected as a whole. Timeout (0.5 s), owner disconnect and Stop hold
the current arm pose. Pause freezes time. Window blur affects manual control only.
An invalid ROS target does not terminate the adapter; a subsequent valid target
can recover control.

每个相机前缀下还发布 `camera_info` 和 `depth/camera_info`。
夹爪命令只需设置 `finger_joint1`，另一侧按物理联动运动。
建议 20 Hz 更新目标；未知、重复、非有限数或越界关节会使整条指令被拒绝。
超过 0.5 秒无指令、控制进程断开或点击停止后，机械臂保持当前位置。
手动控制失焦会停止；程序控制不受窗口焦点影响。暂停冻结仿真时间。

Wrist mounts are relative to `arm_tool`; external-camera mounts are in world
coordinates (`odom` in ROS TF). The dynamic `base_link → arm_tool` transform is
published at 50 Hz from the same state as joint feedback. Static transforms
attach wrist optical frames to `arm_tool` and external optical frames to `odom`.
Numeric and drag edits use the selected parent frame. Save a named robot preset
to retain both mounts. Optional D435i IMU output belongs to the wrist camera.

腕部外参相对 `arm_tool`，第三人称相机外参相对世界坐标，ROS TF 中对应 `odom`。
关节运动会更新 `base_link → arm_tool`，两台相机各自的内部 RGB／深度外参保持刚性。
数值和拖拽编辑使用相同父坐标系；保存命名预设后，两套外参一起恢复。
选择 D435i 时，内置 IMU 属于腕部相机。

```bash
# In Noetic, with ROS 1 selected as the command owner in SpRobo:
rostopic pub -r 20 /panda/joint_position_command sensor_msgs/JointState \
  "{name: ['joint1', 'finger_joint1'], position: [0.35, 0.02]}"
# Equivalent Humble command:
ros2 topic pub -r 20 /panda/joint_position_command sensor_msgs/msg/JointState \
  "{name: ['joint1', 'finger_joint1'], position: [0.35, 0.02]}"
```

Native IPC adds joint commands/state and an external-camera stream while
retaining existing packet IDs and chassis velocity topics. ROS remains optional;
all motion and sensor generation stay in the C++ core. `tools/test_manipulator.py`
checks control ownership, deadlines and both depths against independent physical
rays. `tools/test_console_manipulator.py` uses real native input; the ROS matrix
uses `tests/ipc/arm_probe.py` for the arm and the existing chassis probe otherwise.

### Logs and diagnostics / 日志与诊断

Open **Logs** in the top navigation during configuration or operation. Daily
logging is always on; enable **Detailed event tracing** for scan/image events and
IPC/ROS publication diagnostics. High-rate clock and IMU events are summarized
once per writer batch (approximately one second). Sensor, IPC, ROS and control/request tracing can
be enabled independently. Live filters cover the latest 2,000 records / 4 MiB. **Search retained logs**
explicitly scans the whole retained store in the background, keeping the latest
500 matches; the page renders up to 200. These are display limits, not the
retention limit. Refresh returns to live tailing. Export includes all retained
records and a configuration manifest.

配置和运行阶段均可从顶部“日志”进入。日常日志默认开启；打开“详细事件追踪”后，
逐次记录扫描、图像及其 IPC／ROS 发布调用；高频时钟和 IMU 则在每批写入时
（约一秒）汇总。传感器、进程通信、ROS、控制／
请求可分别启用。实时筛选覆盖最近 2,000 条／4 MiB；“搜索全部保留日志”会在后台
扫描整个保留目录，返回最新 500 条匹配，页面显示最多 200 条。刷新可返回实时日志。
这是显示限制，不是存储限制。诊断包导出全部保留记录及配置清单。

The managed store is `$XDG_STATE_HOME/simple-rbt/logs`, normally
`~/.local/state/simple-rbt/logs`. Its **shared limit is 100,000,000 bytes (100 MB)**,
including GUI, core and GUI-managed ROS records. Segments roll at 5 MB; a batch
that would exceed the total limit evicts the oldest managed segments first. Only
`srbt-<timestamp>-<pid>.jsonl` regular files in that directory are managed. Existing
user files, run artifacts and explicit diagnostic exports are not removed.
Exports are separate user-requested ZIP files outside this rolling quota.

受管日志位于 `$XDG_STATE_HOME/simple-rbt/logs`，默认
`~/.local/state/simple-rbt/logs`。GUI、核心及由 GUI 管理的 ROS 记录共用
**100,000,000 字节（100 MB）**上限；每段约 5 MB，写入将超限时先移除最旧的
受管段。仅管理该目录中 `srbt-时间戳-进程号.jsonl` 普通文件，不移除其他用户文件、
实验产物或显式导出的诊断包。导出的 ZIP 是用户主动保存的文件，不计入自动轮转额度。

Each producer has a 2 MiB queue and a background writer. Events enqueue in memory;
writes happen in batches once per second, plus the final shutdown drain. A shared
nonblocking file lock serializes retention and appends across processes. A busy
store defers the batch to the next tick, without falsely reporting an I/O failure.
Queue exhaustion and failed writes are counted;
`logging.dropped` reports discarded records. The log page shows core/GUI writer
health. Disk failure does not block physics; overflowing queues may lose events.
No per-event fsync is used. A hard kill may lose the current buffered second;
power loss can also lose OS-cached writes. A partially written last record is
trimmed on the next append so later JSONL records remain readable.

每个生产者使用 2 MiB 有界队列和后台写入线程／协程。事件先入内存，每秒批量写盘，
正常关闭时再排空最后一批；不会每次扫描都触发文件写入。跨进程使用非阻塞文件锁
串行执行容量清理和追加，繁忙时推迟到下次定时重试，不误报为磁盘写入失败。
队列溢出、写入失败均计数，丢弃以
`logging.dropped` 明示；页面显示核心和 GUI 写入状态。磁盘故障不阻塞物理循环，
但队列耗尽可能丢事件。不逐事件 fsync；强杀可能丢失当前约一秒的缓冲，断电还可能
丢失操作系统缓存。再次写入时修复末尾半条记录，保持后续 JSONL 可读。

Records include schema version, session, writer identity, per-writer sequence,
component, severity, UTC wall-clock nanoseconds, host monotonic nanoseconds,
simulation time and structured fields. A sample is correlated by session plus
`<stream>:<simulation timestamp>`; GUI actions and core requests have their own
IDs. `ipc.offered` means offered to subscriber queues, not delivery confirmation;
`ipc.received` records adapter receipt, and `ros.publish_called` means the ROS
publish API returned, not that a downstream navigation node received it. Queue
coalescing/drops appear in detailed IPC tracing. Text/parameter changes are captured
at action boundaries or one-second checkpoints; pointer motion is coalesced rather
than recorded per pixel. Full point clouds/images remain a separate recording task.

事件包含格式版本、会话、写入者身份、进程内序号、模块、级别、UTC 墙钟纳秒、
宿主单调时钟纳秒、仿真时间及结构化字段。使用会话＋“流名:仿真时间戳”关联传感器
样本；GUI 操作与核心请求另有编号。`ipc.offered` 表示进入订阅者发送队列的尝试，
不代表送达；`ipc.received` 表示适配器已收到，`ros.publish_called` 表示发布 API
已返回，不代表导航节点已接收。详细 IPC 追踪记录合并／丢弃帧。参数和文本修改
在操作边界或一秒检查点记录，不逐像素记录鼠标移动。完整点云／图像另用数据录制。

For `clock`, `imu` and `camera_imu`, `stream.summary` preserves each pipeline
stage separately: count, first/last simulation timestamp, payload byte total,
and duplicate/backwards timestamp counts. Unexpected timestamp repeats/reversals
also produce individual `stream.timestamp_anomaly` warnings, including across
window boundaries. Session changes flush and reset the summaries. Different
processes have independent batch boundaries: compare overlapping simulation time
ranges, not adjacent lines. Summaries cannot reconstruct every sample or measure
per-sample cross-process latency. Sensor data, IPC packets and ROS publications
are unaffected. LiDAR/image events and errors retain their existing detail.

`clock`、`imu`、`camera_imu` 使用 `stream.summary`，分别保留各环节的次数、
首末仿真时间戳、载荷字节总数、时间戳重复／倒退次数。重复与倒退还会单独生成
`stream.timestamp_anomaly` 警告，跨窗口亦可检测；更换会话时排空并重置汇总。
不同进程的批次边界独立，应按重叠的仿真时间范围比较，不能直接对比相邻行。
汇总无法还原每个样本，也不能计算逐样本的跨进程延迟。传感器数据、IPC 数据包和
ROS 发布均不改变；雷达／图像事件及错误保留原有细节。

For a short investigation requiring every clock/IMU trace, launch SpRobo with
`SIMPLE_RBT_TRACE_RAW_HIGH_RATE=1` and enable detailed tracing. The launcher passes
this setting to its core and both managed ROS adapters. For separately launched
processes, set it in each process environment. Restart without this variable to
restore summaries. This override increases log volume; the 100 MB quota and
one-second batched writes still apply.

需要短时间排查每条时钟／IMU 记录时，使用 `SIMPLE_RBT_TRACE_RAW_HIGH_RATE=1`
启动 SpRobo 并开启详细追踪；启动器将此设置传给核心及两种 ROS 适配器。独立启动的
进程需分别设置。移除该变量后重启即可恢复汇总；逐条模式仍遵守 100 MB 上限和
每秒批量写入，但日志增长更快。

The C++ core has no logging dependency on ROS or GUI. Standalone core/adapters use
the same store format; `SIMPLE_RBT_LOG_DIR` overrides their store directory,
`SIMPLE_RBT_TRACE=1` enables detailed tracing and `SIMPLE_RBT_TRACE_MODULES` accepts
comma-separated `sensor,ipc,ros,control` (all by default). GUI-managed adapters emit
batched structured stdout, which the GUI imports without rewriting event identity
or timestamps; they do not require a writable host bind mount for logging. The
core tells adapters the shared session and trace settings through existing IPC
hello/metadata messages. Older adapters still work but have no structured tracing;
prepare updated adapters to obtain the complete chain.

C++ 核心记录日志不依赖 ROS 或 GUI。独立核心／适配器使用相同格式，支持
`SIMPLE_RBT_LOG_DIR` 指定目录、`SIMPLE_RBT_TRACE=1` 开启追踪，以及
`SIMPLE_RBT_TRACE_MODULES=sensor,ipc,ros,control` 选择模块，默认全部。GUI 管理的
适配器批量输出结构化 stdout，由 GUI 保留原始身份和时间导入统一日志，无需为日志
新增可写宿主挂载。核心通过既有 IPC hello／metadata 传递会话和追踪设置。
旧适配器仍可通信，但不提供结构化追踪；完整链路需要准备更新后的适配器。

### Workspace and user scenes / 工作区与用户场景

SpRobo stores versioned robot presets, scenes, drafts and the current configuration
in `$XDG_CONFIG_HOME/simple-rbt/workspace.json` (normally
`~/.config/simple-rbt/workspace.json`). Checkpoints run once per second and at page
changes, session changes and normal close, writing only changed data by atomic
rename. Explicit `--config` preserves the existing library while starting a new
robot preset. Unsupported workspace files are reported and preserved.

SpRobo 在 `$XDG_CONFIG_HOME/simple-rbt/workspace.json`（通常为
`~/.config/simple-rbt/workspace.json`）保存带版本号的机器人预设、场景、草稿及
当前配置。每秒及切页、切会话、正常关闭时检查，数据变化才通过原子重命名写入。
显式使用 `--config` 会新建机器人预设并保留已有资源库；无法识别的工作区文件
会报错并保留原文件。

The current editor also creates [v2 structured scenes](scene-representation.md):
parent groups, nine parameterized primitives, bounded planes with holes and shared
stair templates. The C++ resolver supplies collision geometry and editor previews.
A session embeds that document under `scene`; `scene_spawn` selects a named spawn
when several are present. The v2 budget is 4096 convex parts. Existing v1 scenes
keep their original geometry and format; they are not converted automatically.

当前编辑器可新建 v2 结构化场景。分组、九种基元、有限平面洞口与共享楼梯模板均由
同一 C++ 解析器生成几何；多个出生点通过 `scene_spawn` 选择。v2 上限为 4096 个
凸几何体。已有 v1 场景保持原格式和几何，不会自动改写。

The following box, floor-connection and rectangular-opening fields describe v1.
以下盒体、楼层连接和矩形洞口字段属于 v1。

A session JSON may contain `scene` with `version: 1`, `id`, `name`, `floors`,
`objects` and `spawn`. Floors have `id`, `name`, `height`; objects have `id`, `name`,
`floor`, `kind` (`floor`, `wall`, `ramp`, `stairs`, `obstacle`), `position`, WXYZ `quaternion`
and full `size` in metres. Object Z is relative to its floor elevation. Spawn has
`floor`, local ground-point `position` and `yaw_deg`; nominal robot clearance is
added automatically. Limits are 32 floors, 2048 generated physical boxes, coordinates ±100 m and box
dimensions 0.02–200 m. The C++ core independently validates the scene and replaces
legacy terrain geometry. Omitting `scene` retains existing session compatibility.
Use continuous routes for wheel chassis; the editor does not validate navigability.

The geometry budget counts boxes after stairs and openings expand. It is a
product resource guard, not a MuJoCo or hardware limit or a real-time guarantee.
Scenes exceeding it are rejected explicitly; geometry is never silently truncated.

会话 JSON 可包含 `scene`：`version: 1`、`id`、`name`、`floors`、`objects` 和
`spawn`。楼层包含 `id`、`name`、`height`；物体包含 `id`、`name`、`floor`、
`kind`（`floor`／`wall`／`ramp`／`stairs`／`obstacle`）、`position`、WXYZ `quaternion`
及米制完整尺寸 `size`。物体 Z 相对楼层标高；出生点包含 `floor`、楼层局部地面点
`position` 和 `yaw_deg`，机器人标称离地高度自动添加。上限为 32 层、2048 个展开后的物理盒体，
坐标 ±100 米，物体尺寸 0.02–200 米。C++ 核心独立校验并替换内置地形；省略
`scene` 时兼容原会话。轮式底盘应布置连续路线，编辑器暂不检查路线可通行性。

几何预算按楼梯、洞口展开后的盒体数量计算。它是产品的资源保护值，不是 MuJoCo
或硬件上限，也不保证实时。超限会明确报错，不会静默删减场景物体。

A connected ramp or staircase also has `connection: {"to_floor": "upper-id"}`;
stairs add integer `steps` (1–128). Its pose is the lower-end ground anchor and
yaw only. `size` means horizontal run, clear width and slab thickness. Rise comes
from the two floors; stairs divide run and rise equally among the steps. The UI
couples ramp incline/run and stair target height/count, and displays the actual
step height after integer rounding. Generated geometry is limited to 2048 boxes.
Changing either floor elevation regenerates the connection. Height alignment is
automatic; users still place both ends against floor slabs in XY. Route
traversability is not automatically validated.

连接式坡道／楼梯还包含 `connection: {"to_floor": "上层 ID"}`，楼梯增加整数
`steps`（1–128）。位姿为下端地面锚点及偏航角，`size` 表示水平长度、净宽和
坡板厚度。高差由两层标高决定；楼梯等分长度和高差。界面联动坡度／水平长度、
期望单级高度／级数，并显示整数级数对应的实际单级高度。展开后最多 2048 个
物理盒体；修改层高会重新生成连接件。标高自动对齐，平面位置仍由用户拖动两端
对准楼板；暂不自动校验路线可通行性。

Scene editing supports object deletion, undo/redo (64 operations for the current
scene; history is not persisted across launches), grid/angle snapping, and world
bounding-box alignment: X/Y centres, +X/+Y edge joins and placement on top. A
continuous drag or field edit is one undo operation. Use toolbar buttons or,
with the viewport focused, Delete, Ctrl+Z, Ctrl+Shift+Z / Ctrl+Y. Text inputs keep
their own editing shortcuts. Grid spacing and angle persist in the workspace;
world translation snaps absolute coordinates, local translation snaps the delta,
and rotation snaps the angle delta. “Snap selected” also rounds the current pose.
Saving the scene remains explicit; undo changes the draft, not an earlier save.

场景编辑支持删除物体、撤销／重做（当前场景最多 64 步，关闭后不保存历史）、
网格／角度吸附，以及按世界包围盒 X/Y 居中、贴合 +X/+Y 边和放到顶面的对齐。
连续拖动或同一输入框的一次编辑合并为一步。可用工具栏按钮，或聚焦视图后使用
Delete、Ctrl+Z、Ctrl+Shift+Z／Ctrl+Y；文字输入框保留自己的编辑快捷键。
网格与角度设置保存在工作区；世界平移吸附绝对坐标，局部平移和旋转吸附变化量。
“对齐网格”可直接调整当前位姿。撤销仅修改草稿，更新已保存场景仍需点击保存。

A floor slab accepts one rectangular through-opening:
`opening: {"center": [0, 0], "size": [2, 3]}` in slab-local XY metres. The UI
exposes its centre and dimensions. It expands into up to four solid rim boxes,
so the hole affects collision, LiDAR and rendering. It must remain inside the slab;
multiple openings can be made with separate slabs. This is not arbitrary mesh CSG.

楼板支持一个矩形贯穿洞口：`opening: {"center": [0, 0], "size": [2, 3]}`，
使用楼板局部 XY 米制坐标。界面可调中心和尺寸；核心将楼板拆为最多四块边缘实体，
洞口同时影响碰撞、雷达和渲染。洞口必须位于楼板内；多个洞口可用多块楼板布置，
不支持任意网格布尔运算。

Connections optionally set `turn_deg` to 90, -90 or 180 and `landing_m` to a
platform depth at least equal to the clear width. Zero or absent turn stays
straight. Two flights and a level landing are generated together; turned stairs
need at least two steps. `size[0]` is the total horizontal run of the flights,
excluding the landing. Ramps split the rise equally; stairs split the integer
step count while retaining equal riser height and tread depth. A 180-degree
landing spans both parallel flights (twice the clear width). Rotate and position
the connection as one object; the upper-end marker follows the resulting geometry.

连接件可选 `turn_deg` 为 90、-90 或 180，并设置不小于净宽的平台深度
`landing_m`；省略或为 0 时直行。两段坡道／楼梯和水平转折平台作为整体生成，
转折楼梯至少两级。`size[0]` 为两段水平长度之和，不含平台。坡道等分高差；
楼梯按整数级数分段，保持单级高度和踏步深度一致。180 度平台横跨两个平行梯段，
总宽为两倍净宽。整体移动、旋转时，上端标记随实际几何更新。

### Keyboard driving / 键盘驾驶

In Manual mode, click the joystick or either driving view to give it keyboard focus.
W/S drive forward/back, A/D move left/right, Q/E turn left/right; Space stops.
Releasing a key stops its axis. Leaving the drive control or losing window focus
clears held keys. The manual speed slider applies to both keyboard and joystick
(default 50% of the selected robot's limits). Diagonal translation is normalized.
Unsupported lateral motion and Ackermann in-place turns remain disabled.
Keyboard commands do not take ownership from Program, ROS 1 or ROS 2.

选择手动控制，点击摇杆或驾驶视图后使用键盘：W/S 前后、A/D 左右横移、
Q/E 左右旋转，空格停车。松开按键停止对应方向；焦点离开驾驶控件或窗口失焦时
清除按键状态。手动速度滑块同时作用于键盘和摇杆，默认使用该机器人上限的 50%。
斜向平移会归一化；不支持横移的底盘及阿克曼原地转向仍受限制。
键盘不会抢占程序、ROS 1 或 ROS 2 的控制权。

### Sensor devices / 传感器设备

MID360 uses the official Livox STEP geometry; D435 and D435i share the official
RealSense mesh. The meshes add no mass or collision geometry. Default mounts on
wheel chassis clear the device housings; saved custom mounts remain authoritative.
Select `d435` (RGB/depth) or `d435i` (also an independent 200 Hz IMU) under Camera device.
D435i's IMU remains active when image outputs are disabled. RGB/depth and the
virtual panorama are separate output selections; panorama is a simulator view,
not a physical D435 capability.

MID360 使用 Livox 官方 STEP 外观；D435 与 D435i 共用 RealSense 官方模型。
外观不增加质量或碰撞体，轮式底盘默认安装位置已为设备外壳留出空间，
保存的自定义外参保持优先。在相机型号中选择 `d435` 或 `d435i`；后者增加独立
200 Hz IMU，关闭图像后仍可输出。RGB、深度和虚拟全景分别开关；全景是仿真视图，
不是 D435 的真实硬件能力。

The editable camera mount is `camera_link`, the depth reference in the official
URDF. Nominal local translations in metres are depth `[0,0,0]`, color `[0,0.015,0]`,
and D435i IMU `[-0.01174,-0.00552,0.0051]`. Optical axes are X right, Y down,
Z forward (mount-to-optical quaternion `[0.5,-0.5,0.5,-0.5]`, WXYZ).
RGB and depth render from their own origins; depth is **not registered to RGB**.
The ideal pinhole images use the configured simulator resolution/FOV, without
hardware distortion/noise. These nominal offsets are not an individual camera's
factory calibration. A session profile can supply `camera_extrinsics` with
`color`, `depth`, `imu` poses (`position`, unit `quaternion` in WXYZ) relative to
`camera_link`; the editor still moves only the entire device. See
[model provenance](../assets/visual/sensors/provenance.json) and
[ROS outputs](noetic.md#camera-publication--相机发布).

相机可编辑外参采用官方 URDF 的 `camera_link`，即深度参考点。标称内部平移（米）：
深度 `[0,0,0]`、彩色 `[0,0.015,0]`、D435i IMU `[-0.01174,-0.00552,0.0051]`。
光学坐标为 X 向右、Y 向下、Z 向前；从安装坐标转到光学坐标的 WXYZ 四元数为
`[0.5,-0.5,0.5,-0.5]`。RGB 与深度从各自原点采样，**深度尚未对齐到 RGB**。
图像是使用仿真分辨率／视场角的理想针孔模型，不模拟硬件畸变和噪声。
这些偏移是标称值，不是某台相机的出厂标定。配置文件可通过 `camera_extrinsics`
的 `color`、`depth`、`imu` pose 提供标定，字段为米制 `position` 和 WXYZ 单位
`quaternion`，均相对 `camera_link`；界面仍只允许整体移动设备。

## Core CLI / 核心命令

```sh
simple-rbt-sim --robot ranger_mini --terrain hills --terrain-level moderate \
  --cmd .3 0 0 --sensors full --backend auto --camera-hz 10 --realtime --duration 10
simple-rbt-sim --list-robots
simple-rbt-sim --runtime-info
sprobo --check-dependencies --ros ros1,ros2
simple-rbt-sim --help
```

`--cmd VX VY WZ` uses m/s, m/s and rad/s. `--session-config JSON --session-dir PATH` starts the desktop/ROS session boundary. The config can select independent sensors, initial mounts, command source and finite duration; without a duration the session runs until shutdown. ROS switches are not core launch modes; use a separate adapter.

`--cmd VX VY WZ` 单位为 m/s、m/s、rad/s。`--session-config JSON --session-dir PATH` 启动桌面／ROS 会话。配置可选择各传感器、初始外参、指令来源和有限运行时长；未指定时长则运行到关闭。核心不提供同进程 ROS 启动模式，应启动独立适配器。

MID360 produces 10 Hz scans, up to 20,000 CSV-ordered rays per frame and a synthetic 200 kHz point clock. IMU samples are 200 Hz. Rolling scans interpolate rigid pose at each point timestamp and are not deskewed. Robot self-occlusion is excluded. CUDA supports the current static plane/box terrain. `auto` attempts CUDA and reports a CPU fallback when unavailable; `cpu` is an explicit independent raycasting path. CPU fallback can be slower than real time. `warp` remains a compatibility name for native CUDA.

MID360 为 10 Hz 扫描，每帧最多 20,000 条 CSV 顺序射线，使用合成的 200 kHz 点时钟；IMU 为 200 Hz。rolling 逐点插值刚体位姿，不做去畸变；排除机器人自身遮挡。CUDA 支持当前静态 plane／box 地形。`auto` 尝试 CUDA，无法使用时报告 CPU 回退；`cpu` 是显式的独立射线路径，回退后可能低于实时速度。`warp` 保留为原生 CUDA 的兼容名称。

Both simulated IMUs (MID360 and D435i) measure completed native physics intervals.
Specific force comes from the change in each site's world velocity, including its
rigid lever arm, minus gravity. Angular rate comes from its rotation increment.
An implicit integrator's instantaneous `qacc`/accelerometer and endpoint gyro do
not describe the same averaged motion. At 200 Hz, specific impulses are accumulated
in world axes; the net rotation is measured over that same 5 ms window. Both are
expressed in the sensor frame at the window centre and timestamped there: the
first published sample is **2.5 ms**, available after completing the first 5 ms.
There is no fabricated zero-time IMU sample. This preserves temporal alignment
with the unchanged rolling LiDAR point clock. Native `Simulation::imu()` reports
per-physics-interval measurements at the interval end; the runtime resampler
provides the centred 200 Hz public stream. Neither path injects position or
orientation into an estimator. Controller body sensors and physics are unchanged;
no artificial stationary warm-up is added. Consumers need normal initialization.

MID360 和 D435i 均测量已完成的原生物理区间：加速度来自各自 site 的世界速度
增量（包含刚性安装偏移）减去重力，角速度来自同一区间的旋转增量。隐式积分器的
瞬时 `qacc`／accelerometer 与端点 gyro 不能直接代表同一段平均运动。
200 Hz 输出按世界坐标累计 5 ms 内的比力冲量，并计算同一窗口内的净旋转；两者
都转换到窗口中心的传感器坐标，并使用中心时间戳。首条发布样本为 **2.5 ms**，
完成首个 5 ms 区间后才可用，不虚构零时刻 IMU 样本。雷达滚动扫描点时钟不变。
原生 `Simulation::imu()` 提供物理区间末端标记的区间读数；运行时重采样器负责
输出中心对齐的 200 Hz 数据。两者均不向估计器注入位置或姿态；控制器机身传感器
和物理运动不变，不添加虚构的静止预热数据，使用者仍需正常初始化。


Perspective output is RGB and metric Z-depth; panorama output is RGB and radial depth. Infinite depth means no surface was hit. A black upper image region can be the empty sky in the wall-bounded scene, rather than robot occlusion. The third-person operator view is independently rendered and does not alter sensor extrinsics.

普通相机输出 RGB 和米制 Z 深度，全景输出 RGB 与径向深度；无限深度表示没有命中表面。围墙场景的黑色上方区域可能是空天空，不代表车体遮挡。第三视角独立渲染，不修改传感器外参。

## Build and files / 构建与文件

The installed package carries its runtime SDK libraries and referenced assets with relative library lookup. Development preparation uses `.venv` for locked MuJoCo/ONNX SDKs and a checksum-verified Go toolchain/MyGo source. Source builds need C++17, CMake 3.22+, EGL/OpenGL, GLFW, PNG, TinyXML2, OpenSSL and native GTK3 libraries. CUDA 12.9 is optional: `./setup.sh --native-only --cpu-only` or `-DSIMPLE_RBT_CUDA=OFF` builds without any CUDA runtime link. `--no-desktop` omits the GUI. ROS images compile separately without the simulator SDK.

安装包携带运行 SDK 库与实际引用的资源，并采用相对运行库查找。开发准备使用 `.venv` 提供锁定的 MuJoCo／ONNX SDK，以及经哈希核验的 Go 工具链／MyGo 源码。源码构建需要 C++17、CMake 3.22+、EGL／OpenGL、GLFW、PNG、TinyXML2、OpenSSL 和原生 GTK3 库；CUDA 12.9 可选。`./setup.sh --native-only --cpu-only` 或 `-DSIMPLE_RBT_CUDA=OFF` 构建不链接 CUDA 的版本。可用 `--no-desktop` 省略 GUI；ROS 镜像独立编译，不需要仿真 SDK。

## Preparation and verification / 准备与验证

`tools/requirements-sdk.lock` pins five packages for native SDK distribution and
independent camera checks. Setup does not install a Python simulator, Warp or
an editable reference package. Committed model XML/catalog data remain the runtime
source; asset preparation verifies their pinned inputs. Python tools have these roles:

`tools/requirements-sdk.lock` 仅固定五个 SDK 分发／独立相机检查依赖。
准备过程不安装 Python 仿真器、Warp 或 editable 参考包；已提交的模型 XML／catalog
保持为运行数据来源。保留的 Python 工具分工如下：

| Tools / 工具 | Purpose / 用途 |
| --- | --- |
| `bootstrap_rl_sar_assets.py`, `bootstrap_turtlebot3_assets.py`, `turtlebot3_model.py` | Verify pinned sources and convert Waffle Pi URDF / 核验固定来源、转换 Waffle Pi URDF |
| `prepare_visual_assets.py`, `export_go2w_policy.py` | Offline mesh/policy preparation / 离线网格与策略准备 |
| `setup-gui.py`, `package_deb.py` | Locked GUI dependencies and packaging / 固定 GUI 依赖与打包 |
| `test_console_native.py`, `test_console_workspace.py`, `tests/console/` | Window input, motion, named resources and reopen E2E / 窗口输入、运动、命名资源及重开验收 |
| `test_ros_session.py`, `tests/ipc/ros_probe.py` | One ROS 1/ROS 2 runner observing the C++ core / 双 ROS 共用入口，观测 C++ 核心 |
| `profile_console.py` | CPU/RAM measurements using the same passive session helpers / 复用被动会话工具测量 CPU／RAM |

These tools do not provide another simulator implementation. CTest remains the
owner of native core/sensor/IPC checks; the two E2E runners own observable desktop
and middleware behavior. / 这些工具不实现第二套仿真器；原生核心、传感器与 IPC 由
CTest 验证，桌面与中间件的可观察行为由两个 E2E 入口验证。

## Program control / 程序控制

The operator explicitly selects one command owner: `gui` (manual joystick), `program` (direct IPC), `ros1` or `ros2`. Focus loss clears only manual input. Changing owner clears the old command; a program must continue sending commands within 0.5 seconds. Disconnect, explicit Stop and Pause remain effective in every mode. ROS programs publish `geometry_msgs/Twist` to `/<robot>/cmd_vel`, for example `/turtlebot3_waffle_pi/cmd_vel`.

操作台显式选择一个来源：`gui`（手动摇杆）、`program`（直接 IPC）、`ros1` 或 `ros2`。窗口失焦只清除手动输入。切换来源会清除旧指令；程序需要在 0.5 秒内持续更新速度。所有模式均保留断连、显式停车与暂停。ROS 程序向 `/<robot>/cmd_vel` 发布 `geometry_msgs/Twist`，例如 `/turtlebot3_waffle_pi/cmd_vel`。

For direct C++ integration, connect using the existing neutral IPC client after selecting **Program · IPC**. The same client can subscribe to odometry, IMU and LiDAR. Configuration requests remain owned by the console; a motion program cannot take over configuration implicitly.

直接 C++ 接入时，在界面选中 **Program · IPC** 后使用现有 IPC 客户端；同一个客户端可订阅里程计、IMU 与雷达。配置请求仍由操作台管理，运动程序不能自行抢占配置权限。

```cpp
simple_rbt::ipc::Client client;
client.connect(session_directory, "program", {"odom", "imu", "lidar"});
// Refresh at 20 Hz while active; use poll() to read actual core samples.
client.send_command({0.2, 0.0, 0.0});
client.send_command({0.0, 0.0, 0.0});
client.close();
```

Headless sessions can instead set `"source":"program"` and `"start_immediately":true` in their session JSON. Go2/Go2W policies use the CPU ONNX provider; neither policy inference nor CPU physics requires CUDA. Rendering can use software Mesa, with throughput depending on the host.

无界面会话也可以在 JSON 中设置 `"source":"program"`、`"start_immediately":true`。Go2／Go2W 使用 CPU ONNX 推理，策略执行和 CPU 物理均不要求 CUDA；渲染可使用 Mesa 软件路径，速度取决于宿主性能。

| Module / 模块 | Responsibility / 分工 |
| --- | --- |
| `src/core.cpp`, `src/scene.cpp`, `src/controllers/` | Physics, model assembly, control / 物理、模型装配、控制 |
| `src/lidar*`, `src/cameras.cpp` | Sensor calculation / 传感器计算 |
| `src/runtime.cpp`, `src/session*.cpp` | Execution order, commands, pacing and session state / 执行顺序、命令、节奏与会话状态 |
| `src/runtime_imu.*`, `src/runtime_lidar.*`, `src/runtime_cameras.*` | Per-sensor sampling, buffers, summaries and artifacts / 各传感器采样、缓冲、摘要与产物 |
| `ipc/server.cpp`, `ipc/server_protocol.cpp`, `ipc/server_transport.cpp` | Connections, protocol validation, public transport methods / 连接、协议校验、公共传输方法 |
| `console/session/wire.go`, `console/session/ipc.go` | Wire encoding and client lifetime / 帧编解码与客户端生命周期 |
| `console/` | UI, projected gizmos, input and process lifecycle / 界面、三维手柄、输入与进程生命周期 |
| `wrappers/`, `deployment/` | Independent ROS nodes and images / 独立 ROS 节点与镜像 |
| `tests/`, `tools/` | Preparation and independent validation / 准备与独立验证 |

User settings use `$XDG_CONFIG_HOME/simple-rbt`; state/results use `$XDG_STATE_HOME/simple-rbt` or the standard home-directory defaults. CLI output refuses an existing destination. `--summary-only` keeps the summary and compiled model without step/sensor files. Package removal preserves user data.

设置位于 `$XDG_CONFIG_HOME/simple-rbt`，状态／结果位于 `$XDG_STATE_HOME/simple-rbt` 或对应 home 默认位置。命令行输出拒绝覆盖已有目录。`--summary-only` 保留摘要与编译模型，不保存逐步／传感器文件。卸载保留用户数据。

Sensor implementations own their sample buffers and metadata; adding sensor logic does not require editing the physics/controller loop. IPC framing remains independent of GUI lifecycle and ROS conversion. Camera render contexts are allocated only for views actually rendered. The observer refresh interval starts after a completed frame so slow software rendering cannot monopolize every physics tick. Software RGB/depth/panorama rendering is still not guaranteed to run in real time.

传感器实现各自管理采样缓冲与元数据，修改其内部实现不需要改动物理／控制器循环。IPC 帧编解码独立于 GUI 生命周期及 ROS 转换。相机只为实际渲染的视图创建上下文；观察窗口从一帧完成后重新计算刷新间隔，避免慢帧占满每个物理步。软件 RGB／深度／全景渲染仍不保证实时。

Tests are registered in `tests/CMakeLists.txt`; production modules contain no test registration. `BUILD_TESTING=OFF` omits test executables. CI links the same `simple_rbt_ipc` library through `ros_ipc_fixture`, without maintaining a second source list. Source worker tests use `SIMPLE_RBT_CPP_DATA_ROOT` to avoid accidentally loading an older installed asset tree.

测试集中登记在 `tests/CMakeLists.txt`，生产模块不再夹带测试配置。`BUILD_TESTING=OFF` 不构建测试程序；CI 的 `ros_ipc_fixture` 链接正式 IPC 库，无需同步维护第二份源码清单。源码 worker 测试用 `SIMPLE_RBT_CPP_DATA_ROOT` 显式指定对应资源，避免混入旧安装版本。

```sh
cmake --build build/optimization --target imu_consistency
ctest --test-dir build/optimization -L sensors --output-on-failure
ctest --test-dir build/optimization -L ipc --output-on-failure
python3 tools/validate_runtime_parity.py --before /path/to/baseline/simple-rbt-sim \
  --after build/optimization/simple-rbt-sim --output output/new-runtime-parity
```

Current acceptance is recorded in [console design and validation](desktop-console-plan.md). Earlier dated reports preserve their own source identity and conditions.
当前验收见[操作台设计与验证](desktop-console-plan.md)；此前带日期的记录保留当时的代码身份和测试条件。

## External program contract / 外部程序接入约定

A complete, ROS-free C++ client is provided in [examples/ipc](../examples/ipc/main.cpp).
It builds with CMake and a C++17 compiler; it does not link MuJoCo, ONNX or ROS.
Copy the **current** IPC session path from SpRobo's Program connection panel.
The path changes when the configuration is restarted.

[examples/ipc](../examples/ipc/main.cpp) 提供完整的独立 C++ 客户端，只需 CMake 与 C++17
编译器，不链接 MuJoCo、ONNX 或 ROS。在操作台“程序接入”面板复制当前 IPC 会话路径；
返回配置或重建会话后须重新复制。

```sh
cmake -S examples/ipc -B build/ipc-example
cmake --build build/ipc-example
# Observe for 10 seconds without commanding motion / 仅观测 10 秒
build/ipc-example/simple-rbt-client /absolute/session/path
# Select Program · IPC first; request 0.2 m/s for up to 10 seconds
# 先选择“程序 · IPC”；最多请求运动 10 秒
build/ipc-example/simple-rbt-client /absolute/session/path 0.2
```

| Contract / 约定 | Meaning / 含义 |
| --- | --- |
| Commands / 指令 | `[vx, vy, wz]`, base coordinates, m/s and rad/s; renew at 20 Hz. The core clips supported commands to the robot's limits. / 机身坐标，建议 20 Hz 更新；核心按车型限幅。 |
| Clock / 时钟 | Sensor packet timestamps are integer simulation nanoseconds. ROS peers must use `/clock` and simulation time. Wall-clock command timeout continues while paused. / 传感器时间戳为仿真纳秒；ROS 使用仿真时钟；指令超时按墙钟判断。 |
| Odometry / 里程计 | Simulation ground-truth pose in the world/`odom` frame; linear and angular velocity in the base frame. It is not a noisy wheel encoder or an estimator result. / 位姿为世界／odom 坐标中的仿真真值，速度为机身坐标；不是轮速计或定位估计。 |
| IMU / 惯导 | IPC uses MID360 IMU coordinates, rad/s and m/s² specific force, including stationary gravity response. `/livox/imu_si` preserves SI units; `/livox/imu` is the driver-compatible g-unit stream using the LiDAR frame label. / IPC 与 SI 话题使用国际单位；兼容话题加速度使用 g，坐标标签为雷达。 |
| LiDAR / 雷达 | Points are in the LiDAR frame; `stamp_ns + offsets_ns[i]` gives point time. Scans are rolling and not deskewed. / 雷达坐标，逐点时间为帧起点加偏移，不预先去畸变。 |
| Extrinsics / 外参 | Hello includes base-relative transforms. Quaternions are WXYZ. Camera TF uses optical X-right, Y-down, Z-forward. MID360 IMU is rigidly coupled. / 握手给出机身相对外参，四元数 WXYZ；相机使用光学坐标，MID360 内部刚性耦合。 |
| Ownership / 控制权 | Sending commands does not acquire ownership. Only the selected source can drive. Stop returns control to manual; Pause freezes simulation time. / 发指令不等于取得控制权；停车交回手动，暂停冻结仿真时间。 |
| Delivery / 传输 | Local Unix sockets, protocol 1; no remote TCP endpoint. Drain `poll()` regularly. Queues are bounded: a slow consumer can lose unsent samples or disconnect. / 仅本机 socket，须及时消费；慢接收者可能丢样或断连。 |

Metadata `command_status` distinguishes `waiting`, `receiving`, `timed_out`, `paused`
and `unowned`; `age_s` is nullable and measures time since the core received a valid
command. Connection counts and this heartbeat describe IPC peers. A ROS adapter can
remain connected and send zeros after its upstream publisher stops; neither value
proves a navigation node is publishing. The UI labels commanded and measured velocity
separately, and reports the selected owner above the live views.

`command_status` 区分等待、接收、超时、暂停和无控制来源；可空的 `age_s` 表示核心最近收到
合法指令后的秒数。连接数与心跳只描述 IPC 对端：ROS 上游停止发布后，适配器仍可保持连接并
发送零指令，不能据此认定导航节点仍在发布。界面分别显示指令速度、实际速度与当前控制来源。
