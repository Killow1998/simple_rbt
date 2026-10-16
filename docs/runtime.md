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

## Configuration / 配置

The robot registry comes from `simple-rbt-sim --list-robots` without initializing MuJoCo or graphics. The asset catalog supplies model/controller combinations and terrain presets. Wheel chassis allow `flat`, `ramp`, `hills`; Go2/Go2W additionally allow `stairs`, `rough`. Terrain levels are `standard`, `moderate`, `hard`, `extreme`.

机器人能力通过 `simple-rbt-sim --list-robots` 从 C++ 登记表读取，不初始化 MuJoCo 或图形。资源 catalog 提供模型／控制器组合和地形预设。轮式底盘使用 `flat`、`ramp`、`hills`；Go2／Go2W 另可使用 `stairs`、`rough`。难度为 `standard`、`moderate`、`hard`、`extreme`。

The `tracked` mode adds simulator velocity/heading feedback. `native` retains the chassis controller alone. Command limits are conservative simulator limits; speed on terrain is measured rather than guaranteed. Sensor selections independently enable MID360, RGB, depth and panorama.

`tracked` 模式增加模拟器速度／航向反馈；`native` 只使用底盘自身控制器。指令上限是保守的仿真上限，不保证所有地形上的实际速度。MID360、RGB、深度和全景输出可以分别选择。

A mount pose is relative to the robot base, with quaternion order `[w,x,y,z]`. Numeric Euler controls and projected 3D handles write the same pose. MID360's internal IMU is fixed at `[0.011,0.02329,-0.04412]` m in the LiDAR housing frame, with identical orientation. Moving the device preserves that relationship. The controller's body IMU is a different site and is never moved by sensor configuration. Mount editing is available before the first Start.

外参相对机器人 base，四元数顺序为 `[w,x,y,z]`。数值欧拉角与三维手柄修改同一 pose。MID360 内部 IMU 在雷达坐标中固定为 `[0.011,0.02329,-0.04412]` m，方向相同。调整设备时保持这一关系。控制器自身的机身 IMU 是另一感测点，不会被传感器配置移动。外参在首次开始之前编辑。

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

A session JSON may contain `scene` with `version: 1`, `id`, `name`, `floors`,
`objects` and `spawn`. Floors have `id`, `name`, `height`; objects have `id`, `name`,
`floor`, `kind` (`floor`, `wall`, `ramp`, `stairs`, `obstacle`), `position`, WXYZ `quaternion`
and full `size` in metres. Object Z is relative to its floor elevation. Spawn has
`floor`, local ground-point `position` and `yaw_deg`; nominal robot clearance is
added automatically. Limits are 32 floors, 512 authored objects, coordinates ±100 m and box
dimensions 0.02–200 m. The C++ core independently validates the scene and replaces
legacy terrain geometry. Omitting `scene` retains existing session compatibility.
Use continuous routes for wheel chassis; the editor does not validate navigability.

会话 JSON 可包含 `scene`：`version: 1`、`id`、`name`、`floors`、`objects` 和
`spawn`。楼层包含 `id`、`name`、`height`；物体包含 `id`、`name`、`floor`、
`kind`（`floor`／`wall`／`ramp`／`stairs`／`obstacle`）、`position`、WXYZ `quaternion`
及米制完整尺寸 `size`。物体 Z 相对楼层标高；出生点包含 `floor`、楼层局部地面点
`position` 和 `yaw_deg`，机器人标称离地高度自动添加。上限为 32 层、512 个物体，
坐标 ±100 米，物体尺寸 0.02–200 米。C++ 核心独立校验并替换内置地形；省略
`scene` 时兼容原会话。轮式底盘应布置连续路线，编辑器暂不检查路线可通行性。

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
| `src/runtime.cpp`, `src/session.cpp` | Sampling, scheduling, session state / 采样、调度、会话 |
| `ipc/`, `include/simple_rbt/ipc.hpp` | Neutral wire protocol / 普通进程通信协议 |
| `console/` | UI, projected gizmos, input and process lifecycle / 界面、三维手柄、输入与进程生命周期 |
| `wrappers/`, `deployment/` | Independent ROS nodes and images / 独立 ROS 节点与镜像 |
| `tests/`, `tools/` | Preparation and independent validation / 准备与独立验证 |

User settings use `$XDG_CONFIG_HOME/simple-rbt`; state/results use `$XDG_STATE_HOME/simple-rbt` or the standard home-directory defaults. CLI output refuses an existing destination. `--summary-only` keeps the summary and compiled model without step/sensor files. Package removal preserves user data.

设置位于 `$XDG_CONFIG_HOME/simple-rbt`，状态／结果位于 `$XDG_STATE_HOME/simple-rbt` 或对应 home 默认位置。命令行输出拒绝覆盖已有目录。`--summary-only` 保留摘要与编译模型，不保存逐步／传感器文件。卸载保留用户数据。

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
