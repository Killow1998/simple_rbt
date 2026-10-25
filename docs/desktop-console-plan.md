# Desktop console and session architecture / 操作台与会话架构

Current stage: source-cloud and trajectory review of the two-storey scene,
with native editor checks and a complete upper-corridor Go2W + Noetic route on
Jammy. The current launch command and evidence are at the end of this document.
Framework name remains `simple_rbt`; launcher remains SpRobo. The installed
Panda/two-camera stage and the public [0.2.1 release](https://github.com/Killow1998/simple_rbt/releases/tag/v0.2.1)
are separate from this local scene revision; no new package was published.
/ 当前阶段：双层场景按原始点云与轨迹核对，完成原生编辑器检查及上层 Go2W／Noetic
走廊全路线验证。当前启动命令和证据见文末。本次更新本地场景，未发布新安装包。
Date / 日期：2026-10-25。

## Product / 产品

### Appearance and language / 配色与语言

The console has complete light and dark palettes, including text, secondary text,
control hover/pressed states, focus, selection and semantic status colours.
English and Simplified Chinese are selectable independently of the appearance;
only the chosen language is displayed. Changing either selector leaves the
worker, robot configuration and command owner intact. Preferences are stored in
`$XDG_CONFIG_HOME/simple-rbt-console/appearance.json` (the standard user config
directory when XDG_CONFIG_HOME is unset). Defaults are dark and the system locale.

操作台提供完整的浅色与深色配色，覆盖正文、辅助文字、控件悬停／按下、焦点、
选区和状态颜色。英文与简体中文可独立切换，每次只显示选中的语言。
切换保留 worker、机器人配置及指令来源。偏好存于
`$XDG_CONFIG_HOME/simple-rbt-console/appearance.json`，未设置该变量时使用标准用户
配置目录；初始默认深色，并按系统区域设置选择语言。

The MyGo dependency remains pinned, with verified local patches documented under
[third_party/mygo](../third_party/mygo/README.md). Desktop LCD font settings
exposed normal glyphs being classified as colour emoji, which bypassed the
requested text colour. Software-rendered acceptance had missed this. Acceptance
now includes actual window-pixel text contrast and both languages/appearances,
alongside font-renderer regression checks and the existing motion-control E2E.

MyGo 保持固定版本，本地补丁及校验见上述目录。桌面的 LCD 字体设置曾将普通字形
误判为彩色 emoji，绕过指定文字颜色；此前的软件渲染验收没有暴露该问题。
现在除字体渲染回归与既有运动控制 E2E 外，还直接检查窗口像素的文字对比度，
覆盖两种语言和两套配色。

Install the Ubuntu 22.04 amd64 Debian package and open the native console from the application menu. The frontend uses pinned [MyGo](https://github.com/egoist/mygo), with native Go widgets and no web frontend. The C++ worker owns MuJoCo, controls, sensor sampling and offscreen rendering. Python remains preparation and independent validation only.

安装 Ubuntu 22.04 amd64 的 `.deb`，从应用菜单打开原生操作台。前端使用固定版本的 [MyGo](https://github.com/egoist/mygo) 原生 Go 控件；C++ worker 持有 MuJoCo、控制、传感器采样和离屏渲染。Python 仅用于准备与独立验证。

The configuration screen selects compatible robot/controller/terrain combinations, sensor outputs and mounting poses. Numerical XYZ/Euler controls and translation/rotation handles edit the same normalized quaternion pose. The viewport shows actual core-rendered geometry, with projected manipulation handles. Sensor picking, base/device axes and orbit/zoom affect the selected mount or observer according to their explicit mode.

配置界面选择相容的机器人／控制器／地形、传感器输出和安装位姿。XYZ／欧拉角输入与平移轴／旋转环编辑同一个归一化四元数 pose。视口显示核心实际渲染的模型，叠加投影操作柄；设备选择、机身／设备轴和观察旋转／缩放按各自模式操作。

Starting enters the SpRobo operator screen with first/third views, chassis joystick or arm joint/gripper controls, command-source choice, stop/pause and measured core CPU/RAM/RTF. Joystick release and window focus loss stop manual motion only. Program IPC/ROS commands continue while unfocused, with the same 0.5-second deadman and owner-disconnect stop. Returning to configuration stops the old worker/adapters and starts a fresh private session. User presets and results keep their existing XDG paths; uninstall preserves them.

开始后进入 SpRobo 操作台，提供第一／第三视角、底盘摇杆或机械臂关节与夹爪控制、指令来源、停止／暂停和实测核心 CPU／RAM／RTF。松开摇杆与失焦仅停止手动运动；程序 IPC／ROS 不受窗口焦点影响，仍保留 0.5 秒超时和控制端断开停止。返回配置时关闭旧 worker／适配器，建立新的私有会话。预设与结果保留既有 XDG 路径，卸载保留用户数据。

The launch page combines built-in terrains and user scenes in one selector; terrain
levels are part of the scene name. A separate scene editor supports floor slabs,
walls, ramps, stairs, obstacles, floor elevations and the robot spawn. These objects become
C++ physics and sensor geometry when applied. Arbitrary mesh import and live mount
changes during motion remain outside this implementation. `tracked` feedback and
robot dynamics retain the documented simulator approximations.

启动页使用同一个入口选择内置场景和用户场景，难度包含在名称中。独立的场景编辑页
支持楼板、墙体、坡道、楼梯、障碍物、楼层标高和机器人出生点；应用后成为 C++ 物理及
传感器几何。暂不提供任意网格导入或运动中的外参热修改。`tracked` 反馈和机器人
动力学保留原有、已说明的仿真近似。

## Process boundary / 进程边界

```text
MyGo console (Go) ↔ control/data sockets ↔ MuJoCo worker (C++)
                                          ↕ same protocol
                             ┌────────────┴────────────┐
                        ROS 1 node (C++)         ROS 2 node (C++)
                        Noetic container         Humble container
```

Core, console and either ROS adapter can have independent lifetimes. Each ROS adapter publishes the core's actual samples directly to its peers; neither ROS runtime is linked into the host core/console. Docker images pin public official bases. Selecting both adapters permits both outputs, with one explicit motion-command owner. Command timeout is 0.5 s. Stop returns ownership to manual control and clears chassis velocity or holds the current arm pose; Pause freezes simulation time.

核心、操作台和 ROS 适配器分别拥有生命周期。适配器直接向各自节点发布核心的实际样本；宿主核心／界面不链接 ROS。Docker 镜像固定公开官方基础镜像。可同时启用两种输出，但运动只接受一个显式选中的来源；超时为 0.5 秒。停止后控制权回到手动，底盘速度归零，机械臂保持当前位置；暂停冻结仿真时间。

Control/status use a separate socket from large sensor/view frames. Wire packets have a 24-byte little-endian header: `SRBT`, uint16 version/kind, uint64 simulation timestamp, uint32 payload length and zero reserved uint32. Payloads are explicit scalar/array schemas, never STL layouts or pointers. Per-packet limit is 32 MiB. Private directory/socket permissions are 0700/0600. GUI queues retain latest complete view/camera/odometry frames and never modify partially sent frames. ROS streams retain sample delivery semantics.

控制／状态与大传感器／图像帧分 socket。协议使用 24 字节小端头：`SRBT`、uint16 版本／类型、uint64 仿真时间、uint32 数据长度与值为零的 uint32 保留字段。数据采用明确的标量／数组格式，不传 STL 布局或指针。单包最多 32 MiB；私有目录／socket 权限为 0700／0600。GUI 队列保留最新完整图像／里程计，不修改发送到一半的帧；ROS 保持传感器样本发布语义。

## Rigid sensor device / 刚性传感器

```text
T_base_imu = T_base_mid360 × T_mid360_imu
T_mid360_imu translation = [0.011, 0.02329, -0.04412] m
T_mid360_imu rotation = identity
```

There is one editable MID360 assembly. Its internal IMU follows every translation/rotation and has no independent editor. Model sites, sampling origins and adapter TF use the same composition. The native controller body IMU is separate. Camera installation orientation is distinct from the fixed ROS optical-frame convention.

MID360 只有一个可编辑总成。内部 IMU 随整体平移／旋转，没有独立编辑入口。模型感测点、采样原点和适配器 TF 使用同一组合；原生控制器的机身 IMU 独立。相机安装方向与固定的 ROS 光学坐标约定分开。

## Sensor gizmos / 传感器操作轴

Translation arrows share the device origin, use X/Y/Z colours with dark outlines,
and highlight the hovered or dragged axis. Base/Local transforms retain their
existing meaning. Following [Godot 4.5's manipulator sizing](https://github.com/godotengine/godot/blob/4.5/editor/scene/3d/node_3d_editor_plugin.cpp#L4241),
world scale follows camera depth to keep a roughly 96-DIP handle; short views
reduce it. All overlays are clipped to the displayed image, including labels,
rotation rings and the camera frustum. Invisible handles cannot be picked.

平移箭头共用设备原点，XYZ 分色并带深色描边，悬停／拖动时高亮；机身／设备坐标
语义不变。参考 Godot 的尺寸处理，按观察距离换算操作轴大小，保持约 96 逻辑像素，
小视口进一步缩小。标签、旋转环、相机视锥统一裁剪到实际图像，不进入画面留白或
旁边控件；不可见部分不响应点击。

The former diagonal distortion came from using MyGo's `Painter.Line`, which only
supports horizontal/vertical lines. Projected geometry now uses vector paths;
each ring is stroked as one curve to avoid seams. Native pixel checks verify
connected diagonal shafts, hover and rings in addition to actual mount dragging.
Software-rendered checks cover all image edges, letterboxing and 1×/2× scaling.

此前误用只支持横竖线的 `Painter.Line`，导致斜轴断开、视锥变形；现改为矢量路径，
旋转环整体描边以消除分段接缝。验收检查实际窗口的斜线像素、高亮、旋转环和真实
拖拽；软件渲染另覆盖四边裁剪、画面留白及 1×／2× 显示缩放。

2026-10-15 source validation: [40 native-window checks](../output/gizmo-20261015/final/report.json),
Go tests and vet passed; [translation](../output/gizmo-20261015/final/gizmo-connected.png)
and [rotation](../output/gizmo-20261015/final/gizmo-rotation.png) screenshots inspected.
This is a local source fix; installed 0.2.1 is unchanged. Run `./run.sh --console`
to test it; a release still follows the existing GitHub CI process.

2026-10-15 源码验收：40 项真实窗口检查、Go 测试及 vet 通过，已复查平移／旋转截图。
本机安装的 0.2.1 未替换；用 `./run.sh --console` 测试本次修复，发布沿用既有 GitHub CI。

## Official visuals / 官方外观

Official visual CAD is attached to existing bodies/joints with zero mass and collision masks. Original collision geometry, inertia, wheel dimensions, actuator bounds and controller parameters remain authoritative. Source commits, file hashes, conversions and fitted visual transforms are recorded under `assets/visual/`. Differences between a drawing and the validated physical approximation are recorded rather than silently replacing dynamics.

官方网格以零质量、零碰撞掩码挂在现有 body／joint 上；碰撞体、惯量、轮子尺寸、执行器边界和控制参数仍以原物理模型为准。`assets/visual/` 记录来源 commit、文件哈希、转换和外观对齐；图纸与物理近似的差异明确记录。

## Distribution / 发行

Runtime installs under `/usr/bin`, `/usr/lib/simple-rbt` and `/usr/share/simple-rbt`, with relative SDK library lookup. The package includes referenced model/policy/scan files and notices; it excludes source checkout, preparation environment and unused vendor data. Duplicate large assets share storage. Optional ROS image build context is installed as data, compiled separately when requested. See [package instructions](../packaging/README.md) and [ROS launch](noetic.md).

程序位于 `/usr/bin`，运行库与数据分别位于 `/usr/lib/simple-rbt`、`/usr/share/simple-rbt`；采用相对 SDK 路径。包内包含实际引用的模型／策略／扫描数据与来源文本，不包含源码目录、准备环境或未使用的 vendor 数据；重复大资源共用存储。可选 ROS 构建资料作为数据安装，需要时单独构建。详见[安装说明](../packaging/README.md)与 [ROS 启动](noetic.md)。

## SpRobo 0.2.1 acceptance / SpRobo 0.2.1 验收

Release source / 发行源码：`3ba2ac9ad1b78671f0159d452803cc265fb39c34`.
CI passed CPU/CUDA builds, CTest, Go race/vet, Pango font regression, clean package installation/removal, 24 native-window checks and analytical camera geometry. The native checks cover all four appearance/language combinations, saved preferences after reopening, editing and motion controls; measured text contrast ranges from 7.10:1 to 13.21:1. Noetic passed 140 checks and Humble passed 147 across seven robots each. The ROS matrix excludes camera streams, which have separate core camera acceptance.

CI 通过 CPU／CUDA 构建、CTest、Go 竞争／静态检查、Pango 字体回归、干净安装／卸载、24 项真实窗口检查及解析相机几何。窗口验收覆盖四种配色／语言组合、重开后恢复偏好、外参编辑及运动控制；抽样文字对比度为 7.10:1–13.21:1。七车型的 Noetic 140 项与 Humble 147 项均通过；本轮 ROS 矩阵不包含图像流，相机另由核心验收覆盖。

The published package is 25,162,228 bytes and installs 119,550 KiB. Its SHA256 is `07299205a4e68ce7e9da0f49f72cd70fd5cee8cb97e938808726622cbdaff6c7`. Downloaded bytes agree with GitHub asset digests, the checksum file, package metadata and CI. Jammy now has this exact package; both installed executable hashes match its metadata and startup dependency checks pass.

发布包为 25,162,228 字节，安装内容为 119,550 KiB；实际下载文件与 GitHub 资产摘要、校验文件、打包记录及 CI 一致。Jammy 已安装此正式包，两个可执行文件哈希均符合打包记录，启动依赖检查通过。[Download verification / 下载核验](../output/releases/v0.2.1/verification.json) · [Installed dependency checks / 安装后依赖检查](../output/releases/v0.2.1/installed-dependencies.json)。

The installed release was also inspected on the actual Jammy desktop in dark Simplified Chinese and light English. Both windows displayed the robot, sensor markers and mounting controls with readable text, continued rendering, and closed their worker normally. / 安装后的正式版也在 Jammy 实际桌面检查了深色简体中文与浅色英文：机器人、传感器标记及外参控件正常显示，文字清晰，窗口持续绘制并正常关闭核心。[Host report / 实机报告](../output/ui-contrast-20261015/installed/verified/report.json) · [Dark Chinese / 深色中文](../output/ui-contrast-20261015/installed/verified/dark-zh-CN/actual-window.png) · [Light English / 浅色英文](../output/ui-contrast-20261015/installed/verified/light-en/actual-window.png)。

## SpRobo 0.2.0 stage / SpRobo 0.2.0 阶段

The launcher command and Debian package are `sprobo`; the UI title is SpRobo. Framework code, repository, `simple-rbt-sim`, ROS message names and existing user paths remain unchanged in identity. `packaging/VERSION` supplies the build/release version. Startup reads the loaded SDK and verifies Ubuntu 22.04 amd64, pinned MuJoCo 3.15.0, model XML/policy digests and referenced assets before opening a worker. Optional ROS probes source the actual Noetic/Humble installation inside their containers; a missing ROS adapter does not block standalone CPU physics.

启动命令与 Debian 包名为 `sprobo`，页面名称为 SpRobo；框架代码、仓库、`simple-rbt-sim`、ROS 消息名与既有用户目录保留原有身份。`packaging/VERSION` 统一构建／发行版本。启动先核查 Ubuntu 22.04 amd64、实际加载的 MuJoCo 3.15.0、模型 XML／权重哈希及资源闭包，再打开 worker。可选 ROS 检查在各自容器内 source 实际 Noetic／Humble 安装；缺少 ROS 不妨碍独立 CPU 物理运行。

Local evidence / 本机证据：

- CPU-only and CUDA-capable builds each pass all seven CTest cases; Go race/vet, clean offline Ubuntu installation, seven registered robots, non-root window/close/uninstall, and the extracted package's 14 native-input checks pass. The analytical camera sees the known 2 m wall at 1.99999976 m. / 纯 CPU 与可选 CUDA 构建各通过七项 CTest；Go 竞争／静态检查、离线干净 Ubuntu 安装、七车型、普通用户窗口／关闭／卸载及提取安装包的 14 项原生输入验收通过。相机对已知 2 米墙面的深度为 1.99999976 米。[Native input / 原生输入](../output/ci/native-input/report.json), [Camera / 相机](../output/ci/camera/analytic-check.json)。
- Native keyboard/pointer checks: 14/14, including manual focus-loss stop, program focus-loss continuation (0.322 m), and program silence timeout. / 真实键鼠 14/14，覆盖手动失焦停车、程序失焦继续运动 0.322 米及程序静默超时。[Result / 结果](../output/sprobo-validation-20261014/native-input/report.json)。
- Noetic/Humble × Go2/Go2W: four CPU-only/no-camera cases passed, with 20 checks per Noetic case and 21 per Humble case. CustomMsg columns/timing agree with PointCloud2 point by point; Humble uses a separate motion observer while the complete sensor receiver stays active. / 两种 ROS、两台多关节机器人均通过纯 CPU／无相机测试；Noetic 每例 20 项，Humble 每例 21 项。CustomMsg 与 PointCloud2 逐点对应；Humble 运动观测与完整传感器接收分进程，避免测试程序漏采里程计。[Noetic](../output/ros-cpu-validation-20261014/noetic-cpu-only-go2-go2w/summary.json), [Humble](../output/ros-cpu-validation-20261014/humble-cpu-only-independent/summary.json)。
- TurtleBot3 Waffle Pi replaces the displayed generic differential-drive reference. Official ROBOTIS URDF/meshes and link mass/inertia are pinned at `90a68bd2e3c61c12966779da89d8eeaec82730e9`. Wheel radius is 0.033 m; URDF wheel centers give 0.288 m track while the official controller uses nominal 0.287 m. Its 1 ms physics step stabilizes the ideal wheel-velocity servo; this remains a simulator actuator model. / TurtleBot3 Waffle Pi 替换正常界面的差速参考车，采用固定版本官方 URDF／网格与各 link 质量／惯量；轮半径 0.033 米、URDF 几何轮距 0.288 米、官方控制 nominal 轮距 0.287 米。理想轮速执行器采用 1 ms 物理步长，不作为硬件电机性能声明。[Manifest / 参数来源](../assets/manifests/turtlebot3_waffle_pi_manifest.json)。
- TurtleBot3 native/tracked forward/reverse, two-way turns, curves and stops, plus flat, 8° ramp and 2.5 cm continuous hills: eight groups passed; no torso contact, maximum ramp tilt 8.14°. / TurtleBot3 两种模式运动与连续地形共八组通过，无机身碰地，坡上最大倾角 8.14°。[Result / 结果](../output/ros-cpu-validation-20261014/turtlebot3-e2e.json)。

The GitHub workflow builds CPU-only and CUDA-capable cores, runs CTest and Go race tests, validates the installed package and actual native inputs, checks analytical RGB/depth, then checks both independent ROS versions with all registered robots. Release publication depends on every job succeeding and preserves existing published versions. / GitHub 流程先执行双构建、CTest 与 Go 竞争检查，再验收安装后的程序、真实窗口输入及解析 RGB／深度，随后两种 ROS 对全部登记车型验收；全部成功后才发布，已发布版本不覆盖。[Workflow / 流程](../.github/workflows/build-release.yml)。

Published 0.2.0 source / 0.2.0 发行源码：`aafd0f008c23487a202d31b85b9b1a4152f8625a`. Online acceptance passed both builds, seven CTest cases per build, Go race/vet, clean installation/removal, all 14 native-input checks and analytic camera geometry, then all seven robots in Noetic and Humble: 14 cases, 287 checks. The native test uses keyboard focus and actual window geometry across font configurations. / 线上通过双构建、各七项 CTest、Go 竞争／静态检查、干净安装／卸载、14 项原生输入和解析相机几何，随后七车型在 Noetic／Humble 中共 14 个案例、287 项检查全部通过。原生输入测试按键盘焦点与实际窗口几何定位，适应不同字体布局。

The published Debian package is 25,153,764 bytes; installed content is 119,514 KiB. Downloaded release bytes, the checksum file, package metadata and CI agree on SHA256 `94ab8b989805ec324236dc23828455813635f9da77d506453719366d0fee9561`. / 发布包为 25,153,764 字节，安装内容为 119,514 KiB；实际下载文件、校验文件、打包元数据和 CI 的 SHA256 一致。[Download verification / 下载核验](../output/releases/v0.2.0/verification.json)。

## Prior validated baseline / 此前已验收基线

The following evidence and resource measurements describe the earlier 0.1 package and recorded executable hashes. They do not identify the new 0.2 release artifact. / 以下证据及资源测量对应此前 0.1 安装包与所记程序哈希，不代表新的 0.2 发行产物。

- CTest: 5/5, including controller braking, reference comparisons and IPC. Go session tests pass with the race detector, including stop while a real ROS publisher continues sending commands. / CTest 5/5；Go 会话通过竞争检测，覆盖真实 ROS 持续发速度指令时的停车。[IPC and ROS evidence / IPC 与 ROS 证据](../output/ipc-validation-20261014/validation-final.json)。
- Official Noetic and Humble containers: seven robots × 18 checks × two ROS versions = 252 passing checks. Each node talks directly to the host C++ core. / 官方 Noetic 与 Humble 容器：七车型、各 18 项、两种 ROS 共 252 项通过；节点直接连接宿主 C++ 核心。[ROS 1](../output/ipc-validation-20261014/official-ros1-final/summary.json), [ROS 2](../output/ipc-validation-20261014/official-ros2-final-receiver/summary.json)。
- Native X11 keyboard/pointer acceptance: 12/12. Rapid robot changes retain one latest worker; numerical edits, translation handles and rotation rings reach the core; MID360 stays rigid; joystick motion, release and actual focus loss work; closing leaves no owned worker. Screenshots read actual OS window pixels. / 真实 X11 键鼠验收 12/12，覆盖快速切换车型、数值与拖动外参、MID360 刚性、物理运动、松手／失焦停车及退出；截图读取操作系统窗口像素。[Result / 结果](../output/desktop-console-20261014/native-x11-os-final/report.json), [operator screenshot / 操作画面](../output/desktop-console-20261014/native-x11-os-final/native-window.png)。
- Four official visual overlays: 790 physical parameter checks and 600 simulation steps per chassis; qpos/qvel differences are exactly zero. / 四台官方网格接入后，790 项物理参数检查及每台 600 步仿真均通过，位置／速度差值为零。[Result / 结果](../output/visual-validation-20261014/dynamics-parity.json)。
- C++ IPC latest-frame test: 12,050,850 bytes/s payload, 250 ms GUI consumption, 16 complete view/camera/odometry groups over 4 s; maximum frame age 0.10688 s. / C++ IPC 完整尺寸负载测试：最大帧龄 0.10688 秒。[Result / 结果](../output/ipc-validation-20261014/slow-viewer.json)。
- Final package installs in a clean public Ubuntu 22.04 image without a source checkout, Python environment or host ROS SDK. Seven robots load, the native window opens as a non-root user, normal close leaves zero workers, and uninstall preserves user configuration. Two builds at the same epoch have identical hashes. / 最终包通过干净 Ubuntu 安装、普通用户真实窗口、七车型启动、正常关闭与卸载保留配置；相同 epoch 的两次构建哈希一致。[Result / 结果](../output/package-validation-20261014/validation-final.json), [installed screenshot / 安装后画面](../output/package-validation-20261014/ubuntu-mygo-os/installed-native-console.png)。

These results identify the tested executable hashes in their JSON records. The final package SHA-256 is `28b660a7dacc350c2c3190580bf7ec11ee6e99e165884488988618677d500c11`, built with `SOURCE_DATE_EPOCH=1791983008`.
上述 JSON 标明实际测试的程序哈希；最终包的 SHA-256 与构建 epoch 如上，不自动证明后续修改。

## Resources / 资源消耗

Twenty-second isolated runs on Jammy use Ranger Mini on flat ground, official CAD, CUDA LiDAR (20,000 rays at 10 Hz), 200 Hz IMU, RGB/Z-depth and panorama at 10 Hz, and live first/third views. CPU 100% means one logical core. RSS sums can count shared pages twice; PSS apportions them. These measurements include the console, worker and optional ROS processes, but exclude the X server, measurement tools, Docker daemon and external ROS consumers.

Jammy 上独立运行 20 秒：Ranger Mini 平地运动、官方网格、CUDA 雷达（20,000 线／10 Hz）、200 Hz IMU、10 Hz RGB／深度／全景与双视角。CPU 的 100% 表示一个逻辑核；RSS 可能重复计入共享页，PSS 按共享比例分摊。统计包含界面、核心及可选 ROS 进程，不含 X server、测量工具、Docker daemon 与外部 ROS 消费节点。

| Running processes / 运行进程 | Mean CPU / 平均 CPU | Peak RSS / 峰值 RSS | Peak PSS / 峰值 PSS | RTF / 实时倍率 |
| --- | ---: | ---: | ---: | ---: |
| Core + MyGo / 核心＋界面 | 24.1% | 534.8 MiB | 479.0 MiB | 0.992 |
| Core + MyGo + Noetic + Humble / 再加双 ROS | 32.6% | 688.5 MiB | 605.4 MiB | 0.994 |

Raw samples: [core + GUI / 核心＋界面](../output/console-resources-20261014/core-gui/report.json), [both adapters / 双适配器](../output/console-resources-20261014/core-gui-ros-final/report.json). RTF is sampled from simulation timestamps; this short flat-terrain run is not a worst-case terrain/resource guarantee.

原始样本见上述链接。RTF 根据仿真时间戳采样；短时平地测量不代表所有地形的最坏资源上限。

The `.deb` is 22.6 MiB and installs 106.3 MiB. The optional ROS images occupy about 2.90 GiB combined, predominantly their pinned official bases; they are separate from the `.deb`. Asset hard links save about 53 MiB. / `.deb` 为 22.6 MiB，安装后 106.3 MiB；可选双 ROS 镜像合计约 2.90 GiB，主要来自固定的官方基础镜像，不计入安装包。资源硬链接节省约 53 MiB。[Package sizes / 安装大小](../output/packages/simple-rbt_0.1.0_amd64.json), [ROS image sizes / 镜像大小](../output/ipc-validation-20261014/ros-image-footprint.json)。

Ubuntu 24.04 has not been validated. Both prepared official ROS images and the installed image-build context passed acceptance; first-use preparation through the GUI from an empty Docker daemon was not verified in this managed proxy environment.

Ubuntu 24.04 尚未验收。已准备的官方 ROS 镜像和安装后的镜像构建资料均通过测试；在当前代理环境中，尚未验证空 Docker daemon 下由 GUI 首次准备镜像的全过程。

## Interface usability stage — 2026-10-15 / 接口易用性阶段

The next integration target is **Go2W + ROS 1 Noetic**. This stage prepares that
boundary; no SLAM or navigation stack is included. See [ROS ownership and launch](noetic.md)
and the [external program contract](runtime.md#external-program-contract--外部程序接入约定).

下一阶段固定为 **Go2W＋ROS 1 Noetic**。本阶段补齐接口与操作体验，尚未接入建图导航栈。
接入约定、真值隔离和运行示例见上述文档。

- Both ROS adapters can suppress dynamic odometry TF and publish truth on a separate
  topic; static sensor TF remains available. Old adapter images are detected through
  runtime capability inspection and require rebuilding.
- Profiles are validated before replacing a running configuration: robot/controller,
  terrain, motion/backend, camera rate, mounts and odometry topic. Original profiles
  without the new ROS settings retain their defaults.
- The console distinguishes command receipt from IPC connectivity and displays command
  versus actual velocity. Command topic and session path can be copied. Robot limits
  are explained. Stop/Pause remain fixed at the bottom of the sidebar while scrolling.
- A standalone C++ example subscribes to real samples and optionally commands motion;
  it builds without the simulation or ROS SDK.

两种适配器支持关闭动态 TF、独立发布真值，并识别旧镜像能力；配置在替换当前会话前校验。
操作台区分指令与实际速度、连接与接收状态，可复制接入路径／话题，显示底盘限制，固定停车／暂停
按钮。独立 C++ 示例无需仿真或 ROS SDK，直接消费实际数据并可选发送指令。

Local evidence is retained in `output/interface-ux-20261015/`: CTest 7/7; Go tests and
vet; 26 native-window checks; Go2W CPU Noetic checks 21/21 with default TF and 22/22
with external odometry; Humble compatibility checks 22/22 and 23/23 respectively.
ROS checks exclude cameras and measured real-time factor 1.000 in these cases. Native
window checks include rendering, sensors, all four theme/language combinations and
actual focus/command behavior. A separate 1024×768 Chinese light-theme run checks the
fixed Pause/Resume/Stop controls and captures actual windows. An invalid startup profile
was corrected through the UI without launching a worker first. The C++ example received
odometry, IMU and scans and observed approximately 0.20 m/s under a 0.2 m/s command.

本机证据保存在上述目录：CTest 7/7、Go 测试与 vet、26 项真实窗口检查；Go2W CPU 模式下，
Noetic 默认／外部里程计分别通过 21／22 项，Humble 对应 22／23 项。ROS 用例关闭相机，实测
实时倍率均为 1.000；窗口检查覆盖渲染、传感器、四组配色语言和真实输入行为。另以 1024×768
浅色中文界面检查固定暂停／继续／停车控件，并验证无效启动配置不会启动核心且可在界面中修正。
C++ 示例接收到真实里程计、IMU 和扫描，0.2 m/s
指令下观测速度约 0.20 m/s。

The local Docker Hub manifest request timed out. Adapter evidence uses freshly compiled
source inside existing official-image-based runtime containers; the Noetic message
header was generated from its installed TF definition. Production Dockerfiles are
unchanged. CI now includes the Go2W external-odometry case for both ROS versions, but
this stage's CI/release has not run. Installed SpRobo 0.2.1 remains unchanged.

本机 Docker Hub manifest 请求超时；适配器使用既有官方镜像基础容器重新编译当前源码，Noetic
缺少的消息头由容器内 TF 定义生成，正式 Dockerfile 未改动。CI 已加入双 ROS 的 Go2W 外部
里程计用例，本阶段尚未运行远端 CI／发布，已安装的 SpRobo 0.2.1 未替换。


## Keyboard and sensor devices — 2026-10-15 / 键盘与传感器阶段

Completed locally: physical W/S/A/D/Q/E input, Space stop, shared manual speed
limits, official MID360/D435 meshes, optional D435i 200 Hz IMU, rigid internal
calibration, and separate RGB/depth optical sampling and ROS frames. Configuration,
units and nominal calibration limits are maintained in [runtime usage](runtime.md)
and [ROS camera outputs](noetic.md#camera-publication--相机发布).

本机已完成键盘驾驶、空格停车、共用手动限速、官方 MID360／D435 外观、可选
D435i 200 Hz IMU、刚性内部标定，以及独立 RGB／深度采样和 ROS 坐标。配置、单位、
标称外参的适用范围统一维护在上述运行及 ROS 文档中。

`src/sensor_devices.cpp` owns device geometry and rigid child frames; the core
samples motion, the renderer accepts separate optical poses, and both ROS nodes
convert IPC data. IPC protocol 1 adds `CameraImu=14` on `camera_imu`: the same
six float64 values as IMU (gyro rad/s, acceleration m/s²), with simulation time in
the header. Hello includes separate camera/depth/IMU extrinsics and presence flags.
Clients should subscribe only to channels they understand. Updated ROS adapters
advertise the camera-device capability before the console launches them.

设备外观和内部坐标由 `src/sensor_devices.cpp` 管理，核心采样、渲染器使用不同
光学原点、双 ROS 节点转换 IPC。协议 1 增加 `CameraImu=14`／`camera_imu`，数据
格式与 IMU 相同：六个 float64（rad/s、m/s²），仿真时间在包头；Hello 增加内部
外参和存在标记。客户端应只订阅自己支持的频道；启动前检查 ROS 节点的新能力。

Two regression lessons are covered by observable checks. MuJoCo's compiled
`site_sameframe` optimization must be invalidated when moving a site: changing
its quaternion alone can leave the actual sensor direction unchanged. Official
STEP files can contain open shells as well as solids; importing only solids
silently omitted the MID360 side housing. Validation therefore compares compiled
world vertices against source-device bounds, rotates sampling frames, reloads a
saved scene, and inspects close-up renders. CAD colors and transparency are visual
choices; sensor models do not add mass or collisions.

此次发现并修复两类问题：修改外参时需要清除 MuJoCo 的 `site_sameframe` 缓存，
否则数值变化而实际采样方向可能不变；STEP 同时含实体与开放壳面，只导入实体会
漏掉 MID360 侧壳。验证覆盖编译后世界坐标顶点、旋转后采样方向、场景保存重载和
放大渲染。CAD 材质是渲染选择；传感器外观不增加质量或碰撞。

Evidence / 证据（`output/keyboard-sensors-20261015/`）：

- [CTest](../output/keyboard-sensors-20261015/ctest-final.log): 8/8; Go tests/vet pass.
  [Device rendering](../output/keyboard-sensors-20261015/devices-final.log) and
  [Mesa software rendering](../output/keyboard-sensors-20261015/devices-software.log)
  verify independent RGB/depth origins, independent IMU, rigid mounts and saved geometry.
- [Native window](../output/keyboard-sensors-20261015/ui-acceptance/report.json):
  Go2W keyboard/joystick motion, release/Space/focus stop, program ownership, all
  four theme/language combinations, and the selected D435i device.
- [Noetic](../output/keyboard-sensors-20261015/noetic-final/go2w/report.json): 37/37;
  [Humble](../output/keyboard-sensors-20261015/humble-final/go2w/report.json): 38/38.
  Both use Go2W, CPU policy/LiDAR, GPU image rendering, MID360, D435i IMU, RGB,
  depth and panorama. Twenty-second runs measured RTF 0.999, core CPU 0.22 logical
  cores and peak core RSS about 586 MiB. This excludes GUI/ROS processes and is
  not a worst-case resource bound.
- [CPU/IMU-only Noetic](../output/keyboard-sensors-20261015/noetic-imu-cpu/go2w/report.json):
  26/26 with CUDA hidden and no image renderer, including external odometry and
  camera IMU 5 ms sampling; RTF 1.000, 0.13 CPU cores and 485 MiB peak core RSS.
- New device meshes total 1,933,783 bytes. The locally verified development `.deb`
  is about 24.4 MiB, installed size 119.1 MiB, with no runtime Python.

上述结果包含 8 项 CTest、Go 测试／vet、真实窗口操作、双 ROS 全传感器，以及禁用
图像与隐藏 CUDA 后的 Noetic 独立 IMU 验收。资源数据只统计核心进程，不能替代整机
最坏负载测量。新增设备网格约 1.84 MiB，本地开发安装包约 24.4 MiB／安装后 119.1 MiB。

CI now includes the D435i external-odometry case and software-rendered device
checks. This stage is a locally verified source change; no new GitHub CI/release
has run, and installed SpRobo 0.2.1 remains unchanged. Try the source build with
`./run.sh --console`. Next release must use the normal CI acceptance before
installation; Go2W + Noetic remains the SLAM/navigation integration target.

CI 已加入 D435i 外部里程计及软件渲染检查。本阶段为本机已验证的源码改动，尚未
运行新一轮 GitHub CI／发布，系统安装的 SpRobo 0.2.1 未替换；可用
`./run.sh --console` 测试源码版。新发布仍须通过既有 CI 验收；后续建图导航目标
保持 Go2W＋Noetic。


## Named presets and scene workspace — 2026-10-16 / 命名预设与场景工作区

Source implementation; the installed 0.2.1 package remains unchanged. Run
`./run.sh --console` to try this stage. One window now separates Launch, Robot
presets, Scene editor and Runtime. Robot presets keep a stable ID when renamed,
can be saved as another preset, and contain robot/controller/sensor mounts rather
than a map. Workspace checkpoints retain drafts and selected resources on reopen.
The launch page has one robot-preset selector without a redundant model caption,
and one scene selector, e.g. `Ramp · Hard`, alongside user-named scenes.

本阶段为源码实现，系统安装的 0.2.1 未替换；使用 `./run.sh --console` 测试。
同一窗口分为启动、机器人预设、场景编辑和运行操作台。预设允许用户命名、改名和
另存为；改名不改变身份，机器人／控制器／传感器外参与地图分开保存。工作区检查点
保留草稿和当前选择，重新打开即可恢复。启动页不再重复显示预设对应的型号；
内置与自定义场景共用一个选择入口，例如“坡道 · 高难”。

Scene editing has a floor/object tree, numerical properties and a clipped 3D view
with translation axes and rotation rings. Right-drag orbits, Shift + right-drag
pans, and the wheel zooms. Objects can be copied; floor hiding changes the editor
only. The two-floor template has a 3 m elevation difference and a continuous ramp.
Preview uses a cached CPU raster capped at 900 pixels on its longest side;
geometry/camera changes invalidate it. Applying a scene restarts the single owned
worker while the native window retains the previous view, explicitly marked as
loading, until the first new frame arrives.

场景页提供楼层／物体树、数值属性和有边界裁剪的三维视图，支持平移轴、旋转环和
物体复制。右键环绕，Shift＋右键平移观察位置，滚轮缩放；隐藏楼层仅影响编辑视图。
双层模板标高差为 3 米，使用连续坡道连接。编辑预览复用 CPU 栅格图，最长边限制为
900 像素，仅在几何或视角改变时重绘。应用场景会重启唯一的自有仿真进程；原生窗口
保留上一幅画面并标明加载状态，直到新进程输出首帧。

Local acceptance: CTest 9/9, Go tests and vet, native input 40/40, workspace
save/reopen/window continuity 14/14. Scene widget tests cover real rendered geometry,
translation, rotation and visual-only floor hiding. C++ acceptance moves Go2W and
TurtleBot3 on the upper floor and verifies MID360 floor/ceiling returns. This does
not establish that every chassis can climb every user-authored slope. The workspace
E2E is also wired into installed-package CI; a new GitHub run/release is pending.
Evidence: `output/pages-scenes-20261016/`.

本地验收：CTest 9/9、Go 测试与 vet、真实窗口输入 40/40、工作区保存／重开／窗口
连续性 14/14。场景控件测试覆盖真实渲染几何、平移、旋转及仅影响显示的楼层隐藏。
C++ 验收覆盖 Go2W 和 TurtleBot3 的二层运动，以及 MID360 对地板和天花板的扫描。
这些结果不代表每台底盘都能爬过任意用户坡道。工作区 E2E 已接入安装包 CI，尚未
运行新一轮 GitHub CI 或发布。证据目录：`output/pages-scenes-20261016/`。


Floor-connection properties / 楼层连接参数：ramps expose lower/upper floors,
clear width, horizontal run, incline and slab thickness; stairs expose floors,
width, tread depth, step count and target step height. A 3 m rise with a 0.18 m
target becomes 17 steps of approximately 0.1765 m. Actual geometry follows the
floor elevations, including after floor edits. C++ ray tests check every tread,
width boundaries and ramp heights along the slope. UI tests edit the requested
step height and slope angle through the property widgets. XY end placement
remains manual; see [scene schema and limits](runtime.md#workspace-and-user-scenes--工作区与用户场景).

连接坡道可设置上下层、净宽、水平长度、坡度及坡板厚度；楼梯可设置上下层、净宽、
踏步深度、级数和期望单级高度。例如 3 米高差、期望 0.18 米，得到 17 级、每级约
0.1765 米。几何始终依据楼层标高生成，修改层高后会更新。C++ 射线验证逐级踏面、
净宽边界及坡面高度；控件测试实际输入单级高度与坡度。两端平面位置仍需手动摆放。


## Scene editing and resource acceptance — 2026-10-16 / 场景编辑与资源验收

The scene page now supports deleting objects, undo/redo, configurable translation
and rotation snapping, and object alignment. Undo groups a continuous drag or
field edit and restores the selection; history is bounded to 64 operations and
is local to the current scene/session. Saved scenes remain separate from drafts.
Floor slabs support rectangular through-openings. Connections can generate two
flights with left/right 90° or 180° return landings, including integer stair-step
splits and exact upper-floor elevation. See [usage and schema](runtime.md#workspace-and-user-scenes--工作区与用户场景).

场景页已支持删除物体、撤销／重做、平移／旋转吸附和物体对齐。连续拖动或同一
输入框的编辑合并为一步，撤销恢复选择；历史最多 64 步，仅属于当前场景／会话。
保存的场景与草稿分离。楼板支持矩形贯穿洞口；连接件可自动生成左／右 90 度及
180 度回转平台，按整数楼梯级数分段，并精确接上层标高。洞口与平台同时进入
C++ 碰撞、雷达和渲染几何。平面布置和机器人路线可通行性仍由用户检查。

Validation includes native-window save/reopen and scene-switch continuity,
delete/undo/redo and turn selection; widget tests cover grouped snapped gestures,
keyboard shortcuts and opening controls; C++ rays check opening rims, rotated
openings and turned-flight/landing heights. Resource profiling uses a separate
four-storey, 242-object fixture with partitions, furniture, openings, ramps and
stairs. It is a workload fixture, not a proof of autonomous inter-floor navigation.

验收覆盖真实窗口保存／重开／切换连续性、删除／撤销／重做及转折选择；控件验收
覆盖吸附拖动合并、快捷键和洞口开关；C++ 射线检查洞口边缘、旋转洞口、转折梯段
及平台高度。资源测试使用独立四层、242 个物体的场景，包含隔墙、家具、洞口、
坡道和楼梯；该场景用于负载测量，不代表已验证自主跨楼层导航。

Reproduce with `python3 tools/profile_scene.py --display DISPLAY --output NEW_DIR
--image NOETIC_IMAGE`, then use `--flat` with a different output directory for the
baseline. Use a normally built standalone Noetic adapter image; the tool does not
depend on a particular host's container configuration. It uses isolated XDG data,
a private test master and an owned disposable container, with read-only real ROS
subscribers. Python is measurement/verification code only.

复现使用 `python3 tools/profile_scene.py --display DISPLAY --output NEW_DIR
--image NOETIC_IMAGE`；加 `--flat` 并换输出目录可跑平地对照。镜像使用常规构建的
独立 Noetic adapter，不依赖某台机器的专用容器配置。测试使用隔离的 XDG 数据、
独立测试 master、自有临时容器及只读 ROS 接收节点；Python 仅用于测量和验证。

Both 60-second runs use Go2W, CPU policy/physics/LiDAR (20,000 points at 10 Hz),
MID360 and D435i IMUs, RGB/depth/panorama at 10 Hz, live operator views and the
independent Noetic adapter. Ten ROS topics have actual receivers, including
CustomMsg, PointCloud2 and panorama RGB/depth. The native GUI ran under Xvfb with
software GL; the core used host-default EGL. CPU 100% means one logical core;
means are weighted by sample intervals. Totals include GUI/core/ROS, excluding
the X server, Docker daemon, measurement code, external receiver and GPU memory.

两组各运行 60 秒：Go2W、CPU 策略／物理／雷达（20,000 点／10 Hz）、MID360 与
D435i 的 IMU、10 Hz RGB／深度／全景、实时操控视图，以及独立 Noetic adapter。
十个话题均有实际接收者，包括 CustomMsg、PointCloud2 和全景 RGB／深度。原生
GUI 在 Xvfb 下使用软件 GL，核心使用宿主默认 EGL。CPU 100% 为一个逻辑核，均值
按采样间隔加权。总量包括 GUI／核心／ROS，不含 X server、Docker daemon、测量
程序、外部接收器和显存。

| Scenario / 场景 | Mean CPU / 平均 CPU | Runtime peak CPU / 运行峰值 CPU | Peak RSS / 峰值 RSS | Peak PSS / 峰值 PSS | RTF |
|---|---:|---:|---:|---:|---:|
| Flat / 平地 | 54.8% | 64.1% | 952.1 MiB | 850.5 MiB | 0.997 |
| 4 floors, 242 objects / 四层 242 物体 | 89.8% | 96.8% | 962.6 MiB | 862.6 MiB | 0.998 |

For the complex case, GUI/core/ROS mean CPU is 18.2% / 66.6% / 5.0%; individual
peak RSS is 145.0 / 672.3 / 146.0 MiB. The separate receiver uses 4.5% mean CPU
and 55.2 MiB peak RSS. Orbiting the scene editor uses 44.7% mean GUI CPU; five
seconds after orbiting average 3.2%. Sampled startup peak for GUI+core is 174%
CPU. These are sampled peaks, not worst-case guarantees or an endurance test;
phase-boundary samples can include activity immediately before the transition.
The main increase over flat ground is core CPU, while runtime peak PSS rises by
about 12 MiB. This does not establish performance for all scenes or hardware.

复杂场景 GUI／核心／ROS 平均 CPU 分别为 18.2%／66.6%／5.0%，各自峰值 RSS
为 145.0／672.3／146.0 MiB；独立接收器平均 CPU 4.5%、峰值 RSS 55.2 MiB。
环绕编辑场景时 GUI 平均 CPU 44.7%，停止操作后的五秒平均为 3.2%。GUI＋核心
启动采样峰值 CPU 为 174%。以上为采样峰值，非最坏情况保证或耐久测试，阶段边界
样本可能包含切换前的活动。相对平地，主要增加核心 CPU，运行峰值 PSS 约增加
12 MiB；不代表任意场景或硬件上的性能。

Evidence / 证据：[complex / 复杂场景](../output/editor-resources-20261016/complex-final/report.json),
[flat / 平地](../output/editor-resources-20261016/flat-final/report.json),
[native workspace / 原生工作区](../output/editor-resources-20261016/workspace-appearance/report.json).
Reports retain raw samples, binary hashes, image digest and ROS reception counts.
Local CTest passes 9/9; Go tests and vet pass; native workspace checks pass 17/17.
This is a source/local-build stage; the installed 0.2.1 package and published
release remain unchanged. Run `./run.sh --console` from the repository to test.

报告保留原始采样、程序哈希、镜像摘要与 ROS 消息计数。本地 CTest 9/9、Go 测试
与 vet、真实窗口工作区 17/17 通过。本阶段为源码及本机构建，已安装的 0.2.1 与
发布版本未替换；在仓库执行 `./run.sh --console` 测试。


## Bounded event diagnostics — 2026-10-16 / 有界事件诊断

Implemented a ROS-independent C++ event sink and compatible Go collector/store,
with a shared 100 MB rolling quota, 5 MB segments, 2 MiB producer queues and
one-second background batching. The GUI imports batched records from its owned
ROS adapters; standalone adapters can write their own configured store. The
existing IPC protocol carries session and module trace settings, without adding a
ROS dependency to the core. See [operation and limits](runtime.md#logs-and-diagnostics--日志与诊断).

已实现不依赖 ROS 的 C++ 事件记录器及兼容的 Go 收集／存储模块：共用 100 MB
滚动额度、约 5 MB 分段、每生产者 2 MiB 队列、每秒后台批量写入。GUI 导入其拥有的
ROS 适配器批量记录；独立适配器可写入指定目录。既有 IPC 传递会话及模块追踪设置，
核心仍不依赖 ROS。操作入口、格式及限制见上述运行文档。

Coverage includes GUI button actions and coalesced parameter changes; session
configuration/load/start/pause/source requests and errors; changed motion
commands and deadman state; per-scan/frame/IMU generation; IPC queue offers,
adapter receipt and ROS publish-call completion; process exit and resource
samples. Session/sample IDs connect the sensor chain; action/request IDs connect
UI intent to core handling. Logs do not prove downstream ROS subscriber receipt,
and they do not record full sensor payloads or every physics substep.

覆盖按钮操作与合并后的参数修改、配置／加载／启动／暂停／控制来源请求及错误、
变化的运动指令与超时状态、逐扫描／图像／IMU 生成、IPC 入队尝试、适配器接收与
ROS 发布调用完成、进程退出和资源采样。会话／样本编号关联传感器链路，操作／请求
编号关联前端意图和核心处理。不以发布调用冒充下游接收，也不记录完整传感器载荷或
每个物理子步。

The dedicated diagnostics page supports live filtering, explicit full-retention
search, module trace switches, writer health and ZIP export with configuration.
Its light/dark and English/Simplified-Chinese native windows were inspected.
The existing short in-memory details pane remains a preview; persisted retention
is byte-based. Normal shutdown allows 1.5 seconds before termination escalation:
the previous 0.5-second grace could interrupt renderer cleanup before the final
core log batch. Tests now verify final records after closing the native window.
File-lock contention during exports is treated as deferred work, not an I/O fault.

独立诊断页支持实时筛选、显式搜索全部保留日志、分模块追踪、写入健康状态及带配置的
ZIP 导出，已检查中英文和深浅色实际窗口。旧的短内存详情栏仅作预览，持久化保留按
字节容量计算。正常关闭的退出宽限改为 1.5 秒：此前 0.5 秒可能在渲染器清理期间
终止进程，丢失核心最后一批日志；现已验收关窗后的末尾记录。导出期间文件锁繁忙
按推迟批次处理，不误报 I/O 故障。

Local validation: CTest 10/10; Go race tests and vet; existing native workspace
checks 17/17; Noetic and Humble native diagnostics E2E. Real adapter publication
records correlate with core-generated scans; rejected requests and adapter
stops are persisted; export contains configuration and logs; normal shutdown
records survive; concurrent core/GUI writers produce valid JSON and unique IDs.
Store tests exercise the actual 100,000,000-byte threshold, unrelated-file
preservation, overflow markers, write failures, partial-tail recovery and old-event
search beyond the live tail.

本地验收：CTest 10/10、Go 并发检查与 vet、原生工作区 17/17、Noetic 与 Humble
诊断端到端测试。真实适配器发布记录可关联到核心扫描；错误请求、节点停止可持久化；
导出含配置和日志；正常退出记录保留；并发写入保持有效 JSON 和唯一事件编号。
存储测试覆盖实际 100,000,000 字节阈值、保留无关文件、溢出标记、写入失败、
末尾半条记录修复，以及超出实时尾部范围的旧事件搜索。

The public Docker build attempt timed out querying Docker Hub's pinned base
manifest. Both updated wrappers were subsequently built and run offline against
existing Noetic/Humble test environments (including the existing generated
Noetic TF message headers); isolated `diagnostics-test` tags preserve installed
images. Public Dockerfiles and packaged build contexts include the new diagnostics
sources. This validates local adapters, not a new clean-network CI/release build.
The installed package and published release are unchanged.

公共 Docker 构建在 Docker Hub 固定基础镜像摘要查询处超时，随后使用已有 Noetic／
Humble 测试环境离线编译并运行更新节点（Noetic 使用已有生成的 TF 消息头）。
独立 `diagnostics-test` 标签保留原安装镜像；公共 Dockerfile 与安装包构建闭包均已
加入诊断源码。本次证明本地适配器可构建运行，不代表已通过新一轮干净网络 CI／发布。
系统安装包与发布版本未替换。

### Clock/IMU trace coalescing — 2026-10-16 / 时钟与 IMU 追踪合并

The full-trace baseline produced 61,724,475 bytes during the one-minute Go2W /
four-floor / Noetic experiment (including startup and shutdown). Clock accounted
for 45.7%, camera IMU 24.8%, and MID360 IMU 24.0%. These were repeated pipeline
metadata records, not recorded point-cloud/image payloads.

此前一分钟 Go2W／四层场景／Noetic 全量追踪产生日志 61,724,475 字节（包含启停）：
时钟占 45.7%，相机 IMU 占 24.8%，MID360 IMU 占 24.0%。主要是同一样本经过
不同环节时重复写入的追踪元数据，不是完整点云或图像。

Clock and both IMU streams now emit bounded, approximately one-second summaries
per process and stage. Counts and first/last simulation timestamps remain;
duplicate/backwards timestamps also emit individual warnings. Session changes
flush and isolate summaries; shutdown drains the final partial window. This is
aggregation, not lossless compression: per-sample timing is available only with
`SIMPLE_RBT_TRACE_RAW_HIGH_RATE=1`. The override reaches both managed ROS adapters.
Sensor outputs, communication frequency, LiDAR/image tracing, errors, the shared
100 MB quota and batch write frequency are unchanged. See the runtime logging
section for the exact record fields and cross-process window limits.

现在按进程及环节，每约一秒汇总时钟和两套 IMU；保留次数、首末仿真时间戳，重复／
倒退另记警告。切换会话会排空并隔离汇总，正常退出保存不足一秒的最后窗口。
这是合并统计，并非无损压缩；逐样本时间分析需显式启用上述环境变量，启动器会传递
给两种 ROS 适配器。传感器输出和通信频率、雷达／图像追踪、错误日志、100 MB
共同上限和批量写入频率均不改变。字段和跨进程窗口限制见运行文档。

The first native run exposed an existing shutdown defect: the store's function-
static regex could be destroyed before the process-static logger's final flush,
causing a vector allocation exception and losing the final batch despite exit 0.
The filename matcher now has no destructible static state. A child-process test
waits for a timer flush, exits normally, then verifies both final user and stop
records. This specifically covers process teardown, which a local logger unit
test does not exercise.

首轮实测发现已有退出缺陷：存储模块的函数静态正则对象可能先于进程日志器销毁，
最后一批写入因此抛异常，进程即使以 0 退出仍丢失尾部日志。文件名匹配已改为无
可析构静态状态的实现；子进程测试等待定时写入后正常退出，再检查末尾事件和停止
事件。只测试局部日志对象的析构不足以覆盖这种真实退出顺序。

Validation / 验证：

- C++ diagnostics and trace-window tests: 2/2, including real process shutdown,
  exact aggregation counts, session boundaries, anomaly visibility and raw mode.
  Go session/store race tests passed.
- Noetic one-minute native measurement: 3,654,680 bytes total (94.08% below the
  full-trace baseline), including 293,130 bytes of clock/IMU summaries. All three
  streams have matching offer/receipt/publication totals; zero log drops and
  successful final drain. RTF 0.9990; runtime mean CPU 92.51% of one logical core,
  peak PSS 846.88 MiB. Measurements cover this scenario, not worst-case bounds.
- Humble native diagnostics: 11/11, including clock/both-IMU stage summaries,
  unchanged LiDAR correlation, export and final shutdown records.
- Both wrappers rebuilt offline in the existing Noetic/Humble test environments
  with isolated `diagnostics-coalesced` image tags. Installed package/images and
  releases remain unchanged.

C++ 两项测试覆盖真实退出、精确计数、会话隔离、异常和逐条开关；Go 会话／存储
并发测试通过。Noetic 一分钟实测总量 3.65 MB（减少 94.08%），其中时钟和 IMU
约 0.293 MB；三个环节计数一致、日志无丢弃、关停排空成功，RTF 0.9990。
整组运行平均占用约 0.925 个逻辑核，峰值 PSS 846.88 MiB。这是该场景的测量，
不是最坏情况保证。Humble 原生端到端 11/11；两种节点使用独立测试镜像重新编译。
系统安装包、用户原有镜像和发布版本未替换。

Evidence / 证据：
[Noetic resource and event counts](../output/diagnostics-20261016/profile-coalesced-final/report.json),
[Humble native checks](../output/diagnostics-20261016/native-humble-coalesced/report.json),
[full-trace baseline](../output/diagnostics-20261016/profile-trace/report.json).

## IMU integration consistency candidate — 2026-10-18 / IMU 积分一致性候选修复

Status: native sensor regression checks pass. Independent full-route acquisition
and mapping validation now **pass** with the unchanged sensor candidate and the
corrected FAST_LIO configuration. This validates the tested simulated input, not
relocalization, half-core CPU operation, or hardware. The candidate remains at
`build/sensor-consistency/simple-rbt-sim`; installed binaries and releases are
unchanged. Earlier uncommitted console/diagnostics work is preserved.

状态：原生传感器回归通过；原候选二进制配合修正后的 FAST_LIO 配置，已通过独立
全路线采集与建图验收。结论仅适用于本次仿真输入，不代表重定位、半核 CPU 或真机
验收通过。候选版本仍使用独立构建目录，未替换桌面安装或发布版本；此前未提交的
界面／日志修改保留。

The native implicit step updates velocity differently from its instantaneous
accelerometer output. Reading the latter after `mj_forward` and labelling it as
an interval measurement produced nonconservative IMU integration. Endpoint gyro
samples also introduced a half-step phase error during sharp motion. The sensor
module now derives specific impulse from each site's native velocity increment
and rotation from its quaternion increment. The runtime accumulates impulses
across physics ticks, then emits both channels in the centre frame of the same
5 ms interval. MID360 and D435i share this implementation. Controller body sensors,
physics integration and rolling LiDAR geometry are unchanged. See
[IMU timing](runtime.md) and MuJoCo's
[integration model](https://mujoco.readthedocs.io/en/stable/computation/#numerical-integration).

隐式积分后的速度变化与重新计算的瞬时 accelerometer 不等价；直接积分后者会
积累错误。剧烈运动中，端点 gyro 也产生半步相位偏差。现分别使用原生 site 的速度
增量和四元数增量计算同一区间的惯性读数，运行时跨物理步累计冲量，统一到 5 ms
窗口中心的坐标和时间。MID360、D435i 共用实现，控制器机身传感器、物理积分和
滚动雷达几何不变。首条公开样本从 0 改为 2.5 ms，之后每 5 ms 一条；首条在
物理时间 5 ms 才可用。停止时不虚构未来 IMU，最后一帧雷达可能仍等待下一条惯性
样本；这与丢失原始消息是不同的边界行为。

Verified / 已验证：

- CTest 12/12. The new regression fails against the old core and passes against
  the candidate, including offset/rotated MID360 and D435i, start/stop/reverse/turn,
  native rotational increments, and 1/2/5/7 ms to 200 Hz resampling.
- Frozen 140 s physics replay: pose and site velocity element differences from
  the original recording are exactly zero. Native interval integration residuals
  are about 1e-11 m/s or smaller; this is an interval-sum check, not a claim that
  endpoint trapezoidal integration is identical.
- Centred 200 Hz replay, initialized once at 7.0025 s: gyro-only attitude maximum
  error 0.0112 degrees; acceleration integrated using native orientation has
  4.1 mm maximum position error; using only integrated gyro for orientation has
  0.317 m maximum position error. These diagnostic truth comparisons are offline;
  truth is not supplied to FAST_LIO.
- The final live CPU route ran 140 s with starts/stops/turns on isolated ROS domain
  193: 1,400 scans and 28,000 raw IMU messages, all captured. Replaying the actual
  applied commands reproduces all 1,400 truth poses and all 28,000 serialized IMU
  accelerometer/gyro records with maximum element difference zero. Only the audit
  observer subscribes to ground truth.
- Independent authored-box ray intersections on 12 sampled live scans / 29,856
  points differ by at most 0.029 mm using native 200 Hz pose history. This checks
  the sampled static scene, not every possible model or moving obstacle.
- At the initial handoff, FAST_LIO failed the existing 1 m / 5 degree gross-drift limits: maximum
  position error 46.05 m, maximum attitude error 37.37 degrees (first-pose-only
  fixed alignment; 1,396 evaluated poses). Data-completeness checks passing does
  not qualify mapping. No FAST_LIO/n3m parameters or thresholds were modified.

本轮测试覆盖安装偏移、旋转轴、启停／倒车／转弯和不同物理步长；旧核心不能通过
新增回归。冻结 140 秒轨迹的原生位姿和速度逐元素保持一致；原生区间积分残差约
1e-11 m/s。中心采样的独立惯导验证中，陀螺积分姿态最大误差约 0.0112°，全惯导
最大位置误差约 0.317 m；使用真值姿态的离线分项检查约 4.1 mm，不能冒充估计器
效果。闭环录制完整，原生重放与 1,400 个真值及 28,000 条线上 IMU 完全一致。
抽查的滚动点云几何最大偏差约 0.029 mm。初次交接时 FAST_LIO 最大位置误差
46.05 m、姿态误差 37.37°，超过原有 1 m／5° 门槛；当时未认定剩余漂移的唯一
原因。ROS 命令到达时间在不同 live 运行间有差异，不把这些 live 运行当作同物理
输入严格 A/B。原失败记录保留；后续同输入隔离及全路线结果如下。

Evidence / 证据：
[CTest](../output/sensor-consistency-20261018/ctest-final.log),
[native centred audit](../output/sensor-consistency-20261018/centred_native140_audit.json),
[exact live wire reproduction](../output/sensor-consistency-20261018/live_wire_audit.json),
[rolling scan geometry](../output/sensor-consistency-20261018/rolling_live_audit.json),
[raw acquisition](../output/sensor-consistency-20261018/live_centred140/summary.json),
[failed mapping score](../output/sensor-consistency-20261018/live_centred140_score.json).


### Independent mapping revalidation — 2026-10-18 / 独立建图复验

Closed result files from the n3m integration run were inspected: all acquisition
and mapping checks pass, with 133/133 waypoints, 1,272.64 m travelled and
1,379.97 s simulated. Only the audit observer subscribes to ground truth. Using
fixed first-pose alignment, the unchanged 1 m / 5 degree gross-drift limits give:

| Output / 输出 | Evaluated poses / 位姿数 | Maximum position error / 最大位置误差 | Maximum rotation error / 最大姿态误差 |
| --- | ---: | ---: | ---: |
| FAST_LIO | 13,796 | 0.81234 m | 1.25612° |
| n3m | 13,724 | 0.84024 m | 1.20299° |

The integration owner isolated the earlier 140 s failure on the same closed bag
and unchanged FAST_LIO binary: their collection tool ran the node directly with
YAML and omitted the standard launch override. With `det_range=100`, the fallback
`cube_side_length=200` and `MOV_THRESHOLD=1.5` caused repeated map-window pruning
even while stationary. Explicit YAML value 500 reduced the maximum errors from
46.047 m / 37.370 degrees to 0.31708 m / 0.33839 degrees (1,396 poses; corrected
score inspected). The standard MID360 launch already sets 1000; this finding is
not a defect attributed to that standard hardware launch. The integration owner
reports the YAML correction in FAST_LIO humble commit
`e0a02d53701c3bac72c952fe399f024d3ebb2987`; the launch still overrides it with 1000.

已核对关闭后的验收文件：133/133 路点、1,272.64 m、1,379.97 s，采集与建图均通过；
真值仅供验收观察器使用。上表使用固定首帧对齐，沿用原 1 m／5° 粗检查门槛。
此前 140 秒剩余漂移由接入方在同一 bag、同一 FAST_LIO 二进制上隔离为采集工具
遗漏 launch 参数覆盖：YAML 回落为 200，设为 500 后最大误差降至
0.31708 m／0.33839°。标准 launch 原本设置 1000，不能据此指认真机标准 launch
存在同一缺陷。传感器一致性修复具有独立验证依据，不再为原失败数值盲改 IMU。

Next: retain the candidate for release integration; independent relocalization
validation remains with n3m. This result does not qualify relocalization,
half-core performance, other simulated inputs, or hardware. No candidate was
published and no desktop installation was replaced as part of this revalidation.

后续：保留候选供发布集成，n3m 继续独立重定位测试；重定位、半核性能、其他仿真
输入及真机尚未由本结果验收。本次复验没有发布候选或替换桌面安装。

Local closed evidence / 本机已关闭证据：
[acquisition summary](/tmp/n3m_sprobo_20261018/varied_corrected_coverage/summary.json),
[mapping quality](/tmp/n3m_sprobo_20261018/varied_corrected_coverage/mapping_quality.json),
[completed route](/tmp/n3m_sprobo_20261018/varied_corrected_coverage/route.json),
[recorded bag](/tmp/n3m_sprobo_20261018/varied_corrected_coverage/bag/bag_0.db3),
[corrected 140 s score](/tmp/n3m_sprobo_20261018/centred_cube500_140/score.json).


## Runtime consolidation and ablation — 2026-10-21 / 运行代码整理与消融

The existing diagnostics and n3m-validated IMU candidate are integrated with a
focused refactor. Runtime features, controller laws, sensor resolution/rates and
ROS wire formats are preserved. This stage updates source; the installed desktop
and release version remain 0.2.1. Earlier uncommitted work was snapshotted before
editing and is included rather than discarded.

本轮整合此前日志系统与 n3m 已验证的 IMU 修复，优化运行代码，不增加运行功能。
控制律、传感器分辨率／频率及 ROS 消息格式保持一致。开始前保存了全部未提交修改，
本轮纳入代码提交；本机桌面安装及发布版本仍为 0.2.1。

### Boundaries and measured removals / 模块边界与实际消融

- `runtime.cpp`: 390 → 234 lines. IMU, LiDAR and camera streams own their sampling,
  buffers, metadata and artifacts in separate private modules. Runtime retains
  command/step order and pacing. An actual incremental build after touching only
  `runtime_imu.cpp` rebuilt only that object, then relinked the runtime/executable.
- IPC server: the 425-line file becomes connection lifecycle (179 lines), protocol
  validation (123) and public transport methods (149), with one private state
  declaration. Go framing moves out of the client lifecycle (315 → 242 lines).
  This reduces the scope of edits; it is not a claim of fewer total source lines.
- Removed obsolete native IMU address copies. A non-session CLI no longer writes
  the same model twice: syscall tracing confirms 2 → 1 model-file truncations.
  Trace-disabled high-rate paths avoid constructing diagnostic JSON.
- Camera render contexts are created only when their view is used. Two interleaved
  before/after trials of Go2W observer-only software rendering (904×304, three
  frames, fresh process) reduced peak RSS from 826.10/826.47 MiB to 646.20/646.90
  MiB. RGB hashes are identical. This is a rendering-process measurement, not
  total desktop RAM or a general CPU speedup claim.
- Slow observer frames previously restarted the next frame immediately and
  starved physics. Refresh timing now starts at frame completion. The original
  real-window test failed before this fix and passes all 40 checks afterward,
  without reducing rendering resolution or relaxing its acceptance criteria.
- Rejected ablation: the 200-line in-memory process-log view still feeds the
  toolbar and failure diagnostics; it is retained alongside the separate 100 MB
  persistent store. Removing the IMU centre timestamp in an isolated mutant is
  caught by the existing consistency test, so that behavioral regression remains.
- Test registration moves into `tests/CMakeLists.txt`, respecting `BUILD_TESTING`.
  CI links `ros_ipc_fixture` against the production IPC target instead of copying
  its source list. Production-only configuration excludes test targets. Source
  worker tests explicitly select matching assets, avoiding the installed 0.2.1
  tree. The stale D435i endpoint-timestamp assertion is corrected to the verified
  2.5 ms centre phase; both IMUs now have independent ROS clock checks.

调度、传感器、IPC 协议和连接生命周期分别归属明确模块；主调度文件缩至 234 行，
IPC 三个实现文件均不足 180 行。删除了失效缓存、重复写盘和关闭追踪时的无用
JSON 构造；按需创建渲染资源，在同图像消融中节省约 180 MiB。修复慢帧连续重绘
导致物理推进受阻的问题。测试集中管理并复用正式构建依赖，保留实际能攔截缺陷的
时间戳、控制权、消息及 GUI 验收。接口变化仍需对应的端到端检查。

### Acceptance / 验收

| Check / 检查 | Result / 结果 |
| --- | --- |
| C++ CTest | 12/12 |
| Go race + vet | 34 top-level tests, zero skips; real C++ worker/IPC and Noetic ownership switch included |
| Runtime before/after | 10 cases: seven robots, rolling/snapshot LiDAR, full D435i cameras; all deterministic artifacts byte-identical; only process/resource/timing summary fields excluded |
| Frozen live 140 s | 28,000 native pose/velocity/IMU rows byte-identical to the n3m sensor candidate |
| Native X11 input | 40/40: themes/languages, gizmos, keyboard/joystick, ownership, focus-stop, shutdown |
| Named workspace E2E | 17/17: presets, scene edits, undo/redo, visible scene switching, reopen |
| GUI → core → Noetic diagnostics | 11/11: correlated events, high-rate summaries, failure/disconnect, export and 100 MB retention |
| ROS 1 / ROS 2 robot matrix | Seven robots each pass motion/MID360/namespace checks, without camera rendering |
| Final Go2W + D435i IMU, external odometry | Noetic 27/27; Humble 28/28, including centred 200 Hz timing and truth-topic isolation |

The full RGB/depth/panorama Ranger probes pass every message/motion assertion
under both ROS versions, but the overall 20 s collection runs time out in software
rendering: only 18.2/18.8 simulated seconds in 53.3/52.6 wall seconds. The Go2W
full-panorama case is slower. These runs are retained as **failed overall E2E**;
no timeout, frequency or precision threshold was relaxed. Software full-panorama
real-time performance remains unqualified. The tested native GUI is functional;
its software-rendered Go2W driving run was about 0.34 real-time factor. No GPU
performance, n3m relocalization or hardware claim is added by this refactor.

Ranger 完整图像消息探针在双 ROS 下的消息／运动断言均通过，但纯软件全景渲染
使整段 20 秒采集超时，因此保留为整体 E2E 失败；Go2W 全景更慢。未放宽超时、
频率或精度条件。软件全景实时性能仍未通过验收；本轮 GUI 软件渲染驾驶约为
0.34 倍实时。现有功能操作已通过，不能把这解释为完整图像负载已实时。GPU 性能、
n3m 重定位和真机不属于本轮新增验证结论。

Evidence is under `output/code-optimization-20261021/` (local, ignored by Git):
[summary](../output/code-optimization-20261021/validation-summary.json),
[artifact parity](../output/code-optimization-20261021/runtime-parity-render/report.json),
[frozen 140 s](../output/code-optimization-20261021/frozen140-parity.json),
[RAM ablation](../output/code-optimization-20261021/camera-resource-ablation.json),
[module build isolation](../output/code-optimization-20261021/module-isolation.json),
[IMU mutation](../output/code-optimization-20261021/imu-ablation.json),
[model writes](../output/code-optimization-20261021/model-write-ablation.json).

This stage was pushed as `b3a86db`. The software panorama follow-up is recorded below; installation remains a separate deployment step.

本阶段已以 `b3a86db` 推送。后续软件全景优化见下；源码更新与本机安装分别记录。


## Software panorama throughput / 软件全景吞吐

Verified 2026-10-21, CPU MuJoCo 3.15.0 and llvmpipe. Xian authorized lower
shadow precision while retaining shadows. No output dimensions, sensor rates,
geometry, physics, controller or ROS interface were reduced or replaced.

2026-10-21 验证，使用 CPU MuJoCo 3.15.0 和 llvmpipe。Xian 允许降低阴影精度，
要求保留阴影。透视 320×240、全景 720×360／六面 256×256，相机与雷达 10 Hz、
每扫描 20,000 条射线、双 IMU 各 200 Hz 均保留。

### Changes / 修改

- Share the perspective/cube render context and build one scene per physical
  sample. Mesh uploads, framebuffer allocation and scene rebuilding are no
  longer duplicated for the sensor views.
- The [private classic drawing adapter](../third_party/mujoco_classic/README.md)
  reuses one-light shadows within that sample and culls bounded geometry outside
  the view. It retains native draw order, depth, transparency, reflections and
  multisampling. Cache lifetime ends on every scene update. Other versions or
  hardware renderers use the public native renderer.
- Software shadow maps are capped at 2048; smaller model settings remain intact.
  Removed the accidental unused renderer locals in panorama lookup construction.
  No asynchronous physics/render state or additional rendering thread pool is
  introduced. Parallel-view, driver and indexed-buffer probes did not justify
  their cost and remain experiments only.
- The native CI now compares moving scenes and changing views against original
  MuJoCo RGB/depth buffers. Runtime parity explicitly requests 10 Hz cameras;
  the standalone CLI default of 1 Hz must never be used as a 10 Hz speed claim.

相机视角复用渲染资源和物理快照，单光源阴影仅在同一快照内复用；视野外的有界
物体跳过绘制，但阴影投射物和反射路径保留。新增性能逻辑集中在小型相机模块；
固定版本的第三方绘制实现单独保存来源、哈希和许可。软件阴影上限为 2048，
不修改模型本身。未采用增加内存却未达到目标的并行渲染方案。

### Measured results / 实测结果

Same Go2W input, full D435i/MID360, 5 simulated seconds, 50 frames per camera,
no concurrent project benchmark. Loop timings exclude initialization and final
artifact saving; RSS is the whole core process high-water mark.

| Metric / 指标 | Before / 原版 | After / 本轮 |
| --- | ---: | ---: |
| Real-time factor / 实时率 | 0.151 | 0.412 |
| Camera pair median / 完整相机输出中位耗时 | 643 ms | 222 ms |
| Camera pair p95 | 768 ms | 274 ms |
| Core peak RSS / 核心峰值内存 | 858 MiB | 678 MiB |
| Mean CPU cores / 平均 CPU 核数 | 2.97 | 2.05 |
| CPU seconds for the same 5 s simulation | 98.6 | 24.9 |

The core executable grows by 41,752 bytes (about 41 KiB); model and policy assets
are unchanged. The 60 s continuous turning run retains 600 complete camera
frames, 600 scans and 12,000 samples from **each** IMU, at RTF 0.413 and peak RSS
679 MiB. This demonstrates sustained improvement, **not real time for Go2W**.

相同输入约提速 2.7 倍，核心峰值内存减少约 180 MiB；程序增加约 41 KiB。
持续转弯 60 秒运行保留完整的 600 帧相机、600 次扫描和每组 12,000 条 IMU，
实时率 0.413，峰值 679 MiB。Go2W 仍未实时；没有用减少采样来改变这一结论。

Ablation at the same 2048 shadow precision and identical input: camera medians
are **222 ms** with both optimizations, **372 ms** without view culling, and
**401 ms** without shadow reuse. All deterministic artifacts, including RGB and
depth, remain byte-identical. Both optimizations are retained on measured benefit.

在相同阴影精度和输入下，关闭视野裁剪使相机耗时从 222 增至 372 ms；关闭阴影
复用则增至 401 ms。三组确定性产物（含 RGB／深度）逐字节相同，因此保留两项。

### End-to-end acceptance / 端到端验收

| Check / 检查 | Result / 结果 |
| --- | --- |
| Native CTest | 12/12 |
| Original MuJoCo RGB/raw-depth oracle | 224/224 passes pixel-identical at equal shadow precision; moving object/light, camera changes, textured surfaces, mesh geometry, transparency, reflection and two shadow lights |
| Runtime artifact parity | Seven robots, rolling/snapshot LiDAR, full D435i; 10 cases pass; additional explicit 10 Hz case passes. The old binary is given the same 2048 shadow precision only for this parity comparison |
| Noetic full sensor matrix | Seven robots; 39/39 checks each; 20 s complete collection, unchanged timeouts |
| Humble full sensor matrix | Seven robots; 40/40 checks each; 20 s complete collection, unchanged timeouts |
| Go2W hills + Noetic | 39/39, complete 20 s, RTF 0.396 |
| Native UI input | 40/40 |
| Presets and scene editing | 17/17 |
| GUI → core → Noetic diagnostics | 11/11 |

Every ROS case uses RGB, depth, panorama, MID360, D435i IMU and external-odometry
isolation. The prior overall image-collection timeouts are resolved in this tested
matrix. Performance remains model-dependent:

| Robot | Noetic RTF | Humble RTF |
| --- | ---: | ---: |
| Go2W | ~0.40 | 0.42 |
| Go2 | 0.48 | 0.50 |
| Scout Mini | 0.72 | 0.77 |
| Ranger Mini | 0.66 | 0.66 |
| FR Mini | ~1.00 | ~1.00 |
| Ridgeback | ~1.00 | ~1.00 |
| TurtleBot3 Waffle Pi | 0.60 | 0.60 |

七种机器人在双 ROS 下的完整图像／雷达／双 IMU 链路均完成，原超时未重现。
FR Mini 和 Ridgeback 约实时；其他车型仍低于实时，尤其 Go2W／Go2。GUI 和
场景编辑可用性通过，不等于所有模型的满传感器软件渲染均已实时。本轮没有重新
证明 n3m 全路线、重定位或真机效果。已安装的 SpRobo 未被测试构建覆盖。

Evidence (local, ignored): [summary](../output/software-panorama-20261021/validation-summary.json),
[performance](../output/software-panorama-20261021/benchmark.json),
[ablation](../output/software-panorama-20261021/ablation.json),
[native image oracle](../output/software-panorama-20261021/render-consistency.log),
[10 Hz parity](../output/software-panorama-20261021/full-10hz-parity/report.json).
The candidate is `build/software-panorama/simple-rbt-sim` with its matching console.
This stage retained original RGB meshes. Xian subsequently authorized controlled
RGB-only simplification; that follow-up is recorded below. Sensor rates remain fixed.

本阶段保留原始 RGB 网格。Xian 随后授权了受控的 RGB 简化；后续结果见下，
传感器频率仍保持不变。候选构建位于 `build/software-panorama/`。


## Adaptive software RGB meshes / 软件 RGB 自适应网格

Verified 2026-10-21 on CPU MuJoCo 3.15.0 and llvmpipe. This follows `2db1155`.
Xian authorized controlled RGB-only simplification, prioritizing real-time speed.
Output sizes, camera/LiDAR rates, ray counts, IMU rates and shadows remain as
specified above. Physics, controllers, LiDAR and sensor depth use original geometry.
No model/policy assets or ROS contracts changed; installed SpRobo 0.2.1 was not replaced.

### Implementation and ablation / 实现与消融

- Reject meshes wholly before the near clipping plane, including the camera's own
  housing. In the isolated prototype this reduced camera median time from about
  110 to 83 ms with **byte-identical artifacts including RGB/depth**.
- Build context-owned indexed buffers once. RGB selects LOD using a projected
  half-pixel **error estimate** with a margin; it is not a certified geometric
  bound or a bound on shading differences. Preserve original normal/UV attributes.
  Shadow meshes use a separate 1 mm estimate, retaining the 2048 shadow map.
  Depth draws every original triangle in original order; it is never simplified.
- Combine base and shadowed lighting for the supported opaque, untextured mesh
  case. Removing this step raised the prototype camera median from about 83 to
  100 ms. Exact position-only depth indexing also helped (~88 to 83 ms).
- Remove arbitrary 5% RGB / 10% shadow triangle-count floors; stop simplification
  at its error limit instead. A sequential 5 s comparison improved the camera
  median from 86 to 81 ms while passing the same image-quality budget.
- Keep CPU mesh preparation, GL buffer ownership, lighting and render orchestration
  in separate small private C++ modules. The pinned MIT simplifier subset has
  source hashes and installed license records. There are no runtime downloads,
  public MuJoCo symbol overrides, new physics threads or disk mesh caches.
- Reject the extra visibility-query, view-thread, vertex-light-cache, plane-tile,
  PBO and index-reordering machinery after measurement. A 1024 shadow map did not
  show a useful speed gain and exceeded the chosen RGB quality budget, so 2048 stays.

跳过完全位于近裁剪面前的自身外壳；RGB 按视口选择简化级别，并合并适用材质的
两次光照绘制。半像素是带余量的投影误差估计，不是严格证明；阴影仍保留，使用
独立的 1 mm 网格误差估计与 2048 贴图。深度保留全部原始三角形，物理、雷达、
控制器和资产均不改动。去掉人为的三角形数量下限后，仍受原来的误差预算约束。
没有收益的实验不进入运行代码。

### Sustained performance / 持续性能

Matched 60 s Go2W turning input, 10 Hz full D435i/MID360, CPU-only. These are
simulation-loop measurements, excluding startup and final artifact saving.
RSS includes libraries and initialization. Other host applications were not stopped.

| Metric / 指标 | Before / 本轮之前 | Final / 最终候选 |
| --- | ---: | ---: |
| Real-time factor / 实时率 | 0.413 | 1.018 |
| Wall time for 60 simulated seconds / 运行阶段墙钟时间 | 145.21 s | 58.93 s |
| Complete camera pair median / 完整相机输出中位耗时 | 223.0 ms | 83.0 ms |
| Camera p95 | 268.9 ms | 101.2 ms |
| Mean logical CPU cores / 平均逻辑核占用 | 2.01 | 2.32 |
| CPU seconds for the same simulation / 同段仿真累计 CPU 时间 | 292.2 s | 136.7 s |
| Core peak RSS / 核心峰值内存 | 678.6 MiB | 691.0 MiB |

This case reaches **average real-time throughput**, not a hard per-frame deadline.
There is limited headroom: ROS and terrain cases below remain slower. The binary
adds 71,696 bytes (~70 KiB), without changing model/policy storage. A synchronized
render-pass probe attributes roughly 50 ms per complete camera sample to RGB plus
shadows and 26 ms to original-geometry depth; software drawing remains the main
cost. These probe times exclude pixel readback and other simulation work.

相同的 60 秒输入从 145.21 秒降到 58.93 秒；进入运行阶段后，平地核心达到平均
实时。累计 CPU 时间减少约 53%，平均占用 2.32 个逻辑核，峰值内存增加约 12 MiB。
可执行文件增加约 70 KiB。仍有慢帧，不能将这一结果解释为所有地形、ROS 和 GUI
组合都保证实时；剩余大头仍是软件 RGB／阴影绘制与保持原模型的深度绘制。

### Acceptance and limits / 验收与边界

- 12/12 native CTest checks; 224 original MuJoCo RGB/depth passes remain exact.
  Another 112 dense-mesh passes check RGB quality and exact segmentation/depth,
  including textured and untextured meshes. This oracle also runs in native CI.
- Ten runtime parity cases pass: seven robots, rolling/snapshot LiDAR and full
  10 Hz D435i. A separate matched 60 s artifact comparison passes. Both runs retain
  600 camera samples, 600 scans and 12,000 samples from each IMU. Every non-RGB
  artifact and deterministic summary field is equal. Saved perspective RGB is
  identical; the final panorama has mean channel error 0.305/255, with 0.818% of
  pixels differing by more than 10/255 in at least one channel (maximum 164/255).
  The explicit budget is mean ≤1/255 and outlier pixels ≤2%; default artifact
  comparison still requires exact RGB unless these limits are requested.
- Seven robots × Noetic (39 checks each) and Humble (40 each) complete the unchanged
  20 s full-sensor E2E, including panorama and external-odometry isolation.
- Go2W hills + Noetic: 39/39, complete collection, RTF **0.777**. Functional
  acceptance passes; this terrain case has **not** reached real time.
- Native desktop input: 40/40; presets/scene editing: 17/17; GUI/core/Noetic
  diagnostics: 11/11. The desktop input case includes RGB/depth and live observer
  views, but disables panorama; it is not a full-panorama performance result.

| Robot | Noetic RTF | Humble RTF |
| --- | ---: | ---: |
| fr_mini | 0.998 | 0.997 |
| go2 | 0.996 | 0.996 |
| go2w | 0.945 | 0.886 |
| ranger_mini | 0.996 | 0.996 |
| ridgeback | 0.997 | 0.998 |
| scout_mini | 0.996 | 0.996 |
| turtlebot3_waffle_pi | 0.997 | 0.997 |

全车型双 ROS 链路通过，超时与消息／运动断言未放宽。Go2W 接入 ROS 后约为
0.89–0.95 倍实时，起伏地形 + Noetic 为 0.777；这些限制与平地核心的 1.018
分别记录。本阶段没有重新验证 n3m 全路线、重定位、GPU 或真机，也没有部署桌面包。

The first GUI runs exposed a test synchronization defect: a `config` handshake
arrives before renderer preparation and the first preview frame. The native test
looked for Start while it was correctly disabled; the workspace test used the
initial 640×480 metadata. Tests now wait for an actual View packet from the current
worker before interacting with that preview. Existing deadlines and behavior
assertions remain unchanged. Original failures and passing reruns are retained.

首轮桌面测试在新模型尚未生成预览时就查找按钮／视口。改为等待当前工作进程的
真实 View 帧后，两组完整测试通过；不再用固定睡眠推断模型已经准备好。保留
初次失败记录，没有修改产品的就绪条件或放宽超时。

Evidence (local, ignored):
[summary](../output/software-panorama-followup-20261021/validation-summary.json),
[60 s artifact parity](../output/software-panorama-followup-20261021/long-parity.json),
[runtime parity](../output/software-panorama-followup-20261021/release-parity/report.json),
[ablation](../output/software-panorama-followup-20261021/realtime-check.json),
[final simplification comparison](../output/software-panorama-followup-20261021/post-profile.json),
[render profile](../output/software-panorama-followup-20261021/release-profile.json).
The verified core is `build/software-panorama/simple-rbt-sim`, SHA-256
`3cf65bc5c74a210a9a3dff3ed0aef9db00466497dc0a4f71475335b6fcb372f1`.
Future throughput work should target original-depth and terrain drawing, using
these exact-output and E2E oracles; lowering sensor rates is not an accepted shortcut.


## CPU hills with Noetic — 2026-10-22 / CPU 起伏地形与 Noetic

Go2W with Noetic now sustains approximately real-time throughput on the tested
hills route, preserving 320×240 perspective RGB/depth, 720×360 panorama from six
256×256 views, 10 Hz cameras and 20,000-ray MID360 scans, both 200 Hz IMUs, original
multisampling and the existing 2048 shadow cap. Models, policies, RGB LOD/error
budgets and sensor mathematics are unchanged. Current design is maintained in
[the runtime guide](runtime.md#software-camera-rendering--软件相机渲染).

### Changes and ablation / 改动与消融

- Finite-plane drawing retains the original indexed quad grid, UVs and primitive
  order, rejecting only wholly invisible strips. Shadows use the full grid;
  infinite planes and reflections retain native drawing. This replaces repeated
  submission of the complete floor, without introducing geometry approximation.
- Default the Mesa software worker pool to at most four threads, respecting an
  existing `LP_NUM_THREADS`. Partition large CPU native ray queries across at
  most four workers, then assemble returns in original order. Small scans stay
  serial. No ROS wrapper or physics/control algorithm changes.
- A dedicated C++ camera worker owns its model, state snapshot and GL context.
  Physics and other sensors advance while it draws. Only one sample can be in
  flight; backpressure waits rather than dropping or replacing frames. Results
  publish on the simulation thread with their capture timestamp. Pause polling
  and final shutdown drain are explicit. The worker is 88 lines; scheduling and
  plane drawing stay in separate modules rather than one enlarged runtime file.
- With the plane/thread/ray changes but synchronous cameras, a 140 s hard-hills
  Noetic route reached RTF 0.965. Overlapping camera work raised two independent
  runs to 0.997 and 0.994, each reaching five alternating endpoints with no sensor
  timestamp gaps in the measured interval. This costs one model/state copy.
- Primitive-lighting fusion gave little benefit and was not retained. Additional
  RGB normal-weight reductions and per-LOD vertex copies were also excluded:
  small short-run gains did not justify changing appearance or extra memory.
  Neither shadow precision nor the accepted RGB error budget was loosened.

有限地板只剔除完全不可见的原始四边形条带；雷达按原始查询并行计算，完成后按
原顺序合并。相机改为单独线程处理冻结快照，物理无需等待整次绘制；最多一帧
任务，慢时等待，绝不靠丢帧达到实时。消息保留采集时间戳，但存在绘制延迟。
暂停／关闭会处理已采集帧。额外开销主要是私有模型副本，不增加磁盘模型缓存。

### Sustained measurements / 持续测量

Final paired 60 s hills input (`go2w`, `.3 0 .15`, full D435i/MID360 at 10 Hz),
with no concurrent task-owned build/test. Both binaries run sequentially on CPU
llvmpipe; unrelated host applications remain running. These are unpaced core
loop measurements, excluding initialization and final artifact saving. The
candidate includes completion of the final camera sample in measured wall time.
Earlier exploratory runs are retained but are not the final paired result.

| Metric / 指标 | Before `91f5561` | Final candidate / 最终候选 |
| --- | ---: | ---: |
| Real-time factor / 实时倍率 | 0.844 | 1.112 |
| Wall time / 墙钟时间 | 71.05 s | 53.98 s |
| Camera median / 相机中位耗时 | 96.4 ms | 86.1 ms |
| Camera p95 | 123.6 ms | 117.4 ms |
| LiDAR median / 雷达中位耗时 | 8.1 ms | 4.7 ms |
| Mean logical cores / 平均逻辑核 | 2.36 | 2.24 |
| CPU seconds / 累计 CPU 时间 | 167.4 s | 120.7 s |
| Core peak RSS / 核心峰值内存 | 692.6 MiB | 764.2 MiB |

The headless **Go2W + Noetic hard-hills** route lasts 140 simulated seconds,
traversing the 48-segment, 8 cm waves (maximum segment slope 7.08°), turning and
stopping between alternating X=0/6 m endpoints. Final wall time is
**140.78 s**, RTF **0.994**; five endpoints reached.
Core average CPU is 2.29 logical cores and peak RSS
765.9 MiB; these resource figures exclude the ROS adapter,
receiver, desktop and display server. The native executable grows 58,280 bytes
(~57 KiB), with no additional model/policy storage.

This is near-real-time throughput, **not a hard per-frame deadline**: final
10 s windows range from 0.940 to 1.061 RTF, and camera p95 is 124.1 ms. Rendering
still dominates; overlapping work does not make each camera frame faster or
eliminate turn-related slow frames. A live GUI observer adds rendering work and
is not included in this full-panorama throughput claim.

最终同输入 60 秒测试从 71.05 秒降至 53.98 秒；核心累计 CPU 时间减少约 28%，
峰值内存增加约 72 MiB。Noetic 的 140 秒、8 cm 起伏往返路线用时 140.78 秒，
达到约 0.994 倍实时，5 次端点到达；核心约占 2.29 个逻辑核、766 MiB 峰值内存。
整体已基本实时，仍有转向慢帧，不能解释为所有场景或每一帧都保证实时。

### Acceptance / 验收

- Native CTest 12/12; 224 original renderer passes plus 224 tilted/textured/
  transparent/infinite-plane passes remain exact. Another 112 dense-mesh passes
  retain the existing RGB quality and exact segmentation/depth checks.
- New snapshot oracle compares all six captured RGB/depth/panorama pairs to
  synchronous rendering while live physics advances; worker shutdown and error
  propagation also pass. It is included in the native CI check.
- Ten unchanged runtime parity cases pass with **exact RGB**, without an image
  tolerance override. The final paired 60 s run also has identical saved RGB,
  depth and LiDAR payloads, full trajectory/IMU histories, and deterministic
  summary fields. This artifact comparison stores the final images/scan, not
  every frame. Both runs generate 600 camera samples, 600 scans and 12,000
  samples from each IMU.
- Both 140 s Noetic camera-worker runs generate 1,400 camera samples/scans and
  28,000 samples per IMU. Independent ROS receipts over simulation seconds
  [5,139) have no missing periods: 1,340 messages per 10 Hz stream and 26,800 per
  IMU. Perspective/panorama RGB and depth dimensions remain unchanged.
- Seven robots × Noetic (39 checks each) and Humble (40 each), now on standard
  hills, complete the original full-sensor 20 s E2E. Motion, command namespaces,
  CustomMsg/PointCloud2 consistency, camera extrinsics and external-odometry
  isolation keep their original assertions and timeouts.
- Native desktop input 40/40, workspace/presets 17/17, diagnostics 11/11. The
  desktop input test includes RGB/depth and observer views with panorama off;
  it validates interaction rather than full-panorama GUI throughput.

| Robot | Noetic hills RTF | Humble hills RTF |
| --- | ---: | ---: |
| fr_mini | 0.998 | 0.997 |
| go2 | 0.996 | 0.996 |
| go2w | 0.994 | 0.995 |
| ranger_mini | 0.995 | 0.996 |
| ridgeback | 0.997 | 0.997 |
| scout_mini | 0.997 | 0.997 |
| turtlebot3_waffle_pi | 0.997 | 0.997 |

全车型双 ROS、桌面交互及数据一致性检查通过，没有降低超时或验收标准。
本阶段未重跑 n3m 建图／重定位、GPU 或真机测试，也未替换系统安装的 SpRobo。

Evidence (local, ignored):
[aggregate](../output/software-hills-noetic-20261022/validation-summary.json),
[final 140 s route](../output/software-hills-noetic-20261022/release-hard140/report.json),
[60 s exact artifact parity](../output/software-hills-noetic-20261022/release-long-parity.json),
[runtime parity](../output/software-hills-noetic-20261022/parity/report.json),
[Noetic](../output/software-hills-noetic-20261022/e2e-ros1/summary.json),
[Humble](../output/software-hills-noetic-20261022/e2e-ros2/summary.json).
Build: `build/software-panorama/simple-rbt-sim`; SHA-256
`a3a01aa2fef1a4005de9b149f0fe61fac0f4eced94ae3605af2866cbe5f39c9b`.

## Installed desktop update — 2026-10-22 / 桌面安装更新

Installed the runtime from `b337910ad1be6128b653182277d6c609010c3d0e` as a
local SpRobo 0.2.1 test package, preserving optional CUDA support. The version
number is unchanged; the package hash and executable hashes identify this build.
The default Noetic/Humble adapter tags now select the images verified in the
preceding full E2E stage; their previous images remain available locally.

Jammy 更新了本地 SpRobo 安装：启动器、核心及资源均来自上述运行代码版本，
保留 CUDA 可选能力和原有用户配置。包版本仍为 0.2.1；本次不是新的公开发行。

Packaging initially rejected the camera worker's `GLIBCXX_3.4.30` dependency.
Jammy updates already provide libstdc++ 12; the package now explicitly requires
`libstdc++6 >= 12` and validates that ABI ceiling instead of the original GCC 11
ceiling. No runtime behavior was changed for this deployment.

验证结果：独立 Ubuntu 容器中的安装、7 车型 CPU 启动、普通用户窗口及卸载均通过；
Jammy 实际安装程序的 40 项桌面交互检查通过。Go2W＋Noetic 起伏地形通过 38 项检查，
20 s 仿真 RTF 为 0.995，产生 200 帧／扫描和每套 IMU 4,000 个样本；
图像尺寸、10 Hz 相机和雷达、200 Hz IMU 均保留。测试使用软件渲染，未重新验收 GPU。

Installed executables use `/usr/share/simple-rbt` assets and bundled libraries,
without a checkout or preparation virtualenv. Existing user settings were hash
checked before and after deployment. Xian's next step is interactive testing via
the application menu or `sprobo`.

Local deployment evidence:
[deployment](../output/jammy-install-b337910-20261022/deployment.json),
[installed desktop](../output/jammy-install-b337910-20261022/gui-installed/report.json),
[installed Noetic](../output/jammy-install-b337910-20261022/noetic-installed/summary.json).

## Fixed-base Panda and two RGB-D sensors — 2026-10-23 / 固定机械臂与双相机

Added the pinned MuJoCo Menagerie Panda model with its original joint limits,
inertias, contacts and coupled fingers. A catalog-driven C++ position controller
provides seven arm targets and a gripper target. The console selects joint
controls by capability; mobile robots retain their existing velocity controls.
Native IPC and both separate C++ ROS adapters carry joint commands and feedback.
The interface currently executes joint-position targets; Cartesian planning and
grasp-task execution have not been implemented or validated.

新增固定底座 Panda：七关节、联动夹爪、手动及程序控制。控制和传感器仍由 C++
核心实现，ROS 1／ROS 2 保持独立。当前支持关节位置目标，不包含末端路径规划或
抓取任务执行。模型来源、单位、超时及话题见
[runtime contract / 运行接口](runtime.md#fixed-arm-and-two-cameras--固定机械臂与双相机)。

Wrist and world-fixed D435 sensors produce separate 320×240 RGB and metric Z
depth at 10 Hz, with camera information and TF. They share a frozen physics
snapshot, rendering worker and context. Numeric and drag editing use each
mount's actual parent frame; a named preset retains both mounts. The external
sensor supplies the third-person runtime view; free Observer remains separate.
Fixed robots can also use a user scene's spawn and floor elevation.

腕部第一人称与世界固定的第三人称相机各自输出 RGB、深度、标定及 TF。
两套外参都支持数值编辑、拖拽和命名预设恢复。双相机共用同一物理快照及渲染线程，
不复制整个仿真实例；机械臂也支持用户场景中的出生点与楼层高度。

### Verified results / 验收结果

| Check / 检查 | Result / 结果 |
| --- | --- |
| Installed native control and RGB-D geometry / 安装后的控制与双相机几何 | 21/21; independent rays validate both depth streams |
| Installed arm desktop E2E / 安装后的机械臂桌面交互 | 15/15; both mounts, presets, joints, gripper, focus ownership and both themes/languages |
| Existing desktop E2E / 既有桌面交互 | 40/40 |
| Panda + Noetic / Humble | 34/34 each; both RGB-D streams, TF, joints, timeout and invalid-command recovery |
| Panda + Noetic without cameras / 关闭相机 | 10/10 |
| Go2W + Scout Mini on hills / 两种底盘起伏地形回归 | Noetic 33 each; Humble 34 each |
| Native CTest / 原生测试 | 12/12; IPC tests run on host because sandbox sockets cannot bind |
| Go race tests / Go 并发检查 | all three packages pass |
| Clean Ubuntu package / 干净 Ubuntu 安装包 | install, eight CPU models, non-root window, dependency isolation and uninstall pass |

The installed core's 12-second arm run generates 120 samples from each camera
at RTF **0.9985**, mean CPU **0.44 logical cores**, and peak RSS **513.4 MiB**.
Both 20-second ROS runs are approximately **0.999 RTF**. These use CPU physics
and Mesa software rendering with shadows. CPU/RSS cover the core process,
excluding console, display server, ROS adapter and receiver. They do not establish
grasping, long-route, GPU or real-hardware performance.

安装后实测保持 320×240、10 Hz 和阴影，双相机基本实时。资源数据仅覆盖核心进程，
不包含界面、显示服务和 ROS。包约 27.7 MiB，安装内容约 152.9 MiB；相较前次本地
安装，磁盘增加约 33.0 MiB，主要来自 Panda 网格。没有要求 GPU。

Two validation findings informed the implementation: the model's automatically
computed 36 m visualization extent clipped the close wrist geometry and reduced
shadow precision. Explicit 2 m extent and a spawn-aware centre restore the 1 cm
near plane without changing physics. Separately, a worker can report running
before software EGL finishes initialization; the lifecycle test now waits for
its first real image within 10 seconds. Sensor rates and runtime deadlines did
not change.

模型自动计算的显示范围过大，曾导致腕部近景被裁剪；现在显式设置机械臂工作区的
显示范围，并随出生点移动。另一个问题是 EGL 初始化晚于运行状态消息；生命周期
测试现等待首张实际图像，不再将状态消息当作图像就绪。频率与运行时限未放宽。

Jammy now uses the verified package and both updated local ROS adapter images.
Existing user configuration hashes are unchanged. The package keeps version
0.2.1 for local testing; no new public release was created. Xian can reopen
SpRobo, choose **Panda · fixed arm**, and test joint/gripper and camera controls.

Jammy 已更新，原有用户配置不变。Xian 下一步可重新打开 SpRobo，选择
**Panda · 固定机械臂**，测试关节、夹爪和双相机。

Local evidence / 本地证据：
[deployment](../output/panda-integration-20261023/deployment.json),
[installed native](../output/panda-integration-20261023/installed-native/report.json),
[installed desktop](../output/panda-integration-20261023/installed-gui/report.json),
[Noetic](../output/panda-integration-20261023/jammy-ros1/summary.json),
[Humble](../output/panda-integration-20261023/jammy-ros2/summary.json).

## Observed scene capacity — 2026-10-24 / 实测场景容量

Removed the separate 512-authored-object restriction. The editor and C++ core
now use the existing budget of 2048 generated physical boxes, including stairs
and openings. This is a resource guard, not a MuJoCo limit or a real-time
guarantee. Both entry points explicitly reject authored or expanded overflow.

取消额外的 512 个编辑对象限制，统一按展开后的物理盒体计数。旧数值没有实测地图
的性能依据；不能据此裁掉合格表面。现有 2048 个盒体预算仍是保护值，不能视为
硬件上限。超限检查、场景及传感器原生测试、Go 竞态检查均通过。

A local offline candidate retains all 1316 supported surface patches with
per-object source and residual records. Both levels pass stationary Go2W
contact and MID360 checks. The native editor loads and saves all patches.
With software rendering, RGB-D, MID360 and a separate Noetic node, the closed
16.89-second core run measures RTF 0.99824. GUI, core, ROS and the read-only
receiver together peak at 1271.4 MiB RSS; mean CPU is 2.86 logical cores.
Display-server cost is excluded. ROS traffic checks establish delivery, not
lossless IMU reception; the diagnostic receiver uses a one-message queue.

这只是观测表面候选：没有补齐未知区域，没有通过验收的楼梯踏面，也没有跨层
可通行性或绝对地图精度结论。4 厘米碰撞厚度是显式近似。楼层静止检查与近实时
运行不等于全路线验收。真实场景数据保留在本地，不进入公共仓库。

The candidate binaries are in `build/panda-jammy`; this stage does not replace
the installed desktop. The next step is review of the observed patches and
missing stair support before any route-based acceptance.

本地证据：[candidate report](../output/observed-scene-20261024/report.json)、
[native editor and Noetic](../output/observed-scene-20261024/gui-full/report.json)、
[budget rejection](../output/observed-scene-20261024/budget-verified/report.json)。

### Follow-up audit and scene contract / 后续核对与表征约定

The fragmented candidate also exposed an extraction issue: validating coverage
only after merging cells discards supported parts when another part lacks held-out
coverage. A local diagnostic moves the same coverage checks before merging;
it recovers supported floor/wall patches without relaxing the final gates.
It is an extraction ablation, not a replacement runnable building model.
Observed stair profiles contain treads and inclined undersides. A shared
parametric fit remains incomplete: some held-out profiles still exceed the
existing 6 cm P95 gate, and width, endpoints and landing joins are not accepted.

缺口不能统一归为源数据不足。楼梯的台阶和底部斜面在源观测中存在；独立水平片段
提取没有表达整体结构。后续按 [v2 场景约定](scene-representation.md) 区分参数基元、
有限边界、共享模板与推定区域；不将内部检查网格直接交付为最终编辑对象。

Closed native/IPC/ROS/bag checks now account for the complete intervals. A
22.062-second Go2W run on a qualified support patch has 4412 samples from each
IMU and 220 scans in all four stages, with exact timestamps, IMU values and
LiDAR point counts. It moves 0.348 m, returns within 0.046 m and turns through
27.7 degrees. This is covered-patch motion acceptance, not a full route.

An additional simultaneous receiver ablation isolates the earlier diagnostic
loss: the old queue=1 receiver misses 711/4011 MID360 IMU samples and 692/4011
camera IMU samples, while IPC, the buffered receiver and the closed bag agree
exactly for the same run. Both receivers receive all 200 scans. No ROS bridge
or runtime change was needed for these results.

完整计数核对区分了接收器与 ROS 桥。所有检查保留原频率、图像尺寸和 CPU 路径，
没有修改 N3M 或安装版。证据见
[audit report](../output/observed-scene-audit-20261024/report.json)。

The v2 stage delivers a bilingual geometry/pose/boundary contract, JSON Schema,
and a synthetic two-storey example with a shared stair template. All nine
primitive definitions and the example pass schema checks; seven malformed
shape cases are rejected. Example references, transforms, ring orientation and
stair/platform joins were checked separately. This establishes the data contract;
v2 C++ loading, editor integration and runtime E2E remain future implementation.

本轮完成表征约定和可校验示例；没有宣称新基元已接入运行时。下一步按规范实现
公共解析、语义校验与几何生成，再逐步接入编辑器。验证记录：
[schema and example](../output/scene-representation-20261024/validation.json)。

## Structured scene v2 — 2026-10-24 / 结构化场景

The development build now loads and edits the [v2 contract](scene-representation.md).
One C++ resolver handles parent transforms, bounded planes and holes, nine primitive
types, shared templates, per-instance overrides, spawn selection and geometry
budgets. SpRobo reuses its output for preview and picking; save/reopen and history
preserve the document. Legacy v1 files retain their original geometry. The native
runtime remains C++; Python is limited to offline reconstruction and verification.

开发版已接入 v2。楼梯是一个编辑对象，内部才展开为碰撞体；底部保留斜面。
有限地板和墙面保留边界与洞口。编辑器支持共享模板、实例覆盖、位姿、总高、
坡度、删除、撤销、保存及重开。本轮未替换安装版，也未发布远端版本。

CPU LiDAR now accepts native curved shapes and convex meshes. A static AABB tree
narrows candidate geometry; exact intersections still use MuJoCo routines.
Forty thousand randomized rays agree with direct MuJoCo within 1e-10 m. On the
same 2814-part cloud scene and input, an eight-second unpaced physics/LiDAR run
improves from 14.933 s to 1.292 s wall time. Both trajectory and LiDAR-world files
have identical hashes. Resolution, scan rate and ray count are unchanged.

同场景消融表明，旧逐几何射线查询是主要瓶颈。增加空间索引后，同输入轨迹和雷达
输出保持逐文件一致；没有降低传感器参数。4096 个凸几何体是 v2 的资源保护预算，
不是所有场景实时运行的保证。显式 CUDA 不支持的几何会报错，auto 可回退 CPU；
本阶段只验证 CPU。

Validation / 验证：

- Native CTest: 14/14 passed, including v1, all nine v2 primitives, open stair
  undersides, bounded holes, collisions, IMU, IPC and a Go2W staircase route.
  Go race checks pass. Native editor E2E verifies template/override edits,
  delete/undo/redo, save/reopen and real core preview in both languages/themes.
- A separate Noetic-controlled Go2W run climbs and descends the synthetic
  two-storey U-stairwell: 8/8 waypoints, 98.562 simulation seconds, RTF 0.999999.
  Each IMU has 19,712 samples; each LiDAR stream has 985 scans. Native summary,
  independent IPC observer, buffered ROS receiver and closed bag agree on full
  counts and timestamps; IMU values and scan point counts agree exactly. This
  is a synthetic route, not acceptance of the reconstructed real staircase.
- On the 2814-part observed scene, CPU-only GUI/core/Noetic plus the test receiver
  measure RTF 1.0053, mean 1.75 logical CPU cores and peak aggregate RSS 1434 MiB
  over a 16-second stationary window. All processes are restricted to four cores.
  D435i RGB/depth remain 320×240 at 10 Hz; MID360 remains 20k rays at 10 Hz, with
  200 Hz IMU. Display-server cost is excluded; receiver cost is included in ROS.
  Camera topic traffic is checked; the exact bag comparison covers LiDAR/IMU,
  not camera payloads.
- The final 2820-part candidate passes separate eight-second stationary contact
  checks on both levels. Its 22.106-second Noetic out/back/turn run has RTF
  0.999917, 4421 samples per IMU and 221 scans per LiDAR stream, with the same
  full-interval accounting. Core peak RSS is 869 MiB; this last figure excludes
  GUI and ROS. It is motion on a qualified patch, not a cross-storey route.

Real-cloud reconstruction / 真实点云重建（2026-10-24 follow-up）：

The original 884,960-point PCD remains unchanged. The derived clean copy retains
833,115 original XYZI records and removes 51,845 confirmed dynamic candidates.
The kept/removed index sets partition the source exactly, without moving points.
All source points within the 2247 previously qualified bounded architectural
surfaces remain; the new stair and landing surface retention checks also remove
zero structural support points.

清理基于原始逐点时间、多次观测的空域证据和建筑表面保护。10,709 帧通过运动补偿
投影检查，11 帧因原始射线与前端点云投影偏差过大而排除。偶数帧提出候选，奇数帧
独立核对；还要求跨时段观测。未通过清理条件的可疑点保留，不能据此宣称完整的
人体语义分割。原始 PCD、轨迹、映射成果和旧失败证据均保留。
留出帧共享同一优化轨迹，属于重复观测核对，不是独立测量真值。

The four flights share 13 risers, 0.149743 m step height and 0.280069 m tread
depth (28.132 degrees). Widths are independently measured as 1.751, 1.619, 1.753
and 1.618 m. Allowing per-flight slab thickness instead of forcing one common
thickness reduces all four held-out central profile P95 errors below the original
6 cm gate: 2.63, 5.40, 3.97 and 5.11 cm. The first underside is unobserved and its
thickness remains explicitly inferred. Failed shared-thickness fits are retained.

平台采用有限轮廓，并与整条梯段端部做并集；八处出入口均覆盖完整梯宽。端部高度差
保留在模型和报告中，范围约 −4.7 至 +9.6 厘米，没有通过移动原始点云或强制齐平来
掩盖。最终场景有 1041 个节点、2880 个物理分解件。887 个原有观测平面区域保持不变，
三段局部楼梯替换为四段完整梯段，并增加五个平台和一个路线出生点。

**Geometric scope:** visible profiles and finite endpoints are measured; occluded
landing interiors and last-tread areas use a documented structural inference.
Full direct surface coverage has not passed everywhere. Platform top and lower
faces must be separated before fitting; a single plane is not a valid model of
both. A successful route is functional acceptance of this mixed reconstruction,
not a survey certificate for its hidden regions.

Mixed reconstruction Go2W + direct Noetic E2E / 混合重建的功能验收：

- Four flights up and down, 21/21 waypoints, 260.750 simulation seconds; maximum
  tilt 29.483 degrees; real-time factor 0.9999993. Command input is namespaced
  `/go2w/cmd_vel`; the simulator runs on the host and Noetic in its own container.
- Both IMUs retain 200 Hz and each produces 52,150 samples. CustomMsg and
  PointCloud2 each contain 2607 scans at 10 Hz / 20,000 rays. RGB and depth each
  contain 2607 frames at 320 × 240 / 10 Hz.
- All 33 checks pass over the whole run: native counts, IPC, buffered ROS receiver
  and the closed bag agree. IMU values and image pixel hashes match; timestamps,
  scan point counts and ROS message fingerprints are retained. No interval is
  trimmed to hide loss. This checks motion and sensor transport; FAST_LIO/n3m
  mapping is a separate downstream validation.
- The native SpRobo window loads, renders and saves all 1041 nodes. Object
  geometry, poses, boundaries, evidence and the shared template survive the
  round trip unchanged. The test uses an isolated workspace; the user workspace
  is not replaced. Closing the window also stops its owned core process.
- CPU affinity is 0,2,4,6; graphics use software rendering. Core mean CPU is
  0.562 cores and peak RSS 872.8 MiB. Separate two-second process sampling gives
  ROS, receiver and recorder together about 10% of one core on average and
  451.6 MiB peak RSS. This headless route excludes GUI/display-server cost.
  The final raw test bag occupies 2.59 GiB; it is evidence, not runtime log usage.

The earlier measured-plane route also passed, but its resource interval overlaps
an independent native probe. Use the final connected-route measurement above for
performance comparisons. Runtime binaries remain the verified development build
from `751e74e`; no installed desktop replacement or source-data upload is performed.

Mixed reconstruction artifacts / 混合重建记录：
[cleaned PCD](../output/observed-clean-route-20261024/cleaned.pcd),
[scene](../output/observed-clean-route-20261024/candidate-connected/scene.json),
[session](../output/observed-clean-route-20261024/candidate-connected/session.json),
[stage report](../output/observed-clean-route-20261024/report.json),
[Noetic E2E](../output/observed-clean-route-20261024/ros-connected-route/report.json),
[native editor](../output/observed-clean-route-20261024/editor-connected-verified/report.json),
[geometry and retention](../output/observed-clean-route-20261024/connected-geometry-verification.json),
[prior incomplete-route diagnosis](../output/structured-scene-20261024/candidate/route-support.json).
Private clouds, trajectories, scripts with private input paths and test bags stay outside Git.

Observed-support correction / 观测支持范围更正（2026-10-24）：

The connected candidate joined full stair endpoints to wall-bounded landing
interiors without requiring support throughout those finite surfaces. Presenting
its 21/21 route as completed observed-scene acceptance overstated the result.
At that correction stage, no explicit permission to fill large unknown interiors
was found. The mixed
scene, closed bag and user's current preview remain unchanged as evidence.

The separate observed-only candidate has 1140 nodes and 3048 resolved parts.
Qualification uses finite surface support and separate training/held-out scans;
checking all 10,722 acquisition scans in the stair ROI (8,197,016 points) does
not qualify the missing connections. A read-only replay of the mixed route
reproduces every saved 20 ms trajectory row byte-for-byte. Of its 14,681 actual
load-bearing contact samples, 4462 match qualified geometry within 6 cm; only
285 of 2576 sampled contact times have support under every contact. The strict
route is not executed through missing support and is **not accepted**.

严格表面未通过支撑检查，不等于真实建筑不通，也不等于缺口完全没有观测。不能把
稀疏采样格直接当作真实物体边界。下一步是核对有限插值，把同一平面的采样小缝合并；
大块未知区域继续单独保留。原来的 21/21 仅证明混合场景中运动和传感器链路可用。

[Observed-only scene / 严格观测场景](../output/observed-strict-fullscan-20261024/scene.json),
[actual-contact coverage / 实际接触覆盖](../output/observed-strict-fullscan-20261024/route-contact-coverage.json),
[landing evidence / 平台证据](../output/observed-strict-fullscan-20261024/landing-evidence-audit.json).

Continuous-boundary investigation / 连续边界核对（2026-10-24）：

The old finite boundaries follow qualified 25 cm sampling cells. Missing coverage
therefore appears as physical gaps, although it is not evidence of actual holes.
`tools/scene/interpolate_gaps.py` now proposes short joins on each existing fitted
plane. Each added component requires repeated normal-aligned support, separate
keyframe coverage, the existing 6 cm plane-residual gate and a crossing-ray veto.
Its 13 cm boundary band limits it to local interpolation; it cannot complete rooms
from wall outlines. Runtime code remains C++; this is offline preparation.

The comparison candidate reduces 887 wall/floor/ceiling regions to 791 and
adds 37.2525 m² of supported interpolation. It has 945 nodes and 2807 resolved
parts. Existing surface coverage, plane poses, stairs and other objects are
preserved. The native editor imports, renders and saves every object unchanged.
The mixed reconstruction's native Go2W route still reaches 21/21 waypoints in
259.146 simulation seconds. This run checks native motion and LiDAR only; it
does not replace the separate ROS acceptance or establish observed-only support.

This is **not completion of whole architectural contours**: floor regions only
decrease from 210 to 201. Direct triangulation of plane samples was also tested
and rejected: it produces more small islands and point-touching rings. Neither
that experiment nor the small-gap comparison replaces the user's current scene.
The subsequent authorized floor-completion stage below addresses whole floor
outlines. The small-gap comparison remains a separate experiment.

小缝合并已验证，但整体地面、墙面轮廓尚未完成。不能把网格未采到点解释为真实缺口，
也不能用一张大矩形覆盖所有未知区域。原始 PCD 和已有窗口保持不变。
[Boundary comparison / 边界对照](../output/continuous-surfaces-20261024/boundary-comparison.png),
[validation / 验证记录](../output/continuous-surfaces-20261024/report.json).

Authorized planar floors / 已授权的平面地面补齐（2026-10-24）：

The operator explicitly authorized completion within wall boundaries, labelled
as inferred, and confirmed no steps within either main storey. The new candidate
uses one plane per storey, anchored at its stair entrance. Reviewed wall lines,
observed floor edges and corridor connections define the finite outlines;
stairwell openings and large courtyard voids remain. The offline exporter
`tools/scene/complete_surfaces.py` (then named `complete_floors.py`) consumes that
reviewed plan. Runtime code is
unchanged.

The scene has **two main storeys**, with intermediate stairs and landings in a
separate group. It replaces 208 main-floor patches with 13 bounded regions
(nine lower, four upper), all coplanar within their storey. These are outline
pieces, not 13 levels. The complete candidate has 829 nodes and 2469 collision
parts, down from 1041 and 2880. All four flights and the three intermediate
landings are unchanged; the entrance landings follow their corresponding floor
plane. Added floors and those entrance landings are explicitly inferred.

The source fitted surfaces differ from the new planes: lower/upper residual
medians are 3.42/3.33 cm, P95 values 32.24/31.06 cm and maxima 48.58/59.59 cm.
These are reconstruction-model differences, **not measured LiDAR precision**.
The original PCD hash and the separate observed-only evidence are unchanged.
Functional traversal of inferred geometry does not establish observed-only
acceptance or real-building accuracy.

Validation: native Go2W completes all 21 up/down waypoints. Direct Noetic E2E
passes 33/33 checks over 261.482 simulation seconds at RTF 1.000. Both 200 Hz
IMUs retain all 52,296 messages; 10 Hz LiDAR, point cloud, RGB and depth each
retain 2614 frames through core IPC, ROS receiver and the closed bag, including
exact timestamps and payload checks. RGB/depth remain 320 × 240 and LiDAR
retains 20,000 rays. Core peak RSS is 856.3 MiB and mean CPU use 0.673 cores;
these figures exclude the GUI and ROS processes. The final hierarchy-only
changes leave every resolved physical part exactly equal to the tested scene.
The actual native editor imports, renders and saves all 829 nodes unchanged.

地面已按两个主要楼层合并并标注推定，跨楼层运动和 Noetic 传感器链路已通过。
中间平台仍属于楼梯。当前窗口和安装版未替换；这是独立候选场景。

Ceiling follow-up: paired local floor/ceiling fits put most sampled headroom at
about 2.45 m. Two lower-storey regions instead show about 2.70 m, each supported
by 11 qualified same-keyframe fits. One spans 183 s and the other 302 s of
acquisition; the second region also contains a lower return. This supports
retaining a local height difference, but does not establish its architectural
type or full extent. Ceiling geometry was not merged in that floor-only
candidate. The subsequent ceiling stage below adds bounded continuous surfaces;
walls retain their prior geometry.

主体天花板适合连续平面表达；下层局部净高比主体高约 25 厘米，需要保留并核对
边界。尚不能据此判定为梁、吊顶或高顶房间，也不能用零碎点云直接认定顶面起伏。

[Final scene / 最终场景](../output/wall-bounded-surfaces-20261024/candidate-final/scene.json),
[stage report / 阶段报告](../output/wall-bounded-surfaces-20261024/report.json),
[Noetic E2E](../output/wall-bounded-surfaces-20261024/ros-planar/report.json),
[geometry audit / 几何核对](../output/wall-bounded-surfaces-20261024/geometry-audit.json),
[native editor / 编辑器](../output/wall-bounded-surfaces-20261024/editor-final/report.json),
[ceiling evidence / 天花板证据](../output/wall-bounded-surfaces-20261024/raw-headroom-locations-audit.json).

Open this candidate on Jammy / 在 Jammy 打开此候选：

```bash
cd ~/Projects/simple_rbt
./build/panda-jammy/simple-rbt-console \
  --worker "$PWD/build/panda-jammy/simple-rbt-sim" \
  --assets "$PWD/assets" \
  --config "$PWD/output/wall-bounded-surfaces-20261024/candidate-final/session-preview.json"
```


Continuous ceilings and Z-drift check / 连续天花板与 Z 漂移核对（2026-10-24）：

The floor-only candidate above remains available. The new ceiling candidate
normalizes only reviewed roof regions: lower main headroom 2.46 m, lower local
higher headroom 2.71 m, and upper main headroom 2.45 m. Fourteen finite regions
share those three planes. Inferred outlines follow the completed floor domains
and nearby observed ceiling edges; the complete stair envelope stays open.
The western lower corridor and upper southwest pocket have other or unresolved
heights and retain their original geometry. This is not a claim that every
ceiling in the building has one of those three heights.

The scene decreases from 829 to 701 nodes and from 2469 to 2317 collision parts.
Ceiling regions decrease from 416 to 297, including retained source fragments.
All non-ceiling nodes and collision parts are exactly unchanged. Lower main
ceiling thickness is an inferred 0.29 m, extending upward to close the local
height transition; higher and upper ceiling thicknesses are 0.04 and 0.08 m.
The original PCD hash is unchanged.

One clipping prototype created micrometre-wide strips that passed the polygon
parser but failed MuJoCo convex-hull compilation. Preparation now uses a 0.1 mm
Boolean grid and removes 44 numerical remnants narrower than 0.5 mm, totalling
0.000665 m². Original ceiling XY coverage is retained within a 1 mm numerical
buffer. These tolerances describe polygon arithmetic, not measurement accuracy.
The runtime geometry checks were not weakened. All 2317 meshes compile in
MuJoCo 3.15; the actual native editor saves all 701 nodes without changing their
geometry, evidence or shared stair template.

Direct Go2W + Noetic E2E completes all 21 up/down waypoints and passes 33/33
checks over 261.072 simulation seconds at RTF 1.000. Both IMUs retain 52,214
samples at 200 Hz; LiDAR, point cloud, RGB and depth each retain 2610 frames
at 10 Hz through native IPC, ROS receiver and the closed bag. LiDAR retains
20,000 rays and RGB/depth retain 320 × 240 pixels. Payloads and timestamps match
exactly across the measured interval. Maximum robot tilt is 29.69 degrees.
Core-only mean CPU use is 0.790 cores and peak RSS is 850.8 MiB; these are not
whole-system figures or a performance improvement claim. The closed bag is
2,948,017,219 bytes. This is functional acceptance of an inferred scene, not
n3m/FAST_LIO or real-building accuracy acceptance.

The proposed Z-drift explanation was checked with fixed point memberships from
45 paired floor/ceiling scan samples. The saved local points match the frontend
`/cloud_registered_body` input exactly. Headroom evaluated in input body, saved
body, pre-graph odometry, optimized map and an artificial +10 m Z shift differs
by at most 3.1e-14 m. n3m did apply Z corrections to these keyframes (roughly
−0.24 to −1.20 m), but a shared rigid pose correction cannot create this local
headroom difference. A single frontend input scan also contains both roof
heights and their connecting vertical face over a continuous floor.

This comparison does **not** prove the map has no residual drift. The input is
already frontend-deskewed; deskew errors, semantic selection and global map
accuracy remain separate questions. The observed local transition is retained
as reconstruction evidence, not assigned an unverified architectural name.

主体天花板已按连续平面生成候选，并保留局部高度转折。尚未确认的顶面区域继续
保留原几何，未统一压平。原始 PCD、地面版场景和严格观测证据保留；安装版及
Xian 当前窗口不自动替换。运行代码仍为 C++，共用的离线生成器已改名为
`tools/scene/complete_surfaces.py`，旧地面计划可逐项复现原场景。

[Ceiling scene / 天花板场景](../output/ceiling-surfaces-20261024/validated/candidate/scene.json),
[geometry audit / 几何核对](../output/ceiling-surfaces-20261024/geometry-audit.json),
[Noetic E2E](../output/ceiling-surfaces-20261024/ros-route/report.json),
[stage report / 阶段报告](../output/ceiling-surfaces-20261024/report.json),
[pose-invariance audit / 位姿对照](../output/ceiling-surfaces-20261024/pose-invariance.json),
[single input scan / 单帧断面](../output/ceiling-surfaces-20261024/single-scan-cross-section.png),
[native editor / 实际界面](../output/ceiling-surfaces-20261024/editor-validated/report.json).

Open the ceiling candidate on Jammy / 在 Jammy 打开天花板候选：

```bash
cd ~/Projects/simple_rbt
./build/panda-jammy/simple-rbt-console \
  --worker "$PWD/build/panda-jammy/simple-rbt-sim" \
  --assets "$PWD/assets" \
  --config "$PWD/output/ceiling-surfaces-20261024/validated/candidate/session-preview.json"
```

Continuous walls and view panning / 连续墙体与视图平移（2026-10-25）：

The previous observed wall patches exposed missing scan coverage as physical
holes. The reviewed completion replaces 261 wall patches in 36 plane groups
with 29 continuous wall panels. Corridors follow fitted wall spans and reviewed
floor boundaries; the four stairwell faces extend across intermediate heights.
Corners meet, tops extend through adjacent retained higher roofs, and actual
trajectory crossings retain passage openings. The completed scene has 462 nodes
and 1761 collision parts, down from 701 and 2317. Floors, ceilings, four flights,
landings, spawn and the shared stair template are exactly unchanged. The source
PCD hash is unchanged. Wall thickness (0.12 m), hidden continuations, upright
faces and doorway dimensions are inferred; this is not a surveyed CAD model.

The shared offline `complete_surfaces.py` accepts reviewed wall plane frames and
finite outlines. Runtime geometry is still parsed by the existing C++ core.
MuJoCo compiles every part. All 26,377 sampled wall rays hit their intended
completed faces; the previous wall scene hits 4,750 of those sampled rays.
All 378 sphere contact probes produce physical contacts. All 29 recorded path
crossing positions remain open; these include repeated crossings of the same
passage. The earlier floor and ceiling export plans reproduce identical scenes.

The native viewport previously rejected the middle button; Shift + right-drag
already worked. Middle-drag now pans, and **Pan view / 平移视图** enables left-drag
panning while hiding object handles. Actual X11 input verifies that all three
produce the same view translation, a reverse drag restores the view, and right
orbit plus wheel zoom still work. Saving through the editor preserves all 462
nodes, poses, evidence and templates exactly. The installed desktop and Xian's
already-open window are unchanged; use the development binary below.

Final Go2W + direct Noetic acceptance completes all 21 up/down waypoints and
33/33 checks over 260.876 simulation seconds at RTF 1.000. Each 200 Hz IMU has
52,175 samples. LiDAR/PointCloud2 and RGB/depth each have 2608 frames at 10 Hz;
LiDAR keeps 20,000 rays and images keep 320 × 240 pixels. IPC, the ROS receiver
and the closed bag match throughout the run. Maximum tilt is 29.45 degrees.
Core-only mean CPU use is 0.786 cores and peak RSS is 838.4 MiB. These resource
figures exclude GUI/ROS and do not establish a general performance improvement.
This verifies physical traversal and sensor delivery, not n3m/FAST_LIO accuracy.

墙体已由扫描碎片改为连续实体，并保留经过核对的通道入口。楼梯间墙面贯穿中间
高度，顶部连接保留的较高天花板。补齐部分、墙厚和门洞尺寸标为推定；原始点云、
已有地面、天花板和楼梯不变。中键、Shift＋右键及“平移视图”左键拖动均经实际
窗口验证。最终 Go2W／Noetic 上下楼 21/21 路点、33/33 数据验收通过。

[Completed scene / 完成场景](../output/solid-walls-20261024/candidate-final/scene.json),
[wall ray/contact audit / 墙体射线与碰撞](../output/solid-walls-20261024/wall-audit-final.json),
[Noetic E2E](../output/solid-walls-20261024/ros-final/report.json),
[native navigation / 实际视图操作](../output/solid-walls-20261024/editor-confirmed/report.json),
[stage report / 阶段报告](../output/solid-walls-20261024/report.json).

Open the completed wall scene on Jammy / 在 Jammy 打开连续墙体场景：

```bash
cd ~/Projects/simple_rbt
./build/panda-jammy/simple-rbt-console \
  --worker "$PWD/build/panda-jammy/simple-rbt-sim" \
  --assets "$PWD/assets" \
  --config "$PWD/output/solid-walls-20261024/candidate-final/session-preview.json"
```

Use the Scene editor tab. Middle-drag pans; the Pan view checkbox enables
left-drag. Select **Stairs and landings / 楼梯及平台** and **Focus selection /
聚焦所选** to inspect the stairwell. Group visibility only changes the editor
view, so roofs can be hidden for interior inspection without deleting them.

Launcher recovery on the actual desktop / 实际桌面启动恢复（2026-10-25）：

A copied invocation supplied literal arguments such as `" --worker"` and
`" --config"`. Go's flag parser stopped at the first positional argument, but
SpRobo did not reject the remainder. The window therefore restored the older
workspace and selected an installed core from PATH. That core rejected
`--resolve-scene`; the editor incorrectly labelled this a geometry failure,
and startup checks prevented simulation frames.

The launcher now rejects unexpected positional arguments with exit status 2.
Default discovery prefers a sibling core and its nearby assets before PATH and
installed data. Explicit options still override discovery. Preview errors no
longer assert that geometry is invalid, and failed startup checks no longer
show an indefinite waiting message.

The malformed invocation was reproduced and rejected. Running the development
launcher from an unrelated working directory selects its matching core/assets.
The isolated native editor passes all 10 navigation/save checks without explicit
worker/assets flags. The identified disconnected desktop window was closed
normally after preserving the workspace, then reopened with the completed scene.
Live core frames and editor rendering were inspected on Xian's actual display;
all previous saved presets and drafts remained intact. The recovered development
window was left running. The installed package and unrelated simulators were
not changed. Physics, sensor code and scene geometry are unchanged from the
previous full-route acceptance.

[Desktop recovery / 桌面恢复](../output/launch-recovery-20261025/desktop-recovery.json),
[actual simulation view / 实际仿真画面](../output/launch-recovery-20261025/recovered-launch.png),
[actual scene editor / 实际场景页](../output/launch-recovery-20261025/recovered-scene.png),
[paired native E2E / 配套核心界面验证](../output/launch-recovery-20261025/paired-editor/report.json).

启动参数带入空格时，现在会明确报错，不再静默调用另一套核心。开发版优先使用
同目录核心及项目资源；当前桌面的新场景画面已恢复，原有预设和草稿完整保留。

Recorded corridor extent correction / 实走走廊范围修正（2026-10-25）：

The earlier wall completion joined disconnected, similarly oriented upper-storey
wall observations across an unwalked gap without floor support. It produced a
false southwest corridor. Its previous stair E2E and wall contact checks did not
validate this connection. The corrected scene truncates that bridge, retains the
walked southwest stub and its west turn, splits the long inner wall into separate
west/east runs, and retracts unsupported southeast tails. The upper spawn now
lies on the recorded corridor and completed floor. All floor, ceiling, flight,
landing and shared-template geometry is unchanged. No floor is invented across
the rejected connection. The core and launcher binaries are unchanged.

The 10,722-pose optimized source trajectory agrees with all 799 pbstream
keyframes: maximum matched position difference is 5.79e-9 m. All 4,526 classified
upper-storey non-stair samples have floor support. Original PCD, pbstream and
trajectory hashes are unchanged. All 12 probes of the rejected bridge hit the
old walls and miss them in the correction. The remaining completed walls pass
24,282/24,282 ray tests and 390/390 contact probes. The candidate contains 464
nodes and 1754 collision parts; inferred geometry remains labelled.

Go2W + direct Noetic traverses 21/21 waypoints along the recorded upper southwest
turn and back, with 37/37 acceptance checks over 66.022 simulation seconds at
RTF 1.000. Each 200 Hz IMU produces 13,204 samples; LiDAR/PointCloud2 and RGB/depth
each produce 660 frames at 10 Hz. LiDAR retains 20,000 rays and images retain
320 × 240 pixels. IPC, independent ROS receiver and the closed bag match in
counts, timestamps and checked payloads. Maximum tilt is 8.58 degrees and the
return error is 0.120 m. Core-only mean CPU is 0.638 cores and peak RSS is
784.8 MiB; these figures exclude GUI/ROS and are not a comparative benchmark.
The four-flight route was not rerun for this wall-only revision.

The isolated native editor passes all 10 rendering, navigation and save checks,
preserving every node and template. Jammy's identified development window was
reopened with the correction after preserving every prior preset and draft.
Its live core preview is available. The desktop was locked during the final
check, so post-unlock desktop interaction remains for Xian to confirm.

At this stage, the lower-storey audit flagged 102 trajectory samples in two
regions outside the finite floor footprint. The lower reconstruction below
addresses those gaps. The upper correction does not establish whole-
building topology accuracy or n3m localization performance.

上层错误来自补墙范围：两端墙段被跨过无地面的空白区连接。现在保留实走短走廊
及转角，收回错误延伸，并将上层出生点移到有地面支撑的轨迹位置。上层轨迹、墙体
碰撞和 Go2W／Noetic 往返验收通过。当时发现的下层两处地面缺口由后续重建处理；不把本次结果
称为整栋建筑验收。Jammy 已保留新窗口，锁屏解除后的桌面交互待 Xian 确认。

[Corrected scene / 修正场景](../output/trajectory-corridors-20261025/candidate/scene.json),
[recorded path overlay / 真实轨迹叠图](../output/trajectory-corridors-20261025/layout-after.png),
[rejected bridge / 错误连接核验](../output/trajectory-corridors-20261025/rejected-bridge-audit.json),
[Noetic E2E](../output/trajectory-corridors-20261025/ros-upper-west-motion/report.json),
[native editor / 实际编辑器](../output/trajectory-corridors-20261025/editor/report.json).

Open the upper-corridor revision / 打开上层走廊修正版：

```bash
cd ~/Projects/simple_rbt
./build/panda-jammy/simple-rbt-console --config "$PWD/output/trajectory-corridors-20261025/candidate/session-preview.json"
```

Raw-cloud lower reconstruction / 原始点云下层重建（2026-10-25）：

The missing lower walls are present in the original PCD and retained in the
cleaned PCD. All 29,321 unique points supporting the reviewed faces survived
dynamic cleanup. Omission occurred during plane/patch qualification and later
named-plane completion; the earlier scene contained only 18 lower wall plane
groups. Global plane extents also joined separate lower east branches and placed
a transverse wall across the broad southern walking space.

The revised lower layout uses 34 finite wall spans. It restores the central west
face, the broad southwest enclosure, its internal stepped wall, and the southeast
recess and terminal stub. It removes the unsupported east connection and the
false transverse wall. Floor and ceiling support follow the reviewed boundaries
on their existing planes. Added occluded surfaces, upright extension, thickness,
corner joins and the sparsely observed internal north cap remain inferred.
Untraversed rooms visible beyond the main route are not all reconstructed.
Upper-storey objects, stairs, landings and shared templates remain exactly equal.
The original PCD, cleaned PCD, trajectory and pbstream hashes are unchanged.

All 4,935 classified lower non-stair trajectory samples now hit compiled floor
geometry; the previous 102 unsupported samples are resolved. At 606 sampled raw
wall locations, the previous scene supplies 134 nearby wall hits and the revision
supplies 606. This checks source support, not independent holdout accuracy.
All 13,017 completion-face rays and 448 contact probes pass. Probes also confirm
removal of the false east walls and floor. The final scene has 501 nodes and 1706
collision parts. Runtime core and launcher binaries are unchanged.

Go2W + direct Noetic completes 165/165 waypoints over the affected southwest,
central and southeast routes, including the southeast return. Simulated odometry
distance is 178.11 m over 479.464 simulation seconds at RTF 1.000. All 33 data
checks pass: each 200 Hz IMU has 95,892 samples; LiDAR/PointCloud2 and RGB/depth
each have 4794 frames at 10 Hz, retaining 20,000 rays and 320 × 240 images.
IPC, independent receiver and the closed bag match in counts, timestamps and
checked payloads. Maximum tilt is 9.11 degrees. Core-only mean CPU is 0.732 cores
and peak RSS is 845.9 MiB; GUI/ROS are excluded. This is traversal and transport
acceptance, not localization accuracy or a fresh stair-route result.

The isolated native editor passes 10/10 render/navigation/save checks and retains
all 501 nodes. Jammy's identified previous window was closed normally, preserving
all saved presets and drafts. The new window loads the reviewed scene and its
core preview is available. Jammy remained locked; post-unlock desktop interaction
still needs confirmation. Existing installed binaries were not replaced.

下层漏墙的点仍在原始及清理后的 PCD 中，遗漏发生在场景提取与补墙阶段。现按
真实墙点和轨迹恢复南侧宽区域、折角墙及东南端部，移除错误连通。原先两处区域中
102 个轨迹采样点缺少地面支撑的问题已解决。约 178 米 Go2W／Noetic 路线通过，
原始数据、上层和楼梯保留。
补齐的遮挡区域与少量封口仍标为推定。新窗口的场景名为“建筑 · 核对后走廊”。

[Scene / 场景](../output/lower-raw-corridors-20261025/candidate/scene.json),
[original cloud overlay / 原始点云叠图](../output/lower-raw-corridors-20261025/raw-after.png),
[raw-face audit / 原始墙点核验](../output/lower-raw-corridors-20261025/raw-face-audit.json),
[floor support / 地面支撑](../output/lower-raw-corridors-20261025/layout-audit.json),
[Noetic E2E](../output/lower-raw-corridors-20261025/ros-lower-route/report.json),
[native editor / 原生编辑器](../output/lower-raw-corridors-20261025/editor/report.json).

Open the lower reconstruction revision / 打开下层重建阶段的场景：

```bash
cd ~/Projects/simple_rbt
./build/panda-jammy/simple-rbt-console --config "$PWD/output/lower-raw-corridors-20261025/candidate/session-preview.json"
```

Complete upper-corridor review / 上层走廊完整核对（2026-10-25）：

The combined scene now uses 43 upper wall spans: 40 faces fitted directly to the
original PCD and three short inferred connectors. It restores the southwest end,
north bay/lobby boundaries, southeast step/recess, east T stub and northeast
end. The unsupported north connection is removed together with its floor and
ceiling. A fitted southeast wall keeps its original line and joins the retained
stair corner through a short connector; forcing its endpoint onto that corner
had displaced the entire face. The accepted lower scene, stair/landing objects
and shared templates are unchanged. Runtime binaries are unchanged.

The upper fits use 63,891 unique original support points. One lies in the earlier
dynamic-removal mask; proximity alone does not distinguish a static wall point
from a near-wall dynamic return. The cleaned PCD is unchanged. Original PCD,
cleaned PCD, trajectory and pbstream hashes all match the previous stage.
Wall thickness, upright completion, occluded support and sparse bay caps remain
inferred. Minor recesses are idealized; incompletely observed rooms outside the
traced corridors are not all reconstructed.

All 4,526 upper non-stair trajectory samples have compiled floor support and a
completed roof. The 950 sampled raw wall positions all hit the new faces
(previously 354); this checks source support, not independent holdout accuracy.
All 11,061 wall rays and 572 contact probes pass. Negative probes confirm the
removed north walls, floor and roof. Retained observed ceiling patches at the
central lobby remain up to 6.83 cm below the inferred roof; actual route clearance
is 2.382–2.450 m. The combined scene contains 557 nodes and 1620 collision parts.

Go2W + direct Noetic completes 325/325 waypoints along the recorded upper route,
including west and northeast returns and the eastern terminal stub: 354.56 m of
simulated odometry over 887.978 simulation seconds, RTF 1.000, maximum tilt
9.81 degrees. All 35 whole-route checks pass. Each IMU has 177,595 samples at
200 Hz; LiDAR/PointCloud2 and RGB/depth each have 8879 frames at 10 Hz, retaining
20,000 rays and 320 × 240 images. Native counts match the independent ROS receiver
and closed bag; complete ROS payload hashes and timestamps match without trimming.

The first harness timed out during offline parsing of the closed 12.9 GB bag,
before saving its in-memory IPC copy. This is an audit-tool failure, not a passed
full-route IPC comparison. The closed native summary, receiver and bag survived
and were checked offline. The harness now saves IPC evidence and closes the
producer before parsing. A separate six-waypoint, 18.088-second run passes all
33 native/IPC/ROS/bag checks with exact IMU values and image bytes. Full-route IPC
bytes remain unavailable. This stage does not add a stair-route or n3m
localization-accuracy result.

The isolated native editor passes 10/10 render/navigation/save checks, retaining
all 557 nodes and templates. Native overview images show the combined scene and
upper-only editor view. Jammy's previous window was closed normally after
preserving every saved preset and draft. The new window loads “Building · both
storeys reviewed / 建筑 · 双层核对场景” and its core preview is ready. Jammy remained
locked; post-unlock desktop interaction still needs Xian's confirmation.

上层已恢复有原始墙点支撑的端墙、凹入区域、东南折角及东北端部，并移除北侧错误
连接及其悬空地面、顶面。下层与楼梯保持原样。约 355 米 Go2W／Noetic 路线走通，
整段 ROS 消息与关闭后的 bag 一致。长测的临时 IPC 副本因核对脚本超时未保存，已
修正保存顺序并补跑独立短测；不将短测写成全程 IPC 验证。未观测完整的外侧房间
仍不作为完整室内模型。下一步由 Xian 解锁桌面，核对实际建筑布局。

[Combined scene / 双层场景](../output/upper-raw-corridors-20261025/candidate/scene.json),
[review report / 核对报告](../output/upper-raw-corridors-20261025/report.json),
[raw-cloud overlay / 原始点云叠图](../output/upper-raw-corridors-20261025/raw-after.png),
[Noetic full route / Noetic 全路线](../output/upper-raw-corridors-20261025/ros-upper-route/report.json),
[supplementary IPC check / IPC 补充检查](../output/upper-raw-corridors-20261025/ros-ipc-check/report.json),
[native editor / 原生编辑器](../output/upper-raw-corridors-20261025/editor/report.json),
[combined overview / 双层总览](../output/upper-raw-corridors-20261025/overview-final/whole-scene.png),
[upper-only view / 仅显示上层](../output/upper-raw-corridors-20261025/overview-final/upper-only.png).

Open the current combined scene / 打开当前双层场景：

```bash
cd ~/Projects/simple_rbt
./build/panda-jammy/simple-rbt-console --config "$PWD/output/upper-raw-corridors-20261025/candidate/session-preview.json"
```

Use Scene editor; middle-drag or Shift + right-drag pans, right-drag orbits, and
the wheel zooms. Hiding “Lower storey” affects the editor view only. Maximize the
window and zoom out for an overview. / 进入“场景编辑”：中键或 Shift＋右键平移，
右键旋转，滚轮缩放。隐藏“下层”仅影响编辑视图；最大化窗口后缩小，可查看总览。
