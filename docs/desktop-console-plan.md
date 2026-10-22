# Desktop console and session architecture / 操作台与会话架构

Status: SpRobo 0.2.1 released after all GitHub acceptance jobs passed and installed on Jammy. Framework name remains `simple_rbt`. / 状态：SpRobo 0.2.1 已通过全部 GitHub 验收、发布并安装到 Jammy；框架名称保持 `simple_rbt`。[Release / 发行](https://github.com/Killow1998/simple_rbt/releases/tag/v0.2.1) · [Successful CI / 通过的 CI](https://github.com/Killow1998/simple_rbt/actions/runs/38708515822)。
Date / 日期：2026-10-15。

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

Starting enters the SpRobo operator screen with first/third views, held joystick, command-source choice, stop/pause and measured core CPU/RAM/RTF. Joystick release and window focus loss clear manual commands only. Program IPC/ROS commands continue while unfocused, with the same 0.5-second deadman and owner-disconnect stop. Returning to configuration stops the old worker/adapters and starts a fresh private session. User presets and results keep their existing XDG paths; uninstall preserves them.

开始后进入 SpRobo 操作台，提供第一／第三视角、按住运动的摇杆、指令来源、停车／暂停和实测核心 CPU／RAM／RTF。松开摇杆与失焦仅清零手动指令；程序 IPC／ROS 不受窗口焦点影响，仍保留 0.5 秒超时和控制端断开停车。返回配置时关闭旧 worker／适配器，建立新的私有会话。预设与结果保留既有 XDG 路径，卸载保留用户数据。

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

Core, console and either ROS adapter can have independent lifetimes. Each ROS adapter publishes the core's actual samples directly to its peers; neither ROS runtime is linked into the host core/console. Docker images pin public official bases. Selecting both adapters permits both outputs, with one explicit motion-command owner. Command timeout is 0.5 s. Stop revokes a prior ROS owner and leaves active physics at zero command; Pause freezes simulation time.

核心、操作台和 ROS 适配器分别拥有生命周期。适配器直接向各自节点发布核心的实际样本；宿主核心／界面不链接 ROS。Docker 镜像固定公开官方基础镜像。可同时启用两种输出，但运动只接受一个显式选中的来源；超时为 0.5 秒。停车撤销先前 ROS 来源，让物理以零指令继续；暂停冻结仿真时间。

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

Next: push the reviewed source and let the existing CI build/test both variants.
Software full-panorama rendering needs separate throughput work before claiming
real-time operation; package installation is a separate deployment step.

后续：推送已审阅代码，由既有 CI 构建／验证两种版本。软件全景吞吐需单独优化后
再宣称实时；本轮不以代码推送冒充本机安装更新。
