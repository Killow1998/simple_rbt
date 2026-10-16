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

Only existing physical terrain presets are selectable. Arbitrary scene import and live mount changes during motion are outside this implementation. `tracked` velocity feedback and all robot dynamics remain the existing documented simulator approximations.

当前选择已有物理地形预设，不提供任意场景导入或运动中的外参热修改。`tracked` 速度反馈和各机器人动力学保留原有、已说明的仿真近似。

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
