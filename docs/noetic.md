# Noetic Docker E2E / Noetic Docker 端到端测试

[English](../README.md) | [简体中文](../README.zh-CN.md)

The simulator runs on the Jammy host with C++ and ROS 2 Humble. The test container runs ROS 1 Noetic and a pinned Foxy `ros1_bridge`; it publishes robot commands and receives the simulator's actual messages.

The bridge is compiled with `CustomMsg` and `CustomPoint` generated from the project's canonical definitions. An explicit package mapping handles the Livox package name; the included QoS patch enables reliable sensor reception and transient-local reception/ROS 1 latching for `/tf_static`. `/clock` retains best-effort delivery. The test includes the Livox message, PointCloud2, both IMU outputs, odometry, TF, camera intrinsics, perspective/panoramic RGB and depth, and simulation time.

## Run

Prepare the native runtime first with `./setup.sh`. Then:

```bash
# Build the test image using an existing local Noetic image.
.venv/bin/python tools/test_noetic.py --base-image YOUR_NOETIC_IMAGE --build-only

# Test every registered robot with the prepared image.
.venv/bin/python tools/test_noetic.py --skip-build

# Select robots and save the results in a new directory.
.venv/bin/python tools/test_noetic.py --skip-build --robots fr_mini go2w \
  --output output/noetic-selected
```

The default base image is `ros:noetic-ros-base`; `--base-image` selects your local image. The derived test image reuses its layers. Image preparation adds Foxy and builds the bridge with two compiler jobs. Subsequent tests reuse the image.

The test shares host networking and IPC for local ROS communication. It uses ROS domain 90 and master port 11383, configurable with `--domain` and `--master-port`. Existing services on the requested master port cause a failure. Test containers are removed on completion or failure.

CUDA and camera simulation execute on the host. The Noetic container needs no GPU devices. Use `--backend cpu` to select CPU LiDAR; the complete cameras still require the host rendering environment.

The Noetic cases use flat terrain; terrain routes are checked by `tools/validate_full_run.py`. The test defaults to `--control-mode native`; `--control-mode tracked` explicitly selects simulator feedback. The test verifies forward/reverse motion, namespace isolation and timeout stopping. Received LiDAR and PointCloud2 are compared with saved native output bytes. Four image streams are compared by timestamp and exact bytes against an independent Humble receiver of the actual published frames; saved PNGs are fresh renders and are not the wire-data oracle. Periodic sensor counts must match every scheduled frame between the first and last received timestamps. It also checks IMU values, camera intrinsics, MID360 IMU/LiDAR extrinsics and real-time pacing. Results go to `output/noetic-*` or `--output`; a failed check returns a nonzero exit status.

## 简体中文

模拟器在 Jammy 宿主机上运行 C++ 和 ROS 2 Humble。测试容器运行 ROS 1 Noetic 与固定版本的 Foxy `ros1_bridge`，发送机器人速度指令并接收真实的模拟器消息。

桥接器使用项目现有定义生成 `CustomMsg`、`CustomPoint`，显式映射 Livox 包名；附带 QoS 补丁启用可靠的传感器接收，并为 `/tf_static` 启用持久接收和 ROS 1 锁存；`/clock` 保持尽力传输。测试覆盖 Livox 消息、PointCloud2、两种 IMU 输出、里程计、TF、相机内参、普通/全景 RGB 与深度，以及模拟时钟。

先用 `./setup.sh` 准备原生程序，再执行上面的命令。`YOUR_NOETIC_IMAGE` 替换为已有的 Noetic 镜像名；不指定时使用 `ros:noetic-ros-base`。派生镜像复用已有镜像层，安装 Foxy 并以两个编译任务构建桥接器。后续测试使用 `--skip-build` 复用测试镜像。

测试默认使用 ROS domain 90、master 端口 11383，可分别用 `--domain`、`--master-port` 修改。共享宿主网络和 IPC 以传输本机 ROS 数据；所选 master 端口已被占用时会报错，结束或失败后都会清理测试容器。

CUDA 雷达和相机在宿主机执行，Noetic 容器不需要 GPU 设备。`--backend cpu` 可改用 CPU 雷达；完整相机仍需要宿主渲染环境。

Noetic 案例使用平地；多地形路线由 `tools/validate_full_run.py` 验证。测试默认使用 `--control-mode native`，可显式指定 `--control-mode tracked` 选择模拟器反馈。测试检查前进、倒退、命名空间隔离和指令超时停车；将雷达与 PointCloud2 和 C++ 保存的数据逐字节比较；四路图像按时间戳与独立 Humble 接收器记录的实际发送帧逐字节比较，不使用结束时重新渲染的 PNG 作为传输原件，并要求各周期传感器在首尾接收时间戳之间的所有帧完整送达，并检查 IMU 数值、相机内参、MID360 IMU/雷达外参和实时节奏。结果写入 `output/noetic-*` 或指定的新输出目录，任一检查失败均返回非零状态。

Bridge source / 桥接器来源：[ros2/ros1_bridge](https://github.com/ros2/ros1_bridge/tree/689a932499befbd1ec3cb273a1054430e55a43c3).

## Ranger Mini tracking fix / Ranger Mini 反馈控制修复 — 2026-10-14

The previous failure came from outer feedback converting a zero target into tiny motion commands during settling. Swerve modules kept changing direction and the alignment gate withheld wheel drive. The same model worked in native mode. Pure wheel chassis now pass a full zero target to their physical brakes and clear feedback memory; Go2/Go2W retain stance feedback. Geometry, torque limits and steering alignment protection are unchanged.

此前的故障来自外层反馈：稳定阶段的零速度指令被微小误差变成运动指令，舵轮不断改变方向，对齐保护持续阻止驱动。同一个模型在 native 模式正常。现在纯轮式底盘收到完整零指令时，直接执行物理刹车并清除反馈记忆；Go2/Go2W 保留站立反馈。几何、扭矩限制和舵轮对齐保护没有修改。

The Noetic tracked probe now measures +0.309 m/s for a +0.30 command, -0.256 m/s for a -0.25 command, and about 2e-9 m/s after command timeout. Ranger Mini, Scout Mini, FR Mini and Ridgeback each passed 46/46 checks with full sensors and 10 Hz cameras; RTF was 0.999942–0.999964. The C++ physical regression also checks lateral, spin, compound motion and restart after braking. CTest passed 11/11, C++/Python trajectory comparisons passed 10/10, and the affected Python controller/integration tests passed 89/89, including the Go2W ramp regression.

Noetic 的 tracked 复测：+0.30 m/s 指令实测 +0.309 m/s，-0.25 m/s 指令实测 -0.256 m/s，指令超时停车后约 2e-9 m/s。Ranger Mini、Scout Mini、FR Mini、Ridgeback 在完整传感器、10 Hz 相机配置下各通过 46/46 检查，RTF 为 0.999942–0.999964。C++ 物理回归还覆盖横移、原地转向、组合运动和刹车后重启；CTest 11/11、C++/Python 轨迹对照 10/10、相关 Python 控制器及集成测试 89/89 均通过，包含 Go2W 坡道回归。

Local evidence / 本机证据：[tracked ROS report / 反馈模式 ROS 报告](../output/noetic-tracked-fix-20261014/summary.json), [ablation / 消融对照](../output/ranger-tracked-fix-20261014/ablation-before.json), [C++ physical phases / C++ 物理阶段](../output/ranger-tracked-fix-20261014/physics-phases.jsonl), [trajectory parity / 轨迹等价](../output/ranger-fix-parity-20261014/summary.json).

Ranger Mini also passed 31/31 formal route checks on both standard ramp and continuous hills at a 0.6 m/s target in tracked mode, with full sensors and ROS commands. RTF was about 1.000; terminal mean speed was below 3e-6 m/s. These checks establish the tested simulator routes, not hardware performance or untested terrain levels.

Ranger Mini 在 tracked 模式、0.6 m/s 目标速度、完整传感器和 ROS 指令下，标准坡道与连续波浪地形均通过 31/31 正式路线检查；RTF 约 1.000，终点停车平均速度低于 3e-6 m/s。结果仅覆盖已测模拟器路线，不代表实车性能或未测难度。

Route evidence / 路线证据：[ramp / 坡道](../output/ranger-fix-ramp-20261014/report.json), [hills / 连续起伏](../output/ranger-fix-hills-20261014/report.json).

## Verified on Jammy / Jammy 验证结果 — 2026-10-14

All seven robots passed 46/46 checks in `native` mode with full sensors, 10 Hz cameras and CUDA LiDAR. RTF ranged from 0.999751 to 0.999961. Periodic messages were complete between first/last received timestamps, and all four image streams matched the actual Humble publications byte for byte. The native CTest suite also passed 10/10. Test sources remained unchanged during the seven-case run.

七台机器人在 `native` 模式、完整传感器、10 Hz 相机和 CUDA 雷达配置下，各通过 46/46 检查；RTF 为 0.999751–0.999961。首尾接收时间戳之间的周期消息完整送达，四路图像与实际 Humble 发布数据逐字节一致，现有 CTest 也通过 10/10。七台验收过程中测试源码未变。

The derived Docker image reuses the base layers and adds about 71.4 MiB of image data, excluding build cache and test output. A single live-container sample used 121.8 MiB RAM; this is not a peak-memory bound or an A/B result.

派生 Docker 镜像复用原有镜像层，新增镜像数据约 71.4 MiB，不含构建缓存与测试输出。一次运行中容器内存采样为 121.8 MiB；该值不是峰值上限，也不是 A/B 结果。

Local evidence / 本机证据：[seven-case report / 七台验收报告](../output/noetic-acceptance-20261014/summary.json).
