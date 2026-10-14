# Direct ROS 1 / ROS 1 直连

[English](#english) | [简体中文](#简体中文)

## English

The simulator runs its C++ core and thin roscpp wrapper on the Jammy host. A Noetic container supplies the ROS master and independent test clients. Commands and sensor messages use TCPROS directly. The test image contains the shared Livox messages and reuses your existing Noetic development image, including its compiler and message generators.

```bash
# Prepare a test image from an existing local Noetic development image.
.venv/bin/python tools/test_ros1.py --base-image YOUR_NOETIC_IMAGE --build-only

# Prepare the host roscpp build; reuse the existing simulator SDK/assets.
./setup.sh --native-only --ros1

# Test every registered robot, or select a subset.
.venv/bin/python tools/test_ros1.py --skip-build
.venv/bin/python tools/test_ros1.py --skip-build --robots ranger_mini \
  --control-mode tracked --output output/ros1-selected
```

`SIMPLE_RBT_NOETIC_IMAGE` selects another image for setup. The local SDK is under `build/ros1-sdk`, generated messages under `build/interfaces/ros1`, and the executable under `build/ros1`. Export retains the required Noetic libraries, matching Boost headers and license notices. A small Boost 1.71 header compatibility change preserves the stack-size bounds check on Jammy glibc; identical license content shares disk storage. No system ROS installation is changed.

For normal use, start a ROS master and run:

```bash
ROS_MASTER_URI=http://127.0.0.1:11311 ./run.sh --robot go2w --ros1 \
  --sensors full --camera-hz 10 --backend cuda
```

The wrapper subscribes to `/<robot>/cmd_vel` and stops after 0.5 seconds without a fresh command. Topics, frames, raw/SI IMU conventions and Livox fields match the ROS 2 wrapper. Physics, model inference, sensor sampling and rendering stay in the shared core. A standalone build and direct ROS 2 build are described in [runtime](runtime.md).

Tests use a private master port, default 11383, configurable with `--master-port`. An occupied port fails before launch. The runner removes its own container on completion or failure. CUDA LiDAR and camera rendering execute on the host; the Noetic container has no GPU devices. `--backend cpu` selects CPU LiDAR. The probe waits for the simulator's two-second settling interval before sending movement commands.

Each case verifies forward/reverse motion, namespace isolation and timeout stopping; Livox and PointCloud2 payload bytes; raw/SI IMU values and frames; rigid MID360 IMU/LiDAR extrinsics; odometry, TF, camera intrinsics and four image streams; complete periodic sensor delivery and real-time pacing. Image bytes are compared with a separate Noetic receiver of actual publications. The cases use flat ground; terrain routes are tested with `tools/validate_full_run.py`.

## 简体中文

仿真核心与薄 roscpp 适配层都在 Jammy 宿主机以 C++ 运行。Noetic 容器提供 ROS master 和独立测试客户端，通过 TCPROS 直接收发指令与传感器消息。测试镜像复用已有 Noetic 开发镜像，只构建同一份 Livox 消息定义；基础镜像需要编译器和消息生成工具。

先按上面的命令准备测试镜像、执行 `./setup.sh --native-only --ros1`，再运行 `tools/test_ros1.py`。`SIMPLE_RBT_NOETIC_IMAGE` 可选择其他准备镜像。本地 SDK、消息和可执行程序分别位于 `build/ros1-sdk`、`build/interfaces/ros1`、`build/ros1`。导出保留需要的 Noetic 库、匹配的 Boost 头文件及版权说明；Boost 1.71 的局部兼容补丁保留栈大小检查，适配 Jammy glibc。相同版权文本共用磁盘存储，不修改宿主系统的 ROS 安装。

正常使用时先启动 master，再用 `ROS_MASTER_URI=... ./run.sh --robot go2w --ros1`。适配层订阅 `/<robot>/cmd_vel`，0.5 秒无新指令后停车。消息字段、传感器话题、坐标系以及原始／SI IMU 约定与 ROS 2 适配层一致。物理、模型推理、采样和渲染由共用核心负责；无 ROS 与 ROS 2 直连构建见[运行说明](runtime.md)。

测试默认 master 端口为 11383，可用 `--master-port` 修改；已占用时先报错。测试结束或失败后清理自己的容器。GPU 雷达和相机在宿主机执行，Noetic 容器不使用 GPU；`--backend cpu` 可选择 CPU 雷达。运动检查等待模拟时钟越过两秒初始化阶段，再发送指令。

检查包括前进、倒退、命名空间隔离、超时停车；Livox／PointCloud2 有效载荷字节；两种 IMU 数值与坐标系、MID360 固定外参；里程计、TF、相机内参和四路图像；周期数据完整性与实时节奏。图像字节与独立 Noetic 接收器记录的实际发送帧比较。此处使用平地，多地形路线由 `tools/validate_full_run.py` 验证。

## Verified on Jammy / Jammy 验证结果 — 2026-10-14

All seven robots passed 46/46 direct ROS 1 checks in native mode with full sensors, CUDA LiDAR and 10 Hz cameras. RTF was 0.999786–0.999992. Ranger Mini also passed 46/46 in tracked mode: +0.30 / -0.25 m/s commands measured +0.306 / -0.257 m/s, with speed near zero after timeout.

七台机器人在 native 模式、完整传感器、CUDA 雷达和 10 Hz 相机下，各通过 46/46 项 ROS 1 直连检查；RTF 为 0.999786–0.999992。Ranger Mini 的 tracked 模式也通过 46/46：+0.30／-0.25 m/s 指令实测 +0.306／-0.257 m/s，超时停车后速度接近零。

The outer-feedback stopping fix remains in the core: a full zero target passes to the physical brakes on pure wheel chassis and clears feedback memory. Go2/Go2W retain stance feedback; geometry, torque bounds and steering alignment protection are unchanged.

纯轮式底盘在完整零指令下直接执行物理刹车并清除反馈记忆，解决 Ranger Mini 停车误差引发舵轮追逐、持续禁止驱动的问题。Go2/Go2W 保留站立反馈，几何、扭矩限制和舵轮对齐保护没有修改。

Evidence / 本机证据：[seven ROS 1 cases / 七台 ROS 1 验收](../output/ros1-direct-final-20261014/summary.json), [Ranger tracked](../output/ros1-ranger-tracked-final-20261014/summary.json), [direct ROS 2 hills / ROS 2 直连起伏路线](../output/ros2-wrapper-hills-final-20261014/report.json), [29 core comparisons / 29 组核心对照](../output/ros-wrapper-core-parity-20261014/summary.json).
