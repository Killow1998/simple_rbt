# Mapping tutorials / 建图教程

The native C++ simulator publishes sensors through an independent Noetic or Humble
wrapper. FAST-LIO and FAST-LIVO2 run in a separate optional container. They receive
LiDAR, IMU and, for FAST-LIVO2, RGB. The simulator and desktop do not load ROS or
an estimator library. No ROS 1/ROS 2 bridge is used.

原生 C++ 仿真器通过独立 Noetic 或 Humble 适配器发布传感器。
FAST-LIO 与 FAST-LIVO2 在单独的可选容器内运行。框架不依赖 ROS 或估计器，
两个 ROS 版本分别直连 IPC。教程首先使用 Go2W，也提供四旋翼示例。

## Prepare once / 一次性准备

Prepare the normal ROS adapter using the [ROS guide](noetic.md). Algorithm builds
use public pinned ROS base images and source revisions in `tutorials/sources.json`.
FAST-LIO has a public source URL. The qualified FAST-LIVO2 native-wrapper commit
is currently a local revision: supply a checkout containing that exact commit or
an exported build context. A public upstream clone alone does not contain it.

先按 [ROS 指南](noetic.md)准备适配器。算法镜像使用公共 ROS 基础镜像和固定源码版本，
不依赖私人 Jammy 镜像。当前 FAST-LIVO2 原生双 wrapper 版本尚未公开发布；需要包含
固定提交的源码，或已有的完整构建目录。不能直接用上游主分支替代。

From the repository root, substitute the two source checkout paths:

```sh
python3 tools/prepare_tutorials.py \
  --fast-lio /path/to/FAST_LIO --fast-livo2 /path/to/FAST_LIVO2 \
  --output build/tutorial-context
docker build -t simple-rbt-tutorials:noetic build/tutorial-context
docker build --build-arg ENTRY=humble \
  --build-arg ROS_IMAGE=ros:humble-ros-base-jammy@sha256:6b4abb5340e430c632bcf4710fb34996df4f80e502993e5755f8fc59a39c7b2e \
  -t simple-rbt-tutorials:humble build/tutorial-context
```

The preparation script exports committed objects without changing source working
trees. Its output is self-contained and records source archive hashes. Build only
the ROS version you use. Docker and image preparation are optional for simulator-only
operation. No Python frontend runs during simulation; preparation and E2E scoring
use Python. Runtime shell scripts only start native executables and load ROS settings.

准备脚本只导出固定提交，不修改源码工作区。生成目录可搬到另一台电脑构建。
只需准备实际使用的 ROS 版本。模拟运行、标定生成、估计器和观测程序均为 C++；
界面为 Go。Python 仅用于准备和独立验收。

## Use SpRobo / 使用操作台

1. Open **Tutorials**, load **Go2W example** or **flight example**.
2. Choose FAST-LIO or FAST-LIVO2 and Noetic or Humble. The required adapter,
   LiDAR and RGB settings are selected. Use the textured corridor example first.
3. Return to configuration, check mounts and start the session. Keep stationary
   during initialization. Manual input unlocks when estimated poses arrive.
4. Drive or fly. The tutorial page shows pose count, simulation-time lag,
   estimated XYZ and the output directory. **Stop frontend** stops only the
   algorithm process; closing the session also stops its owned tutorial container.

在 **教程** 页加载示例，选择算法及 ROS 版本，再返回配置页面启动。
初始化时保持静止，出现估计位姿后再运动。四旋翼需随后点击起飞。
“正在接收估计位姿”只表示输出正常到达，不表示精度已通过验收。

## Input contract / 输入约定

| Input / 输入 | Contract / 约定 |
|---|---|
| `/livox/lidar` | Livox CustomMsg, per-point capture offsets / 每点采样偏移 |
| `/livox/imu_si` | m/s² and rad/s, rigid LiDAR IMU / 与雷达刚性耦合 |
| `/camera/image_raw`, `/camera/camera_info` | RGB optical frame and calibration / 光学坐标与内参 |
| `/clock` | Simulation time, `use_sim_time=true` / 仿真时间 |
| `/ground_truth/odom` | Independent scoring only, never frontend input / 仅独立评分 |

The C++ `simple-rbt-tutorial-config` tool reads the current IPC handshake, actual
RGB dimensions and camera field of view. It writes `frontend.json`, `camera.json`
and `contract.json`. Extrinsics include the camera optical rotation and the rigid
LiDAR-to-IMU offset. FAST-LIVO2 currently requires a base-mounted RGB camera;
a moving wrist camera needs a different estimator contract.

标定工具从实际运行会话读取安装位姿和图像尺寸，计算雷达—IMU、雷达—相机外参。
修改外参后重新启动会话即可重新生成。FAST-LIVO2 教程当前要求相机固定于机体，
不能直接使用运动机械臂的腕部相机。真值 TF 必须关闭。

Noetic frontend forks expect `livox_ros_driver/CustomMsg`; the optional image
includes only the matching message definitions alongside driver2. Their identical
ROS 1 wire schema is exercised by the live tutorial tests. No hardware driver
runs. Humble uses `livox_ros_driver2` directly and UDP for container communication.
The wrapper paces fragments at 64 KiB/ms to avoid bursts overflowing ordinary
socket buffers. Sensor rates and timestamps are unchanged; host sysctl tuning
is not required. The XML profile uses the [Fast DDS 2.6 flow controller](https://github.com/eProsima/Fast-DDS/blob/v2.6.12/test/unittest/xmlparser/test_xml_profiles.xml).

FAST-LIO explicitly sets a 500 m map window with 100 m detection range. This avoids
omitting a launch-file override when running its node directly. Parameters are
reviewable in every run directory; estimator source and scoring thresholds are unchanged.

## Evidence and storage / 验收与存储

Each run stores generated parameters and estimated poses; it does not record raw
images/clouds by default. The observer batches trajectory writes and flushes once
per second. At 100 MB it stops recording and displays a warning; the frontend
continues. Frontend diagnostic logs use their existing bounded logger. Invalid
poses and stale estimates appear as explicit states.

默认不保存原始点云和图像。估计轨迹按秒刷新，达到 100 MB 时停止记录并提示。
算法诊断日志沿用其容量限制。教程数据属于实验产物，不自动删除已有实验。

Live E2E entry point (development checkout and optional image required):

```sh
.venv/bin/python tools/test_tutorial.py --ros ros1 --algorithm fast-lio \
  --build build/desktop --adapter-image simple-rbt-ros1:noetic \
  --output output/tutorial-go2w-noetic
```

Use `--algorithm fast-livo2`, `--ros ros2` with its Humble adapter image, or
`--robot skydio_x2` for the other cases. Tests use independent truth for route
commands and scoring only. Alignment is one fixed first-frame SE(3), without
trajectory fitting or scale adjustment. The gate remains 1 m / 5°, completed
route, recent final estimate and real-time factor above 0.97.

测试按首帧做一次固定对齐，不拟合整段轨迹。通过只证明该模拟输入与配置；
不证明真实场景、重定位或 Avia/N3M 地图恢复已通过。
See the dated [validation record](desktop-console-plan.md#quadrotor-and-algorithm-tutorials--requirements-2026-10-28).
