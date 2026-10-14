# simple_rbt C++ runtime

`../run.sh` 现在启动 `build/native/simple-rbt-native`。物理、控制器、LiDAR、相机及 ROS 2 都在 C++ 程序内执行，不启动 Python，也不使用 Warp／LLVM runtime。原 Python 实现由 `../reference/python/run.sh` 保留，供独立对照。

支持 7 台机器人：`go2`、`go2w`、`diffdrive`、`scout_mini`、`fr_mini`、`ridgeback`、`ranger_mini`。Go2 的 RobotLab／HIMLoco／legacy 是控制器选项，不另算机器人。轮式场景使用 `flat`、`ramp`、`hills`；`tracked` 默认模式含理想模拟器速度／航向反馈，`native` 关闭这层外部反馈，保留各底盘自身控制器。

纯轮式底盘在 `tracked` 模式收到完整零速度指令时，直接交给底盘刹车并清除反馈积分；不会用停车误差生成新的转向指令。Go2／Go2W 保留站立所需的零速度反馈。Ranger Mini 短时倒车失速已修复，实测结果见 [Noetic 测试说明](noetic.md)。

以下命令从 repository 根目录执行：

```bash
# Jammy 已有 SDK、模型及策略资产时，直接编译
./setup.sh --native-only

# 透视＋全景相机、逐点运动 LiDAR，以及 GUI
./run.sh --robot fr_mini --cmd .3 0 0 \
  --sensors full --backend cuda --camera-hz 10 --realtime --viewer --duration 10

# 纯 CPU LiDAR；不开相机或 GUI
./run.sh --robot scout_mini --cmd .3 0 0 \
  --sensors lidar --backend cpu --duration 10

./run.sh --help
```

首次准备 SDK／模型可执行 `./setup.sh`，它会准备锁定的 `.venv` 套件与策略资产，再构建 C++。C++ 直接连结 `.venv` 内的 MuJoCo／ONNX Runtime 共享库与 MuJoCo headers；套件目录需要保留，但执行时不需要 Python 解释器。

构建使用 Jammy 现有 GCC（C++17）、CMake ≥3.22、CUDA 12.9、ROS 2 Humble／`livox_ros_driver2` 讯息套件，以及 EGL／OpenGL、GLFW、PNG、TinyXML2、OpenSSL、pkg-config。默认 CUDA compiler 是 `/usr/local/cuda-12.9/bin/nvcc`、host compiler 是 `/usr/bin/g++`，此次重新构建沿用已运行版本的 CUDA 架构 cache `52`；可在已载入 ROS 环境的 CMake 配置中，用 `-DCMAKE_CUDA_ARCHITECTURES=<架构号>` 明确指定其他架构（例如此台机器的 `120`）。compiler 路径也可由同名 CMake cache 变数覆写。目前构建包含 CUDA，即使执行时选择 CPU LiDAR 也需要 CUDA toolkit。

`setup.sh` 先从 `ros2_interfaces/livox_ros_driver2` 构建原样讯息定义，安装到 `build/ros_interfaces/install/livox_ros_driver2`，再构建 `build/native`。不需要 Livox 硬体 SDK、PCL 或原 workspace driver。`tools/ros_env.sh` 设定本地讯息 prefix；可以用 `SIMPLE_RBT_LIVOX_PREFIX` 明确覆写。`SIMPLE_RBT_BUILD_JOBS` 默认为 2，`SIMPLE_RBT_CPU_THREADS` 为 1，`ROS_DOMAIN_ID` 为 83，`ROS_LOCALHOST_ONLY` 为 1。

加入 `--ros2` 后，速度指令改由 `/<robot>/cmd_vel` 接收，取代 `--cmd`；超过 0.5 s 没有新指令就停车。GUI 支持滑鼠调整视角及 Esc 关闭。`--realtime` 控制即时节奏，并不保证任意场景都能达到即时性能。

`--sensors physics|lidar|full` 分别选择物理／加 LiDAR／再加双相机；IMU 都保留。默认是 `lidar`、CPU backend、rolling 扫描；相机开启后默认 1 Hz，可用 `--camera-hz 10`。MID360 设定为 10 Hz、每帧 20,000 条原始 CSV 射线、200 kHz 合成点时序，IMU 为 200 Hz。`rolling` 逐点插值刚体 pose，输出各点采集当刻的局部座标，尚未 deskew；场景仅含静态环境且排除完整机器人自身遮挡。CUDA 支持目前地形的 plane／box，其他几何会拒绝，不会悄悄略过。`snapshot` 是帧末快照，offset 全零，与 rolling 在 metadata 中分开标示；`--backend warp` 仅是原生 CUDA 的相容名称。

| 文件 | 职责 |
| --- | --- |
| `src/core.cpp`、`src/scene.cpp` | 物理步进、模型／地形装配与感测点 |
| `src/robots.cpp` | `RobotSpec` 登记、命令能力／限幅与默认控制器 |
| `src/controller.cpp`、`src/controllers/` | 控制器建立入口及七个控制实现 |
| `src/controller_policy.cpp`、`src/tracking.cpp` | ONNX 策略执行、可关闭的外层速度／航向反馈 |
| `src/random.cpp` | 与 NumPy 1.26.4 单一 uint64 seed 相容的 SeedSequence＋PCG64 |
| `src/lidar.cpp`、`src/lidar_cuda.cu` | CSV、pose history、独立 `mj_ray` CPU 参照与原生 CUDA 射线 |
| `src/cameras.cpp` | EGL 透视 RGB／Z-depth、全景 RGB／径向深度 |
| `src/ros_bridge.cpp` | rclcpp、Livox／IMU／图像讯息、命名空间与超时停车 |
| `src/runtime.cpp`、`src/viewer.cpp` | 传感器调度与 GLFW GUI |
| `src/artifacts.cpp`、`src/resources.cpp` | 输出文件与程序资源量测 |
| `src/course.cpp`、`src/course_geometry.cpp` | 无 DDS 的路线／速度／接触／摩擦判定 |
| `src/full_run.cpp` | 实际运行的证据聚合、哈希及验收握手 |
| `tools/`、`tests/` | 离线模型汇出及独立 oracle；不由模拟 runtime 启动 |

CMake 分别建立 robots、random、core、LiDAR、cameras、ROS、course、evidence 库。
相机与雷达测试各自连结所需库；路线判定测试不建立 DDS、不依赖图形或 CUDA。
`Controller` 保留控制实现边界，PImpl 隔离 SDK／资源生命周期，`RunObserver` 接收真实运行证据；
这些仍有用途。新增采用现有关节／控制器 schema 的同型模型可登记 `RobotSpec` 与模型 catalog；
新增控制律仍需对应实现和 factory 登记，不能视为任意车型只改数据即可接入。

模型／策略来源与 SHA 保存在 `assets/catalog.json` 及既有 `../assets/manifests/` manifests。ONNX Runtime 1.19.2、nlohmann/json 3.11.3 headers 的版本、来源、SHA 与 MIT 许可见 [third_party/manifest.json](../third_party/manifest.json) 及相邻 LICENSE；SeedSequence／PCG 的 MIT 与 NumPy glue 的 BSD 许可保留在 `src/random.cpp`。

输出默认放在新建的 `output/mujoco/native-*`，或由 `--output NEW_DIRECTORY` 指定；已存在的目录会拒绝。包含 `summary.json`、`compiled-model.xml`、trajectory／SI IMU CSV，以及最后一帧的 LiDAR、相机数据。LiDAR 字段分别储存为 `.npy`，深度亦为独立 `.npy`，RGB 为 PNG，外参及时序见相邻 metadata JSON；格式与 Python 的 NPZ 封装不同。`--summary-only` 略过逐步 CSV 与传感器文件，保留 summary 和模型。

2026-10-13 的版本七台各通过 31/31 正式完整 ROS 路线检查；CTest 10/10、Python 392 tests＋65 subtests 通过，无 skip。
双轮参考模型的正常脚轮接地误判已修正：依既有车型能力与明确脚轮角色分类，没有按机器人名字分支。
真正车壳接触仍被拦截，原 50° 倾角与 0.1 m 高度门槛保留；87 条原生边界断言验证正常支撑与错误碰撞。
该版本的 binary SHA、源码身份和逐案结果见下方阶段纪录；此次目录整理的重新验证另见[结构整理](repository-layout-20261014.md)。
修正前优化快照另有相机 16/16 fixtures 通过，32 组深度逐比特相同；RGB 最大差 1 灰阶。四场景共 320 万条
CUDA／独立 CPU 射线结果一致；FR-mini 完整传感器 GUI 亦完成实际运行。
范围、三台轮式车交错 A/B 资源数据、两个实际故障变异及扩展实验见
[架构整理与消融纪录](cpp-architecture-ablation-20261013.md)。
[C++ runtime 迁移纪录](cpp-runtime-validation-20261013.md)保留前一阶段结果。
