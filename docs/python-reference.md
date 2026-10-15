# simple_rbt 的 MuJoCo 原型

本文保留机器人／传感器规格、独立 Python 对照和此前阶段的验证记录。当前操作台、C++ 核心与 ROS 独立进程的启动方式以 [运行说明](runtime.md)和 [ROS 指南](noetic.md)为准。MyGo 界面使用 Go；核心与 ROS 节点使用 C++；Python 仅用于准备和独立验证。

[C++ 迁移与资源验证](cpp-runtime-validation-20261013.md)保存迁移阶段的完整验收和 CPU／RAM 对照；
本轮[架构整理与消融验证](cpp-architecture-ablation-20261013.md)记录模块拆分、等价性、故障拦截及交错 A/B 实测。
三台轮式车的程序峰值 RSS 均值减少 8.7–13.2 MiB，CPU 差异不足以宣称明确加速。
上述资源数据对应脚轮修正前的优化快照。双轮参考模型的正常脚轮接地误判现已修正，
仍保留车壳撞地与原安全门槛。最终七台各通过 31/31 正式检查，CTest 10/10、
2026-10-13 版本的 Python 392 tests＋65 subtests 全数通过，无 skip；完整版本身份和结果见上方阶段纪录。
[原生构建说明](runtime.md)列出依赖与目前模块。

Go2／Go2W 已加入平地、坡道、楼梯和碎石式起伏地形；Tracer、Limo 等尚未移植。
[轮式底盘部署记录](wheel-chassis-validation-20261013.md)保存 FR-mini、Ridgeback、Ranger Mini
及 Scout Mini 的连续地形、运动、ROS 和资源验证。
纯轮式底盘使用 `flat`、`ramp`、`hills`；不接受有阶跃的 `stairs` 或方块式 `rough`。
各车依公开规格建立程序几何，轮胎／马达／质量分布是工程近似，厂商独立悬架未校准。
[Scout Mini 第一阶段](scout-mini-validation-20261013.md)保留之前的历史结果。

| `--robot` | 运动结构 | `auto` 控制器 | 支持侧移／原地转向 |
|---|---|---|---|
| `go2` | 四足 | `robot_lab` | 是／是 |
| `go2w` | 轮足 | `rl_sar` | 是／是 |
| `diffdrive` | 双轮差速参考 | `none` | 否／是 |
| `scout_mini` | Scout Mini 四轮差速 | `skid_steer` | 否／是 |
| `fr_mini` | 前阿克曼、两后轮驱动 | `ackermann` | 否／否 |
| `ridgeback` | 四轮毂＋40 个被动麦轮滚子 | `mecanum` | 是／是 |
| `ranger_mini` | 四个舵关节＋四个驱动关节 | `swerve` | 是／是 |
[第一阶段记录](validation-20261013.md)保留最初接口验证；
[多地形与资源实测](terrain-resources-20261013.md)记录本阶段结果和限制。
[较高地形测试](hard-terrain-20261012.md)进一步测了 12–22 cm 台阶及 15–25° 长坡，
54 案中 34 案走完全程，两台能力差异与失败原因均保留。
高难度对照的指令前进速度为 **0.3、0.6、1.0 m/s**，实际速度另行测量。
目前适配器下，同场景 Go2 HIMLoco 补测完成 6/27 路线，RobotLab 为 10/27；
[策略对照纪录](hard-terrain-20261012.md#go2-himloco-同場景補測)保留逐案结果及失败演示。
[部署与完整联合验收](deployment-validation-20261013.md)进一步确认起身／扭矩／材料因素，
Go2W 9 组完整 ROS＋传感器案例有 8 组全部合格，低抓地坡道失败亦保留。
上述历史案例使用旧版快照雷达；目前 MID360 改用逐点运动扫描和独立内置 IMU，
本阶段的数据规格、重新验证与资源结果见 [MID360 对齐记录](mid360-validation-20261013.md)。

在原始 `standard` 低难度地形，默认控制器加反馈的两台机器人，合计 30/30 组完整 6 m 路线通过，
并各通过原地转向、侧移、倒退、边走边转和停车的平地检查。
Go2 在 0.6／1.0 m/s 仍有步态偏航摆动；1.0 m/s 指令的平均实速约 0.87–0.91 m/s。
这些测试使用理想状态反馈，不能视为实机验收。
本机 [六格地形演示](../output/stage2-20261013/terrain-preview/terrain-preview.gif)
显示两台机器人的真实物理运行，1.5 倍速回放；输出数据不纳入 Git。

## 在 Jammy 直接运行

从仓库根目录执行；Jammy 已完成原生构建。新环境需要 `git`、`uv`、Python 3.10 作离线准备，
以及原生构建说明中的 C++／CUDA／ROS／图形开发套件，再执行：

```bash
./setup.sh
```

只重编译 C++ 时执行 `./setup.sh --native-only`。Python 对照仍可用
`./reference/python/run.sh`，参数与原入口相同。

启动 Go2W 上下楼梯，同时开启完整传感器与 10 Hz 双相机：

```bash
./run.sh --robot go2w --terrain stairs --backend warp \
  --cmd .6 0 0 --settle 2 --duration 15 --sensors full --camera-hz 10 --realtime
```

在桌面查看机器人（会开启 MuJoCo 视窗）：

```bash
./run.sh --robot go2 --terrain ramp --backend warp \
  --cmd .6 0 0 --settle 2 --duration 15 --sensors full --camera-hz 10 --viewer
```

切换成双轮底盘；不使用 GPU 雷达时选 `cpu`：

```bash
./run.sh --robot diffdrive --backend cpu --cmd .3 0 .2 --duration 10
```

启动 Scout Mini 完整传感器、原生轮速控制和实时时钟：

```bash
./run.sh --robot scout_mini --terrain hills --backend warp \
  --control-mode native --cmd .3 0 0 --settle 2 --duration 25 \
  --sensors full --camera-hz 10 --realtime
```

三台新底盘使用相同入口；例如 FR-mini 走连续波浪、Ridgeback 做平地复合运动、Ranger Mini 走坡道：

```bash
./run.sh --robot fr_mini --terrain hills --terrain-level moderate \
  --cmd .6 0 0 --settle 2 --duration 14 --backend warp --sensors full --camera-hz 10 --realtime
./run.sh --robot ridgeback --terrain flat --cmd .3 .2 .3 \
  --settle 2 --duration 10 --backend warp --sensors full --camera-hz 10 --realtime
./run.sh --robot ranger_mini --terrain ramp --cmd .6 0 0 \
  --settle 2 --duration 14 --backend warp --sensors full --camera-hz 10 --realtime
```

`--cmd` 的三个值依序是前进速度、侧移速度、转向角速度，单位 m/s、m/s、rad/s。
Go2／Go2W 的指令范围为 vx ±1、vy ±0.6、wz ±1。
双轮底盘、Scout Mini 和 FR-mini 不支持侧移；FR-mini 也不接受原地转向。
Scout Mini 的范围为 vx ±1.5、vy=0、wz ±1；双轮底盘和 FR-mini 为 vx ±1、vy=0、wz ±1。
Ridgeback／Ranger Mini 为 vx ±1、vy ±0.6、wz ±1。FR-mini 的转向还受最小半径 1.1 m 的模型解读限制，
实际可发给轮组的 `|wz| <= |vx|/1.1`，限幅会记录。FR 的 `base_link` 与速度作用点位于后轴中心上方，
车壳中心在其前方 0.2 m；其他底盘的参考点见 manifest。
CLI 非法输入报错；C++／Python `Simulation` 非法速度清零后报错；
ROS 非法输入清零并继续仿真，后续有效指令可恢复。
这些是输入限幅，不能视为任意地形、速度组合都能准确追踪。
Go2 与 Go2W 默认使用各自的 rl_sar RobotLab 策略；指令不等于实际速度。
CLI 默认 `--control-mode tracked`，用仿真的理想机体速度与朝向反馈修正发给策略的指令，
减少原生策略的侧偏；没有直接移动机体或额外施力。这是仿真辅助，不能作为实机控制验收。
`--control-mode native` 可关闭外层理想机体速度／朝向反馈；Python `Simulation` 默认不启用它。
C++ 的 `SimulationOptions` 默认与 CLI 一样是 `tracked`；直接使用 C++ API 时也应明确选择控制模式。
Scout Mini 的原生 `skid_steer` 仍使用理想陀螺仪 z 角速度 PI 修正轮速，补偿转向侧滑，
只控制轮子、保留接触摩擦与扭矩限幅；这不是松灵厂商固件。`tracked` 在其外层再加入理想机体反馈。
Go2 可用 `--controller legacy` 或 `himloco` 对照其他策略。
Go2 的 `--go2-torque-mode configured|upstream` 和 `--go2-startup crouched|upstream_getup`
提供部署对照，默认仍为 `configured + crouched`。
GetUp 约 3.005 秒，CLI 的 `--duration` 包含起身及 `--settle`；指令窗口从完成起身并稳定后开始。
GetUp 曾让 12 cm 楼梯走完，但速度仍不合格，因此保留为实验选项，见上方部署纪录。

`--terrain flat|ramp|hills|stairs|rough` 选择地形，`--seed` 固定方块式起伏排列。
`hills` 是 4 m 路段上的两个连续波浪，48 个斜面接缝高度一致；它是分段平面近似，没有垂直台阶，
斜率在接缝会改变。高度查询、真实碰撞与 CPU／CUDA 雷达使用同一份几何。
`--terrain-level` 选择一组明确的地形尺寸；每条楼梯与坡道都包含上行、平台和下行。

| level | 楼梯每阶 × 级数／总高 | 坡度 × 单侧水平长度／顶高 | `rough`／`hills` 峰高 |
|---|---|---|---|
| `standard` | 6 cm × 4／0.24 m | 8° × 2 m／0.28 m | 2.5 cm |
| `moderate` | 12 cm × 6／0.72 m | 15° × 3 m／0.80 m | 5 cm |
| `hard` | 18 cm × 6／1.08 m | 20° × 4 m／1.46 m | 8 cm |
| `extreme` | 22 cm × 8／1.76 m | 25° × 4 m／1.87 m | 12 cm |

新增楼梯踏面深度 30 cm；原始踏面 40 cm。高难度长坡的完成线延伸至 x=10.6 m，
评估器会相应延长时间上限，要求完整下坡后越过终点。
`--difficulty` 仍可缩放 `standard` 坡度与高度（0–3，不含 0）；其他 level 固定为 1，避免双重缩放。
`--friction MU` 可测明确的地面材料，MU 为 (0,2]：floor／course 使用 priority2、condim6，
系数为 `(MU,0.02,0.01)`；省略时保留模型原材料混合。实际轮／脚摩擦可能覆盖地形名义值，
材料对照需检查 contact.friction。已测 25° 坡道低抓地时会超时，不能只看坡度判定能否通过。
这些是测试场景尺寸，不代表控制器已通过每一档。
本机演示：[18 cm 楼梯／20° 坡道](../output/hard-terrain-20261012/preview-hard-v06-final/terrain-preview.gif)、
[22 cm 楼梯／25° 坡道](../output/hard-terrain-20261012/preview-extreme-v03-final/terrain-preview.gif)，成功与失败均保留。

例如测试更高的楼梯：

```bash
./run.sh --robot go2w --terrain stairs --terrain-level hard \
  --cmd .6 0 0 --settle 2 --duration 20 --backend warp --viewer
```

`--sensors physics|lidar|full` 控制负载；`full` 启用雷达、IMU、普通和全景相机。
相机默认 1 Hz，完整运行建议明确指定 `--camera-hz 10`。
`--summary-only` 仍计算所选传感器，只保存摘要以限制磁碟输出。

每次运行会在用户 XDG state 目录下建立新结果目录，或使用显式 `--output`，包含：

- `summary.json`：位移、控制模式、地形、传感器耗时、CPU／RAM 和模型限制。
- `trajectory.csv`：10 Hz 地面真值轨迹和机体座标速度。
- `mid360-imu-si.csv`：200 Hz 内置 IMU，比力用 m/s²、角速度用 rad/s；没有姿态估计。
- `lidar-xyz.npy`、`lidar-offsets-ns.npy`、`lidar-lines.npy`、`lidar-ray-indices.npy`：
  最后一帧原始点云、每点时间／线号与 CSV 索引；`lidar-metadata.json` 保存开始时间和帧末参考位姿。
  `lidar-capture-position-world.npy` 和 `lidar-capture-quaternion-world-xyzw.npy` 保存每点采集位姿。
  Python 对照版本仍保存 `last-scan.npz`。
- `--cameras` 另存普通／全景 RGB PNG、深度 NPY 和 `camera-metadata.json`，包含时间、位姿与标定。

不同传感器的保存时间可能不同，应读取各自的 metadata，不能只按档名视为同步帧。
普通相机深度是沿光轴的 Z 距离，全景深度是从相机中心到表面的径向距离；均以公尺表示，无表面为 `inf`。

## ROS 接口

ROS 1／ROS 2 各自使用独立 C++ 适配进程；启动方式见 [ROS 指南](noetic.md)。在操作台选择相应指令来源后，可在对端 ROS 环境发送 `/<robot>/cmd_vel`。

```sh
ros2 topic pub -r 10 /go2w/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.3}, angular: {z: 0.1}}'
```

非法或不支持的指令整体清零，后续有效指令可恢复；停止发送 0.5 秒后超时停车。消息字段约定如下，适用于两种 wrapper：

| 话题 | 内容／频率 |
|---|---|
| `/<robot>/cmd_vel` | 上表所选机器人的平面速度输入，`geometry_msgs/msg/Twist` |
| `/clock` | 每个物理步：Go2 RobotLab／HIM／四台实名轮式底盘 200 Hz、Go2W 500 Hz、legacy／双轮 250 Hz |
| `/livox/lidar` | `livox_ros_driver2/CustomMsg`，10 Hz，每帧最多 20,000 点 |
| `/livox/points` | 同一点云的 `PointCloud2`，10 Hz |
| `/livox/imu` | 200 Hz，本机 Livox ROS 2 驱动相容：加速度 **g**、角速度 rad/s；header 为 `livox_frame` |
| `/livox/imu_si` | 同一 IMU，200 Hz，标准 SI：m/s²、rad/s；header 为真实内置 IMU 的 `imu_frame` |
| `/odom` | 地面真值里程计，10 Hz；不是估计器输出 |
| `/tf`、`/tf_static` | `odom → base_link → livox_frame / imu_frame / camera_optical_frame` |
| `/camera/image_raw`、`/camera/depth/image_raw` | 普通 RGB／Z 深度，320×240，`--camera-hz` 指定频率 |
| `/camera/camera_info` | 普通相机针孔标定 |
| `/camera/panorama/image_raw`、`/camera/panorama/depth/image_raw` | 360°×180° RGB／径向深度，720×360，同一相机频率 |

全景图像不是针孔模型，因此不发布假的 `CameraInfo`。
控制话题按车名隔离；传感器话题与 TF frame 仍是共用绝对名称，目前验证的是单机器人运行，
不是同一 ROS domain 下的完整多机器人传感器隔离。
传感器外参来自实际 MuJoCo site。Go2 的 MID360 放在顶部，并非上游示例的倒置前雷达位置。
厂商驱动的 raw IMU header 名称沿用雷达 frame，但加速度的作用点仍在内置 IMU 原点。
`/livox/imu` 使用本机 ROS 2 驱动未填姿态时的固定 identity 占位，covariance 为零，**不是姿态量测**。
`/livox/imu_si` 明确标示没有姿态（orientation covariance 首项 -1）；ROS 真值姿态由 `/odom` 和动态 TF 提供。
原本接收 `/livox/imu` 并假设 m/s² 的程式，应改订阅 `/livox/imu_si`。

## 雷达模型和目前边界

沿用仓库的 `mid360-real-centr.csv`（800,000 条方向），200 kHz、4 线、10 Hz。
CSV 第一栏的 `Time/s` 标题不准确，实际内容是连续点序号；时间由 200 kHz 采样率生成。
`CustomMsg.timebase` 保存完整模拟奈秒时间，漏点过滤会保留原始点的时间偏移和线号。

默认 `--lidar-motion rolling`：每个点按自身采集时刻的雷达位姿射线求交，
原始 XYZ 保持在该时刻的雷达局部座标，未预先去畸变。位姿采原生物理步节点，平移内插、旋转短弧 SLERP；不外推。
`--lidar-motion snapshot` 是帧末除错快照，全部点的时间偏移为零，不冒充运动扫描。
逐点扫描共用一份静态环境，排除整台机器人的几何；尚未模拟自身遮挡或动态外部障碍。
CSV 方向、200 kHz 均匀时间和四线索引是模拟采样约定，不是原始硬体时间记录。

MID360 内置 IMU 在雷达座标中位于 **[+0.011,+0.02329,-0.04412] m**，座标轴平行，
按 [官方手册](https://terra-1-g.djicdn.com/851d20f7b9f64838a34cd02351370894/Livox/Livox_Mid-360_User_Manual_EN.pdf) 的标称几何实现。
独立 MuJoCo 感测点自然包含旋转杆臂加速度，控制器仍读原机身 IMU。
IMU 使用共用模拟时钟、零时刻起的 5 ms 栅格；Go2W 的 2 ms 物理步用相邻读值内插，最多延迟 1 ms 才发出。
这是时间内插近似。默认理想、无噪声／偏置／晶片滤波，亦不截断硬体量程；Python sampler 可选 ±4g／±2000°/s 截断。
尚未复现完整光学、UDP／网路延迟、逐台实测标定；反射强度固定。
本阶段没有执行 FAST-LIO 建图，也没有与 Gazebo 做同场景控制品质或速度对照。

## 重新验证

原生 Release 传输、随机序列和验收边界测试（先 source Humble 与 Livox）。目前 CTest 共 10 项：
七台各自的指令进程、一个传感器进程、随机序列、路线门槛。ROS 测试共用 domain 89 资源锁。
开发时可先用 `-L course` 或 `-L ros` 选择受影响模块，再按接口变更重跑真实联合验收：

```bash
ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1 ctest --test-dir build/native --output-on-failure
```

此前同进程版本的完整路线验收命令（对应带日期的版本；当前 C++ ROS 验收使用 `tools/test_ros_session.py`）：

```bash
.venv/bin/python tools/validate_full_run.py --runtime cpp \
  --robot ridgeback --terrain hills --terrain-level moderate --control-mode native \
  --speed .6 --output output/cpp-full-new
```

以下 Python 测试及地形矩阵是参考模型验证；不代表执行了 C++ 版本。
核心、CSV、相机和传输的 Python 对照检查：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 MUJOCO_GL=egl SIMPLE_RBT_TEST_RENDER=1 SIMPLE_RBT_TEST_WARP=1 SIMPLE_RBT_TEST_CUDA=1 SIMPLE_RBT_ROS_TESTS=1 \
  .venv/bin/python -m pytest -q reference/python/tests
```

CPU／CUDA 在同一批移动场景射线上的对照：

```bash
.venv/bin/python tools/benchmark_lidar.py \
  --frames 100 --warmup 5 --rays 20000 --gpu-device cuda:0 \
  --output output/lidar-benchmark.json
```

详细结果包含所有误差，不以平均速度掩盖命中差异。`--require-agreement` 要求命中索引完全一致，
旧快照测试曾有一个墙角边界判定差异；新 rolling 模式的独立结果见上方 MID360 对齐记录，两者不能混用。

完整 ROS 2 实际发布／接收验证命令见脚本 `tools/check_ros2.py --help` 及本轮结果记录。
轮式的所有支持动作、坡道和连续波浪可用统一入口重跑；需要新的输出目录：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python \
  tools/evaluate_wheels.py --control-modes native \
  --output output/wheel-matrix-new
```

默认四台实名轮式底盘，0.3／0.6／1 m/s、8°／15° 坡道、2.5／5 cm 波浪；
FR-mini 不生成侧移与原地转向案例。逐项失败保留于结果，矩阵完成不代表每案通过。
当前 Ridgeback 15° 坡入口会触底，FR-mini 原生 15° 下坡会超速；
全程平均通过也不能取代坡段速度检查，详见上方部署记录。

分别量测物理、CPU／CUDA 雷达及完整双相机的资源和速度（使用新的输出目录）：

```bash
.venv/bin/python tools/profile_resources.py \
  --runtime cpp --duration 6 --output output/resource-profile-new
```

每种设定使用独立程序，RAM 不会混入前一种设定的峰值。CPU 100% 表示一个逻辑核心；
paced 结果反映实时负载，unpaced 结果反映不等待时钟的运行速度。
脚本默认 ONNX／BLAS 单执行绪；不需要安装 Torch。专案磁碟统计不含共用缓存或系统 ROS。
在上述资源命令加入 `--robots scout_mini --terrain hills --control-mode native --speed 1.5` 可单独量测 Scout；
默认仍为 `tracked`。速度上限按所选机器人检查，混合 Go2／Go2W 的批次仍限 1 m/s。

重跑两台机器人完整路线与独立速度检查：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python \
  tools/evaluate_terrains.py --go2-controllers robot_lab \
  --control-modes native tracked --output output/terrain-matrix-new
```

## 程式入口与来源

目前执行来源：

| 文件 | 责任 |
|---|---|
| `../src/core.cpp`、`scene.cpp` | 物理步进与模型／地形装配 |
| `../src/robots.cpp` | `RobotSpec`：车型能力、限幅与默认控制器 |
| `../src/controllers/`、`controller_policy.cpp`、`tracking.cpp` | 七种控制实现、ONNX 策略与外层反馈 |
| `../src/runtime.cpp` | 共用时钟、200 Hz IMU、10 Hz rolling 雷达、相机调度 |
| `../src/artifacts.cpp`、`resources.cpp` | 输出文件与程序资源量测 |
| `../src/lidar.cpp`、`lidar_cuda.cu` | MID360 CSV、位姿内插、CPU／CUDA 射线 |
| `../src/cameras.cpp`、`viewer.cpp` | 普通／全景感测相机与桌面窗口 |
| `../wrappers/ros2/ros_bridge.cpp` | 真实 DDS 收发、命名空间与超时停车 |
| `../src/course.cpp`、`course_geometry.cpp` | 路线／速度、安全包络、接触与实际摩擦门槛 |
| `../src/full_run.cpp` | 同次物理／传感器运行的证据、哈希与 DDS 握手 |
| `scripts/validate_full_run.py`、`scripts/full_run_*.py` | 独立 Python 验收入口、参考 worker、DDS 接收、31 项判定与证据文件 |

以下保留为 Python 模型准备与对照来源：

| 文件 | 责任 |
|---|---|
| `simple_rbt_mujoco/core.py` | 机器人、控制器、物理步进、理想 IMU |
| `simple_rbt_mujoco/go2.py`、`go2w.py` | 各机器人的原生观测、动作与 PD 控制合约 |
| `simple_rbt_mujoco/scout_mini.py` | Scout Mini 参数模型、轮速与陀螺仪转向控制 |
| `simple_rbt_mujoco/fr_mini.py`、`ridgeback.py`、`ranger_mini.py` | 阿克曼、麦轮滚子、舵轮的接触驱动模型与轮组控制 |
| `simple_rbt_mujoco/robots.py` | 车型选择、输入限幅、运动与地形契约 |
| `simple_rbt_mujoco/wheel_geometry.py` | 真实车体／轮组包络、世界走廊投影与非轮组接触 |
| `simple_rbt_mujoco/command_tracking.py` | 可关闭的理想速度与朝向反馈 |
| `simple_rbt_mujoco/terrains.py` | 可重现地形与对应高度查询 |
| `simple_rbt_mujoco/lidar.py` | CSV 扫描方向与帧末快照 |
| `simple_rbt_mujoco/rolling_lidar.py` | 原生位姿历史、逐点移动射线、CPU／Warp 共用静态环境 |
| `simple_rbt_mujoco/mid360.py` | 内置 IMU 几何、200 Hz 采样与明确单位 |
| `simple_rbt_mujoco/cameras.py` | 普通相机、六面拼接全景 |
| `simple_rbt_mujoco/ros2.py` | ROS 2 讯息转换与指令超时 |
| `simple_rbt_mujoco/cli.py` | 可执行回圈与数据保存 |

雷达与 legacy Go2 模型／ONNX 控制器固定于
[`e6b879df`](https://github.com/discoverse-dev/MuJoCo-LiDAR/tree/e6b879df4b96daaed30ef10fe02650d29048fa9b)。
legacy Go2 接口沿用其 `examples/unitree_go2.py`；模型只在记忆体中加装传感器和测试场景。
来源保留在 `.vendor/`，MIT 授权副本见 [LICENSE-MuJoCo-LiDAR.txt](LICENSE-MuJoCo-LiDAR.txt)。
`requirements.lock` 固定本轮 Python 3.10 的依赖，`setup.sh` 固定上游提交。
目前默认 Go2／Go2W 的来源、逐档哈希与 ONNX 汇出验证在 `assets/go2_manifest.json`、
`assets/go2w_manifest.json` 及 [Go2W 来源说明](../assets/manifests/GO2W_ASSETS.md)。只下载所需模型档；
模型库未找到根目录授权，mesh 保留在被 Git 忽略的本机 vendor，未重新发布。
Scout Mini 使用原创程序几何，不下载 vendor mesh 或控制权重；尺寸来源、工程假设、
被舍弃的旧模型参数和版本差异记于 `assets/scout_mini_manifest.json`。
FR-mini、Ridgeback、Ranger Mini 同样使用原创程序几何，来源与工程假设记于各自的 `*_manifest.json`。
