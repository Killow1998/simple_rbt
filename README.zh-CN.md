# simple_rbt

[English](README.md) | [简体中文](README.zh-CN.md)

基于 MuJoCo 的独立 C++17 仿真核心，可选择薄 ROS 1 Noetic／ROS 2 Humble 适配层。输入速度指令，输出物理运动、MID360 雷达/IMU、普通与全景相机、里程计和 TF。

支持 Go2、Go2W、双轮差速参考模型、Scout Mini、FR-mini、Ridgeback 和 Ranger Mini。轮式底盘使用平地、坡道和连续起伏；Go2、Go2W 还支持楼梯及阶跃地形。

## 快速开始

以下命令从仓库根目录执行：

```bash
# 准备 SDK、模型、策略和原生构建。
./setup.sh

# SDK 和资源已准备好时，仅重新构建消息与 C++。
./setup.sh --native-only

# 完整传感器、10 Hz 相机、实时运行和桌面窗口。
./run.sh --robot fr_mini --cmd .3 0 0 --sensors full \
  --backend cuda --camera-hz 10 --realtime --viewer --duration 10

./run.sh --help
```

`run.sh` 启动 C++ 程序。Python 用于离线准备和独立对照。

按需要选择通信方式，三种构建复用同一个核心：

```bash
./setup.sh --native-only --no-ros
SIMPLE_RBT_BUILD_DIR="$PWD/build/standalone" ./run.sh --robot ranger_mini --cmd .3 0 0

./setup.sh --native-only --ros2
./run.sh --robot go2w --ros2 --sensors full --camera-hz 10

.venv/bin/python tools/test_ros1.py --base-image YOUR_NOETIC_IMAGE --build-only
./setup.sh --native-only --ros1
ROS_MASTER_URI=http://127.0.0.1:11311 ./run.sh --robot ranger_mini --ros1
```

ROS 1 示例需要已启动的 master。Jammy 从所选 Noetic 镜像导出本地 C++ SDK；详见 [ROS 1 准备与测试](docs/noetic.md)。

`--cmd VX VY WZ` 分别指定前进速度、侧移速度和转向角速度，单位为 m/s、m/s、rad/s。加上 `--ros1` 或 `--ros2` 后，直接从 `/<robot>/cmd_vel` 接收指令；超过 0.5 秒没有新指令就停车。例如 Go2W 使用 `/go2w/cmd_vel`。

`--control-mode native` 仅使用底盘自身控制器；`tracked` 额外使用模拟器的速度和航向反馈。指令速度不代表所有地形上的实际速度。

## 目录分工

```text
simple_rbt/
├── CMakeLists.txt          C++ 构建入口
├── setup.sh / run.sh       准备与启动入口
├── src/                    C++ 核心、传感器、调度与程序装配入口
├── include/simple_rbt/     C++ 模块接口
├── assets/                 模型、策略、参数与扫描数据
├── tests/                  核心、ROS 1 与 ROS 2 测试
├── tools/                  准备、量测与端到端验收
├── reference/python/       独立 Python 对照实现及测试
├── interfaces/             ROS 1／ROS 2 共用消息定义
├── wrappers/               薄 ROS 适配层与共用消息转换
│   ├── ros1/               roscpp 适配层
│   ├── ros2/               rclcpp 适配层
│   └── common/             名称、校验与消息数据转换
├── third_party/            固定版本头文件与授权
├── docs/                   使用、设计与验证记录
├── build/                  本机构建产物（不进 Git）
└── output/                 实验与测试结果（不进 Git）
```

`.venv/` 提供 MuJoCo/ONNX Runtime SDK 和对照工具；`.vendor/` 存放模型网格及 manifest 管理的依赖。两者不进 Git。

详细说明见 [运行环境](docs/runtime.md)、[机器人与传感器](docs/python-reference.md)和[资源分工](assets/README.md)。

## 验证

```bash
source tools/ros_env.sh
ctest --test-dir build/native --output-on-failure
PYTHONPATH=reference/python .venv/bin/python -m pytest reference/python/tests
.venv/bin/python tools/validate_full_run.py --runtime cpp --robot fr_mini \
  --terrain ramp --control-mode native --speed .6 --output output/full-fr-new
```

每次运行使用新的输出目录。GPU 与 live ROS 测试需要对应的宿主机环境。测试版本和条件见[验证记录](docs/README.md)。

## Noetic Docker 测试

使用已有的 Noetic 镜像测试 ROS 1 直连适配层。镜像准备与测试范围见 [Noetic 测试说明](docs/noetic.md)。

```bash
.venv/bin/python tools/test_ros1.py --skip-build
```
