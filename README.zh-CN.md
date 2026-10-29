# simple_rbt

[English](README.md) | [简体中文](README.zh-CN.md)

基于 C++17 MuJoCo 核心的机器人仿真器，**SpRobo** 是其原生 [MyGo](https://github.com/egoist/mygo) 启动器和运行操作台，框架与仓库继续叫 `simple_rbt`。选择机器人、地形和传感器，在实时三维画面中调整外参，再选择手动或程序控制。ROS 1 Noetic 与 ROS 2 Humble 是可选的独立 C++ 适配进程。

支持 Go2、Go2W、TurtleBot3 Waffle Pi、Scout Mini、FR-mini、Ridgeback、Ranger Mini、固定底座的 Franka Emika Panda 机械臂，以及 Skydio X2 四旋翼。纯轮式底盘只使用平地、坡道和连续起伏；Go2、Go2W 还可选择楼梯及阶跃地形。速度跟踪是模拟器辅助功能，不代表实车性能。

四旋翼使用四个旋翼推力与 C++ 控制器，接受世界 ENU 的 XYZ 线速度和偏航角速度，
横滚、俯仰由控制器计算。操作台提供起飞、降落、悬停和键盘升降。
详见[飞行操控](docs/flight.md)。**教程**页可启动独立 FAST-LIO 或 FAST-LIVO2 前端，
并从当前传感器安装位姿生成标定。算法镜像需单独准备；当前固定 FAST-LIVO2 版本仍需
本地源码。详见[建图教程](docs/tutorials.md)。

## 安装与启动

当前安装包面向 Ubuntu 22.04 amd64：

```sh
sudo apt install ./sprobo_0.2.1_amd64.deb
sprobo
```

也可以从应用菜单打开 **SpRobo**。启动时检查 Ubuntu 22.04 amd64、随包 MuJoCo 3.15.0 与核心兼容性、模型／权重资源及选中的 ROS 容器；不要求宿主安装 ROS。`sprobo --check-dependencies` 输出 JSON 诊断，加上 `--ros ros1,ros2` 可检查两个适配器。详见[安装包与 CI 发布说明](packaging/README.md)。

右上角可以切换**浅色／深色**与 **English／简体中文**。切换立即生效，保留当前
仿真会话，关闭后再打开也会记住选择。首次使用默认深色，语言根据系统区域设置
选择英文或简体中文。

先选择机器人、地形预设和传感器输出。数值面板中的位置／旋转，与三维平移／旋转手柄修改同一组外参。所选 MID360 或 Avia 雷达及内部 IMU 作为一个刚性设备一起移动，不能改变两者之间的相对关系。开始后进入第一／第三视角与摇杆操控；返回配置会建立新的会话。设置和运行结果保存在用户的 XDG 目录中。

## ROS 连接

C++ 无头仿真运行在宿主机，ROS 1、ROS 2 各自运行在独立进程或容器内，通过本地 Unix socket 连接核心。两者可以同时发布同一核心的传感器数据；在操作台选择一个指令来源：手动、ROS 1 或 ROS 2。速度话题为 `/<robot>/cmd_vel`，例如 `/go2w/cmd_vel`。0.5 秒没有新指令后停车。

可选镜像使用公开、固定版本的 ROS 基础镜像，不依赖个人 Jammy 镜像，也不导出宿主 ROS SDK。准备镜像与直接启动节点见 [ROS 使用说明](docs/noetic.md)。

运行页面提供 **Manual／手动**、**Program · IPC／程序 · IPC**、**Program · ROS 1** 和 **Program · ROS 2**。失焦停车仅作用于手动输入；程序控制可在窗口失焦后继续运动。0.5 秒指令超时、控制端断开或显式停车仍会清零，后续建图／导航栈可使用[程序控制接口](docs/runtime.md#program-control--程序控制)。

手动模式下点击摇杆或驾驶视图，即可使用 W/S 前后、A/D 横移、Q/E 转向，空格停车。键盘和摇杆共用速度上限；松键、离开驾驶控件或窗口失焦会停止手动输入。

传感器采用官方 MID360、Avia、D435 外观，可选 D435i 独立 IMU。内部外参保持刚性；RGB 与深度使用各自光学原点。详见[传感器与键盘说明](docs/runtime.md)。

Go2、Go2W 控制器采用 CPU ONNX 推理，运动控制和 MuJoCo 物理不需要 GPU。CUDA 可选用于加速雷达；相机也可使用 Mesa 软件渲染。完整传感器的吞吐取决于机器性能。

Panda 提供七个关节的位置控制和联动夹爪。腕部第一人称相机与世界固定的第三人称相机
均输出 RGB、深度和 CameraInfo，默认 320×240、10 Hz。两台相机都支持数值／三维外参
编辑和自定义名称的预设。详见[机械臂控制与相机话题](docs/runtime.md#fixed-arm-and-two-cameras--固定机械臂与双相机)。

使用 `--lidar-model avia` 或机器人预设中的“雷达设备”选择 Avia。
[Go2W＋Avia＋D435i 示例](examples/go2w-avia-d435i.json)可通过
`simple-rbt-console --config` 加载，并用 `--assets` 指向仓库的 `assets` 目录。
该示例的光学中心相距 9.6 cm，前向轴平行；现有安装位姿不会自动改变。
详见[传感器约定与限制](docs/runtime.md#avia-scan-and-mount--avia-扫描与安装)。

## 开发

从仓库根目录执行：

```sh
# 准备锁定的仿真 SDK／资源、Go／MyGo 依赖并构建。
./setup.sh
# 已有仿真 SDK／资源时：
./setup.sh --native-only
./run.sh --console
# 只构建核心：
./setup.sh --native-only --no-desktop
./setup.sh --native-only --no-desktop --cpu-only
./run.sh --robot scout_mini --cmd .3 0 0 --sensors lidar --backend cpu --duration 10
./run.sh --help
```

物理、控制器、传感器、调度和 ROS 适配器使用 C++。GUI 按 MyGo 的要求使用原生 Go。Python 只用于准备、打包与独立验证；已安装的仿真器和操作台不会启动 Python。

```text
simple_rbt/
├── src/, include/           C++ core, sensors and headless session
├── console/                 Native Go/MyGo configuration and operator UI
├── ipc/                     ROS-independent session protocol
├── wrappers/                Separate C++ ROS 1 / ROS 2 adapters
├── deployment/              Official ROS container build definitions
├── interfaces/              Shared Livox message definitions
├── assets/                  Models, policies, terrain and scan data
├── tests/, tools/            E2E checks and offline preparation
├── third_party/, cmake/     Locked dependencies and build/install rules
├── packaging/, docs/        Debian package and documentation
└── build/, output/          Ignored build products and results
```

`.venv/` 提供构建用 MuJoCo／ONNX SDK；`.vendor/` 存放 manifest 管理的依赖。安装后的程序不依赖这两个开发目录。详见[运行与模块边界](docs/runtime.md)、[资源说明](assets/README.md)及[验证记录](docs/README.md)。

## 验证

```sh
ctest --test-dir build/desktop --output-on-failure
python3 tools/test_ros_session.py --ros ros1 --skip-build --robots go2w
python3 tools/test_ros_session.py --ros ros2 --skip-build --robots ranger_mini
python3 tools/package_deb.py --build-dir build/desktop
# 使用独立的 Xvfb 显示，不使用用户桌面：
python3 tools/test_console_native.py --display :114 --output output/native-window-check
python3 tools/profile_console.py --display :114 --seconds 20 --ros --output output/console-resource-check
```

真实传感器与 GUI 验收在可访问图形设备的宿主机执行。干净容器验收覆盖安装、资源／运行库完整性、CPU 物理运行和卸载。带日期的记录注明具体测试版本与条件。
已完成的[操作台验收与资源测量](docs/desktop-console-plan.md)包含结果和适用范围。
