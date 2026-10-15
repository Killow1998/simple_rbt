# Independent ROS adapters / 独立 ROS 适配器

The host C++ simulator and native console are independent of ROS. ROS 1 Noetic and ROS 2 Humble each run in their own C++ node process/container, using the same local session protocol. They connect directly to ROS peers. The production Dockerfiles pin public official ROS bases and compile the shared Livox definitions; no personal image or host SDK export is needed.

宿主 C++ 仿真和原生操作台独立于 ROS。ROS 1 Noetic、ROS 2 Humble 各自运行在独立 C++ 节点进程／容器，通过同一会话协议连接核心，直接与各自 ROS 节点通信。正式 Dockerfile 固定公开的官方 ROS 基础镜像，并编译共用 Livox 消息；不需要个人镜像或宿主 SDK 导出。

SpRobo validates the actual ROS installation inside each selected container: Noetic uses Ubuntu 20.04; Humble uses Ubuntu 22.04. The supported simulator host is Ubuntu 22.04 amd64. Host `ROS_DISTRO` is not used as evidence that a container is compatible.

SpRobo 检查选中容器内实际的 ROS 安装：Noetic 使用 Ubuntu 20.04，Humble 使用 Ubuntu 22.04；仿真宿主支持 Ubuntu 22.04 amd64，不把宿主的 `ROS_DISTRO` 当成容器版本证据。

## Livox publication / Livox 发布

With MID360 and a ROS adapter enabled, the core sends its real scan through IPC to the independent C++ node. That node publishes `/livox/lidar` as `livox_ros_driver2/CustomMsg` (ROS 1) or `livox_ros_driver2/msg/CustomMsg` (ROS 2), alongside PointCloud2 on `/livox/points`. The conversion preserves scan timebase, point count, per-point offset time, XYZ and line. Tag and reflectivity are simulator values. IMU, odometry, TF and clock use the same core's samples.

启用 MID360 与 ROS 适配器后，核心通过 IPC 将实际扫描交给独立 C++ 节点，节点在 `/livox/lidar` 发布 ROS 1 的 `livox_ros_driver2/CustomMsg` 或 ROS 2 的 `livox_ros_driver2/msg/CustomMsg`，并在 `/livox/points` 提供 PointCloud2。转换保留扫描 timebase、点数、逐点 offset_time、XYZ 与 line；tag 和 reflectivity 为模拟值。IMU、里程计、TF 与时钟也来自同一核心。

## Prepare / 准备

From a source checkout / 在源码目录：

```sh
python3 tools/test_ros_session.py --ros ros1 --build-only
python3 tools/test_ros_session.py --ros ros2 --build-only
```

From an installed package / 安装后：

```sh
docker build -f /usr/share/simple-rbt/ros-adapters/deployment/ros1/Dockerfile \
  -t simple-rbt-ros1:noetic /usr/share/simple-rbt/ros-adapters
docker build -f /usr/share/simple-rbt/ros-adapters/deployment/ros2/Dockerfile \
  -t simple-rbt-ros2:humble /usr/share/simple-rbt/ros-adapters
```

The console can prepare and start optional adapters when Docker is available. Physics and GPU rendering remain on the host. Adapters need no GPU and can both publish a session's sensors. The selected command source alone controls motion.

Docker 可用时，操作台可以准备并启动可选适配器。物理与 GPU 渲染仍在宿主机。适配器不需要 GPU，两者可同时发布一个会话的传感器；只有选中的指令来源控制运动。

## Direct launch / 直接启动

For a core already running with `--session-dir /absolute/session/path`, mount that directory at `/session`. The socket directory belongs to the current user and is private; adapters need matching permissions. The console owns its sessions and shows their path.

核心已通过 `--session-dir /absolute/session/path` 运行时，将该目录挂载为 `/session`。socket 目录归当前用户所有且为私有目录，适配器需要相应权限。操作台拥有自己的会话，并显示路径。

```sh
docker run --rm --network host \
  --mount type=bind,source=/absolute/session/path,target=/session \
  -e ROS_MASTER_URI=http://127.0.0.1:11311 -e ROS_IP=127.0.0.1 \
  simple-rbt-ros1:noetic --session-dir /session

docker run --rm --network host \
  --mount type=bind,source=/absolute/session/path,target=/session \
  -e ROS_DOMAIN_ID=83 -e ROS_LOCALHOST_ONLY=1 \
  simple-rbt-ros2:humble --session-dir /session
```

ROS 1 reuses a reachable master; if the default local master is absent, the container starts its own. A configured unreachable remote master produces an error. ROS 2 peers must use the same domain. Remote peers additionally need normal ROS networking configuration.

ROS 1 复用可达的 master；默认本地 master 未启动时，容器自行启动。配置的远程 master 不可达时明确报错。ROS 2 对端需要使用相同 domain；远程节点还需要正常的 ROS 网络配置。

`/<robot>/cmd_vel` accepts planar m/s, m/s, rad/s commands. Invalid or unsupported commands clear all components. A 0.5 s timeout, adapter disconnect or owner change stops the prior command. Physics, IMU/LiDAR timing, camera rendering and simulation time remain core responsibilities. The MID360 internal transform is fixed and is published consistently by both wrappers.

`/<robot>/cmd_vel` 接收平面速度，单位为 m/s、m/s、rad/s。非法或不支持的指令会整体清零。0.5 秒超时、适配器断开或切换来源会停止先前指令。物理、IMU／雷达时序、相机渲染与仿真时钟由核心负责；MID360 内部外参固定，两种 wrapper 一致发布。

## E2E checks / 端到端检查

```sh
python3 tools/test_ros_session.py --ros ros1 --skip-build --robots go2w
python3 tools/test_ros_session.py --ros ros2 --skip-build --robots ranger_mini --terrain hills
# Compatibility entry for the same independent ROS 1 runner:
python3 tools/test_ros1.py --skip-build --robots go2w
```

Omit `--robots` to test all registered robots. The runner launches the real host core and an official adapter container, then sends commands and inspects real ROS outputs from a separate receiver. It checks namespace isolation, physical forward/reverse motion, stopping, sensor outputs and rigid device transforms. It uses its own session and container, then cleans those resources. Test clients use Python; production nodes remain C++.

不指定 `--robots` 时检查全部登记车型。测试启动真实宿主核心与官方适配器容器，由独立接收器发送指令并检查实际 ROS 输出，覆盖命名空间、物理前进／倒退、停车、传感器数据和设备刚性外参。测试创建并清理自己的会话及容器。测试客户端使用 Python；正式节点仍为 C++。

Current results and binary identities are linked in [console validation](desktop-console-plan.md). Earlier same-process ROS acceptance belongs to its dated development stage and does not describe this launch path.
当前结果与二进制身份见[操作台验收](desktop-console-plan.md)。此前同进程 ROS 验收属于当时的开发阶段，不描述当前启动路径。
