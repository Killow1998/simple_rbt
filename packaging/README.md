# Ubuntu package / Ubuntu 安装包

SpRobo is the launcher for the **simple_rbt** framework. The current target is
Ubuntu 22.04 amd64. Build the C++ simulator and native Go
console first, then prepare the package from the repository root:

SpRobo 是 **simple_rbt** 框架的启动器，当前目标为 Ubuntu 22.04 amd64。
先构建 C++ 仿真器与原生 Go 操作台，再在仓库根目录打包：

```sh
python3 tools/package_deb.py --build-dir build/desktop
sudo apt install ./output/packages/sprobo_0.2.1_amd64.deb
```

`packaging/VERSION` is the single release version for the framework, launcher
and Debian package. `sprobo --check-dependencies` reports required and optional
dependencies before starting a simulation.

`packaging/VERSION` 统一指定框架、启动器和 Debian 包的发行版本。
`sprobo --check-dependencies` 可以在仿真启动前检查必需与可选依赖。

Install Ubuntu 22.04 updates before installing: the asynchronous camera worker
requires `libstdc++6 >= 12`, supplied by Jammy updates. The package declares this
dependency so an older C++ runtime cannot silently pass installation.

安装前请更新 Ubuntu 22.04 系统软件包：异步相机线程需要 Jammy 更新源提供的
`libstdc++6 >= 12`。安装包已声明此依赖，避免在旧 C++ 运行库上安装后无法启动。

Open **SpRobo** from the Ubuntu application menu, or run
`sprobo`. `simple-rbt-sim` is the headless executable. The installed
programs use a C++ simulation worker and a native Go/mygo console; Python is used
only during preparation and independent verification.
No source checkout, preparation virtualenv, ROS SDK, CUDA compiler or private
Docker image is required to start the console. GPU sensor execution still
requires a compatible host GPU driver. Installing the package does not install
or modify GPU drivers, grant Docker access, start ROS masters or pull images.

安装后从 Ubuntu 应用菜单打开 **SpRobo 机器人操作台**；也可以执行
`sprobo`。`simple-rbt-sim` 是 C++ 无头仿真程序，操作台使用原生 Go／mygo；
Python 仅用于准备与独立验证。启动操作台不依赖源码目录、准备用虚拟环境、ROS SDK、
CUDA 编译器或私有 Docker 镜像。GPU 传感器执行仍需兼容的宿主显卡驱动。
安装包不安装或修改显卡驱动、不授予 Docker 权限、不启动 ROS master、不拉取镜像。

Package paths / 安装路径：

| Path | Contents / 内容 |
| --- | --- |
| `/usr/bin/sprobo` | Desktop console / 桌面操作台 |
| `/usr/bin/simple-rbt-sim` | Headless simulator / 无头仿真器 |
| `/usr/lib/simple-rbt/` | Required MuJoCo, ONNX and CUDA runtime libraries / 必需运行库 |
| `/usr/share/simple-rbt/` | Referenced models, policies and scan data / 模型、策略和扫描数据 |
| `/usr/share/simple-rbt/ros-adapters/` | Optional reproducible ROS container build context / 可选 ROS 容器构建资料 |
| `/usr/share/doc/sprobo/` | Existing licenses and provenance / 已有许可与来源 |

Settings and results belong to the user through XDG directories; removal of the
application preserves them. To remove the installed application:

设置与结果通过 XDG 目录归用户保存；卸载程序保留用户设置与结果：

```sh
sudo apt remove sprobo
```

If ROS is needed, prepare either optional adapter image from its installed
context. This step requires Docker; it does not export ROS libraries to the host.

需要 ROS 时，可以从安装的构建资料准备对应的可选镜像。这一步需要 Docker；
不会把 ROS 库导出到宿主机。

```sh
docker build -f /usr/share/simple-rbt/ros-adapters/deployment/ros1/Dockerfile \
  -t simple-rbt-ros1:noetic /usr/share/simple-rbt/ros-adapters
docker build -f /usr/share/simple-rbt/ros-adapters/deployment/ros2/Dockerfile \
  -t simple-rbt-ros2:humble /usr/share/simple-rbt/ros-adapters
```

The builder checks model/policy/official mesh digests, asset closure, Ubuntu 22.04 C/C++ ABI, relative library lookup
and the absence of ROS/Python dependencies in host executables. It installs only
referenced vendor files and stores duplicate CAD assets as hard links. An
unchanged build plus the same `SOURCE_DATE_EPOCH` produces the same package.
Its adjacent JSON records the package hash and size.

打包会检查模型、策略与官方网格哈希、资产闭包、Ubuntu 22.04 C/C++ ABI、相对运行库路径，以及宿主程序未链接 ROS／
Python。仅安装引用到的 vendor 文件；重复 CAD 数据用硬链接节省空间。
构建产物与 `SOURCE_DATE_EPOCH` 不变时，生成相同的包；相邻 JSON 记录哈希与大小。

`Dockerfile.validation` uses a fresh public Ubuntu image and Ubuntu runtime
packages. Prepare the verification-only capture helper, then mount it with the
built `.deb` and `verify_install.sh` into that test container:

```sh
g++ -std=c++17 -O2 packaging/capture_x11.cpp -o build/capture-x11 -lX11 -lpng
# Inside the validation container:
sh /verify_install.sh /packages/sprobo_0.2.1_amd64.deb /evidence /capture-x11
```

The helper reads actual X11 window pixels and requests normal window close;
the script checks that its simulator worker exits.
The acceptance script installs the package, verifies library resolution,
steps every robot using CPU physics and checks the native console as a non-root user,
including a real GTK/MyGo window captured with Xvfb and software OpenGL,
then uninstalls it and checks user settings
remain. This validates packaging; a container without GPU access cannot validate
host rendering or establish a driver failure. Ubuntu 24.04 requires a separate
build and acceptance run; this package does not claim that validation.

`Dockerfile.validation` 从干净的公开 Ubuntu 镜像和 Ubuntu 运行库开始验收。
上述 C++ 截图助手仅用于验证，读取真实 X11 窗口像素并请求正常关闭；脚本检查仿真进程随之退出。
挂载 `.deb`、截图助手与 `verify_install.sh` 后，验收会安装软件包、检查运行库、以普通用户
对全部机器人运行短时 CPU 物理仿真，并用 Xvfb 与软件 OpenGL 启动、截图真实 GTK／MyGo 窗口；
再卸载并检查用户设置仍在。它验证打包过程；
没有 GPU 访问的容器不能验收宿主渲染，也不能据此判断显卡驱动故障。
Ubuntu 24.04 需要单独构建和验收；当前不声称已经验证。

The GitHub workflow builds CPU-only and CUDA-capable variants on a standard
Ubuntu runner. It checks the installed package, real native input events,
analytic RGB/depth geometry and both ROS versions for every registered robot.
Only after all checks pass may its Release job publish the `.deb`, SHA256 and
build report. Main pushes use `packaging/VERSION`; tags and manual version inputs
must match that file. An existing Release is preserved. The published build
includes CUDA acceleration and automatic CPU fallback; CI requires no GPU.

GitHub 流程在标准 Ubuntu runner 上构建 CPU 与 CUDA 两个版本，检查安装后的程序、
真实窗口输入、RGB／深度解析场景，以及全部机器人通过 ROS 1／ROS 2 的控制和传感器输出。
全部检查通过后，Release 任务才发布 `.deb`、SHA256 和构建记录。main 推送使用
`packaging/VERSION`，标签或手动输入的版本必须与其一致；已有 Release 不会覆盖。
发行包保留 CUDA 加速与自动 CPU 回退，CI 不要求显卡。

Asset redistribution status remains exactly as recorded by the existing
manifests and `copyright`. This local installation package does not establish
permission to publicly redistribute assets with unresolved licenses.

资产再分发许可保持原有 manifests 与 `copyright` 中的状态；本机安装包不新增
尚未明确许可资产的公开再分发授权。
