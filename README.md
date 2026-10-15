# simple_rbt

[English](README.md) | [简体中文](README.zh-CN.md)

A robot simulator with a C++17 MuJoCo core and **SpRobo**, its native [MyGo](https://github.com/egoist/mygo) launcher and operator console. The framework and repository remain `simple_rbt`. Configure the robot, terrain and sensors, adjust sensor mounts in a live 3D view, then drive manually or select a program command source. ROS 1 Noetic and ROS 2 Humble are optional independent C++ adapters.

Supported robots: Go2, Go2W, TurtleBot3 Waffle Pi, Scout Mini, FR-mini, Ridgeback and Ranger Mini. Pure wheel chassis use flat ground, ramps and continuous hills. Go2 and Go2W also offer stairs and stepped terrain. Command tracking is a simulator aid, rather than a hardware performance claim.

## Install and launch

The current package target is Ubuntu 22.04 amd64:

```sh
sudo apt install ./sprobo_0.2.1_amd64.deb
sprobo
```

You can also open **SpRobo** from the application menu. Startup checks Ubuntu 22.04 amd64, the bundled MuJoCo 3.15.0/core compatibility, model/policy assets and any selected ROS container. No host ROS installation is required. Run `sprobo --check-dependencies` for JSON diagnostics, or add `--ros ros1,ros2` to inspect both adapters. See [package details and CI release](packaging/README.md).

Use the top-right selectors to choose **Light / Dark** and **English / 简体中文**.
Changes apply immediately without restarting the simulation and are remembered
when you reopen SpRobo. The initial appearance is dark; the initial language
follows the system locale (English or Simplified Chinese).

Select a robot, terrain preset and sensor outputs in the configuration screen. Numeric position/rotation fields and the 3D translation/rotation handles edit the same mount. MID360 LiDAR and its internal IMU move as one rigid device; their relative transform cannot be edited. Press Start to enter first/third-person viewing and joystick control. Return to configuration creates a fresh session. Settings and results use the user's XDG directories.

## ROS connection

The headless C++ worker stays on the host. Each ROS adapter runs in its own process or container, connected through local Unix sockets. Either adapter, or both, can publish the same core's sensors. Select one command owner in the console: Manual, ROS 1 or ROS 2. Velocity commands use `/<robot>/cmd_vel`, for example `/go2w/cmd_vel`. A stale command times out after 0.5 seconds.

The optional images build from public, pinned ROS bases. No personal Jammy image or host ROS SDK export is required. Image preparation and direct node launch are described in the [ROS guide](docs/noetic.md).

Select **Manual**, **Program · IPC**, **Program · ROS 1** or **Program · ROS 2** in the operator screen. Window focus loss stops only manual input. Program commands continue while the window is unfocused; a 0.5-second command timeout, owner disconnect or explicit Stop still clears motion. The [program-control interface](docs/runtime.md#program-control--程序控制) is intended for mapping/navigation integration.

Go2 and Go2W use CPU ONNX inference. Their motion controllers and MuJoCo physics do not require a GPU. CUDA optionally accelerates LiDAR; camera rendering can use software Mesa. Full sensor throughput depends on the machine.

## Development

Run from the repository root:

```sh
# Prepare locked simulator SDKs/assets, Go/MyGo dependencies and build.
./setup.sh
# With simulator SDKs/assets already prepared:
./setup.sh --native-only
./run.sh --console
# Core only, without a desktop build:
./setup.sh --native-only --no-desktop
./setup.sh --native-only --no-desktop --cpu-only
./run.sh --robot scout_mini --cmd .3 0 0 --sensors lidar --backend cpu --duration 10
./run.sh --help
```

Physics, controllers, sensors, scheduling and ROS adapters run in C++. The native GUI runs in Go as required by MyGo. Python is used for preparation, packaging and independent verification. It is not started by the installed simulator or console.

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
├── reference/python/        Independent reference and verification
├── third_party/, cmake/     Locked dependencies and build/install rules
├── packaging/, docs/        Debian package and documentation
└── build/, output/          Ignored build products and results
```

`.venv/` provides build-time MuJoCo/ONNX SDKs; `.vendor/` contains manifest-managed dependencies. Neither is a runtime requirement of the installed package. See [runtime and module boundaries](docs/runtime.md), [assets](assets/README.md) and [validation records](docs/README.md).

## Validation

```sh
ctest --test-dir build/desktop --output-on-failure
python3 tools/test_ros_session.py --ros ros1 --skip-build --robots go2w
python3 tools/test_ros_session.py --ros ros2 --skip-build --robots ranger_mini
python3 tools/package_deb.py --build-dir build/desktop
# An isolated Xvfb display, not the user's desktop:
python3 tools/test_console_native.py --display :114 --output output/native-window-check
python3 tools/profile_console.py --display :114 --seconds 20 --ros --output output/console-resource-check
```

Live sensor and GUI checks run on a host with graphics access. Fresh-container package checks cover installation, asset/library closure, CPU physics and removal. Dated reports identify the exact tested version and conditions.
See the [completed console acceptance and resource measurements](docs/desktop-console-plan.md).
