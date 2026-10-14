# simple_rbt

[English](README.md) | [简体中文](README.zh-CN.md)

A MuJoCo robot simulator built with C++17 and ROS 2 Humble. Send velocity commands and receive physical motion, MID360 LiDAR/IMU, perspective and panoramic cameras, odometry, and TF.

Supported robots: Go2, Go2W, a two-wheel differential-drive reference, Scout Mini, FR-mini, Ridgeback, and Ranger Mini. Wheeled chassis use flat ground, ramps, and continuous hills; Go2 and Go2W also support stairs and stepped terrain.

## Quick start

Run these commands from the repository root:

```bash
# Prepare SDKs, models, policies, and the native build.
./setup.sh

# Rebuild messages and C++ with SDKs/assets already prepared.
./setup.sh --native-only

# Full sensors, 10 Hz cameras, real-time pacing, and a desktop viewer.
./run.sh --robot fr_mini --cmd .3 0 0 --sensors full \
  --backend cuda --camera-hz 10 --realtime --viewer --duration 10

./run.sh --help
```

`run.sh` starts the C++ runtime. Python supports offline preparation and an independent reference implementation.

`--cmd VX VY WZ` sets forward/lateral velocity in m/s and yaw rate in rad/s. With `--ros2`, commands come from `/<robot>/cmd_vel`; the robot stops after 0.5 seconds without a fresh command. For example, Go2W accepts `/go2w/cmd_vel`.

Use `--control-mode native` for the chassis controller alone or `tracked` for additional simulator velocity/heading feedback. Commanded speed is not a guarantee of actual speed on every terrain.

## Repository layout

```text
simple_rbt/
├── CMakeLists.txt          C++ build entry
├── setup.sh / run.sh       Preparation and launch
├── src/                    Physics, controllers, sensors, ROS, scheduling
├── include/simple_rbt/     C++ module interfaces
├── assets/                 Models, policies, parameters, scan data
├── tests/                  Native and Noetic integration tests
├── tools/                  Preparation, measurements, end-to-end checks
├── reference/python/       Independent Python implementation and tests
├── ros2_interfaces/        ROS message definitions
├── third_party/            Pinned headers and licenses
├── docs/                   Usage, design, and validation
├── build/                  Local build products (ignored by Git)
└── output/                 Experiments and test results (ignored by Git)
```

`.venv/` supplies the MuJoCo/ONNX Runtime SDKs and reference tools; `.vendor/` contains model meshes and manifest-managed dependencies. Both are ignored by Git.

See the [runtime guide](docs/runtime.md), [robot and sensor guide](docs/python-reference.md), and [asset guide](assets/README.md) for detailed instructions in Simplified Chinese.

## Validation

```bash
source tools/ros_env.sh
ctest --test-dir build/native --output-on-failure
PYTHONPATH=reference/python .venv/bin/python -m pytest reference/python/tests
.venv/bin/python tools/validate_full_run.py --runtime cpp --robot fr_mini \
  --terrain ramp --control-mode native --speed .6 --output output/full-fr-new
```

Use a new output directory for each run. GPU and live ROS checks require the corresponding host environment. See the [validation records](docs/README.md) for tested versions and conditions.

## Noetic Docker tests

Run optional ROS 1 interoperability checks with an existing Noetic image. See the [Noetic test guide](docs/noetic.md) for image preparation and test coverage.

```bash
.venv/bin/python tools/test_noetic.py --skip-build
```
