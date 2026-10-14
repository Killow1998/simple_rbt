# simple_rbt

## MuJoCo prototype (Jammy / ROS 2 Humble)

The new [MuJoCo simulator](mujoco_sim/README.md) runs seven configurations: Go2,
Go2W, a simple differential-drive base, Scout Mini, FR-mini, Ridgeback R100 and
Ranger Mini 3.0. Pure wheel chassis use flat ground, ramps and continuous hills;
legged models also retain stairs and step-cell roughness. The default runtime is now
C++17: controllers, physics scheduling, MID360 CPU/native CUDA ray casting,
IMU, perspective RGB/depth, panoramic RGB/range and the desktop viewer.
Its optional ROS 2 transport accepts `/<robot>/cmd_vel` and publishes Livox messages,
images, odometry, clock and TF. See the linked guide for tested commands and
current limits; the legacy robot collection has not all been migrated.
The [C++ migration record](mujoco_sim/docs/cpp-runtime-validation-20261013.md)
contains equivalence, complete-course acceptance and CPU/RAM measurements.
The current [architecture and ablation record](mujoco_sim/docs/cpp-architecture-ablation-20261013.md)
records the module split, unchanged physics, fault-detection experiments and repeated
full ROS comparisons. Peak process RSS fell by 8.7–13.2 MiB for the three measured
wheel chassis; the small CPU differences do not establish a speedup. Those measurements
precede the subsequent differential-drive caster classification fix. The final version
preserves chassis-ground collision checks and passes all 31 formal checks for each
of the seven robots, plus 10 CTest cases and 392 Python tests with 65 subtests.
Python remains available through `mujoco_sim/run-python.sh` for reference tests and
offline model preparation; the normal launcher does not start Python.
The latest [deployment and full ROS validation](mujoco_sim/docs/deployment-validation-20261013.md)
records Go2 startup/torque/material comparisons and Go2W's complete multi-terrain,
sensor, ROS and real-time acceptance. Go2's higher-terrain speed limits remain explicit.
The current [MID360 alignment](mujoco_sim/docs/mid360-validation-20261013.md)
adds the device's nominal internal IMU geometry, 200 Hz sampling, explicit raw/SI
outputs and per-point moving LiDAR. Historical snapshot results remain separate.
The [wheel-chassis deployment record](mujoco_sim/docs/wheel-chassis-validation-20261013.md)
covers Ackermann, passive-roller Mecanum and four-module steering, their named ROS
interfaces, continuous-surface acceptance and measured resource use. These are
contact-driven engineering models, not calibrated hardware replicas.
The historical [Scout Mini first-stage record](mujoco_sim/docs/scout-mini-validation-20261013.md)
preserves its earlier terrain results; step traversal is no longer a wheel-chassis requirement.

```bash
./mujoco_sim/setup.sh
./mujoco_sim/run.sh --robot go2w --terrain stairs --backend warp --cmd .6 0 0 --duration 15 --sensors full --camera-hz 10 --realtime
```

## Legacy Gazebo / ROS 1 Quick Start

### 1. WPR Home with Laser Camera IMU

**Terminal 1 (Gazebo World):**
```bash
source devel/setup.bash
roslaunch simple_rbt gazebo_empty_world.launch
```

**Terminal 2 (Spawn Model):**
```bash
source devel/setup.bash
roslaunch simple_rbt wpb_home_laser_camera_imu.launch
```

---

### 2. Tracer with Laser IMU

**Terminal 1 (Gazebo World):**
```bash
source devel/setup.bash
roslaunch simple_rbt gazebo_empty_world.launch
```

**Terminal 2 (Spawn Model):**
```bash
source devel/setup.bash
roslaunch simple_rbt tracer_laser_imu.launch
```

---

### 3. Tracer with Mid360

**Terminal 1 (Gazebo World):**
```bash
source devel/setup.bash
roslaunch simple_rbt gazebo_empty_world.launch
```

**Terminal 2 (Spawn Model):**
```bash
source devel/setup.bash
roslaunch simple_rbt tracer_mid360.launch
```

---

### 4. Tracer with Camera IMU

**Terminal 1 (Gazebo World):**
```bash
source devel/setup.bash
roslaunch simple_rbt gazebo_empty_world.launch
```

**Terminal 2 (Spawn Model):**
```bash
source devel/setup.bash
roslaunch simple_rbt tracer_front_cam_imu.launch
```

---

### 5. Scout Mini

**Terminal 1 (Gazebo World):**
```bash
source devel/setup.bash
roslaunch simple_rbt gazebo_empty_world.launch
```

**Terminal 2 (Spawn Model):**
```bash
source devel/setup.bash
roslaunch simple_rbt scout_mini.launch
```

---

### 6. Limo

**Terminal 1 (Gazebo World):**
```bash
source devel/setup.bash
roslaunch simple_rbt gazebo_empty_world.launch
```

**Terminal 2 (Spawn Model & Controllers):**
```bash
source devel/setup.bash
# Choose one of the sensor configurations:
roslaunch simple_rbt limo_mid360.launch
# roslaunch simple_rbt limo_laser_imu.launch
# roslaunch simple_rbt limo_front_cam_imu.launch
```

---

### 7. Omni Wheel Robot

**Terminal 1 (Gazebo World):**
```bash
source devel/setup.bash
roslaunch simple_rbt gazebo_empty_world.launch
```

**Terminal 2 (Spawn Model & Controllers):**
```bash
source devel/setup.bash
# Choose one of the sensor configurations:
roslaunch simple_rbt omni_mid360.launch
# roslaunch simple_rbt omni_laser_imu.launch
# roslaunch simple_rbt omni_front_cam_imu.launch
```

---

### 8. Go2W (High-Level Controller)

**Terminal 1 (Gazebo World):**
```bash
source devel/setup.bash
roslaunch simple_rbt gazebo_empty_world.launch
```

**Terminal 2 (Spawn Model & High-Level GUI):**
```bash
source devel/setup.bash
roslaunch simple_rbt go2w_mid360_cam_high.launch
```

---

### 9. Go2W (Low-Level Joint Controller)

**Terminal 1 (Gazebo World):**
```bash
source devel/setup.bash
roslaunch simple_rbt gazebo_empty_world.launch
```

**Terminal 2 (Spawn Model & Joint GUI):**
```bash
source devel/setup.bash
roslaunch simple_rbt go2w_mid360_cam_low.launch
```

## Thanks

* **WPR Simulation:** [https://github.com/6-robot/wpr_simulation](https://github.com/6-robot/wpr_simulation)
* **UGV Gazebo Sim:** [https://github.com/agilexrobotics/ugv_gazebo_sim](https://github.com/agilexrobotics/ugv_gazebo_sim)
* **Livox ROS Driver2:** [https://github.com/Livox-SDK/livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)
* **Mid360 Simulation Plugin:** [https://github.com/fratopa/Mid360_simulation_plugin](https://github.com/fratopa/Mid360_simulation_plugin)
* **Limo Simulator:** [https://github.com/limo-agx/limo_simulator](https://github.com/limo-agx/limo_simulator)
* **Omni Wheel Robot GazeboSim:** [https://github.com/KairongWu/Omni_wheel_robot_GazeboSim](https://github.com/KairongWu/Omni_wheel_robot_GazeboSim)
* **Unitree Go2W ROS2:** [https://github.com/Sam-Mag1/unitree_go2w_ros2](https://github.com/Sam-Mag1/unitree_go2w_ros2)
## Dependencies
```bash
sudo apt install -y python3-rosdep python3-catkin-tools libyaml-cpp-dev libopencv-dev libboost-system-dev libboost-thread-dev ros-noetic-xacro ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-controller-manager ros-noetic-joint-state-controller ros-noetic-joint-trajectory-controller ros-noetic-velocity-controllers ros-noetic-cv-bridge ros-noetic-image-transport ros-noetic-tf2 ros-noetic-tf2-ros ros-noetic-tf2-geometry-msgs
```
