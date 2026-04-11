# simple_rbt

## Quick Start

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
