# simple_rbt

## 🚀 Quick Start

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

## 🙏 Thanks

* **WPR Simulation:** [https://github.com/6-robot/wpr_simulation](https://github.com/6-robot/wpr_simulation)
* **UGV Gazebo Sim:** [https://github.com/agilexrobotics/ugv_gazebo_sim](https://github.com/agilexrobotics/ugv_gazebo_sim)
* **Livox ROS Driver2:** [https://github.com/Livox-SDK/livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)
* **Mid360 Simulation Plugin:** [https://github.com/fratopa/Mid360_simulation_plugin](https://github.com/fratopa/Mid360_simulation_plugin)
* **Limo Simulator:** [https://github.com/limo-agx/limo_simulator](https://github.com/limo-agx/limo_simulator)
* **Omni Wheel Robot GazeboSim:** [https://github.com/KairongWu/Omni_wheel_robot_GazeboSim](https://github.com/KairongWu/Omni_wheel_robot_GazeboSim)