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


## 🙏 Thanks

* **WPR Simulation:** [https://github.com/6-robot/wpr_simulation](https://github.com/6-robot/wpr_simulation)
* **UGV Gazebo Sim:** [https://github.com/agilexrobotics/ugv_gazebo_sim](https://github.com/agilexrobotics/ugv_gazebo_sim)
* **Livox ROS Driver2:** [https://github.com/Livox-SDK/livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)
* **Mid360 Simulation Plugin:** [https://github.com/fratopa/Mid360_simulation_plugin](https://github.com/fratopa/Mid360_simulation_plugin)