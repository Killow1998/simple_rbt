# Quadrotor / 四旋翼

SpRobo includes a fixed-pitch Skydio X2 quadrotor and a small C++ controller.
Choose **Skydio X2 → Cascaded flight control**, or load `examples/skydio-x2.json`.
The model comes from the pinned Menagerie source recorded in
`assets/visual/skydio_x2/provenance.json`. `tools/prepare_quadrotor.py` reproduces it.

SpRobo 提供 Skydio X2 四旋翼及简约 C++ 控制器。选择 **Skydio X2 → 级联飞行控制**，
或加载 `examples/skydio-x2.json`。控制器通过四个旋翼的推力与反扭矩驱动机体。
它使用仿真状态稳定飞行；不把真值发送给建图前端。

## Controls / 操控

| Action / 操作 | Behavior / 行为 |
|---|---|
| Take off / 起飞 | Ascend to the requested height above local support / 相对脚下支撑面上升 |
| Land / 降落 | Descend and stop rotors near support / 下降，接近支撑面后停桨 |
| Hover / 悬停 | Hold current position and yaw / 保持当前位置与偏航角 |
| Pause / 暂停 | Freeze simulation time / 暂停物理时间 |
| Cut motors / 停桨 | Set thrust to zero; an airborne vehicle falls / 推力归零，空中会下坠 |

Click the joystick or driving viewport before using keys. W/S and A/D move in
horizontal heading coordinates; R/F climb/descend; Q/E turn; Space holds position.
Release or manual focus loss requests hold. Program ownership survives focus loss.
The normal 0.5-second timeout and owner-disconnect behavior also request hold.

点击摇杆或操控画面后可使用键盘。W/S 前后、A/D 横移，方向随当前水平朝向变化；
R/F 升降，Q/E 偏航，空格悬停。松开或手动失焦后悬停。程序控制不受窗口焦点影响，
但仍受 0.5 秒指令超时和控制端断开保护约束。

## Program contract / 程序接口

Flight IPC uses kind **18** on the existing command socket. Select Program · IPC
as the command owner. Example payload:

```json
{"frame":"world_enu","action":"velocity","velocity":[0.3,0.0,0.2],"yaw_rate":0.1,"takeoff_height":1.0}
```

Linear velocity is world ENU in m/s; yaw rate is rad/s. Supported actions are
`velocity`, `hold`, `takeoff`, `land`, `disarm`. Non-velocity actions require zero
velocity/yaw rate. The core rejects unknown frames/actions, nonfinite values and
the legacy planar IPC command. `hello.flight_control` advertises support.
Limits: 1 m/s per horizontal axis, 0.6 m/s vertical, 1 rad/s yaw, 25° tilt.

飞行指令使用世界 ENU 坐标，线速度单位 m/s，偏航角速度单位 rad/s。
横滚与俯仰由控制器计算。旧平面指令、未知坐标系和无效数值会被拒绝。
`hold`、`takeoff`、`land`、`disarm` 操作的速度分量必须为零。

Native Noetic and Humble wrappers expose:

- `/skydio_x2/cmd_vel` (`geometry_msgs/Twist`): world ENU `linear.xyz`, `angular.z`.
  Nonzero `angular.x/y` is rejected.
- `/skydio_x2/flight_action` (`std_msgs/String`): `takeoff`, `hold`, `land`, `disarm`.
- `takeoff_height` wrapper parameter: local-support-relative height, default 1 m.

选择对应 ROS 指令源后发布上述话题。起飞、降落及速度指令需要持续刷新，
不能依靠一次发送长期保持动作。停发后按超时规则悬停。
ROS 1 与 ROS 2 分别直连核心 IPC，互不依赖。

## Physics and limits / 物理与边界

Dry mass is 1.325 kg. Enabled sensor packages add nominal mass and inertia:
MID360 0.270 kg, Avia 0.500 kg, D435/D435i 0.075 kg. Mount changes update payload
COM, inertia and collision boxes. These are simplified payload models, not a
hardware calibration. Exported model XML preserves the added inertial/collision
properties. RGB rotor discs are translucent visuals; they do not add lift.

空机质量 1.325 kg。已启用传感器按上述标称值加入质量、惯量及碰撞体；修改外参时同步
更新。起飞检查上方空间，天花板和墙面参与碰撞。控制器不提供自动避障，也不模拟
电池、风场、桨叶气动细节或真实飞控故障。建图估计器失效不改变初期稳定控制器的反馈源。

Physics, controller, IMU, LiDAR and camera schedules remain separate. CPU execution
needs no CUDA. Sensor resolution and rate stay configurable through existing
profiles. The tutorials use 320×240 RGB/depth at 10 Hz, MID360 at 10 Hz and IMU
at 200 Hz. See [tutorials](tutorials.md) and the dated
[acceptance record](desktop-console-plan.md#quadrotor-and-algorithm-tutorials--requirements-2026-10-28).
