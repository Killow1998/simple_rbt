package main

import (
	"fmt"
	"github.com/egoist/mygo/ui"
	"simple-rbt/console/session"
)

func (a *app) tutorialState(state string) string {
	if state == "invalid_pose" {
		return a.tr("Invalid estimated pose", "估计位姿无效")
	}
	if state == "storage_error" {
		return a.tr("Trajectory write failed", "轨迹写入失败")
	}
	labels := map[string][2]string{"preparing": {"Preparing calibration", "正在准备标定"}, "initializing": {"Initializing · keep stationary", "正在初始化 · 请保持静止"}, "running": {"Receiving estimated poses", "正在接收估计位姿"}, "stale": {"Estimated poses delayed", "估计位姿延迟"}, "failed": {"Tutorial failed", "教程失败"}, "stopped": {"Stopped", "已停止"}}
	if label, ok := labels[state]; ok {
		return a.tr(label[0], label[1])
	}
	return a.tr("Not started", "尚未启动")
}
func (a *app) tutorialSummary(c *ui.Context) {
	if a.cfg.Tutorial == "" {
		return
	}
	state := a.manager.TutorialStatus()
	ui.Text(c, a.optionLabel(a.cfg.Tutorial)+" · "+a.tutorialState(state.State)).FontSize(12).TextColor(c.Theme().TextMuted)
	if state.Lag != nil {
		ui.Text(c, fmt.Sprintf(a.tr("Pose delay %.3f s · %d poses", "位姿延迟 %.3f 秒 · %d 帧"), *state.Lag, state.Poses)).FontSize(12)
	}
}
func (a *app) tutorialPage(c *ui.Context) {
	ui.Scroll(c).Grow(1).MinHeight(0).Padding(24).Children(func() {
		ui.Row(c).Gap(12).Children(func() {
			ui.Text(c, a.tr("Mapping tutorials", "建图教程")).FontSize(22).Bold().Grow(1)
			ui.Button(c, a.tr("Back", "返回")).OnClick(func() { a.tutorialOpen = false })
		})
		section(c, a.tr("1 · Sensor contract", "1 · 传感器接口"), func() {
			ui.Row(c).Gap(10).Children(func() {
				for _, robot := range []string{"go2w", "quadrotor"} {
					label := a.tr("Load Go2W example", "加载 Go2W 示例")
					if robot == "quadrotor" {
						label = a.tr("Load flight example", "加载飞行示例")
					}
					ui.Button(c, label).Disabled(a.stage != "configure").OnClick(a.action("tutorial.example."+robot, func() {
						cfg := defaultConfig()
						if err := loadJSON(a.manager.TutorialProfile(robot), &cfg); err != nil {
							a.status = err.Error()
							a.showLog = true
							return
						}
						a.applySessionProfile(cfg, label)
					}))
				}
			})
			ui.Text(c, a.tr("Start with Go2W + Noetic. The same inputs also support Humble and the quadrotor.", "先使用 Go2W＋Noetic；相同接口也支持 Humble 和四旋翼。"))
			ui.Text(c, "/livox/lidar · /livox/imu_si · /camera/image_raw · /camera/camera_info · /clock").FontSize(13)
			ui.Text(c, a.tr("LiDAR and IMU stay rigidly coupled. Calibration comes from the current mounts and actual RGB frame size.", "雷达与 IMU 保持刚性耦合。标定取自当前外参和实际 RGB 图像尺寸。"))
			ui.Text(c, a.tr("Ground truth is only for separate evaluation. The frontends receive sensor data.", "真值仅用于独立评估。前端只接收传感器数据。"))
		})
		section(c, a.tr("2 · Select a frontend", "2 · 选择前端"), func() {
			selected := a.cfg.Tutorial
			if selected == "" {
				selected = "none"
			}
			version := a.cfg.TutorialROS
			if version == "" {
				version = "ros1"
			}
			changed := false
			field(c, a.tr("Algorithm", "算法"), func() {
				changed = a.localizedSelect(c.Key("tutorial-algorithm"), &selected, []string{"none", "fast-lio", "fast-livo2"}).Width(280).Disabled(a.stage != "configure").Changed() || changed
			})
			field(c, a.tr("ROS adapter", "ROS 适配器"), func() {
				changed = a.localizedSelect(c.Key("tutorial-ros"), &version, []string{"ros1", "ros2"}).Width(280).Disabled(a.stage != "configure").Changed() || changed
			})
			if changed {
				a.cfg.Tutorial = selected
				a.cfg.TutorialROS = version
				if selected == "none" {
					a.cfg.Tutorial = ""
				} else {
					if a.cfg.Adapters == nil {
						a.cfg.Adapters = map[string]bool{}
					}
					a.cfg.Adapters[version] = true
					a.cfg.Sensors.MID360 = true
					if selected == "fast-livo2" {
						a.cfg.Sensors.Camera = true
					}
					a.cfg.ROS = ROSConfig{false, "/ground_truth/odom"}
				}
				a.startConfig()
			}
			ui.Text(c, a.tr("FAST-LIO uses LiDAR + IMU. FAST-LIVO2 also uses RGB; choose a scene with visible texture and geometry.", "FAST-LIO 使用雷达与 IMU。FAST-LIVO2 还使用 RGB；请选用具有可见纹理和几何特征的场景。"))
			ui.Text(c, a.tr("Enable this before starting a session. Keep still until estimated poses arrive, then drive or fly.", "请在进入操控前选择。等待估计位姿出现，再开始行驶或飞行。"))
		})
		section(c, a.tr("3 · Optional installation", "3 · 可选安装"), func() {
			version := a.cfg.TutorialROS
			if version == "" {
				version = "ros1"
			}
			ui.Text(c, a.tr("Required image: ", "所需镜像：")+session.TutorialImage(version))
			ui.Text(c, a.tr("The image contains independent C++ frontends. It is prepared separately and does not replace the simulator.", "镜像包含独立的 C++ 前端。它单独准备，不替换仿真器。"))
			ui.Text(c, a.tr("Preparation and reproducible commands: ", "准备与复现说明：")+a.manager.TutorialGuide()).FontSize(13)
		})
		section(c, a.tr("4 · Current run", "4 · 当前运行"), func() {
			state := a.manager.TutorialStatus()
			ui.Text(c, a.tutorialState(state.State)).Bold()
			a.tutorialSummary(c)
			if len(state.Position) == 3 {
				ui.Text(c, fmt.Sprintf("X %.3f · Y %.3f · Z %.3f m", state.Position[0], state.Position[1], state.Position[2]))
			}
			if state.Error != "" {
				ui.Text(c, state.Error).TextColor(c.Theme().Danger)
			}
			if state.TrajectoryFull {
				ui.Text(c, a.tr("Trajectory reached 100 MB. Recording stopped; the frontend continues.", "轨迹已达 100 MB，记录已停止，前端继续运行。")).TextColor(c.Theme().Warning)
			}
			if state.Output != "" {
				ui.Text(c, a.tr("Configuration, logs and estimated trajectory: ", "配置、日志与估计轨迹：")+state.Output).FontSize(12)
			}
			ui.Button(c, a.tr("Stop frontend", "停止前端")).Disabled(state.State == "" || state.State == "stopped" || state.State == "failed").OnClick(a.action("tutorial.stop", func() { go a.manager.StopTutorial() }))
			ui.Text(c, a.tr("Mapping success does not certify relocalization or real-hardware performance. Avia map recovery still has known limits.", "建图通过不代表重定位或真机通过。Avia 地图恢复仍有已知限制。"))
		})
	})
}
