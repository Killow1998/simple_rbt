package main

import (
	"fmt"
	"github.com/egoist/mygo"
	"github.com/egoist/mygo/ui"
	"path/filepath"
	"strings"
)

func (a *app) controllers() []string {
	values := []string{}
	prefix := a.cfg.Robot + "-"
	for _, key := range sortedKeys(a.catalog.Models) {
		if strings.HasPrefix(key, prefix) {
			values = append(values, strings.TrimPrefix(key, prefix))
		}
	}
	return values
}
func (a *app) terrains() []string {
	values := []string{}
	spec := a.catalog.Robots[a.cfg.Robot]
	for _, key := range sortedKeys(a.catalog.Terrains) {
		name := key[:strings.LastIndex(key, "-")]
		if spec.Continuous && name != "flat" && name != "ramp" && name != "hills" {
			continue
		}
		if len(values) == 0 || values[len(values)-1] != name {
			values = append(values, name)
		}
	}
	return values
}
func (a *app) configuration(c *ui.Context) {
	section(c, a.tr("Robot", "机器人"), func() {
		field(c, a.tr("Robot", "机器人"), func() {
			if a.localizedSelect(c.Key("robot"), &a.cfg.Robot, sortedKeys(a.catalog.Robots)).FillWidth().Changed() {
				a.changeRobotModel(a.cfg.Robot)
			}
		})
		field(c, a.tr("Controller", "控制器"), func() {
			if a.localizedSelect(c.Key("controller"), &a.cfg.Controller, a.controllers()).FillWidth().Changed() {
				a.startConfig()
			}
		})
		ui.Row(c).Gap(10).Children(func() {
			field(c, a.tr("Motion", "运动"), func() {
				if a.localizedSelect(c.Key("motion"), &a.cfg.ControlMode, []string{"tracked", "native"}).Width(166).Changed() {
					a.startConfig()
				}
			})
			field(c, a.tr("LiDAR computation", "雷达计算"), func() {
				if a.localizedSelect(c.Key("backend"), &a.cfg.Backend, []string{"auto", "cuda", "cpu"}).Width(166).Changed() {
					a.startConfig()
				}
			})
		})
		spec := a.catalog.Robots[a.cfg.Robot]
		ui.Text(c, fmt.Sprintf(a.tr("Limits · vx ±%.2f m/s · wz ±%.2f rad/s", "限幅 · vx ±%.2f 米/秒 · wz ±%.2f 弧度/秒"), spec.Limits[0], spec.Limits[2])).FontSize(12).TextColor(c.Theme().TextMuted)
		if !spec.AllowsLateral {
			ui.Text(c, a.tr("No lateral motion", "不支持横向平移")).FontSize(12).TextColor(c.Theme().TextMuted)
		}
		if !spec.AllowsTurn {
			ui.Text(c, fmt.Sprintf(a.tr("No in-place turn · minimum radius %.2f m", "不支持原地转向 · 最小转弯半径 %.2f 米"), spec.MinRadius)).FontSize(12).TextColor(c.Theme().TextMuted)
		}
		if spec.Continuous {
			ui.Text(c, a.tr("Continuous terrain only", "仅支持连续地形")).TextColor(c.Theme().TextMuted).FontSize(12)
		}
	})
	section(c, a.tr("Sensors", "传感器"), func() {
		changed := false
		ui.Row(c).Gap(14).Children(func() {
			if ui.Checkbox(c.Key("mid360"), &a.cfg.Sensors.MID360, a.tr("MID360 · LiDAR + IMU", "MID360 · 雷达与 IMU")).Changed() {
				changed = true
			}
			if ui.Checkbox(c.Key("rgb"), &a.cfg.Sensors.Camera, "RGB").Changed() {
				changed = true
			}
		})
		ui.Row(c).Gap(14).Children(func() {
			if ui.Checkbox(c.Key("depth"), &a.cfg.Sensors.Depth, a.tr("Depth", "深度")).Changed() {
				changed = true
			}
			if ui.Checkbox(c.Key("panorama"), &a.cfg.Sensors.Panorama, a.tr("Panorama", "全景")).Changed() {
				changed = true
			}
		})
		field(c, a.tr("Camera device", "相机型号"), func() {
			if ui.Select(c.Key("camera-model"), &a.cfg.CameraModel, []string{"d435", "d435i"}).FillWidth().Changed() {
				changed = true
			}
		})
		if a.cfg.CameraModel == "d435i" {
			ui.Text(c, a.tr("Built-in IMU · 200 Hz · /camera/imu", "内置 IMU · 200 Hz · /camera/imu")).FontSize(12).TextColor(c.Theme().TextMuted)
		}
		if changed {
			a.startConfig()
		}
	})
	if a.cfg.Sensors.MID360 || a.cameraDeviceEnabled() {
		a.mountInspector(c)
	}
}
func (a *app) rosConfiguration(c *ui.Context) {
	section(c, a.tr("Optional ROS", "可选 ROS"), func() {
		for _, version := range []string{"ros1", "ros2"} {
			selected := a.cfg.Adapters[version]
			label := "ROS 1 · Noetic"
			if version == "ros2" {
				label = "ROS 2 · Humble"
			}
			if ui.Checkbox(c.Key(version), &selected, label).Changed() {
				a.cfg.Adapters[version] = selected
				a.beginPreflight()
			}
		}
		ui.Text(c, a.tr("Odometry is simulation ground truth", "里程计来自仿真真值")).FontSize(12).TextColor(c.Theme().TextMuted)
		if ui.Checkbox(c.Key("odom-tf"), &a.cfg.ROS.PublishOdomTF, a.tr("Publish odom → base_link TF", "发布 odom → base_link TF")).Changed() {
			a.startConfig()
		}
		topicMode := a.cfg.ROS.OdomTopic
		field(c, a.tr("Ground-truth odometry topic", "真值里程计话题"), func() {
			if ui.Select(c.Key("odom-topic"), &topicMode, odomTopicChoices(topicMode)).FillWidth().Changed() {
				a.cfg.ROS.OdomTopic = topicMode
				a.startConfig()
			}
		})
		ui.Text(c, a.tr("External localization: disable this TF and use /ground_truth/odom for comparison only.", "外部定位：关闭此 TF，选择 /ground_truth/odom 仅作对照。")).FontSize(12).TextColor(c.Theme().TextMuted)
		ui.Button(c, a.tr("Prepare adapters", "准备适配器")).FillWidth().Disabled(a.preparing || a.manager.Busy()).OnClick(a.action("ui_config.prepare_adapters", func() { a.prepareAdapters(false) }))
	})
	ui.Row(c).Gap(10).Children(func() {
		ui.Button(c, a.tr("Export session", "导出会话")).Grow(1).OnClick(a.action("ui_config.export_session", a.saveProfile))
		ui.Button(c, a.tr("Import session", "导入会话")).Grow(1).OnClick(a.action("ui_config.import_session", a.loadProfile))
	})
}
func (a *app) cameraDeviceEnabled() bool { return a.cameraEnabled() || a.cfg.CameraModel == "d435i" }
func (a *app) cameraEnabled() bool {
	return a.cfg.Sensors.Camera || a.cfg.Sensors.Depth || a.cfg.Sensors.Panorama
}
func (a *app) saveProfile() {
	path, err := mygo.Dialog.Save(mygo.SaveDialogOptions{Parent: a.win, Title: a.tr("Save session", "保存配置"), DefaultPath: profilePath(), Filters: []mygo.FileFilter{{Name: "JSON", Extensions: []string{"json"}}}})
	if err == nil && path != "" {
		err = saveJSON(path, a.cfg)
	}
	if err != nil {
		a.status = err.Error()
		a.showLog = true
	}
}
func (a *app) loadProfile() {
	paths, err := mygo.Dialog.Open(mygo.OpenDialogOptions{Parent: a.win, Title: a.tr("Load session", "加载配置"), DefaultPath: filepath.Dir(profilePath()), Filters: []mygo.FileFilter{{Name: "JSON", Extensions: []string{"json"}}}})
	if err != nil {
		a.status = err.Error()
		a.showLog = true
		return
	}
	if len(paths) == 0 {
		return
	}
	cfg := defaultConfig()
	if err = loadJSON(paths[0], &cfg); err != nil {
		a.status = err.Error()
		a.showLog = true
		return
	}
	if err = validateConfig(cfg, a.catalog); err != nil {
		a.status = err.Error()
		a.showLog = true
		return
	}
	if cfg.Mounts == nil {
		cfg.Mounts = map[string]Pose{}
	}
	if cfg.Adapters == nil {
		cfg.Adapters = map[string]bool{}
	}
	if cfg.Controller == "auto" {
		cfg.Controller = a.catalog.Robots[cfg.Robot].DefaultController
	}
	a.checkpoint()
	a.cfg = cfg
	a.robotPreset = RobotPreset{ID: newID(), Name: filepath.Base(paths[0]), RobotConfig: robotConfig(cfg)}
	if cfg.Scene != nil {
		a.sceneDraft = clone(*cfg.Scene)
		a.sceneFloor = a.sceneDraft.Floors[0].ID
		a.sceneObject = ""
	}
	a.startConfig()
}
