package main

import (
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
	section(c, a.tr("01 · Robot & scene", "01 · 机器人与场景"), func() {
		field(c, a.tr("Robot", "机器人"), func() {
			if a.localizedSelect(c.Key("robot"), &a.cfg.Robot, sortedKeys(a.catalog.Robots)).FillWidth().Changed() {
				a.cfg.Controller = a.catalog.Robots[a.cfg.Robot].DefaultController
				a.cfg.Mounts = map[string]Pose{}
				if a.catalog.Robots[a.cfg.Robot].Continuous && (a.cfg.Terrain == "stairs" || a.cfg.Terrain == "rough") {
					a.cfg.Terrain = "flat"
				}
				a.startConfig()
			}
		})
		field(c, a.tr("Controller", "控制器"), func() {
			if a.localizedSelect(c.Key("controller"), &a.cfg.Controller, a.controllers()).FillWidth().Changed() {
				a.startConfig()
			}
		})
		ui.Row(c).Gap(10).Children(func() {
			field(c, a.tr("Terrain", "地形"), func() {
				if a.localizedSelect(c.Key("terrain"), &a.cfg.Terrain, a.terrains()).Width(166).Changed() {
					a.startConfig()
				}
			})
			field(c, a.tr("Level", "强度"), func() {
				if a.localizedSelect(c.Key("level"), &a.cfg.TerrainLevel, []string{"standard", "moderate", "hard", "extreme"}).Width(166).Changed() {
					a.startConfig()
				}
			})
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
		if a.catalog.Robots[a.cfg.Robot].Continuous {
			ui.Text(c, a.tr("Continuous terrain only", "仅支持连续地形")).TextColor(c.Theme().TextMuted).FontSize(12)
		}
	})
	section(c, a.tr("02 · Sensors", "02 · 传感器"), func() {
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
		if changed {
			a.startConfig()
		}
	})
	if a.cfg.Sensors.MID360 || a.cameraEnabled() {
		a.mountInspector(c)
	}
	section(c, a.tr("04 · Optional ROS", "04 · 可选 ROS"), func() {
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
		ui.Button(c, a.tr("Prepare adapters", "准备适配器")).FillWidth().Disabled(a.preparing || a.manager.Busy()).OnClick(func() { a.prepareAdapters(false) })
	})
	ui.Row(c).Gap(10).Children(func() {
		ui.Button(c, a.tr("Save", "保存")).Grow(1).OnClick(a.saveProfile)
		ui.Button(c, a.tr("Load", "加载")).Grow(1).OnClick(a.loadProfile)
	})
}
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
	if _, ok := a.catalog.Robots[cfg.Robot]; !ok {
		a.status = "Unknown robot / 未注册的机器人"
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
	a.cfg = cfg
	a.startConfig()
}
