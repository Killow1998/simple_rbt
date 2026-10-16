package main

import (
	"fmt"
	"github.com/egoist/mygo/ui"
	"strings"
)

func (a *app) navigate(page string) {
	if a.stage != "configure" {
		return
	}
	a.checkpoint()
	if a.pageViews == nil {
		a.pageViews = map[string]viewport{}
	}
	a.pageViews[a.page] = a.viewport
	v, ok := a.pageViews[page]
	if !ok {
		v = viewport{Azimuth: -135, Elevation: 25, Distance: 1.8}
		if page == "scene" {
			v.Distance = 25
			v.Azimuth = -45
			v.Elevation = 40
		}
	}
	a.viewport = v
	if page == "scene" && a.space != "Local / 设备" {
		a.space = "World / 世界"
	} else if page != "scene" && a.space == "World / 世界" {
		a.space = "Base / 机身"
	}
	a.page = page
	a.requestView()
}
func (a *app) navigation(c *ui.Context) {
	ui.Row(c).Padding(8, 18).Gap(8).Background(panelBackground(c)).Children(func() {
		for _, p := range []struct{ id, en, zh string }{{"home", "Launch", "启动"}, {"robot", "Robot presets", "机器人预设"}, {"scene", "Scene editor", "场景编辑"}} {
			p := p
			label := a.tr(p.en, p.zh)
			if a.page == p.id && a.stage == "configure" {
				label = "• " + label
			}
			ui.Button(c.Key("page-"+p.id), label).Disabled(a.stage != "configure").OnClick(func() { a.navigate(p.id) })
		}
		if a.stage != "configure" {
			ui.Text(c, a.tr("• Runtime", "• 运行操作台")).Bold()
		}
		ui.Text(c, "").Grow(1)
		if a.robotDirty() || a.sceneDirty() {
			ui.Text(c, a.tr("Drafts retained · save to update presets", "草稿已保留 · 保存后更新预设")).FontSize(12).TextColor(c.Theme().TextMuted)
		}
	})
}
func (a *app) sidebarWidth() float32 {
	if a.page == "scene" && a.stage == "configure" {
		return 260
	}
	return 380
}
func (a *app) pageSidebar(c *ui.Context) {
	switch a.page {
	case "robot":
		section(c, a.tr("Preset", "预设"), func() {
			ui.TextInput(c.Key("preset-name"), &a.robotPreset.Name).FillWidth().Label(a.tr("Preset name", "预设名称"))
			dirty := ""
			if a.robotDirty() {
				dirty = a.tr("Unsaved changes", "未保存的修改")
			}
			ui.Text(c, dirty).Height(22).TextColor(c.Theme().Warning)
			ui.Row(c).Gap(8).Children(func() {
				ui.Button(c, a.tr("Save preset", "保存预设")).OnClick(func() { a.saveRobot(false) })
				ui.Button(c, a.tr("Save as new", "另存为新预设")).OnClick(func() { a.saveRobot(true) })
			})
		})
		a.configuration(c)
	case "scene":
		a.sceneSidebar(c)
	default:
		a.launchSettings(c)
	}
}
func (a *app) pageAction(c *ui.Context) {
	switch a.page {
	case "scene":
		ui.PrimaryButton(c, a.tr("Use scene", "使用此场景")).FillWidth().OnClick(func() {
			a.useScene()
			if validateScene(&a.sceneDraft) == nil {
				a.navigate("home")
			}
		})
	case "robot":
		ui.PrimaryButton(c, a.tr("Back to launch", "返回启动页")).FillWidth().OnClick(func() { a.navigate("home") })
	default:
		ui.PrimaryButton(c, a.tr("Start session", "进入操控")).FillWidth().Disabled(a.loading || a.viewLoading || a.preparing || a.checking || !a.preflight.Ready || a.manager.Busy() || a.meta.State != "config").OnClick(a.startOperating)
	}
}
func (a *app) launchSettings(c *ui.Context) {
	section(c, a.tr("Robot preset", "机器人预设"), func() {
		choices := []string{}
		ids := map[string]string{}
		selected := ""
		all := clone(a.workspace.Robots)
		for id, r := range a.workspace.RobotDrafts {
			all[id] = r
		}
		all[a.robotPreset.ID] = a.robotPreset
		for _, id := range sortedKeys(all) {
			label := all[id].Name
			for n := 2; ids[label] != ""; n++ {
				label = fmt.Sprintf("%s (%d)", all[id].Name, n)
			}
			choices = append(choices, label)
			ids[label] = id
			if id == a.robotPreset.ID {
				selected = label
			}
		}
		if ui.Select(c.Key("robot-preset"), &selected, choices).FillWidth().Changed() {
			a.chooseRobot(ids[selected])
		}
		ui.Button(c, a.tr("Edit robot preset", "编辑机器人预设")).FillWidth().OnClick(func() { a.navigate("robot") })
	})
	section(c, a.tr("Scene", "场景"), func() {
		choices, ids, custom := a.sceneChoices()
		key := "terrain:" + a.cfg.Terrain + "-" + a.cfg.TerrainLevel
		if a.cfg.Scene != nil {
			key = a.cfg.Scene.ID
		}
		selected := ""
		for name, id := range ids {
			if id == key {
				selected = name
			}
		}
		if ui.Select(c.Key("scene-library"), &selected, choices).FillWidth().Changed() {
			a.chooseScene(ids[selected], custom)
		}
		ui.Button(c, a.tr("Open scene editor", "打开场景编辑器")).FillWidth().OnClick(func() {
			if a.cfg.Scene == nil {
				a.openTerrainScene()
			}
			a.navigate("scene")
		})
	})
	a.rosConfiguration(c)
}
func (a *app) changeRobotModel(name string) {
	old := a.robotPreset.Robot
	if old != "" {
		a.cfg.Robot = old
		a.checkpoint()
	}
	a.cfg.Robot = name
	a.cfg.Controller = a.catalog.Robots[name].DefaultController
	a.cfg.Mounts = map[string]Pose{}
	if a.catalog.Robots[name].Continuous && (a.cfg.Terrain == "stairs" || a.cfg.Terrain == "rough") {
		a.cfg.Terrain = "flat"
	}
	a.robotPreset = RobotPreset{ID: newID(), Name: a.optionLabel(name) + " · " + a.cfg.CameraModel, RobotConfig: robotConfig(a.cfg)}
	a.startConfig()
}

func (a *app) sceneChoices() ([]string, map[string]string, map[string]Scene) {
	names := []string{}
	ids := map[string]string{}
	add := func(name, id string) {
		label := name
		for n := 2; ids[label] != ""; n++ {
			label = fmt.Sprintf("%s (%d)", name, n)
		}
		names = append(names, label)
		ids[label] = id
	}
	for _, terrain := range a.terrains() {
		for _, level := range []string{"standard", "moderate", "hard", "extreme"} {
			if _, ok := a.catalog.Terrains[terrain+"-"+level]; ok {
				add(a.optionLabel(terrain)+" · "+a.optionLabel(level), "terrain:"+terrain+"-"+level)
			}
		}
	}
	all := clone(a.workspace.Scenes)
	for id, s := range a.workspace.SceneDrafts {
		all[id] = s
	}
	all[a.sceneDraft.ID] = a.sceneDraft
	for _, id := range sortedKeys(all) {
		add(all[id].Name, id)
	}
	return names, ids, all
}
func (a *app) chooseScene(id string, custom map[string]Scene) {
	a.checkpoint()
	if strings.HasPrefix(id, "terrain:") {
		key := strings.TrimPrefix(id, "terrain:")
		split := strings.LastIndex(key, "-")
		a.cfg.Terrain, a.cfg.TerrainLevel = key[:split], key[split+1:]
		a.cfg.Scene = nil
		a.startConfig()
	} else if scene, ok := custom[id]; ok {
		a.sceneDraft = clone(scene)
		a.sceneFloor = scene.Floors[0].ID
		a.sceneObject = ""
		a.useScene()
	}
}
func (a *app) openTerrainScene() {
	a.checkpoint()
	key := a.cfg.Terrain + "-" + a.cfg.TerrainLevel
	id := "terrain-edit:" + key
	if s, ok := a.workspace.SceneDrafts[id]; ok {
		a.sceneDraft = clone(s)
	} else {
		a.sceneDraft = a.terrainScene()
		a.sceneDraft.ID = id
	}
	a.sceneFloor = a.sceneDraft.Floors[0].ID
	a.sceneObject = ""
	a.hiddenFloors = map[string]bool{}
}
