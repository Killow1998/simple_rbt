package main

import (
	"fmt"
	"github.com/egoist/mygo/ui"
	"strings"
)

func (a *app) view(c *ui.Context) {
	defer a.checkDriveFocus(c)
	if a.page == "scene" && a.stage == "configure" {
		a.beginSceneFrame()
		defer a.endSceneFrame()
	}
	theme := themeFor(a.preferences.Theme)
	c.SetTheme(theme)
	ui.Column(c).Fill().Background(theme.Background).Children(func() {
		ui.Row(c).Height(66).Padding(14, 22).Background(panelBackground(c)).BorderWidth(0, 0, 1, 0).BorderColor(panelDivider(c)).Gap(12).Children(func() {
			ui.Text(c, "SpRobo").FontSize(23).Bold()
			ui.Text(c, a.tr("Robot simulation", "机器人仿真")).TextColor(theme.TextMuted).Grow(1)
			a.appearanceControls(c)
			ui.Badge(c, a.stateLabel())
		})
		a.navigation(c)
		if !a.initialized {
			ui.Scroll(c).Grow(1).MinHeight(0).Children(func() { a.dependencyPanel(c) })
			return
		}
		if a.showPreflight {
			ui.Scroll(c).Height(245).Children(func() { a.dependencyPanel(c) })
		}
		ui.Row(c).Grow(1).MinHeight(0).AlignItems(ui.Stretch).Children(func() {
			ui.Column(c).Width(a.sidebarWidth()).Shrink(0).Background(panelBackground(c)).BorderWidth(0, 1, 0, 0).BorderColor(panelDivider(c)).Children(func() {
				ui.Scroll(c).Grow(1).MinHeight(0).Padding(18).Children(func() {
					if a.stage == "configure" {
						a.pageSidebar(c)
					} else {
						a.controls(c)
					}
				})
				ui.Column(c).Padding(14, 18).BorderWidth(1, 0, 0, 0).BorderColor(panelDivider(c)).Gap(10).Children(func() {
					if a.stage == "configure" {
						a.pageAction(c)
					} else {
						a.motionButtons(c)
						ui.Button(c, a.tr("New configuration", "返回配置")).FillWidth().OnClick(func() { a.stage = "configure"; a.page = "home"; a.startConfig() })
					}
				})
			})
			ui.Column(c).Grow(1).MinWidth(0).Padding(18).Gap(12).Children(func() {
				if a.page == "scene" && a.stage == "configure" {
					a.sceneEditor(c)
				} else {
					a.media(c)
				}
			})
		})
		ui.Row(c).Padding(7, 18).Gap(10).Background(panelBackground(c)).BorderWidth(1, 0, 0, 0).BorderColor(panelDivider(c)).Children(func() {
			ui.Text(c, a.sessionLabel()).TextColor(theme.TextMuted).Grow(1)
			if a.meta.State != "" {
				ui.Text(c, fmt.Sprintf(a.tr("Core · CPU %.1f%% · RAM %.1f MiB · RTF %.3f", "仿真 · CPU %.1f%% · 内存 %.1f MiB · 实时倍率 %.3f"), a.meta.Resources.CPU, a.meta.Resources.RAM, a.meta.Resources.RTF)).FontSize(12)
			} else {
				ui.Text(c, a.tr("Core · CPU — · RAM — · RTF —", "仿真 · CPU — · 内存 — · 实时倍率 —")).FontSize(12)
			}
			ui.Button(c, a.tr("Details", "详情")).OnClick(func() { a.showLog = !a.showLog })
			ui.Button(c, a.tr("Dependencies", "依赖检查")).OnClick(func() { a.showPreflight = !a.showPreflight })
		})
		if a.showLog {
			ui.Scroll(c).Height(108).Padding(8, 18).Background(panelBackground(c)).Children(func() {
				ui.Text(c, a.localizedMessage(a.status)+"\n"+strings.Join(a.manager.Logs(), "\n")).FontSize(12).TextColor(theme.TextMuted)
			})
		}
	})
}
func (a *app) stateLabel() string {
	if a.preparing || a.manager.Busy() {
		return a.tr("Preparing", "准备中")
	}
	if a.loading {
		return a.tr("Loading", "加载中")
	}
	switch a.meta.State {
	case "running":
		return a.tr("Running", "运行中")
	case "paused":
		return a.tr("Paused", "暂停")
	case "config":
		return a.tr("Configuration", "配置")
	}
	return a.tr("Disconnected", "未连接")
}
func section(c *ui.Context, title string, body func()) {
	ui.Column(c).Gap(10).Margin(0, 0, 18, 0).Children(func() { ui.Text(c, title).Bold().FontSize(15); body() })
}
func field(c *ui.Context, label string, body func()) {
	ui.Column(c).Gap(5).Children(func() { ui.Text(c, label).FontSize(13).TextColor(c.Theme().TextMuted); body() })
}

func (a *app) sessionLabel() string {
	scene := a.optionLabel(a.cfg.Terrain) + " / " + a.optionLabel(a.cfg.TerrainLevel)
	if a.cfg.Scene != nil {
		scene = a.cfg.Scene.Name
	}
	return a.optionLabel(a.cfg.Robot) + " · " + scene
}
