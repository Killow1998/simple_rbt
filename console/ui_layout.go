package main

import (
	"fmt"
	"github.com/egoist/mygo/ui"
	"strings"
)

func (a *app) view(c *ui.Context) {
	theme := *c.Theme()
	theme.Accent = ui.Hex("#347eea")
	theme.Radius = 8
	theme.Spacing = 4
	c.SetTheme(&theme)
	ui.Column(c).Fill().Background(theme.Background).Children(func() {
		ui.Row(c).Height(66).Padding(14, 22).BorderWidth(0, 0, 1, 0).BorderColor(theme.Border).Gap(12).Children(func() {
			ui.Text(c, "SpRobo").FontSize(23).Bold()
			ui.Text(c, "Robot simulation / 机器人仿真").TextColor(theme.TextMuted).Grow(1)
			ui.Badge(c, a.stateLabel())
		})
		if !a.initialized {
			ui.Scroll(c).Grow(1).MinHeight(0).Children(func() { a.dependencyPanel(c) })
			return
		}
		if a.showPreflight {
			ui.Scroll(c).Height(245).Children(func() { a.dependencyPanel(c) })
		}
		ui.Row(c).Grow(1).MinHeight(0).AlignItems(ui.Stretch).Children(func() {
			ui.Column(c).Width(360).Shrink(0).Background(theme.Surface).BorderWidth(0, 1, 0, 0).BorderColor(theme.Border).Children(func() {
				ui.Scroll(c).Grow(1).MinHeight(0).Padding(18).Children(func() {
					if a.stage == "configure" {
						a.configuration(c)
					} else {
						a.controls(c)
					}
				})
				ui.Column(c).Padding(14, 18).BorderWidth(1, 0, 0, 0).BorderColor(theme.Border).Gap(10).Children(func() {
					if a.stage == "configure" {
						ui.PrimaryButton(c, "Start session / 进入操控").FillWidth().Disabled(a.loading || a.preparing || a.manager.Busy() || a.meta.State != "config").OnClick(a.startOperating)
					} else {
						ui.Button(c, "New configuration / 返回配置").FillWidth().OnClick(func() { a.stage = "configure"; a.startConfig() })
					}
				})
			})
			ui.Column(c).Grow(1).MinWidth(0).Padding(18).Gap(12).Children(func() { a.media(c) })
		})
		ui.Row(c).Padding(7, 18).Gap(10).BorderWidth(1, 0, 0, 0).BorderColor(theme.Border).Children(func() {
			ui.Text(c, a.cfg.Robot+" · "+a.cfg.Terrain+" / "+a.cfg.TerrainLevel).TextColor(theme.TextMuted).Grow(1)
			if a.meta.State != "" {
				ui.Text(c, fmt.Sprintf("Core / 核心 · CPU %.1f%% · RAM %.1f MiB · RTF %.3f", a.meta.Resources.CPU, a.meta.Resources.RAM, a.meta.Resources.RTF)).FontSize(12)
			} else {
				ui.Text(c, "Core / 核心 · CPU — · RAM — · RTF —").FontSize(12)
			}
			ui.Button(c, "Details / 详情").OnClick(func() { a.showLog = !a.showLog })
			ui.Button(c, "Dependencies / 依赖").OnClick(func() { a.showPreflight = !a.showPreflight })
		})
		if a.showLog {
			ui.Scroll(c).Height(108).Padding(8, 18).Background(theme.Surface).Children(func() {
				ui.Text(c, a.status+"\n"+strings.Join(a.manager.Logs(), "\n")).FontSize(11).TextColor(theme.TextMuted)
			})
		}
	})
}
func (a *app) stateLabel() string {
	if a.preparing || a.manager.Busy() {
		return "Preparing / 准备中"
	}
	if a.loading {
		return "Loading / 加载中"
	}
	switch a.meta.State {
	case "running":
		return "Running / 运行中"
	case "paused":
		return "Paused / 暂停"
	case "config":
		return "Configuration / 配置"
	}
	return "Disconnected / 未连接"
}
func section(c *ui.Context, title string, body func()) {
	ui.Column(c).Gap(10).Margin(0, 0, 16, 0).Children(func() { ui.Text(c, title).Bold().FontSize(14); body() })
}
func field(c *ui.Context, label string, body func()) {
	ui.Column(c).Gap(5).Children(func() { ui.Text(c, label).FontSize(12).TextColor(c.Theme().TextMuted); body() })
}
