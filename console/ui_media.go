package main

import "github.com/egoist/mygo/ui"

func (a *app) media(c *ui.Context) {
	if a.stage == "configure" {
		ui.Row(c).Gap(8).Children(func() {
			ui.Text(c, "Sensor mounting / 安装外参").Bold().Grow(1)
			ui.Select(c.Key("tool"), &a.tool, []string{"Move / 平移", "Rotate / 旋转"}).Width(150)
			ui.Select(c.Key("space"), &a.space, []string{"Base / 机身", "Local / 设备"}).Width(150)
		})
		ui.Text(c, "Drag axes or rings · orbit empty space · scroll to zoom / 拖动操作轴或旋转环，空白处观察，滚轮缩放").FontSize(11).TextColor(c.Theme().TextMuted)
	}
	if a.stage != "configure" {
		ui.Row(c).Gap(8).Children(func() {
			ui.Text(c, "Live views / 实时视角").Bold().Grow(1)
			if ui.Select(c.Key("view-mode"), &a.viewMode, []string{"Both / 双视角", "First / 第一视角", "Third / 第三视角"}).Width(200).Changed() {
				a.requestView()
			}
		})
	}
	ui.Column(c).Grow(1).MinHeight(0).Gap(12).Children(func() {
		showObserver := a.stage == "configure" || a.viewMode != "First / 第一视角" || !a.cameraEnabled()
		showCamera := a.stage != "configure" && a.viewMode != "Third / 第三视角" && a.cameraEnabled()
		if showObserver {
			ui.Column(c).Grow(1).MinHeight(0).Gap(6).Children(func() {
				ui.Text(c, "MuJoCo observer / 第三视角").FontSize(12).TextColor(c.Theme().TextMuted)
				ui.Box(c.Key("observer-view")).Grow(1).MinHeight(210).Radius(10).Background(ui.Hex("#151c29")).Focusable().Label("3D sensor mount viewport").HandleInput(a.viewportInput).Draw(func(p *ui.Painter, r ui.Rect) {
					width, height := float64(r.W), float64(r.H)
					changed := a.viewport.Width != width || a.viewport.Height != height
					a.viewport.Width = width
					a.viewport.Height = height
					if changed {
						a.requestView()
					}
					if a.observer == nil {
						p.Text(r.X+18, r.Y+28, "Waiting for MuJoCo renderer / 等待真实仿真画面", 14, ui.Hex("#b7c4dc"))
						return
					}
					p.Image(a.observer, r, ui.Contain)
					if a.stage == "configure" && !a.loading {
						a.drawGizmo(p, r)
					}
				})
			})
		}
		if showCamera {
			ui.Column(c).Grow(1).MinHeight(0).Gap(6).Children(func() {
				title := "Onboard RGB / 第一视角"
				if !a.cfg.Sensors.Camera {
					title = "Depth · 0–20 m / 深度视角"
				}
				if !a.cfg.Sensors.Camera && !a.cfg.Sensors.Depth {
					title = "Panorama / 全景视角"
				}
				ui.Text(c, title).FontSize(12).TextColor(c.Theme().TextMuted)
				ui.Box(c.Key("camera-view")).Grow(1).MinHeight(180).Radius(10).Background(ui.Hex("#151c29")).Draw(func(p *ui.Painter, r ui.Rect) {
					if a.camera != nil {
						p.Image(a.camera, r, ui.Contain)
					} else {
						p.Text(r.X+18, r.Y+28, "Waiting for sensor frames / 等待传感器图像", 14, ui.Hex("#b7c4dc"))
					}
				})
			})
		}
	})
}
