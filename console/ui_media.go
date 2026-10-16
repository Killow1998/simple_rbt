package main

import "github.com/egoist/mygo/ui"

func (a *app) media(c *ui.Context) {
	if a.stage == "configure" {
		ui.Row(c).Gap(8).Children(func() {
			ui.Text(c, a.tr("Sensor mounting", "安装外参")).Bold().Grow(1)
			a.localizedSelect(c.Key("tool"), &a.tool, []string{"Move / 平移", "Rotate / 旋转"}).Width(150)
			a.localizedSelect(c.Key("space"), &a.space, []string{"Base / 机身", "Local / 设备"}).Width(150)
		})
		ui.Text(c, a.tr("Drag axes or rings · orbit empty space · scroll to zoom", "拖动操作轴或旋转环；拖动空白处观察；滚轮缩放")).FontSize(12).TextColor(c.Theme().TextMuted)
	}
	if a.stage != "configure" {
		ui.Row(c).Gap(8).Children(func() {
			ui.Text(c, a.optionLabel(sourceLabel(a.meta.Source))+" · "+a.commandStateLabel()).FontSize(13).Grow(1)
			if a.localizedSelect(c.Key("view-mode"), &a.viewMode, []string{"Both / 双视角", "First / 第一视角", "Third / 第三视角"}).Width(200).Changed() {
				a.requestView()
			}
		})
	}
	ui.Column(c).Grow(1).MinHeight(0).Gap(12).Children(func() {
		showObserver := a.stage == "configure" || a.viewMode != "First / 第一视角" || !a.cameraEnabled()
		showCamera := a.stage != "configure" && a.viewMode != "Third / 第三视角" && a.cameraEnabled()
		if showObserver {
			ui.Column(c).Grow(1).MinHeight(0).Gap(6).Children(func() {
				ui.Text(c, a.tr("Third-person view", "第三视角")).FontSize(12).TextColor(c.Theme().TextMuted)
				view := ui.Box(c.Key("observer-view")).Bind(&a.driveHandles[1]).Grow(1).MinHeight(210).Radius(10).Clip().Background(viewportBackground).Focusable().Label(a.tr("3D sensor mount viewport", "传感器外参三维视图")).HandleInput(func(ev ui.InputEvent) bool {
					if a.keyboardInput(ev) {
						return true
					}
					return a.viewportInput(ev)
				})
				x, y, over := view.PointerPosition()
				a.viewport.Pointer, a.viewport.Hovering = point{float64(x), float64(y)}, over
				view.Draw(func(p *ui.Painter, r ui.Rect) {
					width, height := float64(r.W), float64(r.H)
					changed := a.viewport.Width != width || a.viewport.Height != height
					a.viewport.Width = width
					a.viewport.Height = height
					if changed {
						a.requestView()
					}
					if a.observer == nil {
						p.Text(r.X+18, r.Y+28, a.tr("Waiting for simulation view", "等待仿真画面"), 14, viewportText)
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
				title := a.tr("First-person RGB", "第一视角 RGB")
				if !a.cfg.Sensors.Camera {
					title = a.tr("Depth · 0–20 m", "深度 · 0–20 米")
				}
				if !a.cfg.Sensors.Camera && !a.cfg.Sensors.Depth {
					title = a.tr("Panorama", "全景视角")
				}
				ui.Text(c, title).FontSize(12).TextColor(c.Theme().TextMuted)
				ui.Box(c.Key("camera-view")).Bind(&a.driveHandles[2]).Focusable().HandleInput(a.keyboardInput).Grow(1).MinHeight(180).Radius(10).Background(viewportBackground).Draw(func(p *ui.Painter, r ui.Rect) {
					if a.camera != nil {
						p.Image(a.camera, r, ui.Contain)
					} else {
						p.Text(r.X+18, r.Y+28, a.tr("Waiting for sensor images", "等待传感器图像"), 14, viewportText)
					}
				})
			})
		}
	})
}
