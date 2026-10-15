package main

import "github.com/egoist/mygo/ui"

func (a *app) dependencyPanel(c *ui.Context) {
	ui.Column(c).Padding(20).Gap(12).Children(func() {
		ui.Row(c).Gap(12).Children(func() {
			ui.Text(c, a.tr("Startup checks", "启动检查")).Bold().FontSize(18).Grow(1)
			ui.Button(c, a.tr("Check again", "重新检查")).Disabled(a.checking).OnClick(a.beginPreflight)
		})
		if a.checking {
			ui.Text(c, a.tr("Checking the system, simulator and selected ROS adapters…", "正在检查系统、仿真器和所选 ROS 适配器…")).FontSize(13)
		}
		ui.Text(c, a.tr("Ubuntu 22.04 amd64 · ROS adapters are optional", "Ubuntu 22.04 amd64 · ROS 适配器可选")).FontSize(12).TextColor(c.Theme().TextMuted)
		for _, check := range a.preflight.Checks {
			color := c.Theme().TextMuted
			status := a.tr("SKIP", "跳过")
			if check.Status == "pass" {
				status = a.tr("PASS", "通过")
				color = c.Theme().Success
			}
			if check.Status == "fail" {
				status = a.tr("OPTIONAL · NOT READY", "可选项未就绪")
				color = c.Theme().Warning
				if check.Required {
					status = a.tr("FAIL", "失败")
					color = c.Theme().Danger
				}
			}
			ui.Column(c).Padding(12, 14).Radius(7).Background(panelBackground(c)).Gap(6).Children(func() {
				ui.Row(c).Gap(14).Children(func() {
					ui.Text(c, a.dependencyLabel(check.ID)).Bold().FontSize(14).Grow(1)
					ui.Text(c, status).FontSize(12).TextColor(color)
				})
				ui.Text(c, a.localizedMessage(check.Message)).FontSize(13)
			})
		}
		ui.Text(c, a.tr("CPU operation is available. ROS adapters are optional.", "支持 CPU 运行，ROS 适配器可选。")).FontSize(12).TextColor(c.Theme().TextMuted)
	})
}

func (a *app) dependencyLabel(id string) string {
	switch id {
	case "ubuntu":
		return a.tr("Operating system", "操作系统")
	case "core":
		return a.tr("Simulator", "仿真器")
	case "assets":
		return a.tr("Models & controllers", "模型与控制器")
	}
	return a.optionLabel(id)
}
