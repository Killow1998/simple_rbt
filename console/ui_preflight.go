package main

import "github.com/egoist/mygo/ui"

func (a *app) dependencyPanel(c *ui.Context) {
	ui.Column(c).Padding(20).Gap(12).Children(func() {
		ui.Row(c).Gap(12).Children(func() {
			ui.Text(c, "Startup checks / 启动检查").Bold().FontSize(18).Grow(1)
			ui.Button(c, "Check again / 重新检查").Disabled(a.checking).OnClick(a.beginPreflight)
		})
		if a.checking {
			ui.Text(c, "Checking Ubuntu, SpRobo core and selected ROS containers… / 正在检查系统、核心及所选 ROS 容器…").FontSize(13)
		}
		ui.Text(c, "Ubuntu 22.04 amd64 · MuJoCo 3.15.0 · optional Noetic / Humble containers / ROS 容器可选").FontSize(12).TextColor(c.Theme().TextMuted)
		for _, check := range a.preflight.Checks {
			color := c.Theme().TextMuted
			status := "SKIP / 跳过"
			if check.Status == "pass" {
				status = "PASS / 通过"
				color = ui.Hex("#23845b")
			}
			if check.Status == "fail" {
				status = "OPTIONAL / 可选项未就绪"
				color = ui.Hex("#a36918")
				if check.Required {
					status = "FAIL / 失败"
					color = c.Theme().Danger
				}
			}
			ui.Column(c).Gap(4).Children(func() {
				ui.Text(c, status+" · "+check.ID).Bold().FontSize(12).TextColor(color)
				ui.Text(c, check.Message).FontSize(12)
			})
		}
		ui.Text(c, "CPU execution is supported. ROS is optional; no host ROS installation is needed. / 支持纯 CPU 运行；ROS 可选，无需在宿主安装 ROS。").FontSize(12).TextColor(c.Theme().TextMuted)
	})
}
