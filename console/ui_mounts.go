package main

import (
	"fmt"
	"github.com/egoist/mygo/ui"
)

func (a *app) mountInspector(c *ui.Context) {
	choices := []string{}
	if a.cfg.Sensors.MID360 {
		choices = append(choices, "mid360")
	}
	if a.cameraEnabled() {
		choices = append(choices, "camera")
	}
	if len(choices) == 0 {
		return
	}
	if (a.selected == "mid360" && !a.cfg.Sensors.MID360) || (a.selected == "camera" && !a.cameraEnabled()) {
		a.selected = choices[0]
	}
	section(c, "03 · Sensor mount / 安装外参", func() {
		ui.Select(c.Key("sensor-mount"), &a.selected, choices).FillWidth()
		pose, available := a.cfg.Mounts[a.selected]
		if !available {
			ui.Text(c, "Waiting for model defaults / 等待模型默认外参").FontSize(12).TextColor(c.Theme().TextMuted)
			return
		}
		ui.Text(c, "base_link · metres / 米").FontSize(11).TextColor(c.Theme().TextMuted)
		for i, label := range []string{"X", "Y", "Z"} {
			i, label := i, label
			ui.Row(c).Gap(10).Children(func() {
				ui.Text(c, label).Width(52).FontSize(12)
				value := pose.Position[i]
				if ui.NumberInput(c.Key("pos-"+label), &value, -5, 5, .00001).Width(232).Label(label + " translation / 平移").Disabled(a.loading).Changed() {
					pose.Position[i] = value
					a.editMount(a.selected, pose)
				}
			})
		}
		ui.Text(c, "Roll · Pitch · Yaw / 欧拉角 °").FontSize(11).TextColor(c.Theme().TextMuted)
		euler := toEuler(pose.Quaternion)
		for i, label := range []string{"Roll", "Pitch", "Yaw"} {
			i, label := i, label
			ui.Row(c).Gap(10).Children(func() {
				ui.Text(c, label).Width(52).FontSize(12)
				value := euler[i]
				if ui.NumberInput(c.Key("rpy-"+label), &value, -180, 180, .01).Width(232).Label(label + " angle / 角度").Disabled(a.loading).Changed() {
					euler[i] = value
					pose.Quaternion = fromEuler(euler)
					a.editMount(a.selected, pose)
				}
			})
		}
		if a.selected == "mid360" {
			imu := internalImu(pose)
			ui.Column(c).Padding(10).Radius(8).Background(c.Theme().Background).Gap(5).Children(func() {
				ui.Text(c, "MID360 · rigid LiDAR + IMU / 刚性总成").FontSize(11).Bold()
				ui.Text(c, fmt.Sprintf("IMU → base_link: %.5f, %.5f, %.5f m", imu[0], imu[1], imu[2])).FontSize(10).TextColor(c.Theme().TextMuted)
				ui.Text(c, "Body IMU stays independent / 机身 IMU 独立").FontSize(10).TextColor(c.Theme().TextMuted)
			})
		}
	})
}
