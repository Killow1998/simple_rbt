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
	if a.cameraDeviceEnabled() {
		choices = append(choices, "camera")
	}
	if a.cfg.Sensors.ExternalCamera {
		choices = append(choices, "external_camera")
	}
	if len(choices) == 0 {
		return
	}
	if (a.selected == "external_camera" && !a.cfg.Sensors.ExternalCamera) || (a.selected == "mid360" && !a.cfg.Sensors.MID360) || (a.selected == "camera" && !a.cameraDeviceEnabled()) {
		a.selected = choices[0]
	}
	section(c, a.tr("Sensor mount", "安装外参"), func() {
		a.localizedSelect(c.Key("sensor-mount"), &a.selected, choices).FillWidth()
		pose, available := a.cfg.Mounts[a.selected]
		if !available {
			ui.Text(c, a.tr("Waiting for model defaults", "等待模型默认外参")).FontSize(12).TextColor(c.Theme().TextMuted)
			return
		}
		frame := "base_link"
		if a.selected == "external_camera" {
			frame = "world"
		} else if a.selected == "camera" && a.catalog.Robots[a.cfg.Robot].JointControl {
			frame = "arm_tool"
		}
		ui.Text(c, a.tr(frame+" · metres", frame+" · 米")).FontSize(12).TextColor(c.Theme().TextMuted)
		for i, label := range []string{"X", "Y", "Z"} {
			i, label := i, label
			ui.Row(c).Gap(10).Children(func() {
				ui.Text(c, label).Width(52).FontSize(12)
				value := pose.Position[i]
				if ui.NumberInput(c.Key("pos-"+label), &value, -5, 5, .00001).Width(232).Label(a.tr(label+" translation", label+" 轴平移")).Disabled(a.loading).Changed() {
					pose.Position[i] = value
					a.editMount(a.selected, pose)
				}
			})
		}
		ui.Text(c, a.tr("Roll · Pitch · Yaw · degrees", "横滚 · 俯仰 · 偏航 · 度")).FontSize(12).TextColor(c.Theme().TextMuted)
		euler := toEuler(pose.Quaternion)
		for i, label := range []string{"Roll", "Pitch", "Yaw"} {
			i, label := i, label
			ui.Row(c).Gap(10).Children(func() {
				ui.Text(c, a.tr(label, []string{"横滚", "俯仰", "偏航"}[i])).Width(52).FontSize(12)
				value := euler[i]
				if ui.NumberInput(c.Key("rpy-"+label), &value, -180, 180, .01).Width(232).Label(a.tr(label+" angle", []string{"横滚", "俯仰", "偏航"}[i]+"角度")).Disabled(a.loading).Changed() {
					euler[i] = value
					pose.Quaternion = fromEuler(euler)
					a.editMount(a.selected, pose)
				}
			})
		}
		if a.selected == "camera" {
			ui.Text(c, a.tr("Whole-device mount · camera_link reference", "整机安装外参 · camera_link 基准")).FontSize(12).TextColor(c.Theme().TextMuted)
			if a.cfg.CameraModel == "d435i" {
				ui.Text(c, a.tr("Internal IMU fixed to camera · independent of LiDAR", "内置 IMU 与相机固定连接 · 独立于雷达")).FontSize(12).TextColor(c.Theme().TextMuted)
			}
		}
		if a.selected == "mid360" {
			imu := internalImu(pose, a.meta.LidarIMUOffset)
			ui.Column(c).Padding(10).Radius(8).Background(c.Theme().Background).Gap(5).Children(func() {
				ui.Text(c, a.cfg.LidarModel+a.tr(" · rigid LiDAR + IMU", " · 雷达与 IMU 刚性总成")).FontSize(12).Bold()
				ui.Text(c, fmt.Sprintf(a.tr("IMU → base_link (m)\n%.5f, %.5f, %.5f", "IMU → base_link（米）\n%.5f, %.5f, %.5f"), imu[0], imu[1], imu[2])).FontSize(12).TextColor(c.Theme().TextMuted)
				ui.Text(c, a.tr("Body IMU stays independent", "机身 IMU 保持独立")).FontSize(12).TextColor(c.Theme().TextMuted)
			})
		}
	})
}
