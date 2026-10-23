package main

import (
	"fmt"
	"github.com/egoist/mygo/ui"
	"math"
	"simple-rbt/console/session"
)

type jointAxis struct {
	Name    string  `json:"name"`
	Minimum float64 `json:"minimum"`
	Maximum float64 `json:"maximum"`
	Target  float64 `json:"target"`
	Home    float64 `json:"home"`
	Linear  bool    `json:"linear"`
}

func (a *app) setArmTargets(values map[string]float64) {
	if !a.manualMode() {
		return
	}
	a.commandMu.Lock()
	defer a.commandMu.Unlock()
	command := session.JointCommand{}
	for _, axis := range a.meta.JointControls {
		target := axis.Target
		if a.armCommand != nil {
			for i, name := range a.armCommand.Name {
				if name == axis.Name {
					target = a.armCommand.Position[i]
				}
			}
		}
		if value, ok := values[axis.Name]; ok {
			target = value
		}
		command.Name = append(command.Name, axis.Name)
		command.Position = append(command.Position, target)
	}
	a.armCommand = &command
	_ = a.manager.Client.SendJoints(command)
	a.manager.Events.Event("joints.sent", map[string]any{"names": command.Name, "positions": command.Position})
}

func (a *app) armControls(c *ui.Context) {
	ui.Text(c, a.tr("Targets: degrees / gripper opening: mm", "目标：度／夹爪开口：毫米")).FontSize(12).TextColor(c.Theme().TextMuted)
	for index, axis := range a.meta.JointControls {
		index, axis := index, axis
		scale := 180 / math.Pi
		label := fmt.Sprintf("J%d", index+1)
		if axis.Linear {
			scale = 2000
			label = a.tr("Gripper", "夹爪")
		}
		value := axis.Target
		a.commandMu.Lock()
		if a.armCommand != nil {
			for i, name := range a.armCommand.Name {
				if name == axis.Name {
					value = a.armCommand.Position[i]
				}
			}
		}
		a.commandMu.Unlock()
		value *= scale
		actual := axis.Target
		for i, name := range a.jointState.Name {
			if name == axis.Name && i < len(a.jointState.Position) {
				actual = a.jointState.Position[i]
			}
		}
		ui.Row(c.Key("arm-" + axis.Name)).Gap(8).Children(func() {
			ui.Column(c).Width(58).Gap(2).Children(func() {
				ui.Text(c, label).FontSize(12)
				ui.Text(c, fmt.Sprintf("%.1f", actual*scale)).FontSize(11).TextColor(c.Theme().TextMuted)
			})
			if ui.Slider(c.Key("slider-"+axis.Name), &value, axis.Minimum*scale, axis.Maximum*scale).Grow(1).MinWidth(50).Disabled(!a.manualMode()).Changed() {
				a.setArmTargets(map[string]float64{axis.Name: value / scale})
			}
			if ui.NumberInput(c.Key("target"), &value, axis.Minimum*scale, axis.Maximum*scale, .1).Width(160).Disabled(!a.manualMode()).Changed() {
				a.setArmTargets(map[string]float64{axis.Name: value / scale})
			}
		})
	}
	ui.Button(c, a.tr("Home pose", "回到初始姿态")).FillWidth().Disabled(!a.manualMode()).OnClick(a.action("arm.home", func() {
		targets := map[string]float64{}
		for _, axis := range a.meta.JointControls {
			targets[axis.Name] = axis.Home
		}
		a.setArmTargets(targets)
	}))
	ui.Text(c, a.tr("Targets move at a bounded speed. STOP or window blur holds the current pose.", "目标按限速执行。停止或窗口失焦后保持当前位置。")).FontSize(12).TextColor(c.Theme().TextMuted)
}
