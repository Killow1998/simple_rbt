package main

import (
	"fmt"
	"github.com/egoist/mygo"
	"github.com/egoist/mygo/ui"
)

func odomTopicChoices(current string) []string {
	values := []string{"/odom", "/ground_truth/odom"}
	if current != values[0] && current != values[1] {
		values = append(values, current)
	}
	return values
}
func (a *app) commandStateLabel() string {
	if !a.manager.Client.Connected() {
		return a.tr("Core disconnected", "核心未连接")
	}
	switch a.meta.CommandStatus.State {
	case "receiving":
		return a.tr("Core receiving commands", "核心正在接收指令")
	case "timed_out":
		return a.tr("Command timeout · zero command", "指令超时 · 已给零指令")
	case "paused":
		return a.tr("Paused · simulation time frozen", "已暂停 · 仿真时间冻结")
	case "unowned":
		return a.tr("No command owner", "未选择控制来源")
	default:
		if a.meta.Source != "gui" && a.meta.Connections[a.meta.Source] == 0 {
			return a.tr("Waiting for program / adapter connection", "等待程序／适配器连接")
		}
		return a.tr("Waiting for commands", "等待指令")
	}
}
func (a *app) connectionPanel(c *ui.Context) {
	section(c, a.tr("Program connection", "程序接入"), func() {
		ui.Text(c, a.commandStateLabel()).FontSize(13)
		for _, role := range []string{"program", "ros1", "ros2"} {
			ui.Text(c, fmt.Sprintf("%s · %d", a.optionLabel(sourceLabel(role)), a.meta.Connections[role])).FontSize(12)
		}
		ui.Text(c, a.tr("Counts show core connections, not ROS command publishers.", "数量表示核心连接，不代表 ROS 指令发布者。")).FontSize(12).TextColor(c.Theme().TextMuted)
		topic := "/" + a.cfg.Robot + "/cmd_vel"
		ui.Text(c, topic).FontSize(12)
		ui.Button(c, a.tr("Copy command topic", "复制控制话题")).FillWidth().OnClick(func() { mygo.Clipboard.WriteText(topic) })
		ui.Button(c, a.tr("Copy IPC session path", "复制 IPC 会话路径")).FillWidth().Disabled(!a.manager.Client.Connected()).OnClick(func() { mygo.Clipboard.WriteText(a.manager.Directory()) })
		ui.Text(c, a.tr("Update commands at 20 Hz. After 0.5 s without commands, the core requests zero velocity.", "建议以 20 Hz 更新指令；超过 0.5 秒未更新，核心给出零速度指令。")).FontSize(12).TextColor(c.Theme().TextMuted)
		if a.meta.Source == "gui" {
			ui.Text(c, a.tr("Manual: losing focus clears input.", "手动控制：窗口失焦清零输入。")).FontSize(12)
		} else {
			ui.Text(c, a.tr("Program control continues without focus. STOP returns control to manual.", "程序控制不受失焦影响；停车将控制权交回手动。")).FontSize(12)
		}
	})
}
