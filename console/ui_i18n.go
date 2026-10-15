package main

import (
	"strings"
	"unicode"

	"github.com/egoist/mygo/ui"
)

func (a *app) tr(english, chinese string) string {
	if a.preferences.Language == "zh-CN" {
		return chinese
	}
	return english
}

// Session reports and legacy UI state retain their original values. Translate
// only when displaying them; a language change never changes a wire value.
func (a *app) localizedMessage(message string) string {
	english, chinese, bilingual := strings.Cut(message, " / ")
	if bilingual && strings.ContainsFunc(chinese, func(r rune) bool { return unicode.Is(unicode.Han, r) }) {
		return a.tr(english, chinese)
	}
	return message
}

var optionLabels = map[string][2]string{
	"go2":                  {"Go2", "Go2"},
	"go2w":                 {"Go2W", "Go2W"},
	"turtlebot3_waffle_pi": {"TurtleBot3 Waffle Pi", "TurtleBot3 Waffle Pi"},
	"scout_mini":           {"Scout Mini", "Scout Mini"},
	"ranger_mini":          {"Ranger Mini", "Ranger Mini"},
	"fr_mini":              {"FR Mini", "FR Mini"},
	"ridgeback":            {"Ridgeback", "Ridgeback"},
	"flat":                 {"Flat", "平地"},
	"ramp":                 {"Ramp", "坡道"},
	"hills":                {"Hills", "连续起伏"},
	"stairs":               {"Stairs", "楼梯"},
	"rough":                {"Rough ground", "崎岖地形"},
	"standard":             {"Standard", "标准"},
	"moderate":             {"Moderate", "中等"},
	"hard":                 {"Hard", "高难"},
	"extreme":              {"Extreme", "极限"},
	"tracked":              {"Tracked", "运动跟踪"},
	"native":               {"Native", "原生动力学"},
	"auto":                 {"Automatic", "自动"},
	"cuda":                 {"CUDA", "CUDA"},
	"cpu":                  {"CPU", "CPU"},
	"robot_lab":            {"Robot Lab", "Robot Lab"},
	"rl_sar":               {"RL SAR", "RL SAR"},
	"himloco":              {"HiMLoco", "HiMLoco"},
	"legacy":               {"Legacy", "旧版"},
	"diff_drive":           {"Differential drive", "两轮差速"},
	"skid_steer":           {"Skid steer", "四轮差速"},
	"ackermann":            {"Ackermann", "阿克曼"},
	"mecanum":              {"Mecanum", "麦克纳姆轮"},
	"swerve":               {"Swerve drive", "舵轮"},
	"none":                 {"None", "无"},
	"mid360":               {"MID360 · LiDAR + IMU", "MID360 · 雷达与 IMU"},
	"camera":               {"Camera", "相机"},
	"ros1":                 {"ROS 1 · Noetic", "ROS 1 · Noetic"},
	"ros2":                 {"ROS 2 · Humble", "ROS 2 · Humble"},
}

func (a *app) optionLabel(value string) string {
	if labels, ok := optionLabels[value]; ok {
		return a.tr(labels[0], labels[1])
	}
	return a.localizedMessage(value)
}

// SelectBase binds canonical values directly, while its children supply the
// translated labels. Keyboard selection and Changed remain MyGo's own behavior.
func (a *app) localizedSelect(c *ui.Context, selected *string, options []string) ui.Element {
	t := c.Theme()
	sel := ui.SelectBase(c, selected)
	b := sel.Trigger.Padding(t.Space(1.5), t.Space(3.5)).Gap(t.Space(1.5)).Radius(t.Radius).
		Background(t.Surface).TextColor(t.Text).Border(1, t.Border).
		Justify(ui.SpaceBetween).MinWidth(t.Space(35)).Value(a.optionLabel(*selected))
	if !b.IsDisabled() {
		if b.Pressed() {
			b.Background(t.SurfacePressed)
		} else if b.Hovered() {
			b.Background(t.SurfaceHover)
		}
	}
	b.Children(func() {
		ui.Text(c, a.optionLabel(*selected)).SingleLine()
		ui.Text(c, "▾").TextColor(t.TextMuted)
	})
	sel.Popup(func(panel ui.Element) {
		panel.Margin(t.Space(1), 0, 0, 0).Padding(t.Space(1)).Radius(t.Radius+2).
			Background(t.Background).Border(1, t.Border).
			Shadow(0, 6, 20, 0, ui.RGBA(0, 0, 0, .18))
		for _, value := range options {
			item := sel.Item(value).Padding(t.Space(1.5), t.Space(2.5)).Radius(t.Radius)
			if item.Highlighted() {
				item.Background(t.Accent).TextColor(t.AccentText)
			} else if value == *selected {
				item.Background(t.Surface)
			}
			item.Children(func() { ui.Text(c, a.optionLabel(value)).SingleLine() })
		}
	})
	return b
}
