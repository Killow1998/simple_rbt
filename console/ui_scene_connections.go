package main

import (
	"fmt"
	"github.com/egoist/mygo/ui"
	"math"
)

func (a *app) connectionInspector(c *ui.Context, o *SceneObject) {
	connection := o.Connection
	selectFloor := func(label string, value *string, allowed func(Floor) bool) {
		names := []string{}
		ids := map[string]string{}
		selected := ""
		for i, f := range a.sceneDraft.Floors {
			if !allowed(f) {
				continue
			}
			name := fmt.Sprintf("%d · %s", i+1, f.Name)
			names = append(names, name)
			ids[name] = f.ID
			if *value == f.ID {
				selected = name
			}
		}
		field(c, label, func() {
			if ui.Select(c.Key(label), &selected, names).FillWidth().Changed() {
				*value = ids[selected]
			}
		})
	}
	selectFloor(a.tr("Lower floor", "下层"), &o.Floor, func(f Floor) bool { return f.Height < a.sceneDraft.height(connection.ToFloor) })
	selectFloor(a.tr("Upper floor", "上层"), &connection.ToFloor, func(f Floor) bool { return f.Height > a.sceneDraft.height(o.Floor) })
	rise := a.sceneDraft.connectionRise(*o)
	ui.Text(c, fmt.Sprintf(a.tr("Floor rise: %.3f m", "楼层高差：%.3f 米"), rise)).FontSize(12)
	number := func(en, zh string, value *float64, lo, hi, step float64) bool {
		label := a.tr(en, zh)
		changed := false
		field(c, label, func() { changed = ui.NumberInput(c.Key(label), value, lo, hi, step).FillWidth().Label(label).Changed() })
		return changed
	}
	number("Clear width · m", "净宽 · 米", &o.Size[1], .02, 200, .1)
	if o.Kind == "stairs" {
		steps := float64(connection.Steps)
		tread := o.Size[0] / math.Max(1, steps)
		if number("Step count", "级数", &steps, 1, 128, 1) {
			connection.Steps = int(math.Round(steps))
			o.Size[0] = float64(connection.Steps) * tread
		}
		target := rise / math.Max(1, float64(connection.Steps))
		if number("Target step height · m", "期望单级高度 · 米", &target, math.Max(.02, rise/128), math.Max(.02, rise), .01) {
			connection.Steps = int(clamp(math.Ceil(rise/target), 1, 128))
			o.Size[0] = float64(connection.Steps) * tread
		}
		if number("Tread depth · m", "踏步深度 · 米", &tread, .02, 2, .01) {
			o.Size[0] = float64(connection.Steps) * tread
		}
		ui.Text(c, fmt.Sprintf(a.tr("Actual: %d steps × %.3f m\nHorizontal run: %.3f m", "实际：%d 级 × %.3f 米\n水平长度：%.3f 米"), connection.Steps, rise/math.Max(1, float64(connection.Steps)), o.Size[0])).FontSize(12)
		ui.Text(c, a.tr("Actual step height follows the integer count and floor rise.", "按整数级数调整实际单级高度，精确接上层标高。")).FontSize(12).TextColor(c.Theme().TextMuted)
	} else {
		number("Horizontal run · m", "水平长度 · 米", &o.Size[0], .02, 200, .1)
		angle := math.Atan2(rise, o.Size[0]) * 180 / math.Pi
		if number("Incline · degrees", "坡度 · 度", &angle, .1, 80, .5) {
			o.Size[0] = clamp(rise/math.Tan(angle*math.Pi/180), .02, 200)
		}
		number("Slab thickness · m", "坡板厚度 · 米", &o.Size[2], .02, 2, .01)
		ui.Text(c, fmt.Sprintf(a.tr("Slope %.1f%% · surface length %.3f m", "坡比 %.1f%% · 坡面长度 %.3f 米"), 100*rise/o.Size[0], math.Hypot(rise, o.Size[0]))).FontSize(12)
	}
	ui.Text(c, a.tr("Lower end position · m", "下端位置 · 米")).Bold()
	number("X position", "X 位置", &o.Position[0], -100, 100, .1)
	number("Y position", "Y 位置", &o.Position[1], -100, 100, .1)
	yaw := toEuler(o.Quaternion)[2]
	if number("Heading · degrees", "朝向 · 度", &yaw, -180, 180, 1) {
		o.Quaternion = fromEuler(Vec3{0, 0, yaw})
	}
	ui.Text(c, a.tr("Runs along the local X arrow. Align both ends with floor slabs; use continuous routes for wheel chassis.", "沿局部 X 方向上升。请将两端对准楼板；轮式底盘使用连续坡道。")).FontSize(12).TextColor(c.Theme().TextMuted)
}
