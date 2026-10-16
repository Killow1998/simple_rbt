package main

import (
	"fmt"
	"github.com/egoist/mygo/ui"
	"math"
	"time"
)

type joystick struct {
	Held          bool
	Position      point
	Width, Height float64
	Lateral       float64
}

func (a *app) controlInput(ev ui.InputEvent) bool {
	if !a.manualMode() {
		return false
	}
	pad := &a.pad
	switch ev.Kind {
	case ui.InputPointerDown:
		if ev.Button != 0 {
			return false
		}
		pad.Held = true
	case ui.InputPointerMove:
		if !pad.Held {
			return false
		}
	case ui.InputPointerUp:
		a.stopManual()
		return true
	default:
		return false
	}
	radius := math.Min(pad.Width, pad.Height) * .38
	value := point{(float64(ev.X) - pad.Width*.5) / radius, (float64(ev.Y) - pad.Height*.5) / radius}
	if length := plen(value); length > 1 {
		value.X /= length
		value.Y /= length
	}
	if plen(value) < .025 {
		value = point{}
	}
	pad.Position = value
	spec := a.catalog.Robots[a.cfg.Robot]
	command := Vec3{-value.Y * spec.Limits[0], pad.Lateral * spec.Limits[1], -value.X * spec.Limits[2]}
	if !spec.AllowsLateral {
		command[1] = 0
	}
	if !spec.AllowsTurn && spec.MinRadius > 0 {
		command[2] = clamp(command[2], -math.Abs(command[0])/spec.MinRadius, math.Abs(command[0])/spec.MinRadius)
	}
	a.commandMu.Lock()
	a.command = command
	_ = a.manager.Client.SendCommand([3]float64(command))
	a.commandMu.Unlock()
	return true
}
func (a *app) controls(c *ui.Context) {
	section(c, a.tr("Motion control", "运动控制"), func() {
		choices := []string{manualLabel, programLabel}
		if a.meta.Connections["ros1"] > 0 || a.meta.Source == "ros1" {
			choices = append(choices, ros1Label)
		}
		if a.meta.Connections["ros2"] > 0 || a.meta.Source == "ros2" {
			choices = append(choices, ros2Label)
		}
		field(c, a.tr("Command source", "指令来源"), func() {
			if a.localizedSelect(c.Key("source"), &a.sourceLabel, choices).FillWidth().Changed() {
				a.relinquishManual()
				_ = a.manager.Client.SendRequest(map[string]any{"op": "source", "source": sourceName(a.sourceLabel)})
			}
		})
		ui.Box(c.Key("drive-pad")).Height(224).Radius(12).Background(c.Theme().Background).Focusable().Disabled(!a.manualMode()).Label(a.tr("Hold joystick to move; release to stop", "按住摇杆运动，松开停车")).HandleInput(a.controlInput).Draw(func(p *ui.Painter, r ui.Rect) {
			a.pad.Width = float64(r.W)
			a.pad.Height = float64(r.H)
			cx, cy := r.X+r.W*.5, r.Y+r.H*.5
			radius := float32(math.Min(float64(r.W), float64(r.H)) * .38)
			color := c.Theme().Border
			p.Fill(ui.Rect{X: cx - radius, Y: cy - radius, W: radius * 2, H: radius * 2}, c.Theme().Surface, radius)
			p.Stroke(ui.Rect{X: cx - radius, Y: cy - radius, W: radius * 2, H: radius * 2}, color, radius, 2)
			p.Line(cx-radius, cy, cx+radius, cy, 1, color)
			p.Line(cx, cy-radius, cx, cy+radius, 1, color)
			knob := c.Theme().TextMuted
			if a.pad.Held {
				knob = c.Theme().Accent
			}
			p.Fill(ui.Rect{X: cx + float32(a.pad.Position.X)*radius - 19, Y: cy + float32(a.pad.Position.Y)*radius - 19, W: 38, H: 38}, knob, 19)
		})
		hint := a.tr("Select Manual to use the joystick", "选择手动控制后可使用摇杆")
		if a.manualMode() {
			hint = a.tr("Hold to drive · release to stop", "按住运动，松开停车")
		}
		ui.Text(c, hint).FontSize(12).TextColor(c.Theme().TextMuted)
		if a.catalog.Robots[a.cfg.Robot].AllowsLateral {
			ui.Text(c, a.tr("Lateral speed · hold slider", "侧向速度，按住滑块")).FontSize(12)
			slider := ui.Slider(c.Key("lateral"), &a.pad.Lateral, -1, 1).FillWidth().Disabled(!a.manualMode())
			if slider.Changed() {
				a.commandMu.Lock()
				a.command[1] = a.pad.Lateral * a.catalog.Robots[a.cfg.Robot].Limits[1]
				a.commandMu.Unlock()
			}
			if !slider.Pressed() && a.pad.Lateral != 0 {
				a.pad.Lateral = 0
				a.commandMu.Lock()
				a.command[1] = 0
				_ = a.manager.Client.SendCommand([3]float64(a.command))
				a.commandMu.Unlock()
			}
		}
		a.commandMu.Lock()
		command := a.command
		a.commandMu.Unlock()
		if !a.manualMode() {
			command = a.meta.Applied
		}
		ui.Text(c, fmt.Sprintf(a.tr("Command vx %.2f · vy %.2f · wz %.2f", "指令 vx %.2f · vy %.2f · wz %.2f"), command[0], command[1], command[2])).FontSize(12)
		if a.odomStamp != 0 {
			ui.Text(c, fmt.Sprintf(a.tr("Actual vx %.2f · vy %.2f · wz %.2f", "实际 vx %.2f · vy %.2f · wz %.2f"), a.velocity[0], a.velocity[1], a.angular[2])).FontSize(12)
		}
		ui.Text(c, a.tr("vx / vy: m/s · wz: rad/s", "vx／vy：米/秒 · wz：弧度/秒")).FontSize(12).TextColor(c.Theme().TextMuted)

	})
	a.connectionPanel(c)
}
func (a *app) motionButtons(c *ui.Context) {
	ui.Row(c).Gap(10).Children(func() {
		ui.Button(c, a.tr("STOP", "停车")).Grow(1).Background(stopBackground(c)).TextColor(c.Theme().Text).OnClick(func() {
			a.stopManual()
			a.sourceLabel = manualLabel
			_ = a.manager.Client.SendRequest(map[string]any{"op": "stop"})
		})
		label := a.tr("Pause", "暂停")
		op := "pause"
		if a.meta.State == "paused" {
			label = a.tr("Resume", "继续")
			op = "start"
		}
		ui.Button(c, label).Grow(1).OnClick(func() { a.stopManual(); _ = a.manager.Client.SendRequest(map[string]any{"op": op}) })
	})
}

func (a *app) prepareAdapters(startAfter bool) {
	if a.preparing {
		return
	}
	a.preparing = true
	selected := map[string]bool{"ros1": a.cfg.Adapters["ros1"], "ros2": a.cfg.Adapters["ros2"]}
	generation := a.generation
	go func() {
		fail := func(message string) {
			a.win.Update(func() {
				if generation != a.generation {
					return
				}
				a.preparing = false
				a.status = message
				a.showLog = true
			})
		}
		for _, version := range []string{"ros1", "ros2"} {
			if !selected[version] {
				continue
			}
			if startAfter && a.manager.AdapterReady(version) {
				continue
			}
			if err := a.manager.PrepareAdapter(version); err != nil {
				fail(err.Error())
				return
			}
			for a.manager.Busy() && !a.closed.Load() {
				time.Sleep(100 * time.Millisecond)
			}
			if a.closed.Load() {
				return
			}
			if !a.manager.AdapterReady(version) {
				fail("Adapter build failed / 适配器构建失败: " + version)
				return
			}
		}
		a.win.Update(func() {
			if generation != a.generation {
				return
			}
			a.preparing = false
			for _, version := range []string{"ros1", "ros2"} {
				if selected[version] {
					a.adaptersVerified[version] = true
				}
			}
			if startAfter {
				a.startOperating()
			}
		})
	}()
}
func (a *app) startOperating() {
	if a.stage != "configure" || a.loading || a.preparing || !a.preflight.Ready {
		return
	}
	for _, version := range []string{"ros1", "ros2"} {
		if a.cfg.Adapters[version] && !a.adaptersVerified[version] {
			a.prepareAdapters(true)
			return
		}
	}
	for _, version := range []string{"ros1", "ros2"} {
		if a.cfg.Adapters[version] && !a.adapterStarted[version] {
			if err := a.manager.StartAdapter(version); err != nil {
				a.status = err.Error()
				a.showLog = true
				return
			}
			a.adapterStarted[version] = true
		}
	}
	a.stopManual()
	a.stage = "operate"
	a.sourceLabel = manualLabel
	_ = a.manager.Client.SendRequest(map[string]any{"op": "source", "source": "gui"})
	_ = a.manager.Client.SendRequest(map[string]any{"op": "start"})
	a.requestView()
}
