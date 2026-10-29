package main

import (
	"fmt"
	"github.com/egoist/mygo/ui"
	"math"
	"simple-rbt/console/session"
)

type flightStatus struct {
	State     string     `json:"state"`
	Velocity  Vec3       `json:"velocity"`
	Mass      float64    `json:"mass_kg"`
	Thrust    [4]float64 `json:"rotor_thrust_n"`
	Saturated bool       `json:"saturated"`
}

// Call on the UI thread with commandMu held. Keyboard horizontal axes follow
// heading only, so a banked vehicle never turns W/S into a climb command.
func (a *app) updateFlightVelocity() {
	if a.flightCommand == nil || a.flightCommand.Action != "velocity" {
		return
	}
	v := a.flightLocal
	if n := math.Hypot(v[0], v[1]); n > 1 {
		v[0] /= n
		v[1] /= n
	}
	q := a.meta.Base.Quaternion
	yaw := math.Atan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]*q[2]+q[3]*q[3]))
	c, s := math.Cos(yaw), math.Sin(yaw)
	a.flightCommand.Velocity = [3]float64{(c*v[0] - s*v[1]) * a.manualScale, (s*v[0] + c*v[1]) * a.manualScale, v[2] * .6 * a.manualScale}
	a.flightCommand.YawRate = v[3] * a.manualScale
}
func (a *app) fly(action string) {
	if !a.manualMode() {
		return
	}
	clear(a.driveKeys)
	a.pad = joystick{}
	a.commandMu.Lock()
	defer a.commandMu.Unlock()
	a.flightLocal = [4]float64{}
	a.flightCommand = &session.FlightCommand{Action: action, Height: a.takeoffHeight}
	_ = a.manager.Client.SendFlight(*a.flightCommand)
}
func (a *app) sendFlightAxes(axes [4]float64) {
	a.commandMu.Lock()
	defer a.commandMu.Unlock()
	a.flightLocal = axes
	a.flightCommand = &session.FlightCommand{Action: "velocity"}
	a.updateFlightVelocity()
	_ = a.manager.Client.SendFlight(*a.flightCommand)
}
func (a *app) flightKeyboard(ev ui.InputEvent, key ui.Key) bool {
	switch key {
	case ui.KeyW, ui.KeyS, ui.KeyA, ui.KeyD, ui.KeyQ, ui.KeyE, ui.KeyR, ui.KeyF, ui.KeySpace:
	default:
		return false
	}
	if ev.Kind == ui.InputKeyDown && ev.Mods != 0 {
		a.stopManual()
		return false
	}
	if key == ui.KeySpace {
		if ev.Kind == ui.InputKeyDown {
			held := a.driveKeys
			a.driveKeys = nil
			a.stopManual()
			for key := range held {
				held[key] = false
			}
			a.driveKeys = held
		}
		return true
	}
	if a.driveKeys == nil {
		a.driveKeys = map[ui.Key]bool{}
	}
	if ev.Kind == ui.InputKeyDown {
		if _, held := a.driveKeys[key]; ev.Repeat || held {
			return true
		}
		a.driveKeys[key] = true
	} else {
		delete(a.driveKeys, key)
	}
	axis := func(positive, negative ui.Key) float64 {
		var v float64
		if a.driveKeys[positive] {
			v++
		}
		if a.driveKeys[negative] {
			v--
		}
		return v
	}
	a.pad.Held = false
	a.pad.Position = point{}
	a.sendFlightAxes([4]float64{axis(ui.KeyW, ui.KeyS), axis(ui.KeyA, ui.KeyD), axis(ui.KeyR, ui.KeyF), axis(ui.KeyQ, ui.KeyE)})
	return true
}
func (a *app) flightInput(ev ui.InputEvent) bool {
	if a.keyboardInput(ev) {
		return true
	}
	if !a.manualMode() {
		return false
	}
	a.auditInput(ev, "flight_joystick")
	switch ev.Kind {
	case ui.InputPointerDown:
		if ev.Button != 0 {
			return false
		}
		a.pad.Held = true
	case ui.InputPointerMove:
		if !a.pad.Held {
			return false
		}
	case ui.InputPointerUp:
		a.stopManual()
		return true
	default:
		return false
	}
	radius := math.Min(a.pad.Width, a.pad.Height) * .38
	v := point{(float64(ev.X) - a.pad.Width*.5) / radius, (float64(ev.Y) - a.pad.Height*.5) / radius}
	if n := plen(v); n > 1 {
		v.X /= n
		v.Y /= n
	}
	a.pad.Position = v
	clear(a.driveKeys)
	a.sendFlightAxes([4]float64{-v.Y, -v.X, 0, 0})
	return true
}
func (a *app) flightControls(c *ui.Context) {
	if a.takeoffHeight == 0 {
		a.takeoffHeight = 1
	}
	states := map[string][2]string{"grounded": {"Grounded", "已落地"}, "takeoff": {"Taking off", "起飞中"}, "hover": {"Holding position", "悬停"}, "velocity": {"Flying", "飞行中"}, "landing": {"Landing", "降落中"}, "takeoff_blocked": {"Takeoff blocked · check clearance", "起飞受阻，请检查空间"}, "fault": {"Flight stopped · reset required", "飞行故障，请重置"}}
	state := states[a.meta.Flight.State]
	if a.meta.Flight.State == "disarmed" {
		state = [2]string{"Motors off", "已停桨"}
	}
	ui.Text(c, a.tr(state[0], state[1])).Bold()
	field(c, a.tr("Takeoff height above floor · m", "距地起飞高度 · 米"), func() {
		ui.NumberInput(c, &a.takeoffHeight, .25, 5, .1).FillWidth().Disabled(!a.manualMode())
	})
	ui.Row(c).Gap(6).Children(func() {
		ui.Button(c, a.tr("Take off", "起飞")).Grow(1).Disabled(!a.manualMode()).OnClick(a.action("flight.takeoff", func() { a.fly("takeoff") }))
		ui.Button(c, a.tr("Land", "降落")).Grow(1).Disabled(!a.manualMode()).OnClick(a.action("flight.land", func() { a.fly("land") }))
		ui.Button(c, a.tr("Hover", "悬停")).Grow(1).Disabled(!a.manualMode()).OnClick(a.action("flight.hold", func() { a.fly("hold") }))
		ui.Button(c, a.tr("Cut motors", "停桨")).Grow(1).Background(stopBackground(c)).TextColor(c.Theme().Text).Disabled(!a.manualMode()).OnClick(a.action("flight.disarm", func() { a.fly("disarm") }))
	})
	ui.Text(c, a.tr("Motor cut in the air causes a fall.", "空中停桨会下坠。")).FontSize(12).TextColor(c.Theme().Warning)
	ui.Box(c.Key("flight-pad")).Bind(&a.driveHandles[0]).Height(128).Radius(12).Background(c.Theme().Background).Focusable().Disabled(!a.manualMode()).Label(a.tr("Horizontal flight joystick", "水平飞行摇杆")).HandleInput(a.flightInput).Draw(a.drawJoystick(c))
	ui.Row(c).Gap(6).Children(func() {
		buttons := []struct {
			en, zh string
			axes   [4]float64
		}{{"Up", "上升", [4]float64{0, 0, 1, 0}}, {"Down", "下降", [4]float64{0, 0, -1, 0}}, {"Turn L", "左转", [4]float64{0, 0, 0, 1}}, {"Turn R", "右转", [4]float64{0, 0, 0, -1}}}
		for i, b := range buttons {
			ui.Button(c.Key(b.en), a.tr(b.en, b.zh)).Bind(&a.flightHandles[i]).Grow(1).Disabled(!a.manualMode()).HandleInput(func(ev ui.InputEvent) bool {
				if a.keyboardInput(ev) {
					return true
				}
				if ev.Kind == ui.InputPointerDown && ev.Button == 0 {
					clear(a.driveKeys)
					a.sendFlightAxes(b.axes)
					return true
				}
				if ev.Kind == ui.InputPointerUp {
					a.stopManual()
					return true
				}
				return false
			})
		}
	})
	ui.Text(c, a.tr("W/S forward/back · A/D sideways · R/F up/down · Q/E yaw · Space hover", "W/S 前后 · A/D 横移 · R/F 升降 · Q/E 偏航 · 空格悬停")).FontSize(12).TextColor(c.Theme().TextMuted)
	ui.Text(c, fmt.Sprintf(a.tr("Manual speed · %.0f%%", "手动速度 · %.0f%%"), a.manualScale*100)).FontSize(12)
	ui.Slider(c.Key("flight-speed"), &a.manualScale, .1, 1).FillWidth().Disabled(!a.manualMode())
	ui.Text(c, fmt.Sprintf(a.tr("Height Z %.2f m · payload included %.3f kg", "世界高度 Z %.2f 米 · 含载荷质量 %.3f 千克"), a.meta.Base.Position[2], a.meta.Flight.Mass)).FontSize(12)
	ui.Text(c, a.tr("Release/blur: hold. Program control continues.", "松开或手动失焦后悬停；程序控制继续。")).FontSize(12).TextColor(c.Theme().TextMuted)
}
