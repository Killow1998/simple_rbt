package main

import (
	"github.com/egoist/mygo/ui"
	"math"
)

// Keyboard and joystick use the same robot limits and manual control owner.
func (a *app) manualVelocity(input Vec3) Vec3 {
	spec := a.catalog.Robots[a.cfg.Robot]
	if !spec.AllowsLateral {
		input[1] = 0
	}
	if length := math.Hypot(input[0], input[1]); length > 1 {
		input[0] /= length
		input[1] /= length
	}
	for i := range input {
		input[i] *= spec.Limits[i] * a.manualScale
	}
	if !spec.AllowsTurn && spec.MinRadius > 0 {
		limit := math.Abs(input[0]) / spec.MinRadius
		input[2] = clamp(input[2], -limit, limit)
	}
	return input
}
func (a *app) keyboardInput(ev ui.InputEvent) bool {
	a.auditInput(ev, "drive_keyboard")
	if !a.manualMode() || (ev.Kind != ui.InputKeyDown && ev.Kind != ui.InputKeyUp) {
		return false
	}
	key := ev.PhysicalKey
	if key == ui.KeyUnknown {
		key = ev.Key
	}
	switch key {
	case ui.KeyW, ui.KeyS, ui.KeyA, ui.KeyD, ui.KeyQ, ui.KeyE, ui.KeySpace:
	default:
		return false
	}
	if a.catalog.Robots[a.cfg.Robot].JointControl && key != ui.KeySpace {
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
			// GTK need not mark repeat events. Remember held physical keys
			// as suppressed until their release, so Space cannot restart them.
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
	a.pad.Held = false
	a.pad.Position = point{}
	a.pad.Lateral = 0
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
	command := a.manualVelocity(Vec3{axis(ui.KeyW, ui.KeyS), axis(ui.KeyA, ui.KeyD), axis(ui.KeyQ, ui.KeyE)})
	a.commandMu.Lock()
	a.command = command
	_ = a.manager.Client.SendCommand([3]float64(command))
	a.commandMu.Unlock()
	return true
}
func (a *app) checkDriveFocus(c *ui.Context) {
	if len(a.driveKeys) == 0 {
		return
	}
	for _, handle := range a.driveHandles {
		if handle.Focused(c) && a.manualMode() {
			return
		}
	}
	a.stopManual() // Switching to a field/button must not leave a held key moving.
}
