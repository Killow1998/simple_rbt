package main

import (
	"github.com/egoist/mygo/ui"
	"math"
	"simple-rbt/console/session"
	"testing"
)

func TestKeyboardMotionAndOwnership(t *testing.T) {
	a := newApp(session.Options{StateDir: t.TempDir()}, defaultConfig())
	t.Cleanup(a.manager.Events.Close)
	a.catalog.Robots = map[string]RobotSpec{"go2": {AllowsLateral: true, AllowsTurn: true, Limits: Vec3{2, 1, 2}}}
	a.stage = "operate"
	a.meta = metadata{State: "running", Source: "gui"}
	key := func(k ui.Key, down bool) {
		kind := ui.InputKeyDown
		if !down {
			kind = ui.InputKeyUp
		}
		if !a.keyboardInput(ui.InputEvent{Kind: kind, Key: k}) {
			t.Fatal("key not consumed")
		}
	}
	key(ui.KeyW, true)
	if a.command != (Vec3{1, 0, 0}) {
		t.Fatal(a.command)
	}
	key(ui.KeyA, true)
	if math.Abs(a.command[0]-1/math.Sqrt(2)) > 1e-9 || math.Abs(a.command[1]-.5/math.Sqrt(2)) > 1e-9 {
		t.Fatal("diagonal is not normalized", a.command)
	}
	key(ui.KeyQ, true)
	if a.command[2] != 1 {
		t.Fatal("positive yaw", a.command)
	}
	key(ui.KeySpace, true)
	if a.command != (Vec3{}) || a.driveKeys[ui.KeyW] || a.driveKeys[ui.KeyA] || a.driveKeys[ui.KeyQ] {
		t.Fatal("space did not stop")
	}
	a.keyboardInput(ui.InputEvent{Kind: ui.InputKeyDown, Key: ui.KeyW, Repeat: true})
	a.keyboardInput(ui.InputEvent{Kind: ui.InputKeyDown, Key: ui.KeyW})
	if a.command != (Vec3{}) {
		t.Fatal("repeat restarted a stopped key")
	}
	key(ui.KeyW, false)
	key(ui.KeyA, false)
	key(ui.KeyQ, false)
	key(ui.KeyS, true)
	key(ui.KeyD, true)
	key(ui.KeyE, true)
	if a.command[0] >= 0 || a.command[1] >= 0 || a.command[2] != -1 {
		t.Fatal("reverse keys", a.command)
	}
	key(ui.KeyS, false)
	key(ui.KeyD, false)
	key(ui.KeyE, false)
	if a.command != (Vec3{}) {
		t.Fatal("release did not stop")
	}
	key(ui.KeyW, true)
	a.blurManual()
	if a.command != (Vec3{}) {
		t.Fatal("focus loss did not stop")
	}
	for _, owner := range []string{"program", "ros1", "ros2"} {
		a.meta.Source = owner
		a.command = Vec3{.3, 0, 0}
		if a.keyboardInput(ui.InputEvent{Kind: ui.InputKeyDown, Key: ui.KeySpace}) || a.command != (Vec3{.3, 0, 0}) {
			t.Fatal("manual keyboard affected program owner", owner)
		}
	}
}
func TestManualLimitsRespectChassis(t *testing.T) {
	a := newApp(session.Options{StateDir: t.TempDir()}, defaultConfig())
	t.Cleanup(a.manager.Events.Close)
	a.catalog.Robots = map[string]RobotSpec{"go2": {AllowsLateral: false, AllowsTurn: false, MinRadius: .6, Limits: Vec3{2, 0, 2}}}
	if a.manualVelocity(Vec3{0, 1, 1}) != (Vec3{}) {
		t.Fatal("Ackermann lateral/in-place turn allowed")
	}
	v := a.manualVelocity(Vec3{.1, 1, 1})
	if math.Abs(v[2]-math.Abs(v[0])/.6) > 1e-9 || v[1] != 0 {
		t.Fatal("Ackermann curvature exceeded", v)
	}
}
