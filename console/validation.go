package main

import (
	"fmt"
	"github.com/egoist/mygo/ui"
	"math"
)

func smokeTest(a *app) error {
	if err := validateConfig(a.cfg, a.catalog); err != nil {
		return err
	}
	if len(a.catalog.Robots) < 7 {
		return fmt.Errorf("core registry is incomplete")
	}
	pose := Pose{Position: Vec3{.2, 0, .1275}, Quaternion: fromEuler(Vec3{0, 0, 90})}
	imu := internalImu(pose)
	if math.Abs(imu[0]-.17671) > 1e-7 || math.Abs(imu[1]-.011) > 1e-7 || math.Abs(imu[2]-.08338) > 1e-7 {
		return fmt.Errorf("MID360 rigid transform composition failed")
	}
	angles := Vec3{15, -20, 37}
	if norm(sub(toEuler(fromEuler(angles)), angles)) > 1e-6 {
		return fmt.Errorf("quaternion/Euler round-trip failed")
	}
	a.meta = metadata{State: "running", Base: Pose{Quaternion: Quat{1, 0, 0, 0}}, Observer: observerInfo{Position: Vec3{1.2, 1.4, 1.1}, Lookat: Vec3{0, 0, .15}, Up: Vec3{0, 0, 1}, Fov: 45, Width: 640, Height: 480}}
	a.stage = "operate"
	a.meta.Source = "gui"
	a.viewport.Width = 640
	a.viewport.Height = 480
	a.observerSize = [2]int{640, 480}
	p := a.project(Vec3{0, 0, .1275})
	if p.X < 0 || p.X > 640 || p.Y < 0 || p.Y > 480 {
		return fmt.Errorf("3D viewport projection failed")
	}
	a.pad.Width = 300
	a.pad.Height = 220
	a.controlInput(ui.InputEvent{Kind: ui.InputPointerDown, Button: 0, X: 150, Y: 50})
	a.commandMu.Lock()
	moving := a.command[0]
	a.commandMu.Unlock()
	if moving <= 0 {
		return fmt.Errorf("joystick velocity failed")
	}
	a.controlInput(ui.InputEvent{Kind: ui.InputPointerUp, Button: 0})
	a.commandMu.Lock()
	stopped := a.command == Vec3{}
	a.commandMu.Unlock()
	if !stopped {
		return fmt.Errorf("joystick release did not clear motion")
	}
	for robot, spec := range a.catalog.Robots {
		if spec.Continuous {
			previous := a.cfg.Robot
			a.cfg.Robot = robot
			for _, terrain := range a.terrains() {
				if terrain == "stairs" || terrain == "rough" {
					return fmt.Errorf("wheel-only robot received stepped terrain")
				}
			}
			a.cfg.Robot = previous
		}
	}
	return nil
}
