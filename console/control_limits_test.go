package main

import (
	"math"
	"simple-rbt/console/session"
	"testing"
)

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
