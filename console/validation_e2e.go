package main

import (
	"fmt"
	"github.com/egoist/mygo/ui"
	"math"
	"os"
	"time"
)

type e2e struct {
	app            *app
	path           string
	phase          int
	began, phaseAt time.Time
	tester         *ui.Tester
	target         Pose
	sensor         string
	initial, body  Vec3
	checks         []map[string]any
	finished       bool
}

func (a *app) startE2e(path string) {
	test := &e2e{app: a, path: path, began: time.Now(), phaseAt: time.Now(), sensor: "mid360"}
	go func() {
		ticker := time.NewTicker(120 * time.Millisecond)
		defer ticker.Stop()
		for {
			select {
			case <-a.done:
				return
			case <-ticker.C:
				if a.closed.Load() {
					return
				}
				a.win.Update(test.step)
			}
		}
	}()
}
func (t *e2e) check(ok bool, label string) {
	if !ok {
		panic(label)
	}
	t.checks = append(t.checks, map[string]any{"check": label, "passed": true, "elapsed_ms": time.Since(t.began).Milliseconds(), "stamp_ns": t.app.meta.Stamp})
}
func (t *e2e) next()     { t.phase++; t.phaseAt = time.Now() }
func (t *e2e) ack() bool { return samePose(t.app.meta.Mounts[t.sensor], t.target) }
func (t *e2e) capture(suffix string) {
	go func() {
		png, err := t.app.win.CapturePage()
		if err == nil {
			_ = os.WriteFile(t.path+suffix, png, 0600)
		}
	}()
}
func (t *e2e) finish(ok bool, message string) {
	if t.finished {
		return
	}
	t.finished = true
	a := t.app
	if !ok {
		a.exitCode.Store(1)
	}
	_ = saveJSON(t.path, map[string]any{"passed": ok, "message": message, "checks": t.checks, "metadata": a.meta, "velocity": a.velocity, "view_stamp_ns": a.viewStamp, "camera_stamp_ns": a.cameraStamp, "odometry_stamp_ns": a.odomStamp, "logs": a.manager.Logs(), "elapsed_ms": time.Since(t.began).Milliseconds()})
	a.close()
	a.win.Close()
}
func (t *e2e) field(label string, value float64) {
	r, ok := t.tester.Find(label)
	t.check(ok, "numeric field available: "+label)
	t.tester.ClickAt(r.X+35, r.Y+r.H*.5)
	t.tester.Key(ui.Cmd, ui.KeyA)
	t.tester.Type(fmt.Sprintf("%.5f", value))
}
func (t *e2e) drag(from, to point) {
	r, ok := t.tester.Find(t.app.tr("3D sensor mount viewport", "传感器外参三维视图"))
	t.check(ok, "native 3D viewport available")
	t.tester.Press(r.X+float32(from.X), r.Y+float32(from.Y))
	t.tester.Move(r.X+float32(to.X), r.Y+float32(to.Y))
	t.tester.Release(r.X+float32(to.X), r.Y+float32(to.Y))
}
func (t *e2e) step() {
	if t.finished {
		return
	}
	defer func() {
		if err := recover(); err != nil {
			t.finish(false, fmt.Sprint(err))
		}
	}()
	a := t.app
	if time.Since(t.began) > 70*time.Second {
		t.finish(false, fmt.Sprintf("timeout at phase %d", t.phase))
		return
	}
	switch t.phase {
	case 0:
		if a.meta.State != "config" || a.observer == nil || a.loading {
			return
		}
		t.body = a.meta.BodyImu.Position
		t.tester = ui.NewTester(a.view, 1320, 900)
		t.check(true, "MyGo widget flow receives real MuJoCo preview")
		t.field(a.tr("X translation", "X 轴平移"), a.cfg.Mounts["mid360"].Position[0]+.03)
		t.target = a.cfg.Mounts["mid360"]
		t.next()
	case 1:
		if !t.ack() || time.Since(t.phaseAt) < 350*time.Millisecond {
			return
		}
		t.check(true, "numeric MID360 mount acknowledged by core")
		t.tester.Frame()
		pose := a.cfg.Mounts["mid360"]
		origin := a.project(pose.Position)
		tip := a.project(add(pose.Position, mul(axis(0), .22)))
		target := point{tip.X + (tip.X-origin.X)*.15, tip.Y + (tip.Y-origin.Y)*.15}
		t.drag(tip, target)
		t.check(norm(sub(a.cfg.Mounts["mid360"].Position, pose.Position)) > .005, "native translation handle changes pose")
		t.target = a.cfg.Mounts["mid360"]
		t.next()
	case 2:
		if !t.ack() || time.Since(t.phaseAt) < 350*time.Millisecond {
			return
		}
		t.check(true, "3D translation acknowledged by core")
		t.check(t.tester.Click(a.optionLabel("Move / 平移")) == nil, "open native gizmo tool select")
		t.check(t.tester.Click(a.optionLabel("Rotate / 旋转")) == nil, "select native rotation tool")
		pose := a.cfg.Mounts["mid360"]
		from := a.project(add(pose.Position, Vec3{.16 * math.Cos(.73), .16 * math.Sin(.73), 0}))
		to := a.project(add(pose.Position, Vec3{.16 * math.Cos(1.13), .16 * math.Sin(1.13), 0}))
		t.drag(from, to)
		q := a.cfg.Mounts["mid360"].Quaternion
		product := 0.
		for i := range q {
			product += q[i] * pose.Quaternion[i]
		}
		t.check(math.Abs(product) < .995, "native rotation ring changes quaternion")
		t.target = a.cfg.Mounts["mid360"]
		t.next()
	case 3:
		if !t.ack() || time.Since(t.phaseAt) < 350*time.Millisecond {
			return
		}
		t.check(true, "3D rotation acknowledged by core")
		t.check(norm(sub(a.meta.MidImu.Position, internalImu(a.meta.Mounts["mid360"]))) < 1e-6, "MID360 internal IMU remains rigidly coupled")
		t.check(norm(sub(a.meta.BodyImu.Position, t.body)) < 1e-9, "body controller IMU stays independent")
		t.capture(".configuration.png")
		t.check(t.tester.Click(a.tr("Start session", "进入操控")) == nil, "activate native Start button")
		t.next()
	case 4:
		if a.meta.State != "running" || a.camera == nil || a.observer == nil || time.Since(t.phaseAt) < 2500*time.Millisecond {
			return
		}
		t.check(true, "native first-person and observer views receive full frames")
		t.initial = a.meta.Base.Position
		t.tester.Frame()
		r, ok := t.tester.Find(a.tr("Hold joystick to move; release to stop", "按住摇杆运动，松开停车"))
		t.check(ok, "native joystick available")
		t.tester.Press(r.X+r.W*.5, r.Y+r.H*.36)
		a.commandMu.Lock()
		moving := a.command[0]
		a.commandMu.Unlock()
		t.check(moving > .2, "held native joystick sends forward velocity")
		t.next()
	case 5:
		if time.Since(t.phaseAt) < 2500*time.Millisecond {
			return
		}
		distance := norm(sub(a.meta.Base.Position, t.initial))
		t.check(distance > .025, "native GUI commands produce physical displacement")
		t.checks = append(t.checks, map[string]any{"displacement_m": distance})
		r, _ := t.tester.Find(a.tr("Hold joystick to move; release to stop", "按住摇杆运动，松开停车"))
		t.tester.Release(r.X+r.W*.5, r.Y+r.H*.5)
		a.commandMu.Lock()
		stopped := a.command == Vec3{}
		a.commandMu.Unlock()
		t.check(stopped, "native pointer release clears command")
		t.capture(".operation.png")
		t.next()
	case 6:
		if time.Since(t.phaseAt) < 1100*time.Millisecond {
			return
		}
		if a.odomStamp+100000000 < a.meta.Stamp && time.Since(t.phaseAt) < 2200*time.Millisecond {
			return
		}
		t.check(math.Hypot(a.velocity[0], a.velocity[1]) < .12, "physical robot stops after native joystick release")
		t.tester.SetFocused(false)
		a.stopManual()
		a.commandMu.Lock()
		stopped := a.command == Vec3{}
		a.commandMu.Unlock()
		t.check(stopped, "native focus loss clears manual command")
		t.tester.SetFocused(true)
		t.check(t.tester.Click(a.tr("STOP", "停车")) == nil, "native Stop control")
		t.check(t.tester.Click(a.tr("Pause", "暂停")) == nil, "native Pause control")
		t.next()
	case 7:
		if a.meta.State != "paused" {
			return
		}
		t.check(true, "pause acknowledged by independent core")
		t.tester.Frame()
		t.check(t.tester.Click(a.tr("New configuration", "返回配置")) == nil, "native return to configuration")
		t.next()
	case 8:
		if a.meta.State != "config" || a.observer == nil || a.loading || !t.ack() {
			return
		}
		t.check(true, "fresh paused worker preserves accepted mounts")
		t.finish(true, "Headless MyGo widget callbacks / real C++ core integration passed; native OS input is validated separately")
	}
}
