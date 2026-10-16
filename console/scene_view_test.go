package main

import (
	"bytes"
	"github.com/egoist/mygo/ui"
	"math"
	"testing"
)

func TestScenePreviewGeometry(t *testing.T) {
	a := &app{page: "scene", sceneDraft: newScene(true), sceneLookat: Vec3{4, 0, 1}, viewport: viewport{Azimuth: -135, Elevation: 40, Distance: 25}, hiddenFloors: map[string]bool{}}
	a.sceneFloor = a.sceneDraft.Floors[0].ID
	tt := ui.NewTester(func(c *ui.Context) {
		ui.Box(c).Fill().Background(viewportBackground).Draw(func(p *ui.Painter, r ui.Rect) {
			a.viewport.Width, a.viewport.Height = float64(r.W), float64(r.H)
			p.Clip(r, 0, func() { a.drawScene(p, r) })
		})
	}, 740, 635)
	image := tt.Image()
	changed := 0
	for y := 0; y < 635; y++ {
		for x := 0; x < 740; x++ {
			c := image.RGBAAt(x, y)
			if c.R != viewportBackground.R || c.G != viewportBackground.G || c.B != viewportBackground.B {
				changed++
			}
		}
	}
	t.Log("geometry pixels", changed, "eye", a.viewInfo(), "slab", a.project(Vec3{0, 0, 3}))
	if changed < 20000 {
		t.Fatal("scene geometry absent")
	}
}

func TestSceneEditorDragRotateAndFloorVisibility(t *testing.T) {
	a := &app{stage: "configure", page: "scene", tool: "Move / 平移", space: "World / 世界", sceneDraft: newScene(true), sceneLookat: Vec3{0, 0, 3}, viewport: viewport{Azimuth: -45, Elevation: 40, Distance: 15}, hiddenFloors: map[string]bool{}}
	o := &a.sceneDraft.Objects[1]
	a.sceneFloor, a.sceneObject = o.Floor, o.ID
	tt := ui.NewTester(func(c *ui.Context) { ui.Column(c).Fill().Children(func() { a.sceneEditor(c) }) }, 1100, 700)
	drag := func(from, to point) {
		t.Helper()
		r, ok := tt.Find("3D scene viewport")
		if !ok {
			t.Fatal("scene viewport missing")
		}
		tt.Press(r.X+float32(from.X), r.Y+float32(from.Y))
		tt.Move(r.X+float32(to.X), r.Y+float32(to.Y))
		tt.Release(r.X+float32(to.X), r.Y+float32(to.Y))
	}
	pose := a.sceneDraft.pose(*o)
	length := a.gizmoLength(pose.Position)
	drag(a.project(add(pose.Position, Vec3{length, 0, 0})), a.project(add(pose.Position, Vec3{length + 1, 0, 0})))
	if math.Abs(o.Position[0]-1) > .02 || math.Abs(o.Position[2]+.1) > 1e-8 {
		t.Fatalf("drag lost floor-relative transform: %+v", o.Pose)
	}
	a.tool = "Rotate / 旋转"
	tt.Frame()
	pose = a.sceneDraft.pose(*o)
	radius := a.gizmoLength(pose.Position) * .8
	drag(a.project(add(pose.Position, Vec3{radius * math.Cos(.73), radius * math.Sin(.73), 0})), a.project(add(pose.Position, Vec3{radius * math.Cos(1.13), radius * math.Sin(1.13), 0})))
	if math.Abs(toEuler(o.Quaternion)[2]-.4*180/math.Pi) > 1 {
		t.Fatalf("rotation ring failed: %+v", o.Quaternion)
	}
	before := append([]byte(nil), tt.Image().Pix...)
	saved := clone(a.sceneDraft)
	a.hiddenFloors[o.Floor] = true
	tt.Frame()
	if bytes.Equal(before, tt.Image().Pix) {
		t.Fatal("hiding floor did not change preview")
	}
	if !sameJSON(saved, a.sceneDraft) {
		t.Fatal("hiding floor altered physical scene")
	}
	if len(a.viewport.Handles) != 0 {
		t.Fatal("hidden object still has editable handles")
	}
}

func TestConnectionPropertiesGenerateStairsAndRamp(t *testing.T) {
	a := &app{stage: "configure", page: "scene", sceneDraft: newScene(true), hiddenFloors: map[string]bool{}}
	ramp := &a.sceneDraft.Objects[2]
	ramp.Connection = &FloorConnection{ToFloor: a.sceneDraft.Floors[1].ID, Steps: 15}
	ramp.Kind = "stairs"
	ramp.Position = Vec3{}
	ramp.Quaternion = Quat{1, 0, 0, 0}
	ramp.Size = Vec3{4.2, 1.2, .2}
	tt := ui.NewTester(func(c *ui.Context) { ui.Column(c).Fill().Children(func() { a.connectionInspector(c, ramp) }) }, 300, 1000)
	edit := func(label, text string) {
		t.Helper()
		r, ok := tt.Find(label)
		if !ok {
			t.Fatal("field absent", label)
		}
		tt.ClickAt(r.X+30, r.Y+r.H+18)
		tt.Key(ui.Cmd, ui.KeyA)
		tt.Type(text)
		tt.Key(0, ui.KeyTab)
	}
	edit("Target step height · m", "0.18")
	if ramp.Connection.Steps != 17 {
		t.Fatalf("expected 17 steps, got %d", ramp.Connection.Steps)
	}
	boxes := a.sceneDraft.boxes()
	stairCount := 0
	for _, box := range boxes {
		if box.ID == ramp.ID {
			stairCount++
			if math.Abs(box.Size[0]-.28) > 1e-6 {
				t.Fatal("tread depth changed")
			}
		}
	}
	if stairCount != 17 {
		t.Fatal("stair geometry missing")
	}
	last := boxes[2+16]
	if math.Abs(last.Position[2]+last.Size[2]*.5-3) > 1e-6 {
		t.Fatal("top step misses floor")
	}
	ramp.Kind = "ramp"
	tt.Frame()
	edit("Incline · degrees", "30")
	if math.Abs(ramp.Size[0]-3/math.Tan(math.Pi/6)) > 1e-6 {
		t.Fatal("incline/run coupling failed")
	}
	a.sceneDraft.Floors[1].Height = 4
	if err := validateScene(&a.sceneDraft); err != nil {
		t.Fatal(err)
	}
	box := a.sceneDraft.boxes()[2]
	slope := -toEuler(box.Quaternion)[1] * math.Pi / 180
	if math.Abs(math.Tan(slope)*ramp.Size[0]-4) > 1e-6 {
		t.Fatal("ramp does not follow edited floor height")
	}
}
