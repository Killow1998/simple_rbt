package main

import (
	"github.com/egoist/mygo/ui"
	"math"
	"testing"
)

func TestSceneUndoRedoAndTextEdits(t *testing.T) {
	a := &app{stage: "configure", page: "scene", sceneDraft: newScene(true), hiddenFloors: map[string]bool{}}
	a.sceneFloor = a.sceneDraft.Floors[0].ID
	a.sceneObject = a.sceneDraft.Objects[2].ID
	a.workspace.Snap = sceneSnap{Grid: .1, Angle: 15}
	tt := ui.NewTester(func(c *ui.Context) {
		a.beginSceneFrame()
		defer a.endSceneFrame()
		ui.Column(c).Fill().Children(func() { a.sceneActions(c); a.sceneInspector(c) })
	}, 1100, 1400)
	original := clone(a.sceneDraft)
	if err := tt.Click("Delete object"); err != nil {
		t.Fatal(err)
	}
	if len(a.sceneDraft.Objects) != len(original.Objects)-1 {
		t.Fatal("delete did not remove object")
	}
	if err := tt.Click("Undo"); err != nil {
		t.Fatal(err)
	}
	if !sameJSON(a.sceneDraft, original) || a.sceneDraft.object(a.sceneObject) == nil {
		t.Fatal("undo did not restore object and selection")
	}
	tt.Click("Redo")
	if a.sceneDraft.object(original.Objects[2].ID) != nil {
		t.Fatal("redo did not delete")
	}
	tt.Click("Undo")
	r, _ := tt.Find("Object name")
	tt.ClickAt(r.X+25, r.Y+r.H/2)
	tt.Key(ui.Cmd, ui.KeyA)
	tt.Type("edited flight")
	tt.Click("Undo")
	if a.sceneDraft.object(a.sceneObject).Name != original.Objects[2].Name {
		t.Fatalf("text edit was not one undo operation: name=%q history=%d group=%q", a.sceneDraft.object(a.sceneObject).Name, len(a.sceneHistory.undo), a.sceneHistory.group)
	}
	tt.Click("Redo")
	if a.sceneDraft.object(a.sceneObject).Name != "edited flight" {
		t.Fatal("text redo failed")
	}
	// A new edit after undo discards redo, while the saved resource is separate.
	tt.Click("Undo")
	tt.Click("Delete object")
	if len(a.sceneHistory.redo) != 0 {
		t.Fatal("redo retained an abandoned branch")
	}
}
func TestSceneSnapAlignmentAndOpenings(t *testing.T) {
	a := &app{page: "scene", sceneDraft: newScene(true)}
	a.sceneObject = a.sceneDraft.Objects[1].ID
	a.sceneFloor = a.sceneDraft.Floors[1].ID
	a.workspace.Snap = sceneSnap{Grid: .1, Angle: 15}
	o := a.sceneDraft.object(a.sceneObject)
	o.Position[0] = .14
	o.Quaternion = fromEuler(Vec3{0, 0, 23})
	a.snapSelected()
	if math.Abs(o.Position[0]-.1) > 1e-6 || math.Abs(toEuler(o.Quaternion)[2]-30) > 1e-6 {
		t.Fatal("grid/angle snap failed")
	}
	o.Opening = &SlabOpening{Center: [2]float64{1, 0}, Size: [2]float64{2, 2}}
	if len(o.openingBoxes()) != 4 || validateScene(&a.sceneDraft) != nil {
		t.Fatal("opening not represented by four solid rims")
	}
	a.alignTarget = a.sceneDraft.Objects[0].ID
	a.alignSceneObject("right")
	low, _, _ := a.sceneDraft.bounds(o.ID)
	_, target, _ := a.sceneDraft.bounds(a.alignTarget)
	if math.Abs(low[0]-target[0]) > 1e-6 {
		t.Fatal("rotated slab edge alignment failed")
	}
	o.Opening.Size[0] = 30
	if validateScene(&a.sceneDraft) == nil {
		t.Fatal("oversized opening accepted")
	}
}
func TestTurningFlightsEndAtUpperFloor(t *testing.T) {
	s := newScene(true)
	o := s.Objects[2]
	o.Kind = "stairs"
	o.Size = Vec3{4.2, 1.2, .2}
	o.Position = Vec3{}
	o.Quaternion = Quat{1, 0, 0, 0}
	o.Connection = &FloorConnection{ToFloor: s.Floors[1].ID, Steps: 15, Landing: 1.2}
	for _, turn := range []int{-90, 90, 180} {
		o.Connection.Turn = turn
		s.Objects[2] = o
		if err := validateScene(&s); err != nil {
			t.Fatal(err)
		}
		boxes := s.flightBoxes(o)
		if len(boxes) != 16 {
			t.Fatal("flight/landing box count wrong")
		}
		endpoint := s.connectionEnd(o)
		last := s.pose(boxes[len(boxes)-1])
		centre := add(last.Position, rotate(Vec3{boxes[len(boxes)-1].Size[0] / 2, 0, boxes[len(boxes)-1].Size[2] / 2}, last.Quaternion))
		if norm(sub(endpoint, centre)) > 1e-6 || math.Abs(endpoint[2]-3) > 1e-6 {
			t.Fatal("turned flight misses upper elevation", turn, endpoint, centre)
		}
	}
}

func TestSceneSnappedGestureUndo(t *testing.T) {
	a := &app{stage: "configure", page: "scene", tool: "Move / 平移", space: "World / 世界", sceneDraft: newScene(true), sceneLookat: Vec3{0, 0, 3}, viewport: viewport{Azimuth: -45, Elevation: 40, Distance: 15}, hiddenFloors: map[string]bool{}}
	a.sceneObject = a.sceneDraft.Objects[1].ID
	a.sceneFloor = a.sceneDraft.Objects[1].Floor
	a.workspace.Snap = sceneSnap{Enabled: true, Grid: .5, Angle: 15}
	tt := ui.NewTester(func(c *ui.Context) {
		a.beginSceneFrame()
		defer a.endSceneFrame()
		ui.Column(c).Fill().Children(func() { a.sceneEditor(c) })
	}, 1100, 700)
	original := clone(a.sceneDraft)
	pose := a.sceneDraft.pose(*a.sceneDraft.object(a.sceneObject))
	length := a.gizmoLength(pose.Position)
	r, _ := tt.Find("3D scene viewport")
	start := a.project(add(pose.Position, Vec3{length, 0, 0}))
	tt.Press(r.X+float32(start.X), r.Y+float32(start.Y))
	var end point
	for _, delta := range []float64{.3, .7, 1.18} {
		end = a.project(add(pose.Position, Vec3{length + delta, 0, 0}))
		tt.Move(r.X+float32(end.X), r.Y+float32(end.Y))
	}
	tt.Release(r.X+float32(end.X), r.Y+float32(end.Y))
	if math.Abs(a.sceneDraft.object(a.sceneObject).Position[0]-1) > 1e-6 {
		t.Fatal("drag did not snap to half-metre grid")
	}
	tt.Key(ui.Cmd, ui.KeyZ)
	if !sameJSON(original, a.sceneDraft) {
		t.Fatal("one undo did not restore the whole gesture")
	}
	tt.Key(ui.Cmd, ui.KeyY)
	if math.Abs(a.sceneDraft.object(a.sceneObject).Position[0]-1) > 1e-6 {
		t.Fatal("gesture redo failed")
	}
	tt.Key(0, ui.KeyDelete)
	if a.sceneDraft.object(original.Objects[1].ID) != nil {
		t.Fatal("viewport delete shortcut failed")
	}
}

func TestSceneOpeningControls(t *testing.T) {
	a := &app{stage: "configure", page: "scene", sceneDraft: newScene(true)}
	a.sceneObject = a.sceneDraft.Objects[1].ID
	a.sceneFloor = a.sceneDraft.Objects[1].Floor
	tt := ui.NewTester(func(c *ui.Context) {
		a.beginSceneFrame()
		defer a.endSceneFrame()
		ui.Column(c).Children(func() { a.sceneActions(c); a.openingInspector(c, a.sceneDraft.object(a.sceneObject)) })
	}, 1100, 600)
	if err := tt.Click("Cut rectangular opening"); err != nil {
		t.Fatal(err)
	}
	if a.sceneDraft.object(a.sceneObject).Opening == nil {
		t.Fatal("checkbox did not cut opening")
	}
	tt.Click("Undo")
	if a.sceneDraft.object(a.sceneObject).Opening != nil {
		t.Fatal("undo did not fill opening")
	}
	tt.Click("Redo")
	if a.sceneDraft.object(a.sceneObject).Opening == nil {
		t.Fatal("redo did not restore opening")
	}
}
