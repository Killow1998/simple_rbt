package main

import (
	"github.com/egoist/mygo/ui"
	"math"
	"testing"
)

// Render production overlays through MyGo's software painter. Moving a device
// across every image edge must never paint into the letterbox or adjacent UI.
func TestGizmoClipsAllOverlays(t *testing.T) {
	for _, tool := range []string{"Move / 平移", "Rotate / 旋转"} {
		for _, sensor := range []string{"mid360", "camera"} {
			for _, scale := range []float32{1, 2} {
				a := &app{cfg: defaultConfig(), selected: sensor, tool: tool}
				a.cfg.Mounts = map[string]Pose{}
				a.cfg.Sensors.MID360, a.cfg.Sensors.Camera = true, true
				a.meta.Base.Quaternion = Quat{1, 0, 0, 0}
				a.meta.Observer = observerInfo{Position: Vec3{-2, -3, 2}, Up: Vec3{0, 0, 1}, Fov: 45}
				a.viewport = viewport{Width: 300, Height: 240}
				a.observerSize = [2]int{600, 300}
				bg := ui.Hex("#3a4554")
				view := ui.NewTester(func(c *ui.Context) {
					ui.Box(c).Fill().Background(bg).Draw(func(p *ui.Painter, r ui.Rect) {
						a.drawGizmo(p, ui.Rect{X: 20, Y: 20, W: 300, H: 240})
					})
				}, 340, 280)
				view.SetScale(scale)
				for _, position := range []point{{150, 120}, {2, 120}, {298, 120}, {150, 46}, {150, 194}} {
					eye, ray := a.ray(position)
					a.cfg.Mounts[sensor] = Pose{Position: add(eye, mul(ray, 3)), Quaternion: Quat{1, 0, 0, 0}}
					view.Frame()
					img, ink := view.Image(), 0
					for y := 0; y < img.Bounds().Dy(); y++ {
						for x := 0; x < img.Bounds().Dx(); x++ {
							c := img.RGBAAt(x, y)
							if c.R == bg.R && c.G == bg.G && c.B == bg.B {
								continue
							}
							px, py := float32(x)/scale, float32(y)/scale
							if px < 20 || px >= 320 || py < 65 || py >= 215 {
								t.Fatalf("%s %s scale %g origin %v paints outside image at %g,%g", tool, sensor, scale, position, px, py)
							}
							ink++
						}
					}
					if ink < 20 {
						t.Fatal("overlay disappeared instead of clipping", tool, sensor, position)
					}
					for _, h := range a.viewport.Handles {
						for _, p := range h.Points {
							if !a.insideView(p) {
								if _, ok := a.pickHandle(p); ok {
									t.Fatal("invisible handle is pickable", p)
								}
							}
						}
					}
				}
				a.cfg.Mounts[sensor] = Pose{Position: mul(a.meta.Observer.Position, 2), Quaternion: Quat{1, 0, 0, 0}}
				view.Frame()
				if len(a.viewport.Handles) != 0 {
					t.Fatal("device behind camera has handles")
				}
			}
		}
	}
}

func TestGizmoScreenSizeAcrossZoom(t *testing.T) {
	a := &app{tool: "Move / 平移", viewport: viewport{Width: 800, Height: 600}, observerSize: [2]int{800, 600}}
	a.meta.Base.Quaternion = Quat{1, 0, 0, 0}
	a.meta.Observer = observerInfo{Up: Vec3{0, 0, 1}, Fov: 45}
	pose := Pose{Quaternion: fromEuler(Vec3{20, 30, 40})}
	for _, space := range []string{"Base / 机身", "Local / 设备"} {
		a.space = space
		var lengths [3]float64
		for n, distance := range []float64{.45, 1.8, 20} {
			a.meta.Observer.Position = mul(normalized(Vec3{-2, -3, 2}), distance)
			handles := a.gizmoHandles(pose)
			if len(handles) != 3 {
				t.Fatal("missing usable axis", space, distance)
			}
			for _, h := range handles {
				length := plen(psub(h.Points[1], h.Points[0]))
				if n == 0 {
					lengths[h.Axis] = length
				}
				if length < 12 || length > 120 || math.Abs(length-lengths[h.Axis]) > .01 {
					t.Fatal("axis grows or collapses with zoom", space, distance, length)
				}
			}
		}
	}
}
