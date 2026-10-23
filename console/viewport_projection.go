package main

import (
	"github.com/egoist/mygo/ui"
	"math"
)

type point struct{ X, Y float64 }

func psub(a, b point) point { return point{a.X - b.X, a.Y - b.Y} }
func plen(a point) float64  { return math.Hypot(a.X, a.Y) }
func segmentDistance(p, a, b point) float64 {
	d := psub(b, a)
	n := d.X*d.X + d.Y*d.Y
	t := 0.
	if n > 0 {
		t = clamp(((p.X-a.X)*d.X+(p.Y-a.Y)*d.Y)/n, 0, 1)
	}
	return math.Hypot(p.X-a.X-t*d.X, p.Y-a.Y-t*d.Y)
}

type viewport struct {
	Width, Height                float64
	Azimuth, Elevation, Distance float64
	Handles                      []handle
	Pointer                      point
	Hovering                     bool
	Dragging                     bool
	Orbit                        bool
	Current                      handle
	Initial                      Pose
	Start, Previous              point
	Parameter                    float64
	StartVector                  Vec3
}
type handle struct {
	Axis      int
	Direction Vec3
	Points    []point
}

func (v *viewport) request(base Vec3) map[string]any {
	return map[string]any{"op": "view", "enabled": true, "azimuth_deg": v.Azimuth, "elevation_deg": v.Elevation, "distance": v.Distance, "lookat": base, "width": int(clamp(v.Width, 320, 1280)), "height": int(clamp(v.Height, 240, 960))}
}
func (v *viewport) imageRect(size [2]int) ui.Rect {
	w, h := v.Width, v.Height
	if size[0] <= 0 || size[1] <= 0 {
		return ui.Rect{W: float32(w), H: float32(h)}
	}
	scale := math.Min(w/float64(size[0]), h/float64(size[1]))
	iw, ih := float64(size[0])*scale, float64(size[1])*scale
	return ui.Rect{X: float32((w - iw) * .5), Y: float32((h - ih) * .5), W: float32(iw), H: float32(ih)}
}
func (a *app) basis() (Vec3, Vec3, Vec3) {
	o := a.viewInfo()
	forward := normalized(sub(o.Lookat, o.Position))
	right := normalized(cross(forward, o.Up))
	up := cross(right, forward)
	return forward, right, up
}
func (a *app) mountParent() Pose { return a.mountParentFor(a.selected) }
func (a *app) mountParentFor(name string) Pose {
	if a.page == "scene" {
		return Pose{Quaternion: Quat{1, 0, 0, 0}}
	}
	if parent, ok := a.meta.MountParents[name]; ok {
		return parent
	}
	return a.meta.Base
}
func (a *app) world(p Vec3) Vec3 {
	if a.page == "scene" {
		return p
	}
	return add(a.mountParent().Position, rotate(p, a.mountParent().Quaternion))
}
func (a *app) project(p Vec3) point { return a.projectWorld(a.world(p)) }
func (a *app) projectMount(name string, p Vec3) point {
	parent := a.mountParentFor(name)
	return a.projectWorld(add(parent.Position, rotate(p, parent.Quaternion)))
}
func (a *app) projectWorld(p Vec3) point {
	f, right, up := a.basis()
	rel := sub(p, a.viewInfo().Position)
	depth := dot(rel, f)
	if depth <= 1e-6 {
		return point{math.NaN(), math.NaN()}
	}
	r := a.viewport.imageRect(a.viewSize())
	fov := a.viewInfo().Fov
	if fov < 1 {
		fov = 45
	}
	focal := float64(r.H) * .5 / math.Tan(fov*math.Pi/360)
	return point{float64(r.X+r.W*.5) + focal*dot(rel, right)/depth, float64(r.Y+r.H*.5) - focal*dot(rel, up)/depth}
}
func (a *app) ray(p point) (Vec3, Vec3) {
	f, right, up := a.basis()
	r := a.viewport.imageRect(a.viewSize())
	fov := a.viewInfo().Fov
	if fov < 1 {
		fov = 45
	}
	focal := float64(r.H) * .5 / math.Tan(fov*math.Pi/360)
	x := (p.X - float64(r.X+r.W*.5)) / focal
	y := (float64(r.Y+r.H*.5) - p.Y) / focal
	return a.viewInfo().Position, normalized(add(f, add(mul(right, x), mul(up, y))))
}
func axisParameter(eye, ray, origin, direction Vec3) float64 {
	delta := sub(eye, origin)
	b := dot(direction, ray)
	den := 1 - b*b
	if den < 1e-5 {
		return 0
	}
	return (dot(direction, delta) - b*dot(ray, delta)) / den
}
func ringPoint(eye, ray, origin, normal Vec3) Vec3 {
	d := dot(normal, ray)
	if math.Abs(d) < 1e-5 {
		return Vec3{}
	}
	t := dot(normal, sub(origin, eye)) / d
	return normalized(sub(add(eye, mul(ray, t)), origin))
}
