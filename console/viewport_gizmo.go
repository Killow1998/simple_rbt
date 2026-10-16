package main

import (
	"github.com/egoist/mygo/ui"
	"math"
)

var axisColors = []ui.Color{ui.Hex("#f27474"), ui.Hex("#4fd99d"), ui.Hex("#6fa2ff")}

func painterLine(p *ui.Painter, r ui.Rect, a, b point, width float32, color ui.Color) {
	p.Line(r.X+float32(a.X), r.Y+float32(a.Y), r.X+float32(b.X), r.Y+float32(b.Y), width, color)
}
func painterCircle(p *ui.Painter, r ui.Rect, q point, radius float32, color ui.Color) {
	p.Stroke(ui.Rect{X: r.X + float32(q.X) - radius, Y: r.Y + float32(q.Y) - radius, W: radius * 2, H: radius * 2}, color, radius, 2)
}
func (a *app) visibleMount(name string) bool {
	return name == "mid360" && a.cfg.Sensors.MID360 || name == "camera" && a.cameraDeviceEnabled()
}
func (a *app) drawGizmo(p *ui.Painter, r ui.Rect) {
	a.viewport.Handles = nil
	for name, pose := range a.cfg.Mounts {
		if !a.visibleMount(name) {
			continue
		}
		q := a.project(pose.Position)
		color := ui.Hex("#ffffff")
		if name == a.selected {
			color = ui.Hex("#ffd17a")
		}
		painterCircle(p, r, q, 8, color)
		p.Text(r.X+float32(q.X)+12, r.Y+float32(q.Y)-14, name, 12, color)
	}
	pose, ok := a.cfg.Mounts[a.selected]
	if !ok || !a.visibleMount(a.selected) {
		return
	}
	if a.selected == "mid360" {
		painterLine(p, r, a.project(pose.Position), a.project(internalImu(pose)), 2, ui.Hex("#79d4ef"))
		painterCircle(p, r, a.project(internalImu(pose)), 3, ui.Hex("#79d4ef"))
	} else {
		corners := []point{}
		for _, y := range []float64{-.23, .23} {
			for _, z := range []float64{-.1732, .1732} {
				q := a.project(add(pose.Position, rotate(Vec3{.3, y, z}, pose.Quaternion)))
				corners = append(corners, q)
				painterLine(p, r, a.project(pose.Position), q, 1, ui.Hex("#ddc99a"))
			}
		}
		for _, pair := range [][2]int{{0, 1}, {1, 3}, {3, 2}, {2, 0}} {
			painterLine(p, r, corners[pair[0]], corners[pair[1]], 1, ui.Hex("#ddc99a"))
		}
	}
	for i := 0; i < 3; i++ {
		direction := axis(i)
		if a.space == "Local / 设备" {
			direction = rotate(direction, pose.Quaternion)
		}
		h := handle{Axis: i, Direction: direction}
		if a.tool == "Rotate / 旋转" {
			u, v := axis((i+1)%3), axis((i+2)%3)
			if a.space == "Local / 设备" {
				u, v = rotate(u, pose.Quaternion), rotate(v, pose.Quaternion)
			}
			for j := 0; j <= 72; j++ {
				angle := float64(j) * 2 * math.Pi / 72
				h.Points = append(h.Points, a.project(add(pose.Position, mul(add(mul(u, math.Cos(angle)), mul(v, math.Sin(angle))), .16))))
			}
			for j := 1; j < len(h.Points); j++ {
				painterLine(p, r, h.Points[j-1], h.Points[j], 2.5, axisColors[i])
			}
		} else {
			h.Points = []point{a.project(pose.Position), a.project(add(pose.Position, mul(direction, .22)))}
			painterLine(p, r, h.Points[0], h.Points[1], 3, axisColors[i])
			painterCircle(p, r, h.Points[1], 4, axisColors[i])
			p.Text(r.X+float32(h.Points[1].X)+8, r.Y+float32(h.Points[1].Y)-7, string("XYZ"[i]), 12, axisColors[i])
		}
		a.viewport.Handles = append(a.viewport.Handles, h)
	}
}
func (a *app) viewportInput(event ui.InputEvent) bool {
	v := &a.viewport
	pointer := point{float64(event.X), float64(event.Y)}
	switch event.Kind {
	case ui.InputScroll:
		v.Distance = clamp(v.Distance*math.Exp(float64(event.DY)*.004), .45, 20)
		a.requestView()
		return true
	case ui.InputPointerDown:
		if event.Button != 0 {
			return false
		}
		v.Previous = pointer
		v.Start = pointer
		v.Dragging = false
		v.Orbit = true
		if a.stage != "configure" || a.loading {
			return true
		}
		best := 10.
		for _, h := range v.Handles {
			for j := 1; j < len(h.Points); j++ {
				if d := segmentDistance(pointer, h.Points[j-1], h.Points[j]); d < best {
					best = d
					v.Current = h
					v.Dragging = true
				}
			}
		}
		if v.Dragging {
			v.Orbit = false
			v.Initial = a.cfg.Mounts[a.selected]
			eye, ray := a.ray(pointer)
			worldAxis := rotate(v.Current.Direction, a.meta.Base.Quaternion)
			origin := a.world(v.Initial.Position)
			v.Parameter = axisParameter(eye, ray, origin, worldAxis)
			v.StartVector = ringPoint(eye, ray, origin, worldAxis)
			return true
		}
		for name, pose := range a.cfg.Mounts {
			if a.visibleMount(name) && plen(psub(pointer, a.project(pose.Position))) < 18 {
				a.selected = name
				v.Orbit = false
				return true
			}
		}
		return true
	case ui.InputPointerMove:
		if event.Button != 0 {
			return false
		}
		if v.Dragging {
			pose := v.Initial
			eye, ray := a.ray(pointer)
			origin := a.world(pose.Position)
			worldAxis := rotate(v.Current.Direction, a.meta.Base.Quaternion)
			if a.tool == "Rotate / 旋转" {
				current := ringPoint(eye, ray, origin, worldAxis)
				if norm(current) < .1 || norm(v.StartVector) < .1 {
					return true
				}
				angle := math.Atan2(dot(worldAxis, cross(v.StartVector, current)), dot(v.StartVector, current)) * 180 / math.Pi
				pose.Quaternion = qnormalize(qproduct(qaxis(v.Current.Direction, angle), v.Initial.Quaternion))
			} else {
				pose.Position = add(pose.Position, mul(v.Current.Direction, axisParameter(eye, ray, origin, worldAxis)-v.Parameter))
				for i := range pose.Position {
					pose.Position[i] = clamp(pose.Position[i], -5, 5)
				}
			}
			a.editMount(a.selected, pose)
		} else if v.Orbit {
			delta := psub(pointer, v.Previous)
			v.Azimuth -= delta.X * .4
			v.Elevation = clamp(v.Elevation+delta.Y*.3, 3, 85)
			a.requestView()
		}
		v.Previous = pointer
		return true
	case ui.InputPointerUp:
		v.Dragging = false
		v.Orbit = false
		return true
	}
	return false
}
