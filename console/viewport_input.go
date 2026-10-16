package main

import (
	"github.com/egoist/mygo/ui"
	"math"
)

func (a *app) viewportInput(event ui.InputEvent) bool {
	v := &a.viewport
	pointer := point{float64(event.X), float64(event.Y)}
	switch event.Kind {
	case ui.InputScroll:
		v.Distance = clamp(v.Distance*math.Exp(float64(event.DY)*.004), .45, 20)
		a.requestView()
		return true
	case ui.InputPointerDown:
		if event.Button != 0 || !a.insideView(pointer) {
			return false
		}
		v.Previous = pointer
		v.Start = pointer
		v.Dragging = false
		v.Orbit = true
		if a.stage != "configure" || a.loading {
			return true
		}
		v.Current, v.Dragging = a.pickHandle(pointer)
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
