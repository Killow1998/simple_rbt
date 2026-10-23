package main

import (
	"github.com/egoist/mygo/ui"
	"math"
)

func (a *app) viewportInput(event ui.InputEvent) bool {
	a.auditInput(event, "viewport")
	if a.sceneShortcut(event) {
		return true
	}
	v := &a.viewport
	pointer := point{float64(event.X), float64(event.Y)}
	switch event.Kind {
	case ui.InputScroll:
		v.Distance = clamp(v.Distance*math.Exp(float64(event.DY)*.004), .45, a.maxViewDistance())
		a.requestView()
		return true
	case ui.InputPointerDown:
		if (event.Button != 0 && event.Button != 1) || !a.insideView(pointer) {
			return false
		}
		v.Previous = pointer
		v.Start = pointer
		v.Dragging = false
		v.Orbit = true
		if event.Button == 1 {
			return true
		}
		if a.stage != "configure" || (a.page != "scene" && (a.loading || a.viewLoading)) {
			return true
		}
		v.Current, v.Dragging = a.pickHandle(pointer)
		if v.Dragging {
			v.Orbit = false
			v.Initial, _ = a.editPose()
			eye, ray := a.ray(pointer)
			worldAxis := a.worldDirection(v.Current.Direction)
			origin := a.world(v.Initial.Position)
			v.Parameter = axisParameter(eye, ray, origin, worldAxis)
			v.StartVector = ringPoint(eye, ray, origin, worldAxis)
			return true
		}
		if a.page == "scene" {
			a.pickScene(pointer)
			return true
		}
		for name, pose := range a.cfg.Mounts {
			if a.visibleMount(name) && plen(psub(pointer, a.projectMount(name, pose.Position))) < 18 {
				a.selected = name
				v.Orbit = false
				return true
			}
		}
		return true
	case ui.InputPointerMove:
		if event.Button != 0 && event.Button != 1 {
			return false
		}
		if v.Dragging {
			pose := v.Initial
			eye, ray := a.ray(pointer)
			origin := a.world(pose.Position)
			worldAxis := a.worldDirection(v.Current.Direction)
			if a.tool == "Rotate / 旋转" {
				current := ringPoint(eye, ray, origin, worldAxis)
				if norm(current) < .1 || norm(v.StartVector) < .1 {
					return true
				}
				angle := math.Atan2(dot(worldAxis, cross(v.StartVector, current)), dot(v.StartVector, current)) * 180 / math.Pi
				if a.page == "scene" && a.workspace.Snap.Enabled {
					angle = quantize(angle, a.workspace.Snap.Angle)
				}
				pose.Quaternion = qnormalize(qproduct(qaxis(v.Current.Direction, angle), v.Initial.Quaternion))
			} else {
				delta := axisParameter(eye, ray, origin, worldAxis) - v.Parameter
				if a.page == "scene" && a.workspace.Snap.Enabled && a.space == "Local / 设备" {
					delta = quantize(delta, a.workspace.Snap.Grid)
				}
				pose.Position = add(pose.Position, mul(v.Current.Direction, delta))
				if a.page == "scene" && a.workspace.Snap.Enabled && a.space != "Local / 设备" {
					pose.Position[v.Current.Axis] = quantize(pose.Position[v.Current.Axis], a.workspace.Snap.Grid)
				}
				for i := range pose.Position {
					pose.Position[i] = clamp(pose.Position[i], -a.editBound(), a.editBound())
				}
			}
			a.editPoseSet(pose)
		} else if v.Orbit {
			delta := psub(pointer, v.Previous)
			if a.page == "scene" && event.Mods&ui.Shift != 0 {
				_, right, up := a.basis()
				scale := a.viewport.Distance * 2 * math.Tan(math.Pi/8) / math.Max(1, a.viewport.Height)
				a.sceneLookat = add(a.sceneLookat, add(mul(right, -delta.X*scale), mul(up, delta.Y*scale)))
			} else {
				v.Azimuth -= delta.X * .4
				v.Elevation = clamp(v.Elevation+delta.Y*.3, 3, 85)
			}
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

func (a *app) maxViewDistance() float64 {
	if a.page == "scene" {
		return 100
	}
	return 20
}
func (a *app) editBound() float64 {
	if a.page == "scene" {
		return 100
	}
	return 5
}
