package main

import "math"

// Like Godot's Node3DEditorViewport::update_transform_gizmo_view, derive world
// scale from camera depth and pixel size; shrink in a short viewport.
// https://github.com/godotengine/godot/blob/4.5/editor/scene/3d/node_3d_editor_plugin.cpp
func (a *app) gizmoLength(origin Vec3) float64 {
	f, _, _ := a.basis()
	depth := dot(sub(a.world(origin), a.viewInfo().Position), f)
	r := a.viewport.imageRect(a.viewSize())
	if depth <= 1e-6 || r.H <= 0 {
		return 0
	}
	fov := a.viewInfo().Fov
	if fov < 1 {
		fov = 45
	}
	focal := float64(r.H) * .5 / math.Tan(fov*math.Pi/360)
	pixels := 96 * math.Min(1, float64(r.H)/400)
	return math.Min(pixels*depth/focal, depth*.4)
}
func (a *app) gizmoHandles(pose Pose) []handle {
	length := a.gizmoLength(pose.Position)
	if length <= 0 {
		return nil
	}
	handles := make([]handle, 0, 3)
	for i := 0; i < 3; i++ {
		if a.page == "scene" {
			o := a.sceneDraft.object(a.sceneObject)
			if o != nil && o.Connection != nil && ((a.tool == "Rotate / 旋转" && i != 2) || (a.tool != "Rotate / 旋转" && i == 2)) {
				continue
			}
		}
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
				h.Points = append(h.Points, a.project(add(pose.Position, mul(add(mul(u, math.Cos(angle)), mul(v, math.Sin(angle))), length*.8))))
			}
		} else {
			h.Points = []point{a.project(pose.Position), a.project(add(pose.Position, mul(direction, length)))}
			// An axis looking directly into the camera has no usable screen
			// direction. Do not let its collapsed handle steal centre clicks.
			if plen(psub(h.Points[1], h.Points[0])) < 12 {
				continue
			}
		}
		handles = append(handles, h)
	}
	return handles
}
func (a *app) insideView(p point) bool {
	r := a.viewport.imageRect(a.viewSize())
	return validPoint(p) && p.X >= float64(r.X) && p.Y >= float64(r.Y) && p.X < float64(r.X+r.W) && p.Y < float64(r.Y+r.H)
}
func (a *app) pickHandle(p point) (handle, bool) {
	if !a.insideView(p) {
		return handle{}, false
	}
	best, found := 9., false
	var picked handle
	for _, h := range a.viewport.Handles {
		for j := 1; j < len(h.Points); j++ {
			start, end := h.Points[j-1], h.Points[j]
			if !validPoint(start) || !validPoint(end) {
				continue
			}
			if a.tool != "Rotate / 旋转" {
				// Reserve the shared centre for selecting the device.
				d := psub(end, start)
				length := plen(d)
				if length < 12 || plen(psub(p, start)) < 10 {
					continue
				}
				start = point{start.X + d.X/length*10, start.Y + d.Y/length*10}
			}
			if distance := segmentDistance(p, start, end); distance < best {
				best, picked, found = distance, h, true
			}
		}
	}
	return picked, found
}
