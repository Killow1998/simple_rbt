package main

import (
	"math"
)

func (a *app) viewInfo() observerInfo {
	if a.page != "scene" {
		return a.meta.Observer
	}
	az, el := a.viewport.Azimuth*math.Pi/180, a.viewport.Elevation*math.Pi/180
	offset := mul(Vec3{math.Cos(az) * math.Cos(el), math.Sin(az) * math.Cos(el), math.Sin(el)}, a.viewport.Distance)
	return observerInfo{Position: add(a.sceneLookat, offset), Lookat: a.sceneLookat, Up: Vec3{0, 0, 1}, Fov: 45}
}
func (a *app) viewSize() [2]int {
	if a.page == "scene" {
		return [2]int{int(a.viewport.Width), int(a.viewport.Height)}
	}
	return a.observerSize
}
func (a *app) worldDirection(v Vec3) Vec3 {
	if a.page == "scene" {
		return v
	}
	return rotate(v, a.mountParent().Quaternion)
}
func (a *app) editPose() (Pose, bool) {
	if a.page == "scene" {
		if a.sceneDraft.Version != 2 && a.sceneObject == "spawn" {
			p := a.sceneDraft.Spawn.Position
			p[2] += a.sceneDraft.height(a.sceneDraft.Spawn.Floor)
			return Pose{Position: p, Quaternion: fromEuler(Vec3{0, 0, a.sceneDraft.Spawn.Yaw})}, true
		}
		if o := a.sceneDraft.object(a.sceneObject); o != nil && !a.hiddenFloors[o.Floor] && (a.sceneDraft.Version != 2 || !a.sceneDraft.hidden(o.ID, a.hiddenFloors)) {
			return a.sceneDraft.pose(*o), true
		}
		return Pose{}, false
	}
	p, ok := a.cfg.Mounts[a.selected]
	return p, ok && a.visibleMount(a.selected)
}
func (a *app) editPoseSet(p Pose) {
	if a.page != "scene" {
		a.editMount(a.selected, p)
		return
	}
	if a.sceneDraft.Version != 2 && a.sceneObject == "spawn" {
		p.Position[2] -= a.sceneDraft.height(a.sceneDraft.Spawn.Floor)
		a.sceneDraft.Spawn.Position = p.Position
		a.sceneDraft.Spawn.Yaw = toEuler(p.Quaternion)[2]
		return
	}
	if o := a.sceneDraft.object(a.sceneObject); o != nil {
		if a.sceneDraft.Version == 2 {
			if parent := a.sceneDraft.object(o.Parent); parent != nil {
				base := a.sceneDraft.pose(*parent)
				q := base.Quaternion
				q[1], q[2], q[3] = -q[1], -q[2], -q[3]
				p.Position = rotate(sub(p.Position, base.Position), q)
				p.Quaternion = quatProduct(q, p.Quaternion)
			}
			o.Pose = p
			a.sceneDraft.markManualChange(o.ID)
			return
		}
		p.Position[2] -= a.sceneDraft.height(o.Floor)
		if o.Connection != nil {
			p.Position[2] = 0
			p.Quaternion = fromEuler(Vec3{0, 0, toEuler(p.Quaternion)[2]})
		}
		o.Pose = p
	}
}
func (a *app) pickScene(p point) {
	if a.sceneDraft.Version == 2 {
		a.pickStructured(p)
		return
	}
	eye, ray := a.ray(p)
	nearest := math.Inf(1)
	id := ""
	for _, o := range a.sceneDraft.boxes() {
		if a.hiddenFloors[o.Floor] {
			continue
		}
		pose := a.sceneDraft.pose(o)
		q := pose.Quaternion
		q[1], q[2], q[3] = -q[1], -q[2], -q[3]
		origin, dir := rotate(sub(eye, pose.Position), q), rotate(ray, q)
		lo, hi := 0., math.Inf(1)
		for i := 0; i < 3; i++ {
			h := o.Size[i] * .5
			if math.Abs(dir[i]) < 1e-9 {
				if math.Abs(origin[i]) > h {
					hi = -1
				}
				continue
			}
			x, y := (-h-origin[i])/dir[i], (h-origin[i])/dir[i]
			if x > y {
				x, y = y, x
			}
			lo = math.Max(lo, x)
			hi = math.Min(hi, y)
		}
		if hi >= lo && lo < nearest {
			nearest = lo
			id = o.ID
		}
	}
	if id != "" {
		a.sceneObject = id
		a.sceneFloor = a.sceneDraft.object(id).Floor
		a.viewport.Orbit = false
	}
}
