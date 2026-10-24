package main

import "math"

type sceneSnap struct {
	Enabled bool    `json:"enabled"`
	Grid    float64 `json:"grid_m"`
	Angle   float64 `json:"angle_deg"`
}

func quantize(value, step float64) float64 {
	if step <= 0 {
		return value
	}
	return math.Round(value/step) * step
}
func (a *app) snapSelected() {
	p, ok := a.editPose()
	if !ok {
		return
	}
	for i := range p.Position {
		p.Position[i] = quantize(p.Position[i], a.workspace.Snap.Grid)
	}
	angles := toEuler(p.Quaternion)
	for i := range angles {
		angles[i] = quantize(angles[i], a.workspace.Snap.Angle)
	}
	p.Quaternion = fromEuler(angles)
	a.editPoseSet(p)
}
func (s *Scene) bounds(id string) (Vec3, Vec3, bool) {
	low, high := Vec3{math.Inf(1), math.Inf(1), math.Inf(1)}, Vec3{math.Inf(-1), math.Inf(-1), math.Inf(-1)}
	found := false
	if s.Version == 2 {
		geometry, err := resolveScene(s)
		if err != nil {
			return low, high, false
		}
		for _, part := range geometry.Parts {
			if !s.containsNode(id, part.ObjectID) {
				continue
			}
			found = true
			for _, p := range part.Vertices {
				p = add(part.Position, rotate(p, part.Quaternion))
				for j := range p {
					low[j] = math.Min(low[j], p[j])
					high[j] = math.Max(high[j], p[j])
				}
			}
		}
		return low, high, found
	}
	for _, o := range s.boxes() {
		if o.ID != id {
			continue
		}
		found = true
		pose := s.pose(o)
		for i := 0; i < 8; i++ {
			v := Vec3{}
			for j := range v {
				v[j] = (float64((i>>j)&1) - .5) * o.Size[j]
			}
			v = add(pose.Position, rotate(v, pose.Quaternion))
			for j := range v {
				low[j] = math.Min(low[j], v[j])
				high[j] = math.Max(high[j], v[j])
			}
		}
	}
	return low, high, found
}
func (a *app) alignSceneObject(mode string) {
	o := a.sceneDraft.object(a.sceneObject)
	if o == nil || a.alignTarget == o.ID {
		return
	}
	low, high, ok := a.sceneDraft.bounds(o.ID)
	targetLow, targetHigh, other := a.sceneDraft.bounds(a.alignTarget)
	if !ok || !other {
		return
	}
	delta := Vec3{}
	switch mode {
	case "center-x":
		delta[0] = (targetLow[0] + targetHigh[0] - low[0] - high[0]) * .5
	case "center-y":
		delta[1] = (targetLow[1] + targetHigh[1] - low[1] - high[1]) * .5
	case "right":
		delta[0] = targetHigh[0] - low[0]
	case "front":
		delta[1] = targetHigh[1] - low[1]
	case "top":
		if o.Connection != nil {
			return
		}
		delta[2] = targetHigh[2] - low[2]
	}
	pose := a.sceneDraft.pose(*o)
	pose.Position = add(pose.Position, delta)
	a.editPoseSet(pose)
}
