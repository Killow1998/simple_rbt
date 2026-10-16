package main

import "math"

// Connected objects use a lower-floor anchor, yaw and horizontal run/width.
// The upper floor sets the rise; it is never an independent editable value.
type FloorConnection struct {
	ToFloor string  `json:"to_floor"`
	Steps   int     `json:"steps,omitempty"`
	Turn    int     `json:"turn_deg,omitempty"`
	Landing float64 `json:"landing_m,omitempty"`
}

func (s *Scene) connectionRise(o SceneObject) float64 {
	if o.Connection == nil {
		return 0
	}
	return s.height(o.Connection.ToFloor) - s.height(o.Floor)
}
func (s *Scene) boxes() []SceneObject {
	boxes := make([]SceneObject, 0, len(s.Objects))
	for _, o := range s.Objects {
		if o.Opening != nil {
			boxes = append(boxes, o.openingBoxes()...)
			continue
		}
		if o.Connection == nil {
			boxes = append(boxes, o)
			continue
		}
		boxes = append(boxes, s.flightBoxes(o)...)
	}
	return boxes
}
func (s *Scene) connectionValid(o SceneObject, floors map[string]bool) bool {
	c := o.Connection
	if c == nil {
		return o.Kind != "stairs"
	}
	if (o.Kind != "ramp" && o.Kind != "stairs") || !floors[c.ToFloor] || s.connectionRise(o) < .02 || math.Abs(o.Position[2]) > 1e-8 || math.Abs(o.Quaternion[1])+math.Abs(o.Quaternion[2]) > 1e-8 {
		return false
	}
	if c.Turn != 0 {
		if c.Turn != 90 && c.Turn != -90 && c.Turn != 180 {
			return false
		}
		if math.IsNaN(c.Landing) || math.IsInf(c.Landing, 0) || c.Landing < o.Size[1] || c.Landing > 200 || (o.Kind == "stairs" && c.Steps < 2) {
			return false
		}
	}
	if o.Kind == "stairs" {
		return c.Steps >= 1 && c.Steps <= 128 && o.Size[0]/float64(c.Steps) >= .02 && s.connectionRise(o)/float64(c.Steps) >= .02
	}
	return math.Atan2(s.connectionRise(o), o.Size[0]) <= 80*math.Pi/180
}
func (a *app) connectFloors(o *SceneObject) bool {
	base := a.sceneDraft.height(o.Floor)
	top := ""
	height := math.Inf(1)
	for _, f := range a.sceneDraft.Floors {
		if f.Height > base && f.Height < height {
			top, height = f.ID, f.Height
		}
	}
	if top == "" {
		a.status = a.tr("Add a higher floor before placing stairs or a connecting ramp.", "请先添加更高的楼层，再放置楼梯或连接坡道。")
		a.showLog = true
		return false
	}
	o.Connection = &FloorConnection{ToFloor: top}
	o.Position = Vec3{}
	o.Quaternion = Quat{1, 0, 0, 0}
	o.Size = Vec3{(height - base) * 4, 2, .2}
	if o.Kind == "stairs" {
		o.Connection.Steps = int(math.Ceil((height - base) / .18))
		o.Connection.Steps = int(clamp(float64(o.Connection.Steps), 1, 128))
		o.Size[0] = float64(o.Connection.Steps) * .28
	}
	return true
}
