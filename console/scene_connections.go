package main

import "math"

// Connected objects use a lower-floor anchor, yaw and horizontal run/width.
// The upper floor sets the rise; it is never an independent editable value.
type FloorConnection struct {
	ToFloor string `json:"to_floor"`
	Steps   int    `json:"steps,omitempty"`
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
		if o.Connection == nil {
			boxes = append(boxes, o)
			continue
		}
		rise, run := s.connectionRise(o), o.Size[0]
		if rise <= 0 || run <= 0 {
			continue
		}
		makeBox := func(position, size Vec3, rotation Quat) {
			box := o
			box.Connection = nil
			box.Position = add(o.Position, rotate(position, o.Quaternion))
			box.Quaternion = qnormalize(qproduct(o.Quaternion, rotation))
			box.Size = size
			boxes = append(boxes, box)
		}
		if o.Kind == "stairs" {
			steps := o.Connection.Steps
			if steps < 1 || steps > 128 {
				continue
			}
			tread, height := run/float64(steps), rise/float64(steps)
			for i := 1; i <= steps; i++ {
				h := float64(i) * height
				makeBox(Vec3{(float64(i) - .5) * tread, 0, h * .5}, Vec3{tread, o.Size[1], h}, Quat{1, 0, 0, 0})
			}
		} else {
			angle := math.Atan2(rise, run)
			// Top surface meets both elevations exactly. Extra length covers the
			// tilted slab's end caps without a vertical lip at the upper landing.
			thickness := o.Size[2]
			makeBox(Vec3{run * .5, 0, rise*.5 - thickness/(2*math.Cos(angle))}, Vec3{math.Hypot(run, rise) + thickness*math.Tan(angle), o.Size[1], thickness}, qaxis(Vec3{0, 1, 0}, -angle*180/math.Pi))
		}
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
