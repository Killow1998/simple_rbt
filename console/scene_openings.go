package main

import "math"

type SlabOpening struct {
	Center [2]float64 `json:"center"`
	Size   [2]float64 `json:"size"`
}

func (o SceneObject) openingValid() bool {
	h := o.Opening
	if h == nil {
		return true
	}
	if o.Kind != "floor" || o.Connection != nil {
		return false
	}
	for i := 0; i < 2; i++ {
		if math.IsNaN(h.Center[i]) || math.IsInf(h.Center[i], 0) || math.IsNaN(h.Size[i]) || math.IsInf(h.Size[i], 0) || h.Size[i] < .02 || math.Abs(h.Center[i])+h.Size[i]/2 > o.Size[i]/2+1e-8 {
			return false
		}
	}
	return true
}
func (o SceneObject) openingBoxes() []SceneObject {
	if !o.openingValid() {
		return []SceneObject{o}
	}
	h := o.Opening
	x0, x1 := -o.Size[0]/2, o.Size[0]/2
	y0, y1 := -o.Size[1]/2, o.Size[1]/2
	left, right := h.Center[0]-h.Size[0]/2, h.Center[0]+h.Size[0]/2
	bottom, top := h.Center[1]-h.Size[1]/2, h.Center[1]+h.Size[1]/2
	boxes := []SceneObject{}
	for _, rect := range [][4]float64{{x0, left, y0, y1}, {right, x1, y0, y1}, {left, right, y0, bottom}, {left, right, top, y1}} {
		if rect[1]-rect[0] < 1e-8 || rect[3]-rect[2] < 1e-8 {
			continue
		}
		part := o
		part.Opening = nil
		part.Size = Vec3{rect[1] - rect[0], rect[3] - rect[2], o.Size[2]}
		part.Position = add(o.Position, rotate(Vec3{(rect[0] + rect[1]) / 2, (rect[2] + rect[3]) / 2, 0}, o.Quaternion))
		boxes = append(boxes, part)
	}
	return boxes
}
