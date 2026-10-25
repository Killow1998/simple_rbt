package main

import (
	"encoding/json"
	"github.com/egoist/mygo/ui"
	"image"
	"math"
)

func (a *app) structuredTriangles(visit func(scenePart, [3]Vec3)) error {
	resolved, err := resolveScene(&a.sceneDraft)
	if err != nil {
		return err
	}
	for _, part := range resolved.Parts {
		if a.sceneDraft.hidden(part.ObjectID, a.hiddenFloors) {
			continue
		}
		for _, face := range part.Faces {
			var triangle [3]Vec3
			for i, index := range face {
				triangle[i] = add(part.Position, rotate(part.Vertices[index], part.Quaternion))
			}
			visit(part, triangle)
		}
	}
	return nil
}
func (a *app) drawStructured(p *ui.Painter, r ui.Rect) {
	key, _ := json.Marshal([]any{a.sceneDraft, a.sceneObject, a.hiddenFloors, a.viewInfo(), r.W, r.H})
	cache := &a.scenePreview
	if cache.key != string(key) {
		if _, err := resolveScene(&a.sceneDraft); err != nil {
			a.status = err.Error()
			if cache.bitmap != nil {
				p.Image(cache.bitmap, r, ui.Contain)
			}
			gizmoLabel(p, r, point{12, 24}, a.tr("Scene preview failed — see Details", "场景预览失败，请查看详情"), gizmoHighlight)
			return
		}
		cache.key = string(key)
		cache.scale = math.Min(1, 900/math.Max(float64(r.W), float64(r.H)))
		w, h := int(float64(r.W)*cache.scale), int(float64(r.H)*cache.scale)
		if w < 1 || h < 1 {
			return
		}
		if cache.image == nil || cache.image.Bounds().Dx() != w || cache.image.Bounds().Dy() != h {
			cache.image = image.NewRGBA(image.Rect(0, 0, w, h))
			cache.depth = make([]float32, w*h)
		}
		clear(cache.depth)
		for i := 0; i < len(cache.image.Pix); i += 4 {
			copy(cache.image.Pix[i:i+4], []byte{viewportBackground.R, viewportBackground.G, viewportBackground.B, 255})
		}
		forward, _, _ := a.basis()
		eye := a.viewInfo().Position
		err := a.structuredTriangles(func(part scenePart, t [3]Vec3) {
			color := ui.Hex("#8a9db3")
			if part.Semantic == "floor" {
				color = ui.Hex("#657c94")
			}
			if part.Semantic == "stairs" {
				color = ui.Hex("#b88961")
			}
			if a.sceneDraft.containsNode(a.sceneObject, part.ObjectID) {
				color = ui.Hex("#bba56e")
			}
			normal := cross(sub(t[1], t[0]), sub(t[2], t[0]))
			length := norm(normal)
			shade := .65
			if length > 1e-12 {
				shade += .3 * math.Abs(normal[2]/length)
			}
			color.R = uint8(float64(color.R) * shade)
			color.G = uint8(float64(color.G) * shade)
			color.B = uint8(float64(color.B) * shade)
			poly := clipNear(t[:], eye, forward)
			projected := []rasterVertex{}
			for _, v := range poly {
				q := a.project(v)
				projected = append(projected, rasterVertex{q.X * cache.scale, q.Y * cache.scale, 1 / dot(sub(v, eye), forward)})
			}
			for i := 1; i+1 < len(projected); i++ {
				cache.triangle(projected[0], projected[i], projected[i+1], color)
			}
		})
		if err != nil {
			a.status = err.Error()
		}
		if cache.bitmap == nil {
			cache.bitmap = ui.NewBitmap(cache.image)
		} else {
			cache.bitmap.Update(cache.image)
		}
	}
	if cache.bitmap != nil {
		p.Image(cache.bitmap, r, ui.Contain)
	}
	for _, o := range a.sceneDraft.Objects {
		if o.Semantic == "spawn" && !a.sceneDraft.hidden(o.ID, a.hiddenFloors) {
			q := a.project(a.sceneDraft.pose(o).Position)
			painterCircle(p, r, q, 7, gizmoHighlight)
			gizmoLabel(p, r, point{q.X + 12, q.Y}, o.Name, gizmoHighlight)
		}
	}
}
func (a *app) pickStructured(p point) {
	eye, ray := a.ray(p)
	nearest := math.Inf(1)
	id := ""
	_ = a.structuredTriangles(func(part scenePart, t [3]Vec3) {
		e1, e2 := sub(t[1], t[0]), sub(t[2], t[0])
		h := cross(ray, e2)
		det := dot(e1, h)
		if math.Abs(det) < 1e-10 {
			return
		}
		s := sub(eye, t[0])
		u := dot(s, h) / det
		if u < 0 || u > 1 {
			return
		}
		q := cross(s, e1)
		v := dot(ray, q) / det
		if v < 0 || u+v > 1 {
			return
		}
		distance := dot(e2, q) / det
		if distance >= 0 && distance < nearest {
			nearest = distance
			id = part.ObjectID
		}
	})
	if id != "" {
		a.sceneObject = id
		a.viewport.Orbit = false
	}
}
