package main

import (
	"encoding/json"
	"github.com/egoist/mygo/ui"
	"image"
	"math"
)

// The scene uses one bounded image/depth buffer, not a glyph mask for every
// large polygon. Re-render only when geometry, selection or the camera changes.
type sceneRaster struct {
	key    string
	image  *image.RGBA
	depth  []float32
	bitmap *ui.Bitmap
	scale  float64
}
type rasterVertex struct{ x, y, invZ float64 }

func (a *app) drawScene(p *ui.Painter, r ui.Rect) {
	key, _ := json.Marshal([]any{a.sceneDraft, a.sceneObject, a.hiddenFloors, a.viewInfo(), r.W, r.H})
	cache := &a.scenePreview
	if cache.key != string(key) {
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
		for _, o := range a.sceneDraft.boxes() {
			if a.hiddenFloors[o.Floor] {
				continue
			}
			pose := a.sceneDraft.pose(o)
			vertices := [8]Vec3{}
			for i := 0; i < 8; i++ {
				v := Vec3{}
				for j := 0; j < 3; j++ {
					v[j] = (float64((i>>j)&1) - .5) * o.Size[j]
				}
				vertices[i] = add(pose.Position, rotate(v, pose.Quaternion))
			}
			color := ui.Hex("#8a9db3")
			switch o.Kind {
			case "floor":
				color = ui.Hex("#43546a")
				if a.sceneDraft.height(o.Floor) != 0 {
					color = ui.Hex("#8496aa")
				}
			case "ramp":
				color = ui.Hex("#74976a")
			case "obstacle":
				color = ui.Hex("#b88961")
			}
			if o.ID == a.sceneObject {
				color = ui.Hex("#bba56e")
			}
			for k, indices := range [][4]int{{0, 2, 3, 1}, {4, 5, 7, 6}, {0, 1, 5, 4}, {2, 6, 7, 3}, {0, 4, 6, 2}, {1, 3, 7, 5}} {
				poly := []Vec3{}
				for _, i := range indices {
					poly = append(poly, vertices[i])
				}
				poly = clipNear(poly, eye, forward)
				projected := []rasterVertex{}
				for _, v := range poly {
					q := a.project(v)
					projected = append(projected, rasterVertex{q.X * cache.scale, q.Y * cache.scale, 1 / dot(sub(v, eye), forward)})
				}
				shade := .65 + float64(k%3)*.15
				c := color
				c.R = uint8(float64(c.R) * shade)
				c.G = uint8(float64(c.G) * shade)
				c.B = uint8(float64(c.B) * shade)
				for i := 1; i+1 < len(projected); i++ {
					cache.triangle(projected[0], projected[i], projected[i+1], c)
				}
			}
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
	if o := a.sceneDraft.object(a.sceneObject); o != nil && o.Connection != nil && !a.hiddenFloors[o.Floor] {
		base := a.sceneDraft.pose(*o).Position
		top := a.sceneDraft.connectionEnd(*o)
		for i, v := range []Vec3{base, top} {
			q := a.project(v)
			label := a.tr("Lower end", "下端")
			if i == 1 {
				label = a.tr("Upper end", "上端")
			}
			painterCircle(p, r, q, 5, gizmoHighlight)
			gizmoLabel(p, r, point{q.X + 10, q.Y}, label, gizmoHighlight)
		}
	}
	spawn := a.sceneDraft.Spawn.Position
	spawn[2] += a.sceneDraft.height(a.sceneDraft.Spawn.Floor) + .03
	q := a.project(spawn)
	painterCircle(p, r, q, 7, gizmoHighlight)
	gizmoLabel(p, r, point{q.X + 12, q.Y}, a.tr("Spawn", "出生点"), gizmoHighlight)
}
func clipNear(poly []Vec3, eye, forward Vec3) []Vec3 {
	out := []Vec3{}
	previous := poly[len(poly)-1]
	before := dot(sub(previous, eye), forward) - .05
	for _, current := range poly {
		now := dot(sub(current, eye), forward) - .05
		if (before >= 0) != (now >= 0) {
			t := before / (before - now)
			out = append(out, add(previous, mul(sub(current, previous), t)))
		}
		if now >= 0 {
			out = append(out, current)
		}
		previous, before = current, now
	}
	return out
}
func (r *sceneRaster) triangle(a, b, c rasterVertex, color ui.Color) {
	area := (b.x-a.x)*(c.y-a.y) - (b.y-a.y)*(c.x-a.x)
	if math.Abs(area) < 1e-8 {
		return
	}
	w, h := r.image.Bounds().Dx(), r.image.Bounds().Dy()
	left := int(clamp(math.Floor(math.Min(a.x, math.Min(b.x, c.x))), 0, float64(w-1)))
	right := int(clamp(math.Ceil(math.Max(a.x, math.Max(b.x, c.x))), 0, float64(w-1)))
	top := int(clamp(math.Floor(math.Min(a.y, math.Min(b.y, c.y))), 0, float64(h-1)))
	bottom := int(clamp(math.Ceil(math.Max(a.y, math.Max(b.y, c.y))), 0, float64(h-1)))
	for y := top; y <= bottom; y++ {
		for x := left; x <= right; x++ {
			px, py := float64(x)+.5, float64(y)+.5
			u := ((b.x-px)*(c.y-py) - (b.y-py)*(c.x-px)) / area
			v := ((c.x-px)*(a.y-py) - (c.y-py)*(a.x-px)) / area
			t := 1 - u - v
			if u < 0 || v < 0 || t < 0 {
				continue
			}
			inv := float32(u*a.invZ + v*b.invZ + t*c.invZ)
			i := y*w + x
			if inv <= r.depth[i] {
				continue
			}
			r.depth[i] = inv
			offset := y*r.image.Stride + x*4
			r.image.Pix[offset], r.image.Pix[offset+1], r.image.Pix[offset+2], r.image.Pix[offset+3] = color.R, color.G, color.B, 255
		}
	}
}
