package main

import (
	"github.com/egoist/mygo/ui"
	"math"
)

var axisColors = []ui.Color{ui.Hex("#f27474"), ui.Hex("#4fd99d"), ui.Hex("#6fa2ff")}
var gizmoOutline = ui.Hex("#18212d")
var gizmoHighlight = ui.Hex("#ffe19a")

func validPoint(q point) bool {
	return !math.IsNaN(q.X) && !math.IsNaN(q.Y) && !math.IsInf(q.X, 0) && !math.IsInf(q.Y, 0)
}
func painterLine(p *ui.Painter, r ui.Rect, a, b point, width float32, color ui.Color) {
	painterPolyline(p, r, []point{a, b}, width, color)
}
func painterPolyline(p *ui.Painter, r ui.Rect, points []point, width float32, color ui.Color) {
	// Painter.Line only supports horizontal/vertical lines. A path preserves
	// diagonals and draws each ring with continuous joins, not overlapping caps.
	var path ui.Path
	started := false
	for _, q := range points {
		if !validPoint(q) {
			started = false
			continue
		}
		x, y := r.X+float32(q.X), r.Y+float32(q.Y)
		if started {
			path.LineTo(x, y)
		} else {
			path.MoveTo(x, y)
			started = true
		}
	}
	p.StrokePath(&path, width, color)
}

func painterCircle(p *ui.Painter, r ui.Rect, q point, radius float32, color ui.Color) {
	if !validPoint(q) {
		return
	}
	p.Stroke(ui.Rect{X: r.X + float32(q.X) - radius, Y: r.Y + float32(q.Y) - radius, W: radius * 2, H: radius * 2}, color, radius, 2)
}
func painterArrow(p *ui.Painter, r ui.Rect, origin, tip point, color ui.Color) {
	d := psub(tip, origin)
	length := plen(d)
	if length < 12 || !validPoint(tip) || !validPoint(origin) {
		return
	}
	ux, uy := d.X/length, d.Y/length
	head := math.Min(15, length*.3)
	base := point{tip.X - ux*head, tip.Y - uy*head}
	painterLine(p, r, origin, base, 6, gizmoOutline)
	painterLine(p, r, origin, base, 3, color)
	var path ui.Path
	path.MoveTo(r.X+float32(tip.X), r.Y+float32(tip.Y)).
		LineTo(r.X+float32(base.X-uy*5), r.Y+float32(base.Y+ux*5)).
		LineTo(r.X+float32(base.X+uy*5), r.Y+float32(base.Y-ux*5)).Close()
	p.StrokePath(&path, 2, gizmoOutline)
	p.FillPath(&path, color)
}
func gizmoLabel(p *ui.Painter, r ui.Rect, q point, label string, color ui.Color) {
	if !validPoint(q) {
		return
	}
	x, y := r.X+float32(q.X), r.Y+float32(q.Y)
	p.Fill(ui.Rect{X: x - 4, Y: y - 2, W: float32(len(label))*7 + 8, H: 18}, gizmoOutline, 4)
	p.Text(x, y, label, 12, color)
}
func (a *app) visibleMount(name string) bool {
	return name == "mid360" && a.cfg.Sensors.MID360 || name == "camera" && a.cameraDeviceEnabled()
}
func (a *app) drawGizmo(p *ui.Painter, r ui.Rect) {
	a.viewport.Handles = nil
	bounds := a.viewport.imageRect(a.viewSize())
	// Clip all overlays, including labels and the camera frustum, to the actual
	// image, not the surrounding letterbox or the rest of the console.
	p.Clip(ui.Rect{X: r.X + bounds.X, Y: r.Y + bounds.Y, W: bounds.W, H: bounds.H}, 0, func() {
		if a.page != "scene" {
			a.drawMounts(p, r)
		}
		pose, ok := a.editPose()
		if !ok {
			return
		}
		a.viewport.Handles = a.gizmoHandles(pose)
		active := -1
		if a.viewport.Dragging {
			active = a.viewport.Current.Axis
		} else if a.viewport.Hovering {
			if h, ok := a.pickHandle(a.viewport.Pointer); ok {
				active = h.Axis
			}
		}
		for _, h := range a.viewport.Handles {
			color := axisColors[h.Axis]
			if h.Axis == active {
				color = gizmoHighlight
			}
			if a.tool == "Rotate / 旋转" {
				painterPolyline(p, r, h.Points, 5, gizmoOutline)
				painterPolyline(p, r, h.Points, 2.5, color)
			} else {
				painterArrow(p, r, h.Points[0], h.Points[1], color)
				tip, d := h.Points[1], psub(h.Points[1], h.Points[0])
				length := plen(d)
				gizmoLabel(p, r, point{tip.X + d.X/length*17 - 4, tip.Y + d.Y/length*17 - 7}, string("XYZ"[h.Axis]), color)
			}
		}
		origin := a.project(pose.Position)
		if validPoint(origin) {
			p.Fill(ui.Rect{X: r.X + float32(origin.X) - 5, Y: r.Y + float32(origin.Y) - 5, W: 10, H: 10}, gizmoOutline, 5)
			p.Fill(ui.Rect{X: r.X + float32(origin.X) - 3, Y: r.Y + float32(origin.Y) - 3, W: 6, H: 6}, gizmoHighlight, 3)
		}
	})
}
func (a *app) drawMounts(p *ui.Painter, r ui.Rect) {
	for _, name := range []string{"mid360", "camera"} {
		pose, ok := a.cfg.Mounts[name]
		if !ok || !a.visibleMount(name) {
			continue
		}
		q, color := a.project(pose.Position), ui.Hex("#e3e7ed")
		if name == a.selected {
			color = gizmoHighlight
		} else {
			painterCircle(p, r, q, 6, color)
		}
		gizmoLabel(p, r, point{q.X + 14, q.Y + 14}, name, color)
		if name != a.selected {
			continue
		}
		if name == "mid360" {
			painterLine(p, r, q, a.project(internalImu(pose)), 1, ui.Hex("#79d4ef"))
			painterCircle(p, r, a.project(internalImu(pose)), 3, ui.Hex("#79d4ef"))
		} else {
			corners := []point{}
			for _, y := range []float64{-.23, .23} {
				for _, z := range []float64{-.1732, .1732} {
					corner := a.project(add(pose.Position, rotate(Vec3{.3, y, z}, pose.Quaternion)))
					corners = append(corners, corner)
					painterLine(p, r, q, corner, 1, ui.Hex("#ddc99a"))
				}
			}
			for _, pair := range [][2]int{{0, 1}, {1, 3}, {3, 2}, {2, 0}} {
				painterLine(p, r, corners[pair[0]], corners[pair[1]], 1, ui.Hex("#ddc99a"))
			}
		}
	}
}
