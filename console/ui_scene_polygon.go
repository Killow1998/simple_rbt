package main

import (
	"encoding/json"
	"fmt"
	"github.com/egoist/mygo/ui"
)

func (a *app) polygonInspector(c *ui.Context, g, target map[string]any) bool {
	var outer [][2]float64
	var holes [][][2]float64
	raw, _ := json.Marshal(g["outer_xy_m"])
	_ = json.Unmarshal(raw, &outer)
	raw, _ = json.Marshal(g["holes_xy_m"])
	_ = json.Unmarshal(raw, &holes)
	rings := append([][][2]float64{outer}, holes...)
	changed := false
	for ring := range rings {
		title := a.tr("Outer boundary (CCW)", "外边界（逆时针）")
		if ring > 0 {
			title = fmt.Sprintf(a.tr("Opening %d (CW)", "洞口 %d（顺时针）"), ring)
		}
		ui.Text(c, title).Bold().Margin(12, 0, 4, 0)
		for vertex := range rings[ring] {
			for axis := 0; axis < 2; axis++ {
				label := fmt.Sprintf("%d %s", vertex+1, []string{"X", "Y"}[axis])
				value := rings[ring][vertex][axis]
				el := ui.NumberInput(c.Key(fmt.Sprintf("ring-%d-%d-%d", ring, vertex, axis)), &value, -10000, 10000, .05).FillWidth().Label(label)
				a.sceneFieldFocus(el, label)
				if el.Changed() {
					rings[ring][vertex][axis] = value
					changed = true
				}
			}
		}

		ringIndex := ring
		ui.Button(c.Key(fmt.Sprintf("add-vertex-%d", ring)), a.tr("Add boundary vertex", "添加边界顶点")).FillWidth().OnClick(a.action("scene.add_vertex", func() {
			list := rings[ringIndex]
			first, last := list[0], list[len(list)-1]
			list = append(list, [2]float64{(first[0] + last[0]) / 2, (first[1] + last[1]) / 2})
			rings[ringIndex] = list
			target["outer_xy_m"] = rings[0]
			target["holes_xy_m"] = rings[1:]
			a.commitSceneGeometry(target)
		}))
		if ring > 0 {
			ui.Button(c.Key(fmt.Sprintf("remove-hole-%d", ring)), a.tr("Remove opening", "移除洞口")).FillWidth().OnClick(a.action("scene.remove_opening", func() {
				kept := append(rings[1:ringIndex:ringIndex], rings[ringIndex+1:]...)
				target["holes_xy_m"] = kept
				a.commitSceneGeometry(target)
			}))
		}
	}

	ui.Button(c, a.tr("Add rectangular opening", "添加矩形洞口")).FillWidth().OnClick(a.action("scene.add_opening", func() {
		// A proposed opening is validated by the core before Save/Use. Vertices are
		// editable below; invalid boundaries keep the last valid preview visible.
		center := [2]float64{}
		for _, v := range outer {
			center[0] += v[0] / float64(len(outer))
			center[1] += v[1] / float64(len(outer))
		}
		holes = append(holes, [][2]float64{{center[0] - .25, center[1] - .25}, {center[0] - .25, center[1] + .25}, {center[0] + .25, center[1] + .25}, {center[0] + .25, center[1] - .25}})
		target["holes_xy_m"] = holes
		// Preserve instance overrides when this button callback executes later.
		a.commitSceneGeometry(target)
	}))
	if changed {
		target["outer_xy_m"] = rings[0]
		target["holes_xy_m"] = rings[1:]
	}
	// Boundaries remain explicit arrays, including holes; no bounding-box fill.
	return changed
}
