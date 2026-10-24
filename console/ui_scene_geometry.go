package main

import (
	"encoding/json"
	"fmt"
	"github.com/egoist/mygo/ui"
	"math"
)

func (a *app) structuredGeometryInspector(c *ui.Context, o *SceneObject) {
	g := o.Geometry
	target := g
	template, instance := g["template_id"].(string)
	if instance {
		g = clone(a.sceneDraft.Templates[template])
		if g == nil {
			ui.Text(c, a.tr("Missing template", "模板缺失"))
			return
		}
		overrides, _ := o.Geometry["overrides"].(map[string]any)
		for key, value := range overrides {
			g[key] = value
		}
		ui.Text(c, a.tr("Shared template: ", "共享模板：")+template).FontSize(12)
		ui.Checkbox(c, &a.sceneTemplateEdit, a.tr("Edit all template instances", "修改所有模板实例"))
		if a.sceneTemplateEdit {
			g = a.sceneDraft.Templates[template]
			target = g
		} else {
			if overrides == nil {
				overrides = map[string]any{}
			}
			target = overrides
		}
	} else {
		ui.Button(c, a.tr("Create shared template", "创建共享模板")).FillWidth().OnClick(a.action("scene.create_template", func() {
			id := "template_" + newID()
			if a.sceneDraft.Templates == nil {
				a.sceneDraft.Templates = map[string]map[string]any{}
			}
			a.sceneDraft.Templates[id] = clone(o.Geometry)
			o.Geometry = map[string]any{"template_id": id}
		}))
	}
	changed := false
	keys := []string{"length_m", "width_m", "height_m", "radius_m", "bottom_radius_m", "top_radius_m", "cylinder_length_m", "run_m", "rise_m", "step_height_m", "tread_depth_m", "steps", "slab_thickness_m", "thickness_m"}
	labels := map[string][2]string{
		"length_m": {"Length · m", "长度 · 米"}, "width_m": {"Width · m", "宽度 · 米"}, "height_m": {"Height · m", "高度 · 米"},
		"radius_m": {"Radius · m", "半径 · 米"}, "bottom_radius_m": {"Bottom radius · m", "底部半径 · 米"}, "top_radius_m": {"Top radius · m", "顶部半径 · 米"},
		"cylinder_length_m": {"Cylinder length · m", "圆柱段长度 · 米"}, "run_m": {"Horizontal run · m", "水平长度 · 米"}, "rise_m": {"Rise · m", "总高 · 米"},
		"step_height_m": {"Step height · m", "单级高度 · 米"}, "tread_depth_m": {"Tread depth · m", "踏步深度 · 米"}, "steps": {"Step count", "级数"},
		"slab_thickness_m": {"Slab thickness · m", "斜板厚度 · 米"}, "thickness_m": {"Thickness · m", "厚度 · 米"},
	}
	for _, key := range keys {
		raw, ok := g[key]
		if !ok {
			continue
		}
		value, ok := raw.(float64)
		if !ok {
			if n, yes := raw.(int); yes {
				value = float64(n)
			} else {
				continue
			}
		}
		label := labels[key]
		ui.Text(c, a.tr(label[0], label[1])).FontSize(12)
		step, lo, hi := .01, .001, 1000.
		if key == "steps" {
			step, lo, hi = 1, 1, 4096
		}
		if key == "top_radius_m" || key == "bottom_radius_m" || key == "cylinder_length_m" {
			lo = 0
		}
		el := ui.NumberInput(c.Key(key), &value, lo, hi, step).FillWidth().Label(a.tr(label[0], label[1]))
		a.sceneFieldFocus(el, key)
		if el.Changed() {
			if key == "steps" {
				value = math.Round(value)
			}
			target[key] = value
			changed = true
		}
	}
	if g["type"] == "stair_flight" {
		h, _ := g["step_height_m"].(float64)
		d, _ := g["tread_depth_m"].(float64)
		n, _ := g["steps"].(float64)
		ui.Text(c, fmt.Sprintf(a.tr("Rise %.3f m · Run %.3f m · Slope %.2f°", "总高 %.3f 米 · 水平长 %.3f 米 · 坡度 %.2f°"), h*n, d*n, math.Atan2(h, d)*180/math.Pi)).FontSize(12)
		changed = a.stairDerivedInspector(c, h, d, n, target) || changed
	}
	if raw, ok := g["radii_m"]; ok {
		encoded, _ := json.Marshal(raw)
		var radii Vec3
		_ = json.Unmarshal(encoded, &radii)
		if a.editVector(c, a.tr("Radii · m", "半轴 · 米"), &radii, .001, 1000) {
			target["radii_m"] = radii
			changed = true
		}
	}
	if g["type"] == "planar_region" {
		changed = a.polygonInspector(c, g, target) || changed
	}
	if changed {
		a.commitSceneGeometry(target)
	}
}
func (a *app) commitSceneGeometry(target map[string]any) {
	o := a.sceneDraft.object(a.sceneObject)
	if o == nil {
		return
	}
	template, instance := o.Geometry["template_id"].(string)
	if instance && !a.sceneTemplateEdit {
		o.Geometry["overrides"] = target
	}
	if instance && a.sceneTemplateEdit {
		for i := range a.sceneDraft.Objects {
			other := &a.sceneDraft.Objects[i]
			if other.Geometry["template_id"] == template {
				a.sceneDraft.markManualChange(other.ID)
			}
		}
	} else {
		a.sceneDraft.markManualChange(o.ID)
	}
}

func (a *app) stairDerivedInspector(c *ui.Context, h, d, n float64, target map[string]any) bool {
	if n < 1 {
		return false
	}
	changed := false
	rise := h * n
	label := a.tr("Total rise · changes step height", "总高 · 调整单级高度")
	ui.Text(c, label).FontSize(12)
	el := ui.NumberInput(c.Key("derived-rise"), &rise, .001*n, 1000*n, .01).FillWidth().Label(label)
	a.sceneFieldFocus(el, "derived-rise")
	if el.Changed() {
		h = rise / n
		target["step_height_m"] = h
		changed = true
	}
	slope := math.Atan2(h, d) * 180 / math.Pi
	label = a.tr("Slope · changes tread depth", "坡度 · 调整踏步深度")
	ui.Text(c, label).FontSize(12)
	el = ui.NumberInput(c.Key("derived-slope"), &slope, .1, 89.9, .5).FillWidth().Label(label)
	a.sceneFieldFocus(el, "derived-slope")
	if el.Changed() {
		target["tread_depth_m"] = h / math.Tan(slope*math.Pi/180)
		changed = true
	}
	return changed
}
