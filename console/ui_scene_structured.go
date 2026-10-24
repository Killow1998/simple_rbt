package main

import (
	"fmt"
	"github.com/egoist/mygo/ui"
)

func (a *app) structuredSidebar(c *ui.Context) {
	c = c.Key(a.sceneRevision)
	ui.Text(c, a.tr("Structured scene", "结构化场景")).Bold()
	a.sceneFieldFocus(ui.TextInput(c.Key("scene-name"), &a.sceneDraft.Name).FillWidth().Label(a.tr("Scene name", "场景名称")), "scene-name")
	if a.sceneDirty() {
		ui.Text(c, a.tr("Unsaved changes", "未保存的修改")).TextColor(c.Theme().Warning)
	}
	ui.Row(c).Gap(6).Children(func() {
		ui.Button(c, a.tr("Save scene", "保存场景")).OnClick(a.action("ui_scene.save_scene", func() { a.saveScene(false) }))
		ui.Button(c, a.tr("Save as", "另存为")).OnClick(a.action("ui_scene.save_as", func() { a.saveScene(true) }))
	})
	ui.Button(c, a.tr("New structured scene", "新建结构化场景")).FillWidth().OnClick(a.action("scene.new_structured", func() {
		a.checkpoint()
		a.sceneDraft = a.newStructuredScene()
		a.sceneObject = ""
		a.hiddenFloors = map[string]bool{}
		a.checkpoint()
	}))
	ui.Text(c, a.tr("Groups & objects", "分组与物体")).Bold().Margin(12, 0, 4, 0)
	var rows func(string, int)
	rows = func(parent string, depth int) {
		if depth > 128 {
			return
		}
		for _, object := range a.sceneDraft.Objects {
			o := object
			if o.Parent != parent {
				continue
			}
			label := o.Name
			if a.sceneObject == o.ID {
				label = "• " + label
			}
			ui.Button(c.Key(o.ID), label).FillWidth().Margin(0, 0, 0, float32(min(depth, 6)*8)).OnClick(a.action("scene.select_object."+o.ID, func() { a.sceneObject = o.ID; a.sceneTemplateEdit = false }))
			if o.Kind == "group" {
				visible := !a.hiddenFloors[o.ID]
				if ui.Checkbox(c.Key("visible-"+o.ID), &visible, a.tr("Show group", "显示分组")).Changed() {
					a.hiddenFloors[o.ID] = !visible
				}
			}
			rows(o.ID, depth+1)
		}
	}
	rows("world", 0)
	ui.Text(c, a.tr("Place object", "放置物体")).Bold().Margin(12, 0, 4, 0)
	if a.sceneNewKind == "" {
		a.sceneNewKind = "box"
	}
	a.localizedSelect(c, &a.sceneNewKind, []string{"group", "box", "planar_region", "cylinder", "sphere", "ellipsoid", "frustum", "capsule", "ramp", "stair_flight", "spawn"}).FillWidth()
	ui.Button(c, a.tr("Add to selected group", "添加到所选分组")).FillWidth().OnClick(a.action("scene.add_structured", func() {
		parent := "world"
		if o := a.sceneDraft.object(a.sceneObject); o != nil {
			if o.Kind == "group" {
				parent = o.ID
			} else {
				parent = o.Parent
			}
		}
		o := SceneObject{ID: "object_" + newID(), Name: a.optionLabel(a.sceneNewKind), Kind: "primitive", Parent: parent, Pose: Pose{Quaternion: Quat{1, 0, 0, 0}}, Evidence: map[string]any{"basis": "manual"}}
		if a.sceneNewKind == "group" || a.sceneNewKind == "spawn" {
			o.Kind = "group"
			o.Evidence = nil
			if a.sceneNewKind == "spawn" {
				o.Semantic = "spawn"
			}
		} else {
			o.Geometry = defaultPrimitive(a.sceneNewKind)
		}
		a.sceneDraft.Objects = append(a.sceneDraft.Objects, o)
		a.sceneObject = o.ID
	}))
}
func (a *app) structuredInspector(c *ui.Context) {
	c = c.Key(a.sceneRevision)
	o := a.sceneDraft.object(a.sceneObject)
	if o == nil {
		ui.Text(c, a.tr("Select an object or group.", "请选择物体或分组。"))
		return
	}
	a.sceneFieldFocus(ui.TextInput(c, &o.Name).FillWidth().Label(a.tr("Object name", "物体名称")), "object-name")
	if a.editVector(c, a.tr("Position in parent · m", "相对父级位置 · 米"), &o.Position, -10000, 10000) {
		a.sceneDraft.markManualChange(o.ID)
	}
	euler := toEuler(o.Quaternion)
	if a.editVector(c, a.tr("Rotation · degrees", "旋转 · 度"), &euler, -180, 180) {
		o.Quaternion = fromEuler(euler)
		a.sceneDraft.markManualChange(o.ID)
	}
	if o.Semantic == "spawn" {
		ui.Button(c, a.tr("Use this spawn", "使用此出生点")).FillWidth().OnClick(a.action("scene.select_spawn", func() { a.cfg.SceneSpawn = o.ID }))
	}
	if o.Kind == "primitive" {
		if basis, ok := o.Evidence["basis"]; ok {
			ui.Text(c, fmt.Sprintf(a.tr("Source: %v", "来源：%v"), basis)).FontSize(12)
		}
		a.structuredGeometryInspector(c, o)
	}
	ui.Button(c, a.tr("Duplicate object", "复制物体")).FillWidth().Disabled(o.Kind == "group").OnClick(a.action("ui_scene.duplicate_object", func() {
		copy := clone(*o)
		copy.ID = "object_" + newID()
		copy.Name += a.tr(" copy", " 副本")
		copy.Position[0] += 1
		a.sceneDraft.Objects = append(a.sceneDraft.Objects, copy)
		a.sceneDraft.markManualChange(copy.ID)
		a.sceneObject = copy.ID
	}))
	a.alignmentInspector(c, o)
}
func defaultPrimitive(kind string) map[string]any {
	g := map[string]any{"type": kind}
	values := map[string]map[string]any{
		"box":           {"length_m": 2., "width_m": 1., "height_m": .2},
		"planar_region": {"outer_xy_m": [][2]float64{{-2, -2}, {2, -2}, {2, 2}, {-2, 2}}, "holes_xy_m": [][][2]float64{}, "thickness_m": .2},
		"cylinder":      {"radius_m": .3, "height_m": 2.}, "sphere": {"radius_m": .5},
		"ellipsoid": {"radii_m": Vec3{.5, .3, .8}}, "frustum": {"bottom_radius_m": .5, "top_radius_m": .25, "height_m": 1.},
		"capsule":      {"radius_m": .3, "cylinder_length_m": 1.},
		"ramp":         {"width_m": 1.2, "run_m": 4., "rise_m": 1., "slab_thickness_m": .15},
		"stair_flight": {"width_m": 1.2, "step_height_m": .15, "tread_depth_m": .28, "steps": 10., "slab_thickness_m": .15},
	}
	for key, value := range values[kind] {
		g[key] = value
	}
	return g
}
