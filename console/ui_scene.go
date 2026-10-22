package main

import (
	"fmt"
	"github.com/egoist/mygo/ui"
)

func (a *app) sceneSidebar(c *ui.Context) {
	c = c.Key(a.sceneRevision)
	ui.Text(c, a.tr("Scene library", "场景库")).Bold()
	a.sceneFieldFocus(ui.TextInput(c.Key("scene-name"), &a.sceneDraft.Name).FillWidth().Label(a.tr("Scene name", "场景名称")), "scene-name")
	if a.sceneDirty() {
		ui.Text(c, a.tr("Unsaved changes", "未保存的修改")).TextColor(c.Theme().Warning)
	}
	ui.Row(c).Gap(6).Children(func() {
		ui.Button(c, a.tr("Save scene", "保存场景")).OnClick(a.action("ui_scene.save_scene", func() { a.saveScene(false) }))
		ui.Button(c, a.tr("Save as", "另存为")).OnClick(a.action("ui_scene.save_as", func() { a.saveScene(true) }))
	})
	ui.Row(c).Gap(6).Children(func() {
		for _, two := range []bool{false, true} {
			two := two
			label := a.tr("New empty", "新建空场景")
			if two {
				label = a.tr("Two floors", "双层模板")
			}
			ui.Button(c, label).OnClick(a.action("ui_scene.two_floors", func() {
				a.checkpoint()
				a.sceneDraft = a.newScene(two)
				a.sceneFloor = a.sceneDraft.Floors[0].ID
				a.sceneObject = ""
				a.hiddenFloors = map[string]bool{}
				a.checkpoint()
			}))
		}
	})
	ui.Text(c, a.tr("Floors & objects", "楼层与物体")).Bold().Margin(12, 0, 4, 0)
	for _, f := range a.sceneDraft.Floors {
		f := f
		label := f.Name
		if a.sceneFloor == f.ID {
			label = "• " + label
		}
		ui.Button(c.Key("floor-"+f.ID), label).FillWidth().OnClick(a.action("scene.select_floor."+f.ID, func() { a.sceneFloor = f.ID; a.sceneObject = "" }))
		visible := !a.hiddenFloors[f.ID]
		if ui.Checkbox(c.Key("floor-visible-"+f.ID), &visible, a.tr("Show in editor", "编辑器中显示")).Changed() {
			a.hiddenFloors[f.ID] = !visible
		}
		if a.sceneFloor == f.ID {
			for _, o := range a.sceneDraft.Objects {
				if o.Floor != f.ID {
					continue
				}
				o := o
				label := o.Name
				if a.sceneObject == o.ID {
					label = "• " + label
				}
				ui.Button(c.Key("object-"+o.ID), label).FillWidth().OnClick(a.action("scene.select_object."+o.ID, func() { a.sceneObject = o.ID }))
			}
		}
	}
	ui.Button(c, a.tr("Add floor", "添加楼层")).FillWidth().Disabled(len(a.sceneDraft.Floors) >= 32).OnClick(a.action("ui_scene.add_floor", func() {
		h := 0.
		for _, f := range a.sceneDraft.Floors {
			if f.Height >= h {
				h = f.Height + 3
			}
		}
		f := Floor{newID(), fmt.Sprintf(a.tr("Floor %d", "楼层 %d"), len(a.sceneDraft.Floors)+1), h}
		a.sceneDraft.Floors = append(a.sceneDraft.Floors, f)
		a.sceneFloor = f.ID
		a.sceneObject = ""
	}))
	ui.Text(c, a.tr("Place object", "放置物体")).Bold().Margin(12, 0, 4, 0)
	for _, kind := range []string{"floor", "wall", "ramp", "stairs", "obstacle"} {
		kind := kind
		ui.Button(c, a.sceneKindLabel(kind)).FillWidth().Disabled(len(a.sceneDraft.Objects) >= 512).OnClick(a.action("scene.add."+kind, func() { a.addSceneObject(kind) }))
	}
	ui.Button(c, a.tr("Robot spawn", "机器人出生点")).FillWidth().OnClick(a.action("ui_scene.robot_spawn", func() { a.sceneObject = "spawn" }))
	ui.Text(c, a.tr("Hidden floors still exist in simulation.", "隐藏楼层仅影响编辑视图，仿真中仍然存在。")).FontSize(12).TextColor(c.Theme().TextMuted)
}
func (a *app) sceneKindLabel(kind string) string {
	switch kind {
	case "floor":
		return a.tr("Floor slab", "楼板")
	case "wall":
		return a.tr("Wall", "墙体")
	case "stairs":
		return a.tr("Stairs", "楼梯")
	case "ramp":
		return a.tr("Ramp", "坡道")
	}
	return a.tr("Obstacle", "障碍物")
}
func (a *app) addSceneObject(kind string) {
	o := SceneObject{ID: newID(), Name: a.sceneKindLabel(kind), Floor: a.sceneFloor, Kind: kind, Pose: Pose{Quaternion: Quat{1, 0, 0, 0}}, Size: Vec3{1, 1, 1}}
	switch kind {
	case "floor":
		o.Size = Vec3{6, 6, .2}
		o.Position[2] = -.1
	case "wall":
		o.Size = Vec3{4, .2, 2.5}
		o.Position[2] = 1.25
	case "ramp", "stairs":
		if !a.connectFloors(&o) {
			return
		}
	default:
		o.Position[2] = .5
	}
	a.sceneDraft.Objects = append(a.sceneDraft.Objects, o)
	a.sceneObject = o.ID
}
func (a *app) sceneEditor(c *ui.Context) {
	a.sceneActions(c)
	ui.Row(c).Gap(8).Children(func() {
		ui.Text(c, a.tr("Scene editor", "场景编辑器")).Bold().Grow(1)
		a.localizedSelect(c.Key("tool"), &a.tool, []string{"Move / 平移", "Rotate / 旋转"}).Width(130)
		a.localizedSelect(c.Key("space"), &a.space, []string{"World / 世界", "Local / 设备"}).Width(130)
		ui.Button(c, a.tr("Focus selection", "聚焦所选")).OnClick(a.action("ui_scene.focus_selection", func() {
			if p, ok := a.editPose(); ok {
				a.sceneLookat = p.Position
				a.viewport.Distance = 8
				if low, high, ok := a.sceneDraft.bounds(a.sceneObject); ok {
					a.sceneLookat = mul(add(low, high), .5)
					a.viewport.Distance = clamp(norm(sub(high, low))*1.5, 8, 100)
				}
			}
		}))
	})
	ui.Text(c, a.tr("Drag axes · right-drag to orbit · Shift + right-drag to pan · scroll to zoom", "拖动操作轴；右键环绕；Shift＋右键平移视图；滚轮缩放")).FontSize(12).TextColor(c.Theme().TextMuted)
	ui.Row(c).Grow(1).MinHeight(0).AlignItems(ui.Stretch).Gap(12).Children(func() {
		view := ui.Box(c.Key("scene-viewport")).Grow(1).MinWidth(100).Clip().Background(viewportBackground).Focusable().Label(a.tr("3D scene viewport", "场景三维视图")).HandleInput(a.viewportInput)
		x, y, over := view.PointerPosition()
		a.viewport.Pointer, a.viewport.Hovering = point{float64(x), float64(y)}, over
		view.Draw(func(p *ui.Painter, r ui.Rect) {
			a.viewport.Width, a.viewport.Height = float64(r.W), float64(r.H)
			p.Clip(r, 0, func() { a.drawScene(p, r); a.drawGizmo(p, r) })
		})
		ui.Scroll(c).FillHeight().Width(270).Shrink(0).Padding(10).Background(panelBackground(c)).Children(func() { a.sceneInspector(c) })
	})
}
func (a *app) sceneInspector(c *ui.Context) {
	c = c.Key(a.sceneRevision)
	ui.Text(c, a.tr("Properties", "属性")).Bold()
	if a.sceneObject == "spawn" {
		names := []string{}
		ids := map[string]string{}
		selected := ""
		for i, f := range a.sceneDraft.Floors {
			label := fmt.Sprintf("%d · %s", i+1, f.Name)
			names = append(names, label)
			ids[label] = f.ID
			if f.ID == a.sceneDraft.Spawn.Floor {
				selected = label
			}
		}
		if ui.Select(c, &selected, names).FillWidth().Changed() {
			a.sceneDraft.Spawn.Floor = ids[selected]
		}
		a.editVector(c, a.tr("Spawn position", "出生位置"), &a.sceneDraft.Spawn.Position, -100, 100)
		a.sceneFieldFocus(ui.NumberInput(c, &a.sceneDraft.Spawn.Yaw, -180, 180, 1).Label(a.tr("Spawn yaw", "出生朝向")).FillWidth(), "spawn-yaw")
		ui.Text(c, a.tr("Position is a ground point relative to its floor; chassis clearance is added automatically.", "位置是相对所在楼层的地面点，底盘离地高度自动添加。")).FontSize(12)
		return
	}
	o := a.sceneDraft.object(a.sceneObject)
	if o == nil {
		for i := range a.sceneDraft.Floors {
			f := &a.sceneDraft.Floors[i]
			if f.ID == a.sceneFloor {
				a.sceneFieldFocus(ui.TextInput(c, &f.Name).FillWidth().Label(a.tr("Floor name", "楼层名称")), "floor-name")
				ui.Text(c, a.tr("Floor elevation · m", "楼层标高 · 米"))
				a.sceneFieldFocus(ui.NumberInput(c, &f.Height, -100, 100, .1).FillWidth().Label(a.tr("Floor elevation", "楼层标高")), "floor-height")
			}
		}
		ui.Text(c, a.tr("Select an object to edit its transform and size.", "选择物体后可编辑位姿和尺寸。"))
		return
	}
	a.sceneFieldFocus(ui.TextInput(c, &o.Name).FillWidth().Label(a.tr("Object name", "物体名称")), "object-name")
	ui.Text(c, a.sceneKindLabel(o.Kind)).TextColor(c.Theme().TextMuted)
	if o.Connection != nil {
		a.connectionInspector(c, o)
	} else {
		a.editVector(c, a.tr("Position on floor · m", "相对楼层位置 · 米"), &o.Position, -100, 100)
		euler := toEuler(o.Quaternion)
		if a.editVector(c, a.tr("Rotation · degrees", "旋转 · 度"), &euler, -180, 180) {
			o.Quaternion = fromEuler(euler)
		}
		a.editVector(c, a.tr("Dimensions · m", "尺寸 · 米"), &o.Size, .02, 200)
	}
	ui.Button(c, a.tr("Duplicate object", "复制物体")).FillWidth().Disabled(len(a.sceneDraft.Objects) >= 512).OnClick(a.action("ui_scene.duplicate_object", func() {
		copy := clone(*o)
		copy.ID = newID()
		copy.Name += a.tr(" copy", " 副本")
		copy.Position[0] += 1
		a.sceneDraft.Objects = append(a.sceneDraft.Objects, copy)
		a.sceneObject = copy.ID
	}))
	a.openingInspector(c, o)
	a.alignmentInspector(c, o)
}
func (a *app) editVector(c *ui.Context, label string, v *Vec3, lo, hi float64) bool {
	changed := false
	ui.Text(c, label).FontSize(12).Margin(10, 0, 4, 0)
	for i, axis := range []string{"X", "Y", "Z"} {
		i, axis := i, axis
		ui.Row(c).Gap(6).Children(func() {
			ui.Text(c, axis).Width(18)
			element := ui.NumberInput(c.Key(label+axis), &v[i], lo, hi, .01).Width(205).Label(label + " " + axis)
			a.sceneFieldFocus(element, label+axis)
			if element.Changed() {
				changed = true
			}
		})
	}
	return changed
}
