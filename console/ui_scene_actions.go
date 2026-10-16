package main

import (
	"fmt"
	"github.com/egoist/mygo/ui"
)

func (a *app) sceneActions(c *ui.Context) {
	ui.Row(c).Gap(6).Children(func() {
		ui.Button(c, a.tr("Undo", "撤销")).Disabled(len(a.sceneHistory.undo) == 0).OnClick(func() { a.undoScene(false) })
		ui.Button(c, a.tr("Redo", "重做")).Disabled(len(a.sceneHistory.redo) == 0).OnClick(func() { a.undoScene(true) })
		ui.Button(c, a.tr("Delete object", "删除物体")).Disabled(a.sceneDraft.object(a.sceneObject) == nil).OnClick(a.deleteSceneObject)
		ui.Checkbox(c, &a.workspace.Snap.Enabled, a.tr("Snap", "吸附"))
		ui.NumberInput(c.Key("snap-grid"), &a.workspace.Snap.Grid, .01, 5, .05).Width(160).Label(a.tr("Grid spacing in metres", "网格间距（米）"))
		ui.Text(c, "m")
		ui.NumberInput(c.Key("snap-angle"), &a.workspace.Snap.Angle, 1, 90, 5).Width(160).Label(a.tr("Angular snap in degrees", "角度吸附（度）"))
		ui.Text(c, "°")
		ui.Button(c, a.tr("Snap selected", "对齐网格")).Disabled(a.sceneObject == "").OnClick(a.snapSelected)
	})
}
func (a *app) alignmentInspector(c *ui.Context, o *SceneObject) {
	ui.Text(c, a.tr("Align to object", "对齐到物体")).Bold().Margin(12, 0, 4, 0)
	names := []string{}
	ids := map[string]string{}
	selected := ""
	for i, object := range a.sceneDraft.Objects {
		if object.ID == o.ID {
			continue
		}
		label := object.Name + " · " + a.sceneDraft.Floors[0].Name
		for _, floor := range a.sceneDraft.Floors {
			if floor.ID == object.Floor {
				label = object.Name + " · " + floor.Name
				break
			}
		}
		label = fmt.Sprintf("%d · %s", i+1, label)
		names = append(names, label)
		ids[label] = object.ID
		if a.alignTarget == object.ID {
			selected = label
		}
	}
	if ui.Select(c.Key("alignment-target"), &selected, names).FillWidth().Changed() {
		a.alignTarget = ids[selected]
	}
	for _, row := range [][]string{{"center-x", "center-y"}, {"right", "front"}, {"top"}} {
		ui.Row(c).Gap(6).Children(func() {
			for _, mode := range row {
				mode := mode
				labels := map[string][2]string{"center-x": {"Center X", "X 居中"}, "center-y": {"Center Y", "Y 居中"}, "right": {"Join +X edge", "贴合 +X 边"}, "front": {"Join +Y edge", "贴合 +Y 边"}, "top": {"Place on top", "放置到顶面"}}
				label := labels[mode]
				ui.Button(c, a.tr(label[0], label[1])).Disabled(a.alignTarget == "" || (mode == "top" && o.Connection != nil)).OnClick(func() { a.alignSceneObject(mode) })
			}
		})
	}
}
