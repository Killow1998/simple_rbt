package main

import (
	"encoding/json"
	"fmt"
	"github.com/egoist/mygo/ui"
	"reflect"
	"sort"
)

func (a *app) action(name string, callback func()) func() {
	return func() {
		if a.manager == nil || a.manager.Events == nil {
			callback()
			return
		}
		id := newID()
		a.manager.Client.SetAction(id)
		defer a.manager.Client.SetAction("")
		a.manager.Events.Event("ui.action", map[string]any{"action_id": id, "control": name, "page": a.page, "object": a.sceneObject})
		callback()
		a.auditConfiguration()
	}
}
func (a *app) auditInput(event ui.InputEvent, control string) {
	if a.manager == nil {
		return
	}
	if event.Kind == ui.InputPointerDown || event.Kind == ui.InputPointerUp || ((event.Kind == ui.InputKeyDown || event.Kind == ui.InputKeyUp) && !event.Repeat) {
		a.manager.Events.Event("ui.input", map[string]any{"control": control, "kind": event.Kind, "button": event.Button, "key": event.Key})
	}
}
func auditValue(value any) any {
	data, _ := json.Marshal(value)
	if len(data) > 4096 {
		return map[string]any{"truncated": true, "bytes": len(data)}
	}
	return value
}
func (a *app) auditConfiguration() {
	if a.manager == nil {
		return
	}
	// Persist semantic changes at action boundaries / the one-second checkpoint,
	// not every mouse-motion sample. Object IDs keep edits identifiable after rename.
	objects := map[string]any{}
	for _, o := range a.sceneDraft.Objects {
		objects[o.ID] = o
	}
	value := map[string]any{"config": a.cfg, "scene_objects": objects, "scene_floors": a.sceneDraft.Floors, "scene_name": a.sceneDraft.Name, "preset_name": a.robotPreset.Name, "page": a.page, "appearance": a.preferences, "snap": a.workspace.Snap}
	data, _ := json.Marshal(value)
	var current map[string]any
	json.Unmarshal(data, &current)
	if a.auditPrevious != nil {
		var walk func(string, any, any)
		changes := 0
		walk = func(path string, old, next any) {
			if reflect.DeepEqual(old, next) {
				return
			}
			before, bok := old.(map[string]any)
			after, aok := next.(map[string]any)
			if bok && aok {
				keys := map[string]bool{}
				for k := range before {
					keys[k] = true
				}
				for k := range after {
					keys[k] = true
				}
				ordered := []string{}
				for k := range keys {
					ordered = append(ordered, k)
				}
				sort.Strings(ordered)
				for _, k := range ordered {
					walk(path+"/"+k, before[k], after[k])
				}
				return
			}
			changes++
			if changes <= 512 {
				a.manager.Events.Event("ui.changed", map[string]any{"path": path, "before": auditValue(old), "after": auditValue(next)})
			}
		}
		walk("", a.auditPrevious, current)
		if changes > 512 {
			a.manager.Events.Emit("ui.change_summary", "warning", map[string]any{"changes": changes, "detail_limit": 512})
		}
	}
	a.auditPrevious = current
}
func (a *app) diagnosticError(err error) {
	if err != nil {
		a.status = fmt.Sprintf("Diagnostics / 诊断：%v", err)
	}
}
