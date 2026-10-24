package main

import "github.com/egoist/mygo/ui"

// One entry per completed gesture/field edit. History is private to the current
// scene and bounded independently of the on-disk saved preset and draft.
type sceneSnapshot struct {
	Scene         Scene
	Floor, Object string
}
type sceneHistory struct {
	current    sceneSnapshot
	undo, redo []sceneSnapshot
	group      string
}

func (a *app) sceneSnapshot() sceneSnapshot {
	return sceneSnapshot{clone(a.sceneDraft), a.sceneFloor, a.sceneObject}
}
func (a *app) beginSceneFrame() {
	a.sceneField = ""
	if a.sceneHistory.current.Scene.ID != a.sceneDraft.ID {
		a.sceneHistory = sceneHistory{current: a.sceneSnapshot()}
	}
}
func (a *app) endSceneFrame() {
	h := &a.sceneHistory
	if h.current.Scene.ID != a.sceneDraft.ID {
		a.sceneHistory = sceneHistory{current: a.sceneSnapshot()}
		return
	}
	group := a.sceneField
	if a.viewport.Dragging {
		group = "drag:" + a.sceneObject
	}
	if !sameJSON(h.current.Scene, a.sceneDraft) {
		if group == "" || group != h.group {
			h.undo = append(h.undo, h.current)
			if len(h.undo) > 64 {
				h.undo = append([]sceneSnapshot(nil), h.undo[len(h.undo)-64:]...)
			}
		}
		h.redo = nil
		h.current = a.sceneSnapshot()
		h.group = group
	} else {
		h.current.Floor, h.current.Object = a.sceneFloor, a.sceneObject
		if group != h.group {
			h.group = ""
		}
	}
}
func (a *app) restoreScene(s sceneSnapshot) {
	a.sceneRevision++
	a.sceneDraft = clone(s.Scene)
	a.sceneFloor = s.Floor
	a.sceneObject = s.Object
	a.viewport.Dragging = false
	a.viewport.Orbit = false
	a.sceneHistory.current = a.sceneSnapshot()
	a.sceneHistory.group = ""
}
func (a *app) undoScene(redo bool) {
	h := &a.sceneHistory
	from, to := &h.undo, &h.redo
	if redo {
		from, to = &h.redo, &h.undo
	}
	if len(*from) == 0 {
		return
	}
	*to = append(*to, a.sceneSnapshot())
	s := (*from)[len(*from)-1]
	*from = (*from)[:len(*from)-1]
	a.restoreScene(s)
}
func (a *app) sceneFieldFocus(element ui.Element, key string) {
	if a.page == "scene" && element.FocusWithin() {
		a.sceneField = a.sceneObject + ":" + a.sceneFloor + ":" + key
	}
}
func (a *app) deleteSceneObject() {
	if a.sceneDraft.object(a.sceneObject) == nil {
		return
	}
	if a.sceneDraft.Version == 2 {
		remove := map[string]bool{a.sceneObject: true}
		changed := true
		for changed {
			changed = false
			for _, o := range a.sceneDraft.Objects {
				if remove[o.Parent] && !remove[o.ID] {
					remove[o.ID] = true
					changed = true
				}
			}
		}
		kept := []SceneObject{}
		for _, o := range a.sceneDraft.Objects {
			if !remove[o.ID] {
				kept = append(kept, o)
			}
		}
		a.sceneDraft.Objects = kept
		a.sceneObject = ""
		return
	}
	objects := a.sceneDraft.Objects
	for i, o := range objects {
		if o.ID == a.sceneObject {
			a.sceneDraft.Objects = append(objects[:i:i], objects[i+1:]...)
			break
		}
	}
	a.sceneObject = ""
}
func (a *app) sceneShortcut(event ui.InputEvent) bool {
	if a.page != "scene" || a.stage != "configure" || event.Kind != ui.InputKeyDown || event.Repeat {
		return false
	}
	if event.Mods&ui.Cmd != 0 {
		if event.Key == ui.KeyZ {
			a.undoScene(event.Mods&ui.Shift != 0)
			return true
		}
		if event.Key == ui.KeyY {
			a.undoScene(true)
			return true
		}
	}
	if event.Key == ui.KeyDelete && event.Mods == 0 {
		a.deleteSceneObject()
		return true
	}
	return false
}
