package main

import (
	"encoding/json"
	"math"
)

// v1 fields remain unchanged on disk. v2 uses nested poses and parent groups;
// geometry and evidence survive every workspace copy, undo and save.
type structuredPose struct {
	Position   Vec3 `json:"position_m"`
	Quaternion Quat `json:"quaternion_wxyz"`
}

func (o SceneObject) MarshalJSON() ([]byte, error) {
	type legacy SceneObject
	if o.Kind != "primitive" && o.Kind != "group" {
		return json.Marshal(legacy(o))
	}
	row := map[string]any{"id": o.ID, "name": o.Name, "kind": o.Kind, "parent": o.Parent,
		"pose": structuredPose{o.Position, o.Quaternion}}
	if o.Semantic != "" {
		row["semantic"] = o.Semantic
	}
	if o.Geometry != nil {
		row["geometry"] = o.Geometry
	}
	if o.Evidence != nil {
		row["evidence"] = o.Evidence
	}
	return json.Marshal(row)
}
func (o *SceneObject) UnmarshalJSON(data []byte) error {
	type legacy SceneObject
	var raw struct {
		legacy
		Transform structuredPose `json:"pose"`
	}
	if err := json.Unmarshal(data, &raw); err != nil {
		return err
	}
	*o = SceneObject(raw.legacy)
	if o.Kind == "primitive" || o.Kind == "group" {
		o.Pose = Pose{Position: raw.Transform.Position, Quaternion: raw.Transform.Quaternion}
	}
	return nil
}
func (s Scene) MarshalJSON() ([]byte, error) {
	type legacy Scene
	if s.Version != 2 {
		return json.Marshal(legacy(s))
	}
	return json.Marshal(struct {
		Format      string                    `json:"format"`
		Version     int                       `json:"version"`
		ID          string                    `json:"id"`
		Name        string                    `json:"name"`
		Units       map[string]string         `json:"units"`
		Coordinates string                    `json:"coordinate_system"`
		Templates   map[string]map[string]any `json:"geometry_templates,omitempty"`
		Objects     []SceneObject             `json:"objects"`
	}{s.Format, s.Version, s.ID, s.Name, s.Units, s.Coordinates, s.Templates, s.Objects})
}
func (s *Scene) firstFloor() string {
	if len(s.Floors) > 0 {
		return s.Floors[0].ID
	}
	return "world"
}
func (s *Scene) structuredPose(o SceneObject, depth int) Pose {
	p := o.Pose
	n := 0.
	for _, v := range p.Quaternion {
		n += v * v
	}
	if n > 0 {
		for i := range p.Quaternion {
			p.Quaternion[i] /= math.Sqrt(n)
		}
	}
	if o.Parent == "world" || depth > 128 {
		return p
	}
	if parent := s.object(o.Parent); parent != nil {
		base := s.structuredPose(*parent, depth+1)
		p.Position = add(base.Position, rotate(p.Position, base.Quaternion))
		p.Quaternion = quatProduct(base.Quaternion, p.Quaternion)
	}
	return p
}
func quatProduct(a, b Quat) Quat {
	return Quat{
		a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
		a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
		a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1],
		a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0]}
}
func (s *Scene) hidden(id string, hidden map[string]bool) bool {
	for n := 0; n < 128; n++ {
		if hidden[id] {
			return true
		}
		o := s.object(id)
		if o == nil || o.Parent == "world" {
			break
		}
		id = o.Parent
	}
	return false
}

func (s *Scene) containsNode(parent, child string) bool {
	for depth := 0; depth < 128; depth++ {
		if parent == child {
			return true
		}
		o := s.object(child)
		if o == nil || o.Parent == "world" {
			return false
		}
		child = o.Parent
	}
	return false
}
func (s *Scene) markManualChange(id string) {
	for i := range s.Objects {
		o := &s.Objects[i]
		if o.Evidence != nil && o.Evidence["basis"] != "manual" && s.containsNode(id, o.ID) {
			o.Evidence["basis"] = "mixed"
		}
	}
}
