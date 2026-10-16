package main

import (
	"encoding/json"
	"fmt"
	"math"
	"strings"
)

type Floor struct {
	ID     string  `json:"id"`
	Name   string  `json:"name"`
	Height float64 `json:"height"`
}
type SceneObject struct {
	ID    string `json:"id"`
	Name  string `json:"name"`
	Floor string `json:"floor"`
	Kind  string `json:"kind"`
	Pose
	Size       Vec3             `json:"size"`
	Connection *FloorConnection `json:"connection,omitempty"`
	Opening    *SlabOpening     `json:"opening,omitempty"`
}
type Spawn struct {
	Floor    string  `json:"floor"`
	Position Vec3    `json:"position"`
	Yaw      float64 `json:"yaw_deg"`
}
type Scene struct {
	Version int           `json:"version"`
	ID      string        `json:"id"`
	Name    string        `json:"name"`
	Floors  []Floor       `json:"floors"`
	Objects []SceneObject `json:"objects"`
	Spawn   Spawn         `json:"spawn"`
}

func (s *Scene) height(id string) float64 {
	for _, f := range s.Floors {
		if f.ID == id {
			return f.Height
		}
	}
	return 0
}
func (s *Scene) object(id string) *SceneObject {
	for i := range s.Objects {
		if s.Objects[i].ID == id {
			return &s.Objects[i]
		}
	}
	return nil
}
func (s *Scene) pose(o SceneObject) Pose { p := o.Pose; p.Position[2] += s.height(o.Floor); return p }
func validateScene(s *Scene) error {
	if s == nil {
		return nil
	}
	fail := fmt.Errorf("Invalid scene: check names, floors, dimensions and transforms / 场景无效：请检查名称、楼层、尺寸及位姿")
	ids := map[string]bool{}
	identity := func(id, name string) bool {
		if id == "" || len(id) > 80 || ids[id] || strings.TrimSpace(name) == "" || len(name) > 160 {
			return false
		}
		ids[id] = true
		return true
	}
	if s.Version != 1 || !identity(s.ID, s.Name) || len(s.Floors) < 1 || len(s.Floors) > 32 || len(s.Objects) > 512 {
		return fail
	}
	floors := map[string]bool{}
	finite := func(x, b float64) bool { return !math.IsNaN(x) && !math.IsInf(x, 0) && math.Abs(x) <= b }
	for _, f := range s.Floors {
		if !identity(f.ID, f.Name) || !finite(f.Height, 100) {
			return fail
		}
		floors[f.ID] = true
	}
	for _, o := range s.Objects {
		if !identity(o.ID, o.Name) || !floors[o.Floor] {
			return fail
		}
		if o.Kind != "floor" && o.Kind != "wall" && o.Kind != "ramp" && o.Kind != "obstacle" && o.Kind != "stairs" {
			return fail
		}
		if !s.connectionValid(o, floors) || !o.openingValid() {
			return fail
		}
		for i, x := range o.Position {
			if !finite(x, 100) || !finite(o.Size[i], 200) || o.Size[i] < .02 {
				return fail
			}
		}
		n := 0.
		for _, q := range o.Quaternion {
			n += q * q
		}
		if !finite(n, 2) || math.Abs(n-1) > 1e-6 {
			return fail
		}
	}
	if len(s.boxes()) > 2048 {
		return fail
	}
	if !floors[s.Spawn.Floor] || !finite(s.Spawn.Yaw, 180) {
		return fail
	}
	for _, x := range s.Spawn.Position {
		if !finite(x, 100) {
			return fail
		}
	}
	return nil
}
func newScene(twoFloors bool) Scene {
	floor := Floor{newID(), "Ground floor / 首层", 0}
	s := Scene{Version: 1, ID: newID(), Name: "New scene / 新场景", Floors: []Floor{floor}, Objects: []SceneObject{}, Spawn: Spawn{Floor: floor.ID}}
	add := func(name, kind, f string, p, size Vec3, q Quat) {
		s.Objects = append(s.Objects, SceneObject{ID: newID(), Name: name, Kind: kind, Floor: f, Pose: Pose{Position: p, Quaternion: q}, Size: size})
	}
	add("Ground / 地面", "floor", floor.ID, Vec3{0, 0, -.1}, Vec3{44, 24, .2}, Quat{1, 0, 0, 0})
	if twoFloors {
		s.Name = "Two floors / 双层场景"
		upper := Floor{newID(), "Upper floor / 二层", 3}
		s.Floors = append(s.Floors, upper)
		add("Upper slab / 二层楼板", "floor", upper.ID, Vec3{0, 0, -.1}, Vec3{8, 8, .2}, Quat{1, 0, 0, 0})
		add("Connecting ramp / 连接坡道", "ramp", floor.ID, Vec3{16, 0, 0}, Vec3{12, 2, .2}, fromEuler(Vec3{0, 0, 180}))
		s.Objects[len(s.Objects)-1].Connection = &FloorConnection{ToFloor: upper.ID}
		add("Rear wall / 后墙", "wall", upper.ID, Vec3{-4, 0, 1.25}, Vec3{.2, 8, 2.5}, Quat{1, 0, 0, 0})
		add("Side wall / 侧墙", "wall", upper.ID, Vec3{0, 4, 1.25}, Vec3{8, .2, 2.5}, Quat{1, 0, 0, 0})
	}
	return s
}

// Localize defaults once at creation; saved user names remain unchanged.
func (a *app) newScene(twoFloors bool) Scene {
	s := newScene(twoFloors)
	s.Name = a.localizedMessage(s.Name)
	for i := range s.Floors {
		s.Floors[i].Name = a.localizedMessage(s.Floors[i].Name)
	}
	for i := range s.Objects {
		s.Objects[i].Name = a.localizedMessage(s.Objects[i].Name)
	}
	return s
}

// Convert existing course primitives into the same editable scene document.
func (a *app) terrainScene() Scene {
	s := a.newScene(false)
	s.Name = a.optionLabel(a.cfg.Terrain) + " · " + a.optionLabel(a.cfg.TerrainLevel)
	s.Objects[0].Size = Vec3{24, 12, .2}
	var terrain struct {
		Boxes []struct {
			Name     string  `json:"name"`
			Centre   Vec3    `json:"centre"`
			HalfSize Vec3    `json:"half_size"`
			Slope    float64 `json:"slope"`
		} `json:"boxes"`
	}
	_ = json.Unmarshal(a.catalog.Terrains[a.cfg.Terrain+"-"+a.cfg.TerrainLevel], &terrain)
	for _, b := range terrain.Boxes {
		s.Objects = append(s.Objects, SceneObject{ID: newID(), Name: b.Name, Floor: s.Floors[0].ID, Kind: "obstacle", Pose: Pose{Position: b.Centre, Quaternion: fromEuler(Vec3{0, -b.Slope * 180 / math.Pi, 0})}, Size: mul(b.HalfSize, 2)})
	}
	for i, p := range []Vec3{{0, 6, 1.5}, {0, -6, 1.5}, {12, 0, 1.5}, {-12, 0, 1.5}} {
		size := Vec3{24, .2, 3}
		if i >= 2 {
			size = Vec3{.2, 12, 3}
		}
		s.Objects = append(s.Objects, SceneObject{ID: newID(), Name: fmt.Sprintf(a.tr("Wall %d", "围墙 %d"), i+1), Floor: s.Floors[0].ID, Kind: "wall", Pose: Pose{Position: p, Quaternion: Quat{1, 0, 0, 0}}, Size: size})
	}
	return s
}
