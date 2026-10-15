package main

import (
	"encoding/json"
	"fmt"
	"os"
	"path/filepath"
	"sort"
)

type Sensors struct {
	MID360   bool `json:"mid360"`
	Camera   bool `json:"camera"`
	Depth    bool `json:"depth"`
	Panorama bool `json:"panorama"`
}
type Config struct {
	Robot        string          `json:"robot"`
	Controller   string          `json:"controller"`
	ControlMode  string          `json:"control_mode"`
	Terrain      string          `json:"terrain"`
	TerrainLevel string          `json:"terrain_level"`
	Backend      string          `json:"backend"`
	Sensors      Sensors         `json:"sensors"`
	Mounts       map[string]Pose `json:"mounts"`
	CameraHz     float64         `json:"camera_hz"`
	Adapters     map[string]bool `json:"adapters"`
}
type RobotSpec struct {
	Name              string  `json:"name"`
	DefaultController string  `json:"default_controller"`
	Continuous        bool    `json:"continuous_terrain"`
	AllowsLateral     bool    `json:"allows_lateral"`
	AllowsTurn        bool    `json:"allows_in_place_turn"`
	Limits            Vec3    `json:"command_limits"`
	MinRadius         float64 `json:"min_turning_radius"`
}
type Catalog struct {
	Models   map[string]json.RawMessage `json:"models"`
	Terrains map[string]json.RawMessage `json:"terrain_presets"`
	Robots   map[string]RobotSpec       `json:"robots"`
}

func defaultConfig() Config {
	return Config{Robot: "go2", Controller: "auto", ControlMode: "tracked", Terrain: "flat", TerrainLevel: "standard", Backend: "auto", Sensors: Sensors{true, true, true, false}, Mounts: map[string]Pose{}, CameraHz: 10, Adapters: map[string]bool{}}
}
func loadJSON(path string, out any) error {
	data, err := os.ReadFile(path)
	if err != nil {
		return err
	}
	return json.Unmarshal(data, out)
}
func asMap(value any) map[string]any {
	data, _ := json.Marshal(value)
	var out map[string]any
	_ = json.Unmarshal(data, &out)
	return out
}
func decodeAs[T any](value any) T {
	data, _ := json.Marshal(value)
	var out T
	_ = json.Unmarshal(data, &out)
	return out
}
func loadCatalog(root string, specs []json.RawMessage) (Catalog, error) {
	var catalog Catalog
	if err := loadJSON(filepath.Join(root, "assets", "catalog.json"), &catalog); err != nil {
		return catalog, err
	}
	catalog.Robots = map[string]RobotSpec{}
	for _, data := range specs {
		var spec RobotSpec
		if err := json.Unmarshal(data, &spec); err != nil {
			return catalog, fmt.Errorf("decode core robot registry: %w", err)
		}
		catalog.Robots[spec.Name] = spec
	}
	return catalog, nil
}
func sortedKeys[T any](values map[string]T) []string {
	keys := make([]string, 0, len(values))
	for key := range values {
		keys = append(keys, key)
	}
	sort.Strings(keys)
	return keys
}
func saveJSON(path string, value any) error {
	if err := os.MkdirAll(filepath.Dir(path), 0700); err != nil {
		return err
	}
	data, err := json.MarshalIndent(value, "", "  ")
	if err != nil {
		return err
	}
	temp := path + ".tmp"
	if err = os.WriteFile(temp, data, 0600); err != nil {
		return err
	}
	return os.Rename(temp, path)
}
func profilePath() string {
	base, err := os.UserConfigDir()
	if err != nil {
		return "session.json"
	}
	return filepath.Join(base, "simple-rbt", "session.json")
}
