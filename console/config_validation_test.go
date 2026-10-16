package main

import (
	"encoding/json"
	"math"
	"testing"
)

func TestProfileValidation(t *testing.T) {
	catalog := Catalog{Robots: map[string]RobotSpec{"go2w": {DefaultController: "rl", Continuous: false}, "scout_mini": {DefaultController: "drive", Continuous: true}}, Models: map[string]json.RawMessage{"go2w-rl": {}, "scout_mini-drive": {}}, Terrains: map[string]json.RawMessage{"flat-standard": {}, "stairs-hard": {}}}
	cases := []struct {
		name   string
		change func(*Config)
		valid  bool
	}{
		{"old profile defaults", func(c *Config) { _ = json.Unmarshal([]byte(`{"robot":"go2w"}`), c) }, true},
		{"external estimator", func(c *Config) { c.ROS = ROSConfig{false, "/ground_truth/odom"} }, true},
		{"missing controller", func(c *Config) { c.Controller = "missing" }, false},
		{"wheel steps", func(c *Config) { c.Robot = "scout_mini"; c.Terrain = "stairs"; c.TerrainLevel = "hard" }, false},
		{"unknown terrain", func(c *Config) { c.Terrain = "missing" }, false},
		{"zero camera rate", func(c *Config) { c.CameraHz = 0 }, false},
		{"NaN rate", func(c *Config) { c.CameraHz = math.NaN() }, false},
		{"relative topic", func(c *Config) { c.ROS.OdomTopic = "odom" }, false},
		{"zero quaternion", func(c *Config) { c.Mounts["camera"] = Pose{} }, false},
		{"infinite translation", func(c *Config) {
			c.Mounts["camera"] = Pose{Position: Vec3{math.Inf(1), 0, 0}, Quaternion: Quat{1, 0, 0, 0}}
		}, false},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			cfg := defaultConfig()
			cfg.Robot = "go2w"
			tc.change(&cfg)
			if err := validateConfig(cfg, catalog); (err == nil) != tc.valid {
				t.Fatalf("valid=%v error=%v", tc.valid, err)
			}
		})
	}
}
