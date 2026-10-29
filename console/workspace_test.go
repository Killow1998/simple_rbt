package main

import (
	"encoding/json"
	"os"
	"simple-rbt/console/session"
	"testing"
)

func workspaceApp(t *testing.T) *app {
	t.Helper()
	a := newApp(session.Options{StateDir: t.TempDir()}, defaultConfig())
	t.Cleanup(a.manager.Events.Close)
	a.catalog = Catalog{Robots: map[string]RobotSpec{"go2": {DefaultController: "rl"}}, Models: map[string]json.RawMessage{"go2-rl": {}}, Terrains: map[string]json.RawMessage{"flat-standard": {}}}
	return a
}
func TestWorkspaceReopenPreservesNamedResourcesAndDrafts(t *testing.T) {
	t.Setenv("XDG_CONFIG_HOME", t.TempDir())
	a := workspaceApp(t)
	a.initWorkspace(false)
	a.robotPreset.Name = "Xian 的巡检机器人"
	pose := Pose{Position: Vec3{.18, .02, .25}, Quaternion: Quat{1, 0, 0, 0}}
	a.cfg.Mounts["mid360"] = pose
	a.cfg.LidarModel = "avia"
	a.saveRobot(false)
	id := a.robotPreset.ID
	a.robotPreset.Name = "Renamed survey robot"
	a.saveRobot(false)
	if a.robotPreset.ID != id || len(a.workspace.Robots) != 1 {
		t.Fatal("rename changed identity")
	}
	stored := a.workspace.Robots[id]
	cfg := defaultConfig()
	stored.apply(&cfg)
	if cfg.LidarModel != "avia" {
		t.Fatal("saved preset lost Avia")
	}
	stored.LidarModel = ""
	stored.apply(&cfg)
	if cfg.LidarModel != "mid360" {
		t.Fatal("legacy preset lost MID360 default")
	}
	a.sceneDraft.Name = "双层实验室"
	a.saveScene(false)
	sceneID := a.sceneDraft.ID
	a.sceneDraft.Objects[1].Position[0] = 2.5
	a.robotPreset.Name = "Unfinished name"
	a.checkpoint()
	b := workspaceApp(t)
	b.initWorkspace(true)
	if b.cfg.LidarModel != "avia" || b.robotPreset.LidarModel != "avia" || b.robotPreset.Name != "Unfinished name" || b.robotPreset.ID != id || !samePose(b.cfg.Mounts["mid360"], pose) {
		t.Fatal("robot draft did not reopen")
	}
	if b.workspace.Robots[id].Name != "Renamed survey robot" || b.workspace.Scenes[sceneID].Objects[1].Position[0] != 0 || b.sceneDraft.Objects[1].Position[0] != 2.5 {
		t.Fatal("draft overwrote saved resource")
	}
	b.sceneDraft.Name = ""
	b.checkpoint()
	c := workspaceApp(t)
	c.initWorkspace(true)
	if c.workspaceBlocked || c.sceneDraft.Name != "" {
		t.Fatal("incomplete name lost on reopen")
	}
	b.robotPreset.Name = "Another preset"
	b.saveRobot(true)
	if b.robotPreset.ID == id || len(b.workspace.Robots) != 2 {
		t.Fatal("save as replaced old preset")
	}
	d := workspaceApp(t)
	d.initWorkspace(false)
	if len(d.workspace.Robots) != 2 || d.cfg.Robot != "go2" {
		t.Fatal("explicit config erased existing library")
	}
}
func TestUnsupportedWorkspaceIsNotOverwritten(t *testing.T) {
	t.Setenv("XDG_CONFIG_HOME", t.TempDir())
	saveJSON(workspacePath(), map[string]any{"version": 999})
	before, _ := os.ReadFile(workspacePath())
	a := workspaceApp(t)
	a.initWorkspace(true)
	a.checkpoint()
	after, _ := os.ReadFile(workspacePath())
	if !a.workspaceBlocked || string(before) != string(after) {
		t.Fatal("unsupported workspace overwritten")
	}
}
