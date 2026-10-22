package main

import (
	"crypto/rand"
	"encoding/hex"
	"encoding/json"
	"fmt"
	"os"
	"path/filepath"
	"strings"
)

type RobotPreset struct {
	ID   string `json:"id"`
	Name string `json:"name"`
	RobotConfig
}
type RobotConfig struct {
	Robot            string          `json:"robot"`
	Controller       string          `json:"controller"`
	ControlMode      string          `json:"control_mode"`
	Sensors          Sensors         `json:"sensors"`
	Mounts           map[string]Pose `json:"mounts"`
	CameraModel      string          `json:"camera_model"`
	CameraExtrinsics map[string]Pose `json:"camera_extrinsics,omitempty"`
	CameraHz         float64         `json:"camera_hz"`
}
type Workspace struct {
	Snap        sceneSnap              `json:"snap"`
	Version     int                    `json:"version"`
	Config      Config                 `json:"config"`
	RobotID     string                 `json:"robot_id"`
	SceneID     string                 `json:"scene_id"`
	Robots      map[string]RobotPreset `json:"robots"`
	Scenes      map[string]Scene       `json:"scenes"`
	RobotDrafts map[string]RobotPreset `json:"robot_drafts"`
	SceneDrafts map[string]Scene       `json:"scene_drafts"`
}

func newID() string {
	var b [12]byte
	if _, err := rand.Read(b[:]); err != nil {
		panic(err)
	}
	return hex.EncodeToString(b[:])
}
func clone[T any](v T) T {
	data, _ := json.Marshal(v)
	var out T
	_ = json.Unmarshal(data, &out)
	return out
}
func robotConfig(c Config) RobotConfig {
	return clone(RobotConfig{c.Robot, c.Controller, c.ControlMode, c.Sensors, c.Mounts, c.CameraModel, c.CameraExtrinsics, c.CameraHz})
}
func (r RobotConfig) apply(c *Config) {
	r = clone(r)
	c.Robot = r.Robot
	c.Controller = r.Controller
	c.ControlMode = r.ControlMode
	c.Sensors = r.Sensors
	c.Mounts = r.Mounts
	c.CameraModel = r.CameraModel
	c.CameraExtrinsics = r.CameraExtrinsics
	c.CameraHz = r.CameraHz
}
func workspacePath() string { return filepath.Join(filepath.Dir(profilePath()), "workspace.json") }
func (a *app) initWorkspace(restore bool) {
	a.workspace = Workspace{Version: 1, Robots: map[string]RobotPreset{}, Scenes: map[string]Scene{}, RobotDrafts: map[string]RobotPreset{}, SceneDrafts: map[string]Scene{}}
	var w Workspace
	err := loadJSON(workspacePath(), &w)
	if err == nil {
		valid := w.Version == 1 && w.Robots != nil && w.Scenes != nil && w.RobotDrafts != nil && w.SceneDrafts != nil
		for _, scene := range w.Scenes {
			if validateScene(&scene) != nil {
				valid = false
			}
		}
		for _, scene := range w.SceneDrafts {
			if scene.Version != 1 || len(scene.Floors) == 0 {
				valid = false
			}
		}
		if !valid {
			err = fmt.Errorf("Unsupported or invalid workspace / 工作区版本或内容无效")
		} else {
			a.workspace = w
			if restore {
				a.cfg = clone(w.Config)
			} else {
				a.workspace.RobotID = ""
				a.workspace.SceneID = ""
			}
		}
	}
	if err != nil && !os.IsNotExist(err) {
		a.reportError(fmt.Errorf("Workspace could not be read; existing file preserved: %w", err))
		a.workspaceBlocked = true
	}

	if r, ok := a.workspace.RobotDrafts[a.workspace.RobotID]; ok {
		a.robotPreset = r
	} else if r, ok := a.workspace.Robots[a.workspace.RobotID]; ok {
		a.robotPreset = clone(r)
	} else {
		a.robotPreset = RobotPreset{ID: newID(), Name: a.cfg.Robot + " · MID360 + " + a.cfg.CameraModel, RobotConfig: robotConfig(a.cfg)}
		a.workspace.RobotID = a.robotPreset.ID
	}
	if a.cfg.Scene != nil {
		a.sceneDraft = clone(*a.cfg.Scene)
	} else {
		a.sceneDraft = a.newScene(true)
	}
	if s, ok := a.workspace.SceneDrafts[a.workspace.SceneID]; ok {
		a.sceneDraft = clone(s)
	} else if s, ok := a.workspace.Scenes[a.workspace.SceneID]; ok {
		a.sceneDraft = clone(s)
	}
	if a.sceneDraft.Version != 1 || len(a.sceneDraft.Floors) == 0 {
		a.sceneDraft = a.newScene(true)
	}
	if a.workspace.Snap.Grid <= 0 {
		a.workspace.Snap.Grid = .1
	}
	if a.workspace.Snap.Angle <= 0 {
		a.workspace.Snap.Angle = 15
	}
	a.sceneFloor = a.sceneDraft.Floors[0].ID
	a.sceneLookat = Vec3{4, 0, 1}
	a.page = "home"
	a.hiddenFloors = map[string]bool{}
}
func (a *app) reportError(err error) {
	if err != nil {
		if a.manager != nil {
			a.manager.Events.Emit("ui.failed", "error", map[string]any{"message": err.Error(), "page": a.page})
		}
		a.status = err.Error()
		a.showLog = true
	}
}
func (a *app) checkpoint() {
	a.auditConfiguration()
	if a.workspace.Version == 0 || a.workspaceBlocked {
		return
	}
	a.robotPreset.RobotConfig = robotConfig(a.cfg)
	a.workspace.RobotDrafts[a.robotPreset.ID] = clone(a.robotPreset)
	a.workspace.SceneDrafts[a.sceneDraft.ID] = clone(a.sceneDraft)
	a.workspace.Config = clone(a.cfg)
	a.workspace.RobotID = a.robotPreset.ID
	a.workspace.SceneID = a.sceneDraft.ID
	data, err := json.Marshal(a.workspace)
	if err != nil {
		a.reportError(err)
		return
	}
	if string(data) == a.savedWorkspace {
		return
	}
	if err = saveJSON(workspacePath(), a.workspace); err != nil {
		a.reportError(err)
		return
	}
	a.savedWorkspace = string(data)
}
func sameJSON(a, b any) bool {
	x, _ := json.Marshal(a)
	y, _ := json.Marshal(b)
	return string(x) == string(y)
}
func (a *app) robotDirty() bool {
	r := a.robotPreset
	r.RobotConfig = robotConfig(a.cfg)
	saved, ok := a.workspace.Robots[r.ID]
	return !ok || !sameJSON(r, saved)
}
func (a *app) sceneDirty() bool {
	saved, ok := a.workspace.Scenes[a.sceneDraft.ID]
	return !ok || !sameJSON(a.sceneDraft, saved)
}
func (a *app) saveRobot(copy bool) {
	if strings.TrimSpace(a.robotPreset.Name) == "" || len(a.robotPreset.Name) > 160 {
		a.reportError(fmt.Errorf("Enter a preset name / 请输入预设名称"))
		return
	}
	if err := validateConfig(a.cfg, a.catalog); err != nil {
		a.reportError(err)
		return
	}
	if copy {
		a.checkpoint()
		a.robotPreset.ID = newID()
	}
	a.robotPreset.RobotConfig = robotConfig(a.cfg)
	a.workspace.Robots[a.robotPreset.ID] = clone(a.robotPreset)
	a.checkpoint()
}
func (a *app) chooseRobot(id string) {
	a.checkpoint()
	r, ok := a.workspace.RobotDrafts[id]
	if !ok {
		r, ok = a.workspace.Robots[id]
	}
	if !ok {
		return
	}
	candidate := clone(a.cfg)
	r.RobotConfig.apply(&candidate)
	if a.catalog.Robots[candidate.Robot].Continuous && (candidate.Terrain == "stairs" || candidate.Terrain == "rough") {
		candidate.Terrain = "flat"
	}
	if err := validateConfig(candidate, a.catalog); err != nil {
		a.reportError(err)
		return
	}
	a.cfg = candidate
	a.robotPreset = clone(r)
	a.startConfig()
	a.checkpoint()
}
func (a *app) saveScene(copy bool) {
	if err := validateScene(&a.sceneDraft); err != nil {
		a.reportError(err)
		return
	}
	if copy {
		a.checkpoint()
		a.sceneDraft.ID = newID()
	}
	a.workspace.Scenes[a.sceneDraft.ID] = clone(a.sceneDraft)
	a.checkpoint()
}
func (a *app) useScene() {
	if err := validateScene(&a.sceneDraft); err != nil {
		a.reportError(err)
		return
	}
	a.cfg.Scene = clone(&a.sceneDraft)
	a.startConfig()
	a.checkpoint()
}
