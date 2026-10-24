package main

import (
	"sync"
	"sync/atomic"
	"time"

	"github.com/egoist/mygo"
	"github.com/egoist/mygo/ui"
	"simple-rbt/console/session"
)

type observerInfo struct {
	Position      Vec3    `json:"position"`
	Lookat        Vec3    `json:"lookat"`
	Up            Vec3    `json:"up"`
	Fov           float64 `json:"vertical_fov_deg"`
	Width, Height int
}
type metadata struct {
	JointControls []jointAxis     `json:"joint_controls"`
	MountParents  map[string]Pose `json:"mount_parents"`
	Logging       map[string]any  `json:"logging"`
	State         string          `json:"session_state"`
	Source        string          `json:"source"`
	Base          Pose            `json:"base_pose"`
	Mounts        map[string]Pose `json:"mounts"`
	BodyImu       Pose            `json:"body_imu_mount"`
	MidImu        Pose            `json:"mid360_internal_imu"`
	Observer      observerInfo    `json:"observer"`
	Connections   map[string]int  `json:"connections"`
	CommandStatus struct {
		State   string   `json:"state"`
		Age     *float64 `json:"age_s"`
		Timeout float64  `json:"timeout_s"`
	} `json:"command_status"`
	Stamp     uint64 `json:"stamp_ns"`
	Applied   Vec3   `json:"applied_command"`
	Resources struct {
		CPU float64 `json:"cpu_percent_one_core"`
		RAM float64 `json:"rss_mib"`
		RTF float64 `json:"real_time_factor"`
	} `json:"resources"`
}
type app struct {
	armCommand              *session.JointCommand
	jointState              session.JointState
	externalCamera          *ui.Bitmap
	externalStamp           uint64
	auditPrevious           map[string]any
	diagnostics             diagnosticsView
	sceneTemplateEdit       bool
	sceneNewKind            string
	sceneRevision           int
	sceneHistory            sceneHistory
	sceneField, alignTarget string
	scenePreview            sceneRaster
	viewError               string
	page                    string
	workspace               Workspace
	robotPreset             RobotPreset
	sceneDraft              Scene
	sceneFloor, sceneObject string
	hiddenFloors            map[string]bool
	sceneLookat             Vec3
	savedWorkspace          string
	workspaceBlocked        bool
	viewLoading             bool
	pageViews               map[string]viewport

	cfg                                          Config
	preferences                                  Preferences
	catalog                                      Catalog
	meta                                         metadata
	pending                                      map[string]Pose
	manager                                      *session.Manager
	win                                          *mygo.Window
	updates                                      chan struct{}
	done                                         chan struct{}
	closed                                       atomic.Bool
	exitCode                                     atomic.Int32
	selected, tool, space, viewMode, sourceLabel string
	stage                                        string
	status                                       string
	managerStatus                                string
	showLog                                      bool
	observer, camera                             *ui.Bitmap
	observerSize, cameraSize                     [2]int
	viewStamp, cameraStamp                       uint64
	viewport                                     viewport
	driveKeys                                    map[ui.Key]bool
	driveHandles                                 [3]ui.Handle
	manualScale                                  float64
	pad                                          joystick
	command                                      Vec3
	manualHeartbeat                              bool
	commandMu                                    sync.Mutex
	restartMu                                    sync.Mutex
	restartLatest                                atomic.Uint64
	restartPending                               bool
	generation                                   uint64
	loading                                      bool
	preparing                                    bool
	adapterStarted                               map[string]bool
	options                                      session.Options
	angular                                      Vec3
	velocity                                     Vec3
	odomStamp                                    uint64
	preflight                                    session.DependencyReport
	checking, initialized, showPreflight         bool
	checkGeneration                              uint64
	checkCancel                                  func()
	adaptersVerified                             map[string]bool
}

func newApp(options session.Options, cfg Config) *app {
	setSceneWorker(options.WorkerPath)
	if cfg.Mounts == nil {
		cfg.Mounts = map[string]Pose{}
	}
	if cfg.Adapters == nil {
		cfg.Adapters = map[string]bool{}
	}
	a := &app{manualScale: .5, cfg: cfg, pending: map[string]Pose{}, selected: "mid360", tool: "Move / 平移", space: "Base / 机身", viewMode: "Both / 双视角", sourceLabel: "Manual / 手动", stage: "configure", updates: make(chan struct{}, 1), done: make(chan struct{}), adapterStarted: map[string]bool{}, adaptersVerified: map[string]bool{}, options: options}
	a.preferences = loadPreferences()
	a.viewport = viewport{Azimuth: -135, Elevation: 25, Distance: 1.8}
	a.manager = session.NewManager(options, func() {
		select {
		case a.updates <- struct{}{}:
		default:
		}
	})
	return a
}
func (a *app) startConfig() {
	a.checkpoint()
	if err := validateConfig(a.cfg, a.catalog); err != nil {
		a.status = err.Error()
		a.showLog = true
		return
	}
	a.preparing = false
	a.generation++
	generation := a.generation
	a.restartLatest.Store(generation)
	a.restartPending = true
	a.loading = true
	a.status = "Loading model / 正在加载模型"
	a.managerStatus = ""
	a.meta = metadata{}
	a.pending = map[string]Pose{}
	a.viewLoading = true
	a.viewError = ""
	a.camera = nil
	a.externalCamera = nil
	a.externalStamp = 0
	a.jointState = session.JointState{}
	a.viewStamp = 0
	a.cameraStamp = 0
	a.odomStamp = 0
	a.velocity = Vec3{}
	a.angular = Vec3{}
	a.adapterStarted = map[string]bool{}
	a.stopManual()
	configuration := asMap(a.cfg)
	go func() {
		time.Sleep(180 * time.Millisecond)
		if a.closed.Load() {
			return
		}
		a.win.Update(func() {
			if generation != a.generation {
				return
			}
			go func() {
				a.restartMu.Lock()
				defer a.restartMu.Unlock()
				if a.closed.Load() || a.restartLatest.Load() != generation {
					return
				}
				err := a.manager.Start(configuration)
				if a.closed.Load() || a.restartLatest.Load() != generation {
					_ = a.manager.Close()
					return
				}
				a.win.Update(func() {
					if a.closed.Load() || generation != a.generation {
						return
					}
					a.restartPending = false
					if err != nil {
						a.loading = false
						a.viewError = err.Error()
						a.status = err.Error()
						a.showLog = true
					}
				})
			}()
		})
	}()
}
func (a *app) requestView() {
	if a.page == "scene" {
		return
	}
	if a.restartPending || !a.manager.Client.Connected() {
		return
	}
	lookat := a.meta.Base.Position
	if a.catalog.Robots[a.cfg.Robot].JointControl {
		lookat[2] += .45
	}
	request := a.viewport.request(lookat)
	request["enabled"] = a.stage == "configure" || a.viewMode != "First / 第一视角" || !a.cameraEnabled()
	if a.stage != "configure" && a.cfg.Sensors.ExternalCamera && a.viewMode != "Observer / 自由观察" {
		request["enabled"] = false
	}
	_ = a.manager.Client.SendRequest(request)
}
func (a *app) pump() {
	go func() {
		for {
			select {
			case <-a.done:
				return
			case <-a.updates:
				if a.closed.Load() {
					return
				}
				applied := make(chan struct{}, 1)
				a.win.Update(func() {
					if status := a.manager.Status(); status != a.managerStatus {
						a.managerStatus = status
						a.status = status
					}
					a.consume()
					applied <- struct{}{}
				})
				select {
				case <-a.done:
					return
				case <-applied:
				}
			}
		}
	}()
	go func() {
		count := 0
		tick := time.NewTicker(100 * time.Millisecond)
		defer tick.Stop()
		for {
			select {
			case <-a.done:
				return
			case <-tick.C:
				a.sendManualHeartbeat()
				count++
				if count%10 == 0 {
					a.win.Update(func() {
						if !a.closed.Load() {
							a.checkpoint()
							a.refreshDiagnostics()
						}
					})
				}
			}
		}
	}()
}
func (a *app) applyMetadata(value metadata) {
	if value.State == "" {
		return
	}
	first := a.meta.State == ""
	a.meta = value
	a.loading = false
	for name, accepted := range value.Mounts {
		if wanted, ok := a.pending[name]; ok {
			if !samePose(wanted, accepted) {
				continue
			}
			delete(a.pending, name)
		}
		a.cfg.Mounts[name] = accepted
	}
	a.sourceLabel = sourceLabel(value.Source)
	a.commandMu.Lock()
	a.manualHeartbeat = a.manualMode()
	if !a.manualHeartbeat {
		clear(a.driveKeys)
		a.command = Vec3{}
		a.pad = joystick{}
	}
	a.commandMu.Unlock()
	if first {
		a.requestView()
	}
}
func (a *app) editMount(name string, pose Pose) {
	if a.stage != "configure" || a.loading {
		return
	}
	pose.Quaternion = qnormalize(pose.Quaternion)
	a.cfg.Mounts[name] = pose
	a.pending[name] = pose
	request := asMap(pose)
	request["op"] = "mount"
	request["sensor"] = name
	_ = a.manager.Client.SendRequest(request)
}
func (a *app) stopManual() {
	clear(a.driveKeys)
	a.pad.Held = false
	a.pad.Position = point{}
	a.pad.Lateral = 0
	a.commandMu.Lock()
	a.armCommand = nil
	a.command = Vec3{}
	_ = a.manager.Client.SendCommand([3]float64{})
	a.commandMu.Unlock()
}
func (a *app) close() {
	if a.closed.Swap(true) {
		return
	}
	a.checkpoint()
	a.restartLatest.Add(1)
	if a.checkCancel != nil {
		a.checkCancel()
	}
	a.stopManual()
	close(a.done)
	a.restartMu.Lock()
	defer a.restartMu.Unlock()
	_ = a.manager.Close()
	a.manager.Events.Close()
}
