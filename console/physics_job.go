package main

import (
	"context"
	"encoding/json"
	"fmt"
	"github.com/egoist/mygo"
	"github.com/egoist/mygo/ui"
	"os"
	"os/exec"
	"path/filepath"
	"simple-rbt/console/session"
	"strings"
	"time"
)

type physicsJob struct {
	Format      string `json:"format"`
	Assets      string `json:"assets_root"`
	Config      string `json:"session_config"`
	Python      string `json:"python"`
	Reference   string `json:"reference"`
	Checkpoint  string `json:"checkpoint"`
	Output      string `json:"output_root"`
	Device      string `json:"device"`
	Worlds      int    `json:"worlds"`
	Steps       int    `json:"policy_steps"`
	TrainWorlds int    `json:"training_worlds"`
	Iterations  int    `json:"training_iterations"`
	Seconds     int    `json:"wall_limit_s"`
}

func loadPhysicsJob(path string) (physicsJob, error) {
	var job physicsJob
	if err := loadJSON(path, &job); err != nil {
		return job, err
	}
	if job.Format != "sprobo.physics-job.v1" || job.Output == "" || job.Assets == "" || job.Config == "" || job.Worlds < 2 || job.Worlds > 64 || job.Steps < 200 || job.Steps > 1000 || job.TrainWorlds < 1 || job.TrainWorlds > 64 || job.Iterations < 8 || job.Iterations > 64 || job.Seconds < 1 || job.Seconds > 600 {
		return job, fmt.Errorf("Invalid physics job; bounded jobs require 2–64 worlds, 200–1000 steps, 8–64 training iterations, ≤600 s")
	}
	return job, nil
}

func runPhysicsJob(ctx context.Context, path, backend, task string, options session.Options) (string, error) {
	job, err := loadPhysicsJob(path)
	if err != nil {
		return "", err
	}
	if backend != "mujoco" && backend != "warp" {
		return "", fmt.Errorf("physics backend must be mujoco or warp")
	}
	if task != "batch" && task != "train-smoke" || backend == "mujoco" && task != "batch" {
		return "", fmt.Errorf("train-smoke requires Warp")
	}
	if err = os.MkdirAll(job.Output, 0700); err != nil {
		return "", err
	}
	output, err := os.MkdirTemp(job.Output, "run-")
	if err != nil {
		return "", err
	}
	fmt.Println("Physics job / 物理任务:", output)
	ctx, cancel := context.WithTimeout(ctx, time.Duration(job.Seconds)*time.Second)
	defer cancel()
	log, err := os.Create(filepath.Join(output, "process.log"))
	if err != nil {
		return output, err
	}
	defer log.Close()
	exporter := filepath.Join(filepath.Dir(options.WorkerPath), "simple-rbt-physics-contract")
	contract := filepath.Join(output, "shared")
	command := exec.CommandContext(ctx, exporter, job.Assets, job.Config, contract)
	command.Stdout = log
	command.Stderr = log
	if err = command.Run(); err != nil {
		return output, fmt.Errorf("contract export failed: %w; %s", err, output)
	}
	if backend == "mujoco" {
		return output, nil
	}
	adapter := filepath.Join(options.AdapterContext, "physics/job.py")
	if _, err = os.Stat(adapter); err != nil {
		adapter = filepath.Join(options.DataRoot, "physics/job.py")
	}
	command = exec.CommandContext(ctx, job.Python, "-u", adapter, path, "--contract", filepath.Join(contract, "contract.json"), "--task", task, "--output", filepath.Join(output, task))
	command.Env = append(os.Environ(), "OMP_NUM_THREADS=2", "OPENBLAS_NUM_THREADS=2")
	command.Stdout = log
	command.Stderr = log
	if err = command.Run(); err != nil {
		return output, fmt.Errorf("Warp task failed: %w; inspect %s", err, output)
	}
	return output, nil
}

func physicsJobMain(path, backend, task string, headless bool, options session.Options) error {
	if _, err := loadPhysicsJob(path); err != nil {
		return err
	}
	if headless {
		out, err := runPhysicsJob(context.Background(), path, backend, task, options)
		_ = json.NewEncoder(os.Stdout).Encode(map[string]any{"output": out, "passed": err == nil, "physics_backend": backend, "task": task})
		return err
	}
	prefs := loadPreferences()
	status := "Ready / 就绪"
	result := ""
	actual := ""
	busy := false
	var win *mygo.Window
	var cancel context.CancelFunc
	tr := func(en, zh string) string {
		if prefs.Language == "zh-CN" {
			return zh
		}
		return en
	}
	mygo.App.SetName("SpRobo")
	mygo.App.SetVersion(session.Version)
	mygo.App.WhenReady(func() {
		win = mygo.NewWindow(mygo.WindowOptions{Title: tr("SpRobo · Physics and training", "SpRobo · 物理与训练"), Width: 940, Height: 580, Content: ui.View(func(c *ui.Context) {
			c.SetTheme(themeFor(prefs.Theme))
			ui.Column(c).Padding(24).Gap(16).FillWidth().Children(func() {
				ui.Text(c, tr("Physics and training", "物理与训练")).FontSize(24).Bold()
				ui.Text(c, filepath.Base(path)).TextColor(c.Theme().TextMuted)
				ui.Row(c).Gap(16).Children(func() {
					ui.Select(c.Key("physics-choice"), &backend, []string{"mujoco", "warp"}).Width(230).Disabled(busy)
					ui.Select(c.Key("physics-task"), &task, []string{"batch", "train-smoke"}).Width(230).Disabled(busy)
				})
				ui.Text(c, tr("MuJoCo: native C++ CPU · Warp: independent CUDA batch process", "MuJoCo：原生 C++ CPU · Warp：独立 CUDA 批量进程"))
				ui.Text(c, tr("Warp jobs are headless. Sensors and ROS remain in the native interactive session.", "Warp 任务为无头批量运行。传感器和 ROS 使用原生交互会话。")).TextColor(c.Theme().TextMuted)
				ui.Button(c, tr("Run bounded task", "运行有界任务")).Disabled(busy).OnClick(func() {
					busy = true
					status = tr("Running", "运行中")
					ctx, stop := context.WithCancel(context.Background())
					cancel = stop
					go func() {
						out, err := runPhysicsJob(ctx, path, backend, task, options)
						var report map[string]any
						reportPath := filepath.Join(out, task, "report.json")
						if backend == "mujoco" {
							reportPath = filepath.Join(out, "shared", "report.json")
						}
						_ = loadJSON(reportPath, &report)
						win.Update(func() {
							busy = false
							result = out
							values := []string{}
							for _, key := range []string{"physics_backend", "physics_device", "version", "mujoco_version", "warp_version", "mjlab_version"} {
								if value, ok := report[key]; ok {
									values = append(values, fmt.Sprint(value))
								}
							}
							actual = strings.Join(values, " · ")
							if err != nil {
								status = err.Error()
							} else {
								status = tr("Passed · evidence saved", "通过 · 已保存证据")
							}
						})
					}()
				})
				ui.Text(c, status).FontSize(16)
				ui.Text(c, actual).FontSize(16)
				ui.Text(c, result).TextColor(c.Theme().TextMuted)
			})
		})})
		win.OnClose(func(*mygo.CloseEvent) {
			if cancel != nil {
				cancel()
			}
		})
	})
	mygo.App.Run()
	return nil
}
