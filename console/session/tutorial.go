package session

import (
	"bufio"
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"os"
	"os/exec"
	"path/filepath"
	"strings"
	"time"
)

type TutorialStatus struct {
	Algorithm      string    `json:"algorithm"`
	State          string    `json:"state"`
	Poses          uint64    `json:"poses"`
	Lag            *float64  `json:"lag_s"`
	Position       []float64 `json:"estimated_position"`
	InvalidPoses   uint64    `json:"invalid_poses"`
	TrajectoryFull bool      `json:"trajectory_full"`
	Output, Error  string
}

func (m *Manager) TutorialStatus() TutorialStatus {
	m.mu.Lock()
	defer m.mu.Unlock()
	result := m.tutorial
	result.Position = append([]float64(nil), result.Position...)
	return result
}
func TutorialImage(version string) string {
	if version == "ros2" {
		return "simple-rbt-tutorials:humble"
	}
	return "simple-rbt-tutorials:noetic"
}
func (m *Manager) StartTutorial(algorithm, version string) error {
	if algorithm != "fast-lio" && algorithm != "fast-livo2" {
		return errors.New("unknown tutorial algorithm")
	}
	if version != "ros1" && version != "ros2" {
		return errors.New("select Noetic or Humble")
	}
	m.mu.Lock()
	if m.tutorialCancel != nil {
		m.mu.Unlock()
		return errors.New("tutorial is already running")
	}
	directory := m.SessionDir
	if directory == "" {
		m.mu.Unlock()
		return errors.New("start the simulator first")
	}
	ctx, cancel := context.WithCancel(context.Background())
	m.tutorialCancel = cancel
	output := filepath.Join(m.options.StateDir, "tutorials", time.Now().UTC().Format("20060102T150405.000000000"))
	name := "sprobo-tutorial-" + filepath.Base(directory)
	m.tutorial = TutorialStatus{Algorithm: algorithm, State: "preparing", Output: output}
	m.tutorialContainer = name
	m.mu.Unlock()
	m.notify()
	go m.runTutorial(ctx, algorithm, version, directory, output, name)
	return nil
}
func (m *Manager) runTutorial(ctx context.Context, algorithm, version, directory, output, name string) {
	fail := func(err error) {
		if ctx.Err() != nil {
			return
		}
		m.mu.Lock()
		if m.tutorial.Output == output && m.tutorial.State != "stopped" {
			m.tutorial.State = "failed"
			m.tutorial.Error = err.Error()
			m.tutorialCancel = nil
		}
		m.mu.Unlock()
		m.message("Tutorial / 教程: " + err.Error())
	}
	image := TutorialImage(version)
	data, err := exec.CommandContext(ctx, "docker", "image", "inspect", "--format", `{{ index .Config.Labels "org.simple-rbt.tutorial.contract" }}`, image).CombinedOutput()
	if err != nil || strings.TrimSpace(string(data)) != "1" {
		fail(fmt.Errorf("prepare the optional tutorial image / 请先准备可选教程镜像: %s", image))
		return
	}
	tool := filepath.Join(filepath.Dir(m.options.WorkerPath), "simple-rbt-tutorial-config")
	data, err = exec.CommandContext(ctx, tool, "--session-dir", directory, "--data-root", m.options.DataRoot, "--output", output, "--algorithm", algorithm, "--ros", version).CombinedOutput()
	if err != nil {
		fail(fmt.Errorf("%s: %w", strings.TrimSpace(string(data)), err))
		return
	}
	if ctx.Err() != nil {
		return
	}
	arguments := []string{"create", "--pull", "never", "--name", name, "--network", "host", "--user", fmt.Sprintf("%d:%d", os.Getuid(), os.Getgid()), "--mount", "type=bind,source=" + output + ",target=/run"}
	keys := []string{"ROS_MASTER_URI", "ROS_IP"}
	if version == "ros2" {
		keys = []string{"ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY"}
	}
	for _, key := range keys {
		if value := os.Getenv(key); value != "" {
			arguments = append(arguments, "--env", key+"="+value)
		}
	}
	arguments = append(arguments, image, algorithm)
	// Let create finish even if the UI closes; then remove this exact container.
	createCtx, done := context.WithTimeout(context.Background(), 30*time.Second)
	data, err = exec.CommandContext(createCtx, "docker", arguments...).CombinedOutput()
	done()
	if err != nil {
		fail(fmt.Errorf("%s: %w", strings.TrimSpace(string(data)), err))
		return
	}
	defer exec.Command("docker", "rm", "--force", name).Run()
	if ctx.Err() != nil {
		return
	}
	command := exec.CommandContext(ctx, "docker", "start", "--attach", name)
	stdout, _ := command.StdoutPipe()
	stderr, _ := command.StderrPipe()
	if err = command.Start(); err != nil {
		fail(err)
		return
	}
	m.mu.Lock()
	if m.tutorial.Output == output && m.tutorial.State != "stopped" {
		m.tutorial.State = "initializing"
	}
	m.mu.Unlock()
	m.notify()
	go m.watchTutorial(stdout, output)
	go m.watch(stderr, "tutorial: ")
	err = command.Wait()
	if ctx.Err() == nil && err != nil {
		fail(err)
		return
	}
	m.mu.Lock()
	if m.tutorial.Output == output {
		m.tutorial.State = "stopped"
		m.tutorialCancel = nil
	}
	m.mu.Unlock()
	m.notify()
}
func (m *Manager) watchTutorial(reader io.Reader, output string) {
	scanner := bufio.NewScanner(reader)
	scanner.Buffer(make([]byte, 4096), 1024*1024)
	for scanner.Scan() {
		line := scanner.Text()
		payload, ok := strings.CutPrefix(line, "SPROBO_TUTORIAL ")
		if !ok {
			m.message("tutorial: " + line)
			continue
		}
		var value TutorialStatus
		if json.Unmarshal([]byte(payload), &value) != nil {
			continue
		}
		m.mu.Lock()
		changed := m.tutorial.State != value.State
		if m.tutorial.Output == output && m.tutorial.State != "stopped" {
			value.Output = output
			m.tutorial = value
		}
		m.mu.Unlock()
		if changed {
			m.Events.Event("tutorial.state", map[string]any{"algorithm": value.Algorithm, "state": value.State})
		}
		m.notify()
	}
}
func (m *Manager) StopTutorial() {
	m.mu.Lock()
	cancel, name := m.tutorialCancel, m.tutorialContainer
	m.tutorialCancel = nil
	m.tutorialContainer = ""
	if cancel != nil {
		m.tutorial.State = "stopped"
	}
	m.mu.Unlock()
	if cancel == nil {
		return
	}
	ctx, done := context.WithTimeout(context.Background(), 7*time.Second)
	_ = exec.CommandContext(ctx, "docker", "stop", "--time", "3", name).Run()
	done()
	cancel()
	m.notify()
}

func (m *Manager) TutorialGuide() string {
	return filepath.Join(m.options.DataRoot, "docs/tutorials.md")
}
func (m *Manager) TutorialProfile(robot string) string {
	return filepath.Join(m.options.DataRoot, "assets/tutorials", robot+".session.json")
}
