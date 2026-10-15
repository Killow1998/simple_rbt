package session

import (
	"context"
	"errors"
	"fmt"
	"os"
	"os/exec"
	"path/filepath"
	"strings"
	"syscall"
	"time"
)

func adapterImage(version string) (string, error) {
	switch version {
	case "ros1":
		return "simple-rbt-ros1:noetic", nil
	case "ros2":
		return "simple-rbt-ros2:humble", nil
	}
	return "", errors.New("adapter must be ros1 or ros2")
}

// AdapterReady also rejects a failed preparation even if an older tag exists.
func (m *Manager) AdapterReady(version string) bool {
	image, err := adapterImage(version)
	if err != nil {
		return false
	}
	m.mu.Lock()
	failed := m.prepareFailed[version]
	verified := m.verifiedImages[version]
	m.mu.Unlock()
	if failed {
		return false
	}
	ctx, cancel := context.WithTimeout(context.Background(), 15*time.Second)
	defer cancel()
	data, err := runCommand(ctx, "docker", "image", "inspect", "--format", "{{.Id}}", image)
	if err != nil {
		return false
	}
	if verified != "" && verified == strings.TrimSpace(string(data)) {
		return true
	}
	check := CheckROSAdapter(ctx, version)
	if check.Status != "pass" {
		m.message(check.Message)
		return false
	}
	m.RememberAdapterCheck(check)
	return true
}
func (m *Manager) RememberAdapterCheck(check DependencyCheck) {
	if check.Status != "pass" || (check.ID != "ros1" && check.ID != "ros2") {
		return
	}
	id, _ := check.Details["image_id"].(string)
	if id == "" {
		return
	}
	m.mu.Lock()
	m.verifiedImages[check.ID] = id
	m.mu.Unlock()
}
func (m *Manager) PrepareAdapter(version string) error {
	image, err := adapterImage(version)
	if err != nil {
		return err
	}
	if !exists(filepath.Join(m.options.AdapterContext, "deployment", version, "Dockerfile")) {
		return errors.New("installed ROS adapter build context was not found")
	}
	m.mu.Lock()
	if m.busy {
		m.mu.Unlock()
		return errors.New("adapter preparation is already running")
	}
	m.busy = true
	m.prepareFailed[version] = false
	m.mu.Unlock()
	context, err := adapterContext(m.options.AdapterContext, version)
	if err != nil {
		m.mu.Lock()
		m.busy = false
		m.prepareFailed[version] = true
		m.mu.Unlock()
		return err
	}
	// Standard Docker inherits configured proxy/registry settings. No private SDK.
	command := exec.Command("docker", "build", "--pull=false", "-f", filepath.Join(context, "deployment", version, "Dockerfile"), "-t", image, context)
	command.SysProcAttr = &syscall.SysProcAttr{Setpgid: true}
	stdout, _ := command.StdoutPipe()
	stderr, _ := command.StderrPipe()
	if err = command.Start(); err != nil {
		m.mu.Lock()
		m.busy = false
		m.prepareFailed[version] = true
		m.mu.Unlock()
		os.RemoveAll(context)
		return err
	}
	m.mu.Lock()
	m.build = command
	m.mu.Unlock()
	m.message("Preparing " + version + " / 正在准备 ROS 镜像")
	go m.watch(stdout, version+": ")
	go m.watch(stderr, version+": ")
	go func() {
		err := command.Wait()
		os.RemoveAll(context)
		m.mu.Lock()
		if m.build == command {
			m.prepareFailed[version] = err != nil
			m.build = nil
			m.busy = false
		}
		m.mu.Unlock()
		if err != nil {
			m.message("Adapter preparation failed: " + err.Error())
		} else {
			m.message(version + " image prepared / 镜像准备完成")
		}
	}()
	return nil
}
func (m *Manager) StartAdapter(version string) error {
	image, err := adapterImage(version)
	if err != nil {
		return err
	}
	m.mu.Lock()
	session := m.SessionDir
	_, running := m.adapters[version]
	m.mu.Unlock()
	if running {
		return nil
	}
	if session == "" {
		return errors.New("start a core session before its ROS adapter")
	}
	if !m.AdapterReady(version) {
		return fmt.Errorf("%s is unavailable or has an incompatible ROS installation; prepare the ROS image first", image)
	}
	name := "simple-rbt-" + version + "-" + filepath.Base(session)
	arguments := []string{"run", "--rm", "--pull", "never", "--name", name, "--network", "host", "--mount", "type=bind,source=" + session + ",target=/session"}
	environment := []string{"ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY"}
	if version == "ros1" {
		environment = []string{"ROS_MASTER_URI", "ROS_IP", "SIMPLE_RBT_START_MASTER"}
	}
	for _, key := range environment {
		if value := os.Getenv(key); value != "" {
			arguments = append(arguments, "--env", key+"="+value)
		}
	}
	arguments = append(arguments, image, "--session-dir", "/session")
	command := exec.Command("docker", arguments...)
	command.SysProcAttr = &syscall.SysProcAttr{Setpgid: true}
	stdout, _ := command.StdoutPipe()
	stderr, _ := command.StderrPipe()
	if err = command.Start(); err != nil {
		return err
	}
	m.mu.Lock()
	m.adapters[version] = command
	m.containers[version] = name
	m.mu.Unlock()
	m.message("Starting " + version + " / 正在启动 ROS 节点")
	go m.watch(stdout, version+": ")
	go m.watch(stderr, version+": ")
	go func() {
		err := command.Wait()
		m.mu.Lock()
		if m.adapters[version] == command {
			delete(m.adapters, version)
			delete(m.containers, version)
		}
		m.mu.Unlock()
		if err != nil {
			m.message(version + " adapter stopped: " + err.Error())
		} else {
			m.message(version + " adapter stopped / ROS 节点已停止")
		}
	}()
	return nil
}
func (m *Manager) StopAdapter(version string) error {
	if _, err := adapterImage(version); err != nil {
		return err
	}
	m.mu.Lock()
	command, name := m.adapters[version], m.containers[version]
	delete(m.adapters, version)
	delete(m.containers, version)
	m.mu.Unlock()
	if name != "" {
		exec.Command("docker", "stop", "--time", "3", name).Run()
	}
	if command != nil && command.Process != nil {
		syscall.Kill(-command.Process.Pid, syscall.SIGTERM)
	}
	return nil
}
