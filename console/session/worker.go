package session

import (
	"bufio"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"os"
	"os/exec"
	"path/filepath"
	"sync"
	"syscall"
	"time"
)

type Manager struct {
	prepareFailed  map[string]bool
	verifiedImages map[string]string
	connectMu      sync.Mutex
	Client         *Client
	SessionDir     string
	mu             sync.Mutex
	options        Options
	update         func()
	worker         *exec.Cmd
	workerDone     chan struct{}
	generation     uint64
	log            []string
	status         string
	busy           bool
	adapters       map[string]*exec.Cmd
	containers     map[string]string
	build          *exec.Cmd
}

func NewManager(options Options, onUpdate func()) *Manager {
	return &Manager{options: options, update: onUpdate, Client: NewClient(onUpdate), adapters: map[string]*exec.Cmd{}, containers: map[string]string{}, prepareFailed: map[string]bool{}, verifiedImages: map[string]string{}}
}
func (m *Manager) notify() {
	if m.update != nil {
		m.update()
	}
}
func (m *Manager) message(value string) {
	m.mu.Lock()
	m.status = value
	m.log = append(m.log, value)
	if len(m.log) > 200 {
		m.log = m.log[len(m.log)-200:]
	}
	m.mu.Unlock()
	m.notify()
}
func (m *Manager) Directory() string { m.mu.Lock(); defer m.mu.Unlock(); return m.SessionDir }
func (m *Manager) Status() string    { m.mu.Lock(); defer m.mu.Unlock(); return m.status }
func (m *Manager) Busy() bool        { m.mu.Lock(); defer m.mu.Unlock(); return m.busy }
func (m *Manager) Logs() []string {
	m.mu.Lock()
	defer m.mu.Unlock()
	return append([]string(nil), m.log...)
}
func (m *Manager) watch(reader io.Reader, prefix string) {
	scanner := bufio.NewScanner(reader)
	scanner.Buffer(make([]byte, 4096), 1024*1024)
	for scanner.Scan() {
		m.message(prefix + scanner.Text())
	}
	if err := scanner.Err(); err != nil {
		m.message(prefix + err.Error())
	}
}
func (m *Manager) Start(config map[string]any) error {
	if err := m.Close(); err != nil {
		return err
	}
	if m.options.WorkerPath == "" {
		return errors.New("simple-rbt-sim worker was not found")
	}
	session, err := os.MkdirTemp("", "srbt-")
	if err != nil {
		return err
	}
	output := filepath.Join(m.options.StateDir, "runs", filepath.Base(session))
	if err = os.MkdirAll(filepath.Dir(output), 0700); err != nil {
		os.Remove(session)
		return err
	}
	// Copy caller configuration: private output paths never overwrite UI settings.
	data, err := json.Marshal(config)
	if err != nil {
		os.Remove(session)
		return err
	}
	var coreConfig map[string]any
	if err = json.Unmarshal(data, &coreConfig); err != nil {
		os.Remove(session)
		return err
	}
	coreConfig["output"] = output
	coreConfig["start_immediately"] = false
	coreConfig["source"] = "gui"
	data, err = json.MarshalIndent(coreConfig, "", "  ")
	if err != nil {
		os.Remove(session)
		return err
	}
	profile := filepath.Join(session, "config.json")
	if err = os.WriteFile(profile, data, 0600); err != nil {
		os.Remove(session)
		return err
	}
	arguments := []string{"--session-config", profile, "--session-dir", session}
	if m.options.DataRoot != "" {
		arguments = append(arguments, "--project-root", m.options.DataRoot)
	}
	command := exec.Command(m.options.WorkerPath, arguments...)
	command.SysProcAttr = &syscall.SysProcAttr{Setpgid: true}
	stdout, _ := command.StdoutPipe()
	stderr, _ := command.StderrPipe()
	if err = command.Start(); err != nil {
		os.Remove(profile)
		os.Remove(session)
		return err
	}
	client := m.Client
	subscriptions := []string{"view", "odom", "metadata", "status"}
	sensors, _ := coreConfig["sensors"].(map[string]any)
	rgb, _ := sensors["camera"].(bool)
	depth, _ := sensors["depth"].(bool)
	panorama, _ := sensors["panorama"].(bool)
	if rgb || depth {
		subscriptions = append(subscriptions, "camera")
	} else if panorama {
		subscriptions = append(subscriptions, "panorama")
	}
	client.SetSubscriptions(subscriptions)
	m.mu.Lock()
	m.generation++
	generation := m.generation
	m.worker = command
	m.workerDone = make(chan struct{})
	done := m.workerDone
	m.SessionDir = session
	m.mu.Unlock()
	m.message("Starting C++ core / 正在启动 C++ 核心")
	go m.watch(stdout, "Core: ")
	go m.watch(stderr, "Core: ")
	go func() {
		err := command.Wait()
		close(done)
		m.mu.Lock()
		current := m.generation == generation
		m.mu.Unlock()
		if current {
			client.Close()
			if err != nil {
				m.message("Core stopped: " + err.Error())
			} else {
				m.message("Core stopped / 核心已停止")
			}
		}
	}()
	go func() {
		deadline := time.Now().Add(20 * time.Second)
		for time.Now().Before(deadline) {
			m.mu.Lock()
			current := m.generation == generation
			m.mu.Unlock()
			if !current {
				return
			}
			select {
			case <-done:
				return
			default:
			}
			m.connectMu.Lock()
			m.mu.Lock()
			current = m.generation == generation
			m.mu.Unlock()
			if !current {
				m.connectMu.Unlock()
				return
			}
			err := client.Connect(session)
			m.connectMu.Unlock()
			if err == nil {
				m.message("Core connected / 核心已连接")
				return
			}
			time.Sleep(50 * time.Millisecond)
		}
		m.message("Core connection timed out / 核心连接超时")
	}()
	return nil
}
func (m *Manager) Close() error {
	m.mu.Lock()
	m.generation++
	worker, done, client, session := m.worker, m.workerDone, m.Client, m.SessionDir
	m.worker = nil
	m.workerDone = nil
	m.SessionDir = ""
	build := m.build
	m.build = nil
	m.busy = false
	m.mu.Unlock()
	if client != nil && client.Connected() {
		client.SendCommand([3]float64{})
		client.SendRequest(map[string]any{"op": "shutdown"})
	}
	for _, version := range []string{"ros1", "ros2"} {
		m.StopAdapter(version)
	}
	if build != nil && build.Process != nil {
		syscall.Kill(-build.Process.Pid, syscall.SIGTERM)
	}
	if worker != nil && worker.Process != nil {
		select {
		case <-done:
		case <-time.After(500 * time.Millisecond):
			syscall.Kill(-worker.Process.Pid, syscall.SIGTERM)
		}
		select {
		case <-done:
		case <-time.After(3 * time.Second):
			syscall.Kill(-worker.Process.Pid, syscall.SIGKILL)
			<-done
		}
	}
	if client != nil {
		client.Close()
	}
	if session != "" { // Only the private directory created by this manager is removed.
		for _, name := range []string{"config.json", "control.sock", "data.sock"} {
			os.Remove(filepath.Join(session, name))
		}
		if err := os.Remove(session); err != nil && !os.IsNotExist(err) {
			return fmt.Errorf("session cleanup: %w", err)
		}
	}
	return nil
}
