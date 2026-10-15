package session

import (
	"context"
	"encoding/json"
	"fmt"
	"os"
	"os/exec"
	"runtime"
	"strings"
	"time"
)

const Product = "SpRobo"
const Framework = "simple_rbt"

var Version = "0.2.0" // Build injects the canonical packaging/VERSION value.
const MuJoCoVersion = "3.15.0"
const MuJoCoVersionNumber = 3015000

type DependencyCheck struct {
	ID       string         `json:"id"`
	Required bool           `json:"required"`
	Status   string         `json:"status"`
	Message  string         `json:"message"`
	Details  map[string]any `json:"details,omitempty"`
}
type RuntimeInfo struct {
	Product      string            `json:"product"`
	Framework    string            `json:"framework"`
	Launcher     string            `json:"launcher"`
	Version      string            `json:"version"`
	MuJoCo       string            `json:"mujoco_version"`
	Header       int               `json:"mujoco_header_version"`
	Runtime      int               `json:"mujoco_runtime_version"`
	AssetsValid  bool              `json:"assets_valid"`
	AssetsError  string            `json:"assets_error"`
	CUDACompiled bool              `json:"cuda_compiled"`
	Controller   string            `json:"controller_backend"`
	Robots       []json.RawMessage `json:"robots"`
}
type DependencyReport struct {
	Product   string            `json:"product"`
	Framework string            `json:"framework"`
	Version   string            `json:"version"`
	Ready     bool              `json:"ready"`
	Checks    []DependencyCheck `json:"checks"`
	Core      *RuntimeInfo      `json:"core,omitempty"`
}

func (r DependencyReport) ExitCode() int {
	if r.Ready {
		return 0
	}
	return 1
}
func (r DependencyReport) ROSReady(version string) bool {
	for _, check := range r.Checks {
		if check.ID == version {
			return check.Status == "pass"
		}
	}
	return false
}

type commandRunner func(context.Context, string, ...string) ([]byte, error)
type checker struct {
	osRelease []byte
	arch      string
	run       commandRunner
}

func runCommand(ctx context.Context, name string, arguments ...string) ([]byte, error) {
	return exec.CommandContext(ctx, name, arguments...).CombinedOutput()
}
func CheckDependencies(ctx context.Context, options Options, selectedROS map[string]bool) DependencyReport {
	data, err := os.ReadFile("/etc/os-release")
	if err != nil {
		data = nil
	}
	return (checker{data, runtime.GOARCH, runCommand}).check(ctx, options, selectedROS)
}
func (c checker) check(ctx context.Context, options Options, selectedROS map[string]bool) DependencyReport {
	report := DependencyReport{Product: Product, Framework: Framework, Version: Version, Ready: true}
	appendCheck := func(check DependencyCheck) {
		if check.Required && check.Status != "pass" {
			report.Ready = false
		}
		report.Checks = append(report.Checks, check)
	}
	platform := releaseFields(c.osRelease)
	validPlatform := platform["ID"] == "ubuntu" && platform["VERSION_ID"] == "22.04" && c.arch == "amd64"
	message := "Ubuntu 22.04 amd64 supported / 平台受支持"
	if !validPlatform {
		message = "Requires Ubuntu 22.04 amd64 / 当前仅支持 Ubuntu 22.04 amd64"
	}
	appendCheck(DependencyCheck{"ubuntu", true, checkStatus(validPlatform), message, map[string]any{"id": platform["ID"], "version_id": platform["VERSION_ID"], "architecture": c.arch}})
	info, err := c.inspectCore(ctx, options)
	if err != nil {
		appendCheck(DependencyCheck{"core", true, "fail", err.Error(), map[string]any{"executable": options.WorkerPath, "data_root": options.DataRoot}})
	} else {
		report.Core = &info
		appendCheck(DependencyCheck{"core", true, "pass", "simple_rbt core and pinned MuJoCo are compatible / 框架核心与固定 MuJoCo 版本兼容", map[string]any{"executable": options.WorkerPath, "mujoco": info.MuJoCo, "controller_backend": info.Controller}})
		appendCheck(DependencyCheck{"assets", true, checkStatus(info.AssetsValid), assetsMessage(info), map[string]any{"data_root": options.DataRoot, "robots": len(info.Robots)}})
	}
	for _, version := range []string{"ros1", "ros2"} {
		if !selectedROS[version] {
			appendCheck(DependencyCheck{version, false, "skipped", "Optional; not selected / 可选，未选择", nil})
			continue
		}
		appendCheck(c.inspectROS(ctx, version))
	}
	return report
}
func (c checker) inspectCore(ctx context.Context, options Options) (RuntimeInfo, error) {
	var info RuntimeInfo
	file, err := os.Stat(options.WorkerPath)
	if err != nil || !file.Mode().IsRegular() || file.Mode().Perm()&0111 == 0 {
		return info, fmt.Errorf("simple-rbt-sim is missing or not executable / 核心不存在或无法执行: %s", options.WorkerPath)
	}
	deadline, cancel := context.WithTimeout(ctx, 20*time.Second)
	defer cancel()
	data, err := c.run(deadline, options.WorkerPath, "--runtime-info", "--project-root", options.DataRoot)
	if err != nil {
		return info, fmt.Errorf("core dependency check failed / 核心依赖检查失败: %w: %s", err, strings.TrimSpace(string(data)))
	}
	if err = json.Unmarshal(data, &info); err != nil {
		return info, fmt.Errorf("invalid core runtime-info / 核心版本信息无效: %w", err)
	}
	framework := info.Framework
	if framework == "" {
		framework = info.Product
	}
	if framework != Framework || info.Version != Version {
		return info, fmt.Errorf("requires %s %s core; found %s %s / 核心版本不兼容", Framework, Version, framework, info.Version)
	}
	if info.MuJoCo != MuJoCoVersion || info.Header != MuJoCoVersionNumber || info.Runtime != info.Header {
		return info, fmt.Errorf("requires pinned MuJoCo %s, header/runtime %d; found %s/%d/%d / MuJoCo 版本不兼容", MuJoCoVersion, MuJoCoVersionNumber, info.MuJoCo, info.Header, info.Runtime)
	}
	if len(info.Robots) == 0 {
		return info, fmt.Errorf("core robot registry is empty / 核心机器人清单为空")
	}
	return info, nil
}
func releaseFields(data []byte) map[string]string {
	values := map[string]string{}
	for _, line := range strings.Split(string(data), "\n") {
		key, value, ok := strings.Cut(line, "=")
		if ok {
			values[key] = strings.Trim(value, "\"'")
		}
	}
	return values
}
func checkStatus(ok bool) string {
	if ok {
		return "pass"
	}
	return "fail"
}
func assetsMessage(info RuntimeInfo) string {
	if info.AssetsValid {
		return "Models and policies verified / 模型与控制权重已校验"
	}
	return "Assets incompatible / 资源不兼容: " + info.AssetsError
}
