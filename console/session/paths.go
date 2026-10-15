package session

import (
	"os"
	"os/exec"
	"path/filepath"
)

type Options struct{ WorkerPath, DataRoot, AdapterContext, StateDir string }

func exists(path string) bool { _, err := os.Stat(path); return err == nil }
func DefaultOptions() Options {
	result := Options{}
	if worker, err := exec.LookPath("simple-rbt-sim"); err == nil {
		result.WorkerPath = worker
	}
	executable, _ := os.Executable()
	working, _ := os.Getwd()
	roots := []string{"/usr/share/simple-rbt"}
	for _, start := range []string{working, filepath.Dir(executable)} {
		for path := start; path != "/" && path != "."; path = filepath.Dir(path) {
			roots = append(roots, path)
		}
	}
	for _, root := range roots {
		if result.DataRoot == "" && exists(filepath.Join(root, "assets/catalog.json")) {
			result.DataRoot = root
		}
		if result.WorkerPath == "" && exists(filepath.Join(root, "build/desktop/simple-rbt-sim")) {
			result.WorkerPath = filepath.Join(root, "build/desktop/simple-rbt-sim")
		}
		if result.AdapterContext == "" {
			if exists(filepath.Join(root, "ros-adapters/deployment/ros1/Dockerfile")) {
				result.AdapterContext = filepath.Join(root, "ros-adapters")
			} else if exists(filepath.Join(root, "deployment/ros1/Dockerfile")) {
				result.AdapterContext = root
			}
		}
	}
	result.StateDir = os.Getenv("XDG_STATE_HOME")
	if result.StateDir == "" {
		home, _ := os.UserHomeDir()
		result.StateDir = filepath.Join(home, ".local/state")
	}
	result.StateDir = filepath.Join(result.StateDir, "simple-rbt")
	if result.WorkerPath == "" {
		result.WorkerPath = "/usr/bin/simple-rbt-sim"
	}
	if result.DataRoot == "" {
		result.DataRoot = "/usr/share/simple-rbt"
	}
	if result.AdapterContext == "" {
		result.AdapterContext = filepath.Join(result.DataRoot, "ros-adapters")
	}
	return result
}
