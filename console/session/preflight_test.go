package session

import (
	"context"
	"encoding/json"
	"errors"
	"os"
	"path/filepath"
	"strings"
	"testing"
)

func validRuntime() RuntimeInfo {
	return RuntimeInfo{Product: Framework, Framework: Framework, Launcher: Product, Version: Version, MuJoCo: MuJoCoVersion, Header: MuJoCoVersionNumber, Runtime: MuJoCoVersionNumber, AssetsValid: true, Controller: "cpu", Robots: []json.RawMessage{json.RawMessage(`{"name":"ranger_mini"}`)}}
}
func fixtureChecker(t *testing.T, info RuntimeInfo, docker func([]string) ([]byte, error)) (checker, Options) {
	t.Helper()
	worker := filepath.Join(t.TempDir(), "simple-rbt-sim")
	if err := os.WriteFile(worker, []byte("#!/bin/sh\nexit 0\n"), 0755); err != nil {
		t.Fatal(err)
	}
	c := checker{osRelease: []byte("ID=ubuntu\nVERSION_ID=\"22.04\"\n"), arch: "amd64"}
	c.run = func(_ context.Context, name string, arguments ...string) ([]byte, error) {
		if name == worker {
			if strings.Join(arguments, " ") != "--runtime-info --project-root /test/assets-root" {
				t.Fatalf("core preflight arguments: %v", arguments)
			}
			return json.Marshal(info)
		}
		if name != "docker" || docker == nil {
			t.Fatalf("unexpected dependency probe: %s %v", name, arguments)
		}
		return docker(arguments)
	}
	return c, Options{WorkerPath: worker, DataRoot: "/test/assets-root"}
}
func TestRequiredDependenciesAndCPUWithoutROS(t *testing.T) {
	t.Setenv("ROS_DISTRO", "rolling") // Host ROS must never influence container support.
	c, options := fixtureChecker(t, validRuntime(), nil)
	report := c.check(context.Background(), options, nil)
	if !report.Ready || report.ExitCode() != 0 || report.Core.CUDACompiled {
		t.Fatalf("CPU-only framework was rejected: %+v", report)
	}
	for _, check := range report.Checks {
		if (check.ID == "ros1" || check.ID == "ros2") && check.Status != "skipped" {
			t.Fatal("unselected ROS was probed")
		}
	}
}
func TestPreflightRejectsUnsupportedOrBrokenInstallations(t *testing.T) {
	for _, name := range []string{"ubuntu-version", "architecture", "missing-core", "not-executable", "runtime-mismatch", "unapproved-upgrade", "wrong-framework", "assets"} {
		t.Run(name, func(t *testing.T) {
			info := validRuntime()
			switch name {
			case "runtime-mismatch":
				info.Runtime++
			case "unapproved-upgrade":
				info.MuJoCo = "3.16.0"
			case "wrong-framework":
				info.Framework = "other"
			case "assets":
				info.AssetsValid = false
				info.AssetsError = "policy checksum mismatch"
			}
			c, options := fixtureChecker(t, info, nil)
			switch name {
			case "ubuntu-version":
				c.osRelease = []byte("ID=ubuntu\nVERSION_ID=24.04\n")
			case "architecture":
				c.arch = "arm64"
			case "missing-core":
				options.WorkerPath += "-missing"
			case "not-executable":
				if err := os.Chmod(options.WorkerPath, 0644); err != nil {
					t.Fatal(err)
				}
			}
			report := c.check(context.Background(), options, nil)
			if report.Ready || report.ExitCode() != 1 {
				t.Fatalf("unsupported installation accepted: %+v", report)
			}
		})
	}
}
func TestROSDistroIsCheckedInsideIsolatedContainer(t *testing.T) {
	for _, version := range []string{"ros1", "ros2"} {
		for _, correct := range []bool{false, true} {
			name := version + checkStatus(correct)
			t.Run(name, func(t *testing.T) {
				distro := "noetic"
				if version == "ros2" {
					distro = "humble"
				}
				calls := 0
				c, options := fixtureChecker(t, validRuntime(), func(args []string) ([]byte, error) {
					calls++
					if args[0] == "image" {
						return []byte("sha256:fixture\n"), nil
					}
					joined := strings.Join(args, " ")
					for _, required := range []string{"--pull never", "--network none", "/opt/ros/" + distro + "/setup.sh"} {
						if !strings.Contains(joined, required) {
							t.Fatalf("missing isolated installation check %s: %s", required, joined)
						}
					}
					if version == "ros1" && !strings.Contains(joined, "rosversion -d") {
						t.Fatal("ROS1 install was inferred only from environment")
					}
					if version == "ros2" && !strings.Contains(joined, "ros2 pkg prefix rclcpp") {
						t.Fatal("ROS2 install was inferred only from environment")
					}
					if !correct {
						return []byte("SPROBO_ROS_DISTRO=unsupported\n"), nil
					}
					return []byte("SPROBO_ROS_DISTRO=" + distro + "\n{\"configurable_odometry\":true}\n"), nil
				})
				report := c.check(context.Background(), options, map[string]bool{version: true})
				if calls != 2 || report.ROSReady(version) != correct || !report.Ready || report.ExitCode() != 0 {
					t.Fatalf("optional ROS result: %+v", report)
				}
			})
		}
	}
}
func TestUnavailableDockerIsOptional(t *testing.T) {
	c, options := fixtureChecker(t, validRuntime(), func([]string) ([]byte, error) { return nil, errors.New("docker unavailable") })
	report := c.check(context.Background(), options, map[string]bool{"ros1": true})
	if !report.Ready || report.ROSReady("ros1") {
		t.Fatal("optional Docker failure blocked the CPU framework")
	}
}

func TestOldAdapterRequiresRebuild(t *testing.T) {
	c, options := fixtureChecker(t, validRuntime(), func(args []string) ([]byte, error) {
		if args[0] == "image" {
			return []byte("sha256:old"), nil
		}
		return []byte("SPROBO_ROS_DISTRO=noetic\n{\"transport\":\"unix_ipc\"}\n"), nil
	})
	report := c.check(context.Background(), options, map[string]bool{"ros1": true})
	if report.ROSReady("ros1") || !report.Ready {
		t.Fatal("old adapter must require rebuilding without disabling the ROS-free core")
	}
}
