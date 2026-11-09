package main

import (
	"os"
	"testing"
	"time"

	"simple-rbt/console/session"
)

func TestSessionDisconnectLifecycle(t *testing.T) {
	worker := os.Getenv("SIMPLE_RBT_CPP_WORKER")
	if worker == "" {
		t.Skip("requires the standalone C++ core")
	}
	newTestApp := func(t *testing.T, path string) *app {
		t.Helper()
		options := session.DefaultOptions()
		options.WorkerPath = path
		if root := os.Getenv("SIMPLE_RBT_CPP_DATA_ROOT"); root != "" {
			options.DataRoot = root
		}
		options.StateDir = t.TempDir()
		a := newApp(options, defaultConfig())
		t.Cleanup(func() { _ = a.manager.Close(); a.manager.Events.Close() })
		return a
	}
	wait := func(t *testing.T, a *app, predicate func() bool) {
		t.Helper()
		deadline := time.Now().Add(10 * time.Second)
		for time.Now().Before(deadline) {
			a.consume()
			if predicate() {
				return
			}
			time.Sleep(20 * time.Millisecond)
		}
		t.Fatalf("session did not reach expected state: label=%q, metadata=%q, log=%v", a.stateLabel(), a.meta.State, a.manager.Logs())
	}
	t.Run("connection lost", func(t *testing.T) {
		a := newTestApp(t, worker)
		a.loading = true
		if err := a.manager.Start(map[string]any{"robot": "scout_mini", "backend": "cpu", "sensors": map[string]any{"mid360": false, "camera": false, "depth": false, "panorama": false}}); err != nil {
			t.Fatal(err)
		}
		wait(t, a, func() bool { return a.meta.State == "config" })
		a.stage = "operate"
		if err := a.manager.Client.SendRequest(map[string]any{"op": "start"}); err != nil {
			t.Fatal(err)
		}
		wait(t, a, func() bool { return a.meta.State == "running" && a.manualMode() })
		a.manager.Client.Close()
		wait(t, a, func() bool { return a.stateLabel() == a.tr("Disconnected", "未连接") })
		if a.manualMode() || a.manualHeartbeat {
			t.Fatal("manual control remained active after disconnect")
		}
		// Adapter preparation can finish after the core disconnects and resume
		// the start request without another button click.
		a.stage = "configure"
		a.preflight.Ready = true
		a.workspace.Version = 0
		a.startOperating()
		if a.stage != "configure" {
			t.Fatal("disconnected session entered the operating page")
		}
		mount := a.cfg.Mounts["mid360"]
		changed := mount
		changed.Position[0] += .01
		a.editMount("mid360", changed)
		if !samePose(a.cfg.Mounts["mid360"], mount) || len(a.pending) != 0 {
			t.Fatal("disconnected session accepted a mount edit")
		}
	})
	t.Run("mount edit interrupted by disconnect", func(t *testing.T) {
		a := newTestApp(t, worker)
		a.loading = true
		if err := a.manager.Start(map[string]any{"robot": "scout_mini", "backend": "cpu", "sensors": map[string]any{"mid360": false, "camera": false, "depth": false, "panorama": false}}); err != nil {
			t.Fatal(err)
		}
		wait(t, a, func() bool { return a.meta.State == "config" })
		mount := a.cfg.Mounts["mid360"]
		changed := mount
		changed.Position[0] += .01
		a.editMount("mid360", changed)
		if len(a.pending) == 0 {
			t.Fatal("mount edit was not pending before disconnect")
		}
		a.manager.Client.Close()
		wait(t, a, func() bool { return a.stateLabel() == a.tr("Disconnected", "未连接") })
		if !samePose(a.cfg.Mounts["mid360"], mount) || len(a.pending) != 0 {
			t.Fatal("unconfirmed mount edit remained after disconnect")
		}
	})
	t.Run("core exits before connecting", func(t *testing.T) {
		a := newTestApp(t, "/bin/false")
		a.loading = true
		if err := a.manager.Start(map[string]any{}); err != nil {
			t.Fatal(err)
		}
		wait(t, a, func() bool { return a.manager.CoreExited() && a.stateLabel() == a.tr("Disconnected", "未连接") })
	})
}
