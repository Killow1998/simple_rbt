package main

import (
	"context"

	"simple-rbt/console/session"
)

func (a *app) configureRuntime(report session.DependencyReport) bool {
	a.preflight = report
	if !report.Ready || report.Core == nil {
		return false
	}
	catalog, err := loadCatalog(a.options.DataRoot, report.Core.Robots)
	if err != nil {
		a.preflight.Ready = false
		a.preflight.Checks = append(a.preflight.Checks, session.DependencyCheck{ID: "configuration", Required: true, Status: "fail", Message: err.Error()})
		return false
	}
	a.catalog = catalog
	if spec, ok := catalog.Robots[a.cfg.Robot]; ok && a.cfg.Controller == "auto" {
		a.cfg.Controller = spec.DefaultController
	}
	// Keep configuration and Load available even when the supplied profile is
	// invalid; startConfig validates before launching or replacing a worker.
	a.initialized = true
	return true
}
func (a *app) beginPreflight() {
	if a.checkCancel != nil {
		a.checkCancel()
	}
	a.checkGeneration++
	generation := a.checkGeneration
	a.checking = true
	ctx, cancel := context.WithCancel(context.Background())
	a.checkCancel = cancel
	selected := map[string]bool{"ros1": a.cfg.Adapters["ros1"], "ros2": a.cfg.Adapters["ros2"]}
	go func() {
		report := session.CheckDependencies(ctx, a.options, selected)
		if a.closed.Load() {
			return
		}
		a.win.Update(func() {
			if a.closed.Load() || generation != a.checkGeneration {
				return
			}
			a.checking = false
			a.preflight = report
			for _, check := range report.Checks {
				a.manager.RememberAdapterCheck(check)
			}
			for _, version := range []string{"ros1", "ros2"} {
				a.adaptersVerified[version] = report.ROSReady(version)
			}
			if !a.initialized && a.configureRuntime(report) {
				a.startConfig()
			}
		})
	}()
}
