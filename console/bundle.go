package main

import (
	"fmt"
	"os"
	"path/filepath"
	"simple-rbt/console/session"
)

type reproductionBundle struct {
	Format   string `json:"format"`
	Default  string `json:"default_profile"`
	Profiles map[string]struct {
		Root   string `json:"root"`
		Config string `json:"config"`
	} `json:"profiles"`
}

func bundleProfile(root, name string) (string, string, error) {
	var bundle reproductionBundle
	if err := loadJSON(filepath.Join(root, "bundle.json"), &bundle); err != nil {
		return "", "", err
	}
	if bundle.Format != "sprobo.bundle.v1" {
		return "", "", fmt.Errorf("Unsupported bundle format: %s", bundle.Format)
	}
	if name == "" {
		name = bundle.Default
	}
	profile, ok := bundle.Profiles[name]
	if !ok {
		return "", "", fmt.Errorf("Unknown profile %q; available: %v", name, sortedKeys(bundle.Profiles))
	}
	return filepath.Join(root, profile.Root), filepath.Join(root, profile.Config), nil
}

func isolateReproduction(options *session.Options) error {
	base, err := os.UserConfigDir()
	if err != nil {
		return err
	}
	appearance, _ := os.ReadFile(filepath.Join(base, "simple-rbt-console", "appearance.json"))
	parent := filepath.Join(options.StateDir, "reproductions")
	if err = os.MkdirAll(parent, 0700); err != nil {
		return err
	}
	run, err := os.MkdirTemp(parent, "run-")
	if err != nil {
		return err
	}
	config := filepath.Join(run, "config")
	if err = os.MkdirAll(filepath.Join(config, "simple-rbt-console"), 0700); err != nil {
		return err
	}
	if len(appearance) > 0 {
		if err = os.WriteFile(filepath.Join(config, "simple-rbt-console", "appearance.json"), appearance, 0600); err != nil {
			return err
		}
	}
	os.Setenv("XDG_CONFIG_HOME", config)
	options.StateDir = filepath.Join(run, "state")
	fmt.Println("Reproduction records / 复现记录:", run)
	return nil
}
