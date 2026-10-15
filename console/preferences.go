package main

import (
	"os"
	"path/filepath"
	"strings"

	"github.com/egoist/mygo/ui"
)

type Preferences struct {
	Theme    string `json:"theme"`
	Language string `json:"language"`
}

func preferencesPath() string {
	base, err := os.UserConfigDir()
	if err != nil {
		return ""
	}
	return filepath.Join(base, "simple-rbt-console", "appearance.json")
}

func loadPreferences() Preferences {
	prefs := Preferences{Theme: "dark", Language: "en"}
	for _, name := range []string{"LC_ALL", "LC_MESSAGES", "LANG"} {
		if locale := os.Getenv(name); locale != "" {
			if strings.HasPrefix(strings.ToLower(locale), "zh") {
				prefs.Language = "zh-CN"
			}
			break
		}
	}
	var saved Preferences
	if loadJSON(preferencesPath(), &saved) == nil {
		if saved.Theme == "dark" || saved.Theme == "light" {
			prefs.Theme = saved.Theme
		}
		if saved.Language == "en" || saved.Language == "zh-CN" {
			prefs.Language = saved.Language
		}
	}
	return prefs
}

func (a *app) appearanceControls(c *ui.Context) {
	appearance := a.tr("Dark", "深色")
	if a.preferences.Theme == "light" {
		appearance = a.tr("Light", "浅色")
	}
	if ui.Select(c.Key("appearance"), &appearance, []string{a.tr("Dark", "深色"), a.tr("Light", "浅色")}).MinWidth(0).Width(106).Label(a.tr("Colour scheme", "颜色方案")).Changed() {
		a.preferences.Theme = "dark"
		if appearance == a.tr("Light", "浅色") {
			a.preferences.Theme = "light"
		}
		a.savePreferences()
	}
	language := "English"
	if a.preferences.Language == "zh-CN" {
		language = "简体中文"
	}
	if ui.Select(c.Key("language"), &language, []string{"English", "简体中文"}).MinWidth(0).Width(132).Label("Language / 语言").Changed() {
		a.preferences.Language = "en"
		if language == "简体中文" {
			a.preferences.Language = "zh-CN"
		}
		a.savePreferences()
	}
}

func (a *app) windowTitle() string {
	return a.tr("SpRobo · Robot console", "SpRobo · 机器人操作台")
}

func (a *app) savePreferences() {
	if a.win != nil {
		a.win.SetTitle(a.windowTitle())
	}
	path := preferencesPath()
	if path == "" {
		return
	}
	if err := saveJSON(path, a.preferences); err != nil {
		a.status = a.tr("Could not save appearance preferences: ", "无法保存界面偏好：") + err.Error()
		a.showLog = true
	}
}
