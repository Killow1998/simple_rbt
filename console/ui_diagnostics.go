package main

import (
	"encoding/json"
	"fmt"
	"github.com/egoist/mygo/ui"
	"path/filepath"
	"simple-rbt/console/diagnostics"
	"simple-rbt/console/session"
	"strings"
	"time"
)

type diagnosticsView struct {
	Archive                 bool
	Open, Busy              bool
	Query, Level, Component string
	Records                 []diagnostics.Record
	Bytes                   int64
	Error, Export           string
}

func (a *app) refreshDiagnostics() {
	if !a.diagnostics.Open || a.diagnostics.Archive || a.diagnostics.Busy || a.manager == nil || a.win == nil {
		return
	}
	a.diagnostics.Busy = true
	directory := a.manager.Events.Directory
	go func() {
		rows, size, err := diagnostics.ReadRecent(directory, "")
		if a.closed.Load() {
			return
		}
		a.win.Update(func() {
			a.diagnostics.Busy = false
			a.diagnostics.Records = rows
			a.diagnostics.Bytes = size
			a.diagnostics.Error = ""
			if err != nil {
				a.diagnostics.Error = err.Error()
			}
		})
	}()
}
func (a *app) searchDiagnostics() {
	if a.diagnostics.Busy {
		return
	}
	a.diagnostics.Archive = true
	a.diagnostics.Busy = true
	query := a.diagnostics.Query
	directory := a.manager.Events.Directory
	go func() {
		rows, size, err := diagnostics.Search(directory, query)
		if a.closed.Load() {
			return
		}
		a.win.Update(func() {
			a.diagnostics.Busy = false
			a.diagnostics.Records = rows
			a.diagnostics.Bytes = size
			a.diagnostics.Error = ""
			if err != nil {
				a.diagnostics.Error = err.Error()
			}
		})
	}()
}
func (a *app) exportDiagnostics() {
	directory := a.manager.Events.Directory
	destination := filepath.Join(a.options.StateDir, "diagnostics-"+time.Now().Format("20060102-150405")+".zip")
	manifest := map[string]any{"version": 1, "framework_version": session.Version, "config": clone(a.cfg), "scene_draft": clone(a.sceneDraft), "session": a.manager.Events.Session(), "core": clone(a.meta), "note": "Explicit export, outside the rolling 100 MB log store. Sensor payloads are not included."}
	go func() {
		err := diagnostics.Export(directory, destination, manifest)
		if a.closed.Load() {
			return
		}
		a.win.Update(func() {
			if err != nil {
				a.diagnosticError(err)
				a.diagnostics.Error = err.Error()
			} else {
				a.diagnostics.Export = destination
			}
		})
	}()
}
func (a *app) diagnosticsPage(c *ui.Context) {
	if a.diagnostics.Level == "" {
		a.diagnostics.Level = "all"
	}
	if a.diagnostics.Component == "" {
		a.diagnostics.Component = "all"
	}
	ui.Column(c).Fill().Padding(18).Gap(10).Children(func() {
		ui.Row(c).Gap(8).Children(func() {
			ui.Text(c, a.tr("Logs & diagnostics", "日志与诊断")).FontSize(20).Bold().Grow(1)
			ui.Button(c, a.tr("Refresh", "刷新")).OnClick(a.action("logs.refresh", func() { a.diagnostics.Archive = false; a.refreshDiagnostics() }))
			ui.Button(c, a.tr("Export diagnostic bundle", "导出诊断包")).OnClick(a.action("logs.export", a.exportDiagnostics))
			ui.Button(c, a.tr("Back", "返回")).OnClick(a.action("logs.close", func() { a.diagnostics.Open = false }))
		})
		trace := a.manager.Events.TraceEnabled()
		if ui.Checkbox(c, &trace, a.tr("Detailed event tracing (sensors, IPC, ROS)", "详细事件追踪（传感器、IPC、ROS）")).Changed() {
			a.manager.Events.SetTrace(trace)
			if err := a.manager.Client.SendRequest(map[string]any{"op": "logging", "trace": trace, "modules": a.manager.Events.Modules()}); err != nil {
				a.diagnosticError(err)
			}
		}
		ui.Row(c).Gap(16).Children(func() {
			current := a.manager.Events.Modules()
			enabled := map[string]bool{}
			for _, name := range current {
				enabled[name] = true
			}
			for _, module := range []struct{ id, en, zh string }{{"sensor", "Sensors", "传感器"}, {"ipc", "IPC", "进程通信"}, {"ros", "ROS publish", "ROS 发布"}, {"control", "Control / requests", "控制／请求"}} {
				on := enabled[module.id]
				if ui.Checkbox(c.Key(module.id), &on, a.tr(module.en, module.zh)).Changed() {
					enabled[module.id] = on
					names := []string{}
					for _, name := range []string{"sensor", "ipc", "ros", "control"} {
						if enabled[name] {
							names = append(names, name)
						}
					}
					a.manager.Events.SetModules(names)
					if err := a.manager.Client.SendRequest(map[string]any{"op": "logging", "trace": trace, "modules": names}); err != nil {
						a.diagnosticError(err)
					}
				}
			}
		})
		ui.Text(c, a.tr("100 MB shared limit · oldest managed logs rotate automatically · writes batched once per second", "总上限 100 MB · 自动移除最旧的受管日志 · 每秒批量写入")).TextColor(c.Theme().TextMuted)
		health := a.manager.Events.Health()
		ui.Text(c, fmt.Sprintf(a.tr("Stored %.2f / 100 MB · GUI queued %d bytes · dropped %d · write failures %d", "已存 %.2f / 100 MB · GUI 队列 %d 字节 · 丢弃 %d · 写入失败 %d"), float64(a.diagnostics.Bytes)/1e6, health.Queued, health.Dropped, health.WriteFailures)).FontSize(12)
		if health.Error != "" {
			ui.Text(c, health.Error).TextColor(c.Theme().Warning)
		}
		if a.meta.Logging != nil {
			ui.Text(c, fmt.Sprintf(a.tr("Core · queued %v bytes · dropped %v · write failures %v", "核心 · 队列 %v 字节 · 丢弃 %v · 写入失败 %v"), a.meta.Logging["queued_bytes"], a.meta.Logging["dropped"], a.meta.Logging["write_failures"])).FontSize(12)
			if message, ok := a.meta.Logging["error"].(string); ok && message != "" {
				ui.Text(c, message).TextColor(c.Theme().Warning)
			}
		}
		if a.diagnostics.Error != "" {
			ui.Text(c, a.diagnostics.Error).TextColor(c.Theme().Warning)
		}
		ui.Text(c, a.manager.Events.Directory).FontSize(12)
		if a.diagnostics.Export != "" {
			ui.Text(c, a.tr("Exported (outside log quota): ", "已导出（不计入日志额度）：")+a.diagnostics.Export).FontSize(12)
		}
		ui.Row(c).Gap(8).Children(func() {
			ui.Text(c, a.tr("Search", "搜索"))
			ui.TextInput(c.Key("log-search"), &a.diagnostics.Query).Grow(1).Label(a.tr("Search events / IDs", "搜索事件／编号"))
			ui.Button(c, a.tr("Search retained logs", "搜索全部保留日志")).Disabled(a.diagnostics.Busy).OnClick(a.action("logs.search", a.searchDiagnostics))
			ui.Text(c, a.tr("Level", "级别"))
			ui.Select(c, &a.diagnostics.Level, []string{"all", "info", "warning", "error", "trace"}).Width(130).Label(a.tr("Level", "级别"))
			ui.Text(c, a.tr("Module", "模块"))
			ui.Select(c, &a.diagnostics.Component, []string{"all", "gui", "core", "ros1", "ros2"}).Width(130).Label(a.tr("Module", "模块"))
		})
		if a.diagnostics.Archive {
			ui.Text(c, a.tr("Retained-log search · up to 500 matches, 200 shown · Refresh returns to live tail", "全保留日志搜索 · 最多 500 条匹配，显示 200 条 · 刷新返回实时日志")).FontSize(12)
		} else {
			ui.Text(c, a.tr("Live tail: latest 2,000 records / 4 MiB · use Search retained logs for older events", "实时日志：最近 2,000 条／4 MiB · 更早事件可搜索全部保留日志")).FontSize(12)
		}
		ui.Scroll(c).Grow(1).MinHeight(0).Padding(8).Background(panelBackground(c)).Children(func() {
			shown := 0
			for _, r := range a.diagnostics.Records {
				if a.diagnostics.Level != "" && a.diagnostics.Level != "all" && r.Level != a.diagnostics.Level {
					continue
				}
				if a.diagnostics.Component != "" && a.diagnostics.Component != "all" && r.Component != a.diagnostics.Component {
					continue
				}
				data, _ := json.Marshal(r)
				if a.diagnostics.Query != "" && !strings.Contains(strings.ToLower(string(data)), strings.ToLower(a.diagnostics.Query)) {
					continue
				}
				fields, _ := json.Marshal(r.Fields)
				ui.Text(c, fmt.Sprintf("%s  %s · %s · %s\n%s  #%d  sim=%d\n%s", time.Unix(0, r.WallNS).Format("15:04:05.000"), r.Level, r.Component, r.Event, r.Session, r.Seq, r.SimNS, string(fields))).FontSize(12).Margin(0, 0, 10, 0)
				shown++
				if shown >= 200 {
					break
				}
			}
			if shown == 0 {
				ui.Text(c, a.tr("No matching events yet. Buffered events appear after the next batch.", "暂无匹配事件，缓冲记录将在下次批量写入后显示。"))
			}
		})
	})
}
