package main

import (
	"context"
	"encoding/json"
	"flag"
	"fmt"
	"os"
	"path/filepath"
	"strings"
	"time"

	"github.com/egoist/mygo"
	"github.com/egoist/mygo/ui"
	"simple-rbt/console/session"
)

func main() {
	options := session.DefaultOptions()
	worker := flag.String("worker", "", "Independent C++ simulator executable")
	assets := flag.String("assets", "", "Robot assets directory")
	configPath := flag.String("config", "", "Initial session JSON")
	capture := flag.String("capture", "", "Native screenshot PNG then exit")
	delay := flag.Int("capture-after-ms", 6500, "Screenshot delay (milliseconds)")
	smoke := flag.Bool("smoke-test", false, "Verify configuration and editor without display/GPU")
	operate := flag.Bool("operate", false, "Start the configured motion session")
	e2e := flag.String("e2e-test", "", "Exercise headless MyGo widget callbacks against the real C++ core; write JSON")
	dependencies := flag.Bool("check-dependencies", false, "Check Ubuntu, framework MuJoCo/assets and optional container ROS; emit JSON without display/GPU")
	ros := flag.String("ros", "", "Optional container checks: ros1,ros2 (Noetic,Humble); host ROS is not required")
	flag.Parse()
	if *worker != "" {
		options.WorkerPath = *worker
	}
	if *assets != "" {
		options.DataRoot = filepath.Dir(*assets)
	}
	cfg := defaultConfig()
	if *configPath != "" {
		if err := loadJSON(*configPath, &cfg); err != nil {
			fmt.Fprintln(os.Stderr, err)
			os.Exit(1)
		}
	}
	if cfg.Adapters == nil {
		cfg.Adapters = map[string]bool{}
	}
	for _, version := range strings.Split(*ros, ",") {
		if version == "" {
			continue
		}
		if version != "ros1" && version != "ros2" {
			fmt.Fprintln(os.Stderr, "--ros accepts ros1,ros2 only")
			os.Exit(2)
		}
		cfg.Adapters[version] = true
	}
	if *dependencies {
		report := session.CheckDependencies(context.Background(), options, cfg.Adapters)
		encoder := json.NewEncoder(os.Stdout)
		encoder.SetIndent("", "  ")
		if err := encoder.Encode(report); err != nil {
			fmt.Fprintln(os.Stderr, err)
			os.Exit(1)
		}
		os.Exit(report.ExitCode())
	}
	a := newApp(options, cfg)
	if *smoke {
		if !a.configureRuntime(session.CheckDependencies(context.Background(), options, cfg.Adapters)) {
			_ = json.NewEncoder(os.Stderr).Encode(a.preflight)
			os.Exit(1)
		}
		if err := smokeTest(a); err != nil {
			fmt.Fprintln(os.Stderr, err)
			os.Exit(1)
		}
		fmt.Println("PASS: native MyGo configuration, rigid MID360, quaternion, 3D projection, control limits and release")
		return
	}
	mygo.App.SetName("SpRobo")
	mygo.App.SetVersion(session.Version)
	if base, err := os.UserConfigDir(); err == nil {
		mygo.App.SetPath(mygo.PathUserData, filepath.Join(base, "simple-rbt-console"))
		mygo.App.SetPath(mygo.PathLogs, filepath.Join(base, "simple-rbt-console", "logs"))
	}
	if base, err := os.UserCacheDir(); err == nil {
		mygo.App.SetPath(mygo.PathCache, filepath.Join(base, "simple-rbt-console"))
	}
	mygo.App.WhenReady(func() {
		a.win = mygo.NewWindow(mygo.WindowOptions{Title: a.windowTitle(), Width: 1320, Height: 900, StateKey: "sprobo", Content: ui.View(a.view)})
		a.win.OnBlur(a.blurManual)
		a.win.OnClose(func(*mygo.CloseEvent) { a.close() })
		a.pump()
		a.beginPreflight()
		if *e2e != "" {
			a.startE2e(*e2e)
		}
		if *operate {
			go func() {
				for i := 0; i < 200 && !a.closed.Load(); i++ {
					time.Sleep(100 * time.Millisecond)
					attempt := make(chan bool, 1)
					a.win.Update(func() {
						ready := a.meta.State == "config" && !a.loading
						if ready {
							a.startOperating()
						}
						attempt <- ready
					})
					select {
					case <-a.done:
						return
					case started := <-attempt:
						if started {
							return
						}
					}
				}
			}()
		}
		if *capture != "" {
			go func() {
				time.Sleep(time.Duration(*delay) * time.Millisecond)
				png, err := a.win.CapturePage()
				if err == nil {
					err = os.WriteFile(*capture, png, 0600)
				}
				if err != nil {
					fmt.Fprintln(os.Stderr, err)
					a.exitCode.Store(1)
				}
				a.win.Update(func() { a.close(); a.win.Close() })
			}()
		}
	})
	defer a.close()
	if err := mygo.App.Run(); err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}
	if code := a.exitCode.Load(); code != 0 {
		a.close()
		os.Exit(int(code))
	}
}
