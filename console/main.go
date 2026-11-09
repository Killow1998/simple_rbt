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
	physicsJobPath := flag.String("physics-job", "", "Bounded physics/training job JSON; independent optional Warp process")
	physicsBackend := flag.String("physics-backend", "mujoco", "Physical solver for --physics-job: mujoco or warp")
	physicsTask := flag.String("physics-task", "batch", "Bounded task: batch or train-smoke")
	jobHeadless := flag.Bool("job-headless", false, "Run --physics-job without a window; emit result")
	worker := flag.String("worker", "", "Independent C++ simulator executable")
	assets := flag.String("assets", "", "Robot assets directory")
	configPath := flag.String("config", "", "Initial session JSON")
	bundle := flag.String("bundle", "", "Portable reproduction bundle directory")
	profile := flag.String("profile", "", "Named profile within --bundle")
	capture := flag.String("capture", "", "Native screenshot PNG then exit")
	delay := flag.Int("capture-after-ms", 6500, "Screenshot delay (milliseconds)")
	smoke := flag.Bool("smoke-test", false, "Verify configuration and editor without display/GPU")
	operate := flag.Bool("operate", false, "Start the configured motion session")
	fixedConfig := flag.Bool("fixed-config", false, "Start --config as an immutable reproduction profile")
	e2e := flag.String("e2e-test", "", "Exercise headless MyGo widget callbacks against the real C++ core; write JSON")
	dependencies := flag.Bool("check-dependencies", false, "Check Ubuntu, framework MuJoCo/assets and optional container ROS; emit JSON without display/GPU")
	ros := flag.String("ros", "", "Optional container checks: ros1,ros2 (Noetic,Humble); host ROS is not required")
	flag.Parse()
	if *physicsJobPath != "" {
		if *worker != "" {
			options.WorkerPath = *worker
		}
		if err := physicsJobMain(*physicsJobPath, *physicsBackend, *physicsTask, *jobHeadless, options); err != nil {
			fmt.Fprintln(os.Stderr, err)
			os.Exit(1)
		}
		return
	}
	if *bundle != "" {
		if *assets != "" || *configPath != "" {
			fmt.Fprintln(os.Stderr, "--bundle supplies --assets and --config")
			os.Exit(2)
		}
		root, config, err := bundleProfile(*bundle, *profile)
		if err != nil {
			fmt.Fprintln(os.Stderr, err)
			os.Exit(2)
		}
		*assets, *configPath, *fixedConfig = filepath.Join(root, "assets"), config, true
	} else if *profile != "" {
		fmt.Fprintln(os.Stderr, "--profile requires --bundle")
		os.Exit(2)
	}
	if *fixedConfig {
		if *configPath == "" {
			fmt.Fprintln(os.Stderr, "--fixed-config requires --config")
			os.Exit(2)
		}
		if !*dependencies && !*smoke {
			if err := isolateReproduction(&options); err != nil {
				fmt.Fprintln(os.Stderr, err)
				os.Exit(1)
			}
		}
		*operate = true
	}
	if flag.NArg() != 0 {
		fmt.Fprintf(os.Stderr, "Unexpected argument %q / 无法识别参数。Option names must not have leading spaces / 选项前不能带空格。Use --help for usage.\n", flag.Arg(0))
		os.Exit(2)
	}
	if *worker != "" {
		options.WorkerPath = *worker
	}
	if *assets != "" {
		options.DataRoot = filepath.Dir(*assets)
		if _, err := os.Stat(filepath.Join(options.DataRoot, "deployment/ros1/Dockerfile")); err == nil {
			options.AdapterContext = options.DataRoot
		}
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
	a.fixedConfig = *fixedConfig
	a.initWorkspace(*configPath == "" && !*smoke && *e2e == "" && !*dependencies)
	cfg = a.cfg
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
