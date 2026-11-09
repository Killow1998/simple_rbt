# Go2W SuperMimic validation / 外部 Go2W 策略验证

## Scope / 范围

Development candidate based on `c6c4355`. This adapter is opt-in through an
external catalog; the standard robot registry, default `rl_sar` policy,
desktop, and ROS contracts retain their existing behavior. Runtime control
and inference use C++ and ONNX Runtime CPU. No training is involved.

本阶段基于 `c6c4355`。新增外部 catalog 启用的控制器，保留原 Go2W 默认策略。
外部源码、模型及权重仅用于本地验证，不纳入标准 catalog、安装包或源码树。

Reference: `uwvwko-zzz/go2w-supermimic` commit
`019c2d7300e82b59486996d87f6013108effb005`, `model_40000.pt`, terrain revision 4.
The reference uses MuJoCo Warp 3.11; this core uses native MuJoCo 3.15.
Matching inputs and parameters do not imply identical cross-engine trajectories.

参考实现为 MuJoCo Warp 3.11，本框架为原生 MuJoCo 3.15。参数与接口对齐不代表跨引擎轨迹逐位一致。

## Interactive launch / 交互启动

With the prepared local reference assets and development build, run from the
repository root:

```bash
./tools/run_supermimic.sh up
./tools/run_supermimic.sh flat --check
```

The launcher opens a separate SpRobo window with a fixed reproduction profile.
It uses `build/integration/` and the portable bundle at
`~/.local/share/simple-rbt/bundles/go2w-supermimic/`. `SIMPLE_RBT_BUILD` and
`SIMPLE_RBT_BUNDLE` override these locations. It no longer loads runtime resources
from dated experiment directories.
It neither installs a package nor reuses the existing desktop session. Each
launch has its own configuration, cache, logs and random IPC directory.

入口使用开发版和本地外部资源。启动后自动进入操作台。切换地形时关闭这个独立窗口，
再带相应参数启动；原安装版及其配置不受影响。启动日志会输出本次运行记录目录。

| Argument / 参数 | Original terrain / 原始地形 |
|---|---|
| `flat` | Flat / 平地 |
| `up`, `down` | Level-0 stairs, 2 cm × 5 / 每级 2 cm、5 级上下台阶 |
| `slope` | Level-0 3° slope / 3° 缓坡 |
| `rubble` | Level-0 relief ≤2 cm / 最高 2 cm 起伏 |
| `incline-up`, `incline-down` | +10° / −10° slopes / 上下坡 |
| `up-l1` … `up-l5`, `down-l1` … `down-l5` | 4/8/12/16/20 cm per step, five steps / 单级高度、每段五级 |
| `mapping` | Tutorial corridor with MID360 + D435i RGB/depth / 建图走廊与传感器 |

Click the view or joystick, then hold W/S for forward/back, A/D for lateral
motion, and Q/E for yaw. Release, Space, or manual focus loss clears the command.
For the lateral slider, hold the mouse button to move and release it to stop.
The captured drag remains active when the pointer leaves the slider rectangle.
At the initial 50% speed setting, the keys request ±0.5 m/s forward, ±0.3 m/s
lateral, and ±0.5 rad/s yaw. The existing speed slider adjusts these limits.

点击视图或摇杆后使用键盘。当前检查点只接收机身平面 `vx/vy/yaw_rate`，**不支持高度、
俯仰或横滚目标**。俯仰和高度会随平衡及地形自然变化，不是独立控制量。

Terrain profiles render the observer view with sensor streams disabled. `mapping`
enables the existing MID360 and D435i streams; ROS adapters remain independent.
Training stays in the original mjlab/MuJoCo Warp workspace. The live Noetic
acceptance below uses the same mapping profile.

## External asset contract / 外部资源契约

Select robot `go2w` and controller `supermimic`. Use `tracked` for simulated
velocity feedback, or `native` to reproduce the actor without feedback.
New bundles prepared by `tools/prepare_supermimic_bundle.py` select `tracked`;
existing bundles retain the mode saved in their session configuration.
选择 `go2w` 与 `supermimic`。`tracked` 使用仿真速度反馈；`native` 直接运行策略，
用于原始策略对照。新生成的资源包选择 `tracked`；已有资源包保留原配置。
An external `assets/catalog.json` supplies `models.go2w-supermimic` and
`controllers.go2w-supermimic`. The latter contains:

| Field / 字段 | Meaning / 含义 |
|---|---|
| `policy`, `policy_sha256` | Frozen ONNX path and digest; path relative to project root or absolute |
| `base_body` | Original base link name, including any model namespace |
| `joint_names` | Canonical 16 joints: FL, FR, RL, RR hip/thigh/calf, then four wheels |
| `default_dof_pos` | 16 canonical default joint positions |
| `action_scale`, `action_offset` | 16 canonical values exported from the final action manager |
| `env_action_order` | Environment-slot to canonical-slot permutation from the checkpoint |

本次检查点的排列为 `0..15`。最终缩放为 hip/thigh/calf 的 `.25/.6/.8`，轮速为 `35`；
应读取最终 manager 参数，不能套用 flat 配置的统一腿缩放或轮速 `15`。

The actor receives 265 float32 values. Each term stores five chronological
frames: body angular velocity (3), projected gravity (3), relative leg position
(12), leg velocity × .05 (12), wheel velocity × .05 (4), previous environment
action (16), command (3). Reset repeats the initial frame. The model contains
frozen normalization and returns 16 canonical float32 actions.

控制频率为 50 Hz，物理步长须整除 20 ms。当前原模型为 5 ms。
位置／速度目标直接写入原执行器，保留原增益、力矩限制及 armature。
机身角速度使用 link frame；MuJoCo 的 `mjOBJ_BODY` 局部帧是惯性主轴，不能替代 `mjOBJ_XBODY`。

For an imported complete world, the local validation profile uses an empty v2
scene overlay. This retains the original terrain without adding a second floor.
Framework sensor sites are massless. Existing XML `visual/global` and
`visual/map` nodes are reused when applying framebuffer settings.

原始完整场景使用空 v2 overlay，避免重复生成地板。保留模型的原接触、质量和执行器参数。
导出的空无名 default 节点需在离线准备时移除；原 mesh 文件保持原样。
3.15 的默认 `ls_iterations=50` 与原编译模型的 `20` 不同，外部 profile 必须明确保留原值。

## Build and checks / 构建与检查

```bash
cmake -S . -B build/supermimic-20261029 \
  -DSIMPLE_RBT_CUDA=OFF -DSIMPLE_RBT_DESKTOP=OFF \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER_LAUNCHER=ccache
cmake --build build/supermimic-20261029 \
  --target simple-rbt-sim supermimic_probe supermimic_e2e -j4

# MODEL_XML is the original imported model; fixture files are external.
build/supermimic-20261029/supermimic_probe \
  MODEL_XML CONTROLLER_JSON FRAMES_JSON REPORT_JSON --complete-state

build/supermimic-20261029/supermimic_e2e ASSET_ROOTS NEW_OUTPUT_DIRECTORY
build/supermimic-20261029/supermimic_e2e ASSET_ROOTS NEW_COURSE_OUTPUT --courses
```

These executables require local reference fixtures and are not registered as
asset-free CTest checks. `ASSET_ROOTS` contains `flat`, `slope_plus`,
`slope_minus`, and `terrain_{up,down,slope,rubble}` profiles, each with its
catalog and `validation.scene.json`.

`FRAMES_JSON` is an array of `qpos`, `qvel`, `command`, `observation`, `action`
and optional `reset` records. `command` must be the command consumed by the
actor. The reference command manager delays requests by one control step, so
the replay adapter uses the latest command term from the recorded observation.

默认 probe 保留自身动作历史递推，用于数值误差累积压力测试。
`--complete-state` 仅在测试端恢复上一环境动作，再由生产代码构造分项历史；
它不写回整条期望观测，也不参与物理闭环。完整状态对照和自由递推结果分别保存。

## Verified results / 验证结果

The precise export retains float32 normalization and the original weights,
uses float64 dense accumulation and an equivalent exponential ELU expression,
and returns float32 actions. Its local ONNX SHA256 is
`6c315555c5e2b472316f62e306ee62c7e4e373830843c881532e99b60b1da3f1`.

| Check / 检查 | Result / 结果 |
|---|---|
| Existing Go2W smoke after common XML merge fix | Trajectory, compiled XML and IMU bytes identical to baseline |
| Original 3.11 MJB vs assembled 3.15 model | 574 body/joint/collision/actuator/DOF fields exactly matched; inspected solver options matched |
| 600-frame forward/stop replay, recursive action history | Observation max `3.58e-7`; normalized action max `2.04e-7` |
| 600-frame lateral case, complete state | Observation max `1.19e-7`; normalized action max `1.86e-7` |
| Research workspace complete-state coverage | 33/33 trajectories, 20,513 frames; observation max `3.58e-7`, normalized action max `2.79e-7` |
| Actual CPU inference, ORT 1.19.2, one thread | Median about 48–49 µs; p95 about 49–51 µs, 600 real observations |
| Native flat and ±10° slope matrix | 30/30; maximum per-axis tracking MAE `[.03553, .04358, .06077]` m/s, m/s, rad/s |
| Command `.05 m/s` | Mean forward speed `.03511–.03570 m/s` |
| Zero command on ±10° slope | Last-four-second planar drift `.03724–.06434 m` |
| Original level-0 up/down/slope/rubble | 4/4; reached x ≥ 5.4 m, then stopped for 2 s |
| Tracking assistance | Zero tracker updates in all 34 native cases |

坡面仍有缓慢漂移，不能称为绝对静止。level-0 为每级 2 cm × 5 级、3° 坡或最高 2 cm
起伏，不能据此声称通过正常建筑楼梯或真实机器人验收。

The initial float32 export exceeded the `2e-6` gate on extreme synthetic input.
An extended recursive-action replay also retained a marginal failure on one
lateral case; it is an accumulation stress result, separate from complete-state
equivalence. Neither negative result was replaced or relaxed into a pass.

本地证据位于 `output/supermimic-20261029/`：

- `legacy-visual-parity.json`：旧默认 Go2W 回归。
- `model-audit-aligned.json`：原 3.11 编译参数与装配后 3.15 模型核对。
- `precise-forward/report.json`、`complete-state-left-report.json`：数值对照及 CPU 推理计时。
- `complete-state-coverage-summary.json`：科研侧使用相同 probe 完成的扩展完整状态对照汇总。
- `native-e2e-aligned/report.json`、`native-level0-aligned/report.json`：最终闭环及 CSV 轨迹。
- `acceptance-summary.json`：汇总；`native-level0/report-field-correction.json` 保留旧报告字段更正说明。

The next validation boundary is higher terrain difficulty and independent
hardware evaluation. This stage is frozen at the reported cases; the
reference research workspace owns the broader research interpretation.

## Interactive reproduction registration — 2026-10-29 / 交互复现登记

Scope is an independent development launcher and reviewable video using the
same precise ONNX, original robot parameters and already validated source
terrains. Training remains in the upstream mjlab/MuJoCo Warp stack. No new
terrain difficulty, training, installation or default-policy change is included.

先复用 MyGo 操作台的键盘、摇杆与观察相机。增加显式固定配置入口；启动参数选择
原始平地、±10°坡及 level-0 地形。控制只支持机身 `vx/vy/yaw_rate`，不支持高度或俯仰目标。
XDG 配置、缓存和运行记录独立，核心使用已有随机 Unix socket 会话。

Before display testing, the registered budget and gates are:

- At most one short keyboard/interface sequence and one review clip, each
  ≤35 simulated seconds and ≤300 wall seconds; no rerun of the passed 30+4 matrix.
- CPU rendering on an isolated Xvfb display; at most four CPU cores, 16 GiB RAM,
  and 100 MB per log. Record observed resource use and real-time factor.
- Actual OS keyboard input must reach the core: forward, lateral and yaw;
  release, Space and manual focus loss must clear the applied command.
- Configuration stays `supermimic/native`; final tracker updates must equal zero.
- The window must show a changing rendered scene and the unsupported height/pitch
  limitation. The saved video must decode and retain the existing 1320×900 window.
- Preserve the exact scene/configuration, commands, trajectory, binary/model hashes
  and video metadata. Check that installed binary hashes and existing user settings
  are unchanged. Do not close or reuse the user's current window.

Status: completed. Only the registered keyboard sequence and review clip ran;
the previous 30+4 physics matrix was not repeated.

| Display check / 展示检查 | Result / 结果 |
|---|---|
| All seven local profile dependency checks | Passed; model and policy digests verified |
| OS keyboard W/S/A/D/Q/E and each release | Passed through the existing GUI → IPC → C++ path |
| Space with a held key; focus loss | Applied command cleared; key repeat did not restart motion |
| Unsupported height/pitch | Limitation visible in the window; R/F did not generate a vertical command |
| Interface sequence | 14.60 sim s; 18.22 wall s including setup, capture and decoding |
| Level-0 up review | 15.95 sim s; reached x ≥5.4 m, then released and stopped |
| Tracking assistance | Actual tracker counter = 0 in both closed core summaries |
| Average core real-time factor | Interface 1.000; review 0.993 |
| Peak summed RSS | 1048.35 / 1047.85 MiB, including GUI, core, Xvfb and ffmpeg |
| Recorded video | 1320×900, 10 fps; 14.0 / 15.6 s; full decode passed |
| Protected installation and user settings | SHA256 unchanged before/after both runs |

验证使用 4 个 CPU 核和软件渲染，所有单次模拟时间、墙钟时间、内存与日志大小均在登记
预算内。上台阶展示仍为原始 2 cm 低难度场景，不能扩大解释为建筑楼梯验收。

Evidence is retained under `output/supermimic-display-20261029/`:

- `interface-01/` and `review-01/`: `review.mp4`, decoded-video metadata,
  `report.json`, `core-summary.json`, input-event timeline, raw IPC metadata,
  10 Hz odometry trajectory, frame digests and native screenshots.
- `profiles/`: per-terrain session JSON, original checkpoint/model/policy digests
  and preflight results. Existing frozen asset roots remain unchanged.
- `runs/`: isolated session settings, compiled model and bounded GUI/core event logs.
- `handoff.json`: launch command, binary/source/evidence hashes and retained paths.
- `prepare_profiles.py` and `verify_display.py`: offline preparation and independent
  display verification; neither is part of runtime control.

The display preflight initially identified missing `.vendor` references in the
external profile. Each display root now points to the existing local model
library. This fixes resource resolution without modifying the original models
or relaxing asset validation.

## Stair envelope raw tracing / 台阶范围原始记录

The separate preregistered envelope covers 4/8/12/16/20 cm, up/down and initial
yaw 0/−0.2/+0.2 rad: 30 cases. It retains the frozen actor, original v4 terrain,
50 Hz control and 200 Hz physics. Level-0 and the earlier matrix are not rerun.
Training, the default policy and installed desktop remain unchanged.

`supermimic_e2e` now accepts a manifest-driven case without duplicating the
controller. `--inspect` loads and audits inputs without stepping physics:

```bash
taskset -c 2,3 build/supermimic-20261029/supermimic_e2e \
  --inspect MANIFEST CASE_ID ASSET_ROOT NEW_INSPECTION_DIRECTORY
taskset -c 2,3 build/supermimic-20261029/supermimic_e2e \
  --case MANIFEST CASE_ID ASSET_ROOT NEW_CASE_DIRECTORY
```

The manifest supplies the source contract directory, clocks, velocity, distance,
time and original termination bounds. Each case cold-starts from complete
`qpos/qvel/act/ctrl`. At each control boundary it records the pre-action state,
requests the next command and consumes the preceding boundary's command.
This preserves the original 20 ms delay. It requests zero at 5.4 m progress,
then advances 100 control intervals, unless a fall, bounds violation, nonfinite
state or the 35 s budget terminates the case.

记录采用世界系位置、机身 link 坐标系速度；组合重力倾角超过 70° 或 XY 偏移超过
6.5 m 时停止。`goal_stop` 只表示走完到点后的两秒记录，不等于严格验收通过。
净空、姿态、横向偏移、速度误差与最后一秒停稳由科研侧共用分析器计算。

The imported model retains all original named parameter arrays. Static terrain
is additionally audited in world coordinates: count, shape, size, position,
orientation, friction and solver parameters. XML serialization rounded some
terrain values by about `1e-15`; preparation restores their original compiled
MJB local position/quaternion/size at 17-digit precision. The rejected import is
retained, and no physical case used it.

本轮索引位于 `output/supermimic-stairs-20261029/`：

- `manifest.json`, `batch-status.json`: frozen scope, queue status and resource use.
- `profiles/<kind>_l<level>/model-audit.json`: comparison against original MJB fields.
- `<case_id>/input-audit.json`, `initial-state.json`, `compiled-model.xml`: actual inputs.
- `<case_id>/trajectory.csv` and `trajectory.json`: 50 Hz pre-action states, requested
  and consumed commands, and termination flags; JSON also keeps full joint state.
- `<case_id>/terminal-state.json`, `summary.json`: final post-action state, raw reason,
  reached step and actual tracker counter. There is no generated strict-pass field.
- `prepare_profile.py`, `run_batch.py`: independent preparation and bounded sequential
  execution on CPU cores 2 and 3, with a 4 GiB aggregate RSS budget.
- `trace-integrity.json`, `handoff.json`: complete recording checks and source,
  binary, contract and output digests.

Raw batch status: 30/30 cases recorded, no unmeasured cases or runtime errors.
All 30 ended with `goal_stop` and actual tracker count zero. Every case's complete
initial state, one-tick command delay, sampling clock and 100-interval stop window
passed recording checks. All ten model imports matched 674 original named field
arrays; static world geometry and inspected solver options had zero numeric error
in all 30 cases.

The bounded queue took 477.62 wall seconds including waiting for upstream inputs,
recorded a 419.32 MiB peak aggregate RSS and retained about 76.8 MiB of artifacts.
Installed executables and existing user configuration digests were unchanged.
The XML-precision preparation failure remains under `preparation-rejected/`.

Final shared analysis accepts **60/60** cases (30 original, 30 SpRobo); the
independent evidence audit matches all **30 pairs** of initial states, models and
timing. The original research workspace retains the frozen analysis and audit.

Performance limitation remains: SpRobo `up_l5_h0` arrives at **23.58 s**, versus
**13.80 s** upstream, a **9.78 s** delay. It completes the course but does not
reproduce uniformly smooth tracking.

最终共用分析为 60/60 通过，30 对证据核对通过。但 20 cm 正向上台阶 `up_l5_h0`
在首阶附近 **4.52–10.30 s** 有明显低进度段，到点慢 **9.78 s**；属于完成但不顺畅，
不能描述为完美复现或平稳保持 0.5 m/s。此差异尚不能归因于单一组件，亦不代表实机验收。
原始负面结果保留在研究工作区；后续隔离结果见下节。


## Integration closeout / 接入收尾（2026-10-29）

This stage keeps the frozen actor and 3-DOF command contract. It does not change
training, install the candidate, or rerun the accepted 30+4/30-case matrices.
Evidence: `output/supermimic-integration-20261029/`.

### Production timing / 生产接口时序

The controller now latches commands at each 50 Hz boundary and consumes that
latch at the next boundary. GUI, ROS and direct core callers share this 20 ms
policy delay. An asynchronous request also waits for the next boundary; this
is not a guarantee of 20 ms from a keyboard event. The route harness no longer
adds another delay. Numerical fixtures restore the already-consumed command.

`command-contract.json` checks forty actual 5 ms core steps, including reversal
and release. `numerical-probe.json` replays 600 frozen forward/yaw-0.2 frames:
observation error ≤5.97e−8 and normalized action error ≤1.65e−7. Native keyboard
E2E (`ui-01/`) verifies W/S/A/D/Q/E consumption, release, Space and manual focus
loss, with zero tracker updates. Its 15.845 simulated seconds ran at RTF 1.000;
GUI/core/Xvfb/recording together peaked at 1056.9 MiB RSS. The screenshot and
video retain the visible core version/hash, controller and command limitation.

20 ms 延迟现在由控制器统一处理；测试入口不再重复延迟。核心短测和真实键盘检查均通过。
当前仍只接受平面速度与偏航角速度，不能据此声称新训练的高度／俯仰策略已经接入。

### High stair diagnosis / 高台阶差异定位

Four bounded runs use the same production controller, frozen weight, initial
state and 20 cm stair input. Only native MuJoCo version and post-step forward
cadence vary:

| Native engine | Forward every 5 ms | Forward every 20 ms |
|---|---:|---:|
| MuJoCo 3.11 | Arrival 13.72 s | Arrival 13.72 s |
| MuJoCo 3.15 | Arrival 23.58 s | Arrival 23.58 s |

All four end with `goal_stop`. `forward-diagnostic.json` and the four `cadence-*`
directories retain binaries, commands and trajectories. The 3.15 result exactly
matches the prior slow arrival. This narrows the difference to the engine-version
range and excludes forward cadence for this input; it does not identify an
individual MuJoCo internal change. The framework stays on 3.15, without gain or
weight changes. The slow first-stair segment remains a known limitation.

差异已缩小到 MuJoCo 3.11／3.15 的版本范围，排除了本例的补算频率影响。尚未定位引擎内部
哪一项变化导致差异，也未把现有框架整体降级。20 cm 高台阶仍属于可完成但不顺畅。

后续定位已完成：2026-11-04 的单项消融确认圆柱轮／箱体的多接触模型变化导致了接入差异。
按原 Warp 训练接触语义导出模型后，3.11／3.15 的旧265D策略爬阶轨迹逐元素一致，到达均为13.76s。
上述未定位状态为当时的历史记录。当前修复和4h策略结果见
[圆柱接触一致性修复](dual-physics.md)。

### Portable resources / 可搬迁资源

`tools/prepare_supermimic_bundle.py` exports the already-audited external assets.
The bundle contains 18 named profiles and shared meshes, frozen ONNX, scan data
and sensor visuals. Runtime launch uses Go/C++ plus a shell entry; Python is only
an offline exporter and independent test tool. Example preparation:

```bash
python3 tools/prepare_supermimic_bundle.py \
  --profiles output/supermimic-20261029/asset-roots \
  --stairs output/supermimic-stairs-20261029/profiles \
  --output /path/to/new/go2w-supermimic
SIMPLE_RBT_BUNDLE=/path/to/new/go2w-supermimic ./tools/run_supermimic.sh up-l5
```

`bundle-audit.json`: all 18 preflights pass after archive extraction into a path
containing a space; file digests match and symlinks stay inside the bundle. The
relocated 20 cm profile has zero static-world/solver difference. Archive size is
28,846,998 bytes; extracted file size is 100,351,371 bytes. The separate local
deb in `packages/` passes runtime-library, asset and relocation packaging checks;
it is a development candidate, not a published release. The package includes
`sprobo-supermimic`; its bundle is supplied separately.

The mapping profile replaces the upstream training terrain and removes only its
unused wheel-contact sensor definitions. Those definitions reference the removed
terrain body and are not actor inputs. The first startup failure and its exact
model remain in `noetic-fastlio-01/`; the Noetic test subscriber-argument mistake
is retained in `noetic-fastlio-02/`. Neither failure completed a navigation run.

资源已离开临时实验目录。新窗口使用独立配置与运行记录，不覆盖已安装程序或用户工作区。
资源包可搬迁；旧冻结证据保留。建图配置只移除了引用旧地形且不参与策略输入的训练用传感器。

### Live Noetic E2E / Noetic 全链路验收

`noetic-fastlio-03/score.json`: 140 s, 6/6 route goals, real
`/go2w/cmd_vel` commands → native SuperMimic → MID360/RGB-D → separate Noetic
adapter → FAST-LIO. Truth feeds only the route driver and independent scorer,
never FAST-LIO. Original 1 m / 5° gates and fixed-first-frame SE3 alignment remain.

| Measurement | Result |
|---|---:|
| FAST-LIO frames | 1,385 |
| Maximum / RMS position error | 0.012481 / 0.009116 m |
| Maximum orientation error | 0.116966° |
| Largest estimate gap | 0.100 s |
| Real-time factor | 0.999998 |
| MID360 scans / IMU samples | 1,400 / 28,000 |
| RGB and depth frames, each / camera IMU samples | 1,400 / 28,000 |
| Core / adapter / frontend average CPU cores | 0.156 / 0.055 / 0.326 |
| Core / adapter / frontend peak RSS | 663.0 / 112.6 / 148.2 MiB |

`sensor-audit.json` confirms every observed IMU interval is 5 ms with the centered
2.5 ms timestamp, and every LiDAR/RGB/depth interval is 100 ms. RGB remains
320×240 `rgb8`; depth remains 320×240 `32FC1`, both at the existing tutorial's
10 Hz. MID360's rigid IMU offset and rotation pass actual ROS TF checks.
The native controller records zero body-tracker and PD calls.

This is a CPU/software-rendering measurement on a shared host, restricted to
cores 2–5. Training was active at the initial audit; its owning session later
migrated and stopped it. Continuous training overlap was not measured. The
resource rows describe the three runtime process
groups, excluding the training and test driver. The separate GUI check is not a
simultaneous GUI+mapping benchmark. These results do not certify hardware,
relocalization or a new trained policy.

全链路通过，分辨率和传感器频率保持原教程配置。未持续记录训练是否并发；上述 CPU／RAM 是本次实测，
不能视为空闲主机基准。安装版及用户配置的散列未变。下一步由 Xian 试用独立入口；训练结束后
再单独验收新权重，避免把固定旧权重的结论套用到新策略。

## 2026-10-31 upstream-v2 duration comparison adapter

The original a299ed5 v2 Actor has245inputs (five49Dframes), unlike the retained
265D model above. The controller now supports explicit `actor_wheel_vel=false`;
the default stays true for backward compatibility. No installed application,
user configuration, physics engine or controller reward was changed.

Isolated build: `build/supermimic-v2-20261031`, targets `supermimic_probe` and
`supermimic_duration`. The latter runs the actual native Simulation/controller,
with zero tracker assistance, on the imported original geometry and initial state.
Four-hour and resumed-eight-hour policies will be evaluated after the current
four-hour continuation ends. This is inference testing, not training migration.

Interface evidence in the research workspace:
`outputs/diagnostics/duration_interface_smoke_20261031/preflight_complete.json`.
The250frame new-contract replay passes observation maxerror1.4305115e-6 and
normalized action error1.3137732e-6 against unchanged1e-5/2e-6gates. The600frame
legacy replay passes (5.9604645e-8/1.6999680e-7).574named physical fields, solver
options, static geometry and initial state match the original compiled model.
A550control flat smoke finishes without tracker assistance. These results are
interface checks, not complete terrain, braking, low-speed or real-robot approval.
Original MuJoCo3.11 versus SpRobo3.15 is explicit; identical trajectories are not
claimed. Import failures and their exact corrections are retained in that output.


## Lateral slider and low-speed tracking / 侧移滑块与低速跟踪（2026-11-04）

The native Slider's `Pressed()` requires the pointer to remain inside the
widget. Using it as a release condition cleared a held lateral command when the
pointer left the narrow slider rectangle. The console now uses the capture
boolean returned by `Dragged()`. Release and manual window blur still request
zero. Controller weights, gains, contact settings and sensor rates are unchanged.

侧移滑块原先把“鼠标离开控件”误当成“松开鼠标”，因此会在仍按住时清零侧移指令。
现在按指针捕获状态保持指令，松手或手动模式下窗口失焦才清零。

The real X11 mouse → console → IPC → C++ core E2E passes both signed drags,
leaving the widget while held, release, and manual blur. With the frozen four-hour
policy, the positive slider target `0.29814 m/s` gives mean `0.29855 m/s` inside
and `0.30217 m/s` outside the widget. The negative target `-0.29628 m/s` gives
`-0.28907` and `-0.29658 m/s`. End planar speed after release/blur is at most
`0.00414 m/s`. The actual consumed policy command stays constant after initial
application throughout each held phase. Tracker updates remain zero.

A separate native-core test holds commands constant on flat terrain and measures
body-frame lateral velocity over simulation seconds 4–8; motion begins at 2 s.

| Command vy / 指令（m/s） | Mean actual vy / 实际均速（m/s） | MAE / 平均绝对误差（m/s） |
|---|---:|---:|
| +0.30 | +0.29904 | 0.01572 |
| -0.30 | -0.30058 | 0.01754 |
| +0.10 | +0.08005 | 0.01995 |
| -0.10 | -0.07486 | 0.02748 |
| +0.03 | +0.00324 | 0.02676 |
| -0.03 | +0.00003 | 0.03003 |

Rotating the initial heading by 90° retains signed `±0.3 m/s` means of
`+0.29879/-0.29766 m/s`; this does not indicate an axis swap. Before the contact
compatibility correction, the `±0.1 m/s` means were `+0.08246/-0.07469 m/s`.
The weak small-command response therefore also exists with the previous contact
mode. Fixing slider capture does **not** certify low-speed tracking. No input
amplification or simulator feedback was added to mask that remaining limit.

The twelve core probes include two `±0.6 m/s` cases truncated by the reference
harness's lateral lane boundary; those are retained and are not full tracking
passes. The failed `Pressed()+Dragging()` API attempt is also retained: native
Slider capture is not a Draggable element's movement state. Setup failures and
the incomplete first capture are excluded from acceptance.

Evidence: `output/go2w-lateral-tracking-20261104/`, including `result.json`,
`source.patch`, per-case trajectories and the closed `ui-captured/` E2E records.
The existing isolated `run-candidate.sh` entry from the contact-compatibility
stage loads the rebuilt console on reopening. The installed desktop and current
user configuration were not replaced. Low-speed controller quality remains a
separate acceptance item.

## Noetic and low-speed comparison / Noetic 与低速对照（2026-11-09）

Using the same rebuilt core, scene, ROS 1 Noetic adapter and FAST-LIO image,
the 245-input four-hour policy completed 4/6 route goals in 140 s. The 265-input
policy completed 6/6. Each core generated 1,400 MID360 scans, 28,000 IMU
samples and 1,400 RGB/depth frames; their real-time factors were 0.99995 and 0.99993.
Maximum FAST-LIO position errors were 0.01361 m and 0.01242 m respectively.
The policies use different imported robot models, so this is a comparison of
the two complete controller/model combinations, not an isolated weight test.
The ROS subscriber connects after startup, so its received totals are lower;
its recorded sample periods meet the configured rates.

在相同的新核心、场景、ROS 1 Noetic 节点和 FAST-LIO 镜像下，245 输入的四小时
策略在 140 秒内完成 4/6 个目标；265 输入策略完成 6/6。核心每次生成 1400 帧
MID360、28000 条 IMU 和各 1400 帧 RGB／深度，实时系数分别为 0.99995 和
0.99993。FAST-LIO 最大位置误差分别为 0.01361 米和 0.01242 米。两套策略使用
不同的导入机器人模型，因此此结果比较的是完整的控制器／模型组合。
ROS 订阅器在启动后接入，实际接收总数较少；接入后的采样周期符合设定频率。

For the 245-input run, the fifth target remained about 0.096 m away. During
simulation seconds 105–139, the applied lateral command averaged +0.07518 m/s,
while mean world-frame lateral displacement corresponded to 0.000018 m/s.
Separate flat tests at ±0.03 m/s also show little movement with either policy;
the older policy is not an accepted low-speed replacement. All three tested
starting positions completed both the 20 cm ascent and descent, and three
slope starts completed, without simulator tracking assistance. These are
simulation tests and do not establish real-robot performance.

245 输入策略在第 5 个目标附近停在距目标约 0.096 米处。仿真第 105–139 秒，
实际应用的侧移指令均值为 +0.07518 米／秒，世界坐标下的平均横向位移速度约为
0.000018 米／秒。平地 ±0.03 米／秒对照中，两套策略都几乎没有移动；旧策略
不能作为已验收的低速替代方案。三个起点的 20 厘米台阶上、下行和三个坡道起点
均完成，未使用仿真速度跟踪辅助。这些结果仅适用于仿真。

Each center ascent, descent and slope case was also run for 22 s with direct
Noetic commands and the separate ROS adapter. Each crossed 5.4 m; the core
generated 220 LiDAR scans, 4,400 IMU samples and 220 RGB/depth frames per sensor. The
real-time factors were above 0.999; fixed-sensor frame and message-rate checks
passed. These three checks do not include FAST-LIO.

台阶上行、下行和坡道中间起点又分别使用 Noetic 直接控制运行 22 秒，均通过
5.4 米路线；核心每次生成 220 帧雷达、4400 条 IMU 和各 220 帧 RGB／深度。实时
系数均高于 0.999，固定传感器坐标系和话题频率检查通过。这三项未运行 FAST-LIO。

The mapping-bundle preparation removes contact sensors that reference the
replaced training terrain. The 245-input model has both wheel and non-wheel
terrain-contact sensors; these are not actor inputs. Robot bodies, joints,
actuators and solver settings are unchanged. Results and the
15-command lateral sweep are retained under `output/refactor-20261109/`.

建图资源包准备过程会移除引用旧训练场地的接触传感器。245 输入模型同时包含轮部
和非轮部接触传感器，均不作为策略输入；机器人模型和求解器参数保持不变。实验结果
和 15 档侧移测试保存在 `output/refactor-20261109/`。

A repeat with the final core binary completed all six route goals for both
policies. Maximum FAST-LIO position/angle errors were 0.0107 m/0.0501° for the
245-input policy and 0.0184 m/0.0912° for the 265-input policy. Both runs used
the same core, CPU physics and LiDAR, and native control. Host EGL selected a
hardware renderer for these runs. The earlier 245-input run
completed only four goals; this repeat does not establish reliable low-speed
tracking. The final core also passed the three direct Noetic terrain cases above.

最终核心的重复测试中，两套策略均完成六个路线目标。FAST-LIO 最大位置／角度误差
分别为：245 输入策略 0.0107 米／0.0501°，265 输入策略 0.0184 米／0.0912°。
两次均使用同一核心、CPU 物理与雷达计算和原生控制；宿主机 EGL 选择了硬件渲染。
245 输入策略上一轮只完成四个目标；
本次通过不能证明低速侧移已可靠跟踪。最终核心也通过上述三项 Noetic 地形测试。

With Mesa software EGL explicitly selected, the 245-input policy completed all
six goals in the 140 s Noetic/FAST-LIO run. Maximum position/angle errors were
0.01357 m/0.09821°, and the real-time factor was 0.99989. Three separate Noetic
terrain runs (20 cm ascent, 20 cm descent and slope) also passed at real-time
factors above 0.999. These runs do not remove the low-speed tracking limitation.

明确选择 Mesa 软件 EGL 后，245 输入策略完成了 140 秒 Noetic／FAST-LIO 测试的
六个目标。最大位置／角度误差为 0.01357 米／0.09821°，实时系数为 0.99989。
三项独立 Noetic 地形测试（20 厘米上、下台阶及坡道）也通过，实时系数均高于
0.999。这些结果不能消除低速侧移的限制。

An independent camera check found a pre-existing renderer-dependent depth error:
on this host's default hardware EGL, a frozen plane at 4.333 m was rendered at
4.304 m by both SpRobo and MuJoCo 3.15's official renderer. Mesa software EGL
matched the plane within 0.000001 m in this case. This does not establish the
cause inside the driver or a general accuracy bound for either renderer.

独立相机检查发现既有的渲染环境相关深度误差：本机默认硬件 EGL 将距离 4.333 米
的固定平面渲染为 4.304 米；SpRobo 和 MuJoCo 3.15 官方渲染器的结果一致。Mesa
软件 EGL 在该样例中的误差小于 0.000001 米。此结果不能确定驱动内部原因，也不能
代表任一渲染器在所有场景下的精度。

## Low-speed tracking and camera depth correction / 低速跟踪与相机深度修正（2026-11-09）

SuperMimic now supports the existing `tracked` mode. Its velocity loop uses
ideal simulated body velocity and heading, with a lateral integral gain of 1.6;
a complete zero command clears accumulated correction. The frozen actor,
actuator parameters and `native` mode remain unchanged. This is simulator
feedback, not a hardware velocity estimator or a newly trained policy.

SuperMimic 现在支持已有的 `tracked` 模式。速度环使用理想仿真机身速度和航向，
侧向积分增益为 1.6；全零指令清除已累积的修正。冻结策略、执行器参数和 `native`
模式不变。这是仿真反馈，不代表真机状态估计或重新训练的策略。

The flat E2E held each command from simulation seconds 2–8 and averaged body
velocity over seconds 4–8. For the 245-input policy, `native` commands
`+0.03/-0.03 m/s` produced `+0.0032/+0.0000 m/s`; `tracked` produced
`+0.0263/-0.0291 m/s`. The 265-input model's own flat keyframe produced
`+0.0264/-0.0246 m/s` in `tracked` mode. Signed commands from `0.01` to
`0.3 m/s`, zero command and stopping passed the recorded acceptance checks.
The actual X11 slider also passed both low-speed directions, pointer capture,
release and focus loss. When the lateral command was released while moving
forward, lateral velocity settled near zero without stopping forward motion.

平地端到端测试在仿真第 2–8 秒保持指令，统计第 4–8 秒的机身速度。245 输入策略
在 `native` 模式下，`+0.03/-0.03 米/秒` 指令产生 `+0.0032/+0.0000 米/秒`；
`tracked` 分别为 `+0.0263/-0.0291 米/秒`。265 输入模型使用自身平地初始状态时，
`tracked` 分别为 `+0.0264/-0.0246 米/秒`。正反 `0.01–0.3 米/秒`、零指令和停车
通过记录中的验收。真实 X11 侧移滑块的正反低速、指针捕获、松手与失焦也通过。
前进时单独松开侧移指令，侧向速度归零，前进保持。

At `+0.01/-0.01 m/s`, the 245-input policy averaged `+0.0063/-0.0075 m/s`;
the lowest speeds therefore still have noticeable error. Ablating higher
lateral integral gains reduced some mean errors, but increased oscillation:
with gain 12 the 265-input policy reached about `0.134 m/s` mean absolute
error during the `-0.3 m/s` test. The validated gain remains 1.6; improving
the lowest-speed response further requires policy or control work.

指令为 `+0.01/-0.01 米/秒` 时，245 输入策略的平均速度为
`+0.0063/-0.0075 米/秒`，极低速仍有明显误差。增大侧向积分增益的消融实验
降低了部分平均误差，但增加了波动：增益为 12 时，265 输入策略在
`-0.3 米/秒` 测试中的平均绝对误差约为 `0.134 米/秒`。因此保留已验证的 1.6；
要继续改善极低速，需要进一步调整策略或控制方式。

Three start positions each on 20 cm ascent, 20 cm descent and slope passed in
direct core simulation with tracked control; the centre cases also passed
Noetic with MID360, IMU, RGB and depth. A separate 140 s Noetic/FAST-LIO route completed 6/6 goals,
with maximum position error `0.01356 m`, angle error `0.0954°` and real-time
factor `0.99993`. The frontend did not receive the simulator truth used by the
independent route driver and scorer. These results do not establish real-robot
performance.

20 厘米台阶上行、下行和坡道各三个起点通过核心直接仿真，使用 `tracked` 控制；
中间起点还通过 Noetic 雷达、IMU、RGB 与深度测试。独立的 140 秒 Noetic／FAST-LIO
路线完成 6/6 目标，
最大位置误差 `0.01356 米`、角度误差 `0.0954°`、实时系数 `0.99993`。独立路线
驱动器和评分器使用仿真真值；建图前端没有接收该真值。这些结果不证明真机性能。

The hardware depth discrepancy came from multisample depth resolving at a
subpixel location. The depth/segmentation pass now reuses MuJoCo's existing
single-sample resolve framebuffer; RGB keeps its original multisampling. In the
same frozen scene, 663 floor-ray samples on the tested hardware improved from
`0.01898 m` mean and `0.26923 m` maximum absolute error to `0.000042 m` and
`0.000450 m`. The independent inclined-floor regression had `0.000299 m`
maximum error on hardware and `0.00000265 m` with Mesa software rendering.

硬件深度偏差来自多重采样深度解析时选择了像素内偏移的采样点。深度／分割绘制
现在复用 MuJoCo 已有的单采样 resolve 缓冲；RGB 保留原多重采样。同一冻结场景中，
测试硬件上 663 个地面射线样本的平均／最大绝对误差由 `0.01898/0.26923 米` 降至
`0.000042/0.000450 米`。独立倾斜地面回归的最大误差在硬件上为 `0.000299 米`，
在 Mesa 软件渲染上为 `0.00000265 米`。

The repeatable external-fixture comparison is:

可使用以下命令复现外部资源的原生／跟踪模式对照：

```bash
python3 tools/test_supermimic_tracking.py \
  --harness /path/to/supermimic_duration --profile /path/to/flat-profile \
  --contract /path/to/contract.json --output /path/to/new-output
```

Evidence / 证据：`output/quality-fix-20261109/`, including
`gain-and-tracking-history.json` for the gain ablations.
增益消融结果见其中的 `gain-and-tracking-history.json`。
