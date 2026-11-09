# Physics backends and bounded training / 双物理后端与有界训练

The native interactive runtime uses MuJoCo/C++. Optional Warp batch jobs run in
an independent existing Python/mjlab environment. SpRobo selects and supervises
the job; the native core and default desktop need no Python interpreter.

原生交互继续使用 C++ MuJoCo。GPU 批量任务使用独立的 mjlab／MuJoCo Warp 进程，
由 SpRobo 统一选择和启动。当前 GPU 路径用于批量物理与训练流水线烟测。

## Entry / 入口

Build the candidate with the existing setup and CMake workflow. No global driver
or Python dependency upgrade is needed for an already prepared external stack.
For example, a CPU-only native build still supports an optional external Warp job:

```bash
cmake -S . -B build/dual-physics -DSIMPLE_RBT_CUDA=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build build/dual-physics --target simple-rbt-sim simple_rbt_console simple-rbt-physics-contract -j4
build/dual-physics/simple-rbt-console --physics-job /path/to/job.json
```

The job window selects `mujoco` or `warp`, then `batch` or `train-smoke`.
It shows the actual backend/device/library versions returned by the job and its
evidence directory. `train-smoke` requires Warp. Headless use:

```bash
build/dual-physics/simple-rbt-console --physics-job /path/to/job.json \
  --physics-backend warp --physics-task batch --job-headless
build/dual-physics/simple-rbt-console --physics-job /path/to/job.json \
  --physics-backend warp --physics-task train-smoke --job-headless
```

A job JSON uses explicit external resources; the framework does not silently
select a different policy or install an environment:

```json
{
  "format": "sprobo.physics-job.v1",
  "assets_root": "/path/to/four-hour/profile_l5_four_hour",
  "session_config": "/path/to/session.json",
  "python": "/path/to/prepared/venv/bin/python",
  "reference": "/path/to/go2w-supermimic-a299ed5-pristine",
  "checkpoint": "/path/to/model_20500.pt",
  "device": "cuda:0",
  "worlds": 4,
  "policy_steps": 300,
  "training_worlds": 16,
  "training_iterations": 32,
  "wall_limit_s": 600,
  "output_root": "/path/to/new-job-records"
}
```

Each invocation gets a fresh output directory. The native exporter resolves the
session's scene/spawn, writes the compiled MJCF contract and native control trace,
then the Warp process reads that contract. `physics/warp_batch.py` delegates actual
physics and subset reset to `mjlab.sim.Simulation`; `physics/train_smoke.py` injects
the shared environment into the unchanged upstream `train.main`. Learner, replay,
differentiable imagination and optimizers are upstream implementations.

任务窗口支持真实后端选择和状态查看。此入口有界运行，不会自动延长训练。完整的训练课程、
大批量调度和 GPU 传感器／ROS 交互不属于当前最小版本。

## Backend names / 后端命名

| Interface | Meaning |
|---|---|
| Native `--physics-backend mujoco` | Native MuJoCo CPU physics |
| Native `--sensor-backend cpu/cuda/auto` | LiDAR computation only |
| Legacy native `--backend warp` | CUDA LiDAR alias; physical solver stays CPU |
| SpRobo job `--physics-backend warp` | Actual independent CUDA Warp physics |

New console configuration writes `sensor_backend`; legacy `backend` JSON remains
accepted. Native sessions reject `physics_backend: warp` with the independent-job
entry in the error. Native metadata and summaries report actual physics backend,
device, version and sensor backend. GPU reports record MuJoCo, MuJoCo Warp, Warp,
mjlab and device identity. Sensor CUDA is optional at compile time, independently
of the external Warp environment.

`--backend warp` 的旧含义保持兼容。它不表示 GPU 物理。选择真实 GPU 物理应使用上述任务入口。

## Shared contract / 共享契约

`simple-rbt-physics-contract ASSET_ROOT SESSION_JSON NEW_OUTPUT` exports:

- The resolved MJCF, model/policy digests and native compiler version.
- Canonical joint/action names and permutation, default positions, action scales,
  actuator target units, force/control limits, armature, damping and collision data.
- SI units, right-handed Z up, WXYZ quaternions and body-frame `vx/vy/yaw_rate`.
- 5 ms physics, 20 ms control, one policy-boundary command delay. Asynchronous
  input also waits for a control boundary.
- Current actor: 49 values/frame ×5 term-wise chronological frames =245 inputs,
  16 tanh actions. Terms: angular velocity3, gravity3, relative leg positions12,
  leg velocities12×0.05, previous actions16, command3. Normalization stays in the
  frozen actor. This actor has no measured wheel-velocity, height or pitch input.
- Reset state, zero previous action and command latch, repeated first history,
  finite-state/70° tilt termination and a bounded 300-control-step batch task.

The same-state comparison gate is observation ≤1e-5 and normalized action ≤2e-5.
Physics arrays use ≤1e-6 tolerance to allow GPU float32 representation. Visual-only
mesh bounds can differ between compiler versions and are reported separately;
collision sizes and physical coefficients keep the original gate. CPU and GPU
floating trajectories need not be elementwise identical. The existing Warp stack
also warns that cylinder/cylinder and cylinder/box MULTICCD pairs generate at most
one contact; this is a backend limitation, not an adjusted contact coefficient.

Training uses the same analytic static geometry, robot inertials, named joint and
actuator parameters, solver settings and action scale. The current bounded adapter
uses existing flat rewards/H4, no domain randomization or curriculum, and fresh
Adam with the four-hour Actor weights. It is not a matched-Adam continuation or a
reproduction of the complete original terrain curriculum. The GPU batch contract
uses explicit subset reset; the existing trainer retains its documented manager
reset/termination behavior. Future task adapters must declare such differences.

共享模型与控制接口已经验收；相同轨迹逐值一致不作为跨版本门槛。当前训练烟测使用新建 Adam，
不能视为四小时优化器状态续训，也不能据此接纳其输出策略。

## Evidence / 验收记录（2026-11-04）

Local stage: `output/dual-physics-20261104/`, isolated candidate
`build/dual-physics-20261104/`. Existing integration/v2 binaries, the four-hour
ONNX and Xian's running interactive session are protected.

Frozen ONNX SHA256:
`9e1b130bc3c7d0a8fa214adff2b72f5da7fc3e50bd66cf45cda9cfe35a596e9c`.
Checkpoint20500 SHA256:
`0cf37f4497fd72e2d4c8fe1dc9e5fd692f68895efe9b8c8c148f22ecf7ea4500`.

| Check | Result / evidence |
|---|---|
| Native CPU | MuJoCo3.15;300controls, finite/upright; `jobs/run-378416195/shared/` |
| Real GPU batch | CUDA0 on RTX5060Ti;4worlds×300controls×4substeps=4800steps; `jobs/run-2067474582/batch/` |
| Same-state observation / action max error | 2.98e-7 /4.33e-7; frozen ONNX vs Torch3.88e-7 |
| Model / reset | Physical-array error0; all subset-reset gates pass |
| Versions | MuJoCo/MuJoCoWarp3.11;Warp1.18;mjlab1.6 |
| Batch resources | RSS1938.4MiB;Torch allocator12.04MiB;Warp pool19.43MiB;4.38s including setup/audit |
| Upstream training | 512physical transitions;29WM/28Actor/28critic optimizer steps; `jobs/run-4212022801/train-smoke/` |
| Training failures | Zero falls, invalid resets and skipped updates; actual finite H4 imagination losses |
| Trainable-weight max deltas | Actor3.28012e-5;critic0.00528449;WM0.00610102; `train-parameter-audit/report.json` |
| GUI backend selection | Real X11 mouse events;CPU then Warp batch pass; `ui/events.json`, screenshots |

Allocator figures exclude CUDA context/driver allocations; they are not total
process VRAM. RSS is process high-water. The runtime is measured on a shared host,
not an idle-system benchmark. No GPU interactive rendering/sensor performance is
claimed by the batch run.

The original training report's three `parameter_max_deltas=512` included
normalizer buffers. It is superseded by `train-parameter-audit/`: only deterministic
initialization was reconstructed, stopping before sampling/optimization; final
trainable tensors are compared by `named_parameters`, with buffer deltas separate.
No physical steps or optimizer updates were repeated for this correction. Future
smokes save initial network parameters/buffers before updates.

Initial batch failures are retained: boolean subtraction in the audit, visual-only
mesh bounds mixed into the physical-size check, then an incorrect diagnostic
constraint-force member name. All occurred in new adapter/audit code and were fixed
without retuning robot coefficients. GPU native contact constraints are stored
from `efc.force`, with world/geom IDs and addresses. `actuator_force` is kept apart
from `ctrl` target values.

## Additional force trace / 20 cm 抽动补测

`force-trace-registration.json` registers one 20s/4000substep CPU run, frozen4h,
original level5 `sprobo_contract.json` row4. Initial qpos matches exactly. After2s
zero command, vx stays0.5m/s; wall limit120s, RSS2GiB, output100MB. It completes
without tilt/nonfinite/resource termination; payload≈17MiB.

Every5ms retains pre-state, integrated qpos/qvel, target `ctrl`, native
`actuator_force`, generalized actuator force and wheel/nonwheel contacts with
MuJoCo contact-frame forces. Force samples are **after `mj_step` + `mj_forward`**:
instantaneous solve at the integrated state, not interval-average or step impulse.
Raw actuator order is identified by the schema, not assumed to be canonical.

`force-trace/force-analysis.json`: overlapping0.5s windows with dx≤0.02m during
vx0.5 cover7.495s. Only one low-progress sample reaches99% of an actuator force
limit; there is no repeated saturation pattern. Nonwheel-ground contact force
is0. Wheel-ground normal peak397.99N (progressing intervals292.48N). Canonical
leg-target maximum jump is1.047777rad. This evidence does not establish a unique
cause of the observed oscillation by itself. The subsequent contact ablation below
isolates the deployment mismatch without changing rewards, gains or policy weights.

本条补测没有支持“持续力矩饱和”或“腿撞台阶”的判断。存在目标跳变和轮地接触力波动；
采样相位与单条路线限制仍需保留。它不替代已完成路线矩阵，也不构成实机验收。

Offline figures are in `force-trace/figures/`: motion overview, all16joint
targets/measurements, all16actuator torques/limits, a target-step closeup, and the
historical engine comparison. A five-page PDF, full-rate CSV and `metrics.json`
accompany them. No simulation or training was repeated; input hashes remain unchanged.
The four-hour trace's body-forward velocity reaches −0.58236m/s despite the
positive command. At14.385s the RR calf target changes60.033°, while the actual
joint changes0.453° over the adjacent5ms. This target step occurs outside the
low-progress windows and does not establish their cause. The historical
13.72/23.58s engine comparison uses the older265D policy, separately from this245D trace.

`force-trace/oscillation-analysis/` compares existing paired four-hour level5
nominal trajectories, with no new physics. Original Warp reaches the goal at13.30s,
native SpRobo at21.58s. Under identical50Hz stair-region/window definitions,
low-forward-progress unions are0/7.10s, while leg-target step RMS is5.631/5.638°.
Large target updates alone therefore do not identify the problematic behavior.
The native5ms trace matches all1000saved50Hz base positions exactly and leg
positions within6e-17rad. Reconstructed original servo equations match recorded
post-forward actuator force within8e-15N·m. Backend and MuJoCo version vary together
in that paired record. The following CPU-only ablation removes this confound.

## Cylinder contact compatibility / 圆柱接触一致性修复（2026-11-04）

The registered20cm failure is traced to contact-model compatibility. The pinned
Warp3.11 training stack supports at most one contact for cylinder/box wheel pairs.
Native MuJoCo's cylinder multicontact implementation changed in3.13
([upstream change](https://github.com/google-deepmind/mujoco/commit/c9d9976101da9f1ab8bd5d93bba0fa63f942f358)).
Importing identical actuator/geometry numbers with default `multiccd` therefore
did not preserve the training contact behavior. This is an import-contract gap;
the tests do not establish a general defect in MuJoCo's newer contact algorithm.

One-variable CPU experiments use the same production C++ controller,4h policy,
initial state, solver coefficients,5ms physics and50Hz control:

| Native engine | Default multicontact arrival | Single-contact arrival |
|---|---:|---:|
| MuJoCo3.11 | 13.36s | 13.04s |
| MuJoCo3.15 | 21.60s | 13.04s |

Arrival uses the actual completed physics timestamp; the older zero-index report
is20ms earlier. Under single contact, both engines produce exactly equal qpos,
qvel, control targets, actuator forces and constraint forces throughout all3008
substeps. The same equality holds for the two lateral initial offsets, descent,
flat ground and the retained265D policy. On3.15, stair-region low progress falls
from7.10s to0 using the same50Hz/window definition.

The first default-mode divergence is the FR cylinder wheel versus terrain box:
at input time0.095s both engines have identical qpos/qvel, warmstart, controls and
applied torque, but generate3versus1contacts. Constraint forces then differ,
followed by state and policy feedback. Fixed-control replays retain the physical
divergence; replay under the matching single-contact model reproduces the complete
trajectory exactly. All1180recorded policy frames, including the rocking interval,
pass upstream CPU feature/history/Actor parity: observation error0, normalized
action error≤8.86e-7 under the existing2e-6gate.

`prepare_supermimic_bundle.py` now writes `<flag multiccd="disable"/>` into these
pinned external-policy models and records the setting in the bundle manifest.
Physics exports include both option-flag masks; the external-model audit rejects
a missing contact flag. Other robot models and the controller's gains remain
unchanged. All18prepared profiles compile and their113file hashes verify.
The actual C++ core passes ascent, descent and flat E2E with zero tracker updates.

证据：`output/go2w-oscillation-cause-20261104/result.json`。修复的是导入时遗漏的接触模型
条件。仅比较关节波形幅度无法定位该问题；同状态接触分歧、单项消融和真实核心验收构成因果证据。
候选入口为该目录下的 `run-candidate.sh`，使用独立资源与配置。旧窗口、原权重及失败记录保留；
本阶段没有重新训练、升级依赖、替换安装版或重跑原完整矩阵。

## Isolation correction / 固定配置隔离修复

Native keyboard checks passed, but the first new direct `--fixed-config` check
also exposed a pre-existing isolation hole: only bundle entry isolated XDG config.
The test wrote global workspace/window state. All fixed-config entries now isolate
before workspace initialization. The first failed protection report remains under
`native-ui/`. Saved robot/scene drafts and configuration were restored from the
retained upgrade snapshot; window state matches its original byte hash. Workspace
byte identity was not recovered; `isolation-recovery.json` records this explicitly.
The corrected test saves original protected-file bytes before execution.
`native-ui-fixed/report.json` passes all31checks, including unchanged settings
after the fix,20mscommand consumption and actual MuJoCo/CPU identity. Its GUI,
core and recording peakRSS is1041.4MiB across19.17wallseconds.
`package-audit.json` also verifies the extracted deb can execute the native
CPU job with an intentionally missing Python path and a real Warp batch from its
own installation prefix. Both pass; no installation is performed.

This incident adds a concrete rule to this entry: fixed external profiles own
separate preferences/workspace/window state. Ordinary editable workspaces retain
their established behavior. No installed binary is replaced by this stage.

The later [lateral-slider and low-speed checks](go2w-supermimic-validation-20261029.md)
fix pointer capture independently; contact compatibility does not certify low-speed tracking.

## Captured EPA failure / EPA 故障重放（2026-11-05）

`physics/warp_epa.py` supplies `install_epa_face_filter()` for the pinned
MuJoCo Warp 3.11.0 backend. Call it once before constructing mjlab Simulation
and capturing CUDA graphs. An existing Simulation must recapture its graphs
after activation; changing Python factories does not update captured kernels.
The returned metadata identifies the active factories. Installed dependencies,
model coefficients and solver settings are unchanged. Activation is explicit;
this stage does not wire the correction into a running trainer.

The captured 1536-world forward input reproduces nonfinite qacc in world1130.
Its FL cylinder/terrain-box contact lies57.916m from the wheel center. Global
contact IDs, worldid, geom IDs and GPU world geometry frames agree. The selected
EPA face has a plane projection outside its triangle; its barycentric weights
are approximately[-1430056,121,1429936]. Interpolating witnesses from that face
extrapolates outside both geoms. Merely evaluating the weights in float64 does
not repair that geometric error.

The correction only requires the closest candidate face's projection to lie
inside its triangle. Both actual project-module replays retain the two contact
pairs and18constraints in world1130, report no overflow and finite qacc in all
1536worlds. The FL contact is0.089810m from its wheel center and approximately
10micrometers from the CPU3.11 reference contact. No Newton-precision patch is
applied; instrumented finite Newton traces do not establish an indefinite H.

Evidence: `output/warp-a8-37215431-diagnostic/`, especially
`contact-project-module/report.json`, `native-collision.json` and `result.json`.
The archive omits nested opt/stat objects and original scratch/graph history.
Restoration verifies213nonempty top-level model arrays and saved inputs;
meaninertia is recomputed, not claimed to match an archived value. This is a
single frozen-failure result, not a full training or terrain acceptance result.

已验证的是一个冻结故障及其接触几何修复。正式训练集成、续跑与后续验收由训练会话
负责。本阶段不训练、不改依赖，不替换桌面安装版。GPU 脚本计时合计约140秒。
