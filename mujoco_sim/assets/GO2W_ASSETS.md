# Go2W controller and asset provenance

The `go2w` simulator option uses the public rl_sar RobotLab controller as an
experimental deployment adapter. It does not change or replace f1reRL's
canonical LLC, retained checkpoints, or historical gate decisions.

- Controller source: https://github.com/fan-ziqi/rl_sar/tree/36bac3930bca95239de9bcfade283832e6ee4409
- Separate model source: https://github.com/fan-ziqi/rl_sar_zoo/tree/d4cca961c70c9265e92a58639f1fe9aaccb2ad0f
- Exact source paths, sizes and hashes: `go2w_manifest.json`.
- Runtime ONNX: `go2w_policy.onnx`; derivation and numeric comparison:
  `go2w_policy.export.json`. Original TorchScript SHA-256 is
  `eda0c490dfc75428fb6706d50924ebe2a91fca40ef21fae8747565699c81a2ac`.

The minimal local bundle has 22 files / 32,839,932 bytes, including only the
17 meshes referenced by the native MJCF. The ONNX file adds 785,063 bytes.
No Torch package is needed in the simulator environment. The controller and
configuration carry rl_sar's Apache-2.0 license, retained in `RL_SAR_LICENSE`
and in the local bundle.
No root license was found in the pinned separate model repository; asset/CAD
provenance remains unresolved, and those meshes remain ignored local downloads.

Rebuild or verify the local bundle from either pinned public URLs or existing
Git object stores:

```bash
.venv/bin/python mujoco_sim/scripts/bootstrap_rl_sar_assets.py
# Offline equivalent, with existing source checkouts:
.venv/bin/python mujoco_sim/scripts/bootstrap_rl_sar_assets.py \
  --source-repo /path/to/rl_sar --zoo-repo /path/to/rl_sar_zoo
```

For a deliberate one-time ONNX re-export, use an existing CPU Python environment
with Torch, ONNX and ONNX Runtime; run `scripts/export_go2w_policy.py`. This is
not a setup requirement. The retained export used Torch 2.7.0 and opset 17.
The export comparison reports absolute and normwise float32 differences rather
than claiming bit-identical inference. At synthetic observation scales 1/10/100,
maximum raw output differences were 0.0000363/0.000248/0.001648, respectively.

The input contract is 57 values: angular velocity x0.25, projected gravity,
three velocity commands, 16 relative joint positions with wheel slots zeroed,
16 joint velocities x0.05, and 16 previous actions. Output is 12 scaled leg
position offsets and four wheel velocity targets. Policy runs at 50 Hz, native
PD at 200 Hz, physics at 500 Hz. The 200 Hz deadlines alternate two/three
physics steps. Native MJCF motor limits and policy gains are retained; no
feed-forward copy of PD torque is added.

Initialization is a ready-standing keyframe (front thigh 0.5 rad, rear thigh
0.8 rad, calf -1.5 rad, base Z=0.45 m). It does not replay the upstream
three-second GetUp FSM, so current runs are not an exact reproduction of the
old native gate. Ideal sensor sites add no mass or collision geometry.

Known behavior limits: the canonical historical native gate measured 0.219321 m
zero-command drift over 10 seconds and failed its 0.20 m parking threshold.
The new ready-standing integration likewise measured 0.216228 m, although it
remained upright. Its fixed 0.5 m/s command tracked about 0.458 m/s on flat and
0.463 m/s on the 8-degree ramp, but slow heading drift left the 2.4 m corridor
before the finish line; these are partial traversal observations, not completed
course acceptance. Current evidence is in
`output/validation-20261013/go2w-integration-probe.json`. Public source terrain
settings do not certify this particular checkpoint's training or stair success.

Optional `--control-mode tracked` adds bounded velocity/heading feedback around
the unchanged policy. It uses ideal simulator body velocity and heading, and
changes only the three velocity commands. This is a simulator control aid,
not native-policy performance, a hardware estimator, or a repaired historical
LLC gate. `--control-mode native` retains the behavior described above.

The flat-terrain command check used fresh model/data/controller state for each
case: 2 seconds of settling, 8 seconds of command, then 3 seconds at zero command.
The following are measured mean body velocities over the last 4 commanded
seconds; parking drift is the maximum planar displacement during the 8-second
parking phase, after settling. All three configurations used the same tracker,
commands and declared thresholds; their frozen learned policies are distinct.

| Robot / policy, all tracked | In-place yaw, target 0.5 rad/s | Lateral, target 0.3 m/s | Reverse, target -0.3 m/s | Combined vx / wz, target 0.3 / 0.3 | Parking drift |
|---|---:|---:|---:|---:|---:|
| Go2 / HIM | 0.0029, failed | 0.2040 | -0.2977 | 0.2984 / 0.2632 | 0.013 m |
| Go2 / RobotLab | 0.5129 | 0.2969 | -0.2976 | 0.2996 / 0.3258 | 0.001 m |
| Go2W / rl_sar RobotLab | 0.4972 | 0.2990 | -0.2988 | 0.2971 / 0.3079 | 0.028 m |

Go2W and Go2 RobotLab passed all five listed cases. Go2 HIM's in-place turn
failed while its command saturated for 92.1% of that phase; its lateral speed
only narrowly met the declared mean-error limit and its correction saturated
for 45.3% of that phase. The broad mean-error limits were 0.10 m/s for each
linear component and 0.15 rad/s for yaw; the reports also retain instantaneous
MAE/RMSE, displacement and saturation, rather than implying precise tracking
from a passing average alone.

All cases remained upright and met the final stopping checks: during the last
second of the 3-second stop, mean planar speed was below 0.08 m/s and mean
absolute yaw rate below 0.10 rad/s. Go2W's largest observed extra travel during
stopping was 0.0563 m, and its largest final mean planar speed was 0.0140 m/s.
These flat-scene checks do not establish turning on stairs, reverse terrain
traversal, other command magnitudes, long-duration reliability, or sensor-loaded
real-time performance.

Evidence: [Go2 HIM and Go2W motion measurements](../../output/validation-20261013/planar-motion.json),
[Go2 RobotLab motion measurements](../../output/stage2-20261013/planar-motion-go2-robot_lab.json).
Reproduce into a new output file using `scripts/check_motion.py`, selecting
`--go2-controller himloco` or `--go2-controller robot_lab` explicitly. The Go2W
option always uses its separate pinned `rl_sar` adapter and checkpoint; selecting
a Go2 policy does not replace the Go2W policy.
