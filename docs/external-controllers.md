# External CPU controllers / 外部 CPU 控制器

`external` loads an application-owned shared library. The application owns its
observation conversion, history, policy architecture, inference and target
mapping. SpRobo owns the existing model, scene, CPU physics, sensors and display.
There is no fixed policy input dimension or fixed actuator count in this entry.

外部项目负责观测、历史、网络与目标转换；框架不解释策略的输入维度，不增加 PD 或速度反馈。
现有 SuperMimic 与其他内置控制器继续走原工厂入口。

## Resource configuration / 资源配置

In an external resource root, add a model alias and controller to
`assets/catalog.json`. The model must retain the usual SpRobo sensor sites; an
existing compatible imported model can be referenced without copying it.

```json
{
  "models": {
    "go2w-external": {
      "file": "models/go2w.xml",
      "sha256": "MODEL_SHA256"
    }
  },
  "controllers": {
    "go2w-external": {
      "library": "plugins/libcontroller.so",
      "period_s": 0.02,
      "config": {
        "policy": "/absolute/path/student.onnx"
      }
    }
  }
}
```

These are entries to merge into a complete catalog, not a replacement for its
other required sections. `library` is relative to the resource root or absolute.
The inner `config` object is opaque JSON passed to the plugin; its path rules
belong to the plugin. Use absolute paths there unless the plugin defines another
base. The existing `policy`/`policy_sha256` controller fields may additionally
identify a frozen policy for normal asset checks and deployment metadata.

Select `controller: "external"`, `control_mode: "native"` in a session JSON.
The existing console discovers controller choices from the catalog. Its
`--assets RESOURCE_ROOT/assets --config SESSION_JSON --fixed-config` entry can
use this profile. Direct headless execution:

```bash
build/dual-physics-20261104/simple-rbt-sim \
  --project-root RESOURCE_ROOT --robot go2w --controller external \
  --control-mode native --sensor-backend cpu --sensors physics \
  --duration 10 --cmd 0.3 0 0 --output NEW_OUTPUT
```

The callback runs synchronously before physics on the first tick and then every
`period_s`; this period must be an integer multiple of the model timestep.
Other physics ticks hold `data->ctrl`. Commands retain the existing robot limits
and body-frame `vx/vy/yaw_rate` convention. The framework adds no command-history
delay; any policy-specific latch/history belongs to the plugin.

## Plugin ABI / 插件接口

Include [controller_plugin.h](../include/simple_rbt/controller_plugin.h) and
export `simple_rbt_controller_create_v1`. Compile with the same MuJoCo SDK as the
runtime. The factory receives `mj_version()` so it can reject a different SDK
layout. A plugin instance belongs to one Simulation; a new session creates a new
instance. Do not retain borrowed configuration text after creation.

- `update(context, model, data, command, controls, error, capacity)` reads raw
  MuJoCo state and writes **all `model->nu` controls**, in model actuator order.
- `after_step(...)` is optional and runs after **every** `mj_step`, before the
  framework's `mj_forward`. It can record `data->qfrc_actuator[0:model->nv]`,
  `actuator_force[0:model->nu]`, contacts and the native integration result.
- `destroy(context)` releases application state when the Simulation closes.

Callbacks return zero on success. On failure, return nonzero and write a
NUL-terminated message into the error buffer. The runtime stops with that error;
C++ exceptions must not cross the C ABI. The application must release partial
state itself if creation fails. Model/data pointers are borrowed and read-only;
only the supplied control output may be written.

`controls` means native actuator inputs, **not universally joint positions**.
For position/velocity servos, values are targets in rad/rad/s and the model owns
gains, force limits and armature. Torque actuators take their native torque
inputs; their target-to-torque conversion belongs to the external application.
Use `actuator_trnid`, joint names and dof addresses rather than assuming an order.

对于 16 执行器模型，插件输出 16 个值。框架不检查 228 维等策略输入，也不重排关节。
四元数顺序、坐标系、归一化与观测历史均由外部适配器按原策略契约转换。

## Force phase and video / 力的采样阶段与视频

An embedding host can use `Simulation::set_physics_step_observer` for **any**
controller, including an unchanged SuperMimic baseline:

```cpp
sim.set_physics_step_observer([&](const mjModel *m, const mjData *d) {
  // Record d->time and qfrc_actuator[0:m->nv] in the caller's bounded recorder.
});
```

This receives the same native step phase as the plugin's optional `after_step`,
so both policies can use one force-recording path. The callback observes borrowed
read-only state; it must not step or forward the model.
Install it before stepping, for example in `RunObserver::configure`; keep any
captured recorder state alive until it is cleared or the Simulation closes.

At `after_step`, `qpos/qvel/time` have been integrated, while force/contact
fields are those left by native `mj_step`. Record that exact phase. These are
native generalized forces, not impulses or interval averages. The callback
must not call `mj_forward`, which would replace those force fields. Ordinary
`Simulation::data()` reads after `Simulation::step()` see the subsequent forward
solution and therefore represent a different sampling phase.

Video can reuse [CameraRig](../include/simple_rbt/cameras.hpp): after a normal
`Simulation::step()`, `render_observer()` returns top-down, tightly packed RGB24.
A host using [RunObserver](../include/simple_rbt/runtime.hpp) can render in
`before_step()` from the previous fully forwarded state, scheduling frames by
`sim.stamp_ns()`. Keep the rig on its creating thread. An external recorder may
pipe these frames to the existing CPU encoder, for example:

```bash
ffmpeg -f rawvideo -pixel_format rgb24 -video_size 640x480 -framerate 10 \
  -i - -c:v libx264 -threads 2 -pix_fmt yuv420p review.mp4
```

Use the same frame size, frame rate, scene, model state, timing and command
sequence for both compared policies. The console view/IPC and existing X11
recording path are also reusable; this interface adds no separate video engine.

## Prototype evidence / 原型记录（2026-11-05）

`output/external-controller-20261105/` retains the example shared library,
source, catalog and actual native runs. It supplies 16 fixed native controls and is **not**
a trained policy. One second at 5 ms physics / 20 ms control records 200 force
rows and 50 controller calls. A 10 ms control run records 20 physics rows and
10 calls over 0.1 s. Commands pass through, values are finite, and framework
tracker/PD calls remain zero. The public header also compiles as C.

A separate 0.5 s embedding run records 100 rows through the public observer,
identical to the plugin's force rows. It records five 640×480 RGB frames at
10 Hz using llvmpipe CPU rendering and encodes them with the existing CPU video
path. This demonstrates recording, not policy behavior.

No new test suite, RL workspace change, GPU job or installed-desktop update is
part of this prototype. The SuperMimic controller source remains unchanged; its
full behavior matrix was not rerun. RL inference and actual policy comparisons
remain the external application's work.

This prototype resolves an existing local shared library. Portable bundle export
and distribution of native plugins are outside this interface change.
