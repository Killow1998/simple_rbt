# Classic rendering batch / 经典渲染批处理

Private adaptation of the [MuJoCo 3.15.0 classic renderer](https://github.com/google-deepmind/mujoco/blob/3.15.0/src/render/classic/render_gl3.c), Apache-2.0. `manifest.json` records the original source hashes. This is drawing code only; the installed MuJoCo library still owns physics, scene construction, render contexts and pixel readback.

MuJoCo's public `mjr_render` API regenerates shadows for each view and does not expose a reusable shadow pass. This copy provides a narrow optimization for the six panorama views of one frozen physical sample. It is compiled into a private namespace, without overriding library symbols or using preload hooks. Only software llvmpipe/softpipe with matching 3.15.0 headers and runtime uses it. Other renderers and MuJoCo versions use the original public renderer.

Local changes in `render.inc`:

- Cull bounded geometry outside the four view side planes before submitting draws. Shadow casters remain complete. Reflections bypass this culling; planes, height fields, flex, skin and decorative geometry retain the native path.
- Reuse a shadow map only within one snapshot, one render context, unchanged clip distances, and exactly one supported shadow-casting light. Multiple shadow lights redraw normally. `CameraRender::begin()` invalidates it for each scene update, including observer views.
- Keep MuJoCo's arithmetic, draw order, materials, transparency, depth and multisampling. Use private helper names, system OpenGL entry points and explicit C++ narrowing casts. Clip-control capability belongs to the current batch rather than a loader-global variable.

`helpers.inc` contains unchanged arithmetic from `render_util.c`, `mjv_rbound` from `engine_vis_init.c`, and text drawing from `render_gl2.c`. `primitives.inc` contains the original display-list offsets. The stable sort is copied unchanged. Long upstream files are kept recognizable for comparison; project policy is implemented in the small `src/camera_render.cpp` module.

Validation: `camera_render_consistency` compares native and batched RGB/depth passes, with separate framebuffers, across changing geometry/lights/cameras, transparency, reflections and multiple shadow lights. It is included in the native CI checks. Upgrading this adaptation requires updating source hashes, reviewing the upstream changes, and rerunning that oracle plus complete sensor/ROS E2E. A version mismatch safely keeps native rendering and makes no throughput promise.

这份 Apache-2.0 适配仅涉及绘制；物理、场景生成、渲染资源和像素读回仍由原 MuJoCo 库负责。公开 API 无法跨视角复用阴影，因此只为同一物理采样的多个视角保留固定版本实现。软件渲染在 MuJoCo 3.15.0 下使用该路径，其他版本和硬件渲染使用原生实现。每次物理采样或观察画面更新都使缓存失效；双投影光源、反射和视野边界由原生图像对照覆盖。相机和全景输出尺寸、传感器频率不变；软件阴影贴图上限为 2048，较小的模型设置原样保留。
