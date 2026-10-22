# meshoptimizer

Pinned subset of [meshoptimizer v1.3](https://github.com/zeux/meshoptimizer/tree/v1.3), MIT.
`manifest.json` records the exact commit and unmodified source hashes. Only the
allocator, simplifier and public header are compiled. No runtime download or
asset rewrite occurs.

The software camera renderer prepares indexed meshes once per EGL context.
RGB levels retain original vertex positions, normals and UVs, with attribute
weights and an absolute simplification error estimate. Selection projects that
estimate into the current viewport with a half-pixel budget and a safety margin.
This metric is **not a certified Hausdorff bound** or a bound on RGB intensity
changes from shadows, lighting, textures or rasterization. Validation therefore
checks rendered images as well as exact depth and physical outputs.
Shadow-only meshes use a separate 1 mm error estimate. Depth keeps every original
triangle; physics and LiDAR never use these buffers. Small meshes, convex hulls,
transparent geometry and unsupported rendering paths keep native drawing.

该依赖仅用于软件相机渲染，来源及哈希固定。RGB 按当前视口投影误差估计选择
简化级别，以半像素为预算并保留余量；这不是严格的几何误差证明，也不表示
阴影或颜色的差异被限制在半像素内。深度保留全部原始三角形；物理与雷达不使用
简化网格。验收同时检查画面差异及深度、物理输出的一致性。
