# Structured scene representation / 结构化场景表征

**Status: implemented in the development build, 2026-10-24.** The C++ core and
SpRobo editor accept v2 alongside v1. Installation is separate. Real-cloud
reconstruction still requires surface and route qualification; format support
is not acceptance of an inferred building.

**状态：开发版已接入 C++ core 和 SpRobo 编辑器。** v1 保持兼容，安装版单独更新。
真实点云场景仍须验收表面、边界和路线，格式支持不等于建筑重建通过。

[Schema](../schemas/scene-v2.schema.json) ·
[Example / 示例](../schemas/examples/structured-building.scene.json) ·
[Current runtime / 当前运行接口](runtime.md#workspace-and-user-scenes--工作区与用户场景)

## 1. Representation boundary / 表征边界

An object is **identity + local geometry + rigid pose + evidence**. A scene is a
tree of objects and groups. Geometry has no ROS, GUI, MuJoCo or fitting-library
types. One editable object may generate several collision primitives.

一个对象由**标识、局部几何、刚体位姿、依据**组成。楼层和楼梯间是分组；分组本身
没有碰撞几何。长墙、地板和楼梯按连续结构组织。拟合网格、三角片和碰撞分解只作为
内部产物，不能自动变成数千个用户编辑对象。

A geometry-free group can also serve as a named pose marker, such as a robot
spawn. Selecting that marker and applying robot clearance belong to the session,
not the scene geometry. Robot presets, sensors and command ownership remain
separate. 无几何分组也可作为命名出生点；选择标记及增加机器人离地高度由会话负责，
不会因场景中出现一个标记就自动启动机器人。

| Option / 方案 | Decision / 取舍 |
| --- | --- |
| Existing v1 boxes / 现有盒体格式 | Preserve as a compatible runtime format; it cannot express arbitrary boundaries or an open stair underside. 保留兼容入口。 |
| Raw MJCF / 直接使用 MJCF | Reuse as a backend output, not the building-authoring contract. 原生几何可复用，建筑参数和来源约定由框架保存。 |
| Parametric geometry + boundary / 参数基元＋有限边界 | Canonical scene data. 作为新的权威表征。 |
| General symbolic CSG / 任意符号布尔建模 | Not part of v2. Plane holes and explicit groups cover this stage. 本阶段不引入任意布尔表达式或运行时插件语言。 |

MuJoCo already supports analytic primitives, but its `plane` collision is
infinite and box/cylinder dimensions use half extents. The adapter must convert
these conventions explicitly. [MuJoCo geom reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-geom)

## 2. Common coordinates and pose / 坐标与位姿

| Field / 字段 | Convention / 约定 |
| --- | --- |
| `format`, `version` | `sprobo.scene`, `2`; schema identity is `urn:sprobo:scene:2`. |
| Units / 单位 | metres, radians; integers for counts. UI may show degrees. 米、弧度、整数计数。 |
| World / 世界系 | Right-handed, +Z up. No assumed geographic north. 右手系，Z 向上，不默认地图轴等于地理北向。 |
| `pose.position_m` | Object origin in its parent's frame. 父坐标系中的原点位置。 |
| `pose.quaternion_wxyz` | Unit quaternion `[w,x,y,z]`; active rotation from object to parent. 物体系到父系的主动旋转。 |
| `parent` | One object/group ID or reserved `world`. Unique IDs; no cycles. 一个父节点，无循环引用。 |
| `name` | User-editable display name; never used as identity. 用户可改名，标识不变。 |
| `semantic` | Optional descriptive label such as `floor`, `wall`, `ceiling`, `landing`. 标签不改变几何或物理行为。 |

For column vectors, `p_parent = R(q) p_object + t` and
`T_world_object = T_world_parent T_parent_object`. Scale and shear are not pose
fields. Bake scale into dimensions before import. Positive rotation follows the
right-hand rule. Robot frames retain +X forward, +Y left, +Z up; camera optical
frames remain sensor-specific. These choices follow the SI/right-handed parts
of [REP 103](https://raw.githubusercontent.com/ros-infrastructure/rep/master/rep-0103.rst).
Quaternion ordering is explicit because ROS messages use a different field order.

同一位置只通过父子变换累计一次。楼层高度放在楼层分组的位姿中，不同时再写一份
`floor_height` 加到物体位置。四元数归一化由语义校验检查；输出时将等价的 `q/-q`
规范为首个非零分量为正，减少无意义文件差异。

Height and rise parameters are local-axis measurements. For an upright building,
the corresponding local +Z aligns with world +Z. If a whole object/group is tilted,
transform its endpoints to obtain world elevation; do not apply the incline twice.
高度沿基元自身的轴计量；整体倾斜时，世界系高差由端点变换计算，不能重复叠加坡角。

## 3. Primitive catalogue / 基元目录

All dimensions below are **full dimensions**, except explicit radii. Each type
has one local origin and one parameter set; derived values are not saved twice.

| `type` / 基元 | Independent parameters / 独立参数 | Origin and axes / 原点与轴 |
| --- | --- | --- |
| `box` 长方体 | `length_m`, `width_m`, `height_m` | Centre; lengths along X/Y/Z. 中心原点；范围为各轴正负半尺寸。 |
| `planar_region` 有限平面板 | `outer_xy_m`, `holes_xy_m`, `thickness_m` | Reference face at local Z=0; solid extends to Z=−thickness. 基准面外法向 +Z，厚度向 −Z。 |
| `cylinder` 圆柱 | `radius_m`, `height_m` | Centre; axis Z, caps at ±height/2. 中心原点，Z 为轴线。 |
| `sphere` 球 | `radius_m` | Sphere centre. 球心。 |
| `ellipsoid` 椭球 | `radii_m: [rx,ry,rz]` | Centre; principal axes X/Y/Z. 中心及三个主轴。 |
| `frustum` 圆台／圆锥 | `bottom_radius_m`, `top_radius_m`, `height_m` | Centre between cap planes; axis Z. One radius may be zero. 圆锥允许一端半径为零。 |
| `capsule` 胶囊体 | `radius_m`, `cylinder_length_m` | Centre; axis Z; total length = cylinder length + 2r. 中间圆柱长度不包含两端半球。 |
| `ramp` 坡道板 | `width_m`, `run_m`, `rise_m`, `slab_thickness_m` | Lower top edge midpoint; +X uphill, +Y across, +Z up. 下端顶边中点为原点。 |
| `stair_flight` 一段楼梯 | `width_m`, `step_height_m`, `tread_depth_m`, `steps`, `slab_thickness_m` | Lower start at `[0,0,0]`; +X uphill, +Y across, +Z up. 下端起点为原点。 |

Walls, rectangular slabs, beams and rectangular columns reuse `box`; they are
semantic roles, not duplicate geometry types. Platforms reuse a box or a finite
plane board. A nonrectangular floor or wall with openings uses `planar_region`.
矩形地面可用一个长方体；走廊转角、楼梯洞口和门窗必须保留在有限边界中。
不能用整层包围矩形填满中庭、洞口或未确认区域。

`outer_xy_m` is an unclosed, counter-clockwise simple ring seen from +Z.
Holes are unclosed clockwise rings, strictly inside the outer boundary,
nonoverlapping and not nested. Rings have at least three distinct vertices and
nonzero area. Rings may be concave. Geometry touching an outer edge is a notch
in the outer ring, not a hole touching the boundary. Disconnected regions are
separate objects in one group. No automatic convex hull or hole filling is allowed.

外轮廓逆时针、孔洞顺时针，首点不重复写在末尾。轮廓不能自交；孔洞不能相交、嵌套
或触碰外边界。板的正反面和孔壁组成实体；“未知区域”不自动等于“真实孔洞”。

## 4. Stair and ramp definitions / 楼梯与坡道

For one stair flight, let `W=width_m`, `h=step_height_m`, `d=tread_depth_m`,
`N=steps`, `t=slab_thickness_m`:

```
total rise H = N h        horizontal run L = N d
incline θ = atan2(h,d)    exit = [L, 0, H]
```

The editor may accept **width, total height, incline and step count**, then resolve
`h=H/N` and `d=H/(N tan θ)`. It saves only the independent canonical parameters.
If the user edits a derived quantity, the editor must show which canonical
parameter changes. It must not retain conflicting height/angle/depth values.

界面支持 Xian 提出的“宽度、坡度角、级数、总高”；保存时归一为 W、h、d、N、t。
例如 H=3 m、N=20、d=0.28 m，则 h=0.15 m、L=5.6 m、θ≈28.18°。

The convention is **N stair cells**, each with one rise and one tread. Tread `i`
occupies `[(i−1)d, id]` at `z=ih`, for `i=1..N`, and `y∈[−W/2,W/2]`.
The first riser is at x=0. The upper platform starts at x=L, z=H; the final
tread is owned by the stair flight. Importers using N risers but N−1 treads
must convert this ownership explicitly, not silently reuse the same run.

约定 N 个台阶单元，每个单元含一级立面和一个踏面；最后踏面归楼梯，平台从末端
继续。来自“N 个立面、N−1 个独立踏面”的 CAD 数据需要明确转换，避免差一级。

The underside is the plane `z=(h/d)x − t/cos θ`, clipped to the flight footprint.
Here t is the normal thickness below the pitch line through the tread/riser
valleys. Each cell is a convex prism between that inclined underside and its
horizontal tread. The space below remains empty. **Do not fill each step down
to the lower storey.** A ramp uses top `z=(rise/run)x` and the same underside
formula with `θ=atan2(rise,run)`; its x range is `[0,run]`.

楼梯上表面是台阶，下表面是斜板；t 沿坡面法线计量。每级内部可分解为凸棱柱，
编辑器仍显示一段楼梯。现有 v1 楼梯的实心堆叠方式不满足这个下方空间约定。

A `stairwell` group contains flights and separate landings. U/L turns are expressed
by child poses, not a second hidden turn convention. Each join must match
endpoints, elevation and clear width. Shared geometry templates store one set
of W/h/d/t; flights can override N and have independent poses. An inconsistent
flight requires a separate template or an explicit override visible to the user.

同一楼梯间优先共享规格，观测负责验证一致性。平台标高、方向和连接逐段记录。
模型补全的台阶标为推定；同规格的先验不等于每一级都已直接观测。

## 5. Quadric connection / 与二次曲面形式的关系

With homogeneous local point `p̄=[x,y,z,1]ᵀ`, a symmetric 4×4 matrix Q represents
`f(p)=p̄ᵀQp̄`. Surface: f=0; interior: f≤0. Local-to-world pose T gives
`Q_world = T⁻ᵀ Q_local T⁻¹`. Q is **derived**, not an additional editable truth.

| Shape / 形状 | Derived expression / 推导表达式 | Finite boundary / 有限约束 |
| --- | --- | --- |
| Plane / 平面 | `n·p+c=0`; Q's last row/column contain n/2, Q44=c | Polygon rings and thickness. 多边形边界及厚度。 |
| Ellipsoid / 椭球 | `Q=diag(1/rx²,1/ry²,1/rz²,−1)` | Already bounded. 本身有界。 |
| Sphere / 球 | Ellipsoid with rx=ry=rz=r | Already bounded. |
| Cylinder / 圆柱 | `Q=diag(1/r²,1/r²,0,−1)` | `−H/2 ≤ z ≤ H/2` |
| Frustum / 圆台 | `x²+y²−(kz+b)² ≤ 0`, k=(rt−rb)/H, b=(rt+rb)/2 | `−H/2 ≤ z ≤ H/2`; radii nonnegative. |
| Box / 长方体 | Intersection of six plane half-spaces. 六个平面半空间的交集。 | Full dimensions. |
| Ramp / 坡道 | Intersection of six plane half-spaces. | Finite slab footprint. 有限坡板。 |
| Stairs / 楼梯 | Union of ordered convex step prisms. 有序台阶棱柱的并集。 | Count, width, run, underside. |
| Capsule / 胶囊 | Cylinder plus two spherical end regions. 圆柱段与两端半球组合。 | Straight-segment length and radius. |

For the frustum matrix: Q11=Q22=1, Q33=−k², Q34=Q43=−kb, Q44=−b²; other
entries are zero. Multiplying Q by a positive scalar leaves the same solid.
Arbitrary quadric coefficients alone do not encode finite ends, holes, stair
ordering or semantic identity. This is why v2 saves parameters plus boundaries.

`f(p)` is an algebraic residual, not a metric distance. Fitting acceptance uses
distance to the finite surface in metres; `|f|/||∇f||` is only a local approximation
and must not silently replace that metric. Surface normals follow normalized
`∇f` where it is nonzero; edges/apices need the selected face's normal.

二次曲面作为统一的拟合与坐标变换表达；长方体、有限板和楼梯还需要边界或
组合关系。不能为追求同一种方程而丢失门洞、台阶或楼梯下方空间。

## 6. Templates, evidence and compilation / 模板、依据与编译

`geometry_templates` maps IDs to complete primitive definitions. An object's
`geometry` is either an inline primitive or `{template_id, overrides}`. Templates
cannot reference templates. Overrides replace whole parameter fields; they
cannot change `type`, pose or identity. Validate the resolved primitive again.
Changing a shared template affects its instances; converting one instance to
inline geometry detaches it explicitly. No recursive inheritance is introduced.

`evidence.basis` is `manual`, `fitted`, `inferred` or `mixed`. Nonmanual geometry
links a local `report_ref` with source hashes, source frame transforms, fitting
residuals and supported/inferred regions. For mixed stairs, report per-tread,
riser, side and underside support. A fit prior must not become a measured fact.
这些记录不参与碰撞计算，也不把私有点云嵌入场景或上传到仓库。

Compilation order / 编译顺序：

1. Check JSON shape, then resolve templates and parent transforms.
2. Check finite values, unit quaternions, ring topology, references and joins.
3. Produce one backend-neutral resolved geometry representation.
4. Generate rendering, collision and sensor geometry from that same representation.
5. Report resource cost; reject unsupported geometry or exceeded budgets explicitly.

新增基元只增加参数定义、局部几何实现及后端映射；ROS wrapper、控制器和日志协议
不读取基元私有参数。渲染、碰撞、雷达、深度不能各自拟合一套场景。RGB 简化也必须
遵守已约定的图像误差边界，不改变原始碰撞、深度和雷达表面。

MuJoCo mesh collision normally uses convex hulls. A concave floor with holes or
an entire staircase cannot be passed as one mesh and assumed to retain its
collision boundary. The backend must preserve openings through suitable convex
parts; it must not turn a staircase into a ramp. [MuJoCo modeling](https://mujoco.readthedocs.io/en/stable/modeling.html)

## 7. Validation, compatibility and extension / 验证、兼容与扩展

JSON Schema checks structure and scalar ranges. It cannot alone prove quaternion
norm, reference resolution, polygon topology or geometric continuity. These are
mandatory semantic checks before a scene becomes runnable. All numeric values
must be finite; quaternion norm tolerance is 1e−6. No silent normalization of bad
dimensions or automatic repair of invalid boundaries is allowed.

Runtime limits such as 2048 generated boxes belong to the active backend/profile,
not the definition of a building or a schema-wide object count. Author-object,
collision-part and triangle counts must be reported separately. The v1 budget remains 2048 boxes. The v2 backend allows 4096 convex parts,
8192 author nodes, 2048 vertices per planar region, 262144 display vertices and
128 hierarchy levels. The full four-core GUI/Noetic measurement covers 2814
parts; the limit does not promise real time for every scene.

New primitive types require a schema/version capability update, a local-frame
definition, one canonical parameter set, bounds/normal/containment behavior,
and consistent collision/sensor output. Unknown types fail explicitly. No
silent partial loading. Keep shape mathematics in the C++ core and preparation
or independent verification outside the runtime.

Implementation acceptance must include a robot crossing a slab/flight join,
clearance under stairs, rays through real holes, RGB/depth/collision agreement,
save/reload of shared templates and limits under the full GUI/ROS workload.
Schema validation alone is not runtime E2E acceptance. See the measured
[implementation status](desktop-console-plan.md#structured-scene-v2--2026-10-24--结构化场景).

Migration from v1 must preserve IDs, physical placement and user names. A v1
object's floor height becomes a parent transform once. Boxes keep their full
dimensions and quaternion; a v1 ground-point spawn becomes a geometry-free pose
marker. Exact migration of generated stairs/ramps uses a group of their existing
resolved boxes instead of silently reinterpreting their parameters. A legacy filled staircase
must not silently acquire an open underside: offer a deliberate conversion to
the v2 stair model, retaining the old geometry when exact migration is requested.

开发版已实现公共解析、几何生成、编辑和保存。安装版与已有 v1 文件不自动改写。
当前提供两个独立兼容入口，没有自动批量迁移 v1 文件。

## Runtime and editor / 运行与编辑

`simple-rbt-sim --resolve-scene < scene.json` reads a v2 document and emits
resolved node poses, convex parts and display triangles. It does not initialize
a robot, a renderer or ROS. The editor uses this same C++ resolver and caches the
result until the document changes. Invalid edits keep the last valid preview;
Save/Use reject the invalid document with the core's error.

Session JSON embeds the document under `scene`. An optional `scene_spawn` selects
a geometry-free group with `semantic: "spawn"`; multiple markers require an
explicit selection. Robot spawn markers must be upright. With no marker, the
robot starts at the world origin. Ground clearance is applied once.

In the Scene editor, **New structured scene** creates v2. Select a group or
primitive to edit its pose. Stair instances can edit an override or the shared
template; duplicates keep the template reference. Planar boundaries and hole
vertices are editable. Total stair rise edits step height; incline edits tread
depth. Neither derived value is stored separately. Delete, undo/redo, scene naming, save and reload retain
hierarchy, parameters and evidence. Geometry/pose edits change fitted evidence
to `mixed`; they do not erase its report reference.

场景页可新建 v2、编辑分组位姿与基元参数、共享楼梯模板、修改实例参数、编辑洞口、
删除、撤销和保存。模板修改作用于全部引用实例，实例覆盖值仍优先。预览、碰撞、
雷达和深度使用同一解析结果；显示用曲面三角网格不替换原生解析曲面的碰撞或深度。

Bounded planar regions use pinned Earcut triangulation followed by exact convex
merging. Each step is a convex prism with an inclined underside. Frustums use
a convex mesh with a radial surface-error bound of 1 mm; unsupported tessellation
sizes fail explicitly. Other curved primitives use native MuJoCo geometry.
CPU LiDAR uses a static AABB tree as broad phase and MuJoCo's exact per-geometry
ray functions. `auto` falls back to CPU for non-box scenes; explicit CUDA reports
its unsupported geometry instead of dropping it.

Offline preparation helpers are in `tools/scene/`. `merge_observed.py` unions
qualified patches on each fitted plane without expanding their boundary; it
requires NumPy, SciPy and Shapely 2.1.2. `audit_temporal.py` reports transient and
approximate visibility conflicts from keyframe-tagged points. Its mask is a
review artifact, not proof of human removal; original source files are unchanged.
