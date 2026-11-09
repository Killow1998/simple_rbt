# Documentation / 文档

- [CPU/GPU physics / CPU／GPU 双物理后端](dual-physics.md)：统一任务入口、共享控制契约、真实 Warp 批量步进与有界训练验收。
- [C++ runtime / C++ 运行环境](runtime.md)：构建依赖、模块边界、运行参数和输出格式。
- [Runtime state inventory / 运行状态清单](state-machines.md)：会话、控制、界面、进程与传感器的状态和归属。
- [External CPU controllers / 外部 CPU 控制器](external-controllers.md)：通用共享库接口、原始模型状态、执行器目标、物理步力记录与视频复用。
- [Structured scene contract / 结构化场景约定](scene-representation.md)：v2 基元、二次曲面关系、位姿、楼梯共享模板、边界及扩展规则；C++／编辑器入口、Schema、示例及后端预算。
- [SpRobo launcher and acceptance / SpRobo 启动器与验收](desktop-console-plan.md)：启动依赖检查、手动／程序控制、TurtleBot3、独立 ROS、安装与 CI 发布记录。
- [Independent ROS E2E / 独立 ROS 端到端测试](noetic.md)：公开 Noetic／Humble 镜像、独立节点、直接控制与传感器验收。
- [Quadrotor / 四旋翼](flight.md)：物理推力控制、键盘升降、世界 ENU 指令和载荷边界。
- [Mapping tutorials / 建图教程](tutorials.md)：独立 FAST-LIO／FAST-LIVO2、自动标定、Noetic／Humble 示例与验收。
- [Repository cleanup / 仓库清理](repository-layout-20261014.md)：当前工具边界、Python 归档及验证记录。
- [Assets / 运行资源](../assets/README.md)：模型、权重、参数和扫描数据。
- [Architecture and ablation / 架构与消融](cpp-architecture-ablation-20261013.md)：模块拆分、内存 A/B、故障注入与脚轮修正的验证记录。
- [Go2W SuperMimic validation / 外部 Go2W 策略验证](go2w-supermimic-validation-20261029.md)：独立启用的控制器、外部模型契约和 CPU 对照结果。

Dated reports record the tested version and conditions at that time. They do not replace current usage instructions.
带日期的报告记录当时的版本和测试条件，当前操作以使用文档为准。
