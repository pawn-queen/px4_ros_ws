# 项目术语表

本文件只定义在代码、Issue、测试和文档中反复出现的项目词。新增或改变术语时，先改这里，再改实现和文档。

| 术语 | 定义 | 不应当作 |
|---|---|---|
| **runtime** | 飞行或仿真时真正执行任务行为的代码、配置和节点；当前由 `px4_powerplant_mission_cpp` 和 `px4_powerplant_mission` 承担。 | 验证脚本、历史分析、一次性 runbook |
| **validation** | 对 runtime 行为做检查、测量和证据收集的模块；入口在 `validation/`。 | runtime 节点、飞行控制实现 |
| **mission** | 一次从启动、解锁、起飞、探索/巡检到降落的任务周期。 | 单个 ROS 节点、单个算法函数 |
| **mission session** | 一次 mission 的完整运行会话；未来应成为 mission 生命周期的 deep module。当前仍由 `OffboardMissionNode` 承担。 | 规划线程、单个 state 字段 |
| **map snapshot** | mapper 发布的带 epoch/sequence 的四字段体素快照；控制器只接受完整、时间配对的新鲜快照。 | 任意点云、任意 OccupancyGrid |
| **frontier** | 已知空闲且相邻未知的体素；对应候选 viewpoint。 | 任意未知体素、任意航点 |
| **viewpoint** | 为观察 frontier 而选择的已知空闲位姿；需要可见性、净空和返航预算。 | 目标本身、拍照点 |
| **ROI_ACCESS / ROI_WORK** | EXPLORE 内“到达任务区域”和“在任务区域工作”的两个子模式。 | 独立任务阶段、PX4 飞行模式 |
| **takeoff domain** | 起飞点周围的竖直安全柱体；由确认区域、净空和地面支持共同约束。 | 普通起飞高度、任意圆柱 |
| **P0 / P1 / P2** | 2026-10 的历史工作阶段名；当前只用于历史文档和迁移记录，不再作为脚本或新模块名。 | 当前模块名、能力等级 |
| **ADR** | Architecture Decision Record，记录已做出的结构决策和后果。 | 设计草稿、会议记录 |

## 命名规则

- 新增脚本、目录、测试和文档时，优先使用职责名，例如 `run_acceptance.py`、`takeoff_audit.py`，不要新增 `p0_*`/`p2_*` 前缀。
- 历史阶段名可以出现在 `docs/history/` 和 `docs/validation/` 的标题/正文里，但不能出现在新路径中。
- 代码标识、命令、路径和 ROS/协议专有名词保留英文；说明性正文用中文。
