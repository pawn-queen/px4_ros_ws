# SLAM 引入计划：深度相机 + IMU + 2D 雷达，UWB 约束与校验

日期：2026-11-25。第一阶段范围已确认；第二阶段集成设计见 [ADR-0005](../adr/0005-slam-pose-source.md)。

第一阶段交付独立 SITL 实验。软件、原生档 SITL、实机各自报告验证结果。运行方法、证据目录与输出约定见[专项验证入口](../../validation/slam/README.md)。

## 1. 目标与非目标

目标：

- UWB 退化、短时失效或锚点几何变差时，仍有连续、可门控的位姿；
- 建图使用的位姿与定位输出共享同一坐标语义（`map` = PX4 本地 ENU）；
- 具备可核查的 SLAM 要素：自运动估计、关键帧、回环、重定位、位姿图优化；
- 换估计器不动上层：位姿源可替换，契约与门控不随算法改动。

非目标：

- 不重建建图、规划、控制栈；`voxel_mapper` 仍是唯一任务地图；
- 不改 PX4 固件与 EKF 参数；EV 观测偏移继续由飞控侧维护；
- 本计划不承诺把 UWB 变成“真值”；校验与约束的角色分离见 ADR-0005 第 4 条。

## 2. 已定决策

| 编号 | 决策 | 依据 |
|---|---|---|
| D1 | 第一阶段同时验证 RGB-D 里程计、回环、重定位与历史关键帧几何修正 | [ADR-0005](../adr/0005-slam-pose-source.md) 第 1 条 |
| D2 | 第二阶段拟走路线 1：SLAM → EV → EKF → `navigation_mode=px4` | 同上第 2 条；`map` 帧自动等于 PX4 本地帧 |
| D3 | 位姿仲裁不是融合：同一时刻只有一个源 | 同上第 3 条 |
| D4 | UWB 约束锚点与校验锚点分离，验收用独立参考 | 同上第 4 条 |
| D5 | RTAB-Map 先打通 → ORB-SLAM3 精度基线 → 长期再定 | 同上第 5 条；ROS 2 Jazzy 适配成本是硬门槛 |
| D6 | 第二阶段任务地图唯一，回环/合并时 bump `map_epoch` 重建 | 同上第 6 条 |
| D7 | 第二阶段契约 v2，三处镜像同改 | 同上第 7 条 |

## 3. 架构与数据流

```text
第一阶段，已有仿真
OakD-Lite 彩色图、深度图、各自 CameraInfo + 机身原始 IMU + /clock
        → RGB-D 配准、IMU 滤波 → RTAB-Map → /slam/odometry、status、mapGraph、cloud_map
UWB → EV → PX4 → 专项航线控制                     ↓
原始 MCAP + 独立的 Gazebo 机体真值与电厂表面 → 离线评分

第二阶段候选
独立 SLAM → 帧对齐与 EV 切源 → PX4 EKF → localization_node → 任务 mapper、规划与控制
UWB 约束、校验锚点分组与雷达平面约束单独评估。
```

关键约束：EV 桥在同一时刻只接受一个源；导航输出永不回灌 EV（现有规则，见 [external_vision_bridge_node.py](../../src/px4_powerplant_mission/px4_powerplant_mission/nodes/external_vision_bridge_node.py) 模块说明）。

## 4. 阶段计划

| 阶段 | 目标 | 主要改动 | 完成判据 |
|---|---|---|---|
| **一 · 原生在线采集** | 在原电厂中观察 A、B、相似设备并重访 A | 核对加载资产、两路内参和时间戳，审计净空，飞行中记录真值和原始 MCAP，只遮挡 SLAM 的输入副本 | 实际飞行与视场包含指定观测，B 在回访中离开视野 |
| **一 · 标定** | 建立独立 SLAM 的事件与性能基线 | 同一原生包从空数据库做三对回环开/关回放；仅起始段作一次固定尺度平移与偏航对齐 | 三轮均出现经真值核验的非邻接回环与历史匹配，B 旧观测保留且改善超过重复波动。用最差值加极差冻结上限，下限相反 |
| **一 · 盲验** | 检验复现及相似设备误匹配 | 冻结软件、路线、事件规则和门槛后，改变接近速度采集独立航次 | 原生在线与三轮回放分别通过；报告负例数与错误边数、遮挡解除至确认耗时、B 覆盖率与距离 p95 |
| **二 · 集成与实机** | 进入任务导航链路 | 旧草案 S3–S6 的 EV 切换、契约、任务体素图重建、恢复规划与实机评估 | 另行冻结实机指标与故障门控 |

下文 S0–S6 保留为集成议题编号。第一阶段的入口、回环和重定位取证前置，飞行控制与任务地图的集成归入第二阶段。

## 5. 改动清单

| 项 | 类型 | 说明 | 阶段 |
|---|---|---|---|
| `px4_powerplant_slam` 包 | 新建 | 估计核心，输出 `/slam/odometry`（带协方差）与 `/slam/status`（JSON） | 第一阶段 |
| IMU 桥 | 新建 | `gz_sensor_bridge` 增加 `sensor_msgs/msg/Imu`，或改订 PX4 `vehicle_imu` | S1 |
| 标定文件 | 新建 | `config/calibration/` 内外参与时间偏移，证据进 `validation/evidence/` | S1 |
| `slam→map` 对齐 | 新建 | 复用 `uwb_to_map_*` 的 4-DoF（yaw + 平移）模型 | S3 |
| 数据集与验收套件 | 新建 | `validation/slam/`、`validation/tests/test_slam_*.py`、baseline 挂载 | S1 起，S6 收口 |
| 位姿仲裁 | 扩展 | `localization_node` 增加 `pose_source`、切换迟滞、`localization_epoch` | S3 |
| EV 源切换 | 扩展 | `external_vision_bridge_node` 增加 `ev_source: uwb \| slam` 且互斥 | S3 |
| 门控分支 | 扩展 | `localization_ready()`、`motion_health_reason()` 增加 slam 分支 | S3 |
| 契约 v2 | 扩展 | `mission_contract.json`、`mission_contract.py`、`mission_contract.hpp` 同改 | S3 |
| 配置与 profile | 扩展 | `powerplant.yaml` 新段、`localization_slam.yaml`、`runtime_profiles.json` 新档 | S3 |
| 回环与地图纪元 | 扩展 | 第二阶段任务地图接收回环后触发 `localization_epoch` + `map_epoch` 重建 | S4 |

## 6. 参数与标定

| 组 | 关键项 | 确定方式 | 落点 |
|---|---|---|---|
| 语义 | 位姿 owner、UWB 双角色划分、回环后地图策略、EV 唯一输入 | 设计决策 | ADR-0005 |
| 外参 | `T_body_cam`、`T_body_imu`、`T_body_lidar`、深度-彩色外参 | Kalibr；与现有 `camera_*` / `lidar_*` 外参保持同一组值 | `config/calibration/` |
| 时间 | 相机-IMU、雷达-IMU 偏移，PX4 时间域 | 离线标定 + 在线修正；修正不收敛则禁止起飞 | 同上 |
| 前端 | 特征数、最小视差、关键帧间距、跟踪丢失帧数、雷达倾斜投影门限 | 仿真真值下扫描 | `localization_slam.yaml` |
| 回环/全局 | 描述子与 top-k、内点阈值、PGO 鲁棒核；UWB 协方差、鲁棒核、`max_dt` | 召回-误报曲线；NLOS 注入实验 | 同上 |
| 门控 | 位置/姿态方差、跳变阈值、失锁时间、重定位确认帧数、epoch bump 条件 | 故障注入，每项一个红/绿用例 | 契约 + `powerplant.yaml` |

## 7. 验收指标

| 维度 | 指标 | 判定方式 | 证据 |
|---|---|---|---|
| 精度 | ATE(RMSE)、RPE | 仿真真值对齐后统计，多次运行给区间 | `validation/evidence/` |
| 回环 | 召回率、误报率 | 注入重访与近似重访场景 | 同上 |
| 重定位 | 成功率、耗时 | 分场景统计 | 同上 |
| 健康 | 失锁检测延迟、错误切换次数 | 故障注入；错误切换必须为 0 | 同上 |
| 一致性 | 回环后地图与真值偏差 | 重建后必须回到阈值内 | 同上 |
| 资源 | CPU/GPU 占用、p95 延迟 | 目标机实测，不在 WSL 下结论 | 同上 |

## 8. 风险与缓解

| 风险 | 影响 | 缓解 |
|---|---|---|
| 三路时间同步错配 | 表现为“莫名漂移”，调参方向全错 | S1 先做时间同步并出残差证据，再进 S2 |
| 2D 雷达倾斜/变高退化 | 误判为回环能力 | 只做局部约束；倾斜超门限丢弃该帧 |
| UWB 自证 | 验收等于没验 | 约束/校验锚点分离 + 独立参考 |
| 回环跳变冲击控制 | 规划与避障建立在错位地图上 | 先停、bump epoch、重建后再授权 |
| 算力不足 | 与 YOLO/建图/规划抢资源 | S3 起在目标机做资源预算，超限降级 |

## 9. 需要拍板的未决问题

1. **UWB 双角色怎么落地**：锚点分组（≥4 约束 + 1–2 校验）、时间分离，还是只靠独立真值做验收？
2. **2D 雷达进不进估计器**：进则多一层约束与复杂度；不进则只做障碍与校验，回环全靠视觉或深度几何。
3. **回环扩展**：第一阶段采用 RTAB-Map 视觉回环；是否加入深度几何或雷达约束由对照试验决定。
4. **长期选型**：RTAB-Map（BSD，省事）还是 ORB-SLAM3（GPL，精度与表述更强）？
5. **实机资源预算**：Orin NX 8GB、D435if、CUAV X7+ Pro、二维 360° 雷达与约 10 cm 的 UWB 锚点已确定。待测的是实机占用、延迟及遮挡恢复表现。

## 10. 与现有文档的关系

- 研究依据：[native-stack-research.md](../research/native-stack-research.md) 将 VIO 列为 P3，并明确“先做测距与质量评估，再评估 VIO”；本计划不改变该顺序，S1–S2 仍以测量与质量为先。
- 定位现状与 EV 独立性规则：[LOCALIZATION_DEPLOYMENT.md](../../src/px4_powerplant_mission/LOCALIZATION_DEPLOYMENT.md)。
- 验证边界：[ADR-0001](../adr/0001-runtime-validation-seam.md)（validation 不参与 colcon 构建）。
- 契约变动流程：[ADR-0002](../adr/0002-mission-contract.md)。
