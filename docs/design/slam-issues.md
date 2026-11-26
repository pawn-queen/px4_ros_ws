# SLAM 引入：Issue 拆解草案

日期：2026-11-25。配套[计划](slam-plan.md)与 [ADR-0005](../adr/0005-slam-pose-source.md)。

用法：每条按 `.github/ISSUE_TEMPLATE/` 的对应模板创建，标题加模板前缀，正文按下列字段填写；标签除模板自带的 `enhancement`/`refactor` 外，只从 [triage 标签](../agents/triage-labels.md) 的五个里选一个。每条验收项即红/绿用例，实施时先补测试再补实现。

以下编号供草案引用。当前按两次验收组织：

| 阶段 | 工作与依赖 | 验收 |
|---|---|---|
| 一，独立 SITL | #3–#5 原生采集 → #6、#8 RGB-D SLAM → #14a 历史几何与 #15a 重定位 → #18a 原生盲验 | 在冻结门槛后另飞一次。真实 A 回环、相似设备零误回环、遮挡后匹配旧帧、B 离开视野后的修正均须有在线与回放证据 |
| 二，飞控与任务地图 | #9–#13 集成 → #14b 任务图重建 → #15b 恢复规划。#16–#17 评估 UWB → #18b 集成 SITL → #19 实机 | 位姿切换、任务地图纪元、飞行门控和实机性能分别验收 |

第一阶段程序和使用方法见[验证入口](../../validation/slam/README.md)。后续集成项继续沿用下列模板。

---

## S0 决策与文档

### #1 定稿 ADR-0005 与 UWB 双角色方案

- 模板：无（决策记录，正文直接写入 Issue）
- 标签：`ready-for-human`
- 依赖：无

**功能背景**：ADR-0005 已确认独立 SITL 范围。第二阶段 UWB 设计仍待定。UWB 落在"约束锚点"还是"校验锚点"，决定 S5 的全部实现细节与验收口径。

**目标**：ADR-0005 第 4 条定稿并删除已解决的未决项，记录所选方案（锚点分组 / 时间分离 / 仅独立真值）及其数据前提。

**涉及模块**：`docs/adr/0005-slam-pose-source.md`、`docs/design/slam-plan.md` 第 9 节、UWB 与 EV 配置。

**接口或架构变更**：锚点集合在配置中有唯一出处；状态与门控引用同一来源。

**风险**：角色混淆会让验收变成自证；选错后再返工，S5 全部重做。

**验收标准**：

- 实施者不需要二次决策即可开始 #16/#17；
- 验收口径写明"用哪个参考、在哪些场景"，不含由约束源自证。

### #2 更新术语表与文档入口

- 模板：`refactor_request`（标题前缀 `[Refactor] `）
- 标签：`refactor`、`ready-for-agent`
- 依赖：#1（方向确定即可）

**功能背景**：仓库要求新概念先进 [GLOSSARY.md](../../GLOSSARY.md)；本轮已加入 SLAM、VIO、关键帧、回环、重定位、位姿图、位姿源、位姿仲裁、约束/校验锚点、地图纪元。

**目标**：术语定义与实现一致，文档入口唯一。

**涉及模块**：`GLOSSARY.md`、`docs/README.md`。

**接口或架构变更**：无外部接口变化。

**风险**：同一概念出现两种写法，后续契约与代码命名分裂。

**验收标准**：

- 术语与契约字段无命名冲突；
- 文档索引指向 [slam-plan.md](slam-plan.md) 与 [ADR-0005](../adr/0005-slam-pose-source.md)，且不存在第二份 current 文档。

---

## S1 传感器与标定

### #3 接入 IMU 到 ROS 2

- 模板：`feature_request`（标题前缀 `[Feature] `）
- 标签：`enhancement`、`ready-for-agent`
- 依赖：无

**功能背景**：仿真世界已启用 IMU 插件，但 `gz_sensor_bridge` 只桥深度、彩色、雷达、点云；PX4 的 `vehicle_imu` 已有订阅代码，但只在 `adaptive` 模式生效。

**目标**：SLAM 在同一时间基线下拿到标称频率的 IMU 数据。

**涉及模块**：`launch/gz_sensor_bridge.launch.py`、`config/powerplant.yaml`、`LOCALIZATION_DEPLOYMENT.md`。

**接口或架构变更**：新增 `/imu`（`sensor_msgs/msg/Imu`，或独立 PX4 转发节点）与对应 launch 参数；无破坏性变更。

**风险**：ROS 仿真时间与 PX4 时间域混用；IMU 与相机时间戳错配会表现为"莫名漂移"。

**验收标准**：

- `ros2 topic hz` 在标称频率 ±10%，时间戳单调；
- 与相机时间戳的偏差在同一 bag 内有记录；
- 丢包 >1% 时拒绝进入 S2 并记录原因。

### #4 完成相机-IMU-雷达标定与时间同步

- 模板：`feature_request`
- 标签：`enhancement`、`ready-for-agent`
- 依赖：#3

**功能背景**：现有 `camera_fx: 430.0` 等为手填近似值，不是标定结果；SLAM 直接用会导致尺度与重投影错误。

**目标**：三传感器的内外参、时间偏移与 IMU 噪声有可复现的标定产物与证据。

**涉及模块**：`config/calibration/`（新建）、`config/powerplant.yaml` 外参、`validation/evidence/`。

**接口或架构变更**：新增标定文件与读取参数；现有 `camera_*`/`lidar_*` 外参改为引用同一组值。

**风险**：出现两套外参值（建图与 SLAM 各一套）会让地图与位姿互相矛盾；时间偏移未收敛就起飞。

**验收标准**：

- 重投影误差 < 0.5 px；时间偏移残差有数值与分布记录；
- 与现有 YAML 外参的一致性检查通过；
- 时间偏移修正不收敛时，起飞门控拒绝起飞。

### #5 建立 SLAM 数据集录制与回放脚手架

- 模板：`feature_request`
- 标签：`enhancement`、`ready-for-agent`
- 依赖：#3

**功能背景**：没有可复现数据，S2 之后所有指标都无法对比，也无法回归。

**目标**：一条命令录制、一条命令回放，场景与确定性设置可追溯。

**涉及模块**：`validation/slam/`（新建）、`validation/datasets/` 存放约定、`validation/baseline/run_baseline.sh`。

**接口或架构变更**：无 runtime 接口变化；新增 validation 入口（按 ADR-0001 不参与 colcon）。

**风险**：bag 过大进 git 污染仓库；回放不确定性让红/绿判断失真。

**验收标准**：

- 第一阶段分别采集标定与盲验包，两者均含重访、相似设备和遮挡；弱纹理与快速旋转泛化试验留给后续算法对照。
- 同一 bag 两次回放输入时间线一致；
- 大文件不进 git，清单可查。

---

## 原生独立 SLAM 与后续算法对照

### #6 独立 RGB-D 里程计与初段对齐的轨迹误差

- 模板：`feature_request`
- 标签：`enhancement`、`ready-for-agent`
- 依赖：#4、#5

**功能背景**：先证明"能算"，再谈接入。首选 RTAB-Map（BSD-3、官方 ROS 2 Jazzy），或等价开源实现。

**目标**：给出可复现的精度与失效基线，明确失败场景。

**涉及模块**：`validation/slam/`、`config/slam_*.yaml`。

**接口或架构变更**：离线阶段无 runtime 接口；输出格式为报告 + 证据 JSON。

**风险**：只报成功序列；把仿真理想条件当实机性能。

**验收标准**：

- 三次运行的 ATE/RPE 落在统计区间内（非确定性系统不接受单次比对）；
- 失锁、快速旋转、弱纹理场景被记录而非跳过；
- 与真值的对齐方式（4-DoF，因 IMU 已给重力）写入报告。

### #7 2D 雷达平面约束离线对比

- 模板：`feature_request`
- 标签：`enhancement`、`ready-for-agent`
- 依赖：#5

**功能背景**：2D 扫描在机体倾斜时退化，需要先量化"加与不加"的差异，再决定是否进入估计器。

**目标**：用数据回答第 9 节未决问题 2，并确定倾斜门限。

**涉及模块**：`validation/slam/`、`config/slam_*.yaml`。

**接口或架构变更**：离线对比，无 runtime 接口变化。

**风险**：把 2D 匹配的短期改善误读为回环能力；倾斜门限过松引入位姿跳变。

**验收标准**：

- 开/关对比有量化结论（收益或无效都要写）；
- 倾斜超门限时正确丢弃该帧，不产生跳变；
- 结论直接回答"是否进入估计器"。

---

## S3 接入为位姿源

### #8 新建 `px4_powerplant_slam` 包与 `/slam` 接口

- 模板：`feature_request`
- 标签：`enhancement`、`ready-for-agent`
- 依赖：#6

**功能背景**：估计核心不能塞进现有两个包：依赖面（OpenCV/Eigen/Ceres/g2o）会污染 CMake 与安装面，也不符合 ADR-0001 的边界。

**目标**：独立 ROS 2 包，输出带完整协方差的位姿与可门控的状态。

**涉及模块**：`src/px4_powerplant_slam/`（新建）、`config/slam_*.yaml`。

**接口或架构变更**：新增 `/slam/odometry`（`nav_msgs/Odometry`，`map` 帧、ENU/FLU）、`/slam/status`（JSON：tracking、keyframes、loops、reinit、pose_jump、map_epoch）、4-DoF 对齐参数。

**风险**：位姿协方差语义不清会误导门控；引入 PX4 状态输入会破坏独立性。

**验收标准**：

- `colcon build` + gtest 通过；
- 吃 bag 出轨迹，帧与协方差语义写入 README；
- 代码不订阅任何 PX4 状态。

### #9 `localization_node` 位姿仲裁与 `pose_source`

- 模板：`feature_request`
- 标签：`enhancement`、`ready-for-agent`
- 依赖：#8

**功能背景**：现在只有 `navigation_mode: px4 | adaptive` 的早退分支，没有"选源 + 切换"的位置。

**目标**：单一出口、可解释的选源，切换不产生时间倒退或跳变。

**涉及模块**：`nodes/localization_node.py`、`config/powerplant.yaml`、`LOCALIZATION_DEPLOYMENT.md`。

**接口或架构变更**：新增 `pose_source: px4 | uwb | slam`（保留 `adaptive` 兼容）；status 增加 `pose_source` 与切换计数。

**风险**：把仲裁写成融合会重新引入相关性；切换无迟滞会造成抖动切源。

**验收标准**：

- 选择逻辑为纯函数并有单测；
- SITL 注入 SLAM 丢帧，status 可见源切换与 `localization_epoch` 递增；
- 切换瞬间不出现时间倒退或位姿跳变。

### #10 EV 桥支持源切换

- 模板：`feature_request`
- 标签：`enhancement`、`ready-for-agent`
- 依赖：#8

**功能背景**：EV 桥当前写死订阅 UWB；需要在不破坏"导航输出不回灌 EV"规则的前提下切换来源。

**目标**：同一时刻只有一个 EV 源，且切换过程可审计。

**涉及模块**：`nodes/external_vision_bridge_node.py`、`config/powerplant.yaml`。

**接口或架构变更**：新增 `ev_source: uwb | slam`，启动时互斥校验；SLAM 源走真实姿态分支；速度保持 NaN。

**风险**：两源同时发布会造成 EKF 观测冲突与连续 reset；姿态方差未校验会污染 EKF 航向。

**验收标准**：

- 契约测试覆盖 NED 转换、NaN 语义、`reset_counter` 透传；
- 切源后 PX4 不出现连续 reset；
- 导航输出回灌 EV 的路径在代码与测试中均被拒绝。

### #11 mission contract v2

- 模板：`feature_request`
- 标签：`enhancement`、`ready-for-agent`
- 依赖：#9、#10

**功能背景**：当前 `localization_status` v1 没有位姿源与 SLAM 健康字段，C++ 侧无法区分"源失效"与"质量下降"。

**目标**：v2 契约三处镜像一致，缺失字段时明确失败。

**涉及模块**：`config/mission_contract.json`、`px4_powerplant_mission/mission_contract.py`、`include/px4_powerplant_mission_cpp/mission_contract.hpp`、两侧测试。

**接口或架构变更**：新增 required 字段 `pose_source`、`slam_tracking`、`slam_loops`、`pose_jump_m`、`map_epoch`；`has_independent_position` 语义注释改为"独立于 PX4"。

**风险**：只改一侧会让消费端解析失败或静默降级；旧日志与本字段的兼容策略未定。

**验收标准**：

- 三方一致测试通过；
- 新旧样本各有正反例；
- 缺字段时发布侧与消费侧都明确失败，不静默降级。

### #12 门控与健康分支支持 slam

- 模板：`feature_request`
- 标签：`enhancement`、`ready-for-agent`
- 依赖：#11

**功能背景**：`localization_ready()` 与 `motion_health_reason()` 只覆盖 UWB/PX4 语义，SLAM 失锁会被报成含糊的"位姿过期"。

**目标**：失锁、跳变、地图纪元不一致都有明确原因与降级动作。

**涉及模块**：`src/offboard_control.cpp`、`include/px4_powerplant_mission_cpp/offboard.hpp`、`test/test_native.cpp`。

**接口或架构变更**：门控按源取阈值；health 原因增加 slam 分支。

**风险**：阈值过松会带着发散位姿继续飞；原因不可定位会让现场排障变成猜。

**验收标准**：

- 注入 `LOST` 后任务进 HOLD，不继续执行运动；
- 跳变超限触发降级并可审计；
- 失败原因能定位到具体条件。

### #13 配置段、profile 与 launch 参数

- 模板：`feature_request`
- 标签：`enhancement`、`ready-for-agent`
- 依赖：#8

**功能背景**：新节点需要与现有 profile 机制一致的分档，否则每条命令要手写一长串参数。

**目标**：`profile:=slam_simulation` 一条命令起全栈。

**涉及模块**：`config/powerplant.yaml`、`config/runtime_profiles.json`、`config/localization_slam.yaml`（新建）、`launch/powerplant_system.launch.py`。

**接口或架构变更**：新增 `powerplant_slam:` 配置段、`slam_simulation` profile、`use_slam` 与 launch `choices`。

**风险**：参数在两处定义（YAML 与 profile）会互相覆盖；profile 缺失会让 CI 与本地行为不一致。

**验收标准**：

- 一条命令起全栈且 `ros2 param dump` 完整；
- profile 解析单测覆盖新档；
- 与 `localization_slam.yaml` 无重复定义。

---

## S4 回环与地图一致性

### #14b 任务体素图的地图纪元与重建

- 模板：`feature_request`
- 标签：`enhancement`、`ready-for-agent`
- 依赖：#12

**功能背景**：SLAM 的回环只修正它自己的轨迹，voxel map 不会跟着变；不重建就会让规划建立在错位地图上。

**目标**：位姿跳变或跨图合并时，旧地图作废并重建，重建期间禁止规划授权。

**涉及模块**：`/slam/status` 契约、`localization_node`、`mission_state_machine.cpp`、`voxel_mapper_node.cpp`、`offboard_p2_map.cpp`。

**接口或架构变更**：新增回环/跳变事件字段；`localization_epoch` 与 `map_epoch` 联动。

**风险**：重建窗口内仍发布运动目标；重建失败后继续用旧图。

**验收标准**：

- 注入回环，地图重建完成且无错位；
- 重建期间不发布新的运动目标；
- 事件带时间、幅度与来源，可审计。

### #15b 飞行中失锁与重定位后的恢复规划

- 模板：`feature_request`
- 标签：`enhancement`、`ready-for-agent`
- 依赖：#14b、#15a

**功能背景**：没有重定位确认语义时，"追踪恢复"与"仍在漂移"无法区分。

**目标**：失锁判定、重定位确认与降级动作形成闭环。

**涉及模块**：`mission_state_machine.cpp`、`offboard_control.cpp`、`/slam/status`。

**接口或架构变更**：新增失锁/重定位状态与确认帧数参数。

**风险**：过早恢复巡航；误切换导致任务中断。

**验收标准**：

- 注入长时遮挡与快速旋转，检测延迟在上界内；
- 重定位未确认前不恢复巡航；
- 误切换次数为 0。

---

## S5 UWB 融合与校验

### #16 UWB 全局因子

- 模板：`feature_request`
- 标签：`enhancement`、`ready-for-agent`
- 依赖：#13

**功能背景**：借 VINS-Fusion 的 GPS 因子结构，但其标量方差、Huber 核、精确时间戳关联与全历史图都不适用于 UWB。

**目标**：UWB 作为鲁棒的绝对位置约束，不拉偏轨迹、不无界增长。

**涉及模块**：`px4_powerplant_slam` 包、`config/slam_*.yaml`。

**接口或架构变更**：新增协方差白化、鲁棒核、`max_dt` 关联与关键帧/滑窗参数。

**风险**：NLOS 野值造成米级拉偏；全历史图导致长时间运行内存增长。

**验收标准**：

- NLOS 注入不产生米级拉偏；
- 长时间运行内存有上界；
- 与校验通道使用不同锚点集合。

### #17 UWB 校验通道与 NIS 监控

- 模板：`feature_request`
- 标签：`enhancement`、`ready-for-agent`
- 依赖：#16

**功能背景**：校验通道若污染估计结果，验收就变成自证。

**目标**：独立的残差/NIS 监控与告警，不修改估计。

**涉及模块**：`px4_powerplant_slam` 包、`/slam/status`、`validation/slam/`。

**接口或架构变更**：新增校验残差、NIS、连续超限计数与告警字段。

**风险**：把 UWB 故障误判为 SLAM 漂移；告警与实际动作不一致。

**验收标准**：

- 注入 UWB 跳变能报"不一致"且位姿不受影响；
- 误报率与检测延迟有数据；
- 文档写明"不一致 ≠ SLAM 漂移"的判读规则。

---

## S6 验收

### #18b 集成后的 SITL 验收与证据归档

- 模板：`feature_request`
- 标签：`enhancement`、`ready-for-agent`
- 依赖：#14b、#17、#18a

**功能背景**：没有统一入口，指标无法回归，结论无法比对。

**目标**：一条命令跑完 SITL 验收并产出与[计划第 7 节](slam-plan.md#7-验收指标)一一对应的报告。

**涉及模块**：`validation/slam/`、`validation/tests/test_slam_*.py`、`validation/baseline/run_baseline.sh`、`validation/evidence/`。

**接口或架构变更**：无 runtime 接口变化；新增 validation 入口。

**风险**：结论跨 SITL/实机混用；失败场景被静默跳过。

**验收标准**：

- 一条命令出完整报告，失败项有原因；
- 指标全部达阈值；
- 报告区分"软件测试通过 / SITL 通过 / 实机通过"。

### #19 实机验收

- 模板：`feature_request`
- 标签：`enhancement`、`ready-for-human`
- 依赖：#18 与目标平台确认

**功能背景**：仓库内目前没有 Jetson 部署证据，实机结论无法由 SITL 替代。

**目标**：实机场景跑通并留下可追溯证据，在此之前对外只写"正在引入"。

**涉及模块**：实机配置、`validation/evidence/`、`docs/validation/`。

**接口或架构变更**：无；如需改 EV 偏移，由飞控侧参数承担。

**风险**：实机安全（失锁/跳变）；把单次成功当作能力证明。

**验收标准**：

- 失锁、UWB 断链、重定位至少各一次真实记录；
- 实机与 SITL 差异有解释；
- 结论明确区分三种"通过"。
