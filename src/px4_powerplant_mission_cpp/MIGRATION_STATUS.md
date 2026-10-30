# C++ 迁移进度与验证记录

更新：2026-10-30。此文件用于后续继续任务时保持范围与验证结论一致。

## 2026-10-30：Python mapper/controller 已移除

按用户确认的删除方案，Python `voxel_mapper_node`/`offboard_mission_node` 及其专用 `mapping/`、`path_planning/`、`common/initial_scan.py`、相关测试已删除；`setup.py` 移除两个入口，launch 移除 `mapping_package`/`control_package` 回退参数。C++ 测试的旧行为基准改为 `src/px4_powerplant_mission_cpp/test/fixtures/` 下的冻结 fixture，不再在测试运行时 import Python legacy。该删除是在用户明确豁免 Q9=A 的情况下执行的：完整 SITL 和实机地面链仍未通过，不能把删除当作飞行验收。删除后本机 P0 baseline：C++ 400 项、Python 72 项，均 0 失败；双进程 ROS 探针通过。

## 2026-10-29：A—D过渡策略、圆角停止几何与参考管理（最新）

本轮完成仅SITL起飞过渡的A—D白名单（0.10/0.12/0.15m，D半径0.48m且通过完整SDF审计），45秒固定预算及1.0/1.25候选参考管理。停止几何采用参考可达盒与圆柱机体的Minkowski和，外层AABB仅作检索；实际相交的未知/占据仍否决。严格ACK原子恢复0.10m、0.50m、0.60/0.35和3秒策略，并保存不可变的实际采样/参考/停止/版本证据。硬件缺省、PX4源码、world/碰撞及默认ROI不变。

修正实际ROS float32、NumPy布尔标志的验收适配，以及起飞规划容许数值微差却仍生成禁止偏航的逻辑不一致。所有修改保持硬跟踪、占据、时钟、0.75m上限和真实停止证明；不重置参考或改写旧失败日志。

最终两包串行构建通过，24组CTest与共享Python共 **524个直接用例通过，零失败、错误或跳过**。实际电厂`calibration_v6`完成20个策略/I组合，全部实际启动、全部参数回读及源码/资源稳定核对通过；**0次通过起飞初扫，未冻结配置**。5次取得严格ACK的11项瞬间证明，但后续4次严格硬跟踪故障、1次严格恢复超时；全部未进入HOVER_SCAN。整体最大模型原点上升约0.931m，不代表到达5m稳定悬停。

主要退出为11次硬跟踪、4次执行器本地时钟停滞、2次过渡恢复次数耗尽、2次45秒交接到期、1次严格恢复超时。接触19/20通过：A/I=0.5保留2条时钟/epoch未确认；18/20通过新鲜落地上锁收尾，C/I=0.1和0.25仍有独立收尾新鲜度失败。导航深度墙面投影检查20/20未通过、真值投影7次未通过，保留为额外输入/标定卡点，不能用扩大飞行容差掩盖。

按所有允许候选失败分支停止。没有governor关闭对照、默认ROI三次主链或独立ROI接入通过证据。实际时长倍率本组均为1.0，1.25及无候选恢复由软件回归覆盖；没有通过消融对，不能声称参数/几何/参考管理已提高任务成功率。Issue #3保持OPEN，代码/模型不提交、不推送。

原理见 [TAKEOFF_RESPONSE_SYNC.md](TAKEOFF_RESPONSE_SYNC.md)，完整记录见 [ISSUE3_VALIDATION.md](ISSUE3_VALIDATION.md) 及工作区 `log/issue3_margin_governor/20261029_1336/VALIDATION.md`、`final_summary.json`、`calibration_v6/`。旧35秒/五积分失败保留为历史基线，不混入新队列。

## 2026-10-29：起飞响应同步与独立SITL标定（此前基线）

代码已接入采样时间对齐的预测风险、提前制动与会话恢复锁存、物理起落架净空证明、启动意图/反馈探测/严格交接进度，以及预先固定的35秒完整过渡预算。基础容差0.10m、地面过渡0.75m和原硬故障/占据/停止检查保持。普通仿真及硬件profile均可正常IDLE；实际world支持审计后才显式启用临时地面响应配置。没有修改PX4源码或实机缺省。

两包串行构建和最新 **506个直接回归用例** 通过。实际电厂按2、1、0.5、0.25、0.1测试垂向积分，hover初值按模型推导为0.7463305342、全部参数实际回读匹配；五个候选均未通过起飞初扫，**未生成冻结配置，未执行默认ROI三次正式主链或独立ROI接入**。最高有限模型原点上升约7.9cm，不能称完整离地/悬停。标定期间暴露的初始化偏航与已发布HOLD偏航不一致已修复，并从2.0重新测试；旧失败不混入冻结。

当前需要进一步核查控制/导航的实际响应和收敛：低积分运行仍受垂向响应及横向导航/参考误差约束，软恢复无法满足；保留受控降落或硬FAULT分类。发布速率49.886～50.002Hz、p95/p99及最大间隔符合原门限，不能归为本次发布频率瓶颈。完整范围、接触未确认项和原始证据见 [ISSUE3_VALIDATION.md](ISSUE3_VALIDATION.md)、[TAKEOFF_RESPONSE_SYNC.md](TAKEOFF_RESPONSE_SYNC.md) 与 `log/issue3_response_sync/20261029_0150/`。Issue #3保持开启；未提交、未推送。

## 2026-10-28：Issue #3 碰撞资产与局部任务 ROI

本节历史实测：真实GPU渲染和明确5Hz降载已进入SAFE_TAKEOFF，但停止包络下界越界约1.03mm触发FAULT，尚无初扫/ROI接入/返航通过证据。版本绑定的实际GPU雷达方向、深度一致性已通过；名义角度与在线mapper误差单独保留。详细状态及不重复计数的软件结果以 [ISSUE3_VALIDATION.md](ISSUE3_VALIDATION.md) 为准，Issue #3继续开启。

本次修正电厂碰撞资产，并将局部任务区域从全阶段导航安全域中分离。实现、版本与验收入口分别见 [ISSUE3_IMPLEMENTATION.md](ISSUE3_IMPLEMENTATION.md)、[ISSUE3_VALIDATION.md](ISSUE3_VALIDATION.md)；下方 316 项软件回归和 CPU17 飞行失败保留为 2026-10-26 的历史证据。

- 原视觉 STL 保持不变；实心碰撞盒改为保留凹结构的独立静态简化 mesh。最终 **501262 面**，较原 605740 面减少约 **17.25%**。全部顶点和三角形质心的独立双向复验最大距离约 **19.08 mm**；这是有限验证集结果，不是连续 Hausdorff 证明。
- 资产已形成模型仓库的本地可检出提交 `69f6511d0b11e2189b204ce6b10f12cbef24f119`，未推送，不含已有 `x500_depth` 修改。主仓库 `config/powerplant_assets.json` 固定资产提交及递归文件哈希；实际电厂准备阶段发现版本或资源不匹配时拒绝沿用旧验收。
- `explore_min/max_*` 现在定义局部任务 ROI（默认 x=[-24,34]、y=[-9,49]、z=[3.5,7] m），用于收益、任务表面和完成范围。起飞、初扫、已知安全接入/绕障/取景和返航使用完整导航证据域；新增 bounds 不生成空闲体素，占据仍否决确认柱体。
- `EXPLORE` 内增加 `ROI_WORK / ROI_ACCESS`。没有可用 ROI 工作视点时，在 home 与 ROI 联合 AABB 外扩 10 m 的范围内自主探测入口；接入时间按 steady clock 累计，默认 60 s，跨模式不清零，重访代价默认 5 s。接入失败、超时或返航余量不足不报告 ROI 完成。
- 二维初扫投影包含当前位姿及当前飞行高度带；原始融合、新鲜度、姿态和安全首段条件保留。ROI/接入参数只在 IDLE 接受有效原子修改，独立 revision 阻止旧结果采用；一般机体/制动参数仍不是受支持的飞行中热改接口。
- 注册后的真实目标表面须与任务 ROI 相关；外部面片不进入覆盖分母。新的覆盖 scope 为 `reachable_modeled_surface_in_task_roi`，与图像中的目标 ROI、落地 COMPLETE 及全电厂完成分开。

本轮已记录的验证：两包 Release 串行构建通过；C++/对照/跨进程 ROS 等 `colcon` 汇总 **235 项通过**（含 17 个 CTest 外层结果，不重复计数）；共享 Python **127 项通过**。完整和简化 mesh 的动态双侧接触、阳性接触对照及 3 条起飞区自由路径已有通过证据。对应运行目录为 `log/issue3_powerplant/20261027_204849/`。

正式场景入口为 `run_p2_sitl.py --scenario powerplant`；`room/two_rooms` 继续作回归。`--stop-after scan`、`--roi-access-test --stop-after roi-access` 与 `--stop-after mission` 分别标记起飞初扫、接入进展和局部任务范围；真实电厂没有已核验标注 fixture 时，拒绝复用小房间圆柱检测重放。本轮417个直接软件用例通过；正确绑定资源后的六组原始传感器负载成本对照通过5%退化检查，旧未绑定对照作废。**SITL仍未通过，完整巡检未验收**，具体范围以 [ISSUE3_VALIDATION.md](ISSUE3_VALIDATION.md) 为准。

## 2026-10-26：P2 软件逻辑核查与回归完成

已接入无求解器五次时序轨迹、持续反应/制动包络、独立 50 Hz 执行、三维区域/门户引导、同步 RGB-D 注册及稳定表面/合格照片覆盖闭环。主流程不变，`auto_start=false`。任务规格与缺口见 [Issue #2](https://github.com/pawn-queen/px4_ros_ws/issues/2)，原理、参数与接口见 [P2_IMPLEMENTATION.md](P2_IMPLEMENTATION.md)，真实仿真入口和证据范围见 [P2_VALIDATION.md](P2_VALIDATION.md)。

按用户最后确认，本轮以测试和已有日志完成软件逻辑核查，飞控/动力学参数由用户调整。本次发现的下列逻辑问题已修复，完整回归通过；没有把旧 SITL 失败改写为飞行通过，也未继续调飞控参数。

- 完整 ENU 位置协方差、FLU 三轴速度及相关线速度协方差保留，速度经完整姿态转换到 ENU。停止盒采用同一误差椭球的各轴投影；最大特征值的范数门控与各轴跟踪门控同时成立。任一方向不确定度越界均制动、停稳后重建。状态/照片事件提供停止盒和可空的额外净空下界。
- 轨迹生成、未来路线复核和旧快照释放在 worker；当前停止包络逐次检查。取消任务不删除备用停止，地图复核超时不能被连续版本刷新。
- 区域排序进入候选配额；预算只筛选实际执行视点，不能删除已建模可达面片的覆盖责任。过期的未确认目标假设可重新确认，已确认实例保持身份。
- 校正图、双 CameraInfo、源图曝光和采样位姿共同决定注册；仅合格且保存成功的实际源图面片计入覆盖。落地 COMPLETE 与巡检结果分开。
- SITL 暴露单帧 UWB 统计拒绝被误当设备故障的问题，已修正精确采样语义：屏蔽被拒帧，不刷新旧测量 TTL；当前样本撤销、缺时间戳和硬故障仍立即失效。阈值和测距噪声未放宽。
- SITL12 加入真实回环 MAVLink GCS 心跳，保留并实测确认 `NAV_DLL_ACT=2`，PX4 已解锁并进入 Offboard；暴露反复缩小证书预算导致慢速起步停滞的问题。后续准备保留同任务已验证上界，实时停止检查仍使用当前输入。确定性红/绿回归及显式重启清历史均已通过。
- 跨进程回归发现深度/CameraInfo 交付顺序竞态，已取消原生 mapper 的无标定名义内参回退，改为有界等待；确定性测试先失败后通过。
- 长竖直五次段在速度限幅主导时起步过慢，起飞改为由速度/加速度交叉尺度确定的短上升参考，保持初始化偏航。各段须在已解锁 Offboard 下完成当前参考并实测停稳，最后高度才进入初扫；小于位置容差的短段不得跳过。
- CPU15 日志证实短暂峰值在停稳前回落，已增加独立的有效输入峰值预算，必须经新完整几何验证才能采用；CPU16 已消除不确定度反复制动。原始制动原因现在保留到停稳，不再被通用路线切换覆盖。
- 已接入从连续 HOLD 参考开始的 2 cm/4.5 秒一致微上升及实际启动参数只读核对。CPU17 已走完该参考再进入正常上升，但仍未满足实测跟踪包络；该参数初值不是飞行标定结果，硬件模型默认禁用。
- 跟踪门控按导航采样时间解析参考阶段历史，跨 TRACKING/BRAKING/HOLD 不用当前命令冒充过去参考；停止盒仍包含完整延迟。新 HOLD 需要该阶段的新测量，重置/采用不能串坐标 epoch。
- 规划与执行共用保守误差预算快照，评分前分别验证完整路径及实际首局部段；贪心截断、拐角停车和返航偏航均进入真实时序预算。局部运动拒绝保留具体原因，不发布成功选择事件。
- 巡检评分使用去返时序，回调拒绝不退回距离估计；覆盖提交严格绑定真实曝光与面片版本。同 epoch 的配置变更使用新区域 ID，避免旧重访/滞回记忆串区域。

最新串行回归（`log/p2_baseline/`）：

| 检查 | 实际结果 |
| --- | --- |
| 两包 Release 构建，单编译任务 | 通过，最新 `log/p2_build_logic_review.log`，5 分 42 秒，无编译 warning/error |
| C++ 原生 | **156 项通过**，含时间对齐、停稳新测量、预算/采用、区域 ID 和覆盖证据回归 |
| 几何/算法对照 | **26 项通过** |
| 跨进程 ROS 契约 | **7 项通过**，含完整速度协方差、有限 PVA、停止盒/净空字段 |
| Python 包回归 | **127 项通过**，含新 10 项 UWB 拒绝/TTL/EV 用例 |

共 **316 个直接用例**。`colcon test-result` 为 **204 项**，其中另有 15 个 CTest 外层结果，不是额外行为用例。全部零失败、零跳过；最新 C++/对照/ROS 结果为 `log/p2_baseline/native_logic_review_result.log`，Python 结果为 `python_bootstrap.xunit.xml`（6.13 秒）。另有七种启动参数核对检查通过，见 `log/p2_startup_config_checks.json`。下方 P1/P0 是历史记录。

SITL 已实际启动隔离 PX4、Agent、Gazebo 与任务栈，保留至 CPU17 的各次报告。原始渲染负载和 CPU 10 Hz 档曾触发健康失鲜；2.5 Hz 档用于降载功能验证，静态导航/外参和真实 YOLO 源图链路的独立检查已通过。CPU17 完整流程未通过，且发生在本轮参考采样时间等逻辑修复之前；新软件不据此宣称飞行结果。后续参数标定、完整飞行、多房间/物理制动、满负载时延及实机验收由用户继续，操作边界见 P2_VALIDATION.md。

## 2026-10-25：P1 建图效率与观测有效性

本节是 P1 完成时的历史记录：实施用户确认的 **P1 全部项目**，保留 P0 的健康门控、坐标 epoch、起飞柱体与落地确认。原理变化、公式、参数和限制集中见 [P1_IMPLEMENTATION.md](P1_IMPLEMENTATION.md)。当时 P2/P3 尚未实施，P2 最新进展见上节。

- 射线改为按体素面推进的精确 DDA，仅正长度交段提供空闲证据；保留批次去重和命中优先，不用碰撞 supercover 激进清空地图。
- 深度按距离投影足迹、深度/有效性边缘细分，选择真实像素并按图像分区分配射线预算。未观测像素不填充覆盖；遍历深度图的 CPU 成本仍需测量。
- 地图分类差分驱动前沿与净空局部重算；不可变分片索引供 worker 共享，保留有限版本变化历史。完整地图传输、解析和差分仍有全图成本。
- 成功融合的实际 CameraInfo、主点与尺寸提供非对称矩形视锥；缺省要求真实匹配标定。每个候选 yaw 单独检查转向、观察停留、去返路径时间和预算。
- HOVER_SCAN 以实际合格深度射线方向覆盖、稳定状态及已知空闲首段共同退出。固定 yaw 步数、融合计数不能代替覆盖证据；缺失深度时只在确认柱体内慢偏航，雷达失鲜时停止转动。
- 探索视点须对准、停留并取得新合格深度才完成，按位置/yaw 记忆；附近几何变化使记忆失效。重规划分类区分地图等待、标定、定位差、路径受阻、余量不足、局部无候选与前沿耗尽。局部无候选扩大一次搜索，但候选截断不等同于全图完成。
- 异步结果按 epoch、相机/配置、收益区域及完整去返走廊验证；远处变化允许复用，采用前复查最新路径、定位预测与预算。

本轮实际验证（串行，无 Gazebo/飞控）：

| 检查 | 结果 |
| --- | --- |
| 两个包 Release 单编译任务构建 | 通过 |
| C++ 原生测试 | **62 项通过**：原 P0 35、P1 射线/采样 8、增量索引 6、规划 7、任务 6 |
| 几何与算法对照 | **26 项通过**；精确射线使用独立几何参考，旧规划对照显式中和新转向/停留成本 |
| 跨进程 ROS 契约 | **7 项通过**；增加实际相机/观测元数据断言，保留 simulation/hardware 定位与 EV |
| Python 包回归 | **113 项通过** |
| 同机 DDS 话题/服务探针 | 通过，继续使用临时 Cyclone DDS 配置 |

上述为 **208 个直接测试用例**。`colcon test-result` 的 105 项还包括 10 个 CTest 外层结果，不是额外行为用例。射线测试包含 200 组独立线段/AABB 几何对照；增量索引包含随机全量参考对照与旧快照不变性；初扫包含旧帧、倾角/质量拒绝、缺失深度及雷达失鲜；视点测试验证不能仅靠计数完成观测。

复现命令仍为 `scripts/run_p0_baseline.sh`，名字为保留原入口，CMake 已纳入 P1 全部测试。结构化结果位于 `build/px4_powerplant_mission_cpp/test_results/` 和 `log/p0_baseline/`。

**边界：**当前相机模型要求已校正 pinhole 深度与 CameraInfo.K 匹配；未来视点假设水平飞行，未增加任意安装旋转。路径仍为离散已知空闲几何路径，未实现连续机体碰撞证明、动力学制动或动态障碍预测。未完成完整 PX4/Gazebo 飞行、真实传感器标定及大型地图/YOLO 并行负载下的尾延迟测量，不据此宣称性能加速比或实飞验收通过。下方 P0 记录保留当时结果。

## 2026-10-25：P0 输入、地图与任务状态改进

已修改并构建 C++ 建图/控制与共享 Python 定位状态接口。仿真和实机共用这些实现，launch 对定位、建图、YOLO、控制统一传入 `use_sim_time`。以下为 P0 完成时记录；P1 后续进展见上节。

- 新增共享采样时间/六维协方差检查，采样与接收年龄分开，另用单调时钟检测流停止和仿真时钟冻结。重复、乱序旧包不刷新健康；同时间戳的质量退化仍撤销有效状态。
- 定位状态新增时间、序号和坐标 epoch；PX4 估计器 reset、时钟回退和节点重启不能静默拼接旧地图/home。mapper 质量不合格时冻结积分，普通延迟包不再触发历史清图。
- 新地图健康协议包括 epoch、序号、定位与传感器采样时间及点云指纹；保留四字段点云，但旧健康协议不能授权原生控制器飞行。
- 全阶段运动前检查导航、独立定位、飞控状态和预算。不可恢复的本次任务故障锁定为 FAULT，取消任务并停止 heartbeat/setpoint；恢复数据不会自动重新解锁，由已配置的 PX4 Offboard-loss 行为接管。
- 起飞服务要求新鲜导航、飞控已上锁、已确认且足够大的起飞柱体；起飞/返航高度改为相对实际 home。缺省确认关闭、高度为零，不能直接拿示例值当作物理标定。
- 落地必须由命令之后的新鲜 landed 与 disarmed 状态持续确认。超时保留 LANDING 并报告未确认。无新鲜图像、无图 fallback 或写盘失败都不再完成巡检照片计数。

本机 Fast DDS 的原始故障仍表现为单向发现、无数据/服务响应，回环 UDP/显式 locator/TCP 对照未解决，根因未确定。采用**仅当前 shell 生效的 Cyclone DDS + LOCALHOST 配置**后，同机双进程话题与服务恢复，原生地图/控制及 Python 定位/EV 的跨进程验证已通过。无 sudo 安装权限时，锁定 SHA256 的五个 ROS 包已验证后提取到被 git 忽略的 `.p0_deps/`；没有修改系统网络、`.bashrc` 或永久 DDS 默认设置。

本轮验证结果：

| 检查 | 结果 |
| --- | --- |
| 两个包串行构建（Release，单编译任务） | 通过 |
| C++ 原生测试 | **35 项通过**（任务/几何 21、共享输入 4、地图健康 6、mapper 健康 4） |
| Python/C++ 原算法对照 | **26 项通过** |
| 跨进程 ROS 契约 | **7 项通过**：参数 2、建图 1、Offboard/落地 1、旧包重放故障 1、simulation/hardware 定位与 EV 2 |
| Python 包回归 | **113 项通过** |
| 双进程 ROS 探针 | 话题接收和服务应答均通过；结构化结果保存至 `log/p0_baseline/transport.json` |
| 输入重放/故障注入 | 旧包、质量退化、接收冻结、epoch 重置/迟到、点云错配、起飞柱体、全阶段定位故障与降落确认均有用例 |

跨进程定位测试实际启动独立的 localization、EV 和注入/断言进程，分别使用 simulation/hardware profile；检查独立 UWB→EV、PX4→完整导航，停止 UWB 后导航协方差退化且新 PX4 状态不产生 EV 反馈。这里使用合成 ROS 消息，没有连接 Host 串口硬件。

运行入口、依赖锁定、配置及新协议见 [P0_VALIDATION.md](P0_VALIDATION.md)。`scripts/run_p0_baseline.sh` 串行运行探针、C++/对照/跨进程测试和 Python 回归并保存结果；不会启动 Gazebo、Agent、真实飞控或电机。下方 2026-10-24/21 的“未通过”是历史结果，本轮仅在上述新 middleware 配置下解决同机通信；不能据此断言默认 Fast DDS 已修好。

**仍待验收：**完整 PX4/Gazebo 飞行、Agent/PX4 与所选 DDS 的实际互通、真实 UWB/相机/雷达标定及故障数据、起飞顶棚净空、PX4 Offboard-loss 配置和反应、连续动力学制动与真实电量裕量。LOCALHOST 基线不验证 Windows Host 或跨计算机 DDS。原 Python mapper/controller 已于 2026-10-30 删除；建图/控制只保留 C++ 实现。

## 2026-10-24：仿真/实机共用定位链路

本次修改 Python 定位、EV、共享输入检查、配置与 launch；C++ 建图/控制代码及消息接口未改。下方 C++ 迁移记录仍是 2026-10-21 的历史验证结果。

- `localization/uwb.py` 的解算接口保持不变，Host 继续复用；没有用示例锚点冒充实测标定。
- EV 直接订阅独立 UWB 位姿，取消融合里程计输入，保留采样时间、位置协方差并去重；单 Tag 默认不发布姿态/速度。仿真可显式允许独立真值姿态。
- 默认导航使用 PX4 完整位姿，转换完整四元数、协方差和 twist，保留状态采样时间。`fusion.py` 与旧 adaptive 导航模式保留，结果不进入 EV。
- 共用 frame/坐标变换、质量与健康事件处理；UWB 失效使导航协方差退化，PX4 位姿过期停止发布，时钟回退清缓存。
- 新增 `localization_profile:=simulation|hardware`，hardware 默认关闭 UWB 仿真器、Gazebo 真值/时钟桥和仿真时间。默认控制与 YOLO 仍关闭。
- 实机锚点坐标、航向、map/PX4 对齐与天线安装偏移仍需标定。配置方法与已知边界见 [定位部署说明](../px4_powerplant_mission/LOCALIZATION_DEPLOYMENT.md)。

本次实际验证：

| 检查 | 结果 |
| --- | --- |
| 系统 Python / colcon 构建 Python 包 | 通过 |
| Python 包完整回归 | **112 项通过** |
| 同进程真实 ROS Executor，仿真姿态/硬件单 Tag 两种输入 | 通过，收到 EV 与导航话题，检查无融合里程计反馈 |
| 实际 Host 的 `solution_to_pose` / `quality_payload` 接口 | 通过；保留原始时间、完整位置协方差与未知姿态标记 |
| launch 参数及 simulation/hardware 默认开关 | 通过 |
| 两节点跨进程消息联调 | **未通过**：hardware 默认 DDS 和 simulation 临时 UDP transport 均能发现 2 个 UWB、1 个 PX4 订阅者，但 15 秒内收不到导航或 EV 消息 |
| git diff 空白检查 | 通过 |

跨进程问题与既有 DDS 阻塞一致；没有更改永久网络/DDS 配置，也不能由此断言故障根因。由于跨进程数据路径未恢复，本次没有完成 PX4/Gazebo 全链路飞行验收；没有连接真实飞控或 UWB 串口设备。不能将 112 项测试解释为实飞验证通过。下一步仍需先修复 DDS，再验收 Agent/PX4 和真实传感器数据路径。

复现本次回归：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  PYTHONPATH=src/px4_powerplant_mission:$PYTHONPATH /usr/bin/python3 -m pytest \
  -p no:cacheprovider -q src/px4_powerplant_mission/test
```

## 已实施（2026-10-21 C++ 迁移）

- 独立 `ament_cmake` C++17 包：体素概率建图、采样时刻位姿插值、三维已知空闲路径与前沿视点规划。
- Offboard 已拆分为 ROS 接口、地图快照接收、任务状态机、规划调度、PX4 控制与照片保存模块。
- 规划和保存照片分别使用单工作线程，队列最多保留一个待执行任务；过期规划结果不能直接替换当前路线。
- 保留节点名、话题/消息类型、QoS、ENU/NED 转换、状态字符串、JSON 字段及原 YAML。参数声明对应原建图器 47 项、控制器 104 项；其 ROS 参数服务验证仍待通过，见下文。
- 保留独立定位检查、姿态门控、setpoint 预热、解锁检查、限速及已知空闲路径检查。地图云和融合状态按时间戳精确配对，不完整地图不能授权探索。
- `powerplant_system.launch.py` 默认把建图、控制路由到 C++ 包；`use_control=false`、`use_yolo=false` 和 YAML `auto_start=false` 保持不变。
- 原包仅修改 launch 接线、package.xml 依赖及 README；原 Python 节点、算法和 YAML 均未修改或删除。

Python 回退已移除：`mapping_package`/`control_package` 参数和两个 Python 入口均不存在。下方“已验证”表是删除前的历史记录，保留用于追溯。

## 已验证

| 检查 | 结果 |
| --- | --- |
| 两个包的系统 Python / colcon 串行构建 | 通过 |
| 原 Python 包测试 | 95 项通过 |
| C++ 原生测试 | 17 项通过 |
| Python/C++ 算法对照 | 26 项通过 |
| launch 参数检查（历史） | 删除前默认选择 C++、保留 Python 回退开关；该开关已于 2026-10-30 移除 |
| 默认 C++ 两节点加载原 YAML、短时运行、SIGINT 退出 | 通过；未触发任务，无 PX4/Gazebo |
| Python 回退启动（历史） | 删除前确认启动原 Python 两节点；该路径已于 2026-10-30 删除 |

原生测试包括位姿插值/时间回退、点云双精度/字节序/行填充、地图完整性与新鲜度、异步任务取消、起飞门控、初扫超时、目标抢占、返航安全、限速、电量预算，以及照片保存期间地图失效后的完成处理。

对照测试复用原三维规划的 9 个公开接口场景（不包含 2 个内部可见性辅助函数测试），另有 8 组随机体素射线、8 组随机障碍路径和 1 组命中优先级/配置校验。没有据此宣称所有输入完全等价或给出性能加速倍数。

## 未通过与待验收

`ros_contract` 的 4 项跨进程 ROS 测试尚未通过：两个节点的参数服务响应、建图消息接收、Offboard 消息与服务接收均出现超时。独立的最小 Python 发布/订阅程序也收不到消息；本地 UDP 回环和 ROS 定时器能工作。默认 DDS、localhost、静态回环 peer 和临时 UDP 回环配置的诊断未解决问题，根因仍需定位。没有修改机器的永久 DDS/网络设置。

因此，当前不能宣称 ROS 接口联调全部通过。下一步先恢复本机跨进程 ROS 通信，再运行 `ros_contract`，处理其可能揭示的接口差异。随后按原 README 完成 DDS Agent、PX4/Gazebo、定位与外部视觉、深度/雷达建图、初扫、探索、巡检拍照和返航降落的完整仿真验收。本次未启动完整 Gazebo 飞行，也未测量大型地图下的控制周期延迟。

等分候选在 C++ 中按体素键确定顺序，可能选择与 Python 集合遍历不同但同等评分的视点；照片事件在异步写入结束后的控制周期发布。这两项已知差异详见 README。

## 继续验证的命令与资源约束

始终先完成编译再测试；不要同时运行编译、测试和 Gazebo。

```bash
cd /home/pawn/px4_ros_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 colcon build --symlink-install \
  --packages-select px4_powerplant_mission_cpp px4_powerplant_mission \
  --executor sequential --parallel-workers 1 \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release

PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  CTEST_PARALLEL_LEVEL=1 colcon test --packages-select px4_powerplant_mission_cpp \
  --executor sequential --ctest-args -j1 --output-on-failure
colcon test-result --test-result-base build/px4_powerplant_mission_cpp --verbose

PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  PYTHONPATH=src/px4_powerplant_mission:$PYTHONPATH /usr/bin/python3 -m pytest \
  -p no:cacheprovider -q src/px4_powerplant_mission/test
```

ROS 接口测试默认使用隔离的 `ROS_DOMAIN_ID=173`，一次仅启动一个原生节点，注入合成传感器/PX4 消息，不连接真实飞行器。

### 2026-10-28 地面保护续改

C++ 已接入三类起飞授权、实际支持模型、采样时间与解锁时间域检查、执行器严格交接、保守过渡及独立探索时限。SITL runner 支持临时接触监测、源/二进制/资源哈希、参数回读和故障后落地证据。原1.03mm案例在严格模式仍拒绝，合格地面证书可处理。实际主链仍因原跟踪保护未通过，Issue #3保持开启；尚未验收初扫、ROI接入或正常任务返航。详见 TAKEOFF_GROUND_SUPPORT.md 与 ISSUE3_VALIDATION.md。
