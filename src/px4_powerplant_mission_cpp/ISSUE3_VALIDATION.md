# Issue #3：实际电厂验证记录

验证日期：2026-10-28。此前资产/性能证据根目录为 `log/issue3_powerplant/20261027_204849/`；地面保护续改为 `log/issue3_takeoff/20261028_143128/`；最新基础容差0.10m对照为 `log/issue3_tolerance010/20261028_212122/`。仅实际 `powerplant.sdf` 为正式场景；真值与探针只进入评估，不向在线规划注入地图或航点。**目前不能关闭 Issue #3：原1.03mm地面授权问题已用独立支持证书处理，0.10m对照两次仍在真实离地前触发跟踪保护；初扫、ROI接入及正常任务返航尚未通过。** 下方原记录的相对路径仍以此前资产/性能目录为根，最新续改单列于文末。

## 资产与几何：通过

模型仓库本地提交 `69f6511d0b11e2189b204ce6b10f12cbef24f119`，未推送。包括电厂碰撞资源、可重建脚本与此前授权围护；已有 `x500_depth/model.sdf` 修改未纳入。主仓库 `config/powerplant_assets.json` 固定提交和递归资源哈希，不符则拒绝沿用验收。

| 项目 | 结果 |
|---|---|
| 原视觉 STL | 605740 面；字节保持不变 |
| 完整碰撞对照 | 仅清理重复、退化与孤立点，597936 面；没有减面 |
| 最终简化碰撞 | 501262 面，相对原 STL 减少 17.25%；25,063,184 字节 |
| 原→碰撞：全部顶点与面质心 | 892053 个验证点；p99 5.999 mm、max 19.077 mm |
| 碰撞→原：全部顶点与面质心 | 747042 个验证点；p99 4.649 mm、max 15.238 mm |
| 围护间距 | STL 到墙最小 5.960983 m，到顶棚 9.287629 m |

证据：`collision_dense_verification.json`，模型中的 `collision_validation.json`、`collision_manifest.json`。有限采样距离不是连续 Hausdorff 上界。薄件、小件、非流形件及不满足独立约束的部分保留；没有全局凸包、容差焊接或自动填孔。

关键 SHA256：

- world：`8e8af17b20101cc852610673cbb5267e09c492f66553911bed3f1886a46aeee7`
- visual：`076476fcef7535cb1c24d8fc7dffab21987d906f7464a3dbf91374f7d49cdf87`
- collision：`0eb279ccb4f3a28f922872d9d67e909905ab23085f098944e521071c23637cf0`
- 完整清理对照：`89490f0bf4bd796651670c30a66fee8a9989da22ca485273dc2c7954649a8967`

## 构建与软件回归：通过

两包 Release 串行构建通过；新增 harness 后再次增量构建通过（`build_final_with_env.log`）。最终已登记 XML 直接用例 **417 项，零失败、错误或跳过**：原生179、Python/C++ parity26、跨进程ROS7、资产/runner32、GCS20、传感器校验23、球体扫掠3、共享Python127。见 `direct_test_summary.json`。不重复计入 CTest 包装或同组重复执行记录。

其中包括：ROI外起飞/初扫/返航与运动域、ROI增益和接入排序、累计预算、ROI外目标/面片、revision失效；真实 SDF 嵌套资源展开；真实 UDP/MAVLink 协议进程的只读心跳请求与 ACK 无法续健康；单层雷达 NaN 角步长。跨进程 ROS 测试属于接口证据，不是电厂实飞证据。

早期 Python 收集错误来自启动命令覆盖了 ROS 的 PYTHONPATH；修正环境后127项通过。最终构建也曾漏source本地P0中间件环境，补齐既有p0_ros_env后通过。新增真实SDF回归的最初临时model.config缺少合法版本元数据而触发上游解析器崩溃，补成有效模型描述后通过。失败日志均保留；没有降低业务断言或跳过用例。

## 原生动态接触与起飞域

实际加载 Gazebo Sim8.11.0、gz-physics7.6.0、DART6.13.2/ODE，加载库路径记录在各报告。每组独立分区，同一真实 world；完整对照仅通过临时模型描述切换碰撞 URI。

原 `probes_v3_*` 记录每组24次双侧表面接触、1个围墙阳性对照、3条原点轴/水平通行。**同样受旧overlay优先级影响，不能当作完整/简化对照；原模型即已修复的简化网格，简化物理接触证据可保留。**实际接触位置最大误差约 `1.95e-8 m`，记录首个接触、实际位姿、仿真步进和终点误差。原批次偏低处、且首交点未变化，不能代替高处设备/简化区域验收。

补充脚本生成7个3.5–7m高处双侧表面，含2个真正减面改变区域、立柱和设备；另有一条2m设备间通道，双mesh连续表面净空下界1.72338m。

初版 `supplemental_fixtures.json` 用中心射线预计球体接触，导致一个实际碰到相邻边缘的探针被误判6.5cm。没有放宽2.5cm门限：改为对两份完整表面独立计算有限球体扫掠（距离1-Lipschitz下界），保持原起点、速度和半径，并输出 `supplemental_fixtures_sphere.json`。独立预测将原接触误差解释为0.156mm；`probe_sweep_diagnosis.json`、红绿回归及旧失败批次均保留。

- `probes_sphere_simplified/report.json`：14次高处接触+围墙阳性对照+设备间通道均通过，最大接触对原表面预期误差14.447mm，通道终点误差1.6mm。简化文件与实际原模型相同，因此旧路径优先级不影响该组；现场物理库SHA见 `runtime_physics_capture.json`。
- `probes_bound_complete/report.json`：正确资源绑定的完整网格重新运行同组，全部通过，最大接触预期误差0.156mm。展开world和真实完整mesh SHA记录在报告；完整/简化不再混淆。库版本与哈希在 `runtime_physics_libraries`。
- 旧 `probes_sphere_complete` 也受overlay优先级影响，不能算完整组，已由上述bound批次替代。

最终 `sitl_prepare_final/takeoff_domain_audit.json` 通过：对原点 `[-2,2]²×[0.04,6.3]m` 棱柱检查原始/碰撞三角形与围护均分离，并独立验证棱柱内种子点在两份mesh各945个连通部件的所有AABB之外。后者避免把“整个棱柱位于封闭实心体内部、却不碰表面”误报为空闲。几何与配置哈希绑定后才授予柱体确认；地面z=0接触带单列，不声称其为无障碍飞行空间。静态名义包络通过不等于运行中的导航误差、制动或真实起飞响应通过。

## 原始传感器负载性能：正确绑定后重跑通过（旧对照作废）

最初六组虽按30s预热/60s采样完成，但后续用实际 `gz sdf -p` 复核发现：验证器只让 GZ_SIM_RESOURCE_PATH 优先overlay，而SDF_PATH仍优先原模型，导致标为完整的组也加载简化碰撞。**此前“9项退化不超过5%”结论撤回，不能当作有效比较。** 原报告保存在 `performance_comparison_original_unbound.json`，失效证据为 `collision_overlay_red.json`。

现在同一环境的两个资源路径都优先overlay，并在启动前展开真实world，拒绝与请求不符的碰撞URI；记录展开SDF、实际mesh路径/SHA，结束时再核对文件未变。汇总器要求每组有这项实际资源绑定、两组真实哈希不同、同一已识别渲染器；旧报告自动不通过该检查。

重跑使用冻结的校验脚本和哈希（`performance_bound_gpu/validator_snapshot_hashes.json`），实际NVIDIA、原始RGB1920×1080@30、深度640×480@30、360点雷达@15；相同world/后端/烟雾/种子，完整与简化各3次、各30s预热+60s采样。结果见 `performance_bound_gpu/performance_comparison.json`，全部资源/重复/后端核对及9项5%退化检查通过。

| 指标：3次中位数 | 完整网格 | 简化网格 | 退化（负值为改善） |
|---|---:|---:|---:|
| CPU多核合计，% | 150.81 | 151.46 | +0.43% |
| RSS，GB | 1.028 | 0.983 | -4.43% |
| 峰值RSS，GB | 1.046 | 0.989 | -5.46% |
| 启动至首stats，s | 2.478 | 2.474 | -0.17% |
| 实时因子 | 0.2673 | 0.2734 | -2.24% |
| RGB间隔p95，s | 0.1587 | 0.1508 | -4.95% |
| 深度间隔p95，s | 0.1590 | 0.1507 | -5.17% |
| 雷达间隔p95，s | 0.2715 | 0.2541 | -6.43% |

范围仍是整场负载、静态传感器和外围墙慢速探针，不单独证明电厂网格碰撞查询吞吐；三次中位数不是统计保证，也不代替带任务与YOLO的实时飞行验收。

## 深度/雷达与视觉、碰撞：实际采样模型通过，名义角度误差保留

原始样本、CameraInfo、传感器SDF及运行时Scene记录在 `sensor_validation_clear_view/`。校验使用实际外参、采样时刻和内参；此安装版本gz-sensors8.2.2的LaserScan.world_pose存的是sensor-local，只有Scene父链、源配置和加载库身份相符时才允许显式按local解读。

已修复校验器单层 `vertical_angle_step=NaN` 被 `0×NaN` 污染为无效射线的问题，多层仍严格验证。旧全部归入烟雾的结果保留，不能算通过。

修复后同一原始数据独立重放（`sensor_geometry_report_fixed.json`）：

- 深度899个可比较样本，sensor→visual p95 2.648cm，通过8cm门限；visual→collision max约1.145cm。
- 雷达169个可比较样本，sensor→visual p95 38.017cm、max约1.852m，**失败**；另外6条有限回波无相应视觉首交点。
- 同名义雷达射线 visual→collision max约4.177cm，通过5cm门限；它不能替代实际雷达一致性。

独立源码预测已确认根因：360×1 GPU雷达使用128×128立方体面最近邻采样，名义方向与实际纹理中心方向最大差0.633°。未使用回波拟合方向、未删除异常射线，旧数据全部175个有限回波的源码方向预测最大误差约0.038mm，6个无视觉首交点也归零；旧数据缺少渲染器历史哈希，单凭这项诊断不算正式通过。

已重新采集 `sensor_validation_bound_renderer/`，记录真实加载的gz-rendering8.2.3、OgreNext/GL3Plus2.3.3库、5个材质/compositor/shader文件及哈希、Ogre注册目录、Scene父链和原始消息。显式 `--lidar-ray-convention gz-rendering8-cubemap-nearest` 严格版本校验后的 `sensor_geometry_actual_rays.json` **PASS**：深度899个clean样本p95=0.02653785m；雷达175个clean样本p95=0.00059017m。8cm/5cm门限未改变，缺少版本证据的旧样本不能按此模式宣称通过。

默认nominal模式保持原有名义角度语义。**在线mapper仍按LaserScan名义角度投影，其距离相关量化误差是否被地图安全裕量覆盖仍需单独验证**；不能把这个离线通过当作在线角度误差已修复，更不能把GPU模型用于实机驱动。原理与源码见 `research_sources/gz_lidar_ray_convention.md`、`SENSOR_DIAGNOSIS.md`。

## 真实场景 SITL：尚未通过

前两次 `sitl_takeoff_scan{,_v2}/` 均在IDLE，未解锁，不能称起飞/初扫通过。确认并修复了两项 harness 问题：

1. PX4 心跳按仿真时间1Hz，慢速锁步时实际约18.5墙钟秒一次。只读 `MAV_CMD_REQUEST_MESSAGE(HEARTBEAT)` 每墙钟秒请求真实PX4心跳；ACK与实际心跳分别计数，保留3s健康/5s看门狗、真实武装位锁存及正常PX4数据链保护。见 `gcs_diagnosis.md`。
2. CPU临时模型只在GZ_SIM_RESOURCE_PATH优先，SDF_PATH仍优先原资源，嵌套相机实际恢复了原始负载。现在两个路径均让临时overlay优先，并在报告中核对实际尺寸/点数/独立曝光频率；不匹配不能标为有效CPU配置。这项相机问题不改变独立原始rig的分辨率；旧性能六组另因碰撞overlay错误作废，以上新绑定六组才是有效比较。见 `cpu_profile_resolution_diagnosis.md`。

后续实际运行证据：

- `sitl_takeoff_scan_v3/`：CPU实际尺寸与曝光率均匹配；只读心跳请求130次、真实ACK129次，GCS链路健康。服务器仍软件渲染，RTF约0.102，PX4每0.5仿真秒的VehicleStatus在墙钟约4.9s才更新一次，超出P0的1s接收门限，不能累计持续就绪条件，保持IDLE后超时。
- `sitl_takeoff_scan_gpu/`：仅用进程环境 `GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA`，实际Ogre确认为RTX5060；同CPU10Hz相机/15Hz雷达下RTF约0.414，状态仍断续新鲜，保持IDLE超时。不是关闭P0检查来起飞。
- `sitl_takeoff_scan_gpu_5hz/`：使用现有CLI明确降载至相机/雷达5Hz，尺寸/曝光率匹配；实际Ogre日志由本次服务器的写文件描述符和进程组确认归属，GPU身份与日志SHA保存，不能靠环境变量宣称加速。RTF约1.02，状态持续新鲜，真实经历 **IDLE→START→SAFE_TAKEOFF→FAULT** 并解锁。停止包络下界-0.359272m，确认柱体下界-0.358242m，越界约1.03mm而拒绝继续，原因 `MOTION_STOP_CERTIFICATE_INVALID: UNKNOWN_MOTION_ENVELOPE`。没有放宽下界、写假空闲或调控制参数掩盖此失败。

这不是初扫通过。当前起飞接触带由home与clearance参数构造，与实测导航偏移/动态协方差构成的完整停止包络仍不相容，需在有真实地面支持证据和响应标定后处理；不能自动扩大未知授权域。ROI_ACCESS实际移动、任务返航、真实landed+disarmed仍缺合格证据，属于保持Issue开启的原因。

## 真实 Gazebo 截图

`screenshots/` 的原生PNG已逐张查看，保留完整顶棚，没有合成或修改图片；`screenshots.json`记录实际相机位置、原文件和SHA。`01_enclosed_exterior.png`、`02_plant_visual.png`、`03_takeoff_area.png`、`04_wall_clearance.png`、`06_collision_overlay_wide.png` 为1501×1032最终画面。早期窄窗口版本另存，避免旧哈希指向覆盖后的文件。

Issue #3 当前保持开启；最终进度已发布至 https://github.com/pawn-queen/px4_ros_ws/issues/3#issuecomment-6458798754 ，资源对照更正记录为 https://github.com/pawn-queen/px4_ros_ws/issues/3#issuecomment-6458536198 。只有传感器、代表性动态接触、起飞域、局部ROI集成及性能均有合格实际证据后才能关闭。

## 地面接触保护续改：软件通过，实际主链未通过

本节证据根目录：`log/issue3_takeoff/20261028_143128/`。代码基线为本地主仓库 `e929b82d` 加未提交修改，具体源文件、实际可执行文件及哈希保存在各运行 `manifest.json`。模型资产仍为 `69f6511d0b11e2189b204ce6b10f12cbef24f119`；原 world、建筑、碰撞与 ROI 未修改，外部仓库仅保留原有 `x500_depth/model.sdf` 未提交修改。本轮没有提交或推送。

### 已实现并通过软件检查

- 起飞内部阶段为 `GROUND_CONTACT / LIFTOFF_TRANSITION / AIRBORNE_STRICT`。支持模型来自实际 floor 与完整机体审计；32个起落架角点、参考点接地偏置+0.013m、地面0±0.01m，补齐原证明0–0.04m间隙。`audit_actual.json` 为静态通过证据，不等于实际飞行通过。
- 原1.03mm停止包络案例重放：严格证书仍拒绝，合格地面证书可处理，原始盒和协方差未缩小；占据、侧面和顶部仍否决。地面授权绑定 session、定位epoch、支持模型和独立授权版本；过期、错误平面、姿态不确定性、迟到异步结果均有回归。
- 严格交接在执行器真实停稳/HOLD证据、离地样本与完整严格盒合格后回执revision+1，保留参考历史；任务收到回执前不能初扫。地面过渡继续使用慢时间律和限高、限时；已验证的保守协方差预算减少微上升反复制动，实时状态和停止检查保持原值。
- 修复解锁回调与odometry到达顺序竞争、落地采样时间/异常未来时间戳、DDS只调整timestamp而不调整armed_time的时钟差异。解锁期限采用实际事件或更早的DISARMED采样界，不从迟到接收时刻续期。
- 修复测试预算混用：整趟飞行仍为300s，独立 `explore_task_duration_s=60` 从首次EXPLORE开始；共享默认0。严格交接前需要结束则降落，不携地面证书返航。

两包构建通过，最新C++增量再次通过，见 `build_final.log`、`build_clock_final.log`。最终 **473项直接用例，零失败、错误或跳过**，见 `software_test_summary.json` 和复制的 `test_results/`：C++原生200、parity26、跨进程ROS7、资产32、GCS20、传感器23、探针3、支持模型12、验收器23、共享Python127。完整CTest23组通过后，接触分类新增用例单独复测通过；不重复计入包装或重复执行。失败构建、红测试和旧仿真日志均保留。

### 实际电厂运行与新阻断

统一使用真实 `powerplant.sdf`、实际NVIDIA、RGB640×360/深度320×240/雷达360点、5Hz传感器，50Hz控制。启动轨迹仍为2cm、至少4.5s，保持原P/V/A、跟踪、导航、飞控失联与反应预算门限。建图器原2m启动门保持；起飞未通过时融合计数为0符合设计，不是传感器话题缺失。

| 运行 | 范围及结果 |
|---|---|
| `sitl_scan_v1` | 原地面授权可用，但测试将探索60s误用整趟预算，起飞中提前返航；该预算错误已修复 |
| `sitl_scan_v2` | 原PX4控制值；微上升连续完成，正常上升实际响应不足，触发跟踪保护 |
| `sitl_scan_v3_hover` | 单独将临时SITL悬停初值设为0.74633，仍未通过；不是正式原配置验收 |
| `sitl_scan_v4_hover_i0`、`sitl_scan_v5_slow_transition` | 临时诊断再将垂向积分置零，真实上升约0.3–0.4m，但水平/合成跟踪误差仍越界；均未取得严格证书，不作正常控制建议 |
| `sitl_scan_v6_stock_recovery`、`sitl_scan_final_stock` | 恢复原PX4控制值，故障后继续采集实际降落与上锁证据；主链仍失败 |
| `stock_contact_recovery` | 最终代码与原PX4控制值；`IDLE→START→SAFE_TAKEOFF→FAULT→LANDING→COMPLETE`。真实落地上锁和接触检查通过，主链明确FAIL |

最终 `stock_contact_recovery/report.json` 的 `passed=false`、`exit_reason=mission_fault_recovered_land`，`no_mission_fault=false`。`failure_recovery` 的降落服务接受及landed/disarmed均true，`main_mission_pass_restored=false`。COMPLETE只表示失败后的落地结束，不能替代自主任务完成。

当前阻断为 `MOTION_TRACKING_ERROR_EXCEEDS_CERTIFICATE`，不是原1.03mm未知空间问题，也没有证据表明是本次发布性能阻断。对照 `startup_response_diagnosis.json`、ULog及 `tracking.jsonl`：原控制值的垂直响应不足，临时参数诊断改善离地后仍有合成位置误差越界。理想静态悬停量约0.74633只由当前2.1453kg模型、电机常数和输出映射推算，不是实际响应验收，更不能用于硬件。需要继续标定控制响应及导航噪声对应的可兑现跟踪模型；本轮没有提高跟踪门限、扩大体素授权、删除占据或注入航点取得通过。

### 最终有限阶段的独立证据

`stock_contact_recovery/`：实际Ogre确认 `D3D12 (NVIDIA GeForce RTX 5060 Laptop GPU)`，原始传感器尺寸和5Hz曝光率匹配；81份代码/配置/二进制、101份资源的运行前后哈希一致。50Hz发布实测49.996Hz，p95/p99间隔27.3/60.2ms、最大97.9ms，低于原0.3s反应预算；已执行段最大垂向速度0.103m/s、加速度0.129m/s²。它们仅覆盖已运行的有限起飞与失败恢复阶段。

临时4模型副本覆盖10个实际碰撞体的接触监测，展开后物理模型摘要保持一致。最终记录8640条合格支持接触，真实world步进、起落架接地阳性对照、全部话题与订阅通过；无建筑/围墙/旋翼/机身异常接触，无未分类或溢出。另2条是严格交接前FAULT瞬间仍在地面的起落架支持，单列 `prestrict_failure_support_contact_count`：不授予运动权限，也不能充当阳性对照。旧严格阶段或机身/墙面接触仍拒绝，新增回归覆盖此区别。

**尚未通过：实际严格证书交接、HOVER_SCAN、ROI_ACCESS真实接入、自主视点停稳新增观测、正常RETURN_HOME及整条无故障主链。** YOLO巡检未执行；此前满传感器负载的静态性能结果不等于本轮飞行通过。只有这些约定的主链条件取得实际证据后才评估关闭Issue #3。

本轮进度已通过gh发布至 https://github.com/pawn-queen/px4_ros_ws/issues/3#issuecomment-6469365548 ，Issue保持OPEN；`delivery_manifest.json`记录最终本地文件哈希与未完成边界。

## 2026-10-28：基础跟踪容差0.10m单变量对照

按用户要求，仅将仿真覆盖配置 `p2_simulation.yaml` 中 `motion_tracking_position_tolerance_m` 从0.03改为0.10m，共享/硬件参数、PX4原控制值、其他安全门限、真实world/碰撞、默认ROI和自主规划不变。两次均从完整任务入口运行（`--stop-after mission`，无YOLO要求），证据根目录 `log/issue3_tolerance010/20261028_212122/`。

Python包构建及4组相关回归通过，93个直接用例零失败/错误/跳过，不宣称重新运行全部473项。两次运行均确认基础容差实际为0.10，均为 `IDLE→START→SAFE_TAKEOFF→FAULT→LANDING→COMPLETE`；故障后新鲜PX4落地上锁确认通过，主链仍失败；因实际未离地，该收尾不算从空中下降的降落验收。

直接阻断仍为 `MOTION_TRACKING_ERROR_EXCEEDS_CERTIFICATE`：run2有明确配对样本，三维位置误差0.170346m大于当时 `0.10+2×0.035=0.170m` 的门，超0.346mm；各轴与速度分别通过。run1最后可配对误差0.164075m小于0.168m，随后由执行器报告FAULT，不能将前一条通过样本冒充越界帧。首FAULT时过渡分别还剩4.201/4.704s，未触发12s超时或0.75m限高。

关键事实：从任务初始化至FAULT，两次Gazebo模型原点分别506/481条真值均完全不变，仍在接地参考z=-0.0130009016m；未真正离地。run1参考已向上推进到ENU约0.1766m。按真实SDF、world重力及ULog故障前电机命令估算，两次理想推力约12.52/12.32N，小于约21.04N机体重量。这是命令角速度下的模型估算，并非实测力，但与持续接地一起支持控制输出/真实响应不足；尚不能唯一归因到某个PID。更大的容差没有消除参考推进与实际不动之间的差距。

两次实际NVIDIA/5Hz传感器、约50Hz发布、原反应预算、P/V/A和物理接触检查通过。未取得严格空中证书，稳定悬停、初扫、ROI自主接入、任务执行及正常返航均未进入；没有以其他参数或状态机修改强行进入后段。详细原理、可复现命令、ULog/真值重建见本证据目录 `VALIDATION.md`、`comparison.json`、`analyze_runs.py`、各运行 `report.json`。Issue #3继续开启。
