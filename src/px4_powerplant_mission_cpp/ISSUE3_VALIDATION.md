# Issue #3：实际电厂验证记录

验证日期：2026-10-28。证据根目录为 `log/issue3_powerplant/20261027_204849/`（下文路径相对此目录）。仅实际 `powerplant.sdf` 为正式场景；真值与探针只进入评估，不向在线规划注入地图或航点。**目前不能关闭 Issue #3：实际局部任务飞行尚未通过；传感器已补充实际GPU采样方向的合格独立证据。**

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
