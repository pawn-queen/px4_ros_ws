# Issue #3：电厂碰撞与局部任务 ROI

本次保留 ROS 2/PX4、共享 Python 定位/YOLO、C++ 建图规划执行框架和原有顶层状态字符串。默认不自动起飞。真实场景仍是外部 PX4 仓库的 `Tools/simulation/gz/worlds/powerplant.sdf`，包含四壁与不透明顶棚。

## 碰撞资产

原视觉 STL 保持字节不变；原20×20×10 m碰撞盒被同坐标、同单位、比例1的静态凹三角网格替换。模型仓库 `models/power-plant/scripts/build_collision.py` 提供确定性重建，`verify_collision.py --dense` 独立检查最终序列化资产。

生成器删除精确同向重复面、零面积面和清理后无三角引用的孤立点；不容差焊接、不填孔、不生成全局凸包。按连通部件保拓扑简化，冻结边界；薄件、小件、非流形件保留，验证失败回退原面。各候选检查连通、Euler、边界和双向表面距离。薄件逐坐标保留，不靠2 cm误差门限压扁细部。

最终501262面，较605740面原STL减少17.25%，文件30,287,084→25,063,184字节。减面少于早期全局试算，是保护复杂区域后的实际结果。所有顶点和三角形质心双向复验：原→碰撞 p99约6.00 mm、最大19.08 mm；碰撞→原 p99约4.65 mm、最大15.24 mm。这些是有限验证集结果，不是连续Hausdorff证明。

资产提交：`69f6511d0b11e2189b204ce6b10f12cbef24f119`，仅包含电厂模型、生成/验证资源及已授权围护，不包含已有x500_depth改动，未推送。主仓库 `config/powerplant_assets.json` 固定提交及递归资产哈希；准备实际场景时验证资源、模型配置和mesh，不一致则拒绝使用旧验收。传感器机体的既有独立包络审核仍保留。

## 两种范围，三个用途

导航地图保存完整、已验证观测。完整快照计算 observed_bounds（含 uncertain），导航视图并入已确认起飞柱体后加一层未知边界。边界内没有观测的间隙仍未知，真实占据优先。读取 planning_map 不再改变接收器。运动、制动、初扫和返航使用此导航视图。

`explore_min/max_*` 现在定义任务 ROI，默认 x=[-24,34]、y=[-9,49]、z=[3.5,7] m，不是全阶段硬飞行围栏。ROI只限制任务收益、目标表面及完成范围；外部已知安全航路和观察位可用。硬件/仿真共用这套逻辑，硬件的运动参数与起飞域仍须实测配置。

二维投影固定x/y界作为基本显示范围，并合并当前位姿周围至少1 m；启用高度带时直接使用当前z±band，不与旧绝对高度带相交。原有真实融合、时间、姿态稳定、空闲锚点和安全首段门控继续保留。

## EXPLORE 的子模式

- `ROI_WORK`：按实际相机视场计算ROI内未知增益；区域排序和候选名额也先做ROI过滤，避免外部高收益前沿挤掉任务区域。
- `ROI_ACCESS`：同一不可变快照下没有可用ROI工作视点时，选择前沿已知空闲侧的接入观察点。候选/观测搜索范围为home与ROI联合AABB向外扩10 m；安全绕障航路和返航仍用完整导航证据域。
- 接入排序采用实际去程耗时、到ROI的乐观剩余耗时及重访代价；同分按新增可见量和稳定体素顺序选择。保留视场、定位、制动及完整去返预算检查，每次只执行现有局部路径。
- 默认 `roi_access_max_duration_s=60`、`roi_access_margin_m=10`、`roi_access_revisit_penalty_s=5`。接入时间用steady clock累计，模式切换不清零。无候选、超时或余量不足明确报告并转返航；不能据此报告ROI完成。

ROI未知量按稀疏已知格与ROI总格数计算，不枚举整个ROI。局部未观察、不可达与超预算结果和全场完成分开。该模型不保证任意远ROI一定可达；限定范围外的入口不会触发无界全厂探索。

## 巡检、版本与参数

目标注册后按真实三维表面与ROI交集判断相关性，ROI外目标不进入本任务会话。面片按局部法向平面与ROI裁剪面积，外部面片不进入覆盖分母；重复照片、曝光绑定、质量与保存成功条件保持。任务ROI与照片图像ROI是不同概念。

新增 TaskRoi/TaskRoiMode、导航授权revision和任务ROI revision；状态及照片事件追加局部scope、子模式、接入时间和未知量。现有话题和顶层状态名称保留。只有IDLE接受ROI及接入策略参数的原子修改，非法/活动任务修改明确拒绝；其他安全参数问题仍由Issue #9单独跟踪。

导航授权版本覆盖epoch、分辨率和柱体配置，普通远处观测变化继续用P1依赖复核，不反复全局取消。旧任务ROI/授权结果不能采用，路线与停止包络使用最新证据再次验证。

## 验证入口与边界

```bash
# 先source ROS、workspace和tools/env/ros_env.sh
python3 validation/sitl/run_acceptance.py \
  --scenario powerplant --sensor-profile cpu --detection-mode none \
  --stop-after scan --timeout 240
# ROI外起点的接入进展：额外 --roi-access-test --stop-after roi-access
# 局部任务全流程：--stop-after mission
```

CPU profile、原始传感器负载、有限阶段及完整任务分别报告。有限阶段必须观察真实阶段进展，再经land服务与landed/disarmed完成；不能称为完整巡检或全飞行通过。真实电厂不复用小房间的已知圆柱检测重放，未提供真实标注fixture时拒绝此模式。几何真值和动态探针只进入评估器，不作为在线地图、目标或航点。

`validation/model/validate_physics.py` 在同一个实际world中创建临时动态探针，保留接触、位姿、阳性对照及自由通行记录；完整对照只做退化/重复清理，不做减面。性能组固定原始传感器、种子和负载，每组30 s预热+60 s采样。无接触消息本身不能证明空闲，渲染深度/GPU雷达正常也不能证明物理mesh正确。

实际结果与尚未完成项目见 ISSUE3_VALIDATION.md；软件测试和几何检查不代替物理接触、传感器、性能和SITL证据。

## 本轮验证新增的关键原理

动态接触使用有限半径球体，不能把中心射线交点当成球体首次碰撞点。补充fixture现在对原始/简化完整表面分别做基于距离1-Lipschitz下界的球体扫掠，保持原起点、速度、半径及2.5cm判据；旧误判被独立几何解释后仍重新运行实际物理测试。`probe_sweep_diagnosis.json`保留前后预期与实际接触，不后验删点。

Gazebo的GPU雷达实际采用立方体纹理最近邻采样。离线验证将默认名义角度与显式、版本绑定的实际纹理方向分开，门限保持，详见[雷达方向原理](research_sources/gz_lidar_ray_convention.md)。这项离线校验不静默改变在线mapper或实机雷达。

当前主机使用硬件渲染的实测进程环境为：

```bash
GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA \
python3 validation/sitl/run_acceptance.py \
  --scenario powerplant --sensor-profile cpu \
  --cpu-camera-rate-hz 5 --cpu-lidar-rate-hz 5 \
  --render-context display --detection-mode none --stop-after scan
```

这是明确的降载调试配置，不能代替原始传感器负载性能或完整飞行验收。runner按本次Ogre日志的实际写入进程组/文件描述符确认归属，记录真实GL_RENDERER及哈希；环境变量本身不能证明硬件加速。当前实测已进入SAFE_TAKEOFF并解锁，但停止包络越过确认柱体下界约1.03mm而FAULT，未放宽健康、占据或停止包络门控。见[实际记录](ISSUE3_VALIDATION.md)。

## 2026-10-28：地面接触保护续改

新增 `GROUND_CONTACT → LIFTOFF_TRANSITION → AIRBORNE_STRICT` 内部阶段，顶层状态字符串保持。地面平面、误差与完整起落架点来自独立静态审计，不再由探索膨胀半径推导。支持证书只处理向下接触解释，原始协方差、停止包络和占据否决不变；只有真实停稳、离地与完整严格盒证据合格，50Hz执行器才回执新授权版本，任务层锁存交接。硬件默认关闭专用模式。

补齐采样时间、解锁事件时限、会话/epoch/模型版本、异步结果失效、慢速过渡与失败降落证据。修复测试探索60秒误用整趟飞行预算的问题：共享 `explore_max_duration_s=300` 保留，新增 `explore_task_duration_s` 从首次 EXPLORE 开始；默认0，测试显式60。严格交接前需要结束任务时直接降落，不携带地面证书返航。

原理、参数及坐标关系见 [TAKEOFF_GROUND_SUPPORT.md](TAKEOFF_GROUND_SUPPORT.md)。静态与软件通过不能替代实际飞行；当前实飞仍触发原跟踪保护，后续阶段没有通过，详见 [ISSUE3_VALIDATION.md](ISSUE3_VALIDATION.md)。

## 2026-10-29：起飞响应同步

本轮在上述地面支持证书上增加启动意图/物理响应分离、采样时间预测风险及BRAKING→HOLD恢复锁存、一次性完整过渡预算和独立SITL控制标定。首次上升采用已发布HOLD偏航，保持地面禁止转向；物理响应要求实际起落架净空下界、新导航样本及新鲜离地状态，预测交接不能代替实时严格ACK。完整设计、参数、默认launch与审计后临时profile的区别见 [TAKEOFF_RESPONSE_SYNC.md](TAKEOFF_RESPONSE_SYNC.md)。

506项直接软件回归通过；实际电厂全部五个允许积分候选失败，未冻结配置，主链仍未通过。本轮不修改PX4源码、硬件缺省、场景、碰撞或ROI，不提交/推送。失败日志及下一步边界见 [ISSUE3_VALIDATION.md](ISSUE3_VALIDATION.md)。

## 2026-10-29续改：SITL过渡白名单与严格交接证据

A—D过渡策略与严格策略分别作为不可变limits输入，绑定policy ID/revision、session、epoch与支持模型，纳入worker/缓存/发布复核。严格ACK重新证明完整目标停止形状与风险，原子恢复严格策略，不重置参考。停止查询用圆角扫掠体筛除确实不相交的方盒角部，未知/占据接触继续拒绝；保留外框与实际形状的独立输出。

参考管理仅在过渡停稳HOLD选择1.0/1.25时间倍率；无候选进入有限恢复。固定预算计入5秒等待和最坏1.25倍率为44.125秒，向上取整45秒，任务中不续期。审计后的D允许0.48m，严格阶段恢复0.50m；地图余量/协方差倍率未降低。

ACK新增不可变状态/参考/包络/策略快照及源采样时间；验收独立匹配导航、发布与接触边界。修正float32/NumPy布尔类型适配、微小偏航/横移数值差产生禁止运动的规划缺陷。依赖显式声明python3-numpy。参数仅IDLE修改，原顶层状态/话题保留，PX4与硬件缺省不改。

原理与论文借鉴见[TAKEOFF_RESPONSE_SYNC.md](TAKEOFF_RESPONSE_SYNC.md)。524项软件回归通过；实际20候选均未过初扫，5次严格ACK子项通过不代表整体任务通过。完整证据及剩余控制/导航/时钟问题见[ISSUE3_VALIDATION.md](ISSUE3_VALIDATION.md)。
