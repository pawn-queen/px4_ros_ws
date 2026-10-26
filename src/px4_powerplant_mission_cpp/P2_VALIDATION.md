# P2 可重放检查与 PX4/Gazebo 验收

本文件给出 P2 的验证入口与证据边界。运行结果以对应目录中的 `report.json` 为准；准备脚本或通过单元测试不等于完成 SITL，更不等于实机验收。编译、测试和 Gazebo 始终串行执行。

本轮按用户最后确认的范围完成软件逻辑、测试和已有日志核查；飞控/动力学参数由用户调整。下文保留实际飞行验收入口和旧运行证据，供参数标定后继续验证，不以调参通过飞行作为本轮软件收尾条件。

## 1. 构建后的基础回归

保留原有 P0/P1 基线入口；CMake 同时纳入 P2 的运动、拓扑、目标表面与图像质量用例：

```bash
cd /home/pawn/px4_ros_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source src/px4_powerplant_mission_cpp/scripts/p0_ros_env.sh
bash src/px4_powerplant_mission_cpp/scripts/run_p0_baseline.sh
```

拓扑用例检查房间核心与门户、三维上下连接、未知/占据门洞、狭窄空间退化、区域 ID 与 epoch、区域候选/前沿名额、滞回、真实时序回调及安装旋转后的视场。它们检查程序行为，不声称输出了真实建筑语义房间。

## 2. 先准备，不启动仿真

```bash
/usr/bin/python3 src/px4_powerplant_mission_cpp/scripts/run_p2_sitl.py \
  --scenario room --detection-mode replay --prepare-only \
  --output-dir log/p2_sitl_prepared
```

这一步解析共享配置与测试世界，检查可执行文件/ROS Python 依赖，写出 `manifest.json` 和独立运行目录。`report.json` 的 `prepared_only=true` 且 `passed=false`，明确表示尚未执行验收。

依赖为本机已构建 PX4、Gazebo `gz`、Micro XRCE-DDS Agent、ROS 2 Jazzy、Gazebo Python `gz.transport13` / `gz.msgs10`、`pymavlink`、已安装的两个任务包和当前 shell 的 Cyclone DDS 配置。PX4 默认目录为 `/home/pawn/drone-simulation`，可用 `--px4-root` 指定其它已构建 checkout。脚本不会构建 PX4、安装依赖或修改其源码。

## 3. 启动隔离的真实 SITL 链路

```bash
/usr/bin/python3 src/px4_powerplant_mission_cpp/scripts/run_p2_sitl.py \
  --scenario room --detection-mode replay --timeout 300 \
  --output-dir log/p2_sitl_room
```

入口实际启动 Agent、Gazebo、PX4、真实 Gazebo 传感器桥和 ROS 任务 launch。检测到已有 PX4 进程时拒绝运行；只清理自己创建的进程组，不接管或杀死其它任务。

- 使用独立 `ROS_DOMAIN_ID=25`、随机 Gazebo partition 和 Agent UDP 8889；端口/域可显式覆盖。域 25 避开 Linux 常见临时端口范围，降低同机 UDP 端口冲突。
- 独立 GCS 辅助进程排他绑定回环 UDP 14550，收到来自当前 PX4 18570 端口的真实 autopilot 心跳后才向同一对端发送 1 Hz GCS 心跳。它只额外读取 `NAV_DLL_ACT`、`COM_DL_LOSS_T`、`COM_SPOOLUP_TIME` 和 `MPC_TKO_RAMP_T`，不发解锁、模式或轨迹命令；保留当前机型 `NAV_DLL_ACT=2` 的数据链路门控并写入实际参数证据。端口被其它 GCS 占用时拒绝运行，不接管既有 GCS。心跳证据过期或实际策略不符时完整验收失败。
- 每次在 `log/.../runtime/run-.../px4_rootfs` 创建新参数与日志目录，`etc` 只读指向现有 PX4 构建资源。没有复用外部 checkout 的 `parameters.bson`。
- 当前进程通过 `PX4_PARAM_*` 配置 SITL 的独立 external vision、关闭 GPS/磁罗盘依赖，以及 `COM_OBL_RC_ACT=4` 的 Offboard-loss 降落。该数字依据当前 checkout 的参数定义；它不是硬件部署建议。
- 此入口显式设置 `UXRCE_DDS_SYNCT=0`，定位/EV 采用 `px4_timestamp_domain=ros`，使锁步 PX4 与 Gazebo 共用仿真采样时间。硬件仍使用默认系统时钟同步，不能照搬此配置。launch 按已安装 `px4_msgs.MESSAGE_VERSION` 为默认 PX4 输入话题添加版本后缀；自定义话题可继续由参数提供。
- 测试世界保持 `world name=powerplant`，与现有 `/clock`、真值桥和 UWB 仿真器一致；启动的机型仍为 `x500_depth_0`。
- 测试专用配置确认半径 1.5 m、高 4.2 m 的空柱体，起飞/返航高度为相对 home 的 3 m。它只适用于本文件附带空场景，不修改共享默认的 `takeoff_region_confirmed=false` 和 `auto_start=false`。
- `p2_simulation.yaml` 提供从 Gazebo 模型解析的导航参考—相机/雷达偏置及包络；实机 `p2_hardware.yaml` 仍需真实标定。

附带 `p2_room.sdf` 是有地面、顶棚、四面墙及对比纹理目标的闭合室内；`p2_two_rooms.sdf` 增加已知几何可通的门洞。世界里的目标真值只供测试检测器生成检测框，不送给探索器作为已知航点或目标坐标。

仿真入口在请求 START 前，要求定位 ready 和 vehicle status fresh 在同一 localization epoch 连续保持至少 3 秒仿真时间且 4 秒墙钟时间；任一条件失效或 epoch 改变均重新累计。这只是测试初始化稳定等待，不放宽正式节点的健康、时钟或起飞门控。请求时的双时钟等待证据记录在 events.jsonl。

START 前还必须取得真实 GCS 参数读回，并验证启动微上升高度为正且不超过跟踪位置容差，最小运动时长覆盖实际 spool+ramp 时间和反应预算。当前仿真为 2 cm/4.5 秒；这是整段一致的五次 P/V/A 微上升，按采样时钟推进，不是到点后盲等墙钟。`takeoff_startup_validation` 保留配置核对结论，明确 `actual_takeoff_ramp_state_observed=false`；最终仍依靠真实跟踪、停止和飞行证据。缺参数、布尔伪数字、非有限值、过短时间或过大高度均拒绝；对应七种纯函数验证保存为 `log/p2_startup_config_checks.json`。

### 原始与 CPU 传感器负载必须分别报告

`--sensor-profile native` 为默认值，直接使用外部 checkout 原始 OakD-Lite 模型，通常为 RGB 1920×1080、深度 640×480、各 30 Hz；最终以本次 `manifest.json` 中读取的实际 SDF 配置为准。

无 GPU 的本机可以显式选择独立的 CPU 验收负载：

```bash
/usr/bin/python3 src/px4_powerplant_mission_cpp/scripts/run_p2_sitl.py \
  --scenario room --detection-mode replay --sensor-profile cpu \
  --timeout 300 --output-dir log/p2_sitl_cpu
```

`cpu` 将 OakD-Lite 与 x500_depth 完整目录复制到本次 `runtime/models/`，将 RGB 调为 640×360、深度调为 320×240，相机频率默认 10 Hz、GPU 雷达默认 15 Hz。本地模型目录放在 Gazebo resource 搜索首位，PX4 spawner 也显式读取这一本地 x500_depth；外部模型根目录仍为 x500/x500_base 等依赖提供回退。可用 `--cpu-camera-rate-hz` 与 `--cpu-lidar-rate-hz` 分别指定有限正频率，例如 `--cpu-camera-rate-hz 2.5 --cpu-lidar-rate-hz 2.5`；在 native 模式给出非默认覆盖值会拒绝运行。没有修改外部 checkout，也不改变相机 FOV、雷达角度/样本/量程、安装位姿、惯性、碰撞、飞行动力学或物理模型。ROS CameraInfo 使用渲染器对应新分辨率产生的真实标定。

Manifest 记录两套原始/有效 SDF 路径与 SHA256、相机配置尺寸/频率/FOV/安装信息、雷达角度/样本/量程/安装信息，以及实际收到的 RGB/深度尺寸、LaserScan 参数、曝光时间频率和墙钟接收频率。报告明确写 `performance_scope=cpu_reduced_sensor_load`；CPU 负载下的闭环或延迟结果不等于原始分辨率/频率负载的性能验收，原始负载失败也不会被这一模式覆盖。

### 检测重放与真实 YOLO 分开

`--detection-mode replay` 使用真实 Gazebo RGB、RGB CameraInfo 和采样时刻插值的导航位姿，将测试目标的已知几何投影为检测框，再原样转发这一张 RGB 的像素和采样时间。它不绘制假目标、不生成假深度、不写入假空闲地图。该模式验证目标输入后的 RGB-D 注册、目标关联、表面建模、拍照、质量、覆盖和返航链路；**不验证 YOLO 的分类与检测能力**。

```bash
/usr/bin/python3 src/px4_powerplant_mission_cpp/scripts/run_p2_sitl.py \
  --scenario room --detection-mode yolo --timeout 300 \
  --output-dir log/p2_sitl_yolo
```

`yolo` 模式完全使用现有模型与原始渲染图像，不注入检测结果。模型无法识别测试物体、推理过慢或没有同步合格图像时，这一模式会如实失败，不退回合成检测。`none` 模式只验收无目标的探索和返航，不检查巡检覆盖。

### 单独验收真实 YOLO 的图像链路

真实 YOLO 是否能识别附带合成目标，与推理结果是否正确保留曝光和原始像素分别判断。完整 `--detection-mode yolo` 飞行仍保留原来的巡检/照片/覆盖要求；没有识别目标会使完整任务验收失败，同时报告独立的 `yolo_image_chain_checks`。

```bash
/usr/bin/python3 src/px4_powerplant_mission_cpp/scripts/run_p2_sitl.py \
  --scenario room --detection-mode yolo --sensor-profile cpu \
  --image-chain-only --timeout 90 --output-dir log/p2_yolo_image_chain
```

`--image-chain-only` 仅接受真实 `yolo` 模式，不请求 START、不解锁、不执行飞行；取得至少三次真实推理的严格图像配对后结束。报告明确标记 `acceptance_scope=yolo_image_chain_only`、`flight_acceptance_executed=false`，不能将此结果称为起飞、轨迹、建图、目标检测准确率或完整巡检通过。即使模型输出零个目标，也可以通过图像传输链路这一独立检查。

验收同时订阅真实 Gazebo RGB、`yolo_source_image` 和检测 JSON。严格比较三者曝光时间、frame、宽高，并比较原始 RGB 与源图的 encoding、step、大小、端序及像素 SHA256；源图带标注、曝光/尺寸不匹配、JSON 格式错误或配对证据过期均不能通过。`yolo_image_chain.jsonl` 保存每次配对的元数据和哈希，报告记录真实推理结果数、目标框数、成功配对数、错配与待配对/过期数量。

只缓存原始 RGB 的哈希和小元数据，原图索引最多 512 项，源图和 JSON 元数据各最多 32 项、最长 15 秒，不缓存连续原始图像历史。这里的目标框数是模型输出的数量，没有把它当作经真值验证的检测准确率。

### 多房间与受控制动场景

```bash
/usr/bin/python3 src/px4_powerplant_mission_cpp/scripts/run_p2_sitl.py \
  --scenario two_rooms --detection-mode replay --timeout 360 \
  --output-dir log/p2_sitl_two_rooms

/usr/bin/python3 src/px4_powerplant_mission_cpp/scripts/run_p2_sitl.py \
  --scenario room --detection-mode replay --inject-obstacle --timeout 360 \
  --output-dir log/p2_sitl_braking
```

双房间验收额外要求实际地图产生至少两个区域和一个已知门户。制动场景在 EXPLORE 检测到实际水平运动后，将场景专用障碍移到运动前方 2 m，并要求观察到 `BRAKING`，随后 `motion_stopped=true` 且导航速度不超过 0.12 m/s。只有停止后才开始检测重放。移动模型发生在隔离模拟器中，未修改外部模型文件；这不是对任意突然侵入停止距离内的动态障碍的安全保证。

障碍服务异步轮询，不阻塞监视进程接收 PVA 和导航。`braking.json` 记录服务请求/应答的双时钟窗口、首次制动/停稳报告、实际导航路径长度与起止位移、速度，以及注入前和首个制动盒对采样机体包络的包含情况。检查的是相对导航参考的完整机体加位置不确定度，不能只看质心。当前尚无 Gazebo 障碍生效时间、障碍融合/制动触发 ID 和执行器精确输入 ID，采样间连续运动也不能由离散日志证明，因此物理制动结果明确为 `UNCONFIRMED`；附加场景即使观察到制动停稳，完整验收仍保持未通过。后续补齐归因和测量后才能改变这一结论，不能把服务请求时间当成障碍生效时间。

## 4. 验收判据与输出

`report.json` 对每项判据给出布尔值，任何失败均以非零退出码结束：

- 服务确实接受 START，然后顺序经过安全起飞、初扫、探索、目标巡检、返航、降落和 PX4 确认的 COMPLETE。START 服务接受单列证据，因为状态可能在下个周期立刻变为 SAFE_TAKEOFF。
- 实际接收不少于 250 条位置/速度/加速度/yaw/yawspeed 均有限的 `TrajectorySetpoint`，总发布速率至少 40 Hz、p95/p99 接收间隔均不超过 0.1 s；目标值为 50 Hz，报表保留实测值。
- 单独统计已解锁、实际 PX4 Offboard 且任务仍在 SAFE_TAKEOFF—RETURN_HOME 的执行区间。按导航采样时间在两条真实 PVA 参考之间插值，逐样本记录位置/速度误差、该条导航的完整协方差及证书余量。位置协方差已经属于 ENU，速度协方差由完整姿态作 `RΣ_FLU Rᵀ` 转入 ENU，原始 FLU 矩阵、ENU 矩阵和姿态同时保留。除原有范数门限使用 `sqrt(λ_max(Σ))` 外，每个 ENU 轴还必须满足 `abs(e_i) ≤ tolerance + k sqrt(Σ_ii)`；标准差与执行器一样按 `0.001 m / 0.001 m/s` 向上取整，不改写原始矩阵。当前仿真位置/速度容差分别为 0.03 m 和 0.25 m/s，`k=2`，不将其宣称为三维 95% 覆盖保证。至少需 100 条完整证书；协方差非有限、非对称、非半正定、对角为零或坐标系不匹配均不能通过，任何范数/单轴超限或不能配对的有效执行样本均失败。NAV_LAND 后没有 PVA 参考的导航样本单列排除数量，不拿降落运动冒充 Offboard 跟踪。
- 执行区间的发布、导航与 `/clock` 分别记录墙钟/采样间隔的 max、p95、p99，以及导航相对 ROS 时钟的采样年龄、时钟接收年龄和 ROS/wall 间隔差。最大间隔/年龄不得超过配置的 0.3 s 反应预算，导航/发布采样时钟倒退也失败；仅有百分位通过不能掩盖长尾失鲜。
- P2 缺省命令限值：水平/垂直速度不超过 0.61/0.31 m/s，水平/垂直加速度不超过 0.51/0.31 m/s²，包含 float32 量化容差。
- 全程没有任务 FAULT；拓扑已实际建立；附加场景检查门户或受控制动证据。
- 外参验收不预先认定导航参考正确。脚本直接订阅当前隔离 Gazebo partition 的原始 `Pose_V`，从 `dynamic_pose/info` 中只接受精确名称 `x500_depth_0`，保留 `name`、实体 ID 和外层采样时间。Jazzy 的 Pose_V—TF 转换可能丢失名称和时间，因此校验不使用 TF 索引，也不将 `base_link` 当成模型原点。将真值与导航分别插值到真实深度曝光时间；至少十组样本、模型原点误差 p95 不超过 0.20 m，且平均误差必须比误用 `base_link` 的 `+0.24 m` 偏移至少小 0.05 m。不存在具名真值或插值区间时没有通过依据。
- 使用当前 `p2_simulation.yaml` 完整深度外参、实际深度 CameraInfo 和真实深度像素，对测试世界的静态墙/地面/顶棚作稀疏射线校验；检测目标和纹理块参与遮挡判断，不将目标点硬算成墙面。必须累计至少 200 个墙面样本，真值位姿投影残差 p95 ≤ 0.08 m、导航位姿投影残差 p95 ≤ 0.25 m。采样按真值射线确定，不按残差挑选有利像素。这些门限是附带静态世界的校验初值，真实偏差超限会失败，不能据此自动修改外参。
- 巡检模式必须有合格照片写盘和正的已覆盖面积。结果明确为 `PARTIAL` 或 `REACHABLE_MODELED_COMPLETE`，覆盖范围字段必须是 `reachable_modeled_surface`。`PARTIAL` 不会被改写成整物体完整巡检。

每次运行保存：

| 文件 | 证据 |
| --- | --- |
| `manifest.json` | 命令、测试场景、隔离配置、参数文件、检测模式与缺失依赖 |
| `report.json` | 实际判据、阶段、频率、PVA 最大值、照片数、最终覆盖与失败原因 |
| `states.jsonl` / `events.jsonl` | 主状态、运动/制动、区域、覆盖及照片质量事件 |
| `yolo_image_chain.jsonl` | 真实 YOLO 曝光、源图与原始 RGB 的严格配对及像素哈希 |
| `truth.jsonl` | 当前隔离 partition 的原始具名模型原点、实体 ID、采样时间与姿态 |
| `gcs_health.json` / `gcs_heartbeats.jsonl` | 独立 GCS 的实际 PX4 对端、双向心跳数量/年龄和只读参数；不关闭 PX4 数据链路门控 |
| `navigation.jsonl` / `tracking.jsonl` / `calibration.jsonl` | 完整实测速度、姿态、原始位置 ENU/速度 FLU 协方差及旋转后的速度 ENU 协方差；逐采样范数与各轴跟踪误差/上取整界限/余量；导航对模型原点和错误 base_link 参考的偏差、真实深度墙面残差 |
| `setpoints.jsonl` | 带接收时间与采样时间的位置、速度、加速度和偏航输出 |
| `braking.json` | 障碍场景的实际停止距离/耗时、服务窗口、运动快照及采样机体包含证据；当前物理归因与连续边界未确认 |
| `agent.log` / `gazebo.log` / `px4.log` / `sensors.log` / `mission.log` | 各真实进程启动和故障信息 |
| `photos/` | 通过图像质量并实际写盘的 RGB |

场景、参数和 UWB 缺省随机种子可复现运行条件，但 Gazebo 渲染/调度不保证逐位相同；此入口没有默认录制大体积原始图像 rosbag。`replay` 特指明确标记的检测重放，不把它叫作真实 YOLO 或硬件传感器重放。

实机验收还需要机体包络、载荷下制动能力、跟踪误差、时延、完整相机外参和时钟的实测标定。室内未知、玻璃、动态人员和定位完全丢失不能由这些静态测试推导出通用安全性。

## 5. 本机实际执行记录（2026-10-26）

以下日志位于工作区 `log/`，不随源码提交。它们保留每次真实失败，不用后续降载或基础测试覆盖早先结论。

| 验证范围 | 实际结果及证据 |
| --- | --- |
| 两包 Release 单编译任务构建 | 通过，最新 `log/p2_build_logic_review.log`（5 分 42 秒），编译未报告 warning/error |
| C++、对照和跨进程 ROS | **204 项、零失败、零跳过**，`log/p2_baseline/native_logic_review_result.log`；其中 156 原生、26 对照、7 ROS，另含 15 个 CTest 外层结果 |
| Python 共享算法/节点 | **127 项通过**，`log/p2_baseline/python_bootstrap.xunit.xml`（6.13 秒）；加上上述直接用例共 **316 项**，不重复计算 CTest 包装 |
| SDF 机体包络解析检查 | 通过，10 个碰撞体、旋翼任意偏航与总倾角 ≤0.35 rad 的解析界；`research_sources/sitl_body_envelope.json`、`log/p2_body_audit_checks.json`。不代表实际制动响应已标定 |
| 真实 YOLO 源图链路 | **通过独立范围**，`log/p2_sitl_yolo_image_chain/report.json`：3 次真实推理、3 次曝光/原始像素严格配对、零错配，未请求 START；目标框为 0，不代表检测精度或巡检成功 |
| 静态导航参考/墙面投影 | `log/p2_sitl_cpu_11/report.json` 的静态子检查通过：28 帧、3024 个墙面点，真值/导航墙面残差 p95 约 0.0361/0.1949 m，导航模型原点误差 p95 约 0.0488 m；完整报告仍未通过 |
| 完整室内飞行 | **尚未通过**。旧软件/参数下 CPU17 已完成微上升参考并进入下一上升段，随后触发跟踪保护，`log/p2_sitl_cpu_17/report.json` 保留失败。ULog 显示起飞整形产生向下速度参考及正积分偏置，推力/运动响应尚不符合配置；后续软件核查还发现采样时刻参考错配，已修复并通过独立回归。本轮不再调飞控参数或追加试飞，不能据旧失败判定新软件的完整飞行结果 |
| 多房间、障碍制动、原始负载与实机 | 尚未完成验收。原始渲染负载及 CPU 默认 10 Hz 档启动时已触发健康失鲜；2.5 Hz 档只用于明确标记的降载功能验证 |

完整飞行的后续运行必须继续满足第 4 节全部判据；静态子检查、有效 PVA 数量、进入 Offboard 或接受 START 都不能替代已解锁飞行、巡检和落地证据。

## 6. P2 逻辑核查与回归覆盖

本轮发现的问题已修复并纳入上述完整回归。它们独立于飞控参数的调节：

| 逻辑边界 | 修复与验证 |
| --- | --- |
| 延迟导航与阶段切换 | 精确参考阶段历史按采样时间解析；`P2MotionTiming` 四项覆盖合法 0.1 s 延迟、真实误差仍拒绝、跨制动/HOLD 的迟到样本、新 HOLD 需要新测量、重置/采用不串 epoch。旧实现三项确定性失败见 `log/p2_reference_timing_red.log` |
| 规划与执行预算一致 | `P2MissionPlanning` 七项覆盖贪心局部长度、拐角剩余长度、实际停车次数、返航偏航、拒绝结果保留、只读误差预算快照与完整路径/首局部段分别验证 |
| 不确定度短暂峰值 | 当前停止证明有效的输入峰值供下一次完整准备验证；采样回落不会重建过小证书。原始位姿/协方差不改，错任务/epoch/配置与无效状态不复用快照 |
| 区域身份 | 同 epoch 的配置变化不会复用新区域 ID，旧重访/滞回记忆不会串区域；epoch 重置才重新编号 |
| 巡检收益 | 去返实际时序参与观察点评分，回调拒绝、NaN、负耗时均不回退距离估计；覆盖“几何更长但实际更快”的选择 |
| 照片覆盖证据 | 提交须绑定真实曝光及该曝光的面片版本，拒绝伪造采样、跨曝光面片、冲突重复条目；保存失败、质量失败、迟到结果、分母增长和重复样本已有回归 |

安全/标定参数应在任务开始前配置；调整后在上锁状态重启节点/任务。本轮测试不把运行中热切换机体/制动模型视为受支持接口。已验证的内容是当前测试、回放和日志覆盖到的软件行为；真实停止距离、跟踪包络、满负载延迟以及实机结果仍须使用用户标定后的参数实测。
