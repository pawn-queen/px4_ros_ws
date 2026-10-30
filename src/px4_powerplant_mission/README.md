# px4_powerplant_mission

面向 PX4 的 ROS2 Jazzy 任务包，定位与外部视觉支持仿真和实机共用。仿真入口为 `PX4_GZ_WORLD=powerplant make px4_sitl gz_x500_depth`。默认由独立的 `px4_powerplant_mission_cpp` 包执行体素建图、三维规划和 Offboard 控制；定位、UWB、外部视觉桥和 YOLO 继续使用 Python。它不包含硬件驱动；新入口 `profile=simulation` 使用仿真 UWB，`profile=hardware` 接入独立的 UWB Host；旧 `localization_profile` 参数保留兼容。雷达/深度相机/相机通过 Gazebo bridge 或真实驱动接入。定位部署、接口变更、时间与标定要求见 [定位部署说明](LOCALIZATION_DEPLOYMENT.md)。当前工程已在 `drone-simulation/Tools/simulation/gz/models/x500_depth/model.sdf` 中给 `x500_depth` 增加了 360 度平面 `mapping_lidar`，原启动命令不需要改。

## 包结构

Python 包保留定位、UWB、外部视觉和 YOLO；建图与控制由独立的 C++ 包提供，原 Python mapper/controller 已移除。C++ 模块划分、接口差异与验证命令见 [原生包说明](../px4_powerplant_mission_cpp/README.md)。

- `nodes/localization_node.py`：默认把 PX4 完整 odometry 转为 ROS ENU/FLU 的 `/powerplant/localization/odom`，同时独立检查 UWB 健康；旧自适应融合保留为导航侧可选模式。
- `nodes/external_vision_bridge_node.py`：直接把独立 UWB 位姿转成 PX4 NED 下的 `/fmu/in/vehicle_visual_odometry`，不再订阅融合里程计，供 PX4 EKF 室内解锁和 Offboard 位置控制使用。
- `nodes/uwb_simulator_node.py`：用 Gazebo `dynamic_pose` 真值生成带采样时间的逐锚模拟测距，经有效性、残差和锚点几何检查后发布 `/uwb/pose` 与 `/uwb/quality`。
- `nodes/yolo_detector_node.py`：加载包内 `models/yolo/best.pt`，发布 JSON 检测结果和标注图。
- C++ 包 `voxel_mapper_node`：体素建图节点（默认建图器）。
- C++ 包 `offboard_mission_node`：任务状态机、Offboard 控制、探索/巡检/返航（默认控制器）。
- `config/powerplant.yaml`：所有话题名、坐标系、UWB anchor、建图分辨率和探索边界参数。
- `launch/powerplant_system.launch.py`：启动任务栈。
- `launch/gz_sensor_bridge.launch.py`：可选启动 Gazebo sensor 到 ROS2 的桥接。

## 坐标约定

PX4 uORB 使用 `NED/FRD`，本包内部和对外发布使用 ROS 常用的 `ENU/FLU`：

- ENU：`x=east, y=north, z=up`
- NED 到 ENU：`[north, east, down] -> [east, north, -down]`
- FRD 到 FLU：`[forward, right, down] -> [forward, -right, -down]`
- Gazebo `dynamic_pose`/odometry 是 ENU；探索边界和 `uwb_anchors` 直接使用 Gazebo world 的 x/y/z；探索无需预置目标航点。

## 运行顺序

先构建工作区：

```bash
cd /home/pawn/px4_ros_ws
deactivate 2>/dev/null || true
source /opt/ros/jazzy/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 colcon build --symlink-install \
  --packages-up-to px4_powerplant_mission --executor sequential --parallel-workers 1 \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

上述命令同时构建 C++ 包及依赖，并限制包之间和 C++ 编译器内部的并发。WSL 上请在编译结束后再运行测试或 Gazebo，避免同时占用大量内存。

以下参数仅用于本项目带独立 Gazebo 真值姿态的 SITL，不可照搬到单 Tag 实机。首次运行该仿真前，在 PX4 shell 或 QGC 参数页设置：目标是：不要求 GPS/磁罗盘，把 ROS 室内定位作为 EKF external vision 输入：

```bash
param set EKF2_EV_CTRL 11
param set EKF2_EV_QMIN 0
param set EKF2_HGT_REF 3
param set EKF2_GPS_CTRL 0
param set SYS_HAS_GPS 0
param set SYS_HAS_MAG 0
param set EKF2_MAG_TYPE 5
param set SIM_GZ_EN_ODOM 1
param set COM_RC_IN_MODE 4
param set COM_ARM_WO_GPS 2
param set COM_ARM_ODID 0
param set COM_HOME_IN_AIR 1
param save
reboot
```

其中 `EKF2_EV_CTRL=11` 表示融合 external vision 的水平位置、高度和 yaw，不融合速度；当前仿真链路会把 Gazebo 真值姿态作为 UWB pose 的仿真 yaw 来源，避免把 PX4 自己的 yaw 再回灌给 EKF。如果把 `powerplant_uwb_simulator.publish_truth_orientation` 改成 `false`，对应把 `EKF2_EV_CTRL` 改为 `3`，只融合 external vision 的水平位置和高度。默认导航姿态来自 PX4 完整四元数；关闭真值姿态后，仍需 PX4 自身具备可靠姿态和航向。单 Tag 不提供航向。

终端 1，启动 PX4 DDS Agent：

```bash
source /opt/ros/jazzy/setup.bash
MicroXRCEAgent udp4 -p 8888
```

终端 2，启动 PX4 Gazebo：

```bash
cd /home/pawn/drone-simulation
PX4_GZ_WORLD=powerplant make px4_sitl gz_x500_depth
```

终端 3，启动本任务栈：

```bash
cd /home/pawn/px4_ros_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch px4_powerplant_mission powerplant_system.launch.py
```

这个 launch 会默认启动 Gazebo 真值桥接及 `/world/powerplant/clock` 到 ROS `/clock` 的桥接。UWB、定位、外部视觉桥、建图、YOLO 和 Offboard 节点统一使用 `use_sim_time`，使传感器和状态采样处于同一时钟域；原生节点另用单调接收时限检测数据停止和仿真时钟冻结，旧融合计数不能刷新健康。UWB 默认使用 `/world/powerplant/dynamic_pose/info` 的模型位姿；`/model/x500_depth/odometry_with_covariance` 仍保留为可选 odometry 后备源。如果你已经在别处桥接同一 truth topic，可加 `use_gazebo_truth_bridge:=false`；如果已经有 `/clock` 发布者，可加 `use_gazebo_clock_bridge:=false`。默认不会启动 YOLO 和飞控。需要时显式打开：

```bash
ros2 launch px4_powerplant_mission powerplant_system.launch.py use_yolo:=true use_control:=true
```

建图与控制由 `px4_powerplant_mission_cpp` 提供，沿用原节点名、YAML、话题、消息类型、QoS 和服务接口。C++ 已增加 P0 健康协议、[P1 建图/观测改进](../../docs/design/mapping-and-frontier.md) 和 [P2 轨迹/巡检闭环](../../docs/design/motion-and-inspection.md)。原 Python mapper/controller 已移除，不再提供 `mapping_package`/`control_package` 回退开关。

控制仍默认关闭，`auto_start=false` 保持不变。C++ 算法与接口测试不能替代 PX4/Gazebo 完整飞行验收；完整巡检、避障和返航仍需在仿真中验证。

起飞响应同步使用同一 C++ 执行核心：参考领先时执行已验证制动，在实际响应恢复后从 HOLD 重新规划；启动 2cm 参考不能冒充已经离地。隔离验收入口在实际场景地面审计通过后，生成临时运动配置开启反馈探测与一次性过渡预算，严格阶段基础位置容差仍为 0.10m；经实际支持审计的SITL过渡可选择A—D白名单（0.10/0.12/0.15m，D半径0.48m），固定预算上限45秒，地面过渡限高仍为0.75m。普通 launch 在没有该审计时保持严格模式并可正常等待授权；硬件缺省不启用这些仿真专用选项，原 12s 缺省不变。独立 `config/p2_sitl_controller.yaml` 由隔离验收脚本读取，仅在临时 PX4 rootfs 解锁前应用，不能作为硬件 ROS 参数文件。流程、标定次序和实际验收范围见 [起飞响应说明](../../docs/design/takeoff-response-sync.md) 与 [Issue #3 验收记录](../../docs/validation/issue3-actual-plant.md)。

飞控节点启动后仍不会自动执行任务，默认需要服务触发：

```bash
ros2 service call /start_powerplant_mission std_srvs/srv/Trigger
ros2 service call /land_powerplant_mission std_srvs/srv/Trigger
```

如果按“非虚拟环境”启动 ROS 任务栈，也可以直接 `use_yolo:=true`；launch 默认会用
`/home/pawn/px4_ros_ws/.venv/bin/python` 运行 YOLO 节点，避免系统 Python 缺少
`ultralytics` 时产生启动错误。若已经把 YOLO 依赖安装到系统 Python，可覆盖
`yolo_python:=python3`。

如果 QGC 仍显示 `Heading estimate invalid`、`Found 0 compass` 或 `No valid position estimate`，先确认 `/fmu/in/vehicle_visual_odometry` 正在发布，再等 EKF 收敛几秒：

```bash
export ROS2CLI_ENABLE_DAEMON=0
ros2 topic echo /world/powerplant/dynamic_pose/info tf2_msgs/msg/TFMessage --once --timeout 5 --no-daemon
ros2 topic echo /uwb/quality std_msgs/msg/String --once --timeout 5 --no-daemon
ros2 topic echo /uwb/pose --once --timeout 5 --no-daemon
ros2 topic hz /fmu/in/vehicle_visual_odometry
ros2 topic echo /fmu/out/estimator_status_flags px4_msgs/msg/EstimatorStatusFlags --once --timeout 5 --qos-reliability best_effort --no-daemon
ros2 topic echo /fmu/out/failsafe_flags px4_msgs/msg/FailsafeFlags --once --timeout 5 --qos-reliability best_effort --no-daemon
```

建图启动前还应确认 `/clock` 持续推进，且 `/mapping_lidar`、`/depth_camera` 与 `/powerplant/localization/odom` 的 `header.stamp` 同属仿真时间：

```bash
ros2 topic echo /clock rosgraph_msgs/msg/Clock --once --timeout 5 --no-daemon
ros2 topic echo /mapping_lidar sensor_msgs/msg/LaserScan --once --timeout 5 --no-daemon
ros2 topic echo /powerplant/localization/odom nav_msgs/msg/Odometry --once --timeout 5 --no-daemon
```

`estimator_status_flags` 里 `cs_ev_pos`、`cs_ev_yaw`、`cs_ev_hgt` 变成 `true` 后，再调用 `/start_powerplant_mission`。
如果不加 `--no-daemon` 时出现 `TimeoutError: [Errno 110] Connection timed out`，这是 ROS2 daemon
卡住，不是 topic 没有数据；先执行 `ros2 daemon stop` 或继续使用上面的 `ROS2CLI_ENABLE_DAEMON=0`。

当前 `powerplant` world 中 `/model/x500_depth/odometry_with_covariance` 可能只显示有 publisher 但不实际发布消息，所以不要把 UWB 卡在这个 topic 上。若 UWB 日志出现 `waiting for Gazebo dynamic pose truth`，优先检查 `/world/powerplant/dynamic_pose/info` 的 `ros_gz_bridge`；若 `/uwb/pose` 不发布，后面的 localization、external vision 和 PX4 EKF 都不会得到有效位置/航向。

当前链路默认要求 UWB/Gazebo 真值是“独立定位源”。仿真器对每个新的 Gazebo 采样时间只生成一组逐锚测距；非有限值和量程外值不裁剪成伪有效数据，也不发布对应 Range。解算至少需要四个几何有效锚，检查逐锚异常、整体残差与几何条件数，并把解算得到的三轴位置协方差写入 `/uwb/pose`。`/uwb/quality` 会报告已用/剔除的锚、残差、几何条件数、协方差和拒绝原因。当前锚点中有两个不同高度的高锚，删除任意一个锚后其余四锚仍非共面。`range_positive_bias_probability` 与 `range_positive_bias_m` 仅用于可关闭的仿真正偏差测试，默认都是零；它们不是学习式 NLOS 识别。

默认 `localization_node` 从 PX4 获取完整位置与姿态，并保留状态的采样时间。外部视觉桥直接检查并转发独立 UWB 测量，避免将含 PX4 信息的导航结果回灌。两个节点共用 UWB frame、时间、位置协方差与硬件健康检查；停止/故障或 UWB 超时会使导航位置方差退化。`navigation_mode=adaptive` 保留旧自适应融合供对照，结果只用于导航，不进入 EV。详细接口和硬件配置见 [定位部署说明](LOCALIZATION_DEPLOYMENT.md)。

外部视觉桥继续向 PX4 发布逐轴位置方差，但没有经真实数据标定的 0–100 质量分数，因此 `VehicleOdometry.quality=0`。本工程要求 `EKF2_EV_QMIN=0`（上面的参数命令已显式设置）；若已有非零门限，PX4 会停止融合这些 external vision 观测。不要把未标定的分数填成 100 来绕过该门限。

本包提供 Gazebo 模拟逐锚距离，外部 Host 已提供真实串口观测、时间戳及日志接口，但尚未用实测数据完成本项目飞行链路标定；学习式 NLOS 识别与因子图应在这些数据及所需传感器到位后再评估。`publish_truth_orientation=true` 注入的是 Gazebo 真值姿态，UWB 测距本身不提供横滚、俯仰或偏航。

飞行过程中 `/powerplant/control/mission_status` 里有两个位置：`nav_target_enu` 是当前逻辑航点，
`setpoint_enu` 是经过时序轨迹约束后的实际 PX4 setpoint。正常情况下 `setpoint_enu` 会逐步靠近
`nav_target_enu`，不是瞬间跳到远端航点。

调用 `/start_powerplant_mission` 前，应确认定位健康、飞控新鲜且已上锁，并配置已实测确认的起飞柱体。当前原生控制器默认 `takeoff_region_confirmed=false`、`takeoff_region_height_m=0`，条件不足时服务返回拒绝原因。通过后仍保留预热与预检。可用下面的话题确认状态：

```bash
ros2 topic echo /powerplant/control/mission_status std_msgs/msg/String --once --timeout 5 --spin-time 10 --full-length --no-daemon
```

其中 `localization_ready` 必须为 `true`，`pre_flight_checks_pass` 应为 `true`，且定位的横滚／俯仰方差必须低于 `initial_scan_max_tilt_variance_rad2`；
`command_wait_reason` 会显示正在等待定位、preflight、Offboard setpoint 预热、或 PX4
进入 Offboard。满足条件后节点按顺序请求 Offboard，然后再请求 arm。

`/start_powerplant_mission` 会进入完整巡检流程：

1. `START`：等待独立定位和可靠姿态估计，记录当前位置为 home。起飞区及垂直上升通道须预先确认安全；现有前向深度相机与水平雷达无法直接证明头顶无障碍。
2. `SAFE_TAKEOFF`：保持 home 水平位置，在已确认柱体内发布限速度/加速度的 Offboard 位置、速度和加速度参考，上升到 `home.z + takeoff_height_m`，相对起飞高度定义见 [P0 配置](../../docs/validation/input-map-health.md)。
3. `HOVER_SCAN`：在起飞点上方稳定悬停至少 `initial_scan_hover_s`，先用水平雷达取轮廓，再缓慢绕偏航轴转动，让前向深度相机观察各方向。默认 30° 搜索步幅、偏航 setpoint 最大 `0.2 rad/s`，不主动做大幅横滚或俯仰。完成条件改为合格实际深度射线方向覆盖（默认 72 个扇区、至少 90%）、稳定悬停、真实标定、雷达/深度实际融合，以及有净空的已知空闲首段（默认 0.75 m）；名义 12 步不能单独结束初扫。缺失深度时可在已确认起飞柱体内继续慢偏航寻找有效方向，雷达失鲜时暂停转动。若 90 秒仍无法完成，会请求降落。`mission_status` 的 `initial_scan_wait_reason`、`initial_scan_coverage_fraction` 和 `initial_scan_step` 可用于诊断。
4. `EXPLORE`：从三维体素中的**已知空闲与未知的边界**提取前沿，候选观察点位于边界的已知空闲侧，并要求从当前位置和回 home 的路径都在已知空闲体素内，沿途满足三维净空。按成功融合的实际 CameraInfo 视锥对每个候选 yaw 估计新增可见未知体素、目标表面补拍收益、平移/转向/观察停留时间、净空、定位方差和返航余量；只下发最多 `explore_local_path_length_m` 的局部路径。最终观察点必须对准、停留并取得新合格深度才完成，记忆按位置与 yaw 保存。新几何观测、路径受阻、局部段结束或 YOLO 目标会触发决策；远处无关变化允许复用仍安全的异步结果，附近变化重新评估。局部无候选扩大一次搜索，分类报告原因，不把候选截断当作全图探索完成。YOLO 事件优先转入巡检。局部任务 ROI 由 `explore_min/max_*_m` 指定，只限定探索收益、巡检表面和完成范围。起飞、初扫、接入、绕障取景及返航使用完整已知安全空间；缺连接时可限预算自主接入，不预置航点。具体接口与验收见 [Issue #3 实现说明](../../docs/design/issue3-collision-and-roi.md)。
5. `TARGETING_CYCLE`：YOLO 以三次独立注册观测确认目标，控制端按曝光匹配源 RGB、深度、两套 CameraInfo 与插值位姿，构建稳定表面片。按新增可见表面收益选择安全观察点，照片须目标关联、质量与写盘全部成功才提交覆盖；每点最多三次尝试，未满足条件返回 PARTIAL/UNCONFIRMED。达到90%的可达已建模表面覆盖报告 REACHABLE_MODELED_COMPLETE，不宣称完整物体360°覆盖。
6. `RETURN_HOME`：照片完成、允许边界内前沿耗尽、扩大搜索后策略仍无候选，或返航余量不足时，沿已知空闲的三维局部路段返回 home；分别报告原因。若地图过期、不完整或无安全返航路线则悬停并在 `mission_status.explore_wait_reason` 报告，不会向未知空间直飞。
7. `LANDING` / `COMPLETE`：到达 home 上方后发送 PX4 land command，收到降落后的新鲜 landed 状态且飞控已上锁，持续确认后才标记任务完成。超时保留 LANDING 并报告未确认。原生控制器发生定位/时钟/坐标重置等故障时进入锁定的 FAULT，停止 Offboard 输出，数据恢复不会自动重新解锁。

建图器缓存带时间戳的定位位姿，在雷达、深度图和点云采样时刻插值位置与姿态；有逐束 `time_increment` 的雷达扫描会按每束时间补偿偏航运动。没有有效时间戳或附近定位样本的数据不会投进地图。UWB 滤波延迟、相机曝光时间和定位误差仍可能形成投影残差。C++ 深度采样以 16 像素为最大 tile 尺度，按投影足迹和边缘细分，最细 1 像素、每帧最多 12000 条射线；空闲更新使用精确体素遍历。深度图与雷达各最多融合 3 帧/秒，地图以 2 Hz 发布。`/powerplant/map/observed_voxels` 是带 `x,y,z,state` 字段的完整已观测体素云，`state=0` 为已知空闲、`1` 为占据、`-1` 为证据不足；未出现的体素保持未知。控制节点用其时间戳与 `/powerplant/map/fusion_status.map_stamp_ns` 精确配对，只接受完整、健康且 epoch/序号有效、点云指纹一致的快照；分类差分驱动增量前沿/净空索引。新地图健康协议和跨进程 Cyclone DDS 基线入口见 [P0 验证说明](../../docs/validation/input-map-health.md)。`max_publish_points` 是快照上限，超过时控制节点停止新探索；当前上限为 200000。2D `/powerplant/map/occupancy_grid` 仍供初扫判据、辅助避障与可视化使用，巡检及返航路线都由三维已知空闲体素规划。`/powerplant/control/planned_path` 显示探索与返航的当前局部路段，或巡检阶段的已验证路线。

P1 的视场默认要求与成功融合深度匹配的真实 CameraInfo（`explore_require_camera_info=true`），输入须是对应 K 的已校正 pinhole 深度；没有标定时等待，不用示例 FOV 冒充。新增参数、时间预算公式、失败原因及算法原理见 [P1 实现说明](../../docs/design/mapping-and-frontier.md)。P2 在几何路径之后增加五次时间轨迹和持续停止包络，拐角停稳；独立50 Hz发布，主决策10 Hz。所有完整飞行和大地图实时性能结论以迁移记录中的实际验收为准。

返航余量优先使用有效的 PX4 `/fmu/out/battery_status.time_remaining_s`，同时受 `explore_max_duration_s` 限制；电池时间不可用时使用任务时间上限估计。首次 YOLO 发现之前没有目标表面语义，补拍收益为零；目标确认后按真实面片收益巡检。定位质量评分以当前定位协方差为基准，并按离当前位置的欧氏距离做保守方差增长估计（`explore_localization_variance_growth_m2_per_m`）；这不是实测的空间定位质量图。净空半径、定位方差门限和探索边界须按机体尺寸及定位精度调整。

## 可视化窗口

建议先关掉卡住的 ROS2 daemon，后续诊断命令都可以复用同一个终端环境：

```bash
cd /home/pawn/px4_ros_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 daemon stop
export ROS2CLI_ENABLE_DAEMON=0
```

### 启动顺序

RViz 只负责显示，不会主动创建地图或路径。先按正常流程启动 PX4/Gazebo 和任务栈。若需要看到
`/powerplant/control/planned_path`，任务栈必须带 `use_control:=true`，因为该 topic 由
`offboard_mission_node` 发布（C++）；仅运行默认 `ros2 launch px4_powerplant_mission powerplant_system.launch.py`
时不会启动飞控节点，也就不会有路径 topic。

```bash
cd /home/pawn/px4_ros_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch px4_powerplant_mission powerplant_system.launch.py use_yolo:=true use_control:=true
```

`use_control:=true` 只启动 Offboard 节点；默认 `auto_start=false`，不会自动解锁和起飞，仍需手动调用
`/start_powerplant_mission`。

如果图像、深度图或雷达 topic 没有出现，再启动 Gazebo sensor bridge：

```bash
cd /home/pawn/px4_ros_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch px4_powerplant_mission gz_sensor_bridge.launch.py
```

### RViz 一键窗口

推荐直接用本包的 RViz 配置启动，它已经预置栅格、路径、定位、UWB anchor 和点云显示：

```bash
cd /home/pawn/px4_ros_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch px4_powerplant_mission visualization.launch.py
```

RViz2 中 `Fixed Frame` 应为 `map`；`visualization.launch.py` 默认启用 `use_sim_time=true`，以匹配地图和定位 TF 的仿真时间。预置显示项包括：

- `Map`：topic 选 `/powerplant/map/occupancy_grid`
- `Path`：topic 选 `/powerplant/control/planned_path`
- `Odometry`：topic 选 `/powerplant/localization/odom`
- `PointCloud2`：topic 选 `/powerplant/map/occupied_voxels`
- `MarkerArray`：topic 选 `/uwb/anchors`
- `MarkerArray`：topic 选 `/powerplant/map/voxel_markers`

也可以手动启动空 RViz：

```bash
rviz2
```

然后把 `Fixed Frame` 设为 `map`，按上面的 topic 添加显示项。

### 路径 Topic 找不到

如果 RViz 里找不到 `/powerplant/control/planned_path`，优先检查是否启动了 `use_control:=true`：

```bash
ros2 node list --no-daemon --spin-time 10 | grep powerplant_offboard_mission
ros2 topic list --no-daemon | grep planned_path
```

正常应能看到：

```text
/powerplant_offboard_mission
/powerplant/control/planned_path
```

如果看不到 `/powerplant_offboard_mission`，重启任务栈并加上：

```bash
ros2 launch px4_powerplant_mission powerplant_system.launch.py use_yolo:=true use_control:=true
```

如果 node 存在但 topic 还没显示，读取一次消息：

```bash
ros2 topic echo /powerplant/control/planned_path nav_msgs/msg/Path --once --timeout 5 --spin-time 10 --no-daemon
```

如果能 echo 到消息但 RViz 没有线，检查 RViz 的 `Fixed Frame` 是否为 `map`，`Path` display 的
topic 是否手动填成 `/powerplant/control/planned_path`。路径 topic 会周期性重发，打开 RViz
后不需要重新启动任务。

### 相机和深度图

先确认图像 topic 在发布：

```bash
ros2 topic list --no-daemon | grep -E 'camera|depth|annotated'
```

用 `rqt_image_view` 打开窗口后在下拉框选择 topic：

```bash
ros2 run rqt_image_view rqt_image_view
```

常用 topic：

- `/camera`：RGB 相机原图
- `/depth_camera`：深度图
- `/powerplant/perception/yolo_annotated`：YOLO 标注图，只有 `use_yolo:=true` 且 `publish_annotated:=true` 时发布

也可以直接用 `image_view` 打开单个窗口：

```bash
ros2 run image_view image_view --ros-args -r image:=/camera
ros2 run image_view image_view --ros-args -r image:=/depth_camera
ros2 run image_view image_view --ros-args -r image:=/powerplant/perception/yolo_annotated
```

如果这些命令不存在，安装对应工具包：

```bash
sudo apt install ros-jazzy-rviz2 ros-jazzy-rqt-image-view ros-jazzy-image-view
```

### 点云

点云显示依赖三件事：定位 `/powerplant/localization/odom` 正常、`voxel_mapper_node` 正在运行、
至少一个建图传感器 topic 正常发布。先检查：

```bash
ros2 node list --no-daemon --spin-time 10 | grep powerplant_voxel_mapper
ros2 topic echo /powerplant/localization/odom nav_msgs/msg/Odometry --once --timeout 5 --spin-time 10 --no-daemon
ros2 topic echo /depth_camera sensor_msgs/msg/Image --once --timeout 5 --spin-time 10 --no-daemon
ros2 topic echo /mapping_lidar sensor_msgs/msg/LaserScan --once --timeout 5 --spin-time 10 --no-daemon
ros2 topic echo /powerplant/map/occupied_voxels sensor_msgs/msg/PointCloud2 --once --timeout 5 --spin-time 10 --no-arr --no-daemon
```

`/depth_camera` 和 `/mapping_lidar` 只要有一个持续有数据，融合点云就会逐渐出现。若这两个 topic
都没有数据，先启动 `gz_sensor_bridge.launch.py`，或用 `gz topic -l | grep -E 'camera|depth|scan|points'`
确认 Gazebo 侧真实 topic 名称。

用 RViz2 查看建图后的点云时，`Fixed Frame` 设为 `map`，添加：

- `PointCloud2`：topic 选 `/powerplant/map/occupied_voxels`
- `MarkerArray`：topic 选 `/powerplant/map/voxel_markers`

建议 `PointCloud2` 的 `Style` 设为 `Points`，`Size (m)` 设为 `0.05~0.12`。如果 Gazebo 原始点云也已经桥接到 ROS，可以再添加 `PointCloud2` topic `/points`。当前 `voxel_mapper_node` 会把深度图、`mapping_lidar` 和 `/points` 融合后发布 `/powerplant/map/occupied_voxels`，所以通常看这个融合后的 topic 更直观。

## Gazebo 相机/深度图桥接

PX4 的 `/fmu/out/...` 话题由 `MicroXRCEAgent` 提供；Gazebo 传感器图像通常还需要 `ros_gz_bridge`。先查看 Gazebo topic：

```bash
gz topic -l | grep -E 'camera|depth|scan|points'
```

当前 x500_depth_0 默认使用 `/depth_camera`、`/mapping_lidar`，深度 CameraInfo 实际位于 Gazebo `/camera_info`，RGB 与其 CameraInfo 使用包含 world/model/IMX214 的完整路径。bridge 默认已覆盖这一布局，并统一映射到 ROS `/camera`、`/camera/camera_info`、`/depth_camera/camera_info`：

```bash
ros2 launch px4_powerplant_mission gz_sensor_bridge.launch.py
```

如果 topic 是带 world/model 前缀的长名字，就覆盖 launch 参数，例如：

```bash
ros2 launch px4_powerplant_mission gz_sensor_bridge.launch.py \
  depth_gz_topic:=/depth_camera \
  depth_info_gz_topic:=/camera_info \
  rgb_gz_topic:=/world/powerplant/model/x500_depth_0/link/camera_link/sensor/IMX214/image \
  rgb_info_gz_topic:=/world/powerplant/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info \
  scan_gz_topic:=/mapping_lidar
```

## 重要输出话题

- `/powerplant/localization/odom`：融合后的室内位姿，`nav_msgs/Odometry`
- `/fmu/in/vehicle_visual_odometry`：桥接给 PX4 EKF 的室内 external vision 位姿，`px4_msgs/VehicleOdometry`
- `/powerplant/localization/status`：定位状态 JSON
- `/uwb/pose`、`/uwb/range/<anchor>`、`/uwb/quality`：仿真 UWB 位姿、有效逐锚距离和解算质量
- `/powerplant/map/occupied_voxels`：已占据体素点云
- `/powerplant/map/observed_voxels`：带空闲、占据和不确定状态的完整三维已观测体素云
- `/powerplant/map/fusion_status`：建图健康、融合计数、版本、分辨率、快照完整性及成功深度的相机/观测元数据 JSON
- `/powerplant/map/occupancy_grid`：初扫判据、辅助避障和可视化使用的 2D 占据栅格
- `/powerplant/control/planned_path`：当前任务阶段的规划路径，`nav_msgs/Path`
- `/powerplant/perception/yolo_detections`：YOLO JSON 检测结果
- `/powerplant/perception/target_position`：YOLO + 深度相机反投影得到的目标 ENU 三维坐标，`geometry_msgs/PointStamped`
- `/powerplant/control/mission_status`：任务状态机 JSON，包括 `phase`、航点、照片计数、目标坐标
- `/powerplant/control/inspection_events`：目标发现、目标坐标、拍照、返航、降落等事件 JSON
- `/powerplant/perception/yolo_source_image`：与检测同一曝光的未标注源图，用于注册、质量与保存
- `/fmu/in/trajectory_setpoint`：给 PX4 的 Offboard 位置、速度、加速度及偏航参考

## YOLO 说明

默认模型已经放在包内的 `models/yolo/best.pt`，回退模型是 `models/yolo/yolov8n.pt`。当前 `best.pt` 检测类别见 `models/yolo/data.yaml`，已有类别为 `gas_cylinders`。

目标三维坐标的计算链路是：`/powerplant/perception/yolo_detections` 的 `bbox_xyxy` 取中心区域深度，使用 `/depth_camera/camera_info` 的内参反投影到相机光学坐标，再按 `camera_translation_flu` 和 `/powerplant/localization/odom` 姿态转换到 ENU `map` 坐标。RGB 与深度图需要来自同一前向相机或已近似对齐，否则目标坐标会有偏差。

工作区构建继续使用系统 Python，YOLO 依赖不要参与 `colcon build`。如果希望不使用虚拟环境运行 YOLO，需要把推理依赖安装到系统/用户 Python：

```bash
cd /home/pawn/px4_ros_ws
python3 -m pip install --user --break-system-packages -r src/px4_powerplant_mission/requirements-yolo.txt
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select px4_powerplant_mission --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
ros2 launch px4_powerplant_mission powerplant_system.launch.py use_yolo:=true use_control:=true
```

默认会使用 `/home/pawn/px4_ros_ws/.venv/bin/python` 运行 YOLO。也可以显式覆盖启动参数：

```bash
ros2 launch px4_powerplant_mission powerplant_system.launch.py \
  use_yolo:=true \
  use_control:=true \
  yolo_python:=/home/pawn/px4_ros_ws/.venv/bin/python
```

P2 仿真安装与运动参数由 `config/p2_simulation.yaml` 覆盖；`hardware` 使用 `p2_hardware.yaml`，缺省 `motion_model_configured=false`。实机须提供相对PX4导航参考点的实测外参、机体包络、制动/时延与误差范围后启用。相机旋转参数是相机FLU到机体FLU的安装旋转；内部已处理光学轴转换，不能重复填入optical轴转换。仍须独立确认起飞区，不能只为通过检查修改确认值。

当前仿真覆盖配置的 `motion_tracking_position_tolerance_m=0.10` m，作为基础跟踪容差；实际位置误差门还叠加定位不确定性，停止包络同步计入此容差。共享及硬件配置未随本次仿真试验修改。2026-10-28两次原PX4控制值的实际电厂单变量测试仍在离地过渡触发跟踪保护，未完成真实起飞，不能宣布初扫、ROI任务或返航通过；证据见 [Issue #3验收记录](../../docs/validation/issue3-actual-plant.md)。

仿真 P2 起飞先从已解锁后的当前连续、有效 HOLD 参考执行 `motion_takeoff_bootstrap_height_m=0.02` m 微上升，其一致的五次 P/V/A 至少持续 `motion_takeoff_bootstrap_duration_s=4.5` 秒，再经当前停止证书和实测停稳进入正常上升。该时长覆盖当前 PX4 的 spool 和 takeoff ramp，避免正常轨迹先跑完而飞控尚未响应；它不改变跟踪误差门控，也不以位置阶跃触发起飞。SITL 入口只读 `COM_SPOOLUP_TIME`、`MPC_TKO_RAMP_T`，核对微上升时长至少为两者之和加反应预算、微上升高度不超过位置跟踪容差，并记录实测飞行表现。共享与硬件缺省两值均为零，表示未启用或未标定；零不是已验证的零启动时延，硬件应根据实际飞控参数、载荷与响应重新标定，不能复制仿真值。

全系统 launch 根据已安装 `px4_msgs.MESSAGE_VERSION` 将默认 PX4 输入话题映射到对应版本后缀；例如当前 `vehicle_status_v4`、`vehicle_command_ack_v1`。直接运行节点或使用自定义话题时，须在参数中与实际固件一致。RGB 与深度都要求校正后的 pinhole 图像及匹配 CameraInfo；未校正畸变、未处理的立体参考/裁剪/binning 会被拒绝。

实际 P2 验收入口、检测重放与真实 YOLO 的区别、原始与 CPU 降采样渲染档的范围见 [P2 验证说明](../../docs/validation/sitl-acceptance.md)。任务规格与验证缺口跟踪在 [Issue #2](https://github.com/pawn-queen/px4_ros_ws/issues/2)。

本轮已完成 P2 软件逻辑核查，316 个直接回归用例通过；飞控/动力学参数由用户调整，完整飞行尚未验收。安全模型和标定参数应在任务开始前设置，调整后在上锁状态重启节点/任务；本轮不提供飞行中热切换机体、制动或误差模型的保证。上述微上升和时长仍是仿真初值，参数核对通过不等于实际推力或跟踪表现通过。

实机运动覆盖文件通过 `motion_config_file:=/绝对路径/实测_motion.yaml` 传入，优先级高于共享配置及 profile 默认。该文件同时配置mapper与controller的同一安装约定。

2026-10-29续改已接入起飞响应同步、软制动恢复锁存、独立控制配置和实际物理验收；最终506项直接软件回归通过。真实电厂允许的五个积分候选均未完成起飞初扫，未冻结控制配置，后续主链不能宣布可用。前述微上升时长仅证明参考意图已发送，实际离地仍要求起落架净空和连续新样本。入口与结果见 [起飞响应说明](../../docs/design/takeoff-response-sync.md) 和 [最新验收记录](../../docs/validation/issue3-actual-plant.md)。

2026-10-29最新续改：圆角停止扫掠、A—D过渡白名单、固定45秒预算和停稳HOLD参考管理已接入同一C++核心，严格阶段恢复原0.10m/0.50m/0.60—0.35/3秒策略。ACK绑定实际导航样本及不可变P/V/A/停止证据；新增选项只在IDLE接受，硬件缺省和PX4源码不改。两包构建与524项直接用例通过；实际电厂20候选均未过起飞初扫，5次严格ACK子项通过，完整主链仍未验收、未生成冻结配置。详见[最新验证](../../docs/validation/issue3-actual-plant.md)与工作区`log/issue3_margin_governor/20261029_1336/`。
