# P0 输入、地图与任务状态验证

更新：2026-10-25。适用于当前原生 C++ 建图器/控制器；仿真与实机使用同一实现，通过 `localization_profile` 和传感器标定配置区分。

## 已实现的行为

- 导航里程计必须有 ENU `map` / FLU `base_link`、有效采样时间、完整有限位姿与速度，以及六维有效协方差。位置和 roll/pitch/yaw 方差均受限；协方差需对称、半正定，零对角值按未知处理。
- 采样年龄使用同域 ROS 时间，接收年龄同时使用 ROS 时间和单调时钟。重复、倒序和过期样本不刷新健康状态；同采样时间的协方差退化仍会撤销有效状态。
- Python 定位状态新增 `status_stamp_ns`、`status_sequence`、`localization_epoch`。启动、ROS 时钟回退或 PX4 估计器重置会建立新坐标 epoch。它是身份标识，不能当作时间戳相减。UWB 健康/独立位置检查及禁止导航结果回灌 EV 的关系保留。
- mapper 冻结坏定位对应的地图积分；普通旧包不再触发 `PoseHistory` 清图。坐标重置清除地图、位姿历史和等待观测，切换地图 epoch。深度/雷达必须由采样时刻位姿包围后才融合，保留逐束补偿。
- 地图状态强制携带 epoch、序号、定位健康、位姿/传感器采样时间及点云指纹。退役 epoch 和旧序号不能恢复地图；点云仍是 `x,y,z,state` 四字段，指纹用于防止重置后同时间戳的错误配对。
- 每个主动运动阶段在发送 heartbeat、setpoint、模式/解锁请求之前检查健康。坏定位、飞控失联/failsafe、意外退出 Offboard/上锁、坐标重置进入锁定的 `FAULT`，取消任务并停止 Offboard 输出，交给已配置的 PX4 Offboard-loss 行为。恢复数据不会自动重解锁。
- 仅地图/路径暂时无效而导航仍可靠时，保留原有等待与位置保持行为，并持续消耗全程时间预算。连续动力学备用制动轨迹属于后续 P2，本次没有据此声称位置保持已获得形式化安全保证。
- 起飞区必须显式确认；起飞/返航高度以实际起飞点为相对基准。真实占据证据优先于起飞区确认。
- `COMPLETE` 需要降落命令之后的新鲜 `VehicleLandDetected.landed=true` 和新鲜 `VehicleStatus` 已上锁状态持续满足确认时间。高度或超时不能代替落地确认，超时保留 `LANDING` 并发出 `landing_unconfirmed`。
- 无新鲜 RGB 或写盘失败不增加照片数。原 `require_image_for_photo` 参数保留用于旧配置识别，但设为 false 也不能用无图事件完成原生巡检。

`FAULT` 停止 Offboard 输出不等同于已保证安全落地。必须在完整仿真和受控实机中验证实际飞控的 Offboard-loss 配置、反应时间和可用定位条件；本次不自动修改飞控参数。

## 起飞区域与启动

共享配置位于 [powerplant.yaml](../px4_powerplant_mission/config/powerplant.yaml)。默认：

```yaml
takeoff_region_confirmed: false
takeoff_region_radius_m: 1.0
takeoff_region_height_m: 0.0
takeoff_height_m: 5.0
return_home_height_m: 5.0
```

确认区域是以每次实际起飞位置为中心的竖直柱体。应在测量过地面/平台、顶棚和通道后配置；高度至少覆盖 `takeoff_height_m + explore_clearance_radius_m`，半径需覆盖机体净空及允许的横向误差。不能仅为通过检查填写示例高度。前向深度和水平雷达不能直接证明头顶没有障碍。

`start_powerplant_mission` 现在要求：当前无主动任务、导航与定位健康状态新鲜、飞控状态新鲜且已上锁、起飞区域确认并足够大。服务拒绝会给出原因。发生 FAULT 后也只有在这些条件重新满足后才能显式开始新任务；旧 home、目标和路线不得跨坐标 epoch 继续使用。

`takeoff_height_m` 和 `return_home_height_m` 现在是相对起飞高度；`inspection_altitude_m`、探索边界和地图坐标仍按 ENU 地图坐标配置。高平台部署时应同时检查这些任务范围。

## 时间与接口

launch 给定位、建图、YOLO 和控制器统一传入 `use_sim_time`。仿真默认使用 `/clock`，硬件 profile 默认墙钟；设备时间到 ROS 时间的变换仍由定位/驱动负责。PX4 `VehicleStatus` 与 `VehicleLandDetected` 使用设备微秒时间去重，不能直接减 ROS 时间；其流新鲜度使用本地接收时间。墙钟定时器与单调时钟能在 `/clock` 冻结时使导航健康过期。

输入健康状态 `/powerplant/localization/status` 必须包括：

```text
status_stamp_ns, status_sequence, localization_epoch,
navigation_sample_ns, uwb_input_healthy, has_independent_position
```

地图 `/powerplant/map/fusion_status` 的新增必需字段为：

```text
map_epoch, status_sequence, localization_healthy,
pose_last_sample_stamp_ns, pose_last_received_stamp_ns,
depth_last_sample_stamp_ns, lidar_last_sample_stamp_ns,
observed_cloud_fingerprint
```

同时保留原完整性、计数、地图时间戳和版本字段。原生控制器拒绝旧版地图健康协议；原 Python mapper/controller 已于 2026-10-30 删除。默认 C++ mapper/controller 的 simulation/hardware 路径共用新协议。

关键诊断字段：`health_fault_reason`、`last_input_reject_reason`、`odometry_rejected_count`、`odom_sample_stamp_ns`、`odom_received_stamp_ns`、`localization_epoch`、`map_epoch`、`map_localization_healthy`、`vehicle_status_fresh`、`landing_timeout`。

## 本机跨进程 ROS 配置

当前主机上默认 Fast DDS 的同机进程出现单向发现、无数据/服务响应；限制回环 UDP、显式 locator 与 TCP 对照没有恢复。原始根因未确定。Cyclone DDS 对照恢复了真实跨进程话题与服务，因此提供可选、本机范围的基线配置，不改 `.bashrc` 或系统网络。

正常安装可使用对应 Jazzy 软件包；本工作区也支持无 root 的锁定依赖提取：

```bash
cd /home/pawn/px4_ros_ws
/usr/bin/python3 tools/env/bootstrap_ros.py
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source tools/env/ros_env.sh
```

提取器只支持当前 Ubuntu noble amd64，以 [依赖锁定文件](../../tools/env/ros_dependencies.json) 中 SHA256 验证五个软件包并解压到被 git 忽略的 `.p0_deps/`；不安装系统包。其他平台应安装匹配平台的 ROS middleware。此配置设置 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` 和 `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`，只影响当前 shell 及子进程。

**LOCALHOST 配置用于本机联调。** 它没有验收 Windows Host、跨计算机 DDS、Micro XRCE-DDS Agent/PX4 与 Cyclone 的完整互通。外部设备部署应按真实接口和网络重新验证；不要将本机测试结果解释为完整实机部署通过。

## 构建和可重放基线

先构建，再运行测试；不要同时启动 Gazebo 与编译。

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 colcon build --symlink-install \
  --packages-select px4_powerplant_mission_cpp px4_powerplant_mission \
  --executor sequential --parallel-workers 1 \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release
bash validation/baseline/run_baseline.sh
```

入口先运行真实双进程话题/服务探针，再串行执行 C++、Python/C++ 对照、跨进程契约和 Python 回归。失败以非零状态退出，不跳过通信失败。结果保存在 `log/p0_baseline/` 与 `build/px4_powerplant_mission_cpp/test_results/`，可在同一提交/配置下重放比较。

此入口也已纳入后续 P1 的射线/采样、增量索引、视场/时间预算、实际覆盖与视点确认测试；脚本名称保留。新参数及融合观测元数据见 [P1_IMPLEMENTATION.md](P1_IMPLEMENTATION.md)。

可重放输入覆盖：有效采样后重复/乱序/退化、冻结接收、坐标 epoch 重置与迟到旧 epoch、地图指纹不匹配、相对高平台起飞、起飞柱体障碍、各运动阶段失去导航、降落超时和 landed/disarmed 确认。跨进程重放用合成 ROS/PX4 消息，不启动 Gazebo、Agent、真实飞控或任何电机。

本轮实际构建/测试数量和剩余验收见 [MIGRATION_STATUS.md](MIGRATION_STATUS.md)。完整飞行仍需按原项目步骤验证 UWB→EV→PX4→导航→地图→任务链、飞控降级行为、动力学跟踪和真实传感器标定。
