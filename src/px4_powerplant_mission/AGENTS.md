# px4_powerplant_mission 代理工作说明

本文件适用于 `src/px4_powerplant_mission/` 下的工作。开始修改前，先读本包的 `README.md` 和 [改动与验证流程](../../docs/agents/workflows/change-and-verify.md)，并检查 `git status --short`；保留已有的未提交修改。

## 项目约定

- 这是 ROS 2 Jazzy 的 `ament_python` PX4/Gazebo 仿真包。`setup.py` 注册六个节点入口，并安装 `config/`、`launch/`、`models/`、`rviz/` 等运行资源。
- 节点在 `px4_powerplant_mission/nodes/`；启动时的参数以 `config/powerplant.yaml` 为准，节点的 `_declare_parameters()` 只提供默认值。YAML 顶层键须与 launch 中的节点名一致。
- Gazebo 真值和本包内部使用 ENU/FLU；PX4 消息使用 NED/FRD。坐标转换集中在 `common/frames.py`，PX4 与传感器的 QoS 配置集中在 `common/qos.py`。改动消息链路时核对坐标、单位、时间戳、frame 和 QoS。
- 定位链路共用于仿真和实机：独立 UWB（仿真 `/uwb/pose`、Host `/uwb/hardware/pose`）直接进入 external vision；PX4 完整 odometry 经 localization 输出 `/powerplant/localization/odom` 供建图/控制。融合里程计禁止回灌 EV。独立 UWB 过期或故障时，定位提高协方差，外部视觉桥不重发旧观测。配置与坐标/时钟约定见 [定位部署说明](LOCALIZATION_DEPLOYMENT.md)。不要用 PX4 自身的位置估计冒充独立真值来绕过这道检查。
- 原生 P0 消费端同时要求定位状态的 `status_stamp_ns/status_sequence/localization_epoch` 与完整六维协方差；坐标重置不能沿用旧 home/地图，旧包不能刷新健康。C++ 任务新增锁定的 FAULT、显式起飞柱体和 landed+disarmed 确认；原 Python mapper/controller 已移除，建图与控制只使用 C++ 实现及其地图健康协议。修改相关接口时同步检查 [P0 验证入口](../../docs/validation/input-map-health.md)，使用同一个 `use_sim_time`，不要绕过健康门控。
- 原生 P1 使用精确射线、深度自适应采样、增量前沿/净空、实际相机视锥及转向/停留预算；初扫和探索视点须有实际合格深度证据。修改融合元数据、覆盖或异步结果校验时先读 [P1 实现说明](../../docs/design/mapping-and-frontier.md)，同步参数/YAML与对应测试；不能把固定扫描步数、旧融合计数或候选截断当作完成证据。
- 原生P2轨迹、独立确认柱体、区域引导与巡检证据接口见 [P2实现说明](../../docs/design/motion-and-inspection.md)。停止尾段独立于规划任务；地图失鲜与定位故障分别处理。硬件不复制SITL安装参数；照片必须绑定实际曝光、质量和面片证据，不以最后航点或数量宣布覆盖完成。
- 地图由深度图、`/mapping_lidar` 和可选点云生成；`/powerplant/map/observed_voxels` 与 `/powerplant/map/fusion_status` 按时间戳配对后供三维前沿探索，未知体素禁止用于飞行路径。2D `/powerplant/map/occupancy_grid` 仍用于初扫判据与辅助避障。目标位置由 YOLO 检测框、深度图、相机内参和定位位姿计算。
- `powerplant_system.launch.py` 默认不启用 YOLO 和控制；`auto_start=false`，任务由 `/start_powerplant_mission` 服务触发。改动 Offboard 时保留定位、飞行前检查、setpoint 预热、限速和避障保护，并针对改动验证。

## 修改与交付

- 新增或改名节点时同步检查 `setup.py`、launch 和 YAML；新增运行资源时检查 `setup.py` 的 `data_files`。
- 修改话题、参数、任务阶段或操作步骤时同步更新 `README.md`。YOLO 推理依赖单列于 `requirements-yolo.txt`；工作区构建使用系统 Python。
- 按 [改动与验证流程](../../docs/agents/workflows/change-and-verify.md) 做最小相关验证，并在交付时说明运行的命令、结果，以及未做的仿真验证。
