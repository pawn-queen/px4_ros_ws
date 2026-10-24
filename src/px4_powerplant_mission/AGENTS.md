# px4_powerplant_mission 代理工作说明

本文件适用于 `src/px4_powerplant_mission/` 下的工作。开始修改前，先读本包的 `README.md` 和 [改动与验证流程](.agents/workflows/change-and-verify.md)，并检查 `git status --short`；保留已有的未提交修改。

## 项目约定

- 这是 ROS 2 Jazzy 的 `ament_python` PX4/Gazebo 仿真包。`setup.py` 注册六个节点入口，并安装 `config/`、`launch/`、`models/`、`rviz/` 等运行资源。
- 节点在 `px4_powerplant_mission/nodes/`；启动时的参数以 `config/powerplant.yaml` 为准，节点的 `_declare_parameters()` 只提供默认值。YAML 顶层键须与 launch 中的节点名一致。
- Gazebo 真值和本包内部使用 ENU/FLU；PX4 消息使用 NED/FRD。坐标转换集中在 `common/frames.py`，PX4 与传感器的 QoS 配置集中在 `common/qos.py`。改动消息链路时核对坐标、单位、时间戳、frame 和 QoS。
- 定位链路共用于仿真和实机：独立 UWB（仿真 `/uwb/pose`、Host `/uwb/hardware/pose`）直接进入 external vision；PX4 完整 odometry 经 localization 输出 `/powerplant/localization/odom` 供建图/控制。融合里程计禁止回灌 EV。独立 UWB 过期或故障时，定位提高协方差，外部视觉桥不重发旧观测。配置与坐标/时钟约定见 [定位部署说明](LOCALIZATION_DEPLOYMENT.md)。不要用 PX4 自身的位置估计冒充独立真值来绕过这道检查。
- 地图由深度图、`/mapping_lidar` 和可选点云生成；`/powerplant/map/observed_voxels` 与 `/powerplant/map/fusion_status` 按时间戳配对后供三维前沿探索，未知体素禁止用于飞行路径。2D `/powerplant/map/occupancy_grid` 仍用于初扫判据与辅助避障。目标位置由 YOLO 检测框、深度图、相机内参和定位位姿计算。
- `powerplant_system.launch.py` 默认不启用 YOLO 和控制；`auto_start=false`，任务由 `/start_powerplant_mission` 服务触发。改动 Offboard 时保留定位、飞行前检查、setpoint 预热、限速和避障保护，并针对改动验证。

## 修改与交付

- 新增或改名节点时同步检查 `setup.py`、launch 和 YAML；新增运行资源时检查 `setup.py` 的 `data_files`。
- 修改话题、参数、任务阶段或操作步骤时同步更新 `README.md`。YOLO 推理依赖单列于 `requirements-yolo.txt`；工作区构建使用系统 Python。
- 按 [改动与验证流程](.agents/workflows/change-and-verify.md) 做最小相关验证，并在交付时说明运行的命令、结果，以及未做的仿真验证。
