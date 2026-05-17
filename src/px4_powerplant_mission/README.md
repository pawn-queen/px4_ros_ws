# px4_powerplant_mission

面向 `PX4_GZ_WORLD=powerplant make px4_sitl gz_x500_depth` 的 ROS2 Jazzy Python 仿真任务包。它不包含硬件驱动，UWB 为仿真数据源，雷达/深度相机/相机话题通过 Gazebo bridge 或已有 ROS 话题接入。当前工程已在 `drone-simulation/Tools/simulation/gz/models/x500_depth/model.sdf` 中给 `x500_depth` 增加了 360 度平面 `mapping_lidar`，原启动命令不需要改。

## 包结构

- `nodes/localization_node.py`：订阅 PX4 IMU、local position、odometry、深度图和 UWB 位姿，统一输出 ROS ENU/FLU 下的 `/powerplant/localization/odom`。
- `nodes/external_vision_bridge_node.py`：把 `/powerplant/localization/odom` 转成 PX4 NED 下的 `/fmu/in/vehicle_visual_odometry`，供 PX4 EKF 室内解锁和 Offboard 位置控制使用。
- `nodes/uwb_simulator_node.py`：用 Gazebo `dynamic_pose` 真值和 YAML 中的 UWB anchor 生成 range，并发布 `/uwb/pose`。
- `nodes/voxel_mapper_node.py`：从深度图、`LaserScan`、`PointCloud2` 生成体素占据地图、2D occupancy grid 和 RViz marker。
- `nodes/yolo_detector_node.py`：加载包内 `models/yolo/best.pt`，发布 JSON 检测结果和标注图。
- `nodes/offboard_mission_node.py`：PX4 Offboard 状态机，负责起飞、YOLO 全局搜索、目标三维定位、对准拍照、避障返航。
- `config/powerplant.yaml`：所有话题名、坐标系、UWB anchor、建图分辨率、航点参数。
- `launch/powerplant_system.launch.py`：启动任务栈。
- `launch/gz_sensor_bridge.launch.py`：可选启动 Gazebo sensor 到 ROS2 的桥接。

## 坐标约定

PX4 uORB 使用 `NED/FRD`，本包内部和对外发布使用 ROS 常用的 `ENU/FLU`：

- ENU：`x=east, y=north, z=up`
- NED 到 ENU：`[north, east, down] -> [east, north, -down]`
- FRD 到 FLU：`[forward, right, down] -> [forward, -right, -down]`
- Gazebo `dynamic_pose`/odometry 是 ENU；`mission_waypoints_enu` 和 `uwb_anchors` 直接使用 Gazebo world 的 x/y/z。

## 运行顺序

先构建工作区：

```bash
cd /home/pawn/px4_ros_ws
deactivate 2>/dev/null || true
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

首次用室内定位飞行前，建议在 PX4 shell 或 QGC 参数页设置一次 PX4 室内参数。目标是：不要求 GPS/磁罗盘，把 ROS 室内定位作为 EKF external vision 输入：

```bash
param set EKF2_EV_CTRL 11
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

其中 `EKF2_EV_CTRL=11` 表示融合 external vision 的水平位置、高度和 yaw，不融合速度；当前仿真链路会把 Gazebo 真值姿态作为 UWB pose 的仿真 yaw 来源，避免把 PX4 自己的 yaw 再回灌给 EKF。如果把 `powerplant_uwb_simulator.publish_truth_orientation` 改成 `false`，对应把 `EKF2_EV_CTRL` 改为 `3`，只融合 external vision 的水平位置和高度。

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

这个 launch 会默认启动 Gazebo 真值桥接，UWB 默认使用 `/world/powerplant/dynamic_pose/info` 的模型位姿；`/model/x500_depth/odometry_with_covariance` 仍保留为可选 odometry 后备源。如果你已经在别处桥接同一 truth topic，可以加 `use_gazebo_truth_bridge:=false`。默认不会启动 YOLO 和飞控。需要时显式打开：

```bash
ros2 launch px4_powerplant_mission powerplant_system.launch.py use_yolo:=true use_control:=true
```

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
ros2 topic echo /uwb/pose --once --timeout 5 --no-daemon
ros2 topic hz /fmu/in/vehicle_visual_odometry
ros2 topic echo /fmu/out/estimator_status_flags px4_msgs/msg/EstimatorStatusFlags --once --timeout 5 --qos-reliability best_effort --no-daemon
ros2 topic echo /fmu/out/failsafe_flags px4_msgs/msg/FailsafeFlags --once --timeout 5 --qos-reliability best_effort --no-daemon
```

`estimator_status_flags` 里 `cs_ev_pos`、`cs_ev_yaw`、`cs_ev_hgt` 变成 `true` 后，再调用 `/start_powerplant_mission`。
如果不加 `--no-daemon` 时出现 `TimeoutError: [Errno 110] Connection timed out`，这是 ROS2 daemon
卡住，不是 topic 没有数据；先执行 `ros2 daemon stop` 或继续使用上面的 `ROS2CLI_ENABLE_DAEMON=0`。

当前 `powerplant` world 中 `/model/x500_depth/odometry_with_covariance` 可能只显示有 publisher 但不实际发布消息，所以不要把 UWB 卡在这个 topic 上。若 UWB 日志出现 `waiting for Gazebo dynamic pose truth`，优先检查 `/world/powerplant/dynamic_pose/info` 的 `ros_gz_bridge`；若 `/uwb/pose` 不发布，后面的 localization、external vision 和 PX4 EKF 都不会得到有效位置/航向。

当前链路默认要求 UWB/Gazebo 真值是“独立定位源”：`localization_node` 会把 UWB 新鲜度写入
`/powerplant/localization/status`，并在没有近期 UWB 时把 `/powerplant/localization/odom`
协方差抬高；`external_vision_bridge_node` 检测到高协方差后不会继续把 PX4 自己的估计回灌给
`/fmu/in/vehicle_visual_odometry`。因此如果 `/uwb/pose` 不发布，PX4 可能不会进入可控的室内
Offboard 状态，这是有意的保护。先修复 `/world/powerplant/dynamic_pose/info` 到 `/uwb/pose`
的真值链路，不要用 PX4 local position 当 UWB 真值源去绕过这个问题。

飞行过程中 `/powerplant/control/mission_status` 里有两个位置：`nav_target_enu` 是当前逻辑航点，
`setpoint_enu` 是经过限速后的实际 PX4 setpoint。正常情况下 `setpoint_enu` 会逐步靠近
`nav_target_enu`，不是瞬间跳到远端航点。

调用 `/start_powerplant_mission` 后，飞控节点会先等待起飞条件，不会立刻强行解锁。可用下面
的话题确认等待原因：

```bash
ros2 topic echo /powerplant/control/mission_status std_msgs/msg/String --once --timeout 5 --spin-time 10 --full-length --no-daemon
```

其中 `localization_ready` 必须为 `true`，`pre_flight_checks_pass` 应为 `true`；
`command_wait_reason` 会显示正在等待定位、preflight、Offboard setpoint 预热、或 PX4
进入 Offboard。满足条件后节点按顺序请求 Offboard，然后再请求 arm。

`/start_powerplant_mission` 会进入完整巡检流程：

1. `START`：记录当前 ENU 位置为 home，重置目标、航点、照片计数和状态。
2. `TAKING_OFF`：发布 PX4 Offboard 位置 setpoint，起飞到 `takeoff_height_m`。
3. `GLOBAL_SEARCH`：沿 `mission_waypoints_enu` 全局搜索；默认航点覆盖 Gazebo 搜索矩形 `x=-15..24 m, y=0..40 m`、高度 `5 m`，并额外经过 `(0, 40, 5)` 后回到 `(0, 0, 5)`。YOLO 在 `/camera` 上检测 `target_class_name`，飞控节点结合 `/depth_camera` 和 `/depth_camera/camera_info` 把 bbox 中心深度反投影成 ENU 三维坐标，并发布 `/powerplant/perception/target_position`。
4. `TARGETING_CYCLE`：围绕目标生成 `inspection_orbit_points` 个巡检拍照点，机头持续朝向目标；到达航点且 yaw 误差小于 `yaw_acceptance_rad` 后保存照片到 `inspection_photo_dir`，并在 `/powerplant/control/inspection_events` 发布拍照事件。
5. `RETURN_HOME`：拍照完成或搜索超时后规划回 home。
6. `LANDING` / `COMPLETE`：发送 PX4 land command，落地或超时后标记任务完成。

避障路径规划使用 `voxel_mapper_node.py` 输出的 `/powerplant/map/occupancy_grid`。该地图由深度相机、`mapping_lidar` 雷达和点云融合成体素/占据栅格，再由 `offboard_mission_node.py` 调用 2D A* 规划全局搜索、目标靠近和返航路径；如果当前 occupancy grid 还不可用，节点会退化为直连航点。当前规划后的可视化路径会发布到 `/powerplant/control/planned_path`。

## 可视化窗口

建议先关掉卡住的 ROS2 daemon，后续窗口命令都可以复用同一个终端环境：

```bash
cd /home/pawn/px4_ros_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 daemon stop
export ROS2CLI_ENABLE_DAEMON=0
```

### 栅格地图和规划路径

用 RViz2 看避障栅格、A* 规划路径、UWB anchor 和飞机位姿：

```bash
rviz2
```

RViz2 中把 `Fixed Frame` 设为 `map`，然后添加这些显示项：

- `Map`：topic 选 `/powerplant/map/occupancy_grid`
- `Path`：topic 选 `/powerplant/control/planned_path`
- `Odometry`：topic 选 `/powerplant/localization/odom`
- `MarkerArray`：topic 选 `/uwb/anchors`
- `MarkerArray`：topic 选 `/powerplant/map/voxel_markers`

如果 `/powerplant/control/planned_path` 还没有显示，先调用 `/start_powerplant_mission`，因为路径会在任务进入搜索、目标巡检或返航并生成导航目标时发布。

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

用 RViz2 查看建图后的点云：

```bash
rviz2
```

RViz2 中 `Fixed Frame` 设为 `map`，添加：

- `PointCloud2`：topic 选 `/powerplant/map/occupied_voxels`
- `MarkerArray`：topic 选 `/powerplant/map/voxel_markers`

如果 Gazebo 原始点云也已经桥接到 ROS，可以再添加 `PointCloud2` topic `/points`。当前 `voxel_mapper_node.py` 会把深度图、`mapping_lidar` 和 `/points` 融合后发布 `/powerplant/map/occupied_voxels`，所以通常看这个融合后的 topic 更直观。

## Gazebo 相机/深度图桥接

PX4 的 `/fmu/out/...` 话题由 `MicroXRCEAgent` 提供；Gazebo 传感器图像通常还需要 `ros_gz_bridge`。先查看 Gazebo topic：

```bash
gz topic -l | grep -E 'camera|depth|scan|points'
```

如果 Gazebo 里就是 `/depth_camera`、`/camera` 和 `/mapping_lidar`，可以直接启动：

```bash
ros2 launch px4_powerplant_mission gz_sensor_bridge.launch.py
```

如果 topic 是带 world/model 前缀的长名字，就覆盖 launch 参数，例如：

```bash
ros2 launch px4_powerplant_mission gz_sensor_bridge.launch.py \
  depth_gz_topic:=/world/powerplant/model/x500_depth/link/camera_link/sensor/StereoOV7251/depth_image \
  rgb_gz_topic:=/world/powerplant/model/x500_depth/link/camera_link/sensor/IMX214/image \
  scan_gz_topic:=/world/powerplant/model/x500_depth/link/mapping_lidar_link/sensor/mapping_lidar/scan
```

## 重要输出话题

- `/powerplant/localization/odom`：融合后的室内位姿，`nav_msgs/Odometry`
- `/fmu/in/vehicle_visual_odometry`：桥接给 PX4 EKF 的室内 external vision 位姿，`px4_msgs/VehicleOdometry`
- `/powerplant/localization/status`：定位状态 JSON
- `/uwb/pose`、`/uwb/range/<anchor>`：仿真 UWB 结果
- `/powerplant/map/occupied_voxels`：3D 体素点云
- `/powerplant/map/occupancy_grid`：给路径规划使用的 2D 占据栅格
- `/powerplant/control/planned_path`：当前任务阶段的规划路径，`nav_msgs/Path`
- `/powerplant/perception/yolo_detections`：YOLO JSON 检测结果
- `/powerplant/perception/target_position`：YOLO + 深度相机反投影得到的目标 ENU 三维坐标，`geometry_msgs/PointStamped`
- `/powerplant/control/mission_status`：任务状态机 JSON，包括 `phase`、航点、照片计数、目标坐标
- `/powerplant/control/inspection_events`：目标发现、目标坐标、拍照、返航、降落等事件 JSON
- `/fmu/in/trajectory_setpoint`：飞控节点给 PX4 的 Offboard 位置 setpoint

## YOLO 说明

默认模型已经放在包内的 `models/yolo/best.pt`，回退模型是 `models/yolo/yolov8n.pt`。当前 `best.pt` 检测类别见 `models/yolo/data.yaml`，已有类别为 `gas_cylinder`。

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
