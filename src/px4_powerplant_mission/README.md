# px4_powerplant_mission

面向 `PX4_GZ_WORLD=powerplant make px4_sitl gz_x500_depth` 的 ROS2 Jazzy Python 仿真任务包。它不包含硬件驱动，UWB 为仿真数据源，雷达/深度相机/相机话题通过 Gazebo bridge 或已有 ROS 话题接入。当前工程已在 `drone-simulation/Tools/simulation/gz/models/x500_depth/model.sdf` 中给 `x500_depth` 增加了 360 度平面 `mapping_lidar`，原启动命令不需要改。

## 包结构

- `nodes/localization_node.py`：订阅 PX4 IMU、local position、odometry、深度图和 UWB 位姿，统一输出 ROS ENU/FLU 下的 `/powerplant/localization/odom`。
- `nodes/uwb_simulator_node.py`：用仿真真值位置和 YAML 中的 UWB anchor 生成 range，并发布 `/uwb/pose`。
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

## 运行顺序

先构建工作区：

```bash
cd /home/pawn/px4_ros_ws
deactivate 2>/dev/null || true
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

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

默认不会启动 YOLO 和飞控。需要时显式打开：

```bash
ros2 launch px4_powerplant_mission powerplant_system.launch.py use_yolo:=true use_control:=true
```

飞控节点启动后仍不会自动执行任务，默认需要服务触发：

```bash
ros2 service call /start_powerplant_mission std_srvs/srv/Trigger
ros2 service call /land_powerplant_mission std_srvs/srv/Trigger
```

`/start_powerplant_mission` 会进入完整巡检流程：

1. `START`：记录当前 ENU 位置为 home，重置目标、航点、照片计数和状态。
2. `TAKING_OFF`：发布 PX4 Offboard 位置 setpoint，起飞到 `takeoff_height_m`。
3. `GLOBAL_SEARCH`：沿 `mission_waypoints_enu` 全局搜索；YOLO 在 `/camera` 上检测 `target_class_name`，飞控节点结合 `/depth_camera` 和 `/depth_camera/camera_info` 把 bbox 中心深度反投影成 ENU 三维坐标，并发布 `/powerplant/perception/target_position`。
4. `TARGETING_CYCLE`：围绕目标生成 `inspection_orbit_points` 个巡检拍照点，机头持续朝向目标；到达航点且 yaw 误差小于 `yaw_acceptance_rad` 后保存照片到 `inspection_photo_dir`，并在 `/powerplant/control/inspection_events` 发布拍照事件。
5. `RETURN_HOME`：拍照完成或搜索超时后规划回 home。
6. `LANDING` / `COMPLETE`：发送 PX4 land command，落地或超时后标记任务完成。

避障路径规划使用 `voxel_mapper_node.py` 输出的 `/powerplant/map/occupancy_grid`。该地图由深度相机、`mapping_lidar` 雷达和点云融合成体素/占据栅格，再由 `offboard_mission_node.py` 调用 2D A* 规划全局搜索、目标靠近和返航路径；如果当前 occupancy grid 还不可用，节点会退化为直连航点。

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
- `/powerplant/localization/status`：定位状态 JSON
- `/uwb/pose`、`/uwb/range/<anchor>`：仿真 UWB 结果
- `/powerplant/map/occupied_voxels`：3D 体素点云
- `/powerplant/map/occupancy_grid`：给路径规划使用的 2D 占据栅格
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

如果某台机器临时仍想用虚拟环境，只覆盖启动参数即可：

```bash
ros2 launch px4_powerplant_mission powerplant_system.launch.py use_yolo:=true yolo_python:=/path/to/venv/bin/python
```
