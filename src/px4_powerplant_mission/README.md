# px4_powerplant_mission

面向 `PX4_GZ_WORLD=powerplant make px4_sitl gz_x500_depth` 的 ROS2 Jazzy Python 仿真任务包。它不包含硬件驱动，UWB 为仿真数据源，雷达/深度相机/相机话题通过 Gazebo bridge 或已有 ROS 话题接入。当前工程已在 `drone-simulation/Tools/simulation/gz/models/x500_depth/model.sdf` 中给 `x500_depth` 增加了 360 度平面 `mapping_lidar`，原启动命令不需要改。

## 包结构

- `nodes/localization_node.py`：订阅 PX4 IMU、local position、odometry、深度图和 UWB 位姿，统一输出 ROS ENU/FLU 下的 `/powerplant/localization/odom`。
- `nodes/uwb_simulator_node.py`：用仿真真值位置和 YAML 中的 UWB anchor 生成 range，并发布 `/uwb/pose`。
- `nodes/voxel_mapper_node.py`：从深度图、`LaserScan`、`PointCloud2` 生成体素占据地图、2D occupancy grid 和 RViz marker。
- `nodes/yolo_detector_node.py`：接入 `/home/pawn/yolo` 的权重，发布 JSON 检测结果和标注图。
- `nodes/offboard_mission_node.py`：PX4 Offboard 位置控制、航点跟踪和基于 occupancy grid 的 2D A* 避障路径。
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
- `/fmu/in/trajectory_setpoint`：飞控节点给 PX4 的 Offboard 位置 setpoint

## YOLO 说明

默认模型已经放在包内的 `models/yolo/best.pt`，回退模型是 `models/yolo/yolov8n.pt`。当前 `best.pt` 检测类别见 `models/yolo/data.yaml`，已有类别为 `gas_cylinder`。

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
