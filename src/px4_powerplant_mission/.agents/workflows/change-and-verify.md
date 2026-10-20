# 改动与验证流程

适用于本包后续的代码、配置、launch 和文档改动。完整的环境准备、PX4 参数、启动顺序及故障排查见 [README](../../README.md)。

## 1. 确认改动范围

从工作区根目录检查状态，记录已有改动，随后沿输入话题、处理节点、输出话题找到受影响的链路：

```bash
cd /home/pawn/px4_ros_ws
git status --short -- src/px4_powerplant_mission
```

常见链路如下，表中的 Python 路径相对于 `px4_powerplant_mission/`：

| 任务 | 主要文件 | 关键输出 |
| --- | --- | --- |
| 定位与外部视觉 | `nodes/localization_node.py`、`nodes/uwb_simulator_node.py`、`nodes/external_vision_bridge_node.py` | `/uwb/pose`、`/powerplant/localization/odom`、`/fmu/in/vehicle_visual_odometry` |
| 建图与规划 | `nodes/voxel_mapper_node.py`、`mapping/voxel_grid.py`、`path_planning/grid_astar.py` | `/powerplant/map/occupancy_grid`、`/powerplant/control/planned_path` |
| 检测与任务 | `nodes/yolo_detector_node.py`、`nodes/offboard_mission_node.py` | `/powerplant/perception/yolo_detections`、`/powerplant/control/mission_status` |

同时核对 `config/powerplant.yaml`、相关 launch 文件和 `setup.py`。不要仅凭节点内默认参数判断实际启动配置。

## 2. 实施改动

1. 保持 `common/frames.py` 的 ENU/FLU ↔ NED/FRD 约定；修改订阅或发布时核对 `common/qos.py`。
2. 参数改动同步检查节点声明与 YAML；入口、资源或话题改动同步检查 launch、`setup.py` 和 README。
3. 定位或控制改动保留独立定位检查、飞行前条件、setpoint 预热、限速与避障逻辑；涉及这些行为时增加能验证行为的测试或仿真检查。

## 3. 验证

修改纯计算逻辑后运行包内测试；新增计算行为时补充有意义的测试：

```bash
cd /home/pawn/px4_ros_ws
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src/px4_powerplant_mission \
  python3 -m pytest -p no:cacheprovider -q src/px4_powerplant_mission/test/test_core_math.py
```

修改 Python 节点、参数、launch 或打包配置后，用系统 Python 构建本包：

```bash
cd /home/pawn/px4_ros_ws
deactivate 2>/dev/null || true
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select px4_powerplant_mission \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

涉及 ROS 话题、PX4 接口或飞行行为时，再按 README 的顺序启动 DDS Agent、PX4/Gazebo 和任务栈做仿真验证。先在默认配置下检查 Gazebo 真值、`/uwb/pose`、定位和 external vision 是否持续发布，再按需要显式启用 `use_yolo:=true`、`use_control:=true`。控制节点启动后默认仍不会执行任务；只有需要验证完整飞行流程时才调用 `/start_powerplant_mission`，并观察 `/powerplant/control/mission_status` 与安全状态。传感器话题缺失时按 README 检查 `gz_sensor_bridge.launch.py`。

最后检查 `git diff --check -- src/px4_powerplant_mission`，报告验证结果及尚未覆盖的运行场景。
