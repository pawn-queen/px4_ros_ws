# C++ 迁移进度与验证记录

更新：2026-10-24。此文件用于后续继续任务时保持范围与验证结论一致。

## 2026-10-24：仿真/实机共用定位链路

本次修改 Python 定位、EV、共享输入检查、配置与 launch；C++ 建图/控制代码及消息接口未改。下方 C++ 迁移记录仍是 2026-10-21 的历史验证结果。

- `localization/uwb.py` 的解算接口保持不变，Host 继续复用；没有用示例锚点冒充实测标定。
- EV 直接订阅独立 UWB 位姿，取消融合里程计输入，保留采样时间、位置协方差并去重；单 Tag 默认不发布姿态/速度。仿真可显式允许独立真值姿态。
- 默认导航使用 PX4 完整位姿，转换完整四元数、协方差和 twist，保留状态采样时间。`fusion.py` 与旧 adaptive 导航模式保留，结果不进入 EV。
- 共用 frame/坐标变换、质量与健康事件处理；UWB 失效使导航协方差退化，PX4 位姿过期停止发布，时钟回退清缓存。
- 新增 `localization_profile:=simulation|hardware`，hardware 默认关闭 UWB 仿真器、Gazebo 真值/时钟桥和仿真时间。默认控制与 YOLO 仍关闭。
- 实机锚点坐标、航向、map/PX4 对齐与天线安装偏移仍需标定。配置方法与已知边界见 [定位部署说明](../px4_powerplant_mission/LOCALIZATION_DEPLOYMENT.md)。

本次实际验证：

| 检查 | 结果 |
| --- | --- |
| 系统 Python / colcon 构建 Python 包 | 通过 |
| Python 包完整回归 | **112 项通过** |
| 同进程真实 ROS Executor，仿真姿态/硬件单 Tag 两种输入 | 通过，收到 EV 与导航话题，检查无融合里程计反馈 |
| 实际 Host 的 `solution_to_pose` / `quality_payload` 接口 | 通过；保留原始时间、完整位置协方差与未知姿态标记 |
| launch 参数及 simulation/hardware 默认开关 | 通过 |
| 两节点跨进程消息联调 | **未通过**：hardware 默认 DDS 和 simulation 临时 UDP transport 均能发现 2 个 UWB、1 个 PX4 订阅者，但 15 秒内收不到导航或 EV 消息 |
| git diff 空白检查 | 通过 |

跨进程问题与既有 DDS 阻塞一致；没有更改永久网络/DDS 配置，也不能由此断言故障根因。由于跨进程数据路径未恢复，本次没有完成 PX4/Gazebo 全链路飞行验收；没有连接真实飞控或 UWB 串口设备。不能将 112 项测试解释为实飞验证通过。下一步仍需先修复 DDS，再验收 Agent/PX4 和真实传感器数据路径。

复现本次回归：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  PYTHONPATH=src/px4_powerplant_mission:$PYTHONPATH /usr/bin/python3 -m pytest \
  -p no:cacheprovider -q src/px4_powerplant_mission/test
```

## 已实施（2026-10-21 C++ 迁移）

- 独立 `ament_cmake` C++17 包：体素概率建图、采样时刻位姿插值、三维已知空闲路径与前沿视点规划。
- Offboard 已拆分为 ROS 接口、地图快照接收、任务状态机、规划调度、PX4 控制与照片保存模块。
- 规划和保存照片分别使用单工作线程，队列最多保留一个待执行任务；过期规划结果不能直接替换当前路线。
- 保留节点名、话题/消息类型、QoS、ENU/NED 转换、状态字符串、JSON 字段及原 YAML。参数声明对应原建图器 47 项、控制器 104 项；其 ROS 参数服务验证仍待通过，见下文。
- 保留独立定位检查、姿态门控、setpoint 预热、解锁检查、限速及已知空闲路径检查。地图云和融合状态按时间戳精确配对，不完整地图不能授权探索。
- `powerplant_system.launch.py` 默认把建图、控制路由到 C++ 包；`use_control=false`、`use_yolo=false` 和 YAML `auto_start=false` 保持不变。
- 原包仅修改 launch 接线、package.xml 依赖及 README；原 Python 节点、算法和 YAML 均未修改或删除。

Python 回退（可分别指定任一模块）：

```bash
ros2 launch px4_powerplant_mission powerplant_system.launch.py \
  mapping_package:=px4_powerplant_mission control_package:=px4_powerplant_mission \
  use_control:=true
```

## 已验证

| 检查 | 结果 |
| --- | --- |
| 两个包的系统 Python / colcon 串行构建 | 通过 |
| 原 Python 包测试 | 95 项通过 |
| C++ 原生测试 | 17 项通过 |
| Python/C++ 算法对照 | 26 项通过 |
| launch 参数检查 | 默认选择 C++，保留 Python 回退开关 |
| 默认 C++ 两节点加载原 YAML、短时运行、SIGINT 退出 | 通过；未触发任务，无 PX4/Gazebo |
| Python 回退启动 | 确认启动原 Python 两节点；退出时出现原实现重复 `rclpy.shutdown()` 异常，遵照保留原程序要求未修改 |

原生测试包括位姿插值/时间回退、点云双精度/字节序/行填充、地图完整性与新鲜度、异步任务取消、起飞门控、初扫超时、目标抢占、返航安全、限速、电量预算，以及照片保存期间地图失效后的完成处理。

对照测试复用原三维规划的 9 个公开接口场景（不包含 2 个内部可见性辅助函数测试），另有 8 组随机体素射线、8 组随机障碍路径和 1 组命中优先级/配置校验。没有据此宣称所有输入完全等价或给出性能加速倍数。

## 未通过与待验收

`ros_contract` 的 4 项跨进程 ROS 测试尚未通过：两个节点的参数服务响应、建图消息接收、Offboard 消息与服务接收均出现超时。独立的最小 Python 发布/订阅程序也收不到消息；本地 UDP 回环和 ROS 定时器能工作。默认 DDS、localhost、静态回环 peer 和临时 UDP 回环配置的诊断未解决问题，根因仍需定位。没有修改机器的永久 DDS/网络设置。

因此，当前不能宣称 ROS 接口联调全部通过。下一步先恢复本机跨进程 ROS 通信，再运行 `ros_contract`，处理其可能揭示的接口差异。随后按原 README 完成 DDS Agent、PX4/Gazebo、定位与外部视觉、深度/雷达建图、初扫、探索、巡检拍照和返航降落的完整仿真验收。本次未启动完整 Gazebo 飞行，也未测量大型地图下的控制周期延迟。

等分候选在 C++ 中按体素键确定顺序，可能选择与 Python 集合遍历不同但同等评分的视点；照片事件在异步写入结束后的控制周期发布。这两项已知差异详见 README。

## 继续验证的命令与资源约束

始终先完成编译再测试；不要同时运行编译、测试和 Gazebo。

```bash
cd /home/pawn/px4_ros_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 colcon build --symlink-install \
  --packages-select px4_powerplant_mission_cpp px4_powerplant_mission \
  --executor sequential --parallel-workers 1 \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release

PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  CTEST_PARALLEL_LEVEL=1 colcon test --packages-select px4_powerplant_mission_cpp \
  --executor sequential --ctest-args -j1 --output-on-failure
colcon test-result --test-result-base build/px4_powerplant_mission_cpp --verbose

PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  PYTHONPATH=src/px4_powerplant_mission:$PYTHONPATH /usr/bin/python3 -m pytest \
  -p no:cacheprovider -q src/px4_powerplant_mission/test
```

ROS 接口测试默认使用隔离的 `ROS_DOMAIN_ID=173`，一次仅启动一个原生节点，注入合成传感器/PX4 消息，不连接真实飞行器。
