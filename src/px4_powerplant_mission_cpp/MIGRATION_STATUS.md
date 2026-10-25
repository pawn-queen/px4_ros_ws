# C++ 迁移进度与验证记录

更新：2026-10-25。此文件用于后续继续任务时保持范围与验证结论一致。

## 2026-10-25：P1 建图效率与观测有效性

本轮实施用户确认的 **P1 全部项目**，保留 P0 的健康门控、坐标 epoch、起飞柱体与落地确认。原理变化、公式、参数和限制集中见 [P1_IMPLEMENTATION.md](P1_IMPLEMENTATION.md)。P2/P3 尚未实施。

- 射线改为按体素面推进的精确 DDA，仅正长度交段提供空闲证据；保留批次去重和命中优先，不用碰撞 supercover 激进清空地图。
- 深度按距离投影足迹、深度/有效性边缘细分，选择真实像素并按图像分区分配射线预算。未观测像素不填充覆盖；遍历深度图的 CPU 成本仍需测量。
- 地图分类差分驱动前沿与净空局部重算；不可变分片索引供 worker 共享，保留有限版本变化历史。完整地图传输、解析和差分仍有全图成本。
- 成功融合的实际 CameraInfo、主点与尺寸提供非对称矩形视锥；缺省要求真实匹配标定。每个候选 yaw 单独检查转向、观察停留、去返路径时间和预算。
- HOVER_SCAN 以实际合格深度射线方向覆盖、稳定状态及已知空闲首段共同退出。固定 yaw 步数、融合计数不能代替覆盖证据；缺失深度时只在确认柱体内慢偏航，雷达失鲜时停止转动。
- 探索视点须对准、停留并取得新合格深度才完成，按位置/yaw 记忆；附近几何变化使记忆失效。重规划分类区分地图等待、标定、定位差、路径受阻、余量不足、局部无候选与前沿耗尽。局部无候选扩大一次搜索，但候选截断不等同于全图完成。
- 异步结果按 epoch、相机/配置、收益区域及完整去返走廊验证；远处变化允许复用，采用前复查最新路径、定位预测与预算。

本轮实际验证（串行，无 Gazebo/飞控）：

| 检查 | 结果 |
| --- | --- |
| 两个包 Release 单编译任务构建 | 通过 |
| C++ 原生测试 | **62 项通过**：原 P0 35、P1 射线/采样 8、增量索引 6、规划 7、任务 6 |
| 几何与算法对照 | **26 项通过**；精确射线使用独立几何参考，旧规划对照显式中和新转向/停留成本 |
| 跨进程 ROS 契约 | **7 项通过**；增加实际相机/观测元数据断言，保留 simulation/hardware 定位与 EV |
| Python 包回归 | **113 项通过** |
| 同机 DDS 话题/服务探针 | 通过，继续使用临时 Cyclone DDS 配置 |

上述为 **208 个直接测试用例**。`colcon test-result` 的 105 项还包括 10 个 CTest 外层结果，不是额外行为用例。射线测试包含 200 组独立线段/AABB 几何对照；增量索引包含随机全量参考对照与旧快照不变性；初扫包含旧帧、倾角/质量拒绝、缺失深度及雷达失鲜；视点测试验证不能仅靠计数完成观测。

复现命令仍为 `scripts/run_p0_baseline.sh`，名字为保留原入口，CMake 已纳入 P1 全部测试。结构化结果位于 `build/px4_powerplant_mission_cpp/test_results/` 和 `log/p0_baseline/`。

**边界：**当前相机模型要求已校正 pinhole 深度与 CameraInfo.K 匹配；未来视点假设水平飞行，未增加任意安装旋转。路径仍为离散已知空闲几何路径，未实现连续机体碰撞证明、动力学制动或动态障碍预测。未完成完整 PX4/Gazebo 飞行、真实传感器标定及大型地图/YOLO 并行负载下的尾延迟测量，不据此宣称性能加速比或实飞验收通过。下方 P0 记录保留当时结果。

## 2026-10-25：P0 输入、地图与任务状态改进

已修改并构建 C++ 建图/控制与共享 Python 定位状态接口。仿真和实机共用这些实现，launch 对定位、建图、YOLO、控制统一传入 `use_sim_time`。以下为 P0 完成时记录；P1 后续进展见上节。

- 新增共享采样时间/六维协方差检查，采样与接收年龄分开，另用单调时钟检测流停止和仿真时钟冻结。重复、乱序旧包不刷新健康；同时间戳的质量退化仍撤销有效状态。
- 定位状态新增时间、序号和坐标 epoch；PX4 估计器 reset、时钟回退和节点重启不能静默拼接旧地图/home。mapper 质量不合格时冻结积分，普通延迟包不再触发历史清图。
- 新地图健康协议包括 epoch、序号、定位与传感器采样时间及点云指纹；保留四字段点云，但旧健康协议不能授权原生控制器飞行。
- 全阶段运动前检查导航、独立定位、飞控状态和预算。不可恢复的本次任务故障锁定为 FAULT，取消任务并停止 heartbeat/setpoint；恢复数据不会自动重新解锁，由已配置的 PX4 Offboard-loss 行为接管。
- 起飞服务要求新鲜导航、飞控已上锁、已确认且足够大的起飞柱体；起飞/返航高度改为相对实际 home。缺省确认关闭、高度为零，不能直接拿示例值当作物理标定。
- 落地必须由命令之后的新鲜 landed 与 disarmed 状态持续确认。超时保留 LANDING 并报告未确认。无新鲜图像、无图 fallback 或写盘失败都不再完成巡检照片计数。

本机 Fast DDS 的原始故障仍表现为单向发现、无数据/服务响应，回环 UDP/显式 locator/TCP 对照未解决，根因未确定。采用**仅当前 shell 生效的 Cyclone DDS + LOCALHOST 配置**后，同机双进程话题与服务恢复，原生地图/控制及 Python 定位/EV 的跨进程验证已通过。无 sudo 安装权限时，锁定 SHA256 的五个 ROS 包已验证后提取到被 git 忽略的 `.p0_deps/`；没有修改系统网络、`.bashrc` 或永久 DDS 默认设置。

本轮验证结果：

| 检查 | 结果 |
| --- | --- |
| 两个包串行构建（Release，单编译任务） | 通过 |
| C++ 原生测试 | **35 项通过**（任务/几何 21、共享输入 4、地图健康 6、mapper 健康 4） |
| Python/C++ 原算法对照 | **26 项通过** |
| 跨进程 ROS 契约 | **7 项通过**：参数 2、建图 1、Offboard/落地 1、旧包重放故障 1、simulation/hardware 定位与 EV 2 |
| Python 包回归 | **113 项通过** |
| 双进程 ROS 探针 | 话题接收和服务应答均通过；结构化结果保存至 `log/p0_baseline/transport.json` |
| 输入重放/故障注入 | 旧包、质量退化、接收冻结、epoch 重置/迟到、点云错配、起飞柱体、全阶段定位故障与降落确认均有用例 |

跨进程定位测试实际启动独立的 localization、EV 和注入/断言进程，分别使用 simulation/hardware profile；检查独立 UWB→EV、PX4→完整导航，停止 UWB 后导航协方差退化且新 PX4 状态不产生 EV 反馈。这里使用合成 ROS 消息，没有连接 Host 串口硬件。

运行入口、依赖锁定、配置及新协议见 [P0_VALIDATION.md](P0_VALIDATION.md)。`scripts/run_p0_baseline.sh` 串行运行探针、C++/对照/跨进程测试和 Python 回归并保存结果；不会启动 Gazebo、Agent、真实飞控或电机。下方 2026-10-24/21 的“未通过”是历史结果，本轮仅在上述新 middleware 配置下解决同机通信；不能据此断言默认 Fast DDS 已修好。

**仍待验收：**完整 PX4/Gazebo 飞行、Agent/PX4 与所选 DDS 的实际互通、真实 UWB/相机/雷达标定及故障数据、起飞顶棚净空、PX4 Offboard-loss 配置和反应、连续动力学制动与真实电量裕量。LOCALHOST 基线不验证 Windows Host 或跨计算机 DDS。原来的 Python mapper/controller 可保留算法对照，但不具备本轮原生 P0 行为；混用旧地图协议会被拒绝。

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
