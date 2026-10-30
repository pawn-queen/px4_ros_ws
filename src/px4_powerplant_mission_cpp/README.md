# 原生 powerplant 建图与任务控制

本包是 ROS 2 Jazzy 下的 C++17 体素建图、保守三维规划和 Offboard 控制实现。Python mapper/controller 已于 2026-10-30 删除；Python 包现在只提供定位、UWB、外部视觉桥和 YOLO 感知。本包不内嵌或执行 Python 节点。

当前结果与剩余集成检查见 [迁移状态](../../docs/history/migration-status.md)。验证、SITL 和模型审计已迁到仓库根目录的 [validation](../../validation/README.md)；本包只保留 runtime 代码、runtime 配置和 runtime 测试。

## 阶段说明

P0 的输入健康、地图健康、起飞区确认、落地确认和可重放 ROS 检查见 [P0 验证](../../docs/validation/input-map-health.md)。
P1 的建图效率、前沿和观测流程改进见 [P1 实现](../../docs/design/mapping-and-frontier.md)。
P2 的运动证书、区域引导、目标表面和照片证据见 [P2 实现](../../docs/design/motion-and-inspection.md)；P3 仍是后续工作。

P2 软件审计覆盖采样时刻参考历史、HOLD 新鲜确认、规划器/执行器共享的不确定性预算、可执行局部前缀、精确停止/时间记账、区域身份和曝光绑定照片覆盖。历史审计的 316 个直接回归用例见 [P2 验证](../../docs/validation/sitl-acceptance.md)。飞行参数和物理验收独立于这些软件检查；修改安全/标定参数后应在地面重新启动，运行中热切换物理安全模型不在本接口范围内。

## 局部任务 ROI 与电厂碰撞资产

实际电厂场景使用单独简化的静态凹碰撞网格，同时保留原始视觉 STL。资产提交和内容哈希锁定真实场景验收入口。`explore_min/max_*` 定义任务收益和覆盖范围；起飞、初扫、安全接入和返航使用完整观测导航域。EXPLORE 内含受约束的 ROI_ACCESS 和 ROI_WORK 子模式，不要求已知航点。背景见 [Issue #3 实现](../../docs/design/issue3-collision-and-roi.md) 和 [实际验证状态](../../docs/validation/issue3-actual-plant.md)。

起飞响应更新把启动参考和实测升力分开，加入早停认证制动和恢复锁存，并在启动前派生一个不可变的 SITL 地面过渡预算。独立控制器 profile 必须用实际 PX4 参数回读验证后才能解锁；它不修改 PX4 源码或硬件默认值。策略见 [起飞响应设计](../../docs/design/takeoff-response-sync.md)，实际飞行范围见 [Issue #3 验证](../../docs/validation/issue3-actual-plant.md)。

本轮 A–D 过渡策略使用固定 45 秒预算、圆角扫掠包络和有界参考选择，并保留原严格策略的原子恢复。20 个实际策略/积分候选均未通过起飞初扫；5 次获得严格 ACK，但后续严格爬升/恢复失败，均未进入 HOVER_SCAN，未冻结控制器 profile。主飞行、governor 消融和 ROI 验收仍未执行。最新记录见 [验证记录](../../docs/validation/issue3-actual-plant.md)。

## 构建

在工作区根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 colcon build --symlink-install \
  --packages-up-to px4_powerplant_mission --executor sequential --parallel-workers 1 \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release
```

两个限制都是刻意的：`colcon` 的并行 worker 数不会限制 Make 的编译进程。任务传输库用 `-O1` 限制 WSL 上的编译内存，数值核心保留所选构建类型的优化级别。内存受限的机器上不要同时编译、测试和运行 Gazebo。

依赖声明在 `package.xml`：rclcpp、工作区的 px4_msgs、ROS 消息包、cv_bridge/OpenCV、Eigen3 和 JsonCpp。不需要新增 Python 推理依赖。

## 模块

- `input_validation.hpp`：ROS 采样时间与完整位姿协方差检查。
- `core.hpp`、`voxel_grid.cpp`：概率更新、2D 投影、带时间戳位姿历史、初始偏航扫描和 setpoint 斜率限制。
- `raycast.hpp`、`depth_sampling.hpp`：正长度体素遍历，以及按深度/边缘自适应采样，带每帧射线预算。
- `planning_index.cpp`：不可变增量前沿和净空缓存。
- `motion.cpp`、`motion_runtime.cpp`：有界五次轨迹、连续停止包络和独立 50 Hz 位置/速度/加速度流。
- `topology.cpp`：三维已知安全区域/门户分解和区域名额。
- `inspection.cpp`：配准的 RGB/深度观测、稳定表面片、质量评估和事务式照片覆盖。
- `frontier_planner.cpp`：六邻域路径、已知空闲净空、带标定相机边界的视点评分、转向/停留时间和已知空闲返航预算；失败时给出明确原因。
- `voxel_mapper_node.cpp`：有界传感器队列、相机标定匹配、逐束雷达运动补偿和地图发布。
- `map_snapshot.cpp`：点云/状态时间戳精确配对、字段/数量检查、观测状态分类、有界分类变化历史和新旧检查。
- `offboard_node.cpp`、`offboard_subscriptions_*.cpp`：ROS 接口与状态。
- `mission_state_machine.cpp`：显式任务阶段和转换条件。
- `scan_coverage.hpp`：实际融合深度朝向覆盖和相机几何；仅完成名义偏航步进不能授权探索。
- `mission_planning.cpp`、`worker.hpp`：不可变规划任务、有界工作队列、协作取消和过期结果拒绝。
- `offboard_control.cpp`：PX4 heartbeat、NED setpoint、命令、预检/解锁顺序、斜率限制和障碍物保护。
- `inspection_recorder.cpp`：ROS 定时器外的图像编码和磁盘写入。

任务 ROS 回调只有一个状态写入者。规划 worker 只处理复制的 map 快照；控制路径的每个安全检查不会复制 map。取消不会在 ROS 回调里等待规划线程；节点有序退出时会 join worker 线程。

## 接口兼容

可执行文件仍是 `voxel_mapper_node` 和 `offboard_mission_node`；节点名仍是 `powerplant_voxel_mapper` 和 `powerplant_offboard_mission`。核心 ROS 话题/类型、服务名、QoS、ENU/FLU→NED 约定和四字段地图布局保留。P0 增加严格定位/地图健康、`VehicleLandDetected`、锁定 `FAULT` 阶段和显式起飞区参数；P1 增加成功深度相机/观测元数据和覆盖/规划原因诊断。契约测试检查保留的默认值和有意变更的 `require_image_for_photo=true`。启动条件和中 middleware profile 见 [P0 验证](../../docs/validation/input-map-health.md)，新增参数见 [P1 实现](../../docs/design/mapping-and-frontier.md)。

正常探索默认要求实际匹配的 CameraInfo；P2 巡检还要求真实 RGB 标定和 YOLO 源曝光。硬件运动参数必须显式配置。`/powerplant/map/observed_voxels` 仍使用四个 float32 字段（`x,y,z,state`）；未知/缺失和不确定（`state=-1`）单元不能承载飞行路径，截断快照不能授权探索。launch 给定位、建图、YOLO 和控制传入同一个 `use_sim_time`。消息头使用 ROS 时间；墙钟定时器和单调接收时限也能检测仿真时钟冻结。

Python/C++ 的定位状态和 YOLO 检测仍是 `String` JSON，但字段和版本已收进 [mission contract](../../docs/design/mission-contract.md)：新消息写 `contract_version=1`，旧字段保留；两端常量和契约测试共同防止静默漂移。

C++ 的算法平局是确定性的（体素键字典序），所以完全相等的候选优先级可能选择与 Python set 迭代不同的等价视点。轨迹检查已知空闲净空，不要求路径消息逐字节相同；体素发布顺序不是接口保证。数值计算使用 double，地图线格式使用 float32，与已退役的 Python 线格式一致。

图像保存已改为异步；照片事件和计数按后续控制周期的写入结果更新。只有新图像成功写入后才计数。落地完成需要新鲜的 PX4 landed 和 disarmed；超时保持未确认。无效导航会锁定 `FAULT` 并停止 heartbeat/setpoint，让配置的 PX4 Offboard-loss 行为接管；恢复不会自动重新解锁。

## 验证

构建完成后先跑完整可复现基线：

```bash
bash validation/baseline/run_baseline.sh
```

单独检查时，先 source 同一套中间件环境：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source tools/env/ros_env.sh
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 CTEST_PARALLEL_LEVEL=1 \
  colcon test --packages-select px4_powerplant_mission_cpp --executor sequential \
  --ctest-args -j1 --output-on-failure
colcon test-result --verbose
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  PYTHONPATH=src/px4_powerplant_mission:$PYTHONPATH /usr/bin/python3 -m pytest \
  -p no:cacheprovider -q src/px4_powerplant_mission/test
```

validation 的离线测试单独运行：

```bash
python3 -m pytest -p no:cacheprovider -q \
  validation/tests/test_powerplant_assets.py \
  validation/tests/test_powerplant_sensors.py \
  validation/tests/test_powerplant_probes.py \
  validation/tests/test_ground_support_audit.py
```

完整的 validation 套件（含 DDS 集成）由 `validation/baseline/run_baseline.sh` 运行。

runtime 测试覆盖几何、时间历史、地图契约、增量索引、自适应采样、标定可见性、转向/停留预算、扫描覆盖、视点确认、规划依赖、setpoint 限制和取消。测试专用 `core_probe` 通过 JSON 输入/输出验证原规划不变量和随机障碍路线。历史规划对照显式中和新的转向/停留成本；P1 原生测试覆盖生产模型。精确射线更新有意不同于已退役 Python 固定步长采样，并用独立线段几何 oracle 检查。ROS 契约测试一次只启动一个原生进程，注入合成传感器和 PX4 消息，验证参数默认值、发布和服务；定位/EV 测试还会为两个 profile 启动独立 Python 进程。这些测试不能替代完整 PX4/Gazebo 飞行验收。

起飞地面接触保护的三阶段、独立地面模型、时间域和严格证书交接见 [docs/design/takeoff-ground-support.md](../../docs/design/takeoff-ground-support.md)。硬件默认严格模式；当前实际电厂主链仍未通过跟踪验收，结果以 [docs/validation/issue3-actual-plant.md](../../docs/validation/issue3-actual-plant.md) 为准。失败后落地上锁不能替代自主任务完成。
