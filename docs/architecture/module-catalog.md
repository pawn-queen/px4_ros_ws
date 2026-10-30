# 模块目录：职责、语言与运行状态

> **路径约定**：仓库级路径写作 `src/px4_powerplant_mission/...` 或 `src/px4_powerplant_mission_cpp/...`；包内短路径 `config/…`、`launch/…`、`nodes/…` 相对于 `src/px4_powerplant_mission/`，`src/…`、`include/…`、`test/…`、`scripts/…` 相对于 `src/px4_powerplant_mission_cpp/`；`UWB/…`、`Host/…` 相对于 `C:\Users\pawn-\UWB-loaclization\`；`Tools/simulation/gz/…`、`ROMFS/…` 相对于 `/home/pawn/drone-simulation/`。

> 证据基线：2026-10-30。Python mapper/controller 已删除；行数为当前文件行数（`wc -l`），用于判断模块体量，不代表质量。
> 标签：`[事实]` / `[推断]` / `[建议]` / `[待核实]`。
> 相关文档：[总览](README.md) · [仓库映射](repository-map.md) · [运行模式](runtime-modes.md) · [依赖图](dependency-map.md) · [架构债务](architecture-debt.md)

## 1. 状态图例

| 状态 | 含义 |
|---|---|
| **默认** | `powerplant_system.launch.py` 默认启动 |
| **可选** | launch 参数显式打开（YOLO/控制/传感器桥/RViz） |
| **仿真专用** | 只在 SITL/Gazebo 路径使用 |
| **实机专用** | 只在 hardware profile 使用 |
| **已删除** | 历史实现已移除，仅保留在 git 历史/冻结 fixture 中 |
| **外部** | 不在主工作区，需单独构建/部署 |

## 2. Python 模块（`px4_powerplant_mission`）

| 模块 | 行数 | 语言 | 职责 | 被谁使用 | 状态 | 证据 |
|---|---:|---|---|---|---|---|
| `common/frames.py` | 209 | Python | NED/FRD ↔ ENU/FLU、四元数、yaw、地理坐标转换 | 所有节点 | 共享 | `common/frames.py:1-12` |
| `common/qos.py` | 36 | Python | PX4（best-effort/transient-local）、传感器、可靠话题的 QoS | 所有节点 | 共享 | `common/qos.py:11-35` |
| `localization/inputs.py` | 168 | Python | UWB 输入契约：frame/协方差/时间/重复/健康事件/坐标变换 | localization、EV 桥 | 共享 | `localization/inputs.py:1-5,80-168` |
| `localization/uwb.py` | 334 | Python | Anchor 解析、测距模拟、鲁棒三边/多边解算、质量 | UWB 仿真器、Host | 共享 | `localization/uwb.py:12-29,135-265` |
| `localization/fusion.py` | 161 | Python | 导航侧自适应融合/倾角修正；不进入 EV | localization（`adaptive`） | 旧实现/可选 | `localization/fusion.py:1-6`；`localization_node.py:51-53` |

`[事实]` Python 节点只通过 `common/`、`localization/` 组合；`mapping/` 与 `path_planning/` 已随 legacy mapper/controller 删除。见 `nodes/*.py` 的 import 段。

## 3. Python 节点

| 节点/入口 | 行数 | 默认 | 主要输入 | 主要输出 | 职责 | 证据 |
|---|---:|---|---|---|---|---|
| `localization_node` | 777 | **默认** | `/fmu/out/vehicle_imu`、`vehicle_local_position`、`vehicle_odometry`、`/uwb/*`、`/depth_camera` | `/powerplant/localization/odom`、`/pose`、`/status`、TF | PX4 完整位姿 → ENU/FLU 导航；UWB 健康/独立位置检查 | `localization_node.py:43-139,238-314,486-525` |
| `external_vision_bridge_node` | 121 | **默认** | `/uwb/pose`、`/uwb/quality`、`/uwb/status`（硬件为 `/uwb/hardware/*`） | `/fmu/in/vehicle_visual_odometry` | 独立 UWB → PX4 EV；单 Tag 不伪造姿态/速度 | `external_vision_bridge_node.py:60-106` |
| `uwb_simulator_node` | 493 | **默认（仿真）** | Gazebo 真值 `/world/powerplant/dynamic_pose/info`、`/model/.../odometry_with_covariance`、PX4 local position | `/uwb/pose`、`/uwb/quality`、`/uwb/range/<anchor>`、`/uwb/anchors` | 逐锚测距 + 解算 + 质量 | `uwb_simulator_node.py:232-299,319-390` |
| `yolo_detector_node` | 204 | 可选（`use_yolo`） | `/camera` | JSON 检测、标注图、源图 | Ultralytics 推理与检测框 | `yolo_detector_node.py:21-56,126-192` |

`[事实]` C++ `voxel_mapper_node` 与 `offboard_mission_node` 现在是唯一建图/控制入口；Python 版本及 `mapping_package`/`control_package` 回退参数已删除。

## 4. C++ 库与可执行文件（`px4_powerplant_mission_cpp`）

### 4.1 构建产物

| 产物 | 类型 | 源文件 | 依赖 | 职责 | 证据 |
|---|---|---|---|---|---|
| `powerplant_core` | 静态库 | `voxel_grid.cpp`、`frontier_planner.cpp`、`planning_index.cpp`、`motion.cpp`、`topology.cpp` | Eigen3（无 ROS） | 纯算法核心：体素、前沿、规划索引、运动、拓扑 | `CMakeLists.txt:23-25` |
| `powerplant_mission` | 静态库 | `map_snapshot.cpp`、`offboard_node.cpp`、`offboard_subscriptions_*.cpp`、`offboard_control.cpp`、`mission_state_machine.cpp`、`mission_planning.cpp`、`inspection_recorder.cpp`、`inspection.cpp`、`motion_runtime.cpp`、`offboard_p2_*.cpp` | `powerplant_core`、rclcpp、px4_msgs、ROS 消息、cv_bridge、OpenCV、JsonCpp、Threads | ROS 集成层 + 任务状态机 + 巡检 | `CMakeLists.txt:26-29` |
| `voxel_mapper_node` | 可执行 | `voxel_mapper_node.cpp` | `powerplant_core`、ROS、OpenCV、JsonCpp | 建图节点 | `CMakeLists.txt:30-33` |
| `offboard_mission_node` | 可执行 | `offboard_main.cpp` + `powerplant_mission` | 同上 | 任务控制节点 | `CMakeLists.txt:34-36` |

`[事实]` `powerplant_core` 的 `target_link_libraries` 只有 `Eigen3::Eigen`（`CMakeLists.txt:25`），不依赖 ROS/OpenCV；这是本项目最好的模块边界之一。`powerplant_mission` 则把建图消费、任务状态机、巡检、运动运行时全部放在一个库里（`CMakeLists.txt:26`），并通过 `-O1` 控制 WSL 编译内存（`:28`）。

### 4.2 头文件职责

| 头文件 | 职责 | 证据 |
|---|---|---|
| `core.hpp` | 基础类型、`VoxelGrid`、`PoseHistory`、`ExplorationConfig`、`PlanningMap`、`YawScan`、`SlewState` | `core.hpp:20-37,76-121,216-238` |
| `raycast.hpp` / `depth_sampling.hpp` | 精确体素遍历；深度自适应采样 | `raycast.hpp:11`；`depth_sampling.hpp:15-30` |
| `planning_index.hpp` | 不可变前沿/净空增量索引 | `planning_index.hpp:8-21` |
| `frontier_planner.cpp` | 已知空闲路径、前沿视点评分 | `frontier_planner.cpp:352,806` |
| `topology.hpp` / `topology.cpp` | 三维区域/门户分解与配额 | `topology.hpp:6-56`；`topology.cpp:56-323` |
| `motion.hpp` / `motion.cpp` | 五次轨迹、停止包络、证书 | `motion.hpp:82-149`；`motion.cpp:280-574` |
| `motion_runtime.hpp` / `motion_runtime.cpp` | 独立 50 Hz P/V/A 运行时 | `motion_runtime.hpp:9-23`；`motion_runtime.cpp:91-275` |
| `map_snapshot.hpp` / `map_snapshot.cpp` | 点云/状态时间戳配对、指纹、异步提交 | `map_snapshot.hpp:25-82`；`map_snapshot.cpp:5-107` |
| `inspection.hpp` / `inspection.cpp` | RGB-D 注册、表面片、照片覆盖 | `inspection.hpp:34-142`；`inspection.cpp:263-625` |
| `inspection_recorder.cpp` | 图像编码/写盘（ROS 定时器外） | `inspection_recorder.cpp:8-173` |
| `scan_coverage.hpp` | 实际深度方向覆盖 | `scan_coverage.hpp:8-25` |
| `worker.hpp` | 规划任务、取消、陈旧结果拒绝 | `worker.hpp:11` |
| `input_validation.hpp` | 采样时间/协方差检查 | `input_validation.hpp:20-35` |
| `parameters.hpp` / `ros_support.hpp` | 参数声明、QoS、JSON、点云工具 | `parameters.hpp:6-80`；`ros_support.hpp:95-145` |
| `offboard.hpp` | `OffboardMissionNode` 状态与接口 | `offboard.hpp:86-205` |

`[事实]` 这些 P2 头文件已在 `4800616b` 进入仓库；当前未提交的是本次 Python legacy 删除与 fixture 迁移。见 [D1](architecture-debt.md#d1-本次删除改动未提交)。

## 5. launch、配置与脚本

| 文件 | 类型 | 用途 | 关键点 | 证据 |
|---|---|---|---|---|
| `launch/powerplant_system.launch.py` | Python launch | 启动定位/EV/时钟/真值/UWB/建图/YOLO/控制 | 默认 8 个节点条件；profile 切换；PX4 话题版本后缀 | `:12-151` |
| `launch/gz_sensor_bridge.launch.py` | Python launch | Gazebo 图像/深度/雷达/点云 → ROS | 6 组 bridge；默认长 RGB 话题名 | `:13-58` |
| `launch/visualization.launch.py` | Python launch | RViz + 预置配置 | `use_sim_time=true` | `:10-27` |
| `config/powerplant.yaml` | YAML | 共享参数总表 | 话题、UWB anchors、建图、控制、巡检、坐标 | `:1-378` |
| `config/localization_simulation.yaml` | YAML | 仿真 UWB 源/姿态开关 | `/uwb/pose`，允许真值姿态 | `:1-13` |
| `config/localization_hardware.yaml` | YAML | 实机 UWB 源/姿态开关 | `/uwb/hardware/*`，禁止姿态 | `:1-13` |
| `config/p2_simulation.yaml` | YAML | SITL 相机/雷达外参、机体包络、起飞 bootstrap | `motion_model_configured=true` | `:1-26` |
| `config/p2_hardware.yaml` | YAML | 实机运动参数占位 | `motion_model_configured=false`，bootstrap=0 | `:1-10` |
| `scripts/run_p0_baseline.sh` | Bash | 串行测试基线：探针 + C++ + Python | `ROS_DOMAIN_ID=173`，Cyclone | `:1-25` |
| `scripts/p0_ros_env.sh` | Bash | 当前 shell 的 Cyclone/LOCALHOST 环境 | 不写 `.bashrc` | `:1-16` |
| `scripts/bootstrap_p0_ros.py` | Python | 无 root 解包锁定 Cyclone 包 | 仅 noble amd64 | `:16-60` |
| `scripts/run_p2_sitl.py` | Python | 启动隔离的 Agent/Gazebo/PX4/GCS/传感器/任务 | 独立 domain/port/rootfs | `:249-329` |
| `scripts/p2_sitl_gcs.py` | Python | 只读 MAVLink GCS 心跳/参数证据 | 不发送解锁/模式/轨迹 | `:23-83` |
| `scripts/derive_sitl_body_envelope.py` | Python | 从 SDF 推导机体包络 | 生成包络审计证据 | `research_sources/sitl_body_envelope.md` |
| `test/worlds/p2_*.sdf` | SDF | P2 验收世界 | room / two_rooms | `p2_room.sdf`、`p2_two_rooms.sdf` |

`[事实]` `src/px4_powerplant_mission/launch/powerplant_system.launch.py:16-24` 用已安装 `px4_msgs` 的 `MESSAGE_VERSION` 生成 `/fmu/out/*_vN` 重映射；直接 `ros2 run` 节点时不会自动加后缀，必须手工匹配固件（`src/px4_powerplant_mission/README.md:409`）。

## 6. UWB Host（外部包 `uwb_hardware_bridge`）

以下文件名除特别标注外，均相对于 `Host/uwb_hardware_bridge/`。

| 模块 | 行数 | 职责 | 证据 |
|---|---:|---|---|
| `cli.py` | 74 | `check-config`、`live`、`replay` 命令入口 | `Host/setup.py:23-26` |
| `config.py` | 136 | 不可变硬件配置、锚点 ID/ENU、时间/噪声/几何门限、SHA-256 | `config.py:16-80` |
| `protocol.py` | 154 | ASCII 行帧、CRC32、RANGE v1/v2 字段、常量 | `protocol.py:111`；`Host/README.md:54-58` |
| `processing.py` | 285 | 时钟区间、来源/轮次/年龄/批跨门控、调用共享解算器 | `processing.py:107-249` |
| `session.py` | 592 | 单串口所有权、AT 事务、重连、状态机、交付 | `session.py:1-40` |
| `recording.py` | 39 | JSONL 记录（原始分片、命令、结果） | `Host/README.md:58-63` |
| `replay.py` | 130 | 离线回放，不打开串口 | `Host/README.md:42-44` |
| `ros_node.py` | 189 | 4 个发布者、2 个服务、`use_sim_time=false` 强制 | `ros_node.py:79-104` |
| `solver.py` | 26 | 唯一解算器加载 + 源码哈希 | `solver.py:1-20` |
| `tests/*` | 5 文件 | 假串口/PTY/同进程 ROS 测试 | `Host/README.md:95-111` |

`[事实]` Host 不包含第二份三边定位实现；`processing.py` 调用 `solve_uwb_ranges()`，缺失时拒绝输出（`processing.py:110-119,234-249`）。`[事实]` 它发布 4 个话题，主工作区只消费 3 个；`/uwb/hardware/observations` 是诊断用途。

## 7. UWB 固件（`UWB/`，STM32F103 + DW3000）

| 层/模块 | 行数 | 语言 | 职责 | 证据 |
|---|---:|---|---|---|
| `App/Src/app.cpp` | 53 | C++17 | 初始化、主循环、LED 心跳 | `app.cpp:16-53` |
| `App/Src/uwb_service.cpp` | 539 | C++17 | AT 命令、角色/信道配置、RANGE v1/v2、配置存储、状态机 | `uwb_service.cpp:227-360,422-470,491-510` |
| `BSP/Src/board.c` | 490 | C | 板级时钟/GPIO/SPI/IRQ/USART、LED | `board.c:1-40` |
| `BSP/Src/dw3000_port.c` | 343 | C | DW3000 SPI/EXTI/HAL 适配 | `dw3000_port.c:245,340` |
| `BSP/Src/serial_transport.c` | 133 | C | USART1 中断队列/发送 | `serial_transport.c:1-40` |
| `BSP/Src/config_store.c` | 193 | C | 双页 CRC 配置存储 | `config_store.c:1-40` |
| `BSP/Src/crc32.c` | 26 | C | CRC32 | `crc32.c` |
| `Middleware/DW3000/Src/deca_device.c` | 4950 | C | 厂家 DW3000 驱动 | `middleware-sources.json` |
| `Middleware/AiThinker/Src/vendor_backend.c` | 470 | C | 厂家 TWR 单步包装/状态 | `vendor_backend.c` |
| `Middleware/AiThinker/Src/vendor_tag.c` | 124 | C | Tag 回调 | `vendor_tag.c` |
| `Middleware/AiThinker/Src/vendor_provenance.c` | 215 | C | 来源/构建可追溯 | `vendor_provenance.c` |
| `Middleware/AiThinker/Lib/twr-gcc.a` / `twr.lib` | — | 二进制 | 厂家 TWR 库（GCC/Keil） | `middleware-sources.json` |

`[事实]` BU03/BU04 共享 `App/`、`BSP/`、`Middleware/`，差异在 `Boards/BU03|BU04/` 的 CubeMX 生成配置（BU03 启用 I2C2、天线形态不同）；两者同为 STM32F103C8T6、64 KiB Flash/20 KiB RAM（`UWB/README.md:3`；`UWB/Boards/BU03/board_profile.h`、`BU04/board_profile.h`）。

`[事实]` 固件默认 `WAIT_START`，`AT+SETCFG` 会启动测距（`uwb_service.cpp:305,507`）；默认 RANGE 版本为 v1（`:496`），而 Host 只接受 v2（`processing.py:153`；`Host/README.md:54`）。`[推断]` 每块参与定位的板都必须刷同一版固件并完成 `AT+RANGEVER=2` 握手，否则 Host 会持续拒绝。

`[事实]` 固件不做三维解算/PDoA：`AT+GETDEV` 返回 `pos_enable:0,pos_dimen:0`（`uwb_service.cpp:307`），设置位置解算返回 `ERR,POSITION_SOLVER_NOT_INCLUDED`（`:327`）。三维坐标由 Host 调用主工作区解算器得到。

## 8. PX4 / Gazebo 侧

| 文件 | 作用 | 证据 |
|---|---|---|
| `ROMFS/.../airframes/4002_gz_x500_depth` | `x500_depth` 机型参数：EV、无 GPS/磁、Offboard 门控 | `:11-24` |
| `ROMFS/.../airframes/4001_gz_x500` | `PX4_GZ_WORLD`、`PX4_SIM_MODEL`、机体参数 | `:17-18` |
| `Tools/simulation/gz/worlds/powerplant.sdf` | `powerplant` 世界（地面、厂房/目标模型） | `:3,16-17,253-254` |
| `Tools/simulation/gz/models/x500_depth/model.sdf` | OakD-Lite、`mapping_lidar`（360°/15 Hz/35 m） | `:4-17,29-54` |
| `Tools/simulation/gz/models/x500_base/model.sdf` | `base_link` 位于模型原点上方 0.24 m | `:4` |
| `Tools/simulation/gz/models/power-plant/` | 目标模型/网格 | `powerplant.sdf:253-254` |

`[事实]` PX4 侧没有任务栈源码；它是运行时/仿真依赖（见 [repository-map.md](repository-map.md#8-依赖映射)）。`[推断]` 主工作区的 P2 相机/雷达外参与 Gazebo 模型强耦合，任何模型偏移改动都会改变建图/控制输入。

## 9. 模块规模与语言汇总

| 语言/层 | 主要文件数 | 说明 |
|---|---:|---|
| Python 节点/库 | 4 节点 + 5 库模块 | 定位/UWB/EV/YOLO |
| C++ | 21 `.cpp` + 16 `.hpp` | 2 库 + 2 可执行 |
| C/C++ 固件 | App 2 + BSP 5 + Middleware 4 + 厂家驱动 | STM32F103，GCC/Keil 双构建 |
| Python Host | 9 模块 + 5 测试文件 | 串口协议/时间/记录/ROS |
| SDF | `powerplant` world + `x500_depth` + `power-plant` + 2 个测试世界 | Gazebo 资源 |

`[事实]` 当前最大的单文件是 C++ `motion.cpp`（1140 行）；Python 侧最大文件为 `localization_node.py`（777 行）。修改高复杂度核心时应优先依赖现有测试/契约。
