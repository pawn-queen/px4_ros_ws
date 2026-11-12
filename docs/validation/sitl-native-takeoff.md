# SITL 原生位置起飞 GUI 验证（ADR-0004）

2026-11-12。本记录对应一次**本机带 GUI（非无头）仿真**的完整闭环：原生位置起飞 → 实测悬停处严格交接 → 初扫 → 探索 → 返航降落，全程无 fault。它是 ADR-0004 路径在本机的可复现验证，**不是**隔离验收（`run_acceptance.py`）的替代：隔离验收仍受资产基线与 1.5 m 审计柱体约束，见文末“范围与限制”。

## 结论

- 任务链完整走通：`SAFE_TAKEOFF → HOVER_SCAN → EXPLORE → RETURN_HOME → LANDING → COMPLETE`，总时长 261.3 s，`health_fault_reason` 为空。
- 原生起飞到 5 m 悬停用 14.4 s；`px4_takeoff_strict_hold_ack` 在**实测悬停处**完成，随后进入 `HOVER_SCAN`（不是地面预测交接）。
- 初扫覆盖 0.944，用时 46.8 s；探索 188.2 s（22 条 `safety_stop`，约每 18 s 一次近距减速）；返航 0 s 触发、降落 11.9 s 触地。
- 结束地图 222,572 free / 27,494 occupied 体素，`map_localization_healthy=true`。

## 阶段时间线

| 阶段 | 起始 (s) | 时长 (s) | 关键事件 |
|---|---:|---:|---|
| SAFE_TAKEOFF | +0.0 | 14.4 | `px4_takeoff_initialized` → `px4_takeoff_hover_reached`（z≈4.94 m）→ `px4_takeoff_strict_hold_ack` |
| HOVER_SCAN | +14.4 | 46.8 | `initial_scan_rotation_started`（+17.3）→ `initial_scan_complete`（+61.2，覆盖 0.944） |
| EXPLORE | +61.2 | 188.2 | 前沿规划航点并实飞；`safety_stop` 22 条 |
| RETURN_HOME | +249.4 | ~0 | `return_home`（探索储备触发） |
| LANDING | +249.4 | 11.9 | `landing` → 触地 z=-0.01 |
| COMPLETE | +261.3 | — | `mission_complete` |

关键事件（完整 33 条见工作区 `log/sim_session_20261112_180153/timeline3.jsonl`）：

| 本地时间 | sim t | 阶段 | 事件 | z (m) | 关键字段 |
|---|---:|---|---|---:|---|
| 22:57:50 | +0.0 | SAFE_TAKEOFF | `px4_takeoff_initialized` | -0.01 | px4=WAITING_FOR_CONTROL |
| 22:58:05 | +14.3 | SAFE_TAKEOFF | `px4_takeoff_hover_reached` | +4.94 | px4=READY_FOR_STRICT_HOLD |
| 22:58:05 | +14.4 | SAFE_TAKEOFF | `px4_takeoff_strict_hold_ack` | +4.94 | cert=AIRBORNE_STRICT |
| 22:58:05 | +14.4 | HOVER_SCAN | `phase_changed` | +4.94 | — |
| 22:58:52 | +61.2 | HOVER_SCAN | `initial_scan_complete` | +4.96 | coverage=0.944 |
| 22:58:52 | +61.2 | EXPLORE | `phase_changed` | +4.95 | — |
| 23:02:00 | +249.4 | RETURN_HOME | `return_home` | +4.95 | — |
| 23:02:00 | +249.4 | LANDING | `landing` | +4.95 | — |
| 23:02:12 | +261.3 | COMPLETE | `mission_complete` | -0.01 | — |

## 复现配置

| 文件 | 作用 |
|---|---|
| `config/sitl_native_takeoff.yaml` | `motion_config_file`：`px4_takeoff_enabled=true`、严格域容差 0.15、柱体 3.0 m×6.5 m、预算 45 s，关闭地面支撑/响应同步/过渡预算 |
| `config/sitl_gui_validation.yaml` | `config_file`：在 `config/powerplant.yaml` 基础上 UWB 接受门限 8.0/4.0 m²、`uwb_recent_timeout_s=1.0`、mapper `max_publish_points=600000`/`publish_rate_hz=1.0`、`motion_input_timeout_s=1.0`/`max_localization_age_s=1.0` |

启动方式（与 README 的 SITL 流程一致，只替换两个配置）：

```bash
ros2 launch px4_powerplant_mission powerplant_system.launch.py \
  config_file:=src/px4_powerplant_mission/config/sitl_gui_validation.yaml \
  motion_config_file:=src/px4_powerplant_mission/config/sitl_native_takeoff.yaml
```

轨迹触发器仍为 `/start_powerplant_mission`；事件流 `/powerplant/control/inspection_events`，状态 `/powerplant/control/mission_status`。

## 本轮代码与验证侧改动

- `mission_state_machine.cpp`：新增两条节流诊断日志（`scan gate` 逐子条件、`scan segment ... reason=` 逐方向计划结果）。它们定位了“初扫段运动计划越出确认柱体”这一真实阻塞，保留为长期诊断。
- 地面阶跃响应标定（见 [起飞响应说明](../design/takeoff-response-sync.md)）：`validation/sitl/ground_response.py`、`validation/sitl/calibrate_ground_response.py`、冻结配置 `config/p2_sitl_ground_response.yaml`，以及 `run_acceptance.py --ground-response-profile` 把策略表接到冻结文件。
- `run_acceptance.py` 的主机适配：`--px4-instance`（本机 `/tmp/px4-sock-0` 属其他用户）、GCS 期望端口随实例偏移、显式 `MAV_SYS_ID=1`。

## 证据

- 工作区：`log/sim_session_20261112_180153/`（`timeline3.jsonl`、`TIMELINE_native_takeoff.md`、`motion_native_takeoff.yaml`、`powerplant_native.yaml`、`ros_launch.log`、`px4.log`、`screenshots/`）。
- 截图：`screenshots/native_hover_gazebo.png`（5 m 悬停）、`screenshots/explore_running_gazebo.png`（探索实飞）、`screenshots/native_scan_rviz.png`（初扫体素图）。

## 范围与限制

- 本记录是**本机 GUI 仿真**的完整闭环，不是隔离验收的正式通过。`takeoff_domain_audit` 里“1.5 m 上限”的根因是**证据棱柱写死为 `[-2,2]×[-2,2]×[0.04,6.3]`**（`validation/model/takeoff_audit.py`），不是场景几何：把棱柱扩到 `±4 m` 后，半径 3.0 m（甚至 3.5 m）在 `(0,0)` 的网格/包围盒/种子检查全部通过。也就是说，本会话为容纳严格停止盒使用的 3.0 m 柱体**在场景上是可审计的**，缺的是把棱柱与证据记录一起扩大；该改动不在本 PR 内，须单独评估并重生成审计证据。
- `sitl_native_takeoff.yaml` 的柱体为 3.0 m × 6.5 m；若按现有棱柱（z 上限 6.3 m）审计，柱体高度应取 6.2 m（仍满足 `5.0+0.3+0.5=5.8` 的下界）。
- UWB/时延相关参数是本机定位节点的抖动适配（悬停实测里程计 30 Hz、最大间隔 40 ms；偶发 >0.5 s 抖动曾触发 `MOTION_NAVIGATION_STALE`），不是控制律变更。
- 资产树与本机模型仓库存在本地定制（雷达高度、相机命名空间），隔离验收的资产钉住与传感器话题契约尚未对齐；以正式验收为准时须先对齐资产基线。
- 未修改 PX4 源码、默认 ROI、场景碰撞或硬件默认参数；原生起飞路径在硬件默认仍关闭。
