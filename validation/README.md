# validation 模块

本目录只放验证、审计、标定和 SITL 编排，不放 runtime 节点。runtime 代码仍在 `src/px4_powerplant_mission_cpp/`；`validation/` 不参与 `colcon` 构建，也不会随 ROS package 安装。

## 目录

| 目录 | 内容 |
|---|---|
| `baseline/` | 可重复的本地回归入口 |
| `probes/` | 跨进程 ROS 传输探针 |
| `sitl/` | PX4/Gazebo SITL 验收、标定、GCS 和飞行证据 |
| `model/` | 机体包络、接触、起飞域和物理审计 |
| `sensors/` | 传感器样本与雷达方向验证 |
| `support/` | 评估 fixture 和 benchmark 汇总 |
| `evidence/` | 已冻结的标定/审计证据（JSON），由 `validation/` 脚本生成并入库 |
| `archive/` | 已冻结的历史分析，不应被运行代码 import |
| `tests/` | validation 自身的 pytest |

## 常用入口

```bash
# 本地完整回归：runtime 测试 + validation 测试
bash validation/baseline/run_baseline.sh

# PX4/Gazebo SITL 验收
python3 validation/sitl/run_acceptance.py --scenario powerplant --prepare-only \
  --output-dir log/validation_prepared

# 离线 validation 测试
python3 -m pytest -q \
  validation/tests/test_powerplant_assets.py \
  validation/tests/test_powerplant_sensors.py \
  validation/tests/test_powerplant_probes.py \
  validation/tests/test_ground_support_audit.py
```

## 边界

- `validation/tests/test_ros_contract.py` 需要真实 ROS/DDS 运行时，不进 GitHub 离线 CI。
- `validation/sitl/run_acceptance.py` 会启动 PX4/Gazebo/Agent/GCS，只应在隔离环境运行。
- `validation/archive/` 是历史分析，不参与 CI，也不被 runtime 或 validation 入口 import。
- `validation/evidence/` 是入库的冻结证据，不是 runtime 输入；修改它等于改验收基线，必须同时重跑生成脚本并更新对应 `docs/research/` 说明。

## PX4 位置模式起飞专项

按 `docs/adr/0004-px4-position-takeoff.md` 的边界，专项使用实际电厂、已审计柱体和临时 PX4 rootfs，不覆盖悬停推力或垂向积分。与旧 `calibrate_takeoff.py` 的 A–D 标定分开，不能把旧冻结配置传入新路径。

```bash
# 仅准备、展开配置及资源审计
bash scripts/ci/issue3_acceptance.sh takeoff-prepare

# 串行：3 次真实 5 m 起飞至严格 HOLD / HOVER_SCAN，接着漂移与超时各 1 次
bash scripts/ci/issue3_acceptance.sh takeoff
```

新路径命令行为：`run_acceptance.py --scenario powerplant --px4-position-takeoff --stop-after hover`。`hover` 在首次严格悬停交接后主动降落；不要求完成初扫、ROI 接入或返航。`--stop-after scan/mission` 仍保留对应后续阶段的建图与运动检查。

`position_takeoff_evidence.json` 记录固定位置、锁存偏航、P/V/A 指令语义、实际悬停锚点和严格 HOLD ACK；`physical_flight_evidence.json` 使用独立 Gazebo 真值与导航采样验证 5 m 和停稳。位置模式故意留空的速度、加速度、偏航速率在 ROS 消息中是 NaN，JSON 写为 null，`position_only_takeoff=true` 标明语义。完整 P/V/A 与原跟踪门只评价严格执行阶段；爬升期的完整时钟、新鲜度与 50 Hz 发布证据继续记录。

`--takeoff-fault drift` 仅把控制器的里程计输入切到测试转发话题，保留时间戳、协方差和姿态，在解锁后追加明确的横向偏移。建图、独立 UWB、EV 和真值不接收偏移；原始与注入样本分别保留。`timeout` 只在隔离配置中设置 8 s 固定期限。二者均要求实际观察到 `NAV_LAND` 指令、PX4 `AUTO_LAND` 状态及最终落地上锁；不能用测试端调用降落服务替代保护触发。

`hover` 只判定起飞专项，深度投影残差继续作为 `mapping_diagnostics` 记录，不能据此宣称初扫或建图已通过。漂移/超时保护触发很早，时钟与发布检查保留原间隔界，但仅要求至少一个有效间隔，报告不能当作完整巡航的长时间性能证据。
