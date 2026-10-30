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
