# ADR-0001：把 validation 从 runtime package 拆出来

- 状态：Accepted
- 日期：2026-10-30
- 相关 PR：[#33](https://github.com/pawn-queen/px4_ros_ws/pull/33)

## 背景

`px4_powerplant_mission_cpp` 同时包含 runtime 代码、17 个验证脚本、SITL runner、物理/传感器审计和 validation pytest。runtime package 的 CMake 文件还注册 validation 测试，并安装 validation 文档。结果是：

- Agent 无法从路径判断“生产代码”还是“验证工具”；
- runtime 的 build/test/install 面被 validation 污染；
- 修改验证脚本可能影响 runtime 的测试入口；
- P0/P2 阶段名混进脚本路径，难以判断职责。

## 决策

1. validation 成为仓库级 `validation/` module，不参与 `colcon` 构建，也不安装。
2. 环境辅助工具放在 `tools/env/`；CI 专用脚本留在 `scripts/ci/`。
3. validation 测试放在 `validation/tests/`；SITL 世界放在 `validation/sitl/worlds/`。
4. runtime CMake 只注册 runtime gtest 和 `reference_parity`。
5. CI 增加离线 validation 步骤；不把 SITL、DDS 或 Gazebo 放进 GitHub hosted CI。
6. `validation/baseline/run_baseline.sh` 作为本地完整回归入口。

## 后果

- runtime package 的安装面缩小；validation 不再随包安装。
- validation 可以用自己的节奏演进，但仍需在 `run_baseline.sh` 中与 runtime 一起回归。
- `validation/tests/test_ros_contract.py` 仍依赖 DDS，只走本地/带环境的 baseline，不进离线 CI。
- 历史 P0/P1/P2 文档保留在 `docs/design`、`docs/validation`、`docs/history`，但不再作为新路径前缀。

## 未采用

- 新建 `px4_powerplant_validation` ROS package：会增加 build/rosdep/ccache 面，当前不需要。
- 把所有 validation 都塞进 `simulation.yml`：会把 PX4/Gazebo 的分钟级成本和 DDS 不稳定性带进每次 PR。
