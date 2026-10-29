# 贡献流程（CONTRIBUTING）

本仓库使用 GitHub Flow：`main` 是唯一长期维护分支，**任何功能代码都不直接推 `main`**。

## 1. 从 `main` 创建分支

```bash
git switch main
git pull --ff-only
git switch -c feat/uwb-stale-sample
```

分支名前缀见 [BRANCHING.md](BRANCHING.md)。一个分支只做一件事；如果开发中发现了别的问题，**开 Issue**，不要顺手塞进当前分支。

## 2. 提交

```bash
git add -p
git commit -m "fix: reject UWB samples older than max_navigation_age_s"
```

- 提交信息格式：`<type>: <subject>`，`type ∈ feat | fix | refactor | docs | chore | ci | test | perf`
- 一条提交只做一件事，便于 `git revert`
- 不要提交 `build/`、`install/`、`log/`、`__pycache__/`、模型权重或任何凭据

## 3. 关联 Issue

```bash
git commit -m "fix: reject stale UWB samples

Closes #8"
```

- `Closes #N` / `Fixes #N`：合并时自动关闭 Issue
- `Refs #N`：只引用，不关闭

## 4. 创建 PR

```bash
git push -u origin feat/uwb-stale-sample
gh pr create --draft --fill
```

PR 模板里的每一项都要填。**"验证结果"表格不允许留空**，跑不了就写"未运行"并说明原因。

## 5. 本地测试

### 纯逻辑测试（不需要 ROS，几秒钟）

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src/px4_powerplant_mission \
  python3 -m pytest -p no:cacheprovider -q \
  src/px4_powerplant_mission/test/test_core_math.py \
  src/px4_powerplant_mission/test/test_frontier_3d.py \
  src/px4_powerplant_mission/test/test_uwb_quality_solver.py
```

### 构建

```bash
source /opt/ros/jazzy/setup.bash
colcon build \
  --packages-up-to px4_powerplant_mission_cpp px4_powerplant_mission \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
```

### C++ 测试（gtest，15 个目标 / 约 223 个用例）

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon test --packages-select px4_powerplant_mission_cpp \
  --executor sequential --ctest-args -j1 -R "^test_" --output-on-failure
colcon test-result --test-result-base build/px4_powerplant_mission_cpp --verbose
```

### 仿真链路

```bash
source /opt/ros/jazzy/setup.bash
bash scripts/ci/sitl_smoke.sh 180
```

只验证 PX4 SITL + Gazebo + uXRCE-DDS + ROS 2 是否连通。任务级验收请用
`src/px4_powerplant_mission_cpp/scripts/run_p2_sitl.py`，两者的结论不能互相代替。

### 完整基线（较慢，跑之前先确认机器空闲）

```bash
bash src/px4_powerplant_mission_cpp/scripts/run_p0_baseline.sh
```

## 6. Review

Review 关注四件事，按顺序：

1. **这个 PR 是否只做了一件事**
2. **验证结果是否真实**：静态检查通过 ≠ 仿真通过 ≠ 实机可用
3. **影响面是否被诚实标注**：offboard 链路、topic/QoS/坐标系、实机安全
4. **回滚是否可行**

发现问题的处理方式：

- 能让作者当场改的小问题 → 在 PR 里留 comment
- 需要额外工作的独立问题 → **开 Issue**，在本 PR 用 `Refs #N` 引用，不要扩大本 PR 范围

## 7. 合并

- 合并方式：**Squash and merge**（迁移类结构性 PR 除外，见该 PR 说明）
- 合并后删除分支：`gh pr merge --squash --delete-branch`
- 合并前确认：CI 绿、Review 意见已解决、Issue 已关联

## 8. CI 失败怎么办

```bash
gh pr checks                      # 看哪些检查失败
gh run list --limit 5             # 找到对应 run
gh run view <run-id> --log-failed # 只看失败步骤的日志
```

- 修代码 → 推新提交，CI 自动重跑
- 确认是 CI 环境问题（不是代码问题）→ 开 Issue 记录，在 PR 里说明，不要用 `continue-on-error` 掩盖
- **不允许**为了让检查变绿而删除检查、放宽断言或写 `echo success`

## 9. 回滚错误更改

```bash
# 尚未合并：直接在分支上 revert 或修正
git revert <sha>

# 已合并进 main：在独立分支上 revert，再走一次 PR
git switch -c fix/revert-<topic>
git revert <merge-or-squash-sha>
git push -u origin fix/revert-<topic>
gh pr create --fill
```

**禁止**用 `git reset --hard` + `git push --force` 回滚 `main`：Ruleset 会拒绝，且会破坏其他人的克隆。
