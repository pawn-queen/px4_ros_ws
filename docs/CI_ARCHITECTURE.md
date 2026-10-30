# CI 架构（CI_ARCHITECTURE）

## 两条流水线，两台机器

| | `ci.yml` | `simulation.yml` |
| --- | --- | --- |
| 运行位置 | GitHub-hosted `ubuntu-24.04` | 自托管 `[self-hosted, sim-heavy]`（`ubuntu-lab-sim`） |
| 触发 | 每个 PR、每次 push `main`、手动 | 手动 / PR 打 `run-sim` 标签 / push `main` |
| 能接触外部 PR 的代码 | 可以（VM 用完即毁） | **不可以** |
| 干什么 | 语法、纯逻辑测试、包元数据、ROS 2 构建、gtest | 构建、SITL + Gazebo 链路存活检查 |

拆开的原因不是"性能"，是**安全边界**：hosted VM 每次都是干净的、销毁即消失；自托管机器上有真实文件系统、内网和凭据，不能让它执行陌生人的代码。完整推理见 `.github/workflows/simulation.yml` 顶部的注释。

## ROS 2 / PX4 / Gazebo 链路

`ci.yml` 走的是：

```
ros-tooling/setup-ros (Jazzy) → rosdep install → colcon build → ctest -R "^test_"
```

不需要 Gazebo，不需要 PX4 源码树，所以能在 hosted 上跑。

`simulation.yml` 走的是：

```
colcon build → scripts/ci/sitl_smoke.sh
                 ├─ MicroXRCEAgent udp4 -p 8888
                 ├─ gz sim -r -s <world>
                 ├─ px4 -d -i 0 -w <rootfs>
                 └─ 等 /fmu/out/vehicle_status 出现
```

**它只证明链路是活的**，不证明任务能飞。任务级验收仍然要用
`validation/sitl/run_acceptance.py`，其结论以它自己的 `report.json` 为准。

## 运行环境要求

Github-hosted 侧：ROS 2 Jazzy（Ubuntu 24.04 / noble / amd64 / Python 3.12）。
注意 `tools/env/bootstrap_ros.py` 会硬性拒绝非 noble 或非 x86_64 的环境。

自托管侧需要：

- Ubuntu 24.04 + ROS 2 Jazzy（`/opt/ros/jazzy/setup.bash`）
- PX4 源码树与已编译的 SITL 二进制。默认取 `$HOME/drone-simulation`，可用仓库变量 `PX4_ROOT` 覆盖。
  注意 `run_acceptance.py` 的 `--px4-root` 默认值是 `/home/pawn/drone-simulation`，在 lab 上手动跑时要显式传参。
- **不需要免密 sudo**：runner 用户没有 root，所有安装都落在 `$HOME` 下。这是刻意选择——给自托管 runner 开免密 sudo，等于让任何合入 `main` 的 workflow 拿到那台机器的 root。
- Gazebo（`gz`）、`MicroXRCEAgent`
- Python: `rclpy`、`numpy`、`yaml`、`pymavlink`、`vtkmodules`、`scipy`、`gz.transport13`、`gz.msgs10`

缺什么，`Simulation (self-hosted) → Runner environment` 这一步会打印出来。
**没有这些依赖时，smoke 会明确报"command not found / missing file"，不会假装通过。**

## 启用 / 维护专用 Runner

1. 在目标机器上安装 `actions-runner`，注册到**仓库级**（本仓库是个人账号，没有 org runner）：
   ```bash
   ./config.sh --url https://github.com/pawn-queen/px4_ros_ws --token <TOKEN>
   sudo ./svc.sh install && sudo ./svc.sh start
   ```
2. 给 runner 打标签：`self-hosted`（自动加）、`Linux`、`X64` 之外，再加一个 `sim-heavy`。
   流水线用 `[self-hosted, sim-heavy]` 定位它，不依赖主机名。
3. 验证：
   ```bash
   gh api repos/pawn-queen/px4_ros_ws/actions/runners \
     --jq '.runners[] | {name, status, busy, labels: [.labels[].name]}'
   ```
4. 维护：runner 主机的 ROS/PX4 升级后，手动跑一次 `simulation.yml` 的 `Runner environment` 作业确认。

## 安全隔离要求

这是公开仓库，自托管 runner 的规矩必须写死：

1. **绝不在 `pull_request` 上无条件跑自托管作业。**`simulation.yml` 有两道闸：
   `pull_request.head.repo.full_name == github.repository`（挡掉所有 fork）和
   `run-sim` 标签（只有 write 权限能打）。两道闸都在 `guard` 作业里集中判定。
2. **不用 `pull_request_target`。**它会带着写权限 token 执行 PR 代码，是最典型的 RCE 入口。
3. **不在自托管作业里暴露 secret。**当前 `simulation.yml` 不需要任何 secret。
4. **runner 机器上不要放长期凭据**：`~/.ssh`、`~/.config/gh`、Tailscale 密钥、飞控私钥都不应该在这台机器的默认用户下。
5. **`permissions: contents: read`** 写死在两个 workflow 里，不依赖仓库默认值。
6. 如果将来要接入外部协作者：先给 runner 换成容器/VM 隔离，再考虑允许其分支触发。

## 故障排查

| 现象 | 先看什么 |
| --- | --- |
| PR 上没有任何检查 | workflow 文件是否在默认分支上（`pull_request` 读的是 base 分支的定义） |
| 检查一直 `queued` | `gh api .../actions/runners` 确认 runner 在线、标签匹配 |
| `submodule ... is not populated` | `.gitmodules` 的 URL 是否可解析、gitlink SHA 是否存在于上游 |
| `rosdep` 解析失败 | 该 key 是否存在于 Jazzy 的 rosdep；必要时用 `--skip-keys` 并在此文档登记原因 |
| `ctest -R "^test_"` 一个都没匹配 | 测试目标是否被 `BUILD_TESTING=OFF` 关掉 |
| smoke 卡在等 topic | 先看上传的 `sitl-smoke-logs`：`px4.log` / `gz.log` / `xrce.log` |
| 两个 SITL 互相干扰 | `concurrency` 已按 ref 互斥；不同 ref 同时跑时 `GZ_PARTITION` 与 `ROS_DOMAIN_ID` 是随机的 |
