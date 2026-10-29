# 发布流程（RELEASE_PROCESS）

## 原则

**没有验证过的 commit 不得被称为 stable。** 版本号的语义由证据决定，不由期望决定：

| 称呼 | 需要什么证据 |
| --- | --- |
| `stable` | 至少一次完整 SITL 任务验收通过 **且** 实机飞行记录（谁、何时、多久、结果） |
| `beta` | 完整 SITL 验收通过，未上实机 |
| `archived snapshot` | 不声称任何运行验证，仅作历史留存 |

本仓库目前**没有任何实机飞行记录**，因此任何 Release 都不能标 stable。

## 1. 选择已验证的 commit

```bash
git switch main
git pull --ff-only
gh pr list --state merged --limit 10        # 确认要发布的内容都已合并
gh run list --branch main --limit 5         # 确认 main 上的 CI 是绿的
```

必须同时满足：

- 目标 commit 在 `main` 上，且 CI 通过
- 该 commit 上跑过至少一次与你声明级别相符的验证
- 验证证据可指认（日志路径、run id、SITL 场景、时间）

如果这些条件不满足，发布一个 `archived snapshot` 类型的 Release，或者先不发布。

## 2. 打 annotated tag

**必须用 annotated tag**（带 tagger、日期、消息），轻量 tag 不携带这些信息。

```bash
git tag -a v0.2.0 -m "v0.2.0 — <一句话说明>" \
  -m "验证：SITL <场景> @ <日期>，CI run <run-id>；未实机飞行。"
git push origin refs/tags/v0.2.0
```

tag 消息里必须写清验证级别。不允许写"稳定""可用于实机"这类没有证据的结论。

## 3. 发布 Release

```bash
gh release create v0.2.0 \
  --verify-tag \
  --title "v0.2.0 — <标题>" \
  --notes-file /tmp/notes.md \
  --latest
```

- `--verify-tag`：tag 不存在就失败，避免打错版本号
- 旧版本的归档发布用 `--latest=false`，且**不要**加 `--prerelease` 除非它确实是一个预发布

Release Notes 必须包含：

1. 本版本做了什么（面向使用者，不是 commit 列表）
2. **验证状态**：SITL 场景与结果、是否实机、CI run 链接
3. **依赖版本**：ROS 2 发行版、PX4 commit、Gazebo 版本、Python 版本
4. **子模块 commit**：`src/px4_msgs`、`src/px4_ros_com` 的 SHA
5. 已知限制与未验证部分
6. 启动与构建方法
7. 是否继续维护

## 4. 记录依赖版本

```bash
git submodule status
grep -E 'jazzy|noble' docs/CI_ARCHITECTURE.md | head
ros2 --version 2>/dev/null || echo "ros2 CLI not on PATH"
gz sim --versions 2>/dev/null | head -3
python3 --version
```

把输出原样贴进 Release Notes。只写"ROS 2"而不写发行版和系统版本，等于没写。

## 5. 回退到历史版本

```bash
git fetch --tags
git switch --detach v0.1.0
```

或者从归档分支取文件：

```bash
git show legacy/stable-2026:src/px4_powerplant_mission/config/powerplant.yaml > /tmp/old.yaml
```

**不要**删除或覆盖已发布的 tag。tag 一旦推送就是公开事实；要修正只能发新版本并在 Notes 里说明上一版的问题。
