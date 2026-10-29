# 分支规范（BRANCHING）

## 长期分支

| 分支 | 职责 | 是否接受推送 |
| --- | --- | --- |
| `main` | 唯一持续集成主线，永远是可构建的最新状态 | 仅通过 PR |
| `legacy/stable-2026` | 2026-10 旧主线的冻结快照（`e5334419`） | **只读**，不再维护 |

`legacy/*` 的规则：

- 不接受任何提交，不接受 PR
- 不 rebase、不合并 `main`、不改源码
- 它存在的唯一目的是让旧版本、旧调参、旧 README 永远可查
- 需要旧版本里的某个改动时，cherry-pick 到新分支，而不是在 `legacy/*` 上改

## 短期分支

全部从 `main` 切出，合并后立即删除。

| 前缀 | 用途 | 例子 |
| --- | --- | --- |
| `feat/` | 新功能、新能力 | `feat/uwb-quality-gate` |
| `fix/` | 缺陷修复、回归修复 | `fix/stale-uwb-sample` |
| `refactor/` | 结构调整，外部行为不变 | `refactor/voxel-grid-to-cpp` |
| `docs/` | 只改文档 | `docs/ci-architecture` |
| `chore/` | 工程配置、依赖、脚手架 | `chore/github-workflow` |
| `ci/` | 只改 CI 配置 | `ci/add-gtest-job` |

命名要求：

- 全小写，用 `-` 分隔
- 不超过 4 个单词，能看懂改哪块
- 不要用 `test`、`tmp`、`dev`、`wip` 这类无信息量的名字

## 特殊分支

`migration/*` 只用于一次性的结构性迁移（例如把旧主线替换为新基线）。合并后删除，不参与日常开发。

## 什么时候删除分支

- PR 合并后立刻删除（`gh pr merge --squash --delete-branch`）
- PR 关闭但未合并：确认没有遗漏提交后删除
- 长期无进展（> 30 天）的分支：开 Issue 记录进度，然后删除，需要时从 Issue 重建

已合并分支的提交仍然可以通过 `git log --all --grep` 或 PR 页面找回，删除分支不会丢代码。

## 为什么不设长期 `develop` 分支

1. **多一条长期分支就多一份合并债务**：`develop` 与 `main` 的差异会持续积累，最终要么频繁互相同步，要么在发布时一次性爆炸。
2. **这个仓库的发布节奏是"打 tag"，不是"合 develop"**：版本 = `main` 上某个已验证 commit 的 annotated tag，见 [RELEASE_PROCESS.md](RELEASE_PROCESS.md)。
3. **单人 + Agent 协作时，分支越少越好**：Agent 需要一眼看出"现在该基于哪个分支工作"，`main` 是唯一答案。
4. 如果将来需要预发布通道，用 `release/x.y` 短期分支，发完即删，而不是常年存在的 `develop`。
