# Agent 入口

## 先读

1. [项目文档索引](docs/README.md)
2. [术语表](GLOSSARY.md)
3. 与任务相关的 ADR：`docs/adr/`

## 项目地图

| 任务 | 入口 |
|---|---|
| 运行/构建 runtime | [C++ README](src/px4_powerplant_mission_cpp/README.md)、[Python README](src/px4_powerplant_mission/README.md) |
| SITL / 验证 | [validation/README](validation/README.md) |
| 架构与依赖 | [docs/architecture](docs/architecture/README.md) |
| 设计/验证/历史 | `docs/design/`、`docs/validation/`、`docs/history/` |
| 研究 | `docs/research/` |
| CI/分支/发布 | `docs/CONTRIBUTING.md`、`docs/CI_ARCHITECTURE.md`、`docs/BRANCHING.md`、`docs/RELEASE_PROCESS.md` |

## 工作规则

- 任务与规格使用 GitHub Issues；操作前读取 [docs/agents/issue-tracker.md](docs/agents/issue-tracker.md)。
- 使用五个默认 triage 标签；分类前读取 [docs/agents/triage-labels.md](docs/agents/triage-labels.md)。
- 采用 single-context 领域布局：根目录 `GLOSSARY.md` 与 `docs/adr/`。
- 新概念先写术语表，再改代码/测试/文档；不要新增 `p0_*`/`p2_*` 路径。
- 修改 runtime/validation 边界时，先读 [ADR-0001](docs/adr/0001-runtime-validation-seam.md)。
