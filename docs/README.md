# 项目文档索引

这是 `px4_ros_ws` 的唯一文档入口。先读本页，再跳到对应模块；不要从一堆 P0/P2 文件里猜“当前该信哪份”。

## 先读什么

| 你想做什么 | 先读 |
|---|---|
| 第一次了解系统 | [架构总览](architecture/README.md)、[术语表](../GLOSSARY.md) |
| 构建/运行仿真 | [C++ runtime README](../src/px4_powerplant_mission_cpp/README.md)、[运行模式](architecture/runtime-modes.md) |
| 跑验证/SITL | [validation/README](../validation/README.md) |
| 改 runtime/validation 边界 | [ADR-0001](adr/0001-runtime-validation-seam.md) |
| 改 Python/C++ JSON 契约 | [mission contract](design/mission-contract.md)、[ADR-0002](adr/0002-mission-contract.md) |
| 查历史 P0/P1/P2 | [设计文档](design)、[验证记录](validation)、[历史](history) |
| 做研究/实验 | [研究文档](research) |
| 参与开发/CI/发布 | [CONTRIBUTING](CONTRIBUTING.md)、[CI 架构](CI_ARCHITECTURE.md)、[分支规范](BRANCHING.md)、[发布流程](RELEASE_PROCESS.md) |
| 用 Agent 工作 | [AGENTS.md](../AGENTS.md)、[领域文档说明](agents/domain.md) |

## 文档分类

| 目录 | 回答的问题 | 状态 |
|---|---|---|
| `architecture/` | 系统由哪些 module 组成、依赖和运行拓扑是什么？ | 当前架构 |
| `design/` | 某个能力为什么这样设计、边界是什么？ | 当前设计 |
| `validation/` | 某个能力实际验证了什么、没验证什么？ | 当前验证记录 |
| `research/` | 研究、外部资料和实验依据是什么？ | 参考/研究 |
| `history/` | 过去的迁移和阶段记录是什么？ | 只读历史 |
| `adr/` | 已经做出的结构决策有哪些？ | 决策记录 |
| 根目录 | 术语、开发流程、CI、发布、Agent 入口 | 跨项目 |

## 当前状态与历史

- 当前 runtime/validation 边界见 [ADR-0001](adr/0001-runtime-validation-seam.md)。
- P0/P1/P2 是历史阶段名，不是当前模块名；新代码不要再用这些前缀。
- `history/` 下的文档只用于追溯，不要把其中的命令或结论当作当前入口。
- 验证结论必须区分“软件测试通过”“SITL 通过”“实机通过”，不能互相替代。

## 语言约定

除专业名词、命令、路径、代码标识外，说明性正文用中文。新增文档先放进上述分类，再在 [AGENTS.md](../AGENTS.md) 或本页加一条入口；不要让同一主题拥有两个 current 文档。
