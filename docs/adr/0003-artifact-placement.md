# ADR-0003：按“可重建性”和“消费者”划分仓库内资产

- 状态：Proposed（草案）
- 日期：2026-10-31
- 相关：[ADR-0001](0001-runtime-validation-seam.md)（validation 不随包安装）、[repository map](../architecture/repository-map.md)

## 背景

`src/px4_powerplant_mission*` 两个 runtime 包里混进了三类不属于 runtime 的东西：

- 训练权重：`best.pt`（单类 `gas_cylinders`，训练数据与训练 run 均不在本仓库，无法重建）和 `yolov8n.pt`（官方 COCO 预训练，可公开下载）；另有只指向不存在目录的 `data.yaml`。
- 生成物：`sitl_body_envelope.json` 由 `validation/model/body_envelope.py` 生成，消费者是 `validation/sitl/run_acceptance.py`，却被 CMake 装进 `share/`，成为包安装面的一部分。
- 文档：包内 `.agents/workflows/` 与根 `.agents/`（由 `skills-lock.json` 管理的第三方技能）共用同一命名空间。

同时，`validation/`、`docs/research/`、`docs/agents/` 已经分别是验证、研究、Agent 约定的既定位置，资产却留在 runtime 包里。

历史层面另有一笔账：初始提交含 `build/`、`install/`、`log/`，删除后仍留在历史中；可达历史未压缩 blob 约 179 MB，其中构建产物约 150 MB、模型权重 18 MB。

## 决策

1. **判据**：只清“可再从上游取得”或“已无消费者”的东西；不可复现的运行资产原地保留；文档与证据按消费者归位。不采用“二进制一律不入库”的硬规则。
2. **证据**：SITL 机体包络证据归 `validation/evidence/`，runtime CMake 不再安装它；代码里的路径只命名一次（`BODY_ENVELOPE_JSON`），文档写完整相对路径。
3. **研究元数据**：Consensus 检索元数据归 `docs/research/`，与其引用的研究文档同级。
4. **Agent 文档**：包级流程文档归 `docs/agents/workflows/`；根 `.agents/` 只放安装器管理的技能，项目自写内容不进该命名空间。
5. **权重**：可公开取得的 `yolov8n.pt` 和无消费者的 `data.yaml` 删除；随删移除 `fallback_model_path`，模型缺失时报错而不是静默回退到通用 COCO 权重。唯一且不可重建的 `best.pt` 继续入库并随包安装，其来源事实记录在包 README。
6. **历史**：本次只做 forward-only。`build/`、`install/`、`log/` 与已删除权重仍在历史中；用 `git filter-repo` 重写需要单独 issue、单独批准，并让所有 clone 重新拉取。

## 后果

- runtime 包只留下运行时代码、配置、launch、rviz、ament 标记和唯一权重；`share/` 不再夹带证据。
- 证据文件移动或重命名时，必须同时更新生成命令、验收 runner、`docs/research/` 的说明和验证记录；代码侧已用一个常量收敛。
- 证据缺失或 SDF 与记录不一致时，SITL 验收直接抛错，不会跳过或降级。
- 删除 `data.yaml` 会丢失训练集布局记录，已在包 README 以事实形式保留（类别、epochs、数据配置、起始权重、日期）。
- 仓库 clone 成本不变：历史体积是独立问题，forward-only 不减少它。
- 包测试不依赖权重文件（YOLO 测试用假检测器），CI 不因权重出库或入库改变。

## 未采用

- **全部二进制出树 + `fetch_assets.py` + 校验清单**：会让 clone 不再自包含，需要多一步取数；本次明确不采用。
- **Git LFS 或 GitHub Release 托管权重**：为 6 MB 的单文件引入外部存储与配额依赖，收益不足以抵消维护面。
- **保留 `yolov8n.pt` 作为回退模型**：它是 80 类 COCO 权重，与本项目单类巡检任务不匹配，且会掩盖“模型没装好”这一真实故障。
- **立即 `git filter-repo`**：不可逆、影响所有已有 clone，当前收益主要是仓库体积与推送带宽，风险高于收益，留待独立决策。
