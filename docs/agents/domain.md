# Domain Docs

采用 single-context 布局：根目录 GLOSSARY.md 与 docs/adr/。

## 探索代码前

- 存在 GLOSSARY.md 时，读取其领域术语。
- 存在 docs/adr/ 时，读取与当前修改相关的决策。
- 文件缺失时继续工作；由 domain-modeling 在术语或决策确定后按需创建。

## 使用规则

Issue、设计、测试名称和代码使用术语表定义的词汇。
新增领域概念交由 domain-modeling 记录。
方案与已有 ADR 冲突时，明确指出对应 ADR 和重新讨论的原因。
