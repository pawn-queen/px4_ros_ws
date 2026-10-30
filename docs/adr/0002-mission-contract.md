# ADR-0002：用轻量 contract 模块统一 Python/C++ JSON 契约

- 状态：Accepted
- 日期：2026-10-31
- 相关设计：[mission contract](../design/mission-contract.md)

## 背景

`localization_status` 和 `yolo_detections` 通过 `String` 传递 JSON。字段名只存在于 Python 生产者、C++ 消费者和测试里；一端改名时，另一端可能静默拒收，编译器无法发现。

## 决策

1. 新增 `mission_contract.json` 作为字段清单和版本的机器可读来源。
2. Python 使用 `mission_contract.py` 常量与校验；C++ 使用 `mission_contract.hpp` 常量。
3. 新消息写入 `contract_version: 1`；旧消息没有该字段时仍按旧格式读取。
4. 不改 ROS 消息类型，不引入自定义 message；先锁字段和测试，后续再评估 schema 工具。
5. C++ 解析器使用常量而不是散落字符串；Python 测试和 C++ gtest 覆盖必填字段和版本。

## 后果

- 字段改名会同时触发 Python/C++ 测试，而不是只在运行时静默失败。
- schema 与语言常量仍有重复，必须由测试检查；这是没有代码生成时的已知成本。
- 运行时健康、范围和时间检查仍保留在消费者，contract 不替代它们。

## 未采用

- 立即改成自定义 ROS message：会改变 DDS 接口和所有记录，超出本轮“保持旧字段”的范围。
- 只写文档：无法阻止两端漂移。
