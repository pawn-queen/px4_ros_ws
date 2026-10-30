# Mission JSON 契约

本文件说明 Python 与 C++ 之间通过 `std_msgs/msg/String` 传递的 JSON 契约。它不是 ROS 消息定义；真正的类型仍然是字符串，契约负责字段名、必填项、版本和拒绝原因。

## 单一 owner

| 文件 | 作用 |
|---|---|
| `src/px4_powerplant_mission/config/mission_contract.json` | 机器可读 schema；字段清单和版本的事实来源 |
| `src/px4_powerplant_mission/px4_powerplant_mission/mission_contract.py` | Python 常量与校验 |
| `src/px4_powerplant_mission_cpp/include/px4_powerplant_mission_cpp/mission_contract.hpp` | C++ 常量；解析器使用这些键名 |
| `validation/tests/test_ros_contract.py` | 跨进程、真实 DDS 的集成检查 |

## localization_status

Python 定位节点发布 `/powerplant/localization/status`，C++ 控制器消费。当前版本为 `1`：

必填旧字段：

- `status_stamp_ns`
- `status_sequence`
- `localization_epoch`
- `navigation_sample_ns`
- `uwb_input_healthy`
- `has_independent_position`
- `position_variance_enu_m2`

新增字段：

- `contract_version`：新消息写 `1`；旧日志可以没有，校验器按旧格式接受。

C++ 对缺失核心字段、错误类型、过期/未来时间和 epoch/sequence 回退继续拒绝。契约模块只统一字段名和版本，不替代这些运行时健康检查。

## yolo_detection

Python YOLO 节点发布 `/powerplant/perception/yolo_detections`，C++ 巡检消费。当前版本为 `1`：

- 顶层：`stamp.sec`、`stamp.nanosec`、`frame_id`、`image_width`、`image_height`、`detections`
- 每个 detection：`class_name`、`confidence`、`bbox_xyxy`（四个数）
- 新增：`contract_version: 1`

## 兼容规则

1. 不移除旧字段；新增字段先写 schema 和版本。
2. Python 与 C++ 字段常量必须同名；测试会检查 schema 与 Python 常量一致，C++ 测试会检查解析器所需的常量集合。
3. 缺失 `contract_version` 的旧日志可读；新发布消息必须写当前版本。
4. 字段类型、范围、时间和 epoch/sequence 的运行时校验仍由各消费者负责，不能只看 schema 通过。
5. 契约变更需要同时改 schema、Python 常量、C++ 常量、契约测试和本文件；不能只改一端。
