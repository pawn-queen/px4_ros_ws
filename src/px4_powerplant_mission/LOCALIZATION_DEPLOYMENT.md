# 仿真与实机共用的定位链路

更新：2026-10-24。范围仅为定位、独立外部位置输入及对应启动配置；不代表整个飞行任务已经完成实机验收。

## 数据流与独立性

```text
Gazebo → uwb_simulator ─┐
                       ├→ PoseWithCovarianceStamped → external_vision_bridge → PX4 EKF
真实 Tag → UWB Host ───┘                  │                                  │
                                        └→ 独立来源健康检查                 │
                                                                           ↓
                                                   localization ← vehicle_odometry
                                                          ↓
                                      /powerplant/localization/odom → 建图/控制
```

两种环境运行相同的节点和 `localization/inputs.py`，通过配置选择数据源。`localization/uwb.py` 的测距解算、残差与几何评估接口保持不变，Host 继续调用它。实测锚点、逐锚噪声及设备时钟由 Host 配置；本包不虚构硬件校准值。

外部视觉桥只订阅独立 UWB `PoseWithCovarianceStamped`，不再订阅 `localized_odom_topic`。不使用 PX4 位置、速度或航向筛选/修正后再回灌。消息变换、时间、位置协方差检查通过后，每个新测量最多发一次。位置创新的飞控侧门控交给 PX4 EKF。

默认 `navigation_mode=px4`：定位节点把飞控输出的完整位置、姿态及协方差转换为 ROS ENU/FLU。UWB 过期或收到明确的故障/停止事件时，即使 PX4 仍在输出，导航位置方差也升到后备高值，保留控制端的独立定位条件。PX4 位姿本身过期则停止发布里程计，不制造新状态时间。

`navigation_mode=adaptive` 保留旧导航侧自适应融合与 NIS 检查，供对照试验；`fusion.py` 的公式可继续复用。该结果可能含 PX4 先验/速度，不是独立测量，禁止接回 EV。默认部署不需要该模式，尤其不要把安装偏离机体原点的单 Tag 位置直接当作机体位置。

## 配置选择

仿真默认命令保持兼容：

```bash
ros2 launch px4_powerplant_mission powerplant_system.launch.py
# 等价于 localization_profile:=simulation
```

实机定位接入（Host 和 DDS Agent 单独启动）：

```bash
ros2 launch px4_powerplant_mission powerplant_system.launch.py \
  localization_profile:=hardware use_mapping:=false
```

此命令只启动定位与 EV；控制、YOLO 默认关闭，`use_mapping:=false` 用于先验证定位。真实相机/雷达与建图的部署不在本次修改范围。

| 配置 | simulation | hardware |
|---|---|---|
| UWB pose | `/uwb/pose` | `/uwb/hardware/pose` |
| UWB quality | `/uwb/quality` | `/uwb/hardware/quality` |
| UWB 仿真器、Gazebo 真值/时钟桥 | 默认开启 | 默认关闭 |
| `use_sim_time` | true | false |
| 独立姿态输入 | 显式允许 Gazebo 真值姿态 | 默认禁止 |

对应文件为 `config/localization_simulation.yaml`、`config/localization_hardware.yaml`，叠加在 `config/powerplant.yaml` 之后。启动参数可显式覆盖仿真开关；不要在实机运行时打开仿真源。两个节点共享的测量 frame/变换/质量上限必须配置一致。

## 姿态与速度

单 Tag 的单位四元数只占位，姿态协方差 `1e6` 表示未知。实机 EV 发布 `q=NaN`、未知姿态方差和未知速度，不编造航向。`use_independent_orientation=true` 只用于确有独立姿态的源；还必须通过完整四元数和姿态协方差检查，不能只看开关。

仿真保留真值姿态时可继续用原 SITL 外部位置和 yaw 融合设置。实机单 Tag 应只选择有效的位置融合项，不启用外部 yaw/速度融合；保留实际有效的飞控航向来源。不要直接复制 README 中的 SITL 关闭磁罗盘/GPS 参数到实机。

PX4 完整四元数按 `R_ENU←NED · R_NED←FRD · R_FRD←FLU` 转换，包含横滚和俯仰。导航默认使用 PX4 姿态，不依赖 UWB 提供姿态。姿态方差在 PX4 body 与 ROS world 间旋转；ROS Odometry 的 twist 使用 `child_frame_id=base_link` 的 FLU 分量。

## 坐标与天线偏移

本次保留原控制器的 ENU→NED 航点转换。因此 `map` 必须是与 PX4 local NED 对齐的 ENU：原点一致，轴向满足原有 ENU/NED 约定。代码拒绝未知的 PX4 pose_frame 和不匹配的 UWB frame，不会凭 frame 名猜变换。

如果实测锚点使用另一坐标系，可以在两个节点中同时配置：

```yaml
uwb_frame: survey
uwb_to_map_translation_m: [0.0, 0.0, 0.0]  # 替换为标定结果
uwb_to_map_yaw_rad: 0.0                  # 替换为标定结果
```

变换为 `p_map = Rz(yaw) p_survey + translation`，同时旋转完整位置协方差及可用姿态。该模型要求测绘坐标 z 与重力竖直方向一致，不支持任意倾斜的测绘坐标系。参数不是自动标定结果。

UWB 位置仍代表天线位置。默认 PX4 导航链路应在飞控中设置实测 `EKF2_EV_POS_X/Y/Z`（FRD，单位米；按实际固件核对），由 EKF 处理观测点到机体参考点的安装偏移。桥不利用 PX4 姿态预先修正 UWB，以免再引入相关的反馈。不得在桥和飞控重复补偿。

PX4 reset_counter 变化时丢弃缓存并撤销独立健康，等待新 UWB。位置/航向重置可能改变旧地图适用性；若实际发生坐标原点改变，需要重新校准并重建地图。本次没有添加自动地图重定位功能。

## 时间与健康事件

- 硬件 UWB 使用 Host 的测量 UTC 时间，直接保留到 `timestamp_sample`，转换单位 ns→us。导航使用 PX4 的 `timestamp_sample`，发布定时器不把旧状态重新打成 now。
- 默认 `px4_timestamp_domain=system`，对应开启 uXRCE-DDS 时间同步。PX4 boot↔系统时间由中间件转换，本节点不再手工减飞控开机偏移。
- 仿真中 UWB、导航、建图和 EV 节点都接 `/clock`。EV 将 ROS 测量年龄映射到系统通信时钟；PX4 回传样本反向映射到 ROS 时钟。该跨时钟年龄映射按实时速率运行的 SITL 使用；加速、暂停/回放的严格时间重建需要一致的时钟配置，不能把它视为任意倍速时钟标定。
- 若整个 PX4 DDS 链路明确使用 ROS 时钟，可统一设置两个节点的 `px4_timestamp_domain=ros`。它不是关闭时间检查的开关，未同步的 boot 时间会被拒绝。
- 拒绝缺失、过期、超前过多、重复或倒序的样本；允许最多 100 ms 的小幅超前以容忍仿真 `/clock` 调度。时钟回退清空历史并增加 EV reset_counter。
- `/uwb/hardware/status` 的 session 结束/开始、设备故障、时钟拒绝、非运行状态，以及 quality 拒绝事件撤销可信状态；被停止事件覆盖的旧排队样本不能恢复定位。新有效 pose 才能恢复，单独的 accepted=true 不恢复。
- 不订阅质量话题也不会伪造质量：位置协方差与超时检查仍执行。丢失状态消息时依赖超时退化。质量分数继续为 0，不虚构 0–100 标定分数。

## 验证边界

自动测试覆盖完整姿态变换、硬件无姿态输入、独立 EV 与 PX4 导航分离、位置协方差/坐标变换、重复/过期/倒序样本、故障队列、时钟回退、原始采样时间，以及旧测距与自适应融合回归。实际执行结果见 `../px4_powerplant_mission_cpp/MIGRATION_STATUS.md` 的本次定位更新记录。

硬件锚点坐标、噪声模型、天线偏移、航向来源、串口与飞控版本仍需要实机测量和配置；自动测试不代表真实飞行验收。


## 2026-10-25：原生 P0 消费端健康协议

定位状态新增 `status_stamp_ns`、`status_sequence` 和 `localization_epoch`。序号在同一 epoch 内递增，ROS 时钟回退或 PX4 estimator reset 后切换 epoch；导航里程计继续保留原采样时间。C++ mapper/controller 同时检查这些状态与六维 pose 协方差，旧包不能刷新健康，坐标 epoch 变化不能沿用旧地图/home。单 Tag、独立 UWB→EV 与 PX4→导航的数据来源关系保持不变。

launch 将 `use_sim_time` 同步到控制与 YOLO，硬件 profile 仍默认墙钟。P0 起飞柱体配置、落地确认、FAULT 恢复条件，以及仅限本机的 Cyclone DDS 验证步骤见 [P0_VALIDATION.md](../px4_powerplant_mission_cpp/P0_VALIDATION.md)。本机跨进程通信通过不代表真实 UWB、Agent/PX4、飞控 failsafe 或完整飞行已经验收。
