# P1 建图、前沿与观测流程改进

日期：2026-10-25。本轮按已确认的 **P1 全部项目**实施，在已完成的 P0 输入/地图健康、FAULT、起飞柱体、落地确认之上改进。P2 的连续动力学轨迹、备用制动轨迹、房间拓扑与表面巡检覆盖不在本轮实现范围。

## 1. 哪些地方改变了原理

| 内容 | 原实现 | P1 实现 | 代码 |
| --- | --- | --- | --- |
| 射线空闲更新 | 沿射线按半体素间隔取点 | 精确跨越体素边界，只给具有正长度交段的格子增加空闲证据 | [raycast.hpp](include/px4_powerplant_mission_cpp/raycast.hpp)、[voxel_grid.cpp](src/voxel_grid.cpp) |
| 深度采样 | 固定像素步长 | 按深度、投影足迹、深度/有效性边缘细分 tile，保留真实像素并限制射线预算 | [depth_sampling.hpp](include/px4_powerplant_mission_cpp/depth_sampling.hpp)、[voxel_mapper_node.cpp](src/voxel_mapper_node.cpp) |
| 前沿和净空 | 每次规划从全图重复计算 | 分类差分驱动局部更新；不可变分片索引供规划快照共享 | [planning_index.cpp](src/planning_index.cpp)、[map_snapshot.cpp](src/map_snapshot.cpp) |
| 相机可见性 | 核心默认对称 FOV | 使用成功融合帧匹配的 CameraInfo 与实际尺寸、主点，构成非对称矩形视锥 | [scan_coverage.hpp](include/px4_powerplant_mission_cpp/scan_coverage.hpp)、[frontier_planner.cpp](src/frontier_planner.cpp) |
| 时间与收益 | 主要按路程除速度 | 对每个候选偏航分别计入转向、观察停留与返航转向，再筛选预算和评分 | [frontier_planner.cpp](src/frontier_planner.cpp) |
| 初扫完成 | 固定步数和新融合计数 | 实际深度射线方向覆盖率、稳定悬停、真实标定和已知空闲首段共同决定 | [mission_state_machine.cpp](src/mission_state_machine.cpp) |
| 重规划 | 空结果含义不明；全局地图版本变就丢弃 | 明确失败原因、局部搜索不足后扩大搜索；按依赖区域与最新安全条件检查异步结果 | [mission_planning.cpp](src/mission_planning.cpp) |
| 视点记忆 | 一个位置看过就永久跳过 | 记录位置与偏航 bin；附近几何变化后允许重新观察 | [mission_state_machine.cpp](src/mission_state_machine.cpp)、[mission_planning.cpp](src/mission_planning.cpp) |

研究依据与适用边界见 [research.md](research.md)：借鉴 R1 的体素遍历思想，以及 R3/R5 的增量处理思路。本项目没有完整移植 FUEL、FALCON 或其它论文系统，也没有据此继承其性能或安全保证。

## 2. 精确射线：正长度交段与命中优先

线段写为 `p(t)=origin+t(endpoint-origin)`，`0≤t<1`。DDA 计算下一次与 x/y/z 网格面的相交时刻，从一个体素推进到下一个体素；同时跨多个面时共同推进这些轴。

只有 `t_exit > t_enter` 的体素才提供空闲证据。射线仅触碰某格的一个角点、棱或终点，不足以把该格清空。例如：

```text
r=1，(.5,.5,.5) → (2.5,2.5,2.5)
正长度交段： (0,0,0)、(1,1,1)、(2,2,2)
侧方 (1,0,0) 只有角接触，不增加空闲证据。
```

端点仍作为命中；同一批次的命中集合优先于空闲集合，重复射线不会在同帧无限加强概率。无效/超长射线在本批积分前拒绝，避免半帧地图已经写入。

这与碰撞检查所需的保守 supercover 不同。本轮没有把“正长度空闲射线”直接当成连续机体扫掠体证明；局部路线仍依赖已知空闲净空和离散路径检查，连续轨迹/制动属于 P2。

旧 Python 的半体素采样会遗漏短交段，因此 C++ 的随机射线结果与旧 Python **有意不同**。新测试使用独立线段/AABB 裁剪，以及全局网格面交点排序参考，验证几何集合、批次概率与投影；没有通过放宽浮点容差来掩盖差异。

## 3. 深度自适应采样：把预算用在需要的地方

每个最大步长 tile 先检查实际有效像素。下列情况继续细分，直到达到最小像素步长：

- 最大深度下的横向/纵向投影足迹超限，近似为 `z×pixel_span/focal_length`。
- tile 内有深度突变，或有效/无效像素混合。

叶节点使用其中最近的**真实像素及其真实坐标**。不能把最近距离放到 tile 中心，那会把细障碍投影到错误方向。一个原来固定步长没有选到的细管像素，现在可以通过边缘细分被保留。

射线预算按 coarse tile 轮询分配，先保留每块的最近观测；连每 tile 一条都装不下时，在图像范围内均匀选择 tile，并报告 `budget_limited=true`。没有被实际选中的像素方向不会补进覆盖率。预算限制意味着仍可能漏掉细节，不能把 `0.20 m` 足迹参数解释为任意场景下的障碍检测保证。

当前缺省参数：

| 参数 | 值 | 含义 |
| --- | --- | --- |
| `depth_adaptive_sampling_enabled` | true | 启用自适应采样 |
| `depth_stride_px` | YAML 16 | 最大 coarse tile 尺度，保留原参数名称 |
| `depth_min_stride_px` | 1 | 最细叶节点尺度 |
| `depth_max_ray_spacing_m` | 0.20 | 投影足迹目标 |
| `depth_edge_threshold_m` | 0.25 | 深度边缘细分阈值 |
| `depth_max_rays_per_frame` | 12000 | 单帧实际投影/射线数上限 |
| `depth_min_valid_fraction` | 0.05 | mapper 的覆盖元数据资格阈值 |

为了检测粗采样间隙中的障碍，这个实现会读取全部深度像素，必要时递归细分。它限制投影和射线数量，**不保证 CPU 比原 stride16 更低**；应在真实分辨率、噪声和机载平台上测量积分尾延迟。NaN、失效与量程外值继续不提供空闲证据。

## 4. 增量前沿与膨胀缓存

体素状态分类为已知空闲、占据和未知；不确定与缺失均不允许飞行。前沿和净空定义保持：

```text
frontier(k) = known_free(k) 且存在未知的六邻体素
safe(k) = 机体净空 footprint 中每个邻格均已知空闲
```

接收到完整快照后，receiver 比较新旧空闲/占据集合，得到 `changed_keys`：

- 前沿只重算变化格及其六邻域。
- 净空只重算 footprint 可能覆盖到变化格的中心格。
- 分辨率或净空半径改变时重建，不能沿用旧 footprint。
- 任务 bounds 是查询约束；净空 footprint 的全部轴向极值也必须在 bounds 内。改变任务边界不会把边界之外的空间误当可飞。

缓存按 256 个哈希分片进行写时复制。旧规划任务持有不可变旧索引，主线程更新新索引时只复制受影响分片，避免 worker 与 ROS 回调同时修改同一缓存。配置不匹配时规划器回退到完整计算。

测试示例：`21³=9261` 个空闲体素、分辨率 `0.25 m`、净空半径 `0.35 m`，改变内部一个格时，前沿与净空各重算 **7** 个中心，而非各重算全图。这个数字是该 footprint 的工作量，不是整体加速比。

**仍存在的成本：**ROS 传输仍为完整点云，接收解析/分类差分仍是全图工作；查询安全集合、距离搜索、快照复制以及受影响哈希桶的复制也可能随地图规模增长。本轮没有实现增量地图线协议，也不宣称整条链路为 `O(changed)`。

## 5. 实际相机视锥与时间预算

### 5.1 使用实际匹配的标定

mapper 成功融合深度后发布 `camera_intrinsics`，包含实际图像尺寸、`fx/fy/cx/cy`、frame、CameraInfo 时间和标定来源。相机主点不居中时，左右/上下角界不同：

```text
yaw_min = atan2(cx-(width-.5), fx)
yaw_max = atan2(cx+.5, fx)
pitch_min = atan2(cy-(height-.5), fy)
pitch_max = atan2(cy+.5, fy)
```

FLU 中 yaw 正向为左，pitch 正向为上。规划对候选 yaw 做逆旋转，按相机前向 `x>0`、`atan2(y,x)` 和 `atan2(z,x)` 检查矩形视锥；相机平移偏置也随 yaw 旋转。这样不会把原来的对称 75°×60° 缺省值称为实际视场。

`explore_require_camera_info=true` 为默认行为；没有成功融合且匹配的 CameraInfo 时不进入正常自主探索。静态标定的零时间戳可以合法，不能仅因 stamp 为零就否定它。显式关闭此要求才允许缺省/fallback 模型，状态会保留非真实标定标记。

输入应为与 CameraInfo.K 对应的已校正 pinhole 深度。当前没有增加 raw fisheye 去畸变、ROI/binning 自动修正或任意安装旋转。mapper 的实际投影仍用采样时刻完整四元数；规划未来视点按稳定的水平飞行假设设计，不能据此声称做了六自由度主动感知优化。

### 5.2 每个偏航候选都经过完整预算

探索执行时沿局部路段保持观察 yaw；返航则面向各路段。因此预算采用：

```text
T_out = L_go / v_go + |wrap(view_yaw-current_yaw)| / yaw_rate + dwell
T_home = L_home / v_home + Σ(return heading changes) / yaw_rate
margin = remaining - T_out - T_home - reserve
```

同一位置不同 yaw 会有不同信息收益和时间代价。先检查完整去返路径、定位门限和预算，再将其纳入原有信息/表面/时间/净空/定位/余量评分。`rotation_time_s`、`observation_time_s` 与完整去返路径保留在计划中，执行时只发布局部段。

这里把平移和转向顺序相加，属于偏保守的预算模型；实际控制可并行进行。它尚不包含完整加速度/jerk、所有成像与计算延迟，也不是最短时间轨迹。`explore_view_yaw_rate_radps` 与实际 `max_yaw_rate_radps` 取较小值，观察停留使用 `explore_viewpoint_dwell_s`。候选上限、偏航 bin、间距和六项评分权重已接入 ROS 参数/YAML。

## 6. 覆盖驱动的 HOVER_SCAN 与视点确认

mapper 为成功积分的深度保留最多 32 条观测：采样 ID/时间、采样位置/yaw/tilt、有效像素比例、预算状态，以及实际射线命中的方向扇区。**没有用名义整幅 FOV 直接填充已看见方向。**

初扫缺省分为 72 个方向扇区，每格 5°；完成比例为 0.9。旧 `initial_scan_yaw_steps=12` 现在用于选择慢偏航的搜索步幅，不能单独决定扫描完成。

mapper 的覆盖元数据目前固定为 72 扇区。控制端配置更细的扇区时不能从粗元数据推断覆盖，可能一直等待；默认两端均为 72。该覆盖率衡量合格射线涉及的水平朝向，不代表整个三维表面、每个像素或每个体素都已观测。

退出到 EXPLORE 要同时满足：

1. P0 定位/姿态/起飞柱体仍有效，并已完成稳定悬停。
2. 真实新雷达/深度融合和栅格检查满足条件。
3. 合格实际深度观测覆盖达到阈值；重复 ID、起扫前/过期帧、位置偏离、倾角过大或有效深度不足不能增加覆盖。
4. 存在至少 `initial_scan_min_free_path_m=0.75 m`、满足净空的已知空闲首段，并在退出前稳定停留。

某方向深度无效时，仍可在已确认柱体内保持位置并缓慢偏航寻找其它方向；到达该朝向后 `initial_scan_heading_timeout_s=3` 秒仍无观测就换方向，不伪造覆盖。全局超时仍请求降落。不会增加大幅横滚/俯仰或未知空间平移。

普通探索视点也必须经历朝向对准、停留和**对准后新取得的合格融合深度**。单纯增加融合计数不算观察完成。记忆按 `(位置格, yaw bin)` 保存；地图近处分类变化会使相关记忆重新参与评估，远处变化保留记录。状态变化格数量只作为实际地图更新诊断，不称为完整信息熵或表面覆盖。

## 7. 有原因的重规划与异步结果采用

| 原因 | 含义与处理 |
| --- | --- |
| `WAITING_FOR_MAP` | 无完整、新鲜、可用空闲地图，等待并保留 P0 保护 |
| `CALIBRATION_UNAVAILABLE` | 没有要求的实际相机模型，等待标定/有效融合 |
| `LOCALIZATION_POOR` | 当前或候选去返定位门限不满足，不能靠高收益抵消 |
| `BLOCKED` | 起点/home 没有净空，或已知空闲连通性/路线失效 |
| `RETURN_RESERVE_LOW` | 硬余量不足，或经有效候选检查后时间预算不成立 |
| `NO_LOCAL_CANDIDATE` | 仍有前沿，但当前范围/候选/FOV/访问策略没有找到可执行视点；不等于全图完成 |
| `FRONTIER_EXHAUSTED` | 允许任务边界内没有剩余空闲—未知边界，才按探索耗尽返航 |
| `CANCELLED` / `INVALID_INPUT` | 取消或配置/数值输入错误，不能当作探索完成 |

局部无候选时，在 worker 内对允许的已知空闲连通区域扩大一次搜索；路径长度上界由已知体素数与分辨率给出，而非用欧氏对角线冒充迷宫通路长度。全域重试仍有候选/视角限制；若最终返航，原因明确为策略未找到观测，不会报告地图已经完整探索。

receiver 保存至多 64 条、共 200000 个变化格的版本历史。旧计划的坐标 epoch、标定/配置、视点收益量程区域、完整去程和返航走廊共同构成采用条件：

- 远处变化不影响这些依赖时，可以保留结果。
- 近处变化、历史缺口、epoch/标定变化必须重算。
- 采用前仍检查最新完整去返路径、定位预测与当前余量，不执行未知捷径。

这减少远处新观测造成的全局作废；近处持续变化或规划本身超时仍需结合机载 profiling 继续优化。没有在控制回调里同步执行新的 Dijkstra。

主要控制参数如下；ROS 节点声明与共享 YAML 均已接通：

| 参数 | 缺省值 | 用途 |
| --- | --- | --- |
| `initial_scan_coverage_bins` | 72 | 接收覆盖扇区数 |
| `initial_scan_required_coverage_fraction` | 0.9 | 初扫实际方向覆盖门限 |
| `initial_scan_min_valid_depth_fraction` | 0.2 | 初扫及探索视点的有效深度比例门限 |
| `initial_scan_min_free_path_m` | 0.75 | 允许转入探索的已知空闲首段长度 |
| `initial_scan_heading_timeout_s` | 3 | 到达朝向后等待无效深度的上限 |
| `explore_require_camera_info` | true | 要求成功融合帧使用真实匹配标定 |
| `explore_view_yaw_rate_radps` | 0.6 | 预算偏航速率，与实际控制速率取较小值 |
| `explore_viewpoint_dwell_s` | 0.4 | 对准后的观察停留 |
| `explore_max_candidates` / `explore_max_frontier_targets` | 96 / 400 | 单次搜索候选与可见性目标上限 |
| `explore_yaw_bins` | 12 | 每位置偏航候选数，允许 1—64 |
| `explore_candidate_spacing_m` / `explore_min_goal_distance_m` | 0.75 / 0.75 | 候选分桶间距及最小观察点移动距离 |
| `explore_information_weight` / `explore_surface_weight` | 3 / 2 | 信息与补拍收益权重 |
| `explore_travel_time_weight` / `explore_clearance_weight` | 0.12 / 0.5 | 时间与净空权重 |
| `explore_localization_weight` / `explore_reserve_weight` | 0.5 / 0.5 | 定位与余量权重 |

## 8. 验证入口与剩余边界

构建后使用同一基线入口，保留 P0 的本机 Cyclone DDS 配置：

```bash
bash src/px4_powerplant_mission_cpp/scripts/run_p0_baseline.sh
```

名字保留以便复现原基线；新增 P1 测试会随 CMake 一并执行。主要新增验证为：独立 DDA 几何 oracle、薄障碍像素与预算、随机增量索引与全量参考、实际非对称相机视锥、转向/停留预算、方向记忆、覆盖初扫、迟到旧观测和规划依赖区域。实际数量和结果记录在 [MIGRATION_STATUS.md](MIGRATION_STATUS.md)。

仍需完整 PX4/Gazebo、真实设备标定和大地图/YOLO 并行负载评测。当前没有实飞速度提升、覆盖率提升百分比或运行时延保证；连续机体碰撞、制动余量和动态障碍预测继续属于后续阶段。
