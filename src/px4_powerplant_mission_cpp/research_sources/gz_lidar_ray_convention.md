# Gazebo GPU 雷达：消息名义角度与实际纹理采样方向

日期：2026-10-28。范围：Issue #3 的离线传感器、视觉和碰撞一致性验证。**不改变在线 mapper 的 LaserScan 语义，也不将仿真模型套用到实机雷达。**

## 两个独立问题

Gazebo 的单层原生 LaserScan 在当前 gz-sensors 8.2.2 中产生 `vertical_count=1`、上下角均为零、`vertical_angle_step=NaN`。[其角度分辨率实现](https://github.com/gazebosim/gz-sensors/blob/gz-sensors8_8.2.2/src/Lidar.cc#L396-L400) 对单层计算 `0/0`。唯一一层应直接采用唯一高度角，不能计算 `0 × NaN`。校验器现在对单样本轴忽略没有意义的步长；多样本轴仍检查步长有限且端点一致。粒子 AABB 测试也拒绝非有限几何，避免将非法射线误报为受烟雾影响。

第二个问题是实际 GPU 渲染方向。gz-rendering 8.2.3 的 [Ogre2GpuRays 实现](https://github.com/gazebosim/gz-rendering/blob/gz-rendering8_8.2.3/ogre2/src/Ogre2GpuRays.cc) 分两遍生成扫描：第一遍渲染六个透视立方体面，第二遍用一张 float32 UV 表按最近邻取样。当前 360×1 扫描的立方体面尺寸为 128×128。其像素中心方向与消息名义连续角度存在量化差异，可能打到薄件的另一侧或跨过遮挡边缘。

`powerplant_lidar_rays.py` 依源码依次计算：

1. 按 `CreateSampleTexture` 的角度推进顺序构造 `(yaw × pitch) × ray`，保留 gz-math 四元数运算顺序。
2. 按 `SampleCubemap` 的分面规则得到 UV，再写成 float32。
3. 使用已验证材质的 nearest 过滤和 [OgreNext 2.3.3 默认 TAM_CLAMP](https://github.com/OGRECave/ogre-next/blob/v2.3.3/OgreMain/src/OgreHlmsSamplerblock.cpp)，取得确定像素中心。
4. 按各立方体面相机朝向转回 FLU，再经已捕获且验证的 Scene 父子位姿变换到世界系。

方向生成不读取回波距离，没有寻找最匹配角度或按误差修剪射线。

## 独立诊断证据

对真实电厂 `sensor_validation_clear_view` 捕获的 360 条射线进行独立预测，未排除任何方向：

| 指标 | 消息名义射线 | 源码预测的纹理中心射线 |
| --- | --- | --- |
| 有限回波具有视觉首交点 | 169/175 | 175/175 |
| 回波对视觉 p95 | 0.380171759 m | 0.000010747 m |
| 回波对视觉最大差 | 1.851727182 m | 0.000037766 m |
| 有限回波但没有对应范围内视觉交点 | 6 | 0 |
| 非有限回波但解析预测存在范围内视觉交点 | 2 | 0 |

最大方向量化角差为 0.6330093°。原始名义射线误差保持在报告中；它揭示消息的理想角度与渲染器实际采样的差异。上述旧捕获没有保存当时渲染器库哈希，因此该辅助诊断本身不是新的正式通过证明。

## 显式使用和版本门控

`verify_powerplant_sensor_samples.py` 默认继续使用 `--lidar-ray-convention nominal`。只有显式指定 `gz-rendering8-cubemap-nearest` 时才采用上述方向，且必须同时具备：

- 新捕获中 `/proc/.../maps` 记录的已审核 gz-rendering 8.2.3、OgreNext/GL3Plus 2.3.3 库及哈希；当前文件也必须仍匹配。
- Ogre 运行日志注册的唯一材质、compositor 和三个 GLSL shader 路径，以及实际文件哈希、日志哈希。
- 已验证的运行时 Scene 模型、link、sensor ID、父子关系和位姿；本地消息位姿约定还要求既有传感器库证据。

历史捕获缺少上述元数据或遇到未审核版本时输出 `UNCONFIRMED`，不能根据当前安装文件补造旧加载证据；已经记录的文件发生变化则拒绝。当前支持的二进制和资源哈希是有限目录，其他编译构建需要先核对源码与构建，不能只凭同版本字符串自动通过。

两种模式始终使用原来的传感器→视觉 p95 ≤ 8 cm、视觉→简化碰撞最大差 ≤ 5 cm 标准以及最少清洁样本数；不通过提高阈值掩盖差异。名义模式的结果与实际纹理方向模式应分别保存，前者仍用于评价 ROS 消息方向假设带来的误差，后者用于评价实际视觉、传感器渲染和简化碰撞的一致性。

## 仍须独立验证

GPU 雷达和深度传感器采样视觉几何，不能代替 DART 物理接触验证。在线建图仍采用消息名义角度，方向量化的距离相关误差是否落在地图、定位和机体裕量内，需要另行验证。这个离线模型不改变任务航点、地图证据或实机雷达驱动。
