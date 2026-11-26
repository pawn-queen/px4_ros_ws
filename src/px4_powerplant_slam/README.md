# 独立 RGB-D SLAM 仿真

本包运行 RTAB-Map RGB-D 局部里程计与图优化，Madgwick 根据机体 IMU 的角速度和比力估计重力方向。适用传感器为 Gazebo `x500_depth` 上的共光心、相互平行且已去畸变的 OakD-Lite 彩色/深度相机和 `x500_base` IMU。外参取自两层 SDF 安装变换之积。D435if 的左右红外和真实深度配准需要各自的输入适配和标定。

仿真与 rosbag 回放使用同一个入口。每次运行给出新的绝对数据库文件名。省略时，启动器在新建的临时目录中创建数据库。独立包通过 `package.xml` 声明可选 SLAM 栈的依赖。已在 Jazzy 的 RTAB-Map 0.23.7 和 imu_filter_madgwick 2.1.5 上验证话题接口。在工作空间根目录安装依赖并构建后，按现有流程启动电厂 SITL 和定位链路。

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/px4_powerplant_slam --ignore-src -r -y
colcon build --symlink-install --packages-select px4_powerplant_slam
source install/setup.bash
```

以下两个 launch 在不同终端运行。现有传感器桥启动时加上 IMU 路径，使同一组相机只有一个桥接发布者。专项飞行使用验证入口启动下方的 SLAM launch。

```bash
ros2 launch px4_powerplant_mission gz_sensor_bridge.launch.py \
  imu_gz_topic:=/world/powerplant/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu

ros2 launch px4_powerplant_slam simulation.launch.py \
  database_path:=/tmp/powerplant-run-01.db loop_closure:=true
```

运行时输入为 `/camera`、`/camera/camera_info`、`/depth_camera`、`/depth_camera/camera_info`、`/slam/input/imu`，以及可选的 `/slam/input/occluded` 布尔遮挡事件。回放需要重放以上输入和唯一的仿真 `/clock`。RGB、深度与标定信息按完全相同的采样时间配对。适配器把 RGB 每个像素的射线按深度内参投回原深度像素，输出与彩色同分辨率的米制深度。视场外和无效深度保持 NaN。所有惯性样本进入滤波前清除 Gazebo 的姿态四元数。固定安装变换和所有动态 TF 在私有 `/slam/tf_static` 和 `/slam/tf` 上发布。

| 输出话题 | 语义 |
|---|---|
| `/slam/rgb/image`、`/slam/rgb/camera_info`、`/slam/depth_registered/image` | 配准后的共同 RGB 光学系与采样时间。深度编码 32FC1，单位 m。 |
| `/slam/odometry`、`/slam/odom_info` | `slam_odom → slam_body` 的局部里程计与跟踪诊断；丢失样本按原生 RTAB-Map 的失锁标记解释。 |
| `/slam/status` | JSON 含 `stamp_ns`、`tracking`、`masked`、`inliers`、累计图节点数 `keyframes`、视觉闭环数 `loops` 和最近的匹配节点 `loop_id`。跟踪状态来自实际 OdomInfo，遮挡标志由同采样时刻图像关联。 |
| `/slam/info`、`/slam/mapGraph`、`/slam/mapData` | RTAB-Map 原生回环、关键帧和优化后的图。订阅者应在回放前就绪。 |
| `/slam/cloud_map` | 图优化位姿生成的 `slam_map` 点云，按需向订阅者发送。 |

`loop_closure:=false` 关闭全局视觉闭环和时空近邻匹配，局部里程计和相邻节点连接继续工作。`occlusion_start_s` 相对本轮第一组同步图像的采样时刻，`occlusion_duration_s` 指定持续时长。也可由录制的 Bool 开关触发遮挡。遮挡期间每组图像仍送给后端，彩色变黑且深度全部失效，IMU 继续更新。重复回放使用全新的数据库和同一份带事件的输入包。

正式采集和回放使用 `wait_for_capture_start:=true`。适配器在第一次收到 `/slam/input/occluded` 后开始转发 RGB-D 和 IMU。采集程序等 MCAP 订阅就绪再发送首个值 `false`。手动查看实时输出时，该参数默认关闭。
