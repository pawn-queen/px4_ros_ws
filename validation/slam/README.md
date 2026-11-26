# 原生电厂 RGB-D SLAM 验证

本实验使用现有 ROS 2 Jazzy、Gazebo Harmonic、powerplant 世界、gz_x500_depth、OakD-Lite 和机身 IMU。UWB 经 EV 进入 PX4，专项航线程序独占 Offboard 指令。SLAM 仅从已桥接的相机、内参、原始 IMU、时钟和遮挡事件读取输入。Gazebo 按模型名记录的真值与工厂网格只供验收程序使用。

## 准备仿真

按照[现有 SITL 说明](../../src/px4_powerplant_mission/README.md)设置 `PX4_ROOT`、构建基础工作区并启动 DDS、PX4、Gazebo 及 GCS 心跳。选择原生相机资产。运行[独立包的构建步骤](../../src/px4_powerplant_slam/README.md)，传感器桥与任务栈分别在已加载工作区环境的终端启动。每种桥只启动一次。

```bash
ros2 launch px4_powerplant_mission gz_sensor_bridge.launch.py \
  imu_gz_topic:=/world/powerplant/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu

ros2 launch px4_powerplant_mission powerplant_system.launch.py \
  config_file:="$(ros2 pkg prefix --share px4_powerplant_mission)/config/sitl_gui_validation.yaml" \
  use_control:=false use_mapping:=false use_yolo:=false
```

单独查看估计器可运行 `ros2 launch px4_powerplant_slam simulation.launch.py`。正式采集由下面的入口启动自己的估计器，每轮使用新的数据库。开始前让模型在原点落地、解锁前检查就绪，并确认没有其他航线控制器。验证使用系统 Python、SciPy、NumPy、Matplotlib、PyYAML、Gazebo Python 绑定和 ROS Jazzy 的 MCAP 后端。

## 标定与盲验

从工作区根目录运行。每次指定新的采集、回放输出目录。若同时运行其他 ROS 网络，为回放选择空闲的独立 domain。原始彩色图和深度图按实际原生分辨率写入 MCAP，需要为两个航次及数据库预留磁盘空间。

```bash
out="$PWD/log/slam_trials"
python3 validation/slam/run.py capture --split calibration --prepare-only \
  --output-dir "$out/preflight"
python3 validation/slam/run.py capture --split calibration --output-dir "$out/cal-flight"
python3 validation/slam/run.py calibrate --capture "$out/cal-flight" \
  --output-dir "$out/cal-replays" --thresholds "$out/frozen.json"

# 门槛和软件身份均冻结后，在原生环境的新航次执行
python3 validation/slam/run.py capture --split blind --output-dir "$out/blind-flight" \
  --thresholds "$out/frozen.json"
python3 validation/slam/run.py verify --capture "$out/blind-flight" \
  --output-dir "$out/blind-replays" --thresholds "$out/frozen.json"
```

标定与盲验沿同一审计区域飞行。盲验改变接近 A 的速度。起始有效段只做一次固定尺度的偏航与平移对齐。三个实验共用观测时钟与事前定义的航线窗口。

| 实验 | 核查对象 |
|---|---|
| 真实重访与负例 | /slam/mapGraph 中首次与重访 A 的非邻接回环边，两端真值与视场，相似设备之间的可比负例数及错误回环数 |
| 原图重定位 | 输入副本同时遮蔽 RGB 与深度并保留 IMU。依据里程计的实际 LOST、旧关键帧匹配与持续位姿误差判定恢复 |
| 历史几何修正 | B 首次观测的关键帧和点在重访时仍保留。对同一批旧深度样本分别应用回环前后图位姿，与 B 的原生网格参考面比较，同时报告关回环对照 |

## 证据

每航次保留 `route.json`、`manifest.json`、`raw/` MCAP、`truth.csv`、`events.json`、`b_reference.npy` 和 `online/`。正式回放按录制时钟以原速运行，三对回环开关实验各自从空库开始。冻结文件记录标定指标及界限，旁边的 `.identity.json` 记录版本、资产和程序哈希。盲验对照同航次在线输出和三轮回放。成功评分时产出 `report.json` 和对照图。返回非零时按本轮 `failure.json`、采集 manifest 与节点日志定位失败阶段，保留已有证据。CPU 降载或合成输入结果单列为调试记录。
