# tools/env

本目录存放开发环境辅助工具，不属于 runtime，也不属于 validation 场景。

| 文件 | 作用 |
|---|---|
| `bootstrap_ros.py` | 在 Ubuntu noble amd64 上无 root 解包固定版本的 Cyclone/ROS middleware 到 `.p0_deps` |
| `ros_env.sh` | 为当前 shell 设置 Cyclone + LOCALHOST；不写 `.bashrc` |
| `ros_dependencies.json` | `bootstrap_ros.py` 使用的固定包和 SHA256 锁定 |

```bash
python3 tools/env/bootstrap_ros.py
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source tools/env/ros_env.sh
```
