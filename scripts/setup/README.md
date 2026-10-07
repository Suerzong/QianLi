# 环境安装脚本

默认目标是原生 Ubuntu 22.04.5 + ROS 2 Humble，Python 3.10。

| 脚本 | 用途 |
|---|---|
| install_ros2_humble.sh | 校验系统后安装 Humble、colcon 和所需系统依赖 |
| setup_python_envs.sh | 固定依赖，重建 .venv-ros 和隔离的 .venv-train |
| source_env.sh | 当前 Bash shell 加载 Humble 与 ROS 虚拟环境，拒绝混入 Jazzy |
| source_train.sh | 新 Bash shell 加载独立训练环境，拒绝混入 ROS |
| install_device_rules.sh | 安装 CH343P 串口别名和用户组权限；需重新登录/插拔 |
| install_ros2_jazzy.sh | 旧 Ubuntu 24.04 虚拟机的历史安装脚本 |

[迁移与验收步骤](../../docs/UBUNTU22_MIGRATION.md)。不复制 Windows 虚拟环境。
