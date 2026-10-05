# scripts/tools — 开发辅助脚本

| 脚本 | 用途 |
|---|---|
| build.sh | colcon build 封装（--symlink-install，支持指定包与 CMake 参数） |
| clean.sh | 清理 qianli_ws 的 build / install / log |
| status.sh | 工作区健康检查（git status + 可构建 package 列表） |
| vm_ssh.sh | 一键 SSH 连接开发虚拟机（别名 qianli-vm，见 [docs/SSH.md](../../docs/SSH.md)） |
| vm_check.sh | 虚拟机健康检查：连通性 / sshd / 磁盘 / GPU / ROS / conda |

> 在 Ubuntu 24.04 + ROS 2 Jazzy 环境中执行。
> 连接虚拟机（SSH）相关，先读 [docs/SSH.md](../../docs/SSH.md)。
