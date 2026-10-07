# scripts/tools — 开发辅助脚本

| 脚本 | 用途 |
|---|---|
| build.sh | colcon build 封装（--symlink-install，支持指定包与 CMake 参数） |
| clean.sh | 清理 qianli_ws 的 build / install；保留含回滚资产的 log |
| status.sh | 工作区健康检查（git status + 可构建 package 列表） |
| vm_ssh.sh | 一键 SSH 连接开发虚拟机（别名 qianli-vm，见 [docs/SSH.md](../../docs/SSH.md)） |
| vm_check.sh | 虚拟机健康检查：连通性 / sshd / 磁盘 / GPU / ROS / conda |

> 构建与验收目标为 Ubuntu 22.04 + ROS 2 Humble。SSH 脚本用于保留的旧虚拟机。
> 连接虚拟机（SSH）相关，先读 [docs/SSH.md](../../docs/SSH.md)。

| 迁移工具 | 用途 |
|---|---|
| migration_assets.py | 导出/校验资产，恢复时拒绝路径越界和覆盖不同文件 |
| migration_check.py | 系统、安装模块、资源、标定、相机和渲染检查（按参数启用） |
| validate_ubuntu22.sh | Humble 构建测试、视觉与模拟 ROS 验收，不使能真机 |
| arm_readonly_check.py | 只读六个位置/扭矩；通信、软限位与关闭扭矩均通过才返回成功 |
| train_smoke.py | 双进程 PPO、CUDA 运算与模型保存/加载验收 |

[完整说明](../../docs/UBUNTU22_MIGRATION.md)。
