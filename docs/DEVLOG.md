# DEVLOG — 开发日志

> 按时间顺序记录 QianLi 开发过程、决策与验证结果。

## 2026-10-04 — 项目初始化（Phase 0 / Milestone 0）

### 环境扫描摘要

- 当前开发机：Windows 11 家庭版（10.0.26100，64 位），主机 SUERZONG，用户 sez18；
- Git 2.53.0.windows.2（身份已配置）；Python 3.13.12（conda）+ 3.14.5；CMake 4.3.1；
- VS Code（E:\Applications\Microsoft VS Code）与 CLion 2026.2.1 可用；
- **无 ROS 2、无 colcon**；WSL 未注册发行版（E:\Applications\WSL 下有 Ubuntu-24.04 目录，未注册）；
- 磁盘：D: 剩余 198.9 GB（QianLi 所在盘）。

### 完成内容

- 创建 QianLi 完整目录结构（docs / hardware / firmware / simulation / datasets / scripts / qianli_ws）；
- 初始化 Git 仓库与 .gitignore；
- 创建基础 ROS 2 package 骨架：**qianli_interfaces、qianli_description、qianli_bringup**（package.xml + CMakeLists.txt，可编译）；
- 其余 11 个 package（qianli_arm / qianli_base / qianli_control / qianli_localization / qianli_slam / qianli_navigation / qianli_exploration / qianli_perception / qianli_semantic_map / qianli_task_planner / qianli_manipulation / qianli_safety）先创建目录 + README 占位，待对应 Milestone 启动时初始化；
- 创建辅助脚本：install_ros2_humble.sh、source_env.sh、build.sh、clean.sh、status.sh；
- 编写 README / PROJECT / ARCHITECTURE / ROADMAP / ENVIRONMENT / HARDWARE 文档；
- 创建第一个 Git commit。

### 决策记录

| 决策 | 说明 |
|---|---|
| 仓库位置 | D:\projects\QianLi（已有空目录，非破坏性使用，D 盘空间充足） |
| LICENSE | 采用 MIT；如需更换协议，替换 LICENSE 文件即可 |
| 可编译 package | 仅 3 个（interfaces / description / bringup）；其余 11 个先占位，避免无意义代码 |
| 构建验证 | 本机无 ROS 2，colcon build 待 Ubuntu 22.04 验证（见 ENVIRONMENT.md §3） |
| 系统环境 | 未安装任何软件、未修改任何全局配置（PATH / shell 等） |

### 待办

- [ ] 在 Ubuntu 22.04 安装 ROS 2 Humble 并执行首次 `colcon build`（验证 Milestone 0 达成）
- [ ] Milestone 1：qianli_description 中机械臂 URDF / Xacro 建模（见 ROADMAP.md）
