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
- 创建第一个 Git commit（07d9477）；
- 补充 `.gitattributes` 统一 LF 行尾（目标平台为 Linux，避免 Windows 下 CRLF 破坏 shell 脚本），并规范化已跟踪文件（第二个 commit）。

### 决策记录

| 决策 | 说明 |
|---|---|
| 仓库位置 | D:\projects\QianLi（已有空目录，非破坏性使用，D 盘空间充足） |
| LICENSE | 采用 MIT；如需更换协议，替换 LICENSE 文件即可 |
| 可编译 package | 仅 3 个（interfaces / description / bringup）；其余 11 个先占位，避免无意义代码 |
| 构建验证 | 本机无 ROS 2，colcon build 待 Ubuntu 22.04 验证（见 ENVIRONMENT.md §3） |
| 系统环境 | 未安装任何软件、未修改任何全局配置（PATH / shell 等） |

### 待办

- [ ] Milestone 1：qianli_description 中机械臂 URDF / Xacro 建模（见 ROADMAP.md）

## 2026-10-04（续）— 接入开发虚拟机并验证构建（Milestone 0 达成）

### 虚拟机接入

- 通过 VMware 识别运行中虚拟机：`D:\Ubuntu-VM\ubuntu24-ros2.vmx`（"Ubuntu 24.04 ROS2 Jazzy"）；
- 从 VMware DHCP 租约确认 VM IP：**192.168.26.128**（hostname `ros2-ubuntu`）；
- SSH 登录成功（用户 `ros`，Windows OpenSSH + SSH_ASKPASS 密码认证）；
- VM 环境确认：Ubuntu 24.04.4 LTS、ROS 2 **Jazzy**（/opt/ros/jazzy）、colcon、Python 3.12.3、CMake 3.28.3、Git 2.43.0；
- 已装关键包：urdf / xacro / rviz2 / robot-state-publisher / joint-state-publisher / ros2-control / moveit；
- 待装包（后续 Milestone 需要时）：nav2-bringup / robot-localization / gazebo-ros-pkgs。

### 决策记录（续）

| 决策 | 说明 |
|---|---|
| 目标平台变更 | **Ubuntu 24.04 + ROS 2 Jazzy**（原 22.04 + Humble），因开发虚拟机为 24.04/Jazzy；文档（README/PROJECT/ENVIRONMENT/ROADMAP/scripts）已同步更新 |
| 安装脚本 | install_ros2_humble.sh 由 install_ros2_jazzy.sh 取代；source_env.sh 自动检测 jazzy/humble |
| 仓库同步 | 虚拟机无 hgfs 共享，使用 scp 传输（保留 .git，虚拟机内可直接 git 操作） |

### 构建验证（Milestone 0）

- [x] 工作区传输至虚拟机 `~/QianLi`；
- [x] 首次构建失败：`Unknown CMake command "ament_package"` —— 根因：CMakeLists.txt 缺少显式 `find_package(ament_cmake REQUIRED)`（colcon 只注入路径，不注入宏加载）；
- [x] 三个 CMakeLists.txt 补上 `find_package(ament_cmake REQUIRED)` 后重建，**全部通过**：

  ```
  Summary: 3 packages finished [4.55s]
  ```

- [x] `ros2 pkg list` 识别：qianli_bringup / qianli_description / qianli_interfaces；
- [x] 构建产物 install/ 生成完整。

### 待办

- [ ] Milestone 1：机械臂 URDF / Xacro 建模（qianli_description，见 ROADMAP.md）
