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

## 2026-10-04（续 2）— Milestone 1：机械臂建模（进行中）

### 完成内容

- 在虚拟机 `~/QianLi` 上配置项目文件夹：`hardware/mechanical_arm/docs/joint_limits.md`（来自 `~/arm-final/docs/`）；
- 识别真实机械臂：**SO-ARM101**（6 DOF，HX-30HM 舵机 ×6，Feetech SCS/STS 兼容，12V，/dev/ttyACM0）；
- 将虚拟机已有模型整合进 `qianli_description`：
  - `urdf/so101.urdf`（452 行，onshape-to-robot 生成，6 旋转关节 + 1 固定，带 transmission）；
  - `meshes/*.stl`（18 个视觉/碰撞网格，资产路径改为 `package://qianli_description/meshes/`）；
  - 新增 `launch/display.launch.py`（robot_state_publisher + joint_state_publisher_gui + RViz2，支持 use_rviz:=false 无头验证）；
  - 新增 `rviz/arm.rviz`；
  - 更新 CMakeLists.txt（install urdf/meshes/rviz/launch）与 package.xml（exec_depend 补齐）。

### 决策记录（续）

| 决策 | 说明 |
|---|---|
| 整合哪个模型 | 以**真实硬件 SO-ARM101**（~/arm-final）为 Milestone 1 模型；~/arm_ws 的 my_arm（CAD 推导）作为备选参考 |
| 模型命名 | 保留原始 URDF（`so101_new_calib`），资产路径统一改为 `package://qianli_description/meshes/` |

### 待办（Milestone 1 剩余）

- [ ] VM 构建 qianli_description 并验证（URDF 解析 + TF）—— ✅ 已完成，见下
- [ ] RViz 中机械臂正确显示（虚拟机桌面）—— 配置就绪，待桌面环境确认
- [ ] 硬件参数核对（关节限位已由 joint_limits.md 提供，URDF limit 与之比对）

### Milestone 1 验证结果（2026-10-04，虚拟机）

- ✅ `colcon build`：3 packages finished（qianli_description 含模型完整安装）；
- ✅ URDF 解析：`so101_new_calib`，7 joints（6 revolute + 1 fixed）、8 links；
- ✅ `robot_state_publisher`："Robot initialized"；
- ✅ `/joint_states`：6 关节（shoulder_pan / shoulder_lift / elbow_flex / wrist_flex / wrist_roll / gripper）全部发布；
- ✅ TF 链完整：`tf2_echo base_link → gripper_frame_link` 有效变换 [0.391, 0, 0.226]；
- ✅ 话题 /tf、/tf_static 正常；
- ⚠️ 已知非致命警告：KDL 提示根 link 带 inertia（ROS1 遗留传输块 hardwareInterface 标签）；joint_state_publisher_gui 需桌面显示（SSH 下无显示属环境限制，虚拟机桌面正常）。

### 决策记录（续）

| 决策 | 说明 |
|---|---|
| 整合哪个模型 | 以**真实硬件 SO-ARM101**（~/arm-final）为 Milestone 1 模型；~/arm_ws 的 my_arm（CAD 推导）作为备选参考 |
| 模型命名 | 保留原始 URDF（`so101_new_calib`），资产路径统一改为 `package://qianli_description/meshes/` |

### 待办（下一步）

- [ ] Milestone 1 收尾：虚拟机桌面运行 `ros2 launch qianli_description display.launch.py` 确认 RViz 显示
- [ ] Milestone 2：qianli_arm 接入真实机械臂（参考 ~/arm-final 的 so101_bringup 驱动栈）

## 2026-10-04（续 3）— 拖动示教工具 qianli_teach

### 背景

用户询问"能否自然进入拖动示教"。现状：机械臂（SO-101，direct 只读模式）可自由拖动，
`/joint_states` 实时反映真实姿态（绝对式编码器），但缺"记录"和"回放"两个环节。

### 完成内容

- 新建 `qianli_ws/src/qianli_teach`（ament_python 包）：
  - `teach_node.py`：record（订阅 /joint_states + 键盘触发录制，存 YAML）/ playback（读 YAML，按原间隔发布 /joint_commands，--speed 倍率）
- 修复记录：
  - `setup.cfg` 中 `install_scripts=$base/lib/qianli_teach`（初版缺 `$base` 键导致 colcon 构建失败）
  - 录制 dt 防御：第一帧仅作起点；`dt>1s` 帧跳过（避免消息源切换/时钟跳变污染轨迹）
- VM 验证（模拟数据）：
  - 录制 79 帧 / 3.95s，dt 干净（最大 0.0536s）
  - 回放完整发出 79 帧 /joint_commands（时长与录制一致）

### 闭环说明

```
拖（人手搬动，扭矩关闭）→ 记（teach_node record）→ 放（teach_node playback → /joint_commands → driver）
```

安全：record 零风险；playback 是否动真臂由 driver `allow_motion` 决定（sim 仅 RViz 演示）。

### 待办（下一步）

- [ ] 真实硬件拖动示教演练：record 录一段真实拖动轨迹 → sim 回放确认 → （确认校准后）direct 回放
- [ ] Milestone 2：把 so101 驱动栈整合进 QianLi（qianli_arm），teach 工具直接对接

## 2026-10-04（续 4）— 修复"找不到虚拟机 SSH"问题

### 背景

用户反馈：之前多次尝试（含 AI 会话）"找不到虚拟机 SSH"。排查确认：

- **实际连通**：Windows 主机 `ssh qianli-vm` 免密成功（别名在 `C:\Users\sez18\.ssh\config`，私钥 `id_ed25519` ↔ VM `authorized_keys` 的 `suerzong@outlook.com`）；
- **sshd 状态**：`systemctl is-enabled ssh` = **enabled**、`is-active` = **active**（开机自启已配置，ENVIRONMENT.md 旧记录"自启 disabled"已过时）；
- **根因**：连接信息分散（别名/密钥只在主机 `~/.ssh/config`），项目文档未给出"怎么连"，新会话无从下手。

### 修复内容

- 新建 **[docs/SSH.md](SSH.md)**：连接信息、快速命令、排查清单（端口/sshd/免密/别名）、常见问题表、别名配置块；
- 新增 `scripts/tools/vm_ssh.sh`：一键 SSH（自动探测别名，回退私钥/密码）；
- 新增 `scripts/tools/vm_check.sh`：连通性 + sshd + 磁盘 + GPU + ROS 健康检查；
- 更新 `docs/ENVIRONMENT.md`：§2.2 修正（sshd 已 enabled、无 GPU、内存 5.8G、磁盘 12G、~/arm_ws 已不存在），新增 §2.1.1 SSH 快速连接；
- 更新 `scripts/tools/README.md`、`scripts/setup/README.md`：登记新脚本与 SSH 入口。

### 环境复核结果（本次 SSH 实测）

| 项 | 结果 |
|---|---|
| SSH 连通 | ✅ 免密成功（别名 qianli-vm） |
| GPU | ❌ 无（VMware 未直通）→ 深度学习训练需 GPU 云服务器 |
| 磁盘 | 12 GB 剩余（84%） |
| 内存 | 5.8 GiB / 可用 3.5 GiB |
| conda | ❌ 未安装（系统 Python 3.12.3） |
| ROS 2 | ✅ jazzy |

### 待办（下一步）

- [ ] 机器人抓取学习环境：本机无 GPU，若在 VM 上跑 robotic-grasping 只能 CPU 版（慢）；建议 GPU 云服务器上搭建（方案见对话记录）
