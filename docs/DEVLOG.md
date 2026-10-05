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

## 2026-10-05 — QianLi Simulation v0.3

在原有 `/home/ros/QianLi/qianli_ws` 推进；保留机械臂模型、真实驱动和当前 Domain 0 会话。
所有新增仿真测试使用 ROS_DOMAIN_ID=73 / GZ_PARTITION=qianli_v03，Xvfb :98。
用户授权后安装 slam-toolbox / navigation2 / nav2-bringup，92 个新依赖包，0 升级/删除。

阶段提交：

| Commit | 内容 |
|---|---|
| 7def68c | Geometry v0.1 checkpoint |
| 6d33f1a | 修正 Omni X-drive 定义 |
| ab03705 | 官方 ros2_control omni controller、mock、运动学/运动测试 |
| 06c9578 | Harmonic、gz_ros2_control、ideal_kinematic_sim |
| 579f2c8 | IMU/LiDAR、bridge、系统验收 |
| 0f4b6d2 | SLAM 与保存测试地图 |
| ccbd677 | Holonomic Nav2、整机 bringup、三目标验收 |
| 1c7ba81 | 保存地图 AMCL localization、TF 重复发布检查、验收脚本可执行位 |

验证事实：

- 9 packages `colcon build --symlink-install` 成功；几何与运动学 `colcon test` 零失败。
- Default Xacro 仍为 6 links / 5 joints；28-triangle closed convex STL、八边形和四 outward axes 通过自动检查。
- 官方 4.40.1 controller 的实际 mock 回显命令与独立接触几何公式一致：
  前进/横移/旋转/斜移和 0.5 s 超时停止通过。R=0.4157787873、r=0.075、wheel_offset=π/4。
- Gazebo independent ground truth 的固定时长 forward/strafe/rotate/diagonal 与 commanded odometry 匹配。
- `/scan` 360 束、10 Hz、0.12–12 m、frame lidar_link，实际墙/box 距离有效；
  IMU 50 Hz / imu_link，CCW 0.3 rad/s 测得 0.3，停止后零。
- 在线 SLAM 与 Nav2 整套启动：三个目标均 status=SUCCEEDED；最大 |vx|/|vy|/|wz| = 0.25/0.25/0.6。
  收到 39 条计划，1752 个独立 Gazebo 位姿样本，padded octagonal footprint 与场景障碍相交次数 0。
- RViz 实际点击 Nav2 Goal，目标约 (-1.0,1.01)，最终 map pose (-0.923,0.892)，action 成功。
- 保存地图 AMCL（OmniMotionModel）模式：同样三目标成功，44 条计划、2017 个位姿样本、相交次数 0。
- 保存地图 159×159、0.05 m/cell，已知 23690 cells，occupied 1069。地图来自实际 scan/SLAM，非场景投影生成。
- map→odom 由 SLAM 或 AMCL 互斥发布；odom→base_footprint 仅由 omni controller；TF 发布者数量检查通过。
- Nav2 五个 lifecycle server active、local/global octagonal footprint 和 padding=0.06、所有检查的仿真节点 sim_time=true。

修复记录：

- GenericSystem 与 Gazebo manager 不同场景串行运行，避免 transient robot_description 串扰。
- `--controller-ros-args=-r` 精确 remap，~/cmd_vel→/cmd_vel、~/odom→/odom。
- joint_state_broadcaster 成功后顺序启动 omni controller，service/switch timeout=30 s。
- VM EGL headless 渲染会产生全 infinity scan；GLX/Xvfb 软件渲染有效，默认关闭 EGL headless。
- IMU 动态测试等待 DDS 发现完成后再发命令；避免将未送达的短命令误判为传感器失效。
- VM 软件渲染下 Nav2 的默认 20 ms action acknowledge 不稳定，改为 1000 ms。
- GroupAction 隔离子 launch 参数，确保顶层 rviz 参数不被覆盖。
- 保存地图定位由独立 localization.launch 显式检查/传入 map_file；消除外部 launch 参数歧义。
- 失败 launch 遗留测试进程组已清理；测试结束发送零速度。原机械臂四个进程仍运行。

边界：这是 **ideal_kinematic_sim**，机器人接触/重力禁用，scene 感知与控制软件链有效；
不能验证 omni 滚子摩擦、打滑、力矩或碰撞停止。`/odom` 是 commanded/open-loop，非实测。
simulation inertial 为集中 PLACEHOLDER，等待称重；实机方向、encoder、STM32、反馈里程计/EKF 尚未实现。
当前 LiDAR 360 束会在地图远端留下少量未知栅格；地图是测试样本，非完整环境覆盖或定位精度标定。
RViz 初始 map shader / TF cache 日志警告不影响随后 Global Status OK 与地图/激光显示，后续关注 VM 渲染性能。

新增 qianli_exploration 仅为可编译 package skeleton、标准 action/map 接口与 TODO，没有探索算法。
复现命令见根 README；验收 JSON、运行日志和截图在 `qianli_ws/log/sim_v03`，不纳入 Git。
