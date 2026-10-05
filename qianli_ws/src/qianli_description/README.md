# QianLi Base Geometry v0.1

在原有 `QianLi/qianli_ws/src/qianli_description` 包内新增移动底盘几何模型。
目标环境 Ubuntu 24.04 / ROS 2 Jazzy / RViz2，后续面向 Gazebo Harmonic、Nav2、ros2_control。
本版只包含几何、TF、显示配置、Nav2 footprint 片段与验证脚本。
不包含驱动、控制器、滚子、传感器或机械臂装配，也不定义质量/惯量。
原有 SO-ARM101 URDF、18 个 STL、RViz 配置保留，可通过 `arm_display.launch.py` 独立显示。

## 文件与单一尺寸来源

| 文件 | 用途 |
|---|---|
| `urdf/common.xacro` | 集中尺寸、轮安装角、材料；几何数字的唯一来源 |
| `urdf/qianli.urdf.xacro` | 底盘模型入口 |
| `urdf/base.xacro` | 地面 frame、底板及固定关节 |
| `urdf/wheels.xacro` | 四个轮子的复用宏与径向安装 |
| `meshes/base/qianli_base.stl` | 程序生成的米制、闭合凸八边形薄板，16 顶点、28 三角面 |
| `scripts/generate_geometry.py` | 从 common.xacro 生成 STL 与 footprint，无需 CAD/网络/第三方 Python 库 |
| `scripts/validate_geometry.py` | 展开 URDF、尺寸/TF/轴向、网格闭合/法向/体积和 footprint 验证 |
| `config/nav2_footprint.yaml` | local/global costmap 的八边形 footprint 参数片段 |
| `launch/display.launch.py` | 底盘 robot_state_publisher、joint_state_publisher、RViz |
| `rviz/qianli.rviz` | 地面 0.1 m 网格、RobotModel、TF 轴及地面 Axes |
| `launch/arm_display.launch.py` | 保留此前机械臂 display 启动逻辑 |

## 最终尺寸（m，角度除外）

| 参数 | v0.1 值 | 定义 |
|---|---:|---|
| base_size | 0.70 | 底板最大 X/Y 尺寸 |
| base_long_edge | 0.40 | 四条直边 |
| base_cut | 0.15 | 每角切掉等腰直角三角形的直角边 |
| base_thickness | 0.004 | 厚度 |
| base_height | 0.09 | 底板**中面**及 base_link 的地面高度 |
| wheel_radius | 0.075 | 轮半径 |
| wheel_width | 0.038 | 沿轴向宽度 |
| wheel_x / wheel_y | 0.294 / 0.294 | 各轮中心坐标绝对值 |
| wheel_z | 0.075 | 轮心相对地面的高度 |
| wheel_mount_angle | pi/4 | FL 安装角 45°，其他轮按对称关系得到 |

`base_long_edge = base_size - 2*base_cut`，斜边 `sqrt(2)*base_cut = 0.212132034 m`。
底板局部 z 为 ±0.002 m；地面坐标的上下表面为 0.088 / 0.092 m。
90 mm 的测量暂解释为中面高度，待实测确认测的是上表面、下表面还是中面。
轮径 0.15 m，轮底位于地面 z=0，轮顶 z=0.15 m。
左右、前后对应轮心间距均为 0.588 m。
板面积 0.445 m²，几何体积 0.00178 m³（不据此推断质量）。

## ROS 坐标与 TF

遵循 REP-103：+X 向前（用户照片上方），+Y 向左，+Z 向上。
base_footprint 位于机器人几何中心的地面投影；base_link 位于底板中面中心。

```text
base_footprint                     ground (0, 0, 0)
└── base_link                      fixed: (0, 0, 0.09)
    ├── front_left_wheel_link      continuous
    ├── front_right_wheel_link     continuous
    ├── rear_left_wheel_link       continuous
    └── rear_right_wheel_link      continuous
```

固定关节名 `base_footprint_joint`，轮关节名分别为
`front_left_wheel_joint`、`front_right_wheel_joint`、`rear_left_wheel_joint`、`rear_right_wheel_joint`。
无 map/odom，也不发布假里程计。robot_state_publisher 发布固定 /tf_static 与活动关节 /tf；
joint_state_publisher 仅用于可视化零位/滑块关节状态。

| 轮子 | 地面坐标轮心 (x,y,z) | joint origin 相对 base_link | joint RPY | 轴向（base_link 中） |
|---|---|---|---|---|
| FL | (0.294, 0.294, 0.075) | (0.294, 0.294, -0.015) | (0, pi/2, pi/4) | (1,1,0)/sqrt(2) |
| FR | (0.294, -0.294, 0.075) | (0.294, -0.294, -0.015) | (0, pi/2, -pi/4) | (1,-1,0)/sqrt(2) |
| RL | (-0.294, 0.294, 0.075) | (-0.294, 0.294, -0.015) | (0, pi/2, 3*pi/4) | (-1,1,0)/sqrt(2) |
| RR | (-0.294, -0.294, 0.075) | (-0.294, -0.294, -0.015) | (0, pi/2, -3*pi/4) | (-1,-1,0)/sqrt(2) |

四个 joint 的 `<axis xyz="0 0 1"/>` 均以**关节局部坐标系**表示。
URDF 圆柱也沿局部 Z，因此 visual/collision 不再额外旋转。
`Rz(yaw) Ry(pi/2) (0,0,1) = (cos(yaw), sin(yaw), 0)`，圆柱轴、关节轴、实车指定电机轴一致。
RViz TF 显示中，各轮的**蓝色 +Z 轴**应沿斜边法向朝外；红色 X 为轮局部轴，不是电机轴。
continuous joint 的正角度仅是右手坐标约定，不代表电机正转、编码器正方向或Omni X-drive 电机接口符号。

## 八边形、Collision 与 Nav2 footprint

STL 上下表面采用同一组逆时针顶点（米），侧面封闭，法向朝外。
visual 与 collision 共用这一个低面数凸 mesh；轮子 collision 是普通 cylinder。

```yaml
[[0.35, 0.20], [0.20, 0.35], [-0.20, 0.35], [-0.35, 0.20],
 [-0.35, -0.20], [-0.20, -0.35], [0.20, -0.35], [0.35, -0.20]]
```

`config/nav2_footprint.yaml` 提供 Nav2 所需的 polygon **字符串**参数，分别放在
`local_costmap.local_costmap.ros__parameters` 和 `global_costmap.global_costmap.ros__parameters`，
`robot_base_frame: base_footprint`。将来合并进完整 Nav2 配置，本版不启动 Nav2。

该 footprint 严格采用用户指定底板八边形。按现有几何，轮子沿斜边法向外探
`sqrt(2)*(0.294-0.275)+0.038/2 = 0.045870 m`，含轮 XY 包围尺寸约 0.720936 m。
所以此片段记录的是底板轮廓，不是包含轮子/支架的整车保守碰撞包络。
接入真实导航前，应实测整车外探并决定完整包络及 padding；不能直接把本版底板轮廓当作整车安全边界。

## 构建、生成与验证

```bash
source /opt/ros/jazzy/setup.bash
cd ~/QianLi/qianli_ws
colcon build --symlink-install
source install/setup.bash

xacro src/qianli_description/urdf/qianli.urdf.xacro -o /tmp/qianli.urdf
check_urdf /tmp/qianli.urdf  # 仅在已经安装时使用
python3 src/qianli_description/scripts/validate_geometry.py --urdf /tmp/qianli.urdf
colcon test --packages-select qianli_description
colcon test-result --verbose

ros2 launch qianli_description display.launch.py
# SSH 无显示验证：
ros2 launch qianli_description display.launch.py use_rviz:=false
# 手动检查关节旋转（仅显示，无硬件指令）：
ros2 launch qianli_description display.launch.py gui:=true
```

已有独立机械臂节点使用同名 base_link 或 /robot_description 时，在新终端运行
`export ROS_DOMAIN_ID=71` 后再启动底盘，避免两个模型同时占用同一 TF 树。
这只是本次显示隔离；包没有硬编码 ROS_DOMAIN_ID。

更改尺寸时，先编辑 `urdf/common.xacro`，再执行：

```bash
cd ~/QianLi/qianli_ws/src/qianli_description
python3 scripts/generate_geometry.py
python3 scripts/generate_geometry.py --check
python3 scripts/validate_geometry.py
```

STL 与 YAML 是提交到项目的生成源资产；构建时只校验，不自动改写源码。
生成产物过期会令构建失败并提示重新生成，避免参数/视觉/footprint 悄悄不一致。
改变板大小时保持 `base_long_edge = base_size-2*base_cut`；改变轮径时重新确认轮心高度与接地关系。
本版限定径向 45° 安装，验证会发现错误的非对称轴向。

RViz 检查：Fixed Frame 为 base_footprint，RobotModel 与 TF 均正常；
用 0.1 m 网格和顶视确认八边形/轮心对称，斜视确认薄板与四轮接地；
开启 TF/Axes，核对每个 wheel frame 蓝色 Z 指向对应的 ±45°/±135° 方向。
尺寸以数值验证为准，截图辅助确认渲染与资源加载。

## 实测、推导与未知项

Measured / confirmed（用户提供，约值不表示更高精度）：

- overall width ≈ 700 mm
- overall length ≈ 700 mm
- straight edges ≈ 400 mm
- diagonal edges ≈ 210 mm
- plate thickness ≈ 4 mm（初始测量 3–4 mm，本版取 4 mm）
- plate height ≈ 90 mm
- wheel diameter = 150 mm
- wheel width = 38 mm
- wheel center spacing ≈ 590 mm
- motors centered on diagonal edges
- motor shafts perpendicular to diagonal edges

Derived：

- corner cut = 150 mm
- exact diagonal edge = 212.13 mm
- wheel center coordinate ≈ ±294 mm
- wheel center spacing model = 588 mm
- 前后间距暂按中心对称同取 588 mm

Still unknown / TODO：

- omni 小滚子尺寸与接触特性（轮型已确认为 Omni，无需 Mecanum X/O 配置）
- encoder polarity
- motor positive rotation direction
- exact mass（整车、底板、每个轮子及其他部件）
- center of mass
- precise inertial parameters
- 电机几何尺寸、支架/紧固件形状与外探
- 90 mm 高度的测量基准；前后轮距、实际安装偏差与带载有效轮径

当前 URDF **没有 inertial**；RViz 几何显示无需质量/惯量。
它尚不能当作 Gazebo 动力学就绪模型使用，不对 Gazebo 接地、摩擦或运动作验证结论。
后续若初步仿真必须添加惯性，应集中定义并明确标注 `TODO / PLACEHOLDER`：
“仅供初步仿真，等待实车称重后更新。”本版不填假精确值。

## 下一阶段

先核实四轮滚子安装、轮距和轴向，记录电机/编码器正方向，再测量质量、重心及电机几何。
之后再决定这套非传统径向布局的运动学、Gazebo 接触模型和 ros2_control 接口。

## 本次验收（2026-10-05）

在 `ros@192.168.26.128` 的原工作区执行：

- `colcon build --symlink-install`：全工作区 4 packages finished。
- Xacro 展开与 `check_urdf`：成功，根节点 base_footprint，6 links / 5 joints。
- `scripts/validate_geometry.py`：STL 闭合/朝外法向/尺寸/体积、轮轴及 footprint 全部通过。
- `colcon test --packages-select qianli_description` 与 `colcon test-result --verbose`：
  2 tests（同一个校验的 CTest/JUnit 记录），0 errors、0 failures、0 skipped。
- 实际启动 `ros2 launch qianli_description display.launch.py`，隔离在 ROS_DOMAIN_ID=71。
- 实时订阅 `/robot_description`、`/joint_states`，并从 tf2 查询 5 个子 frame 的地面位置与轴向，全部通过。
- RViz 在桌面 `:0` 启动；桌面截屏接口返回黑图，因此另用 Xvfb `:98` + 软件 OpenGL
  运行实际 RViz 完成斜视、俯视、侧视截图检查。三视图 Global Status 均为 Ok，
  八边形和四轮显示正常，轮轴径向朝外，无断开的 TF 或错误位置的 link。
- 日志、展开 URDF、实时 TF 数据及截图位于工作区 `log/geometry_v0_1/`（不提交构建产物）。
- 没有执行 Gazebo 动力学测试；没有安装系统软件；原机械臂资产保留。

## 官方参考

- [REP-103 坐标约定](https://www.ros.org/reps/rep-0103.html)
- [ROS Jazzy Cylinder：长度沿 Z 轴](https://docs.ros.org/en/ros2_packages/jazzy/api/geometric_shapes/generated/classshapes_1_1Cylinder.html)
- [robot_state_publisher](https://github.com/ros/robot_state_publisher)
- [Nav2 footprint](https://docs.nav2.org/rolling/configuration_and_development/first_time_robot_setup_guide/footprint/setup_footprint/)

---

## 原有 SO-ARM101 独立模型资料

QianLi 机器人 **URDF / Mesh / TF / RViz** 配置。

## 内容（Milestone 1）

| 文件 | 说明 |
|---|---|
| `urdf/so101.urdf` | **SO-ARM101** 机械臂 URDF（onshape-to-robot 生成，`so101_new_calib`）：6 DOF（shoulder_pan / shoulder_lift / elbow_flex / wrist_flex / wrist_roll / gripper）+ gripper_frame 固定关节，带 transmission |
| `meshes/*.stl` | 视觉/碰撞 STL 网格（18 个，来自 `~/arm-final` 资产） |
| `launch/arm_display.launch.py` | robot_state_publisher + joint_state_publisher_gui + RViz2 |
| `rviz/arm.rviz` | RViz 配置（固定系 base_link） |

## 使用

```bash
# 构建后
source install/setup.bash

# 带 RViz 显示（虚拟机桌面环境）
ros2 launch qianli_description arm_display.launch.py

# 无头验证（SSH 环境）
ros2 launch qianli_description arm_display.launch.py use_rviz:=false
```

## 关节链

```
base_link → shoulder_pan → shoulder_link → shoulder_lift → upper_arm_link
  → elbow_flex → lower_arm_link → wrist_flex → wrist_link → wrist_roll
  → gripper_link → [gripper (moving_jaw)] / [gripper_frame_link (fixed)]
```

## 来源与参考

- 模型来源：真实机械臂 **SO-ARM101**（HX-30HM 舵机，Feetech SCS/STS 兼容），硬件资料见 [docs/HARDWARE.md](../../docs/HARDWARE.md) 与 [hardware/mechanical_arm/docs/joint_limits.md](../../../hardware/mechanical_arm/docs/joint_limits.md)；
- 虚拟机原始文件：`~/arm-final/ros2_ws/src/so101_bringup/urdf/so101.urdf` + `~/arm-final/ros2_ws/src/so101_bringup/urdf/assets/*.stl`；
- 待办：Milestone 3 将接入 MoveIt2（SRDF / kinematics / OMPL 配置）。

## v0.3 可选扩展

默认 geometry 入口仍为 6 links / 5 joints，无 ros2_control、传感器和惯性。
`control_mode:=mock|gazebo` 加载四轮接口；`simulation:=true` 启用集中 TODO/PLACEHOLDER 惯性；
`sensors:=true` 加载虚拟 imu_link/lidar_link 和 Gazebo sensors。
推荐经 qianli_bringup/sim.launch.py 启动，由 launch 自动传 controllers_file。
激光 ground z=0.25 m 仅为虚拟安装。全向轮不是 Mecanum，电机与编码器实机方向仍待校准。
