# qianli_description

QianLi 机器人 **URDF / Mesh / TF / RViz** 配置。

## 内容（Milestone 1）

| 文件 | 说明 |
|---|---|
| `urdf/so101.urdf` | **SO-ARM101** 机械臂 URDF（onshape-to-robot 生成，`so101_new_calib`）：6 DOF（shoulder_pan / shoulder_lift / elbow_flex / wrist_flex / wrist_roll / gripper）+ gripper_frame 固定关节，带 transmission |
| `meshes/*.stl` | 视觉/碰撞 STL 网格（18 个，来自 `~/arm-final` 资产） |
| `launch/display.launch.py` | robot_state_publisher + joint_state_publisher_gui + RViz2 |
| `rviz/arm.rviz` | RViz 配置（固定系 base_link） |

## 使用

```bash
# 构建后
source install/setup.bash

# 带 RViz 显示（虚拟机桌面环境）
ros2 launch qianli_description display.launch.py

# 无头验证（SSH 环境）
ros2 launch qianli_description display.launch.py use_rviz:=false
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
