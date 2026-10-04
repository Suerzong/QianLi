# mechanical_arm — 机械臂硬件资料

真实机械臂 **SO-ARM101**（SO101）的型号、参数、通信协议、驱动文档与测试记录。

## 资料清单

| 文件 | 内容 |
|---|---|
| [docs/joint_limits.md](docs/joint_limits.md) | 6 关节实测限位（舵机 raw 寄存器 + 角度）、固件通信特性、lerobot 补丁说明 |

## 硬件要点

- **6 DOF**：shoulder_pan / shoulder_lift / elbow_flex / wrist_flex / wrist_roll / gripper；
- **舵机**：HX-30HM ×6（Feetech SCS/STS 兼容，Model 777，映射 STS3215），12V，1 Mbps；
- **通信**：USB 芯片 CH343P（ServoDebugger 板）→ 虚拟机内 `/dev/ttyACM0`；
- **协议**：SYNC_WRITE (0x83) / GroupSyncRead (0x84) 稳定；WRITE 指令 fire-and-forget（不回状态包）；
- **URDF**：`qianli_ws/src/qianli_description/urdf/so101.urdf`（onshape-to-robot 生成）；
- **驱动栈**（虚拟机 `~/arm-final` 参考实现）：so101_bringup（driver_node / ik_node / servo_protocol）、so101_grasp、so101_interfaces、so101_mission、so101_vision。

## 相关文档

- [docs/HARDWARE.md](../../docs/HARDWARE.md)
- [docs/ROADMAP.md](../../docs/ROADMAP.md)（Milestone 2：真实机械臂接入）
