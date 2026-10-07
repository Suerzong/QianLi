# SO-101 驱动源码与兼容包

从 2026-10-07 起，上级 `qianli_arm/` 已是可直接 colcon 构建的完整 ROS 包，公开名称为 **so101_bringup**。本目录保存驱动、IK、协议和参数的唯一实现；`so101_bringup/` 是兼容导入命名空间。

launch、配置、测试、资源标记和 setup 文件均在上级包中。ROS 模型使用仓库 qianli_description 的现有 TCP URDF/网格，历史训练继续使用 dual_twin。构建不再依赖 VM 中的 legacy 工作区。

原始来源、校验值和限位合并见上级 migration_provenance.json；有效参数由构建安装到 share/so101_bringup/config，也可用 QI_DRIVER_CONFIG 显式覆盖。默认 sim/allow_motion=false。迁移不写入舵机 EEPROM。

[完整迁移与验收](../../../../../docs/UBUNTU22_MIGRATION.md)

以下为既有安全修复记录：

## 这次改了什么（2026-10-06）

### 1. 限位只有一个来源

`driver_node.py` 原来在模块级硬编码了一份 `LOWER_LIMITS/UPPER_LIMITS`
（照抄 URDF），同时 `driver_params.yaml` 里又有一份从 URDF 反算的
`raw_min/raw_max`。两份数据各写各的，谁也不知道对方的值。

现在关节限位**从 `raw_min/raw_max` 推导**（`_derive_joint_limits()`），
舵机原始计数是唯一事实来源。

### 2. 启动即校验，不合法就拒绝启动

`raw_min/raw_max` 必须落在 `0..4095`（Goal_Position 只有一个 12 位字）。
旧配置让 `raw_max` 溢出后被静默截断——`wrist_roll` 就这样丢了 71.2°。
现在直接 `raise`，并要求先用 `servo_homing_shift.py` 重新定零。

启动时会把 6 个关节的包络打成一张表，一眼能看出哪一侧被谁约束。

### 3. 超限必报告，绝不静默吞掉

原来 `_on_command` 用 `np.clip` 把超限目标悄悄改成限位值，无日志、无状态。
现在每次裁剪都会：

* 以 2 Hz 限流打印告警（含关节名、请求值、限位值、被吞掉多少度）；
* 计入 `/arm/status` 的 `clip_events`，并给出 `clip_last` 与 `clip_seen`；
* 可用 `reject_out_of_range: true` 改为整条指令拒绝。

`ik_node.py` 同理：`_clip_bounds` 记录被夹的关节与越界量，
解算结束后汇报；并把"URDF ∩ 驱动"的有效包络和**哪一侧在约束**打出来。

## 回滚

原文件在虚拟机上备份于：

```
~/QianLi/qianli_ws/log/limit_fix_backup/
  driver_node.py.before_limits_fix
  ik_node.py.before_limits_fix
  driver_params.yaml.before_limits_fix
```

舵机零位（`wrist_roll` Homing_Offset）的回滚：

```bash
~/mj/bin/python ~/QianLi/qianli_ws/src/qianli_vision/scripts/servo_homing_shift.py \
    --restore /tmp/homing_backup.json
```

## 相关脚本（在 `qianli_ws/src/qianli_vision/scripts/`）

| 脚本 | 作用 |
|---|---|
| `servo_register_audit.py` | 只读审计 6 个舵机全部寄存器 |
| `joint_range_calibrate.py` | 手动扫行程测真实机械死点（扭矩必须为 0） |
| `derive_joint_limits.py` | 实测 → 软限位/零点方案 + 新 yaml |
| `servo_homing_shift.py` | 安全改写 Homing_Offset（带备份/校验/回滚） |
| `clear_stale_goal.py` | 清掉残留的越界 Goal_Position |
| `servo_status_diag.py` | 带校验和的舵机状态诊断 |
| `verify_limit_reporting.py` | 回归测试：超限指令必须被报告 |
| `tcp_geometry.py` | 从 URDF 网格算爪口开度剖面 |
| `tcp_calibrate.py` | TCP 标定（定点法 0.5mm / 平面法 2mm） |
| `extrinsic_calib_multi.py` | 外参标定（多点最小二乘，含残差与格宽校验） |

限位的完整实测数据、推导规则与编码坑见
[`hardware/mechanical_arm/docs/joint_limits.md`](../../../../hardware/mechanical_arm/docs/joint_limits.md)。
