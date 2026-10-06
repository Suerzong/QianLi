# so101_overlay — 机械臂驱动的 QianLi 改动层

这里放的是 **SO-101 驱动栈在 QianLi 项目里的改动版本**，不是一份独立的 ROS 包。

## 为什么是"改动层"而不是把整包搬进来

真机驱动栈原本在虚拟机的另一个工作区里：

```
~/legacy/arm/arm-final/ros2_ws/src/so101_bringup/
```

它带有 ~20 MB 的 STL 网格，并且是 `colcon --symlink-install` 的 egg-link 安装
（`install/.../site-packages/so101-bringup.egg-link` 指向源码树），改源码即生效。
把整包复制进本仓库会造成两份实现、互相漂移——正是"限位对不上"这类 bug 的温床。

所以这里只保存 **被 QianLi 改过的文件**，作为版本记录与同步源。

## 文件清单与同步目标

| 本目录 | 同步到 |
|---|---|
| `driver_node.py` | `~/legacy/arm/arm-final/ros2_ws/src/so101_bringup/so101_bringup/driver_node.py` |
| `ik_node.py` | `~/legacy/arm/arm-final/ros2_ws/src/so101_bringup/so101_bringup/ik_node.py` |
| `driver_params.yaml` | 源树 `src/so101_bringup/config/` **和** `install/so101_bringup/share/so101_bringup/config/`（**install 那份是普通文件，不是软链，必须单独覆盖**） |

`install` 里的 `driver_params.yaml` 不是软链接——launch 读的就是它。
只改源码的话，`ros2 launch` 仍然用旧限位。`build/` 下若也有副本一并覆盖，
免得下次 `colcon build` 又把它盖回去。

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
