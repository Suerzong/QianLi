# qianli_teach — 拖动示教

QianLi 的**拖动示教**节点：把"拖 → 记 → 放"闭环补全。

- **record**：订阅 `/joint_states`（真实编码器反馈），键盘触发开始/停止，保存关节轨迹为 YAML
- **playback**：读取 YAML 轨迹，按原时间间隔发布 `/joint_commands`（由 driver 执行）

## 用法

```bash
# 先 source 环境（QianLi + 机械臂驱动栈）
source ~/QianLi/qianli_ws/install/setup.bash
source ~/legacy/arm/arm-final/ros2_ws/install/setup.bash   # 或你的驱动环境

# ① 录制：按 Enter 开始 → 手动拖动机械臂 → 再按 Enter 停止保存
ros2 run qianli_teach teach_node --mode record --file ~/traj1.yaml

# ② 回放：按 Enter 开始回放（--speed 1.0 原速 / 0.5 半速 / 2.0 两倍速）
ros2 run qianli_teach teach_node --mode playback --file ~/traj1.yaml --speed 1.0
```

## 安全

| 模式 | 风险 | 说明 |
|---|---|---|
| record | 零风险 | 纯读 `/joint_states`；扭矩关闭时手臂自由，可随意拖动 |
| playback | 取决于 driver | 发布 `/joint_commands`；`allow_motion=false`（sim）只在 RViz 演示；`allow_motion=true`（direct）会真实驱动舵机，需确认校准与安全距离后再用 |

## 轨迹文件格式（YAML）

```yaml
joint_names: [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper]
frames:
  - dt: 0.1002        # 与上一帧的时间间隔（秒）
    positions: [0.02, 0.2, 0.3, 0.4, 0.5, 0.6]   # 6 关节弧度
  - dt: 0.0997
    positions: [...]
```

## 设计说明

- 录制用 **ROS header.stamp 时间差**（不是单调时钟），保证回放节奏与真实一致
- 防御异常跳变：`dt > 1s` 的帧会被跳过（消息源切换/时钟跳变时不污染轨迹）
- 回放按 tick（10ms）逐帧发布，`--speed` 控制倍率
