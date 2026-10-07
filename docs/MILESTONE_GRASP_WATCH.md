# Milestone: 自动抓取 + (0,0) 值守归位系统（真实机械臂）

日期：ROS 2 Jazzy / SO-101 机械臂 / 虚拟机 `ros2-ubuntu`（192.168.26.128，用户 `ros`）

## 一句话总结

**用户把黄色物块放到棋盘上任意位置 → 系统自动定位、mesh 几何居中、载荷闭环抓取、抬升、放到棋盘 (0,0) 放置点、腕部摆正、归位折叠 → 值守模式监控 (0,0)：一旦发现物块被拿走就自动重新抓回。全程无需任何硬编码姿态。**

## 组成文件（scripts/）

| 文件 | 作用 |
|---|---|
| `grasp_auto.py` | 主流程：几何探测 → mesh 自动居中(+用户 y 偏置) → READY → 悬停 → 分段净空下探 → 载荷闭环合爪(+命令角加压) → 载荷监控抬升 → 移动到 (0,0) 上方 → 下降松爪 → 回中间位腕部摆正 → 回折叠位 |
| `check_at_00.py` | (0,0) 放置点**窗口化掩码重合**检测（只在 (0,0) 周围 ±70px 窗口取黄掩码，与参考掩码比重合；`--reset` 带面积校验 600–5000px，防夹爪黄件误录） |
| `watch.sh` | 值守：连续 2 次判定"被拿走"才触发；每轮等臂归位后重建参考掩码 |
| `auto_cycle.sh` | 一键循环：`board_frame --locate` → `pick_block` → `measure_size_median`(失败回退 33mm) → `grasp_auto` |
| `board_frame.py` | 棋盘定位/单应/仿射（重建坐标系：格 31.25mm、正交、`affine_source=drag-measured`） |
| `pick_block.py` / `measure_size_median.py` | 候选物块选择 / 多帧尺寸中位数 |
| `gripper_model.py` | FK + 爪口开度模型（`jaw_opening(rad)→mm`，max≈60.4mm@0.58rad） |
| `probe_cube.py` / `measure_cube_cells.py` | 触觉/棋盘格测物块真实尺寸的验证工具 |
| `stop_all.sh` / `deploy_watch*.sh` / `grasp_now.sh` / `verify_*.py` / `check_arm.sh` | 运维与验证辅助 |

## 关键参数（当前生效）

- 抓取偏置 `--y-offset-mm -20`（总偏置 = mesh 自动居中 + 额外 −20mm，实测 `total_offset_mm ≈ −9.8mm`）
- 抓取深度 `--grasp-depth-mm -48`（爪尖距桌面 ≈17mm，稳定值，勿改回 −38）
- `--gap-mm 0.5`、`--target-load 20`、`--extra-squeeze-rad 0.05`
- 动作已提速：下探/合爪/抬升节拍压缩约一半，移动改为连续插值（`smooth_move`），值守轮询 2s

## 修复的关键 Bug（本 milestone 内）

1. **固定爪尖端坐标系错误（−105mm）**：由 mesh z-`argmin` 改 `argmax` → 固定爪触点 `(1.0, 0, 6.3)mm`（与示教位验证误差 <2mm）
2. **棋盘格 ≠33mm**：拖拽实测 31.25mm，重建正交仿射（旧 33mm 假设导致各向异性伪影）
3. **方块尺寸假设错**：视觉量出 ~50mm 但按 33mm 计算 → 爪口过度张开抓空；现多帧中位数 + 回退 33mm
4. **"再收紧"用实测角反而张开**：方块把爪反推，实测角 > 命令角 → 改为按**命令角**加压
5. **归位张开掉方块**：先在 READY/放置点刻意松爪，再腕部摆正、爪张开回折叠位
6. **值守自激循环**：放完方块瞬间录参考把夹爪黄件录进去 → 改为**臂归位后 + 面积校验 600–5000px** 才重建参考；并加**连续 2 次确认**防抖
7. **USB 直通掉线**（相机 `/dev/video0` 消失、串口超时）：硬件层问题，重连后恢复（`usb_recover.sh` 可尝试软件恢复 cdc_acm）

## 验证结果（最后一次真实运行）

```
选中: (0.1960, -0.1093)  grid=(13.46, 11.40)cm  on_board=True
接触载荷 22.6% → 抬升 34.3mm 保持 30.6% ✅ 夹住
→ 放到 (0,0) → 归位 → 参考掩码重建 RESET_OK area=1883 ✅
→ 值守待命（不再自触发）✅
```

## 已知待办（非阻塞）

- `measure_size_median.py` 多帧采样仍返回 NONE（回退 33mm 已验证可用，但视觉实测 ~50mm，两者有 5.6% 单应单位偏差待统一）
- 抬升高度受 IK 可达范围限制（22–34mm，足够搬运）
- `servo_status.py` 桌面常数仍显示旧值 −69.09mm（真值 −64.85mm，仅外观）

## 设备拓扑

- VM：`ros2-ubuntu` 192.168.26.128，SSH 别名 `qianli-vm`
- 串口 `/dev/ttyACM0`（QinHeng 1a86:55d3）、相机 `/dev/video0`（ARC 05a3:9230，VMware USB 直通）
- 关节序：`[shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper]`
- TCP = `gripper_frame_link`；固定爪 `gripper_link`，活动爪 `moving_jaw_so101_v1_link`
