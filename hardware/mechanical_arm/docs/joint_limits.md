# SO-ARM101 从臂关节限位记录（阶段 1 实测）

> 状态：阶段 1 单关节测试完成（2026-09-26）。**精确限位待测**：手动轻推每个关节到
> 机械极限，读 `03_read_states.py` 或 `01_ping_motors.py` 的 raw 值记录在下表，
> 作为阶段 2 STM32 固件限位表输入。

## 原始值约定

- 舵机 raw 范围 0–4095，中心 2048
- 角度换算：`(raw - 2048) × 360 / 4096`
- lerobot 0.3.4 Model Number = **777**（HX-30HM 报告值，映射 sts3215）

## 关节表

舵机硬件限位 = **厂家成品预烧写**（Min/Max_Position_Limit 寄存器，2026-09-26 读出），
无需手动扳动测量。URDF 限位是软件语义层，两层都保留。

| ID | 关节 | URDF 限位 (rad) | 硬件限位 min~max (raw) | 硬件限位（角度，相对 raw2048 中位） | Homing_Offset | MaxTorque |
|----|------|------------------|------------------------|-----------------------------------|---------------|-----------|
| 1 | shoulder_pan | ±1.920 (±110°) | 674 ~ 3260 | -120.6° ~ +106.6° | -363 | 1000 |
| 2 | shoulder_lift | ±1.745 (±100°) | 856 ~ 3266 | -104.8° ~ +107.0° | 645 | 1000 |
| 3 | elbow_flex | ±1.690 (±96.8°) | 787 ~ 3069 | -110.9° ~ +89.7° | -1751 | 1000 |
| 4 | wrist_flex | ±1.658 (±95°) | 838 ~ 3199 | -106.3° ~ +101.1° | -577 | 1000 |
| 5 | wrist_roll | -2.744~2.841 | 505 ~ 3987 | -135.7° ~ +170.3° | -1215 | 1000 |
| 6 | gripper | -0.175~1.745 | 2034 ~ 3233 | -1.2° ~ +104.2° | -334 | **500**（厂家减半防夹坏） |

> Homing_Offset 非零 = 厂家已做零位校准。阶段 2 STM32 软件限位直接采用上表 raw 值
> （外扩 2% 安全边距）。lerobot-calibrate 时会另建 homing 标定文件，与此独立。

## 阶段 1 测试结论（2026-09-26）

| 项目 | 结果 |
|---|---|
| 协议兼容性 | HX-30HM = Feetech SCS/STS 兼容，Model Number 777，默认 **1 Mbps** |
| 6 关节 ±10° 往返 | 全部通过，跟随误差 0.2°–1.6°（gripper 齿轮间隙正常） |
| 30s 连续读取 | 1650 次 0 失败 |
| 电压 | 12.0–12.3V（12V 5A 适配器） |
| 温度 | 36–39℃ 空载；连续运动后需复测 |
| 通信链路 | 控制板 USB 芯片 CH343P（ServoDebugger 板），VM 内枚举为 /dev/ttyACM0 |

## 固件特性（重要，写 STM32 固件时遵守）

1. **WRITE 指令执行但不回状态包**（Response_Status_Level 出厂值改写无效）→
   主机写指令一律 **fire-and-forget**，不得等待应答
2. **SYNC_WRITE (0x83) / GroupSyncRead (0x84) 工作正常且稳定** → 优先使用
3. 寄存器表与 Feetech SMS/STS 官方一致（十进制地址）：Torque_Enable=40、
   Goal_Position=42(2B)、Present_Position=56(2B)、Present_Voltage=62、
   Present_Temperature=63、Lock=55
4. 写后紧接读之前建议清空串口输入缓冲（否则残留字节导致流错位）

## lerobot 0.3.4 补丁（VM: /home/ros/lerobot-venv）

`site-packages/lerobot/motors/motors_bus.py` 的 `_write()`：
`packet_handler.writeTxRx` → **`writeTxOnly`**（备份：motors_bus.py.orig）。
原因同上第 1 条。重新安装 lerobot 后需重打。
