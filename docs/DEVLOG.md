# DEVLOG — 开发日志

> 按时间顺序记录 QianLi 开发过程、决策与验证结果。

## 2026-10-06（凌晨）— 真机根源：shoulder_lift 舵机编码器跑飞 + RL 结果

### 🎯 真机所有怪现象的根源：ID2 舵机位置计数跑飞

用户提出"要不要重新确定一下机械限位" —— **直觉命中**。
写了 `read_servo_limits.py`，直接按 Feetech 协议读 6 个舵机的
原始位置 / 错误标志 / 温度 / 电压 / 负载，与 `driver_params.yaml`
的 zero_raw/raw_min/raw_max 对比：

| ID | 关节 | 位置 | 配置范围 | 状态 |
|---|---|---|---|---|
| 1 | shoulder_pan | 2282 | [826,3330] | ✅ |
| **2** | **shoulder_lift** | **33413** | [842,3118] | ❌ **超限 +30295，错误标志 0x02** |
| 3 | elbow_flex | 2553 | [1974,4095] | ✅ |
| 4 | wrist_flex | 2580 | [954,3116] | ✅ |
| 5 | wrist_roll | 1264 | [1264,4095] | ✅ |
| 6 | gripper | 2027 | [1916,3168] | ✅ |

**连续 6 次读数稳定 33413（有符号 −32123）** —— 不是读错，是编码器多圈计数跑飞。
温度 36~40°C、电压 12V 都正常 → 不是硬件烧毁，是计数丢失。

**连锁反应**（这才是真机不稳的真因）：
- 驱动反复 `servo 2 reported error flags 0x02` → 卡在 reconnect pending
- 驱动见错误标志 → **安全失能**（表现为"机械臂突然不动了"）
- shoulder_lift 是主承重关节 → 动作走不准、抓取中途失败

**修复**：断电 5 秒重启 → 上电时绝对编码器重新初始化 → ID2 恢复为 2742，
6 个舵机全部恢复正常（33~37°C / 12V / 无错误）。

> 踩过的协议坑：Feetech 读指令的 `LENGTH = 参数个数 + 2`（读 2 字节 → 4），
> 一开始写成 3，导致所有舵机"无应答"。

### 视觉三处修正（"检测效果变差"的真因）

| 现象 | 真因 | 修正 |
|---|---|---|
| 检测不到物块 | `max_sat` 从 127 收到 60，**把物块自己滤掉了** | 改回 120 |
| 标定 100% 失败（1734 次） | 停靠位把机械臂停在棋盘上方**挡住棋盘** | 用"标定成功次数"当指标自动测出 (0.06,0,0.26) |
| 误报"越界" | **网格原点是棋盘左上角不是中心**，却按 ±14cm 判 | 改成 0~26 × 0~19.5cm |

### 真机 IK 与稳定性

- `patch_ik_node.py` 给驱动打 DLS IK 补丁：抓取点到位误差 **1.9~2.2mm**
  （同一目标 ikpy 之前判"不可达，残差 386.9mm"）
- `real_grasp_ok.py`：10Hz 心跳（避免 0.5s 指令看门狗失能）
  + 监视 `/arm/status` 自动重新使能（实测生效）
- `driver_params.yaml`：`state_rate 5→2`、`command_write_rate 8→4`、
  `command_timeout 0.5→1.0`（降串口流量）

### 🤖 强化学习结果（子代理完成）

**结论：纯 PPO 学不出来，必须行为克隆热启动。**

| 方法 | 成功率 |
|---|---|
| 纯 PPO 从零（4 次尝试、最高 150 万步）| **0%**（entropy 卡在最大值）|
| 行为克隆 BC（320 条专家示范 / 24k 样本）| **90%** |
| PPO 微调（BC 热启动，60 万步）| **随机位姿 85% / 固定位姿 75~100%** |

**为什么纯 PPO 不行**（实测成功区宽度）：在专家动作上加高斯噪声
σ=0.10 → 16/20，σ=0.20 → 4/20，σ=0.40 → **0/20**；
均匀随机探索 ≈ σ0.6~1.0 → **永远摸不到成功区**。

**踩过的观测设计坑**：缺"回合起手时物块在哪"和"步数"特征 → 网络分不清
"该下压"还是"该闭爪"，BC 0%；补上后 BC 直接 90%。

产物：`rl_env.py`（Gymnasium + MuJoCo 全保真 CoACD 夹爪）、`rl_collect_demos.py`、
`rl_bc.py`、`rl_train.py`（PPO/SB3）、`rl_eval.py`、`rl_out/`（TB 日志 + checkpoint）。

### 当前阻塞

舵机电源未接通（连续 3 轮 ping 6 个舵机全部无应答；USB 适配器正常）。

## 2026-10-06 — 抓取终于真正可用（找到真凶：IK 瞬移）✅✅

### 真凶：IK 直接改 `data.qpos` = 瞬移，会绕过约束求解器

我此前把 IK 的结果**直接写进 `data.qpos`**（因为"解出来就摆过去"很自然）。
后果：机械臂瞬移 → **已夹住的物块被挤掉** → 抬升时物块纹丝不动。

**决定性验证**（人為摆好 + 夹紧 + 抬起，两种方式对比）：

| 抬升方式 | 结果 |
|---|---|
| 每步改 `qpos`（瞬移） | ❌ 物块升高 **-0.7mm**（拿不住） |
| **只发 `ctrl`，伺服自己动** | ✅ 物块升高 **+145.6mm**（一路跟着走） |

夹持接触力实测 **0.55~0.67 N/接触点**（11 个接触）—— 物理本来就没问题。

### 可用抓取流程（孪生里 20mm 物块，实测通过）

```
TCP 目标 = 物块中心 + (dx, dy, 0)mm      dx ∈ [4,12], dy ∈ [-8,0]（dy=-4 最稳）
接近角  ≈ 0.6（此角度两指尖接近同高）
闭合 → settle 800 步（等伺服真正合拢）→ 抬起
全程只发 ctrl，绝不瞬移 qpos
```

**验证结果**：
- 重复性：同参数 **3/3 抓起**，升高 +104.9mm
- 鲁棒性：20 组偏移 **10/20 成功**（dy=-8 与 dy=-4 两行 **10/10 全成功**）
- 抬起时夹爪角稳定在 +0.009，接触保持 7~14 个
- 爪子最低点 -0.0528（棋盘 -0.0494）→ 仅贴桌面，不穿模

### 本会话踩过的坑（全部写进代码注释）

1. MuJoCo 对 MESH 碰撞**用凸包** → 凹爪子被填平，必须用 **CoACD 凸分解**
2. compile 后改 `model.geom_pos`、MjSpec 里改 `gainprm` **都不生效**
3. 脚本开头 `sys.argv=[argv[0]]` 会**清掉 main() 的参数**
4. **IK 瞬移**（本页主题）
5. 闭合斜坡跑完时伺服仍滞后 → 必须加 **settle**
6. 接触过滤要用 **geom id**（不是 body id）；`contact.frame` 要 `reshape(3,3)`
7. `ikpy` 定位精度仅 4~50mm 且会发散 → 换 **MuJoCo 雅可比 DLS IK**（0.1mm）

### 复现命令

```bash
~/mj/bin/python sim_grasp_ok.py --obj-size 0.020 --repeat 3   # 3/3 抓起
~/mj/bin/python sim_grasp_ok.py --obj-size 0.020 --sweep      # 鲁棒性 10/20
~/mj/bin/python sim_hold_test.py --obj-size 0.020             # 物理夹持验证
~/mj/bin/python view_grasp_ok.py                              # 桌面实时窗口
```

## 2026-10-05（深夜）— 孪生里训练出可用抓取策略 ✅

### 根因：TCP 参考点比爪口低 3.4cm

在孪生里逐层测量（世界坐标、工具朝下）：

| 部位 | z (相对 TCP) |
|---|---|
| `gripper_frame_link`（我们一直当抓取点用） | 0 |
| 活动爪尖端 | **+3.38 cm** |
| 固定爪最低点 | **+9.81 cm** |

**即爪口在 TCP 上方 3.4cm** —— 把 TCP 对准物块中心时，爪子其实在物块
**上方 3.4cm 的空中**，所以每次都"夹空"、并把物块推走。
这解释了真机与仿真里**全部**的夹空现象。

### 训练出的策略（孪生验证 100% 成功）

| 参数 | 值 | 依据 |
|---|---|---|
| TCP 目标高度 | 物块中心 **-30mm**（沿工具轴） | 扫描：dz∈[-20,-40] 均可，-30 最稳 |
| 物块尺寸 | **12~16mm** | 10mm 夹不住；18/20mm 塞不进爪口 |
| 横向对齐 | 单次仅 ±2mm（X）/ ±1mm（Y） | 爪口 19.6mm vs 物块 14mm，余量小 |
| **二维螺旋搜索** | 1mm 步长、半径 4mm、81 点 | 硬案例全部命中（含需 67 次试的 (4,4)） |
| 夹爪力 | kp=20（力矩 ±3 N·m） | 真舵机堵转约 2.9 N·m |

**成功率**：
- 单向螺旋（±8mm，9 点）：±6mm 误差下 **11/11 = 100%**
- 二维螺旋（2mm 步长，13 点）：2D 误差下 10/11 = 91%
- **二维螺旋（1mm 步长，81 点）：硬案例 (3,3)(-3,-3)(3,-3)(-4,4)(4,4)
  全部成功 ✅**

**关键结论**：**2cm 物块物理上抓不了**（爪口内侧仅 19.6mm）——
真机想抓成功，**必须换 ≤16mm 的物块**。

### 复现命令

```bash
~/mj/bin/python sim_policy.py --list     # 打印策略参数
~/mj/bin/python sim_policy.py            # 验证鲁棒性（当前 11/11）
~/mj/bin/python sim_grasp.py --scan-size # 复现物块尺寸窗口
```

### 待办（真机落地时）

- [ ] 换一个 **14mm 左右的物块**（关键前提）
- [ ] 把 `dz=-30mm` 写进 `auto_grasp.py` 的抓取目标
- [ ] 加螺旋搜索重试（每次抓失败后偏移 ±2/±4mm 重试）
- [ ] wrist_roll 编码器回绕截断 65.6° 行程（真机）+ USB 走 xHCI

## 2026-10-05（夜）— 数字孪生搭建（MuJoCo）与四大坑

### 目标

用户要求：**不再动真机，先在数字孪生里训练出可用夹取策略**。

### 成果 1：孪生保真度 1mm

MuJoCo 3.14（装在 `~/mj` venv）+ 真机 URDF + 实测场景几何
（桌面 z=-0.0524、5cm 底座、棋盘 22.8×16.2cm @ -97.75°、2cm 物块）。

**验证**：把真机读到的关节角喂给 MuJoCo，TCP 与真机 TF 对比：

| | TCP (base_link) |
|---|---|
| 真机 | (0.161, 0.009, 0.103) |
| MuJoCo | (0.1605, 0.0090, 0.1026) |
| 差异 | **0.5mm / 0mm / 0.4mm** ✅ |

### 成果 2：找出并修复 4 个隐藏坑（否则仿真完全不可用）

1. **MjSpec 里改 `actuator.gainprm` 不生效** → 关节完全不跟随。
   必须在 `spec.compile()` **之后**改 `model.actuator_gainprm[i]`。
2. **MjSpec 从 URDF 载入后，`add_body` 加的静态 body 会被 compile 丢弃**
   → 桌面/底座/棋盘改用 `worldbody.add_geom()` 直接挂。
3. **MuJoCo 载入 URDF 时会自动加一个地板平面（z=0）**，正好穿过机械臂
   基座 → 12 个接触、约束力 **75 N·m**，伺服被顶住完全跟不动。
   → 把所有无名 world geom 的碰撞关掉（我们有自己的桌面）。
4. **数值发散**：URDF 连杆转动惯量仅 ~1e-4 kg·m²，配 kp=120 时
   自然频率 ω=√(kp/I)≈1000 rad/s，**远超 2ms 步长的奈奎斯特频率** →
   wrist_roll 在 -2.8~+1.7 rad 之间疯狂振荡。
   → 加**转子惯量 armature=0.02**（真舵机减速箱惯量），MuJoCo 官方推荐做法。

修复后：全部关节跟随误差 ≤0.006 rad，**TCP 误差 1mm** ✅

### 成果 3：关键几何发现

- 爪尖相对 `gripper_frame_link` 沿工具轴偏 **+7.3mm**（URDF 网格精算）
- **两爪内侧开度仅约 19.6mm** —— 2cm 物块正好卡在极限！
  这解释了真机反复"夹空"
- 抓取时 `orientation_mode='Z'` 只保证工具轴朝下，**爪口朝向（滚转）自由**
  → 物块常不在爪口平面内，从旁边擦过

### 待办（下一轮继续训练）

- [ ] 仿真里夹爪能碰到物块（滚转扫描有 2~6 个接触）但**夹不住、物块被推走**
      → 需调：下压深度、闭合时机/速度、夹爪力、接近偏移
- [ ] wrist_roll 回绕截断（真机正向少 65.6° 行程）→ 重标定 raw 范围或改
      driver 支持跨圈范围
- [ ] 建"搜索最优抓取参数"的训练循环（滚转 × 高度 × 偏移）

## 2026-10-05（晚）— 首次真机自动抓取 + 串口稳定性攻坚

### 里程碑：机械臂首次自主抓取成功

`auto_grasp.py` 一条命令完成：归位 → 视觉定位 → 张开 → 三段接近 → 闭合 → 抬起。
真机实测（分步下压 6 段，误差 3.7~6.0mm，抬起后夹爪关节停在 0.063 rad = 夹住物块）。

### 抓取高度标定（三条独立证据交叉验证）

| 证据 | 物块中心 z | 说明 |
|---|---|---|
| 拖动示教记录 | -0.0656 | 松扭矩后机械臂先下垂，偏低 2.3cm ❌ |
| 用户实测"最低端离桌面 5cm" | -0.0424 | URDF 算最低点仅比 base_link 低 0.24cm |
| 实际抓成功时的 TCP z | -0.0428 | 与上一行只差 0.4mm ✅ |

结论：**桌面 z ≈ -0.0524 m**，物块中心 ≈ -0.0424 m。后两者互证，示教值废弃。

### 抖动根因（控制问题）

`tracking_integral_gain=3.0` 导致极限环振荡：TCP 峰峰 1.9mm。
调为 0.0 后**完全静止（峰峰 0.0000）**，代价是位置均值偏移约 2mm（原本补偿抵消的重力下垂）。
已固化到 `driver_params.yaml`（**注意：install 副本也要改**，launch 读的是 install 里的）。

### 抓取策略升级（用户指定，工业标准）

1. 夹爪**完全朝下** 2. **固定爪在物块右侧** 3. 活动爪从左侧**慢速合上**（自定心）

关键发现：ik_node 的 **`orientation_mode` 默认 `'none'`** → **完全忽略姿态**，
所以只发位置时爪子朝向随机，必然夹空。改为 `'Z'` 后：

- 姿态受控：工具轴 (0.025,-0.001,-1.0) = 垂直向下 ✅（误差 5.86°）
- 固定爪方向 (-0.099,-0.995,-0.001) = base -Y = 右侧 ✅
- 活动关节从 4 个变成 5 个（含 wrist_roll），5 DOF 匹配"位置3+姿态2"约束

已改：`ik_demo.launch.py`（src + install 两份）加 `'orientation_mode': 'Z'`；
`tool_pose.py` 算四元数并支持分步走（IK 是局部优化，需好种子）。

### 主要障碍：USB 串口反复假死（硬件层）

`1a86:55d3` (CH343) 挂在 **USB 1.1 UHCI 控制器**（12Mbps 跑 1Mbps 协议）：
- 每 ~10 分钟假死一次，运动中更频繁
- driver 重连后**安全禁用运动**（`Hardware motion is disabled; target ignored`）
  → 长脚本必然中断，之前的"下压 30mm 差""抬起 147mm 差"都是这个原因
- 已采取措施：① `arm_usb_watchdog.py` 自动检测+USB复位+驱动重绑（日志可查）
  ② 降低串口速率 control_rate 30→15、state_rate 10→5、command_write_rate 15→8
  ③ `auto_grasp.py` 检测到禁用会自动重新使能（最多 3 次）
- **根治**：把 VM 的 USB 控制器兼容性从 USB 1.1 改为 **USB 3.1 / 2.0**（需关机改）

### 新增脚本

| 脚本 | 作用 |
|---|---|
| `auto_grasp.py` | 一键抓取（姿态受控 + 慢速闭合 + 串口自愈） |
| `tool_pose.py` | 算"爪子朝下+偏航"四元数，验证姿态，支持分步走 |
| `teach_grasp.py` | 拖动示教抓取高度 |
| `calib_grasp_offset.py` | 标定"视觉位置→夹爪实际位置"偏移 |
| `z_jog_gui.py` | Tkinter Z 微调前端 |
| `arm_move.py` | 精确移动工具（支持 --enable/--wait） |
| `measure_jitter.py` | 抖动量化（控制参数对照实验） |
| `capture_background.py` | 拍背景图（背景差分检测） |
| `arm_usb_watchdog.py` | 串口看门狗（自动复位 USB） |

### 待办

- [ ] **改 VM USB 控制器为 USB 3.x**（根治串口假死）
- [ ] `max_ik_residual_m` 默认 3mm 偏严，姿态约束下常拒绝；考虑放宽到 6~8mm
- [ ] 姿态约束后工作空间变小：park 点需重选（原 (0.26,0.01,0.18) 被拒）
- [ ] 用舵机 `Present Load`（寄存器 60，driver 未读）做接触检测，自动找物块顶面
- [ ] 抓取偏移标定（`calib_grasp_offset.py`）跑一次，消除 TCP 与夹取中心偏差

## 2026-10-05（下午）— 全自动棋盘格定位 + 机械臂串口急救 + 外参标定

### 1. 自动调参（替代手动滑块）

用户要求滤除"长宽差 > 5"的候选、面积限定 330~350。据此写 `autotune.py`
遍历 HSV 阈值网格，自动搜出最优参数：

| 参数 | 值 | 依据 |
|---|---|---|
| S_MAX | 127（100~155 等效） | 自动搜索：饱和度不区分物块/背景 |
| V_MIN | 140 | 143 时面积掉到 330 边界，140 有余量 |
| V_MAX | 167 | 175 时面积涨到 372 超出区间 |
| 面积 | 330~350 | 用户指定 |
| 长宽差 | ≤5 | 用户指定（立方体投影近似正方形） |
| 边长 | 20~50 | 滤掉长条杂质（如 15×72） |

**实测**：全画面无 ROI 也只命中唯一候选（area=339, 24×24）→ ROI 不再必要。

### 2. 关键发现：标定板是黑白棋盘格，改用角点标定

原 Hough 格线法的问题：格线聚类易漏线/多线，原点不确定，
把物块算到 **X=28.9cm（超出棋盘 23.1cm 边界，明显错误）**。

改用 `cv2.findChessboardCorners`（内角点 7×5 = 8×6 方格）：
- **重投影误差 平均 0.08cm、最大 0.65cm**（亚像素 cornerSubPix）
- 原点 = 棋盘左上第一个内角点，**在画面上用红圈标出**，彻底解决"哪个是原点"
- 物块 → **(13.0, 6.5) cm**，落在棋盘范围内，合理
- 每 3 秒自动重标定，被挡时不覆盖旧 H（抓取途中遮挡不影响）

### 3. 机械臂串口"Write timeout"急救（重要经验）

症状：driver 报 `servo reconnect pending: Write timeout`，
/joint_states 无数据，RViz 机械臂不动。

诊断（`arm_serial_probe.py`）：**连写 8 字节都超时** → 不是舵机问题，
是 USB CDC 端点假死。设备 `1a86:55d3`（CH343）走通用 `cdc_acm` 驱动。

修复（需要 sudo，已验证有效）：
```bash
# 1) USB 复位
sudo python3 -c "import fcntl; fd=open('/dev/bus/usb/001/002','wb'); fcntl.ioctl(fd,0x5514,0)"
# 2) 重绑驱动
echo -n '1-1:1.0' | sudo tee /sys/bus/usb/drivers/cdc_acm/unbind
echo -n '1-1:1.0' | sudo tee /sys/bus/usb/drivers/cdc_acm/bind
```
修复后：写入 1.4ms，**6 个舵机全部响应 ping**，driver 正常
（`Connected to six servos; torque disabled`）。

### 4. 外参标定（grid → base_link）

`extrinsic_calib.py`：实时记录 TCP（`gripper_frame_link`）位置，
SSH 侧 `touch /tmp/mark_A` 打点，无需用户在终端操作。
两点法（原理：一点定位置，两点定朝向）：

| 点 | grid 坐标 | base_link (m) |
|---|---|---|
| A | (0, 0) | (0.3420, 0.0584, -0.0310) |
| B | (3.3, 0) | (0.3374, 0.0241, -0.0405) |

结果：`grid_origin=(0.3420, 0.0584, -0.0310)`，`θ=-97.75°`
校验：A→B 实测 3.46cm vs 理论 3.3cm（+0.16cm，含手搬误差 → 待用尺子核格宽）

### 5. 抓取桥接验证

`grab_bridge`：自动读 `/tmp/extrinsic.txt` → 旋转+平移 → 发布
`/arm/target_position`（base_link）。oneshot 模式锁存首个稳定目标
（连续 3 帧 <5mm），避免抓取途中物块被机械臂遮挡导致目标中断。

模拟验证：grid (13.0, 6.5)cm → base (0.3889, -0.0792, 0.0190)m，
与手算一致；oneshot 只发送一次 ✓

### 已知问题 / 下一步

- [ ] 机械臂下压会遮挡棋盘（眼在手外固有）→ 先归位标定，再抓取
- [ ] 用尺子量棋盘整宽（8 格）校准格宽；量物块尺寸定抓取高度
- [ ] 使能真实运动：`allow_motion:=true calibrated:=true` + `/arm/enable`
- [ ] 首次抓取测试（低速 max_joint_speed:=0.3，随时 /arm/stop）

## 2026-10-05 — RGB 相机物块定位 + 抓取链路（视觉学习/教学）

### 背景

用户目标：用普通 RGB 相机让机械臂自己抓取小物块。环境：VM（Ubuntu 24.04 + Jazzy，
12 核/8G，已调优）+ USB 相机（ARC International，640x480）+ 3.3cm 网格纸标尺 + 灰色立方体物块。

### 学习过程（教学脚本迭代）

| 版本 | 脚本 | 教学点 |
|---|---|---|
| v1 | detect_demo.py | 边缘/轮廓检测（Canny+approxPolyDP），发现灰方块不稳 |
| v2 | detect_demo.py(v2) | 自适应 Canny + 形态学 + 平滑，仍未解决对比度问题 |
| v3 | detect_gray.py | **HSV 灰色识别**：S 低 + V 适中，5 窗口可视化 |
| v4 | detect_gray_roi.py | ROI 排除灰色桌面干扰 |
| v5 | detect_gray_trackbar.py | 滑块实时调参 + 参数自动落盘 |
| v6 | detect_gray_multi.py | 多物块 + mask 半透明叠加 |
| v7 | detect_diagnose.py | 诊断：显示所有轮廓 + 面积落盘 → 定位"过滤吞物块" |
| v8 | mask_diagnose.py | 尺寸过滤（30~50px） |

### 关键 bug 与修复

1. `cv2.HoughLinesP` 新版返回 (N,4) 而非 (N,1,4) → `lines[:,0]` 解包崩溃 → 兼容 reshape
2. 物块实际 24x25px（391px 面积），但 MIN_AREA=602 把它滤掉 → 降到 100 + 尺寸范围过滤
3. selectROI 阻塞 rclpy executor → 独立线程做初始化

### 标定验证（成功）

- 自动网格标定：行线 6 条、列线 8 条，Homography（RANSAC）成功
- 物块定位：X=26.2cm Y=25.6cm（像素 306,291），读数稳定一致
- 物块尺寸：24×25 px（很小，注意 30px 下限）

### 代码成果

- `qianli_ws/src/qianli_vision/`（新包，构建通过）
  - `object_localizer`：检测 + 标定 + 发布 /object_pose（grid 系）
  - `grab_bridge`：grid → base_link 外参变换 → /arm/target_position
  - scripts/：8 个教学脚本

### 待办（下一步）

- [ ] 用户确认物块尺寸过滤参数（30~50px 还是放宽）
- [ ] 外参标定：机械臂末端碰网格纸原点，记录 base_link 坐标
- [ ] 接 ik_node 实际抓取测试（先 sim 后 direct）

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
