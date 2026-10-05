# SO-ARM101 仿真抓取验收 — 2026-10-05

已通过 SSH 在 `ros2-ubuntu` 修复并验证 MuJoCo 抓取。本轮验收范围为仿真，没有发送真机运动或使能指令。

## 结果

| 测试 | 结果 | 说明 |
|---|---:|---|
| 标准场景重复 | 10/10 | 20mm、8g 立方体，升高约 107mm，持稳 2 秒 |
| 调参样本，seed=101 | 40/40 | 首次 35/40；5 次通过第二次尝试恢复 |
| 独立样本，seed=20261005 | 39/40 | 首次 31/40；8 次通过第二次尝试恢复，1 次下降不到位终止 |
| 16g、摩擦系数 0.6 | 5/5 | 同一套抓取流程 |
| 步长从 2ms 改为 1ms | 5/5 | 升高约 109mm，持稳 2 秒 |
| 不可达目标 | 正确拒绝 | 预抓取 IK 残差约 872mm，退出码 1 |
| 目标低于棋盘 | 正确失败 | 下降误差约 25.3mm，退出码 1 |
| 交互窗口 | 通过 | 与命令行使用同一套物理步骤和判定 |

随机样本同时改变物块世界位置（每轴 ±5mm）、物块偏航（0–90°）和观测位置误差（每轴 ±2mm）。每个样本只在开始时复位一次；失败后的重试保留真实物理状态，不重置物块。

独立样本中成功的 39 次，持稳期间最低升高为 **104.3–116.3mm**，双侧有效接触占比最低 **99.1%**，三个运动阶段最大 TCP 误差 **3.42mm**。成功样本全程最大环境接触穿透为 **0.774mm**；标准场景为零。MuJoCo 使用软接触，因此验收允许小于 1mm 的穿透，不等同于全部随机样本都无接触。

剩余失败案例：物块偏移约 `(4.08, 1.32)mm`、偏航 `2.24°`、观测误差 `(1.79, 1.82)mm`。首次夹空，第二次下降误差 **9.53mm**，超过 4mm 限值后终止，没有继续闭爪。记录位于 `independent.json`。

## 修复内容

- 将夹爪、前臂与桌面/棋盘/底座的碰撞掩码配置移至 **MjSpec 编译前**。旧模型关闭了这些环境碰撞；编译后只修改 geom 掩码也可能与 body 掩码、碰撞结构不一致。
- 使用完整数组设置伺服 gain/bias，避免 Python 绑定中的数组副本问题。执行器显式启用控制和力矩限制，控制范围来自 URDF 关节限位。
- 机械臂和闭爪指令均按 **0.3rad/s 的目标变化率**渐变；实际关节速度由动力学决定，不能把目标变化率称为实际速度上限。
- 默认 TCP 相对物块中心偏移改为 **`(8, -4, +8)mm`**，夹爪接近角 **0.6rad**、闭合目标 **0rad**。较低的抓取高度会擦碰棋盘。
- 初始化物块中心改到棋盘坐标 **`(111, 20)mm`**，确保整个 20mm 立方体得到支撑。旧测试把中心放在棋盘边缘。
- 用抬升后持续持稳替代旧版“升高超过 5mm”的判定；不可达或阶段误差超限立即终止。
- 增加最多两次物理连续的反馈重试。候选偏移依次为 `(8,-4,8)`、`(8,-4,6)`、`(10,-6,8)mm`；重试先限速张开、等待落稳，再读取仿真物块实际位置，并加上同一观测误差。
- `sim_grasp_ok.py`、`sim_grasp_validate.py` 和 `view_grasp_ok.py` 共用验收流程；窗口按约 60Hz 刷新。

碰撞配置依据：[MuJoCo collision detection](https://mujoco.readthedocs.io/en/latest/computation/index.html#collision-detection)、[runtime model changes](https://mujoco.readthedocs.io/en/latest/programming/simulation.html#model-changes)。

## 成功判据

必须同时满足：物块相对下降前静止位置升高至少 **50mm**，持续 **2 秒**；至少 95% 的持稳步骤中，两侧夹爪都与物块有大于 **0.02N** 的法向接触；预抓取、下降、抬升的 TCP 误差都小于 **4mm**；全程环境穿透小于 **1mm**，关节越界小于 **0.01rad**，状态保持有限数值。

IK 只在临时 MjData 中求解。物理运行期间只改变 `ctrl` 并执行 `mj_step`，不瞬移机械臂或物块。关节与物块的 qpos 只在回合初始化时设置。

## 虚拟机复现

```bash
cd ~/QianLi/qianli_ws/src/qianli_vision/scripts

# 标准抓取，默认包含最多两次重试
~/mj/bin/python sim_grasp_ok.py --repeat 10 \
  --report ../../../log/grasp_validation/repeat.json

# 独立扰动测试；39/40 时退出码为 1，失败不会掩盖
~/mj/bin/python sim_grasp_validate.py --robust 40 --seed 20261005 \
  --report ../../../log/grasp_validation/independent.json

# 单次抓取，不重试
~/mj/bin/python sim_grasp_validate.py --retries 0

# 物理敏感性检查
~/mj/bin/python sim_grasp_validate.py --mass-g 16 --friction 0.6 --repeat 5
~/mj/bin/python sim_grasp_validate.py --timestep 0.001 --repeat 5

# Ubuntu 桌面窗口（保留当前桌面会话的 DISPLAY/XAUTHORITY）
MUJOCO_GL=glfw ~/mj/bin/python view_grasp_ok.py

# 已有 Xvfb :98 可用于离屏截图
DISPLAY=:98 MUJOCO_GL=glfw ~/mj/bin/python sim_grasp_validate.py \
  --render-dir ../../../log/grasp_validation/final_frames
```

MuJoCo **3.14.0**；依赖 `~/mj` 环境中的 NumPy、ikpy，渲染还需 OpenGL、OpenCV。模型仍使用现有 `~/legacy/arm/arm-final/ros2_ws/install/so101_bringup/share/so101_bringup/urdf/so101.urdf` 与 `~/mj_parts/manifest.txt` 的 CoACD 凸分解资产。换机器需提供这两套资产。

## 验证边界与记录

当前是 20mm 立方体的仿真验收。对象位置来自模拟状态加观测误差，没有验证真实相机识别；孪生中的质量、摩擦、伺服增益、转子惯量和力矩曲线仍是模型假设。模拟臂关节力矩上限 3N·m、夹爪 1.5N·m，并非实测曲线。

前臂使用原 URDF 网格的凸包与环境碰撞，夹爪使用 CoACD 凸分解；臂自碰撞、前臂与物块接触仍未建模。已经存在的 RL checkpoint 来自旧碰撞模型，不能用本次结果宣称它们已通过新模型验收。未经实测标定，不应直接把这套偏移和孪生成功率当作真机成功率。

JSON、日志和四阶段物理截图位于虚拟机与本地的 `qianli_ws/log/grasp_validation/`，该目录按项目规则不纳入 Git。主要代码修改保存在本地及虚拟机的 `qianli_vision/scripts/`。本轮未修改真实抓取脚本、舵机驱动、视觉标定或已有 RL 实现。
