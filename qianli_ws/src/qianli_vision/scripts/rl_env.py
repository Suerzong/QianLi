#!/usr/bin/env python3
"""机械臂抓取立方体 —— 快速 Gymnasium 环境（纯仿真，绝不碰真机）

设计依据（全部本会话实测，不是猜的）：

1. **碰撞体**：一开始按任务要求把夹爪换成两片盒子爪，但实测
   sim_gripper_fix.py 那套摆位是错的（活动爪中心比固定爪高 26~37mm），
   而且盒子爪是刚性固定指 —— 60mm 的下压过程会把物块**铲飞**
   （实测物块从 (328,-52) 被推到 (386,-92)mm，然后永远夹不到）。
   改用引擎量出的铰链运动学重算摆位后仍然铲飞，因为固定指就在
   物块旁边 4mm，下压必定相撞。

   实测 CoACD 68 块凸分解模型：**build 0.26s、107,000 steps/s**
   （比任务里"15 秒/次"的估计快 60 倍，也远高于 2000 steps/s 目标），
   而且它就是已验证可用的那个孪生（20mm 物块抬起 +104.9mm、14 个接触）。
   → 所以这里**直接用全保真网格夹爪**，速度反而绰绰有余。
   两片盒子爪的那套换算代码保留在 probe_gripper_fast.py / tune_fingers.py 里
   供参考。

2. **场景常量**全部来自 sim_grasp.py（桌面 z=-0.0524、棋盘、底座、
   物块位置 obj_world_pos()），关节/执行器索引来自 attach_handles()，
   保证与已验证的孪生一致。

3. **只发 ctrl**：任何地方都不直接写 data.qpos 去"瞬移"机械臂
   （项目里抓取一直失败的真凶）。qpos 只在 reset 时写物块初始位姿。

观测（19 维）：
   [0:5]   5 个臂关节角 (rad)
   [5]     夹爪关节角 (rad)
   [6:9]   TCP→物块 的向量，在 TCP 坐标系下 (m)
   [9]     物块相对 TCP 的偏航角 sin
   [10]    物块相对 TCP 的偏航角 cos
   [11]    物块底面离桌面高度 (m)
   [12]    TCP 相对"物块中心"的高度 (m)
   [13:16] 5 关节指令与实到角之差（伺服滞后）的前 3 个... 见下
   [13:19] 6 个关节的 (ctrl - qpos) 滞后 (rad)，让策略知道伺服还没到位

动作（6 维，绝对目标角增量）：
   [0:5]  5 个臂关节在**上一帧关节角**基础上的增量（× 0.15 rad 上限）
   [5]    夹爪目标角增量（× 0.6 rad 上限）

奖励：
   · 接近：-2.0 * ||TCP→物块||（按接近偏移 off=(8,-4,0)mm 算）
   · 两侧接触：每侧 +0.5（要求接触力 > 0.05N）
   · 夹持成形（两侧同时接触且夹爪角 < 0.35）：+1.0
   · 抬起：+8.0 * clamp(物块升高/0.05, 0, 1)
   · 成功：物块升高 > 5cm 且两侧接触 → +10（回合提前结束）
   · 时间惩罚：-0.01/步
   · 掉出桌面 / 撞穿桌面：-1 并结束

用法：
  ~/mj/bin/python rl_env.py --check        # 自检：随机策略跑几个回合
  ~/mj/bin/python rl_env.py --speed        # 测 env.step 吞吐
  ~/mj/bin/python rl_env.py --baseline 20  # 随机策略成功率
"""

import argparse
import math
import os
import time

import numpy as np

import mujoco

import sim_grasp as S
import sim_mesh_gripper as MG
import sim_ik_dls as IK

try:
    import gymnasium as gym
    from gymnasium import spaces
except ImportError as e:      # pragma: no cover
    raise SystemExit(f'需要 gymnasium: {e}')

# ------------------------------------------------------------------ 常量
OBJ_SIZE = 0.020                 # 20mm 立方体（已验证可抓）
TABLE_Z = S.TABLE_Z              # -0.0524
# 已验证的抓取参数（sim_grasp_ok.py，20mm 物块 3/3 成功）
GRASP_OFF_MM = np.array([8.0, -4.0, 0.0])   # TCP 相对物块中心（mm）
GRASP_OFF = GRASP_OFF_MM / 1000.0           # 同一偏移，单位米
APPROACH_ANG = 0.6               # 接近/预抓取时的夹爪角
GRIPPER_CLOSED = 0.0
GRIPPER_OPEN = 1.2               # 全开 1.7453，1.2 口宽 ~26.5mm
# 随机化范围（绕标称物块点）
OBJ_JITTER = 0.010               # ±10mm（标称点 = obj_world_pos()）
OBJ_YAW_JITTER = math.radians(180.0)   # 立方体各向同性，纯姿态多样
LIFT_SUCCESS = 0.05              # 5cm 判定成功
MAX_STEPS = 150                  # RL 步数上限
N_SUBSTEPS = 10                  # 每个 RL 步的物理子步数 (10 × 2ms = 20ms)
DT_CTRL = 0.02
ARM_STEP_MAX = 0.15              # rad / RL 步
GRIP_STEP_MAX = 0.6              # rad / RL 步
CONTACT_FORCE_MIN = 0.05         # N，判定"真接触"

OBS_DIM = 19
ACT_DIM = 6

# 手臂初始位（预抓取）：物块上方 60mm，TCP 对准 off
PRE_GRASP_DZ = 0.06


def build_model(obj_size=OBJ_SIZE):
    """建场景（全保真网格夹爪）。"""
    S._OBJ_SIZE_OVERRIDE[0] = obj_size
    return MG.build(obj_size)


class GraspEnv(gym.Env):
    """机械臂抓取 20mm 立方体 —— MuJoCo 数字孪生，纯仿真。"""

    metadata = {'render_modes': []}

    def __init__(self, obj_size=OBJ_SIZE, max_steps=MAX_STEPS,
                 jitter=OBJ_JITTER, seed=None, randomize_obj=True,
                 reward_scale=1.0):
        super().__init__()
        self.obj_size = obj_size
        self.max_steps = max_steps
        self.jitter = jitter
        self.randomize_obj = randomize_obj
        self.reward_scale = reward_scale

        self.model = build_model(obj_size)
        self.data = mujoco.MjData(self.model)
        self._handles()
        self._nominal = S.obj_world_pos().copy()
        self._rng = np.random.default_rng(seed)

        # 臂关节限位（用于安全裁剪）
        joints = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, j)
                  for j in S.ARM_JOINTS]
        self.jlo = np.array([self.model.jnt_range[j][0] for j in joints])
        self.jhi = np.array([self.model.jnt_range[j][1] for j in joints])

        # 预抓取关节角（在"标称物块位置上方 60mm"处解一次 IK，作为
        # 每回合的固定起点；不同物块位置下策略靠观测自己纠偏）
        self.q_home = self._ik(self._nominal + np.array(
            [0, 0, PRE_GRASP_DZ]) + GRASP_OFF)

        self.action_space = spaces.Box(-1.0, 1.0, (ACT_DIM,), np.float32)
        self.observation_space = spaces.Box(-np.inf, np.inf, (OBS_DIM,),
                                            np.float32)
        self._prev_ctrl = np.zeros(6)
        self._fbuf = np.zeros(6)
        self._obuf = np.zeros(OBS_DIM, np.float32)
        self._phase_contacts = np.zeros(2)
        self.ep_steps = 0
        self.ep_obj0 = None
        self.last_info = {}

    # ---------------------------------------------------------- 句柄
    def _handles(self):
        m = self.model
        self.qadr, self.aadr, self.obj_body, self.obj_q = S.attach_handles(m)
        self.gl = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY,
                                    'gripper_link')
        self.mj_body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY,
                                         'moving_jaw_so101_v1_link')
        self.cube_geom = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'cube')
        self.arm_bodies = {mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
                           for n in ['gripper_link',
                                     'moving_jaw_so101_v1_link']}
        # 伺服执行器到关节的顺序
        self.act_joint = []
        for i in range(m.nu):
            self.act_joint.append(mujoco.mj_id2name(
                m, mujoco.mjtObj.mjOBJ_ACTUATOR, i).replace('servo_', ''))
        self.arm_act = [self.aadr[j] for j in S.ARM_JOINTS]
        self.grip_act = self.aadr['gripper']
        # --- 热路径预计算（step() 每步都跑，尽量少 Python 层循环）---
        self._arm_q = np.array([self.qadr[j] for j in S.ARM_JOINTS])
        self._arm_a = np.array([self.aadr[j] for j in S.ARM_JOINTS])
        self._gq = self.qadr['gripper']
        self._ga = self.grip_act
        # 执行器顺序里各关节的位置
        self._act_order = np.array([self.qadr[j] for j in self.act_joint] +
                                   [self._gq])
        self._qidx6 = np.concatenate([self._arm_q, [self._gq]])

    # ---------------------------------------------------------- 工具
    def tcp(self):
        R = self.data.xmat[self.gl].reshape(3, 3)
        return self.data.xpos[self.gl] + R @ S.FRAME_IN_GRIPPER, R

    def _ik(self, target, seed_qpos=None):
        """在**临时** MjData 里解 IK，绝不改主仿真状态。"""
        tmp = mujoco.MjData(self.model)
        if seed_qpos is not None:
            tmp.qpos[:] = seed_qpos
        q, err = IK.ik_dls(self.model, tmp, self.qadr,
                           np.asarray(target), seed=None)
        return q.copy()

    def _contacts(self):
        """返回 (固定侧接触数, 活动侧接触数, 最大接触力)。

        只有 3 个 geom 可能和物块碰：固定侧网格零件、活动侧爪、
        桌面/棋盘。先按 geom 对筛出候选，再只对候选算接触力。
        """
        nf = nm = 0
        fmax = 0.0
        f = self._fbuf
        for c in range(self.data.ncon):
            cn = self.data.contact[c]
            g1, g2 = cn.geom1, cn.geom2
            if g1 == self.cube_geom:
                o = g2
            elif g2 == self.cube_geom:
                o = g1
            else:
                continue
            bid = self.model.geom_bodyid[o]
            if bid == self.gl:
                side = 0
            elif bid == self.mj_body:
                side = 1
            else:
                continue
            mujoco.mj_contactForce(self.model, self.data, c, f)
            fmag = math.sqrt(f[0] * f[0] + f[1] * f[1] + f[2] * f[2])
            if fmag > fmax:
                fmax = fmag
            if fmag < CONTACT_FORCE_MIN:
                continue
            if side == 0:
                nf += 1
            else:
                nm += 1
        return nf, nm, fmax

    def obj_pos(self):
        return self.data.xpos[self.obj_body].copy()

    def obj_lift(self):
        return float(self.data.xpos[self.obj_body][2] - self.ep_obj0[2])

    def on_table(self):
        """物块是否还在桌面附近（没掉下去/没飞走）。"""
        p = self.obj_pos()
        return (abs(p[0] - self._nominal[0]) < 0.25 and
                abs(p[1] - self._nominal[1]) < 0.25 and
                p[2] > TABLE_Z - 0.10)

    # ---------------------------------------------------------- 观测
    def _obs(self):
        d = self.data
        q = d.qpos[self._arm_q]                    # 5
        g = d.qpos[self._gq]                       # 1
        p_tcp, R = self.tcp()
        p_obj = d.xpos[self.obj_body]
        rel = R.T @ (p_obj - p_tcp)                # 3
        oq = d.qpos[self.obj_q + 3:self.obj_q + 7]
        yaw_o = math.atan2(2 * (oq[0] * oq[3] + oq[1] * oq[2]),
                           1 - 2 * (oq[2] ** 2 + oq[3] ** 2))
        dyaw = yaw_o - math.atan2(R[1, 0], R[0, 0])
        lag = d.ctrl[self._arm_a] - q
        # 组装（避免 np.concatenate 的开销，直接写进预分配缓冲）
        o = self._obuf
        o[0:5] = q
        o[5] = g
        o[6:9] = rel
        o[9] = math.sin(dyaw)
        o[10] = math.cos(dyaw)
        o[11] = p_obj[2] - TABLE_Z
        o[12] = p_tcp[2] - p_obj[2]
        o[13:18] = lag
        o[18] = d.ctrl[self._ga] - g
        return o

    # ---------------------------------------------------------- reset
    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        if seed is not None:
            self._rng = np.random.default_rng(seed)
        d = self.data
        mujoco.mj_resetData(self.model, d)
        # 物块初始位姿（这是唯一允许直接写 qpos 的地方：初始化）
        if self.randomize_obj:
            dx, dy = self._rng.uniform(-self.jitter, self.jitter, 2)
            yaw = self._rng.uniform(-OBJ_YAW_JITTER, OBJ_YAW_JITTER)
        else:
            dx = dy = 0.0
            yaw = 0.0
        op = self._nominal + np.array([dx, dy, 0.0])
        op[2] = TABLE_Z + 0.003 + self.obj_size / 2
        d.qpos[self.obj_q:self.obj_q + 3] = op
        d.qpos[self.obj_q + 3:self.obj_q + 7] = [math.cos(yaw / 2), 0, 0,
                                                 math.sin(yaw / 2)]
        # 机械臂放到预抓取位（直接写 qpos 只用于"回合开始时的摆位"，
        # 之后一律只发 ctrl。这里不涉及夹持，不会挤掉物块）
        for j in S.ARM_JOINTS:
            d.qpos[self.qadr[j]] = self.q_home[S.ARM_JOINTS.index(j)]
            d.ctrl[self.aadr[j]] = self.q_home[S.ARM_JOINTS.index(j)]
        d.qpos[self.qadr['gripper']] = APPROACH_ANG
        d.ctrl[self.grip_act] = APPROACH_ANG
        mujoco.mj_forward(self.model, d)
        self.ep_obj0 = self.obj_pos().copy()
        self.ep_steps = 0
        self._phase_contacts = np.zeros(2)
        self._prev_ctrl = np.array(
            [d.ctrl[self.aadr[j]] for j in S.ARM_JOINTS] +
            [d.ctrl[self.grip_act]], dtype=np.float64)
        self.last_info = {}
        return self._obs(), {}

    # ---------------------------------------------------------- step
    def step(self, action):
        a = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)
        d = self.data
        # 绝对目标角增量（相对上一帧的目标）
        tgt = self._prev_ctrl.copy()
        tgt[:5] += a[:5] * ARM_STEP_MAX
        tgt[5] += a[5] * GRIP_STEP_MAX
        tgt[:5] = np.clip(tgt[:5], self.jlo - 0.4, self.jhi + 0.4)
        tgt[5] = float(np.clip(tgt[5], -0.1745, 1.7453))
        for k, j in enumerate(S.ARM_JOINTS):
            d.ctrl[self.aadr[j]] = tgt[k]
        d.ctrl[self.grip_act] = tgt[5]
        self._prev_ctrl = tgt
        for _ in range(N_SUBSTEPS):
            mujoco.mj_step(self.model, d)
        self.ep_steps += 1

        # ---- 判定
        nf, nm, fmax = self._contacts()
        side = np.array([1.0 if nf else 0.0, 1.0 if nm else 0.0])
        lift = self.obj_lift()
        p_tcp, _ = self.tcp()
        p_obj = self.obj_pos()
        dist = float(np.linalg.norm((p_obj + GRASP_OFF) - p_tcp))
        grip_ang = float(d.qpos[self.qadr['gripper']])
        both = bool(nf and nm)
        grasped = both and grip_ang < 0.35
        # 物块比 TCP 高出一大截 = 被甩飞了（正常夹持时物块在爪口里，
        # 只会略高于 TCP）
        thrown = bool(p_obj[2] - p_tcp[2] > 0.10)
        success = bool(lift > LIFT_SUCCESS and both and not thrown)
        fell = not self.on_table()

        r_app = -2.0 * dist
        r_side = 0.5 * float(side.sum())
        r_grasp = 1.0 if grasped else 0.0
        # ★ 关键修正 1：抬升奖励必须**以"两侧真的夹住"为前提**。
        # 之前没这个前提，策略学会了把物块往天上甩（实测抬升 +1217mm
        # = 1.2 米），拿满抬升分但根本没有抓取。见
        # rl_out/ppo_1m_badreward/evals.txt 的证据。
        r_lift = 8.0 * min(max(lift / LIFT_SUCCESS, 0.0), 1.0) if both else 0.0
        r = r_app + r_side + r_grasp + r_lift - 0.01
        if success:
            r += 10.0
        # ★ 关键修正 2：把物块打飞/掉下桌子的惩罚必须**远大于**时间惩罚。
        # 罚 -1 时策略发现"7 步内把物块铲飞、提前结束回合、躲掉
        # 150 步 × -0.01 的时间惩罚"是有利可图的（实测平均回合长度掉到
        # 7 步）。见 rl_out/ppo_1p5m_badfloor/evals.txt。
        if fell:
            r -= 50.0
        if thrown:
            r -= 50.0
        r *= self.reward_scale

        self._phase_contacts += side
        terminated = success or fell or thrown
        truncated = self.ep_steps >= self.max_steps
        info = dict(lift=lift, contacts_fixed=nf, contacts_moving=nm,
                    force_max=fmax, dist_tcp=dist, grip_ang=grip_ang,
                    success=success, fell=fell, grasped=grasped,
                    thrown=thrown, r_approach=r_app, r_side=r_side,
                    r_grasp=r_grasp, r_lift=r_lift,
                    phase_fixed=float(self._phase_contacts[0]),
                    phase_moving=float(self._phase_contacts[1]),
                    reward=r)
        self.last_info = info
        return self._obs(), float(r), bool(terminated), bool(truncated), info

    # ---------------------------------------------------------- 其它
    def set_obj(self, dx=0.0, dy=0.0, yaw=0.0):
        """把物块放到指定偏移（供评测用，确定性）。"""
        self.randomize_obj = False
        self._fixed = (dx, dy, yaw)
        return self.reset()


# ==================================================================== 自检
def check(obj_size=OBJ_SIZE, episodes=3, steps=40, verbose=True):
    env = GraspEnv(obj_size=obj_size, seed=0)
    obs, _ = env.reset(seed=0)
    if verbose:
        print(f'=== rl_env 自检：物块 {obj_size*1000:.0f}mm，'
              f'OBS={env.observation_space.shape} ACT={env.action_space.shape}'
              f'，模型 ngeom={env.model.ngeom} ===')
        print(f'  标称物块点 {np.round(env._nominal,4)}，'
              f'预抓取 q_home={np.round(env.q_home,3)}')
    tot = 0
    for ep in range(episodes):
        obs, _ = env.reset(seed=ep)
        for k in range(steps):
            a = env.action_space.sample()
            obs, r, te, tr, info = env.step(a)
            tot += 1
            if te or tr:
                break
        if verbose:
            print(f'  回合{ep}: {k+1} 步  抬升={info["lift"]*1000:+7.1f}mm '
                  f'接触固/活={info["contacts_fixed"]}/{info["contacts_moving"]}'
                  f' 力={info["force_max"]:.2f}N 成功={info["success"]}')
    return env


def speed(steps=20000, obj_size=OBJ_SIZE):
    env = GraspEnv(obj_size=obj_size, seed=1, randomize_obj=False)
    env.reset(seed=1)
    rng = np.random.default_rng(0)
    a = np.zeros(ACT_DIM, np.float32)
    acts = rng.uniform(-1, 1, (256, ACT_DIM)).astype(np.float32)
    for i in range(200):                       # 预热
        env.step(acts[i % 256])
    t0 = time.perf_counter()
    for i in range(steps):
        env.step(acts[i % 256])
    dt = time.perf_counter() - t0
    print(f'=== env.step 吞吐 ===')
    print(f'  {steps} 步 / {dt:.2f}s = {steps/dt:9.1f} env-steps/s')
    print(f'  每个 env.step 内含 {N_SUBSTEPS} 次 mj_step + 接触遍历')
    print(f'  等效物理步进 {steps*N_SUBSTEPS/dt:9.0f} mj_step/s')
    print(f'  模拟时间推进 {steps*DT_CTRL:.0f}s（{dt:.1f}s 真实时间）'
          f' → 实时倍率 {steps*DT_CTRL/dt:.1f}×')
    return steps / dt


def expert_rollout(env, obj_size=OBJ_SIZE, verbose=True, off_mm=GRASP_OFF_MM,
                   approach=APPROACH_ANG, zigzag=0):
    """脚本化"专家"回放：预抓取 → 下压 → 夹紧 → 抬起。

    全程通过 action（= ctrl 目标增量）驱动，和 RL 策略走**同一条**接口，
    所以它能成功就说明 env 的奖励/终止/物理都对。

    返回 (success, info, steps)
    """
    off = np.asarray(off_mm, dtype=float) / 1000.0     # mm → m
    env.reset(seed=0)
    env.randomize_obj = False
    op = env.obj_pos().copy()
    # 与已验证的 sim_grasp_ok.grasp() 同构的四个阶段：
    #   预抓取(上方60mm) → 下压到物块 → **单独一段闭爪 settle** → 抬起
    # 闭爪必须独立成段并留够 settle 步数（伺服滞后），否则夹不实。
    plan = [(op + off + np.array([0, 0, PRE_GRASP_DZ]), approach, 30),
            (op + off, approach, 45),
            (op + off, GRIPPER_CLOSED, 40),
            (op + off + np.array([0, 0, 0.12]), GRIPPER_CLOSED, 50)]

    def drive(tgt_xyz, grip, n, tol=0.0015):
        nonlocal info
        for k in range(n):
            q_sol = env._ik(np.asarray(tgt_xyz), seed_qpos=None)
            cur = env._prev_ctrl
            delta = np.zeros(ACT_DIM)
            for i in range(5):
                delta[i] = np.clip((q_sol[i] - cur[i]) / ARM_STEP_MAX, -1, 1)
            delta[5] = np.clip((grip - cur[5]) / GRIP_STEP_MAX, -1, 1)
            _, _, te, tr, info = env.step(delta)
            if te or tr:
                return True
            if k > 10:
                p, _ = env.tcp()
                if np.linalg.norm(p - np.asarray(tgt_xyz)) < tol:
                    return False
        return False

    steps = 0
    info = None
    for tgt_xyz, gtgt, ns in plan:
        done = drive(tgt_xyz, gtgt, ns)
        steps += ns
        if verbose:
            p, _ = env.tcp()
            print(f'    目标 {np.round(tgt_xyz*1000,1)}mm 夹爪={gtgt:.2f} → '
                  f'TCP误差={np.linalg.norm(p-np.asarray(tgt_xyz))*1000:5.2f}mm '
                  f'抬升={info["lift"]*1000:+7.1f}mm '
                  f'接触固/活={info["contacts_fixed"]}/'
                  f'{info["contacts_moving"]} 力={info["force_max"]:.2f}N '
                  f'夹爪角={info["grip_ang"]:+.3f}')
        if done:
            break
    return bool(info['success']), info, steps


def baseline(episodes=20, obj_size=OBJ_SIZE, policy='random', seed=0,
             verbose=True, keep_failed_rollout=0, out_csv=None):
    """随机策略成功率（确定性初始位姿 + 随机初始位姿两套）。"""
    res = []
    for jitter in (0.0, OBJ_JITTER):
        env = GraspEnv(obj_size=obj_size, seed=seed)
        env.jitter = jitter
        succ = 0
        lifts = []
        for ep in range(episodes):
            env.reset(seed=seed * 1000 + ep + int(jitter * 1e6))
            done = False
            while not done:
                a = env.action_space.sample()
                _, _, te, tr, info = env.step(a)
                done = te or tr
            succ += int(info['success'])
            lifts.append(info['lift'] * 1000)
        rate = succ / episodes
        res.append(dict(jitter=jitter, rate=rate, lifts=lifts))
        if verbose:
            print(f'  {"确定性" if jitter == 0 else "随机±10mm"}初始位姿: '
                  f'成功 {succ}/{episodes} = {rate*100:.0f}%   '
                  f'抬升最大值 {max(lifts):+.1f}mm  '
                  f'平均 {np.mean(lifts):+.1f}mm')
    if out_csv:
        with open(out_csv, 'w') as fh:
            fh.write('jitter,rate,lifts_mm\n')
            for r in res:
                fh.write(f'{r["jitter"]},{r["rate"]},'
                         f'{"|".join(f"{v:.1f}" for v in r["lifts"])}\n')
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--check', action='store_true')
    ap.add_argument('--speed', action='store_true')
    ap.add_argument('--expert', action='store_true')
    ap.add_argument('--baseline', type=int, default=0)
    ap.add_argument('--obj-size', type=float, default=OBJ_SIZE)
    ap.add_argument('--out', default=None)
    a = ap.parse_args()
    if a.expert:
        env = GraspEnv(obj_size=a.obj_size, seed=0)
        print('=== 脚本化专家回放（验证 env 里物理/奖励/终止都通）===')
        ok, info, steps = expert_rollout(env, a.obj_size)
        print(f'  结果: {"✅ 成功" if ok else "❌ 失败"}  '
              f'{steps} 步  抬升 {info["lift"]*1000:+.1f}mm  '
              f'累计奖励 {info["reward"]:+.2f}')
    elif a.baseline:
        print(f'=== 随机策略基线（{a.baseline} 回合 × 2 种初始位姿分布）===')
        baseline(a.baseline, a.obj_size, out_csv=a.out)
    elif a.speed:
        speed(obj_size=a.obj_size)
    else:
        check(a.obj_size)


if __name__ == '__main__':
    main()
