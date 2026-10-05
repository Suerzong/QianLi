#!/usr/bin/env python3
"""从脚本化专家采集示范数据（用于行为克隆热启动 PPO）。

为什么需要：
  实测了"探索能不能摸到成功区"——在专家动作上加高斯噪声：
      σ=0.00 → 20/20 成功
      σ=0.05 → 18/20
      σ=0.10 → 16/20
      σ=0.20 →  4/20
      σ=0.40 →  0/20     ← 均匀随机探索大致相当于 σ≈0.6~1.0
  也就是说成功区**很窄**，纯随机探索几乎不可能碰到（这也解释了为什么
  PPO 从零开始训练时 entropy 一直卡在最大值、成功率长期为 0；早先一次
  80k 步跑到 90% 是运气，复现不出来）。

  标准解法：用**已经确定可用**的专家策略（在 rl_env 自己的接口上跑，
  20/20 成功）产生示范，先行为克隆，再让 PPO 微调。

注意：专家只动仿真里的 ctrl，不碰真机。

用法：
  ~/mj/bin/python rl_collect_demos.py --episodes 160 --workers 10 \
      --out rl_out/demos.npz
"""

import argparse
import os
import time
from multiprocessing import Pool

import numpy as np

import rl_env as R

HERE = os.path.dirname(os.path.abspath(__file__))


def collect_episode(seed, jitter, noise, max_steps, only_success):
    """跑一条专家示范，返回 (obs 数组, action 数组, 是否成功)。"""
    rng = np.random.default_rng(seed)
    env = R.GraspEnv(seed=seed, jitter=jitter, max_steps=max_steps)
    env.reset(seed=seed)
    op = env.obj_pos().copy()
    off = R.GRASP_OFF
    plan = [(op + off + np.array([0, 0, R.PRE_GRASP_DZ]), R.APPROACH_ANG, 30),
            (op + off, R.APPROACH_ANG, 45),
            (op + off, R.GRIPPER_CLOSED, 40),
            (op + off + np.array([0, 0, 0.12]), R.GRIPPER_CLOSED, 50)]
    obs_list, act_list = [], []
    info = {}
    done = False
    for tgt, grip, n in plan:
        if done:
            break
        for k in range(n):
            if done:
                break
            q = env._ik(tgt, seed_qpos=None)
            cur = env._prev_ctrl.copy()
            a = np.zeros(R.ACT_DIM)
            for i in range(5):
                a[i] = np.clip((q[i] - cur[i]) / R.ARM_STEP_MAX, -1, 1)
            a[5] = np.clip((grip - cur[5]) / R.GRIP_STEP_MAX, -1, 1)
            obs = env._obs().copy()
            a_noisy = np.clip(a + rng.normal(0, noise, R.ACT_DIM)
                              if noise > 0 else a, -1, 1)
            obs_list.append(obs)
            act_list.append(a_noisy)
            _, _, te, tr, info = env.step(a_noisy)
            done = te or tr
    obs = np.asarray(obs_list, np.float32)
    act = np.asarray(act_list, np.float32)
    succ = bool(info.get('success', False))
    if only_success and not succ:
        return None
    return obs, act, succ, info.get('lift', 0.0) * 1000


def _worker(args):
    seed, jitter, noise, max_steps, only_success = args
    try:
        r = collect_episode(seed, jitter, noise, max_steps, only_success)
        if r is None:
            return {'ok': False, 'err': None}
        obs, act, succ, lift = r
        return {'ok': True, 'obs': obs, 'act': act, 'succ': succ, 'lift': lift}
    except Exception as e:      # pragma: no cover
        return {'ok': False, 'err': repr(e)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--episodes', type=int, default=160)
    ap.add_argument('--workers', type=int, default=10)
    ap.add_argument('--noise', type=float, default=0.10,
                    help='示范动作上的高斯噪声 σ（增加状态覆盖度）')
    ap.add_argument('--jitter', type=float, default=R.OBJ_JITTER)
    ap.add_argument('--max-steps', type=int, default=R.MAX_STEPS)
    ap.add_argument('--only-success', action='store_true', default=True)
    ap.add_argument('--out', default=os.path.join(HERE, 'rl_out/demos.npz'))
    a = ap.parse_args()

    t0 = time.perf_counter()
    tasks = [(1000 + i, a.jitter, a.noise, a.max_steps, a.only_success)
             for i in range(a.episodes)]
    print(f'=== 采集专家示范 {a.episodes} 条 '
          f'（workers={a.workers}, noise σ={a.noise}, '
          f'抖动 ±{a.jitter*1000:.0f}mm）===', flush=True)
    with Pool(a.workers) as pool:
        res = pool.map(_worker, tasks, chunksize=2)
    obs_all, act_all = [], []
    nsucc = 0
    lifts = []
    nerr = 0
    for r in res:
        if not r['ok']:
            if r['err']:
                nerr += 1
                if nerr <= 3:
                    print('  采集出错:', r['err'])
            continue
        nsucc += int(r['succ'])
        lifts.append(r['lift'])
        if a.only_success and not r['succ']:
            continue
        obs_all.append(r['obs'])
        act_all.append(r['act'])
    if not obs_all:
        raise SystemExit('没有采到任何示范')
    obs = np.concatenate(obs_all)
    act = np.concatenate(act_all)
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    np.savez_compressed(a.out, obs=obs, act=act)
    print(f'  专家成功率 {nsucc}/{a.episodes} = '
          f'{nsucc/a.episodes*100:.0f}%  平均抬升 '
          f'{np.mean(lifts):+.1f}mm')
    print(f'  有效示范 {len(obs_all)} 条轨迹，{len(obs)} 个 (obs, action) 样本')
    print(f'  obs {obs.shape}  act {act.shape}  '
          f'act 范围 [{act.min():+.2f}, {act.max():+.2f}]')
    print(f'  已保存 {a.out}  （耗时 {time.perf_counter()-t0:.0f}s）')


if __name__ == '__main__':
    main()
