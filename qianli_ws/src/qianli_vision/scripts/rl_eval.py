#!/usr/bin/env python3
"""策略评测：在**同一组**确定性初始位姿上比较 随机策略 / 训练后策略。

为什么单独写：报告里的"成功率对比"必须是同一批初始条件、同一套判定，
否则数字不可比。这里固定 seed 序列生成 N 个物块初始位姿，两个策略各跑一遍。

用法：
  ~/mj/bin/python rl_eval.py --model rl_out/ppo_1m/ppo_1m_final.zip --episodes 50
  ~/mj/bin/python rl_eval.py --model ... --jitter 0.01 --out eval_1m.csv
  ~/mj/bin/python rl_eval.py --random-only --episodes 50
"""

import argparse
import csv
import os

import numpy as np

import rl_env as R


def rollout(env, policy, max_steps=None):
    """跑一个回合，返回 info。policy(obs) -> action。"""
    obs = env._obs()
    done = False
    info = {}
    n = 0
    rew = 0.0
    while not done:
        a = policy(obs)
        obs, r, te, tr, info = env.step(a)
        rew += r
        n += 1
        done = te or tr
        if max_steps and n >= max_steps:
            break
    info['ep_len'] = n
    info['ep_rew'] = rew
    return info


def make_random_policy(env, seed=0):
    rng = np.random.default_rng(seed)

    def pol(obs):
        return rng.uniform(-1, 1, R.ACT_DIM)
    return pol


def make_model_policy(model):
    def pol(obs):
        a, _ = model.predict(obs, deterministic=True)
        return a
    return pol


def evaluate(env, policy, episodes, seed0, verbose=True, label=''):
    succ = 0
    lifts = []
    rews = []
    lens = []
    for ep in range(episodes):
        env.reset(seed=seed0 + ep)
        info = rollout(env, policy)
        succ += int(info['success'])
        lifts.append(info['lift'] * 1000)
        rews.append(info['ep_rew'])
        lens.append(info['ep_len'])
    out = dict(label=label, episodes=episodes, success=succ,
               rate=succ / episodes, lifts=lifts, rews=rews, lens=lens)
    if verbose:
        print(f'  {label:22s} 成功 {succ:3d}/{episodes:<3d} = '
              f'{succ/episodes*100:5.1f}%  '
              f'抬升 平均{np.mean(lifts):+7.1f}mm 最大{np.max(lifts):+7.1f}mm  '
              f'平均回报{np.mean(rews):+8.1f}  平均步数{np.mean(lens):5.1f}')
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', default=None, help='SB3 .zip 模型路径')
    ap.add_argument('--models', nargs='*', default=None,
                    help='一次评测多个模型（在同一批初始位姿上对比，便于做置信区间）')
    ap.add_argument('--episodes', type=int, default=50)
    ap.add_argument('--jitter', type=float, default=R.OBJ_JITTER)
    ap.add_argument('--seed', type=int, default=777)
    ap.add_argument('--obj-size', type=float, default=R.OBJ_SIZE)
    ap.add_argument('--out', default=None)
    ap.add_argument('--random-only', action='store_true')
    ap.add_argument('--no-random', action='store_true')
    ap.add_argument('--max-ep-steps', type=int, default=R.MAX_STEPS)
    a = ap.parse_args()

    env = R.GraspEnv(obj_size=a.obj_size, seed=a.seed, jitter=a.jitter,
                     max_steps=a.max_ep_steps)
    print(f'=== 策略评测（物块 {a.obj_size*1000:.0f}mm，'
          f'{a.episodes} 个确定性初始位姿，抖动 ±{a.jitter*1000:.0f}mm，'
          f'seed={a.seed}）===')
    res = []
    rnd = None
    if not a.no_random:
        rnd = evaluate(env, make_random_policy(env, a.seed), a.episodes,
                       a.seed, label='随机策略')
        res.append(rnd)
    models = a.models or ([a.model] if a.model else [])
    if models:
        from stable_baselines3 import PPO, SAC
        for path in models:
            cls = SAC if os.path.basename(path).startswith('sac') else PPO
            model = cls.load(path, device='cpu')
            res.append(evaluate(env, make_model_policy(model), a.episodes,
                                a.seed,
                                label=os.path.basename(path).replace('.zip', '')))
    if rnd is not None and len(res) > 1:
        print()
        for r in res[1:]:
            d = (r['rate'] - rnd['rate']) * 100
            # 成功率差的 95% 置信区间（两独立比例的正态近似）
            n = a.episodes
            se = np.sqrt(max(rnd['rate'] * (1 - rnd['rate']) / n +
                             r['rate'] * (1 - r['rate']) / n, 1e-12))
            print(f'  {r["label"]}: vs 随机 {d:+.0f} 个百分点 '
                  f'(±{1.96*se*100:.0f}, 95%CI)')
    if a.out:
        with open(a.out, 'w', newline='') as fh:
            w = csv.writer(fh)
            w.writerow(['label', 'episodes', 'success', 'rate',
                        'mean_lift_mm', 'max_lift_mm', 'mean_reward',
                        'mean_ep_len', 'all_lifts_mm'])
            for r in res:
                w.writerow([r['label'], r['episodes'], r['success'],
                            r['rate'], round(float(np.mean(r['lifts'])), 3),
                            round(float(np.max(r['lifts'])), 3),
                            round(float(np.mean(r['rews'])), 3),
                            round(float(np.mean(r['lens'])), 2),
                            '|'.join(f'{v:.1f}' for v in r['lifts'])])
        print(f'  结果已写 {a.out}')
    return res


if __name__ == '__main__':
    main()
