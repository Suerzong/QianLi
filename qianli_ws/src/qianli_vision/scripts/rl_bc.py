#!/usr/bin/env python3
"""行为克隆热启动：用专家示范预训练 PPO 的策略网络，再交给 PPO 微调。

为什么必须这么做（实测数据）：
  在专家动作上加噪声测"成功区有多宽"：
    σ=0.00 → 20/20    σ=0.05 → 18/20    σ=0.10 → 16/20
    σ=0.20 →  4/20    σ=0.40 →  0/20
  均匀随机探索相当于 σ≈0.6~1.0 → 等于 0。所以从零开始的 PPO 学不会
  （实测 entropy 长期卡在最大值 8.5，成功率长期 0）。先用示范把策略
  放到成功区附近，PPO 才有可能接着优化。

做法：
  1. 用与 PPO 完全一致的网络结构建一个 ActorCriticPolicy
     （MlpPolicy 256x256，同一 obs/act 空间）
  2. 监督学习：让网络均值输出匹配示范动作（MSE）
  3. 保存为 BC 模型；rl_train.py --pretrain <该模型> 会把策略权重
     拷进 PPO 再微调

用法：
  ~/mj/bin/python rl_bc.py --demos rl_out/demos.npz --epochs 40 \
      --out rl_out/bc_policy.zip
"""

import argparse
import os

import numpy as np
import torch
import torch.nn as nn

import rl_env as R

from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--demos', default='rl_out/demos.npz')
    ap.add_argument('--epochs', type=int, default=40)
    ap.add_argument('--batch-size', type=int, default=256)
    ap.add_argument('--lr', type=float, default=1e-3)
    ap.add_argument('--net-arch', default='256,256')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--out', default='rl_out/bc_policy.zip')
    ap.add_argument('--log-std', type=float, default=-1.0)
    a = ap.parse_args()

    data = np.load(a.demos)
    obs = np.asarray(data['obs'], np.float32)
    act = np.asarray(data['act'], np.float32)
    print(f'=== 行为克隆 ===\n  示范样本 {len(obs)} 条  obs {obs.shape} '
          f'act {act.shape}')

    torch.manual_seed(a.seed)
    rng = np.random.default_rng(a.seed)

    env = R.GraspEnv(seed=0)
    venv = DummyVecEnv([lambda: env])
    arch = tuple(int(x) for x in a.net_arch.split(',') if x)
    model = PPO('MlpPolicy', venv, seed=a.seed, device='cpu',
                policy_kwargs=dict(net_arch=list(arch)))

    # tanh 反变换：PPO 的动作是 tanh(pre_tanh)，所以目标应为 atanh(a)
    act_c = np.clip(act, -0.999, 0.999)
    target = np.arctanh(act_c).astype(np.float32)

    policy = model.policy
    opt = torch.optim.Adam(policy.parameters(), lr=a.lr)
    lossf = nn.MSELoss()
    n = len(obs)
    n_val = max(1, n // 20)
    tr_idx = np.arange(n - n_val)
    va_idx = np.arange(n - n_val, n)

    def to_t(x):
        return torch.as_tensor(x, dtype=torch.float32)

    print(f'  训练 {a.epochs} epochs，batch={a.batch_size}，lr={a.lr}')
    best = None
    for ep in range(a.epochs):
        policy.train()
        perm = rng.permutation(tr_idx)
        tot = 0.0
        nb = 0
        for i in range(0, len(perm), a.batch_size):
            b = perm[i:i + a.batch_size]
            ob = to_t(obs[b])
            tg = to_t(target[b])
            dist = policy.get_distribution(ob)
            pred = dist.distribution.mean
            loss = lossf(pred, tg)
            opt.zero_grad()
            loss.backward()
            opt.step()
            tot += float(loss.item())
            nb += 1
        policy.eval()
        with torch.no_grad():
            d = policy.get_distribution(to_t(obs[va_idx]))
            vl = float(lossf(d.distribution.mean, to_t(target[va_idx])).item())
        if best is None or vl < best[0]:
            best = (vl, {k: v.clone() for k, v in policy.state_dict().items()})
        if ep % 5 == 0 or ep == a.epochs - 1:
            print(f'  epoch {ep:3d}  train MSE {tot/max(nb,1):.5f}  '
                  f'val MSE {vl:.5f}', flush=True)
    # 载入最优权重
    policy.load_state_dict(best[1])
    with torch.no_grad():
        policy.log_std.fill_(a.log_std)
    print(f'  最优 val MSE {best[0]:.5f}；log_std 设为 {a.log_std}')

    # 评测 BC 策略（确定性）
    st = evaluate(model, R.GraspEnv(seed=999, jitter=R.OBJ_JITTER),
                  episodes=30)
    print(f'  BC 策略（未微调）成功率 = {st["rate"]*100:.0f}%  '
          f'平均抬升 {st["lift"]:+.1f}mm  '
          f'（参照：随机 0%，专家 100%）')

    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    model.save(a.out)
    print(f'  已保存 {a.out}')
    print(f'  微调命令: ~/mj/bin/python rl_train.py --pretrain {a.out} ...')
    return st


def evaluate(model, env, episodes=30, max_steps=R.MAX_STEPS):
    succ = 0
    lifts = []
    for ep in range(episodes):
        env.reset(seed=100 + ep)
        done = False
        info = {}
        while not done:
            o = env._obs()
            a, _ = model.predict(o, deterministic=True)
            _, _, te, tr, info = env.step(a)
            done = te or tr
        succ += int(info.get('success', False))
        lifts.append(info.get('lift', 0.0) * 1000)
    return dict(rate=succ / episodes, lift=float(np.mean(lifts)))


if __name__ == '__main__':
    main()
