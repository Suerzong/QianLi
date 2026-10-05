#!/usr/bin/env python3
"""宿主机 GPU vs CPU 训练速度对比（判断显卡对这个问题到底有没有用）。

为什么值得单独测：
  这个任务的瓶颈是 **MuJoCo 物理步进 + Python 环境步进**（纯 CPU），
  神经网络只是 22→256→256→6 的小 MLP。所以"搬上 GPU"未必更快 ——
  实测出数字再说，不猜。

做法：同样的 SubprocVecEnv、同样的 PPO 超参，只改 device，
跑固定步数，比 env-steps/s 和总耗时。

用法：
  python bench_gpu.py --steps 30000 --n-envs 10
"""

import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import win_twin  # noqa: E402  必须在 rl_env 之前，负责路径改写

import numpy as np  # noqa: E402
import torch  # noqa: E402

from stable_baselines3 import PPO  # noqa: E402
from stable_baselines3.common.callbacks import BaseCallback  # noqa: E402
from stable_baselines3.common.vec_env import SubprocVecEnv  # noqa: E402
from stable_baselines3.common.monitor import Monitor  # noqa: E402

import rl_env as R  # noqa: E402


def make_env(rank, seed=0):
    def _init():
        return Monitor(R.GraspEnv(seed=seed + rank))
    return _init


class Timer(BaseCallback):
    def __init__(self):
        super().__init__()
        self.t0 = None
        self.steps_at_start = 0

    def _on_training_start(self):
        self.t0 = time.perf_counter()
        self.steps_at_start = self.num_timesteps

    def _on_step(self):
        return True

    def elapsed(self):
        return time.perf_counter() - self.t0


def run(device, steps, n_envs, seed=0):
    n_sub = max(1, n_envs)
    venv = SubprocVecEnv([make_env(i, seed) for i in range(n_sub)],
                         start_method='spawn')
    model = PPO('MlpPolicy', venv, device=device, seed=seed,
                n_steps=256, batch_size=1024, n_epochs=10,
                learning_rate=3e-4, gamma=0.98, ent_coef=0.002,
                policy_kwargs=dict(net_arch=[256, 256]), verbose=0)
    dev_used = model.policy.device
    timer = Timer()
    t0 = time.perf_counter()
    model.learn(total_timesteps=steps, callback=timer, progress_bar=False)
    dt = time.perf_counter() - t0
    venv.close()
    return dt, steps / dt, str(dev_used)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--steps', type=int, default=30000)
    ap.add_argument('--n-envs', type=int, default=10)
    a = ap.parse_args()

    print('=== 硬件 ===')
    print(f'  torch            {torch.__version__}')
    print(f'  cuda available   {torch.cuda.is_available()}')
    if torch.cuda.is_available():
        print(f'  GPU              {torch.cuda.get_device_name(0)}')
        print(f'  GPU 显存         '
              f'{torch.cuda.get_device_properties(0).total_memory/2**30:.1f} GiB')
    print(f'  CPU 核数         {os.cpu_count()}')
    print(f'  并行环境         {a.n_envs}  (MuJoCo 物理在 CPU)')
    print()

    results = {}
    for dev in ('cpu', 'cuda'):
        if dev == 'cuda' and not torch.cuda.is_available():
            continue
        # 先跑一小段热身，避免把 CUDA 初始化/编译算进去
        run(dev, 2048, a.n_envs)
        dt, sps, used = run(dev, a.steps, a.n_envs)
        results[dev] = (dt, sps, used)
        print(f'  device={dev:5s} (实际用 {used:5s})  '
              f'{a.steps} 步 / {dt:7.2f}s = {sps:8.1f} env-steps/s')
        sys.stdout.flush()

    if 'cpu' in results and 'cuda' in results:
        pc = results['cpu'][1]
        pg = results['cuda'][1]
        print()
        print(f'  GPU/CPU 加速比 = {pg/pc:.2f}×  '
              f'({"GPU 更快" if pg > pc*1.05 else "基本一样" if abs(pg-pc) <= pc*0.05 else "GPU 更慢"})')
        print('  说明：本任务瓶颈是 MuJoCo 物理步进（CPU）+ Python 环境步进，'
              '神经网络只是 22→256→256→6 的小 MLP。')


if __name__ == '__main__':
    main()
