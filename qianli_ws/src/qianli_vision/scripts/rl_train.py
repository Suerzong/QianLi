#!/usr/bin/env python3
"""机械臂抓取立方体 —— RL 训练脚本（stable-baselines3 PPO，纯仿真）

设计要点：
  · 环境用 rl_env.GraspEnv（全保真 CoACD 夹爪孪生，与已验证的抓取一致）
  · 并行：12 个 SubprocVecEnv（机器 12 核），CPU 训练
  · 日志：TensorBoard (runs/) + CSV (csv/) + 后台 nohup 日志
  · 定期 checkpoint（每 --save-every 步）+ 最终模型
  · 回调里定期做**确定性评测**（固定初始位姿 + 随机 ±10mm 位姿各若干回合），
    把成功率写进 TensorBoard/CSV —— 报告里的学习曲线就是这个

用法：
  # 短基线（先验证能学到东西）
  ~/mj/bin/python rl_train.py --steps 20000 --n-envs 8 --tag ppo_short

  # 正式训练（后台）
  nohup ~/mj/bin/python rl_train.py --steps 1000000 --n-envs 12 \
        --tag ppo_1m > /tmp/rl_train_ppo_1m.log 2>&1 &

  # SAC（可选）
  ~/mj/bin/python rl_train.py --algo sac --steps 300000
"""

import argparse
import csv
import os
import time

import numpy as np
import torch

# 单进程内限制 torch 线程数（外面用 SubprocVecEnv 并行）
torch.set_num_threads(2)

from stable_baselines3 import PPO, SAC
from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import SubprocVecEnv, DummyVecEnv

import rl_env as R

HERE = os.path.dirname(os.path.abspath(__file__))


def make_env(rank, seed=0, jitter=R.OBJ_JITTER, max_steps=R.MAX_STEPS):
    def _init():
        env = R.GraspEnv(seed=seed + rank, jitter=jitter, max_steps=max_steps)
        env = Monitor(env)
        return env
    return _init


class MetricCallback(BaseCallback):
    """定期做确定性评测，把成功率/抬升写进 TensorBoard 与 CSV。"""

    def __init__(self, eval_every=10000, n_eval=10, csv_path=None,
                 verbose=0, max_steps=R.MAX_STEPS):
        super().__init__(verbose)
        self.eval_every = eval_every
        self.n_eval = n_eval
        self.csv_path = csv_path
        self.max_steps = max_steps
        self.rows = []
        self._last_eval = 0
        self._t0 = time.time()
        self.eval_env = R.GraspEnv(seed=12345, jitter=R.OBJ_JITTER,
                                   max_steps=max_steps)
        if csv_path:
            os.makedirs(os.path.dirname(os.path.abspath(csv_path)),
                        exist_ok=True)
            with open(csv_path, 'w', newline='') as fh:
                csv.writer(fh).writerow(
                    ['timesteps', 'wall_s', 'success_rate', 'mean_reward',
                     'mean_lift_mm', 'mean_ep_len', 'max_lift_mm',
                     'success_fixed_pos'])

    def _eval(self, deterministic=True):
        succ = 0
        lifts = []
        rews = []
        lens = []
        for ep in range(self.n_eval):
            self.eval_env.reset(seed=1000 + ep)
            done = False
            tot = 0.0
            n = 0
            info = {}
            while not done:
                obs = self.eval_env._obs()
                act, _ = self.model.predict(obs, deterministic=deterministic)
                _, r, te, tr, info = self.eval_env.step(act)
                tot += r
                n += 1
                done = te or tr
            succ += int(info.get('success', False))
            lifts.append(info.get('lift', 0.0) * 1000)
            rews.append(tot)
            lens.append(n)
        # 固定初始位姿（无 jitter）单独一列，作为"最干净"的指标
        self.eval_env.jitter = 0.0
        succ_fixed = 0
        for ep in range(self.n_eval):
            self.eval_env.reset(seed=2000 + ep)
            done = False
            info = {}
            while not done:
                obs = self.eval_env._obs()
                act, _ = self.model.predict(obs, deterministic=deterministic)
                _, _, te, tr, info = self.eval_env.step(act)
                done = te or tr
            succ_fixed += int(info.get('success', False))
        self.eval_env.jitter = R.OBJ_JITTER
        return dict(success_rate=succ / self.n_eval,
                    mean_reward=float(np.mean(rews)),
                    mean_lift_mm=float(np.mean(lifts)),
                    max_lift_mm=float(np.max(lifts)),
                    mean_ep_len=float(np.mean(lens)),
                    success_fixed_pos=succ_fixed / self.n_eval)

    def _on_step(self):
        if self.num_timesteps - self._last_eval < self.eval_every:
            return True
        self._last_eval = self.num_timesteps
        st = self._eval()
        wall = time.time() - self._t0
        for k, v in st.items():
            self.logger.record(f'eval/{k}', v)
        self.logger.record('eval/wall_s', wall)
        self.logger.record('eval/sps', self.num_timesteps / max(wall, 1e-9))
        row = [self.num_timesteps, round(wall, 1)] + \
            [round(st[k], 4) for k in
             ('success_rate', 'mean_reward', 'mean_lift_mm', 'mean_ep_len',
              'max_lift_mm', 'success_fixed_pos')]
        self.rows.append(row)
        if self.csv_path:
            with open(self.csv_path, 'a', newline='') as fh:
                csv.writer(fh).writerow(row)
        print(f'[eval] {self.num_timesteps:>9d} 步  {wall:7.1f}s  '
              f'成功率(随机位姿)={st["success_rate"]*100:5.1f}%  '
              f'(固定位姿)={st["success_fixed_pos"]*100:5.1f}%  '
              f'平均抬升={st["mean_lift_mm"]:+7.1f}mm  '
              f'最大={st["max_lift_mm"]:+7.1f}mm  '
              f'平均回报={st["mean_reward"]:+7.2f}  '
              f'平均步数={st["mean_ep_len"]:.0f}', flush=True)
        return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--algo', choices=['ppo', 'sac'], default='ppo')
    ap.add_argument('--steps', type=int, default=200000)
    ap.add_argument('--n-envs', type=int, default=8)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--tag', default=None)
    ap.add_argument('--eval-every', type=int, default=10000)
    ap.add_argument('--n-eval', type=int, default=10)
    ap.add_argument('--save-every', type=int, default=50000)
    ap.add_argument('--lr', type=float, default=3e-4)
    ap.add_argument('--ent-coef', type=float, default=0.005)
    ap.add_argument('--n-epochs', type=int, default=10)
    ap.add_argument('--gamma', type=float, default=0.98)
    ap.add_argument('--net-arch', default='256,256')
    ap.add_argument('--n-steps', type=int, default=256,
                    help='PPO 每个 env 每次更新的步数')
    ap.add_argument('--batch-size', type=int, default=1024)
    ap.add_argument('--jitter', type=float, default=R.OBJ_JITTER)
    ap.add_argument('--max-ep-steps', type=int, default=R.MAX_STEPS)
    ap.add_argument('--outdir', default=os.path.join(HERE, 'rl_out'))
    ap.add_argument('--device', default='cpu')
    a = ap.parse_args()

    tag = a.tag or f'{a.algo}_{a.steps}'
    outdir = os.path.join(a.outdir, tag)
    os.makedirs(outdir, exist_ok=True)
    db_dir = os.path.join(outdir, 'tb')
    csv_path = os.path.join(outdir, 'eval.csv')
    ckpt_dir = os.path.join(outdir, 'ckpt')
    os.makedirs(ckpt_dir, exist_ok=True)

    n_envs = max(1, a.n_envs)
    print(f'=== RL 训练 {tag} ===')
    print(f'  算法={a.algo.upper()}  总步数={a.steps}  并行环境={n_envs}  '
          f'seed={a.seed}')
    print(f'  物块抖动 ±{a.jitter*1000:.0f}mm  回合上限={a.max_ep_steps} RL 步 '
          f'({a.max_ep_steps*R.DT_CTRL:.1f}s 仿真时间)')
    print(f'  输出目录 {outdir}')

    vec_cls = SubprocVecEnv if n_envs > 1 else DummyVecEnv
    venv = vec_cls([make_env(i, a.seed, a.jitter, a.max_ep_steps)
                    for i in range(n_envs)])

    arch = tuple(int(x) for x in a.net_arch.split(',') if x)
    policy_kwargs = dict(net_arch=list(arch))
    if a.algo == 'ppo':
        model = PPO('MlpPolicy', venv, verbose=1, seed=a.seed,
                    device=a.device, learning_rate=a.lr,
                    n_steps=a.n_steps, batch_size=a.batch_size,
                    n_epochs=a.n_epochs, gamma=a.gamma, gae_lambda=0.95,
                    clip_range=0.2, ent_coef=a.ent_coef, vf_coef=0.5,
                    max_grad_norm=0.5, policy_kwargs=policy_kwargs,
                    tensorboard_log=db_dir)
    else:
        model = SAC('MlpPolicy', venv, verbose=1, seed=a.seed,
                    device=a.device, learning_rate=a.lr,
                    batch_size=256, gamma=a.gamma, tau=0.005,
                    train_freq=1, gradient_steps=1,
                    learning_starts=2000, ent_coef='auto',
                    policy_kwargs=policy_kwargs,
                    tensorboard_log=db_dir)

    cb = MetricCallback(eval_every=a.eval_every, n_eval=a.n_eval,
                        csv_path=csv_path, max_steps=a.max_ep_steps)
    ck = CheckpointCallback(save_freq=max(1, a.save_every // n_envs),
                            save_path=ckpt_dir, name_prefix=f'{tag}_ckpt')
    t0 = time.perf_counter()
    try:
        model.learn(total_timesteps=a.steps, callback=[cb, ck],
                    progress_bar=False)
    finally:
        final = os.path.join(outdir, f'{tag}_final.zip')
        model.save(final)
        dt = time.perf_counter() - t0
        print(f'\n训练结束（{dt:.0f}s，{a.steps/dt:.0f} 步/s）。'
              f'模型已保存 {final}', flush=True)
        # 训练结束再评测一次并落盘
        st = cb._eval()
        print(f'  最终评测: 成功率(随机位姿)={st["success_rate"]*100:.0f}%  '
              f'(固定位姿)={st["success_fixed_pos"]*100:.0f}%  '
              f'平均抬升={st["mean_lift_mm"]:+.1f}mm', flush=True)
        with open(os.path.join(outdir, 'final_eval.txt'), 'w') as fh:
            for k, v in st.items():
                fh.write(f'{k}\t{v}\n')
        venv.close()


if __name__ == '__main__':
    main()
