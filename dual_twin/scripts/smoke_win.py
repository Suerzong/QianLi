#!/usr/bin/env python3
"""冒烟测试：宿主机上 SubprocVecEnv（spawn）能否正常起子进程并跑训练。

Windows 上 spawn 会重新导入模块，之前靠 monkeypatch 传路径会在子进程里丢失
（表现为 EOFError）。改用环境变量后应该正常。
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import win_twin                      # noqa: E402  设置环境变量 + sys.path

from stable_baselines3 import PPO                       # noqa: E402
from stable_baselines3.common.monitor import Monitor    # noqa: E402
from stable_baselines3.common.vec_env import SubprocVecEnv  # noqa: E402
sys.path.insert(0, win_twin.LOCAL_SCRIPTS)
import rl_env as R                                      # noqa: E402


def make_env(rank):
    def _init():
        return Monitor(R.GraspEnv(seed=rank))
    return _init


if __name__ == '__main__':
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 2
    print(f'父进程 URDF={win_twin.S.URDF}')
    print(f'父进程 PARTS={win_twin.MG.PARTS_DIR}')
    venv = SubprocVecEnv([make_env(i) for i in range(n)],
                         start_method='spawn')
    print(f'SubprocVecEnv(n={n}) 起来了')
    model = PPO('MlpPolicy', venv, device='cpu', n_steps=128,
                batch_size=256, verbose=0,
                policy_kwargs=dict(net_arch=[64, 64]))
    model.learn(total_timesteps=1024, progress_bar=False)
    print('训练 1024 步 OK')
    venv.close()
    print('SMOKE_OK')
