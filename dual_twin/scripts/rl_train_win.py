#!/usr/bin/env python3
"""宿主机侧训练入口（用上 Windows 的 RTX 显卡 + 更多 CPU 核）。

与虚拟机上的 rl_train.py 完全同一套环境（win_twin 只改资源路径，
不改任何物理参数），区别只是：
  · 设备可选 cuda（宿主机有 RTX 5070 Ti，虚拟机没有 GPU）
  · 并行环境默认开更多（宿主机核数更多）

用法（后台）：
  python rl_train_win.py --steps 600000 --n-envs 10 --device cuda \
      --pretrain rl_out/bc_policy.zip --tag ppo_win_gpu
"""

import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import win_twin  # noqa: E402

# 注意：torch / stable_baselines3 / rl_env 的 import 必须放在 main() 里面。
# Windows 上 SubprocVecEnv 用 spawn，子进程会把本文件当 __main__ 重新执行
# 一遍；如果在模块顶层 import torch，子进程就会**第二次**加载 torch 的 DLL，
# 直接崩掉（实测 _load_dll_libraries() 失败）。
# 模块顶层只保留 win_twin（它很轻，且必须先把路径环境变量设好）。


def make_env(rank, seed=0, jitter=None, max_steps=None):
    import rl_env as R

    def _init():
        from stable_baselines3.common.monitor import Monitor
        return Monitor(R.GraspEnv(seed=seed + rank, jitter=jitter,
                                  max_steps=max_steps))
    return _init


def main():
    import torch
    torch.set_num_threads(1)      # 并行环境各自单线程，避免 CPU 超订

    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import SubprocVecEnv
    sys.path.insert(0, win_twin.LOCAL_SCRIPTS)
    import rl_env as R
    import rl_train as RT        # 复用回调/评测逻辑，避免两份实现

    ap = argparse.ArgumentParser()
    ap.add_argument('--steps', type=int, default=600000)
    ap.add_argument('--n-envs', type=int, default=10)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--tag', default='ppo_win')
    ap.add_argument('--device', default='cuda')
    ap.add_argument('--pretrain', default=None)
    ap.add_argument('--eval-every', type=int, default=10000)
    ap.add_argument('--n-eval', type=int, default=20)
    ap.add_argument('--save-every', type=int, default=100000)
    ap.add_argument('--n-steps', type=int, default=256)
    ap.add_argument('--batch-size', type=int, default=1280)
    ap.add_argument('--n-epochs', type=int, default=10)
    ap.add_argument('--lr', type=float, default=1e-4)
    ap.add_argument('--ent-coef', type=float, default=0.001)
    ap.add_argument('--jitter', type=float, default=None)
    ap.add_argument('--max-ep-steps', type=int, default=None)
    ap.add_argument('--outdir', default=os.path.join(win_twin.DUAL, 'rl_out'))
    a = ap.parse_args()
    if a.jitter is None:
        a.jitter = R.OBJ_JITTER
    if a.max_ep_steps is None:
        a.max_ep_steps = R.MAX_STEPS

    outdir = os.path.join(a.outdir, a.tag)
    os.makedirs(os.path.join(outdir, 'ckpt'), exist_ok=True)
    csv_path = os.path.join(outdir, 'eval.csv')
    tb_dir = os.path.join(outdir, 'tb')

    print(f'=== 宿主机训练 {a.tag} ===')
    print(f'  device={a.device}  cuda可用={torch.cuda.is_available()}')
    if torch.cuda.is_available():
        print(f'  GPU={torch.cuda.get_device_name(0)}')
    print(f'  并行环境={a.n_envs}  总步数={a.steps}  '
          f'lr={a.lr}  ent={a.ent_coef}  抖动±{a.jitter*1000:.0f}mm')
    print(f'  输出 {outdir}', flush=True)

    venv = SubprocVecEnv([make_env(i, a.seed, a.jitter, a.max_ep_steps)
                          for i in range(a.n_envs)], start_method='spawn')
    model = PPO('MlpPolicy', venv, verbose=1, seed=a.seed, device=a.device,
                learning_rate=a.lr, n_steps=a.n_steps,
                batch_size=a.batch_size, n_epochs=a.n_epochs,
                gamma=0.98, gae_lambda=0.95, clip_range=0.2,
                ent_coef=a.ent_coef, vf_coef=0.5, max_grad_norm=0.5,
                policy_kwargs=dict(net_arch=[256, 256]),
                tensorboard_log=tb_dir)

    if a.pretrain:
        bc = PPO.load(a.pretrain, device=a.device)
        sd_bc = bc.policy.state_dict()
        sd = model.policy.state_dict()
        n = 0
        for k, v in sd_bc.items():
            if k in sd and sd[k].shape == v.shape:
                sd[k] = v.clone()
                n += 1
        model.policy.load_state_dict(sd)
        print(f'  ✅ 已载入行为克隆权重 {n} 项（{a.pretrain}）', flush=True)

    cb = RT.MetricCallback(eval_every=a.eval_every, n_eval=a.n_eval,
                           csv_path=csv_path, max_steps=a.max_ep_steps)
    cb.init_callback(model)
    st0 = cb._eval()
    print(f'  训练前评测: 成功率={st0["success_rate"]*100:.0f}%  '
          f'平均抬升={st0["mean_lift_mm"]:+.1f}mm', flush=True)

    t0 = time.perf_counter()
    try:
        model.learn(total_timesteps=a.steps, callback=cb, progress_bar=False)
    finally:
        dt = time.perf_counter() - t0
        final = os.path.join(outdir, f'{a.tag}_final.zip')
        model.save(final)
        print(f'\n训练结束 {dt:.0f}s ({a.steps/dt:.0f} 步/s) → {final}',
              flush=True)
        st = cb._eval()
        print(f'  最终评测: 成功率={st["success_rate"]*100:.0f}%  '
              f'平均抬升={st["mean_lift_mm"]:+.1f}mm', flush=True)
        venv.close()


if __name__ == '__main__':
    main()
