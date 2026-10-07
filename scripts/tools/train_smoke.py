#!/usr/bin/env python3
"""Two spawned simulators, PPO update, model round trip and CUDA arithmetic."""
import argparse
import json
from pathlib import Path
import sys
import tempfile

from project_paths import project_path
sys.path.insert(0, project_path('dual_twin/scripts'))
import twin_runtime  # noqa: E402,F401


def make_env(size, seed):
    def create():
        from rl_env import GraspEnv
        return GraspEnv(obj_size=size, seed=seed, max_steps=16)
    return create


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', choices=['cpu', 'cuda'], default='cpu')
    parser.add_argument('--obj-size', type=float, choices=[0.02, 0.04], default=0.04)
    parser.add_argument('--steps', type=int, default=1024)
    args = parser.parse_args()
    import numpy as np
    import torch
    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import SubprocVecEnv
    torch.set_num_threads(1)
    gpu = None
    if args.device == 'cuda':
        if not torch.cuda.is_available():
            parser.error('CUDA unavailable')
        # Exercise the actual GPU kernel; availability alone is insufficient.
        a = torch.arange(64, dtype=torch.float32, device='cuda').reshape(8, 8)
        product = a@a.T
        torch.cuda.synchronize()
        torch.testing.assert_close(product.cpu(), a.cpu()@a.cpu().T)
        gpu = torch.cuda.get_device_name(0)
    env = SubprocVecEnv([make_env(args.obj_size, 73+i) for i in range(2)], start_method='spawn')
    try:
        model = PPO('MlpPolicy', env, n_steps=32, batch_size=64, n_epochs=2,
                    seed=73, device=args.device, policy_kwargs={'net_arch':[32,32]})
        before = {k:v.clone() for k,v in model.policy.state_dict().items()}
        model.learn(total_timesteps=args.steps)
        assert any(not torch.equal(before[k], v) for k,v in model.policy.state_dict().items())
        observation = env.reset()
        assert np.isfinite(observation).all()
        action, _ = model.predict(observation, deterministic=True)
        with tempfile.TemporaryDirectory(prefix='qianli-model-') as directory:
            path = Path(directory)/'policy.zip'
            model.save(path)
            restored = PPO.load(path, device=args.device)
            restored_action, _ = restored.predict(observation, deterministic=True)
            np.testing.assert_array_equal(action, restored_action)
        print(json.dumps(dict(passed=True, obj_size_m=args.obj_size, seed=73,
                              workers=2, steps=model.num_timesteps, device=args.device,
                              gpu=gpu, torch=torch.__version__, model_roundtrip=True)))
    finally:
        env.close()


if __name__ == '__main__':
    main()
