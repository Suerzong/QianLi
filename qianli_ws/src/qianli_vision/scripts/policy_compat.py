"""Load project PPO archives across the NumPy 2 -> 1.26 migration.

SB3 stores Box spaces with pickle, including their unused sampling RNG. NumPy 2
changed both array module names and RNG pickle signatures. Rebuild those spaces
from SB3's accompanying JSON description; leave policy and optimizer data intact.
As with PPO.load, only load trusted model archives.
"""
import json
from pathlib import Path
import zipfile


def load_grasp_policy(path, *, device='auto', env=None):
    import numpy as np
    from gymnasium.spaces import Box
    from stable_baselines3 import PPO

    archive = Path(path)
    if not archive.is_file() and archive.suffix != '.zip':
        archive = Path(str(archive) + '.zip')
    with zipfile.ZipFile(archive) as bundle:
        data = json.loads(bundle.read('data'))
    spaces = {}
    for key, dimension in [('observation_space', 22), ('action_space', 6)]:
        description = data[key]
        if (description.get(':type:') != "<class 'gymnasium.spaces.box.Box'>"
                or description.get('_shape') != [dimension]
                or description.get('dtype') != 'float32'):
            raise ValueError(f'{key} does not match the QianLi grasp environment')
        bounds = []
        for name in ('low', 'high'):
            value = np.fromstring(description[name].strip('[]'), sep=' ', dtype=np.float32)
            if value.shape != (dimension,) or np.isnan(value).any():
                raise ValueError(f'Invalid {key}.{name} in model metadata')
            bounds.append(value)
        spaces[key] = Box(*bounds, dtype=np.float32)
        expected = getattr(env, key, None) if env is not None else None
        if expected is not None and spaces[key] != expected:
            raise ValueError(f'Archived {key} differs from the selected environment')
    # A new simulator cannot resume the archived live episode. PPO.load defaults
    # to force_reset=True; clear the remaining per-episode NumPy snapshots too.
    spaces.update(_last_obs=None, _last_original_obs=None, _last_episode_starts=None)
    return PPO.load(archive, device=device, env=env, custom_objects=spaces)
