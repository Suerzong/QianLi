#!/usr/bin/env python3
"""Evaluate frozen policy files, optionally removing preferred-speed advantage.

This command cannot train or save policy parameters. The speed-matched baseline
is an explanatory ablation only; it never influences CEM checkpoint selection.
"""
import argparse
import json
from pathlib import Path

from policy import load_parameters
from train import detailed_evaluation, digest, write_json


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene-root', required=True, type=Path)
    parser.add_argument('--tasks', required=True, type=Path)
    parser.add_argument('--baseline', required=True, type=Path)
    parser.add_argument('--trained', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--dt', type=float, default=None)
    parser.add_argument('--speed-ablation', action='store_true')
    args = parser.parse_args()
    cfg = json.loads(args.tasks.read_text(encoding='utf-8'))
    tasks = cfg['evaluation_tasks']
    dt = args.dt if args.dt is not None else cfg['shared_limits']['training_dt_s']
    paths = [args.scene_root / variant / 'manifest.json' for variant in cfg['evaluation_variants']]
    hashes = {'baseline': digest(args.baseline), 'trained': digest(args.trained)}
    params = {'baseline': load_parameters(args.baseline), 'trained': load_parameters(args.trained)}
    if args.speed_ablation:
        speed_matched = dict(params['baseline'])
        speed_matched['preferred_speed'] = params['trained']['preferred_speed']
        params['baseline_speed_matched'] = speed_matched
    result = {
        'schema_version': 1, 'stage': 'Frozen policy evaluation, no checkpoint selection',
        'dt_s': dt, 'policy_sha256': hashes, 'task_config_sha256': digest(args.tasks),
        'manifests': [{'path': str(p), 'sha256': digest(p)} for p in paths],
        'results': {name: detailed_evaluation(paths, tasks, parameters, dt)
                    for name, parameters in params.items()},
    }
    baseline = result['results']['baseline']['summary']
    trained = result['results']['trained']['summary']
    result['comparison'] = {
        'baseline_successes': baseline['successes'], 'trained_successes': trained['successes'],
        'baseline_seconds': baseline['total_elapsed_seconds'], 'trained_seconds': trained['total_elapsed_seconds'],
        'time_change_percent': 100. * (trained['total_elapsed_seconds'] - baseline['total_elapsed_seconds']) / baseline['total_elapsed_seconds'],
    }
    if args.speed_ablation:
        matched = result['results']['baseline_speed_matched']['summary']
        result['comparison']['speed_matched_baseline_seconds'] = matched['total_elapsed_seconds']
        result['comparison']['beyond_preferred_speed_time_change_percent'] = 100. * (
            trained['total_elapsed_seconds'] - matched['total_elapsed_seconds']) / matched['total_elapsed_seconds']
    assert hashes == {'baseline': digest(args.baseline), 'trained': digest(args.trained)}
    write_json(args.output, result)
    print(json.dumps(result['comparison'], indent=2))


if __name__ == '__main__':
    main()
