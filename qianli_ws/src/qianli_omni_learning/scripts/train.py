#!/usr/bin/env python3
"""Train a local holonomic controller by seeded CPU cross-entropy search.

Only the declared training scene manifests are read during model selection.
Held-out evaluation happens once after writing the selected policy. The fixed
baseline has the same safety layer, observation API and command limits.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
import os
from pathlib import Path
import time

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
os.environ.setdefault('OMP_NUM_THREADS', '1')
import numpy as np

from environment import GeometryScene, run_episode
from policy import (BASELINE, LOWER, UPPER, HolonomicPolicy,
                    PARAMETER_NAMES, parameters_to_vector, vector_to_parameters)


WORKER_SCENES = []
WORKER_TASKS = []
WORKER_DT = .2


def write_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding='utf-8')
    temporary.replace(path)


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def initialize_worker(manifests, tasks, dt):
    global WORKER_SCENES, WORKER_TASKS, WORKER_DT
    WORKER_SCENES = [GeometryScene(path) for path in manifests]
    WORKER_TASKS = tasks
    WORKER_DT = dt


def summarize(episodes):
    success = sum(result['success'] for result in episodes)
    return {
        'episodes': len(episodes), 'successes': success,
        'success_rate': success / max(len(episodes), 1),
        'raw_collisions': sum(result['raw_collision'] for result in episodes),
        'padded_overlaps': sum(result['padded_overlap'] for result in episodes),
        'mean_elapsed_seconds': float(np.mean([r['elapsed_seconds'] for r in episodes])),
        'total_elapsed_seconds': sum(r['elapsed_seconds'] for r in episodes),
        'mean_path_length_m': float(np.mean([r['path_length_m'] for r in episodes])),
        'min_padded_sat_margin_m': min(r['min_padded_sat_margin_m'] for r in episodes),
        'mean_objective': float(np.mean([r['objective'] for r in episodes])),
    }


def evaluate_worker(vector):
    params = vector_to_parameters(vector)
    episodes = [run_episode(scene, task, HolonomicPolicy(params), dt=WORKER_DT,
                            max_seconds=task.get('max_sim_time_s', task.get('timeout_s', 150.)))
                for scene in WORKER_SCENES for task in WORKER_TASKS]
    return summarize(episodes)


def policy_document(params, kind, seed, manifests, task_path):
    return {
        'schema_version': 1, 'type': 'sensor_only_holonomic_parameter_policy',
        'kind': kind, 'params': params,
        'limits': {'max_translation_m_s': .25, 'max_yaw_rad_s': .60},
        'observation': {
            'goal_body': ['x_m', 'y_m'], 'yaw_error': 'rad',
            'scan': '72 uniformly spaced body-frame rays over 2*pi, metres',
            'scan_max_range_m': 4., 'dt': 'seconds',
        },
        'training': {
            'algorithm': 'CEM over eight bounded controller parameters',
            'seed': seed, 'manifests': [{'path': str(p), 'sha256': digest(p)} for p in manifests],
            'task_config_sha256': digest(task_path),
            'selection': 'Minimum training mean objective; held-out results never select parameters',
        },
        'limitations': [
            'Waypoint route is supplied by a caller; this is not a global planner.',
            '2D ideal kinematics only: no omni-wheel contact, friction or actuator dynamics.',
            'This parameter optimization is neither PPO nor deep reinforcement learning.',
        ],
    }


def detailed_evaluation(manifests, tasks, params, dt):
    episodes = [run_episode(GeometryScene(path), task, HolonomicPolicy(params), dt=dt,
                            max_seconds=task.get('max_sim_time_s', task.get('timeout_s', 150.)), record=True)
                for path in manifests for task in tasks]
    return {'summary': summarize(episodes), 'episodes': episodes}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene-root', required=True, type=Path)
    parser.add_argument('--tasks', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--iterations', type=int, default=6)
    parser.add_argument('--population', type=int, default=16)
    parser.add_argument('--workers', type=int, default=4)
    parser.add_argument('--seed', type=int, default=20261005)
    parser.add_argument('--dt', type=float, default=None)
    args = parser.parse_args()
    if args.iterations < 1 or args.population < 4 or args.workers < 1:
        raise ValueError('Need >=1 iteration, >=4 candidates and >=1 worker')
    args.output.mkdir(parents=True, exist_ok=True)
    configuration = json.loads(args.tasks.read_text(encoding='utf-8'))
    if args.dt is None:
        args.dt = configuration.get('shared_limits', {}).get('training_dt_s', .1)
    training_tasks = configuration.get('training_tasks', configuration.get('tasks'))
    evaluation_tasks = configuration.get('evaluation_tasks', training_tasks)
    train_variants = configuration.get('training_variants', ['train_000', 'train_001'])
    heldout_variants = configuration.get('evaluation_variants', ['test_101'])
    if set(train_variants) & set(heldout_variants):
        raise ValueError('Training and evaluation scene variants must be disjoint')
    training_paths = [args.scene_root / variant / 'manifest.json' for variant in train_variants]
    # Do not load heldout files until the final selected policy is saved.
    rng = np.random.default_rng(args.seed)
    baseline_vector = parameters_to_vector(BASELINE)
    mean = (baseline_vector - LOWER) / (UPPER - LOWER)
    std = np.full(len(mean), .24)
    best_vector = baseline_vector.copy()
    elite_count = max(2, args.population // 4)
    history = {
        'schema_version': 1, 'algorithm': 'Cross-Entropy Method', 'seed': args.seed,
        'parameter_names': list(PARAMETER_NAMES), 'lower_bounds': LOWER.tolist(),
        'upper_bounds': UPPER.tolist(), 'dt_s': args.dt,
        'population': args.population, 'elite_count': elite_count,
        'training_variants': train_variants, 'heldout_variants': heldout_variants,
        'training_tasks': [task['id'] for task in training_tasks],
        'objective': '1200*failure + 2000*padded_overlap + seconds + 1.5*path_m + .2*command_variation + 80*max(.10-margin,0) + 2*integrated_heading_error_rad_s',
        'iterations': [],
    }
    started = time.monotonic()
    baseline_document = policy_document(BASELINE, 'fixed_baseline', args.seed, training_paths, args.tasks)
    write_json(args.output / 'baseline_policy.json', baseline_document)
    with ProcessPoolExecutor(max_workers=args.workers, initializer=initialize_worker,
                             initargs=(training_paths, training_tasks, args.dt)) as pool:
        baseline_summary = next(pool.map(evaluate_worker, [baseline_vector]))
        best_score = baseline_summary['mean_objective']
        history['baseline_training'] = baseline_summary
        print(json.dumps({'baseline_training': baseline_summary}), flush=True)
        for iteration in range(args.iterations):
            candidates_normalized = np.clip(rng.normal(mean, std, size=(args.population - 1, len(mean))), 0., 1.)
            candidates = LOWER + candidates_normalized * (UPPER - LOWER)
            # Retain the best TRAINING checkpoint, including the fixed baseline.
            candidates = np.concatenate((best_vector[None, :], candidates), axis=0)
            summaries = list(pool.map(evaluate_worker, candidates))
            scores = np.array([item['mean_objective'] for item in summaries])
            indices = np.argsort(scores)
            if scores[indices[0]] < best_score:
                best_score = float(scores[indices[0]])
                best_vector = candidates[indices[0]].copy()
            elite = (candidates[indices[:elite_count]] - LOWER) / (UPPER - LOWER)
            mean = .35 * mean + .65 * elite.mean(axis=0)
            std = np.maximum(.035, .35 * std + .65 * elite.std(axis=0))
            item = {
                'iteration': iteration + 1, 'elapsed_wall_s': round(time.monotonic() - started, 3),
                'candidate_objectives': scores.tolist(),
                'candidate_params': [vector_to_parameters(candidate) for candidate in candidates],
                'candidate_summaries': summaries, 'elite_indices': indices[:elite_count].tolist(),
                'distribution_mean_normalized': mean.tolist(), 'distribution_std_normalized': std.tolist(),
                'best_training_objective': best_score, 'best_params': vector_to_parameters(best_vector),
            }
            history['iterations'].append(item)
            write_json(args.output / 'training_history.json', history)
            write_json(args.output / 'checkpoint.json', policy_document(
                vector_to_parameters(best_vector), 'training_checkpoint', args.seed, training_paths, args.tasks))
            print(json.dumps({'iteration': iteration + 1, 'best_training_objective': best_score,
                              'candidate_success_rates': [s['success_rate'] for s in summaries],
                              'wall_seconds': item['elapsed_wall_s']}), flush=True)
    selected = vector_to_parameters(best_vector)
    trained_document = policy_document(selected, 'cem_selected_on_training_only', args.seed, training_paths, args.tasks)
    trained_document['training']['iterations'] = args.iterations
    trained_document['training']['candidates_evaluated'] = 1 + args.iterations * args.population
    trained_document['training']['mean_objective'] = best_score
    write_json(args.output / 'trained_policy.json', trained_document)
    # The heldout split is opened for the FIRST time here, after model selection.
    heldout_paths = [args.scene_root / variant / 'manifest.json' for variant in heldout_variants]
    result = {
        'schema_version': 1, 'evaluation_stage': 'after frozen training policy selection',
        'dt_s': args.dt, 'baseline': {}, 'trained': {},
    }
    for label, params in [('baseline', BASELINE), ('trained', selected)]:
        result[label]['training'] = detailed_evaluation(training_paths, training_tasks, params, args.dt)
        result[label]['heldout'] = detailed_evaluation(heldout_paths, evaluation_tasks, params, args.dt)
    b = result['baseline']['heldout']['summary']
    t = result['trained']['heldout']['summary']
    result['heldout_comparison'] = {
        'baseline_success_rate': b['success_rate'], 'trained_success_rate': t['success_rate'],
        'baseline_seconds': b['total_elapsed_seconds'], 'trained_seconds': t['total_elapsed_seconds'],
        'time_change_percent': 100. * (t['total_elapsed_seconds'] - b['total_elapsed_seconds']) / b['total_elapsed_seconds'],
        'baseline_raw_collisions': b['raw_collisions'], 'trained_raw_collisions': t['raw_collisions'],
        'baseline_padded_overlaps': b['padded_overlaps'], 'trained_padded_overlaps': t['padded_overlaps'],
        'claim_rule': 'Call the trained policy better only if success and collision safety do not regress.',
    }
    result['wall_seconds'] = round(time.monotonic() - started, 3)
    history['completed_wall_seconds'] = result['wall_seconds']
    write_json(args.output / 'training_history.json', history)
    write_json(args.output / 'offline_comparison.json', result)
    print(json.dumps({'heldout_comparison': result['heldout_comparison'],
                      'wall_seconds': result['wall_seconds']}), flush=True)


if __name__ == '__main__':
    main()
