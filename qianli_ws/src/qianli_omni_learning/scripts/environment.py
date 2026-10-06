#!/usr/bin/env python3
"""Pure NumPy 2D kinematic evaluation environment from scene manifests.

Policy observations contain no object list or true absolute robot pose. True
pose is used only for rendering a raycast sensor and objective/evaluation.
No traction, inertia, wheel contacts or motor dynamics are simulated.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

from policy import PADDING, RAW_FOOTPRINT, wrap


class GeometryScene:
    def __init__(self, path):
        self.path = str(path)
        self.document = json.loads(Path(path).read_text(encoding='utf-8'))
        self.variant = self.document['variant']
        objects = self.document['obstacles']
        self.centers = np.array([[o['x'], o['y']] for o in objects], dtype=float)
        self.half = np.array([[o['sx'] / 2, o['sy'] / 2] for o in objects], dtype=float)
        self.yaws = np.array([o.get('yaw', 0.) for o in objects])
        self.cos, self.sin = np.cos(self.yaws), np.sin(self.yaws)
        self.radii = np.linalg.norm(self.half, axis=1)
        self.ids = [o['id'] for o in objects]
        self.vertices = []
        for center, half, c, s in zip(self.centers, self.half, self.cos, self.sin):
            local = np.array([[-half[0], -half[1]], [half[0], -half[1]],
                              [half[0], half[1]], [-half[0], half[1]]])
            self.vertices.append(local @ np.array([[c, s], [-s, c]]) + center)
        self.vertices = np.array(self.vertices)
        self.angles = np.linspace(-math.pi, math.pi, 72, endpoint=False)
        edges = np.roll(RAW_FOOTPRINT, -1, axis=0) - RAW_FOOTPRINT
        normals = np.column_stack((edges[:, 1], -edges[:, 0]))
        normals /= np.linalg.norm(normals, axis=1)[:, None]
        constants = np.sum(normals * RAW_FOOTPRINT, axis=1) + PADDING
        padded = []
        for i, current in enumerate(normals):
            previous = normals[(i - 1) % len(normals)]
            padded.append(np.linalg.solve(np.array([previous, current]),
                                          [constants[(i - 1) % len(constants)], constants[i]]))
        self.padded = np.array(padded)

    def scan(self, pose, max_range=4.):
        """Exact oriented-box slab intersections along 72 rays at the body origin."""
        xy, yaw = pose[:2], pose[2]
        nearby = np.flatnonzero(np.linalg.norm(self.centers - xy, axis=1) < self.radii + max_range)
        relative = xy - self.centers[nearby]
        c, s = self.cos[nearby], self.sin[nearby]
        origins = np.column_stack((c * relative[:, 0] + s * relative[:, 1],
                                   -s * relative[:, 0] + c * relative[:, 1]))
        directions = self.angles[None, :] + yaw - self.yaws[nearby, None]
        ray = np.stack((np.cos(directions), np.sin(directions)), axis=-1)
        safe = np.where(abs(ray) < 1e-12, np.copysign(1e-12, ray), ray)
        lower = (-self.half[nearby, None, :] - origins[:, None, :]) / safe
        upper = (self.half[nearby, None, :] - origins[:, None, :]) / safe
        entry = np.max(np.minimum(lower, upper), axis=-1)
        exit = np.min(np.maximum(lower, upper), axis=-1)
        hits = np.where((exit >= np.maximum(entry, 0.)) & (entry >= 0.), entry, max_range)
        return np.minimum(max_range, hits.min(axis=0, initial=max_range))

    def clearance(self, pose, padded=True):
        """Exact polygon intersection; separating gap gives a conservative margin."""
        robot_local = self.padded if padded else RAW_FOOTPRINT
        c, s = math.cos(pose[2]), math.sin(pose[2])
        robot = robot_local @ np.array([[c, s], [-s, c]]) + pose[:2]
        distances = np.linalg.norm(self.centers - pose[:2], axis=1) - self.radii
        nearby = np.flatnonzero(distances < 2.)
        if not len(nearby):
            return float(distances.min(initial=2.)), None
        boxes = self.vertices[nearby]
        # A convex polygon pair intersects if no edge normal separates it.
        robot_edges = np.roll(robot, -1, axis=0) - robot
        robot_axes = np.column_stack((-robot_edges[:, 1], robot_edges[:, 0]))
        robot_axes /= np.linalg.norm(robot_axes, axis=1)[:, None]
        rp = robot @ robot_axes.T
        bp = boxes @ robot_axes.T
        robot_gaps = np.maximum(bp.min(axis=1) - rp.max(axis=0), rp.min(axis=0) - bp.max(axis=1))
        c, s = self.cos[nearby], self.sin[nearby]
        box_axes = np.stack((np.column_stack((c, s)), np.column_stack((-s, c))), axis=1)
        rp = np.einsum('va,nka->nvk', robot, box_axes)
        bp = np.einsum('nva,nka->nvk', boxes, box_axes)
        box_gaps = np.maximum(bp.min(axis=1) - rp.max(axis=1), rp.min(axis=1) - bp.max(axis=1))
        all_gaps = np.maximum(robot_gaps.max(axis=1), box_gaps.max(axis=1))
        best = int(np.argmin(all_gaps))
        return float(all_gaps[best]), self.ids[nearby[best]]


def polyline_distance(point, points):
    starts, ends = points[:-1], points[1:]
    deltas = ends - starts
    lengths_squared = np.sum(deltas * deltas, axis=1)
    fractions = np.clip(np.sum((point - starts) * deltas, axis=1) / np.maximum(lengths_squared, 1e-12), 0., 1.)
    return float(np.linalg.norm(point - starts - fractions[:, None] * deltas, axis=1).min())


def run_episode(scene, task, policy, dt=.1, max_seconds=150., record=False):
    pose = np.array(task['start'], dtype=float)
    waypoint_data = task['waypoints']
    targets = [np.array([*point[:2], point[2] if len(point) > 2 else task.get('yaw', pose[2])])
               for point in waypoint_data]
    waypoint = 0
    policy.reset()
    elapsed = path_length = effort = heading_error_sum = 0.
    combined_seconds = maximum_yaw_deviation = maximum_cross_track = maximum_drift = 0.
    maximum_reference_deviation = maximum_tracked_yaw_error = 0.
    start = pose.copy()
    reference = np.array([start[:2], *[point[:2] for point in targets]])
    tolerances = task.get('tolerances', {})
    position_tolerance = tolerances.get('position_m', .12)
    yaw_tolerance = tolerances.get('yaw_rad', .08)
    minimum_margin = float('inf')
    collision = padded_overlap = False
    trajectory = [pose.tolist()] if record else None
    previous_command = np.zeros(3)
    initial_margin, _ = scene.clearance(pose)
    if initial_margin <= 0:
        raise ValueError('Task starts in padded collision: ' + task['id'])
    while elapsed < max_seconds:
        target = targets[waypoint]
        error = target[:2] - pose[:2]
        distance = float(np.linalg.norm(error))
        yaw_error = wrap(target[2] - pose[2])
        if distance < position_tolerance and abs(yaw_error) < yaw_tolerance:
            waypoint += 1
            if waypoint == len(targets):
                break
            continue
        c, s = math.cos(pose[2]), math.sin(pose[2])
        goal_body = [c * error[0] + s * error[1], -s * error[0] + c * error[1]]
        scan = scene.scan(pose)
        command = policy.action(goal_body, yaw_error, scan, scene.angles, dt)
        previous_pose = pose.copy()
        # Two integration samples per action bound travel to <= 0.032 m.
        for _ in range(2):
            c, s = math.cos(pose[2]), math.sin(pose[2])
            velocity_world = np.array([c * command[0] - s * command[1],
                                       s * command[0] + c * command[1]])
            pose[:2] += velocity_world * (dt / 2)
            pose[2] = wrap(pose[2] + command[2] * (dt / 2))
            margin, _ = scene.clearance(pose)
            minimum_margin = min(minimum_margin, margin)
            if margin <= 0:
                padded_overlap = True
                raw_margin, _ = scene.clearance(pose, padded=False)
                collision = raw_margin <= 0
                break
        elapsed += dt
        path_length += float(np.linalg.norm(command[:2])) * dt
        effort += float(np.linalg.norm(command - previous_command))
        previous_command = command
        heading_error_sum += abs(yaw_error) * dt
        physical_speed = float(np.linalg.norm(pose[:2] - previous_pose[:2])) / dt
        physical_yaw_rate = abs(wrap(pose[2] - previous_pose[2])) / dt
        if physical_speed >= .04 and physical_yaw_rate >= .08:
            combined_seconds += dt
        maximum_yaw_deviation = max(maximum_yaw_deviation, abs(wrap(pose[2] - start[2])))
        maximum_cross_track = max(maximum_cross_track, abs(float(pose[0] - start[0])))
        maximum_drift = max(maximum_drift, float(np.linalg.norm(pose[:2] - start[:2])))
        maximum_reference_deviation = max(maximum_reference_deviation, polyline_distance(pose[:2], reference))
        maximum_tracked_yaw_error = max(maximum_tracked_yaw_error, abs(yaw_error))
        if record:
            trajectory.append(pose.tolist())
        if padded_overlap:
            break
    final_error = float(np.linalg.norm(pose[:2] - targets[-1][:2]))
    final_yaw_error = abs(wrap(pose[2] - targets[-1][2]))
    thresholds = task.get('thresholds', {})
    constraints = {
        'all_waypoints_completed': waypoint == len(targets),
        'within_sim_timeout': elapsed <= max_seconds + 1e-9,
        'physical_final_position': final_error <= thresholds.get('final_position_m', .15),
        'physical_final_yaw': final_yaw_error <= thresholds.get('final_yaw_rad', .12),
        'zero_raw_intersections': not collision, 'zero_padded_intersections': not padded_overlap,
    }
    if task.get('yaw_mode') == 'fixed':
        constraints['fixed_heading_max'] = maximum_yaw_deviation <= thresholds.get('fixed_heading_max_rad', .1)
    if task.get('pure_lateral'):
        constraints['lateral_cross_track'] = maximum_cross_track <= thresholds.get('pure_lateral_cross_track_m', .15)
    if task.get('pure_rotation'):
        constraints['rotation_position_drift'] = maximum_drift <= thresholds.get('rotation_drift_m', .06)
    if task.get('require_combined_motion'):
        constraints['combined_motion_duration'] = combined_seconds >= thresholds.get('combined_motion_min_s', .5)
        constraints['tracked_yaw_reference'] = maximum_tracked_yaw_error <= thresholds.get('max_yaw_error_rad', .35)
    success = all(constraints.values())
    # Failures dominate efficiency. The low-clearance term is integrated into
    # the metric without using test episodes to alter learned parameters.
    objective = (0. if success else 1200.) + elapsed + 1.5 * path_length + .2 * effort
    # Use an INTEGRAL, never an average: averaging would reward an artificially
    # long low-error settling tail on pure rotations.
    objective += 80. * max(.10 - minimum_margin, 0.) + 2. * heading_error_sum
    if padded_overlap:
        objective += 2000.
    result = {
        'scene': scene.variant, 'task': task['id'], 'success': success,
        'raw_collision': collision, 'padded_overlap': padded_overlap,
        'elapsed_seconds': round(elapsed, 4), 'path_length_m': round(path_length, 4),
        'min_padded_sat_margin_m': round(minimum_margin, 5),
        'mean_heading_error_rad': round(heading_error_sum / max(elapsed, dt), 6),
        'final_error_m': round(final_error, 5), 'final_yaw_error_rad': round(final_yaw_error, 6),
        'max_heading_deviation_rad': round(maximum_yaw_deviation, 6),
        'max_lateral_cross_track_m': round(maximum_cross_track, 6),
        'max_rotation_drift_m': round(maximum_drift, 6),
        'combined_motion_seconds': round(combined_seconds, 6),
        'max_reference_deviation_m': round(maximum_reference_deviation, 6),
        'max_tracked_yaw_error_rad': round(maximum_tracked_yaw_error, 6),
        'constraints': constraints, 'waypoints_completed': waypoint,
        'objective': round(objective, 5), 'final_pose': pose.tolist(),
    }
    if record:
        result['trajectory'] = trajectory
    return result
