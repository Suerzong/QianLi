#!/usr/bin/env python3
"""Sensor-only holonomic velocity policy shared by offline training and ROS.

This is a parameterized local controller, not a neural network or a global
planner. The caller supplies a body-frame waypoint and heading error. Collision
objects and absolute pose are deliberately absent from the policy API.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np


PARAMETER_NAMES = (
    'preferred_speed', 'goal_gain', 'repulsion_gain', 'repulsion_radius',
    'tangent_gain', 'acceleration_limit', 'yaw_gain', 'preview_time',
)
LOWER = np.array([.16, .45, .04, .40, .00, .20, .6, .25])
UPPER = np.array([.25, 1.40, .42, 1.30, .28, .65, 2.0, 1.10])
BASELINE = dict(zip(PARAMETER_NAMES, [.24, .8, .18, .8, .10, .40, 1.20, .60]))
RAW_FOOTPRINT = np.array([
    [.35, .20], [.20, .35], [-.20, .35], [-.35, .20],
    [-.35, -.20], [-.20, -.35], [.20, -.35], [.35, -.20],
])
PADDING = .06


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def vector_to_parameters(vector):
    return dict(zip(PARAMETER_NAMES, np.clip(vector, LOWER, UPPER).tolist()))


def parameters_to_vector(parameters):
    return np.array([float(parameters[name]) for name in PARAMETER_NAMES])


def load_parameters(path):
    document = json.loads(Path(path).read_text(encoding='utf-8'))
    return document.get('params', document.get('parameters', document))


class HolonomicPolicy:
    """Return body vx, vy, wz using a local target and an instantaneous scan.

    scan_angles are radians in the BODY frame, not sensor-frame angles. A ROS
    wrapper must apply the lidar static transform, including its translation.
    Inputs may have any ray count >= 24; +inf ranges mean no return. dt is in
    seconds. reset() is required when beginning a new independent episode.
    """

    def __init__(self, params=None):
        self.params = dict(BASELINE if params is None else params)
        missing = set(PARAMETER_NAMES) - set(self.params)
        if missing:
            raise ValueError('Missing parameters: ' + ', '.join(sorted(missing)))
        values = parameters_to_vector(self.params)
        if not np.all(np.isfinite(values)) or np.any(values < LOWER) or np.any(values > UPPER):
            raise ValueError('Policy parameters outside the documented search bounds')
        edges = np.roll(RAW_FOOTPRINT, -1, axis=0) - RAW_FOOTPRINT
        self.normals = np.column_stack((edges[:, 1], -edges[:, 0]))
        self.normals /= np.linalg.norm(self.normals, axis=1)[:, None]
        self.constants = np.sum(self.normals * RAW_FOOTPRINT, axis=1) + PADDING
        self.previous = np.zeros(3)

    def reset(self):
        self.previous[:] = 0.

    def action(self, goal_body, yaw_error, scan_ranges, scan_angles, dt):
        p = self.params
        goal = np.asarray(goal_body, dtype=float)
        ranges = np.asarray(scan_ranges, dtype=float)
        angles = np.asarray(scan_angles, dtype=float)
        if goal.shape != (2,) or ranges.ndim != 1 or ranges.shape != angles.shape:
            raise ValueError('Expected goal_body[2] and equally sized scan vectors')
        if not np.all(np.isfinite(goal)) or not math.isfinite(yaw_error) or dt <= 0:
            raise ValueError('Invalid goal, yaw error or dt')
        valid = np.isfinite(ranges) & (ranges > .01)
        if np.count_nonzero(valid) < 8:
            self.previous[:] = 0.
            return self.previous.copy()
        rays = np.column_stack((np.cos(angles[valid]), np.sin(angles[valid])))
        endpoints = rays * ranges[valid, None]
        # Half-space distance is a conservative lower bound on true Euclidean
        # point-to-polygon distance, and exactly correct along flat edges.
        clearance = np.max(endpoints @ self.normals.T - self.constants, axis=1)
        distance = float(np.linalg.norm(goal))
        if distance < .075:
            command = np.array([0., 0., np.clip(p['yaw_gain'] * wrap(yaw_error), -.60, .60)])
            self.previous = command
            return command.copy()
        direction = goal / max(distance, 1e-9)
        speed = min(p['preferred_speed'], p['goal_gain'] * distance)
        desired = speed * direction
        influence = np.clip(1. - clearance / p['repulsion_radius'], 0., 1.) ** 2
        if influence.max(initial=0.) > 0:
            strongest = influence >= .5 * influence.max()
            weights = influence[strongest]
            away = -np.sum(rays[strongest] * weights[:, None], axis=0) / max(weights.sum(), 1e-9)
            desired += p['repulsion_gain'] * influence.max() * away
            obstruction = (rays @ direction > .75) & (clearance < p['repulsion_radius'])
            if obstruction.any():
                closest = int(np.argmin(np.where(obstruction, clearance, np.inf)))
                tangent = np.array([-rays[closest, 1], rays[closest, 0]])
                alignment = float(tangent @ direction)
                if alignment < -.02:
                    tangent *= -1
                desired += p['tangent_gain'] * influence[closest] * tangent

        norm = float(np.linalg.norm(desired))
        desired *= min(1., p['preferred_speed'] / max(norm, 1e-9))
        wz = np.clip(p['yaw_gain'] * wrap(yaw_error), -.60, .60)
        # A fixed sensor-only short-horizon safety filter scores headings around
        # the potential-field command. This is shared unchanged by both policies.
        heading = math.atan2(desired[1], desired[0]) if norm > 1e-7 else math.atan2(direction[1], direction[0])
        offsets = np.deg2rad([0, 15, -15, 30, -30, 45, -45, 60, -60, 90, -90, 120, -120, 180])
        candidate_speed = float(np.linalg.norm(desired))
        candidate_angles = heading + offsets
        velocities = np.column_stack((np.cos(candidate_angles), np.sin(candidate_angles))) * candidate_speed
        velocities = np.concatenate((velocities, velocities * .45, np.zeros((1, 2))), axis=0)
        # Enforce acceleration before scoring so the executed velocity is the
        # one checked by the filter, including its distance from the last command.
        differences = velocities - self.previous[:2]
        magnitudes = np.linalg.norm(differences, axis=1)
        velocities = self.previous[:2] + differences * np.minimum(
            1., p['acceleration_limit'] * dt / np.maximum(magnitudes, 1e-9))[:, None]
        rotation = float(wz) * p['preview_time']
        c, s = math.cos(rotation), math.sin(rotation)
        if abs(wz) > 1e-5:
            displacement = velocities @ np.array([[s / wz, (1. - c) / wz],
                                                   [(c - 1.) / wz, s / wz]])
        else:
            displacement = velocities * p['preview_time']
        future = endpoints[None, :, :] - displacement[:, None, :]
        future = future @ np.array([[c, -s], [s, c]])
        margin = np.max(future @ self.normals.T - self.constants, axis=2)
        nearest = np.min(margin, axis=1)
        # 72 finite beams undersample continuous obstacle faces. A common 4 cm
        # guard protects against this sensor discretization in both policies.
        unsafe = nearest < .04
        progress = velocities @ direction
        deviations = np.linalg.norm(velocities - desired, axis=1)
        effort = np.linalg.norm(velocities - self.previous[:2], axis=1)
        scores = progress - .15 * deviations - .04 * effort
        scores -= .03 * np.maximum(.15 - nearest, 0.)
        scores[unsafe] = -np.inf
        if np.isfinite(scores).any():
            best = int(np.argmax(scores))
            command = np.array([velocities[best, 0], velocities[best, 1], wz])
        else:
            command = np.zeros(3)
        # At a reached waypoint stop without residual velocity.
        self.previous = command
        return command.copy()
