"""Scoring-only map coverage over a fixed, truth-derived reachable domain.

The exploration policy must never import this module or receive its reference
map. Alignment here is fixed at the simulation spawn pose; the metric measures
2D map coverage, not reconstruction accuracy or drift-free registration.
"""
import math
import cv2
import numpy as np
from frontier_core import Grid, safe_cells


def reachable_domain(reference, spawn, radius=.47, bounds=None):
    cells = reference.cells.copy()
    rows, cols = np.indices(cells.shape)
    x, y = reference.world(cols, rows)
    if bounds is not None:
        xmin, ymin, xmax, ymax = bounds
        cells[(x < xmin) | (x > xmax) | (y < ymin) | (y > ymax)] = 100
    safe = safe_cells(Grid(cells, reference.resolution, reference.origin_x,
                           reference.origin_y, reference.origin_yaw), radius)
    c, r = reference.cell(*spawn[:2])
    if not (0 <= r < safe.shape[0] and 0 <= c < safe.shape[1] and safe[r, c]):
        raise ValueError('Reference spawn must be in body-safe free space')
    _, labels = cv2.connectedComponents(safe.astype(np.uint8), connectivity=4)
    return labels == labels[r, c]


def map_coverage(reference, domain, mapped, spawn):
    if domain.shape != reference.cells.shape or not np.any(domain):
        raise ValueError('Expected a nonempty reference-domain mask')
    rows, cols = np.nonzero(domain)
    wx, wy = reference.world(cols, rows)
    dx, dy = wx-spawn[0], wy-spawn[1]
    c, s = math.cos(spawn[2]), math.sin(spawn[2])
    # The initial odometry / SLAM frame starts at the configured spawn.
    mx, my = c*dx+s*dy, -s*dx+c*dy
    dx, dy = mx-mapped.origin_x, my-mapped.origin_y
    c, s = math.cos(mapped.origin_yaw), math.sin(mapped.origin_yaw)
    mc = np.floor((c*dx+s*dy)/mapped.resolution).astype(np.int64)
    mr = np.floor((-s*dx+c*dy)/mapped.resolution).astype(np.int64)
    h, w = mapped.cells.shape
    inside = (mr >= 0) & (mr < h) & (mc >= 0) & (mc < w)
    values = np.full(len(rows), -1, np.int16)
    values[inside] = mapped.cells[mr[inside], mc[inside]]
    total = len(rows)
    free = int(((values >= 0) & (values <= 20)).sum())
    known = int((values >= 0).sum())
    return {'reachable_cells': total, 'reachable_area_m2': total*reference.resolution**2,
            'mapped_free_cells': free, 'mapped_known_cells': known,
            'free_coverage_pct': 100*free/total, 'known_coverage_pct': 100*known/total,
            'unknown_reachable_cells': total-known,
            'alignment': 'Fixed simulation spawn -> initial SLAM frame; not an accuracy metric'}
