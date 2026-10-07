#!/usr/bin/env python3
"""Compatibility entry point for physics-only grasp acceptance.
Uses environment collision, limited commands, and a two-second hold.
See sim_grasp_validate.py --help. Failures return a nonzero exit code.
"""
import numpy as np
from sim_ik_dls import solve_only
from sim_grasp_validate import DEFAULT_APPROACH


def goto(model, data, qadr, aadr, target, steps=500, grip=0.0,
         tol=0.002, max_rounds=6, yaw=-90.0):
    """Compatibility helper: ramp commands instead of jumping targets.

    Historical steps/tol/max_rounds arguments are retained for callers.
    Acceptance uses its own 4mm stage tolerance and measured settling.
    """
    from sim_grasp_validate import Trial
    return Trial(model, data).move(np.asarray(target), grip, yaw=yaw)


def grasp(model, data, qadr, aadr, off_mm, obj_size, approach=DEFAULT_APPROACH,
          verbose=False):
    """Return acceptance result and measured physical metrics."""
    from sim_grasp_validate import Trial
    info = Trial(model, data).execute(off_mm, approach)
    info['up'] = info.get('lift_mm', 0.)
    if verbose:
        print(info, flush=True)
    return info['success'], info


def main():
    from sim_grasp_validate import main as validate
    return validate()


if __name__ == '__main__':
    raise SystemExit(main())
