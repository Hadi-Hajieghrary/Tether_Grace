"""Numerical validation of Theorem 5 (reshape optimality).

Checks that for N=4 → M=3 the ±30° symmetric rotation dominates every
other surviving-triple assignment on a 1° angular grid.
"""
from __future__ import annotations

import itertools
import math

import numpy as np


def peak_tension_asymmetry(phis: list[float]) -> float:
    ux = sum(math.cos(p) for p in phis) / len(phis)
    uy = sum(math.sin(p) for p in phis) / len(phis)
    return math.hypot(ux, uy)


def test_thirty_degree_rotation() -> None:
    best_value = math.inf
    best_cfg = None
    step = math.radians(1.0)
    candidate_angles = np.arange(-math.pi, math.pi, step)
    gap = 0.0
    baseline = [math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0]

    for delta1, delta2 in itertools.product(candidate_angles, repeat=2):
        cfg = [
            baseline[0] + delta1,
            baseline[1],
            baseline[2] + delta2,
        ]
        cost = peak_tension_asymmetry(cfg)
        if cost < best_value:
            best_value = cost
            best_cfg = (delta1, delta2)

    d1, d2 = best_cfg
    assert abs(d1 - (-math.pi / 6.0)) < math.radians(1.5), d1
    assert abs(d2 - ( math.pi / 6.0)) < math.radians(1.5), d2


if __name__ == "__main__":
    test_thirty_degree_rotation()
    print("test_reshape_optimality: OK")
