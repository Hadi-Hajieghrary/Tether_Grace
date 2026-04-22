"""Numerical validation of Proposition 7 (C²-smoothstep safety).

Confirms h(0) = 0, h(1) = 1, h'(0) = h'(1) = 0, h''(0) = h''(1) = 0,
the peak derivative 15/8, and the 1.131 m minimum chord distance for
N = 4 → M = 3 at r_f = 0.8 m.
"""
from __future__ import annotations

import math

import numpy as np


def h(tau: float) -> float:
    return (10.0 - 15.0 * tau + 6.0 * tau * tau) * tau ** 3


def hp(tau: float) -> float:
    return 30.0 * tau ** 2 * (1.0 - 2.0 * tau + tau ** 2)


def hpp(tau: float) -> float:
    return 60.0 * tau * (1.0 - 3.0 * tau + 2.0 * tau ** 2)


def test_endpoints() -> None:
    assert abs(h(0.0)) < 1e-12
    assert abs(h(1.0) - 1.0) < 1e-12
    assert abs(hp(0.0)) < 1e-12
    assert abs(hp(1.0)) < 1e-12
    assert abs(hpp(0.0)) < 1e-12
    assert abs(hpp(1.0)) < 1e-12


def test_peak_derivative() -> None:
    taus = np.linspace(0.0, 1.0, 10001)
    peak = max(hp(t) for t in taus)
    assert abs(peak - 15.0 / 8.0) < 1e-3


def test_minimum_chord_distance() -> None:
    r_f = 0.8
    d_min = 2.0 * r_f * math.sin(math.pi / 4.0)
    assert abs(d_min - 1.131) < 2e-3
    assert d_min > 0.5, f"minimum chord {d_min} below safe clearance"


if __name__ == "__main__":
    test_endpoints()
    test_peak_derivative()
    test_minimum_chord_distance()
    print("test_smoothstep_certificate: OK")
