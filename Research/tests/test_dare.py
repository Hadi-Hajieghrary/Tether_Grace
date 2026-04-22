"""Numerical validation of Theorem 3 (DARE-based asymptotic stability).

Solves the discrete Riccati equation for the scalar-axis double
integrator used by `MpcLocalController::CalcInitialGain` and checks
that the closed-loop spectral radius is strictly less than one.
"""
from __future__ import annotations

import numpy as np
from scipy.linalg import solve_discrete_are


def test_dare_spectral_radius() -> None:
    Ts = 1e-2
    A = np.array([[1.0, Ts], [0.0, 1.0]])
    B = np.array([[0.5 * Ts * Ts], [Ts]])
    Q = np.diag([100.0, 10.0])
    R = np.array([[1.0]])

    P = solve_discrete_are(A, B, Q, R)
    K = np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)
    Acl = A - B @ K
    rho = float(np.max(np.abs(np.linalg.eigvals(Acl))))

    assert P.shape == (2, 2)
    assert np.all(np.linalg.eigvalsh(P) > 0.0), "DARE P not PD"
    assert rho < 1.0, f"spectral radius {rho} ≥ 1"


if __name__ == "__main__":
    test_dare_spectral_radius()
    print("test_dare: OK")
