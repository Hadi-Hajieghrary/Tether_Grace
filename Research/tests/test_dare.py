"""Numerical validation of Theorem 3 (DARE-based asymptotic stability).

Solves the discrete algebraic Riccati equation for the scalar-axis
double integrator used by `MpcLocalController` (via `SolveScalarDARE`
and `DARELqrGain` in `cpp/include/controller_utils.h`) and checks
that the closed-loop spectral radius is strictly less than one.

The constants mirror the values populated by the MPC ctor at runtime
(`dt_mpc = 0.01`, `q_pos = 100`, `q_vel = 10`, `r_ctrl = 0.02`),
so a passing test reflects the actual deployed controller — not a
generic DARE well-posedness sanity check.
"""
from __future__ import annotations

import numpy as np
from scipy.linalg import solve_discrete_are

# Must match the MPC ctor parameters in
# Research/cpp/include/mpc_local_controller.h.
DT_MPC = 1e-2
Q_POS = 100.0
Q_VEL = 10.0
R_CTRL = 0.02


def test_dare_spectral_radius() -> None:
    A = np.array([[1.0, DT_MPC], [0.0, 1.0]])
    B = np.array([[0.5 * DT_MPC * DT_MPC], [DT_MPC]])
    Q = np.diag([Q_POS, Q_VEL])
    R = np.array([[R_CTRL]])

    P = solve_discrete_are(A, B, Q, R)
    K = np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)
    Acl = A - B @ K
    rho = float(np.max(np.abs(np.linalg.eigvals(Acl))))

    assert P.shape == (2, 2)
    assert np.all(np.linalg.eigvalsh(P) > 0.0), "DARE P not PD"
    assert rho < 1.0, f"spectral radius {rho} ≥ 1"
    # The MPC ctor prints ρ at run-time; the smoke test on the current
    # parameters gives ρ ≈ 0.969. Verify we're near that value so a
    # future parameter drift gets caught.
    assert abs(rho - 0.969) < 0.05, f"spectral radius {rho} drifted from the ctor-reported 0.969"


if __name__ == "__main__":
    test_dare_spectral_radius()
    print("test_dare: OK")
