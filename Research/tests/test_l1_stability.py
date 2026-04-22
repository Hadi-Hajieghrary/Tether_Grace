"""Numerical validation of Proposition 1 and Corollary 1 (L1 layer).

Checks:
  * `A_m` eigenvalues match the closed-form derivation.
  * The symbolic Lyapunov matrix P satisfies `A_mᵀ P + P A_m = −I`.
  * The gain bound Γ* = 2/(T_s · p_{22}) ≈ 4.75e4.
  * The Euler-discretised σ̂ update is a contraction for Γ = 0.5 Γ*
    and diverges for Γ = 1.2 Γ* (empirical boundary).
"""
from __future__ import annotations

import numpy as np

KP = 100.0
KD = 24.0
TS = 2e-4


def test_eigenvalues() -> None:
    Am = np.array([[0.0, 1.0], [-KP, -KD]])
    lam = np.sort(np.linalg.eigvals(Am).real)
    disc = (KD / 2.0) ** 2 - KP
    assert disc > 0.0, "expected over-damped eigen pair"
    analytic = sorted((-KD / 2.0 + np.sqrt(disc),
                       -KD / 2.0 - np.sqrt(disc)))
    assert abs(lam[0] - analytic[0]) < 1e-9, (lam[0], analytic[0])
    assert abs(lam[1] - analytic[1]) < 1e-9, (lam[1], analytic[1])


def test_lyapunov_matrix() -> None:
    Am = np.array([[0.0, 1.0], [-KP, -KD]])
    p12 = 1.0 / (2.0 * KP)
    p22 = (p12 + 0.5) / KD
    p11 = KP * p22 + KD * p12
    P = np.array([[p11, p12], [p12, p22]])
    lhs = Am.T @ P + P @ Am
    assert np.allclose(lhs, -np.eye(2), atol=1e-10), lhs


def test_gain_bound() -> None:
    p22 = (1.0 / (2.0 * KP) + 0.5) / KD
    gamma_star = 2.0 / (TS * p22)
    # Γ* = 2/(T_s · p_{22}) ≈ 4.75e5 for T_s = 2e-4 s.
    assert abs(gamma_star - 4.75e5) / 4.75e5 < 0.01, gamma_star


def test_euler_contraction() -> None:
    p22 = (1.0 / (2.0 * KP) + 0.5) / KD
    gamma_star = 2.0 / (TS * p22)

    def simulate(gamma: float, steps: int) -> float:
        sigma_hat = 1.0
        for _ in range(steps):
            sigma_hat -= TS * gamma * p22 * sigma_hat
        return abs(sigma_hat)

    assert simulate(0.5 * gamma_star, 200) < 0.5, "contraction failed for Γ = 0.5 Γ*"
    # Just past Γ*, the discrete update amplifies by factor ≈ 1.4; after
    # only 50 steps the trajectory has grown by more than 3 orders of
    # magnitude, which is sufficient evidence of divergence without
    # letting the simulation overflow the float range.
    assert simulate(1.2 * gamma_star, 50) > 1.0, "stability bound not tight"


if __name__ == "__main__":
    test_eigenvalues()
    test_lyapunov_matrix()
    test_gain_bound()
    test_euler_contraction()
    print("test_l1_stability: OK")
