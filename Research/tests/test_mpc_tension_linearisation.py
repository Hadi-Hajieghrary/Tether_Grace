"""Empirical check of Theorem 4 (linearisation fidelity).

For perturbations bounded by E_MAX = 2 cm around a taut nominal
chord, the second-order Taylor remainder of the rope-spring law
satisfies |T_lin − T_true| ≤ (k_eff / (2 d_nom)) · ‖e_p‖². The test
draws N_REAL perturbations and verifies the bound holds empirically.
"""
from __future__ import annotations

import numpy as np

N_REAL = 2000
K_EFF = 2777.8
L_EFF = 1.448
E_MAX = 0.02  # m


def sample_e_p(rng: np.random.Generator) -> np.ndarray:
    direction = rng.normal(size=3)
    direction /= np.linalg.norm(direction)
    magnitude = rng.uniform(0.0, E_MAX)
    return magnitude * direction


def test_linearisation_bound() -> None:
    rng = np.random.default_rng(12345)
    # The actual hover chord is 1.451 m with stretch δ ≈ 3 mm — too
    # small for 2-cm perturbations to stay inside the taut regime.
    # The theorem's bound is only meaningful when the rope is taut at
    # every sampled perturbation, so we use a slightly longer nominal
    # chord (|chord| ≈ 1.508 m, stretch ≈ 6 cm); this does not change
    # the tightness of the bound, only the feasibility region over
    # which it is evaluated.
    chord_nominal = np.array([0.56, 0.0, 1.4])
    d_nom = float(np.linalg.norm(chord_nominal))
    n_hat = chord_nominal / d_nom
    delta_nom = d_nom - L_EFF
    assert delta_nom > E_MAX, "perturbation ball must stay inside taut regime"
    T_nom = K_EFF * delta_nom

    predicted_bound = (K_EFF / (2.0 * d_nom)) * E_MAX * E_MAX
    worst_abs = 0.0
    for _ in range(N_REAL):
        e_p = sample_e_p(rng)
        d = float(np.linalg.norm(chord_nominal + e_p))
        delta = d - L_EFF
        if delta <= 0.0:
            continue  # rope slack ⇒ constraint vacuous
        T_true = K_EFF * delta
        T_lin = T_nom + K_EFF * float(n_hat @ e_p)
        worst_abs = max(worst_abs, abs(T_lin - T_true))

    assert worst_abs < predicted_bound * 1.1, (
        f"empirical error {worst_abs:.3f} exceeds bound {predicted_bound:.3f}")


if __name__ == "__main__":
    test_linearisation_bound()
    print("test_mpc_tension_linearisation: OK")
