#!/usr/bin/env python3
"""Compute H∞ norms for small-gain verification.

Reads A, B, C, D matrices from CSV (output by linearize_plant) and computes:
  - γ_ISS: H∞ norm of position-loop transfer function
  - Frequency response plot
  - Small-gain condition check

Can also compute the scalar model norms directly without linearize_plant.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from numpy.linalg import eigvals, inv, svd


def hinf_norm(A: np.ndarray, B: np.ndarray, C: np.ndarray, D: np.ndarray,
              omega_range: tuple[float, float] = (1e-3, 1e3),
              n_points: int = 10000) -> tuple[float, float]:
    """Compute H∞ norm via frequency sweep.

    Returns (gamma, omega_peak).
    """
    omegas = np.logspace(np.log10(omega_range[0]),
                         np.log10(omega_range[1]), n_points)
    n = A.shape[0]
    I = np.eye(n)
    gamma_max = 0.0
    omega_peak = 0.0

    for w in omegas:
        jw = 1j * w
        G = C @ inv(jw * I - A) @ B + D
        # For SISO: magnitude; for MIMO: max singular value
        if G.shape[0] == 1 and G.shape[1] == 1:
            mag = abs(G[0, 0])
        else:
            mag = svd(G, compute_uv=False)[0]  # largest singular value
        if mag > gamma_max:
            gamma_max = mag
            omega_peak = w

    return float(gamma_max), float(omega_peak)


def finite_time_gain(A: np.ndarray, B: np.ndarray, C: np.ndarray,
                     tau: float, n_steps: int = 1000) -> float:
    """Compute finite-time gain ||C ∫₀^τ e^{Aτ}B dτ||."""
    n = A.shape[0]
    dt = tau / n_steps
    integral = np.zeros((n, B.shape[1]))
    eA = np.eye(n)
    for _ in range(n_steps):
        integral += eA @ B * dt
        eA = (np.eye(n) + A * dt) @ eA
    return float(np.linalg.norm(C @ integral))


def scalar_model_analysis(kp: float = 8.0, kd: float = 8.0,
                          ki: float = 0.15) -> dict:
    """Analyze the scalar per-axis model from Proposition 1."""
    A = np.array([[0, 1, 0],
                  [-kp, -kd, -ki],
                  [1, 0, 0]])
    B = np.array([[0], [1], [0]])
    C = np.array([[1, 0, 0]])
    D = np.array([[0.0]])

    eigs = eigvals(A)
    gamma_iss, omega_peak = hinf_norm(A, B, C, D)
    dc_gain = float(abs(C @ inv(A) @ B))
    gamma_ft = finite_time_gain(A, B, C, tau=0.5)

    return {
        "kp": kp, "kd": kd, "ki": ki,
        "eigenvalues": [
            {"real": float(e.real), "imag": float(e.imag),
             "magnitude": float(abs(e))}
            for e in eigs
        ],
        "all_hurwitz": all(e.real < 0 for e in eigs),
        "gamma_ISS": gamma_iss,
        "omega_peak_rad_s": omega_peak,
        "dc_gain": dc_gain,
        "gamma_FT_0_5s": gamma_ft,
        "predicted_peak_cm": gamma_ft * 4.4 * 100,
        "max_gamma_cable_for_stability": 1.0 / gamma_iss,
    }


def load_matrices(matrix_dir: Path) -> tuple:
    """Load A, B, C, D from CSV files."""
    A = np.loadtxt(matrix_dir / "A.csv", delimiter=",")
    B = np.loadtxt(matrix_dir / "B.csv", delimiter=",")
    C = np.loadtxt(matrix_dir / "C.csv", delimiter=",")
    D = np.loadtxt(matrix_dir / "D.csv", delimiter=",")
    return A, B, C, D


def main():
    parser = argparse.ArgumentParser(description="H∞ small-gain verification")
    parser.add_argument("--matrix-dir", type=str, default=None,
                        help="Directory with A.csv, B.csv, C.csv, D.csv")
    parser.add_argument("--scalar-only", action="store_true",
                        help="Only analyze scalar per-axis model")
    parser.add_argument("--output", type=str, default=None,
                        help="Output JSON file")
    args = parser.parse_args()

    results = {}

    # Scalar model analysis (always available)
    print("=== Scalar Per-Axis Model (Proposition 1) ===\n")
    scalar = scalar_model_analysis()
    results["scalar_model"] = scalar

    print(f"Eigenvalues of A:")
    for e in scalar["eigenvalues"]:
        print(f"  λ = {e['real']:.4f} + {e['imag']:.4f}j  "
              f"(|λ| = {e['magnitude']:.4f})")
    print(f"\nAll Hurwitz: {scalar['all_hurwitz']}")
    print(f"\nH∞ norm (γ_ISS): {scalar['gamma_ISS']:.6f}")
    print(f"Peak frequency: {scalar['omega_peak_rad_s']:.2f} rad/s")
    print(f"DC gain: {scalar['dc_gain']:.6f}")
    print(f"\nFinite-time gain (τ=0.5s): {scalar['gamma_FT_0_5s']:.6f}")
    print(f"Predicted peak error: {scalar['predicted_peak_cm']:.1f} cm")
    print(f"\nFor small-gain condition γ_ISS · γ_cable < 1:")
    print(f"  Required: γ_cable < {scalar['max_gamma_cable_for_stability']:.2f}")

    # Full-plant analysis (if matrices available)
    if args.matrix_dir and not args.scalar_only:
        matrix_dir = Path(args.matrix_dir)
        if (matrix_dir / "A.csv").exists():
            print("\n=== Full-Plant Linearized Model ===\n")
            A, B, C, D = load_matrices(matrix_dir)
            print(f"State dimension: {A.shape[0]}")
            print(f"Input dimension: {B.shape[1]}")
            print(f"Output dimension: {C.shape[0]}")

            eigs = eigvals(A)
            n_unstable = sum(1 for e in eigs if e.real > 0)
            print(f"\nEigenvalues: {len(eigs)} total, "
                  f"{n_unstable} with Re(λ) > 0")
            if n_unstable > 0:
                print("WARNING: Unstable eigenvalues detected!")
                for e in eigs:
                    if e.real > 0:
                        print(f"  λ = {e.real:.6f} + {e.imag:.6f}j")

            gamma, omega = hinf_norm(A, B, C, D)
            results["full_plant"] = {
                "state_dim": A.shape[0],
                "input_dim": B.shape[1],
                "output_dim": C.shape[0],
                "n_unstable_eigenvalues": n_unstable,
                "gamma_ISS": gamma,
                "omega_peak_rad_s": omega,
            }
            print(f"\nH∞ norm (full plant): {gamma:.6f}")
            print(f"Peak frequency: {omega:.2f} rad/s")
        else:
            print(f"\nNo matrices found in {matrix_dir}")

    # Save results
    if args.output:
        Path(args.output).write_text(json.dumps(results, indent=2))
        print(f"\nResults saved to {args.output}")


if __name__ == "__main__":
    main()
