#!/usr/bin/env python3
"""WP6.2: Empirical fit of the ISS gain gamma_eta.

The hybrid practical-ISS theorem bounds the closed-loop tracking
error by a sum of class-K gains times the corresponding inputs:
    |xi(t)| <= beta(...) + gamma_w |w| + gamma_theta |theta-theta*|
              + gamma_eta * eta + ...
where eta is the slack-excursion duty cycle. We estimate
gamma_eta by regressing observed peak post-fault sag against
slack duty cycle across the V1--V6 capability missions.

Output: prints fitted gamma_eta and reports goodness of fit.
"""
from __future__ import annotations
import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
from plot_capability_demo import num_drones  # noqa: E402

CAP = Path("/workspaces/Tether_Grace/output/capability_demo")
SLACK_THR = 0.2

# (variant_tag, slack_duty_cycle_pct from tab:domain-audit)
VARIANTS = [
    ("V1_nominal_nowind", 1.34),
    ("V2_nominal_wind", 1.52),
    ("V3_single_wind", 1.52),
    ("V4_dual_5s_wind", 1.52),
    ("V5_dual_10s_wind", 1.52),
    ("V6_dual_5s_fullstack", 1.66),
]


def _peak_sag_post_fault(df):
    """Maximum |sag| over t > 11 s (after pre-fault settle)."""
    t = df["time"].values
    m = t > 11.0
    sag = df["ref_z"].values - df["payload_z"].values
    return float(np.max(np.abs(sag[m]))) if m.any() else np.nan


def main():
    rows = []
    for tag, eta_pct in VARIANTS:
        csv = CAP / tag / f"scenario_{tag}.csv"
        if not csv.exists():
            print(f"  {tag}: missing"); continue
        df = pd.read_csv(csv)
        sag_mm = _peak_sag_post_fault(df) * 1000
        rows.append((tag, eta_pct, sag_mm))
        print(f"  {tag}: eta={eta_pct:.2f}%  peak_sag={sag_mm:.1f}mm")

    if len(rows) < 3:
        print("insufficient data points")
        return

    eta = np.array([r[1] for r in rows])
    sag = np.array([r[2] for r in rows])
    # Linear fit: sag = beta + gamma_eta * eta
    A = np.vstack([np.ones_like(eta), eta]).T
    coef, *_ = np.linalg.lstsq(A, sag, rcond=None)
    beta, gamma_eta = coef
    pred = A @ coef
    ss_res = np.sum((sag - pred) ** 2)
    ss_tot = np.sum((sag - np.mean(sag)) ** 2)
    r2 = 1 - ss_res / max(ss_tot, 1e-9)

    print(f"\nLinear fit  sag(mm) = beta + gamma_eta * eta(%)")
    print(f"  beta       = {beta:.2f} mm")
    print(f"  gamma_eta  = {gamma_eta:.2f} mm/%  "
          f"(= {gamma_eta * 0.01 * 1e-3:.4f} m per unit eta)")
    print(f"  R^2        = {r2:.3f}")


if __name__ == "__main__":
    main()
