#!/usr/bin/env python3
"""WP3.5 + WP3.6: Direct empirical fits for chi(Delta_f) jump
magnitude (Lemma 5) and dwell-cycle contraction rho_hat across
the canonical V3, V4, V5 traces.

Method.
The Lyapunov-energy proxy is the squared-norm of the altitude
slot-tracking error around its local equilibrium:
    V(t) = || zeta(t) - zeta_eq(t) ||_{P_v}^2 / 2
where zeta = (e_z, e_{vz}) and zeta_eq is a 0.2-s local mean.
The jump magnitude is
    chi_hat = V(t_f^+) - V(t_f^-),
sampled with 50 ms windows around each fault. The dwell-cycle
contraction is
    rho_hat = V(t_{k+1}^-) / V(t_k^-).
Each is reported per-fault and aggregated.

Outputs:
    output/jump_magnitude/summary.csv
"""
from __future__ import annotations
import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
from plot_capability_demo import num_drones  # noqa: E402

CAP = Path("/workspaces/Tether_Grace/output/capability_demo")
OUT = Path("/workspaces/Tether_Grace/output/jump_magnitude")
OUT.mkdir(parents=True, exist_ok=True)

P_V = np.array([[2.224, 0.005], [0.005, 0.021]])
SLACK_THR = 0.2

VARIANTS = [("V3_single_wind", [(12.0, 0)]),
            ("V4_dual_5s_wind", [(12.0, 0), (17.0, 2)]),
            ("V5_dual_10s_wind", [(12.0, 0), (22.0, 2)])]


def _load(tag):
    csv = CAP / tag / f"scenario_{tag}.csv"
    return pd.read_csv(csv) if csv.exists() else None


def _V_proxy(df: pd.DataFrame, t_lo: float, t_hi: float) -> np.ndarray:
    t = df["time"].values
    m = (t >= t_lo) & (t <= t_hi)
    if not m.any():
        return np.array([])
    ep_z = df["ref_z"].values[m] - df["payload_z"].values[m]
    ev_z = df["ref_vz"].values[m] - df["payload_vz"].values[m]
    # Detrend by 0.2 s pre-window mean to isolate transient
    bm = (t >= max(0.0, t_lo - 0.2)) & (t < t_lo)
    if bm.any():
        ep_z -= float(np.mean(df["ref_z"].values[bm]
                              - df["payload_z"].values[bm]))
        ev_z -= float(np.mean(df["ref_vz"].values[bm]
                              - df["payload_vz"].values[bm]))
    zeta = np.stack([ep_z, ev_z], axis=1)
    return 0.5 * np.einsum("ij,jk,ik->i", zeta, P_V, zeta)


def main():
    rows = []
    for tag, faults in VARIANTS:
        df = _load(tag)
        if df is None:
            print(f"  {tag}: missing"); continue
        # Lyapunov-proxy peaks before/after each fault (50 ms window)
        peaks_pre, peaks_post = [], []
        for tf, _q in faults:
            V_pre = _V_proxy(df, tf - 0.05, tf - 1e-3)
            V_post = _V_proxy(df, tf + 1e-3, tf + 0.05)
            if V_pre.size == 0 or V_post.size == 0:
                continue
            peaks_pre.append(float(np.max(V_pre)))
            peaks_post.append(float(np.max(V_post)))
        chi_hat = [post - pre for post, pre in zip(peaks_post, peaks_pre)]
        # Dwell-cycle contraction (only meaningful for >= 2 faults)
        rho_hat = []
        if len(faults) >= 2:
            for k in range(len(faults) - 1):
                rho_hat.append(peaks_pre[k + 1] / max(peaks_pre[k],
                                                      1e-9))
        rows.append({
            "variant": tag,
            "n_faults": len(faults),
            "V_pre_max_mean": float(np.mean(peaks_pre)) if peaks_pre else np.nan,
            "V_post_max_mean": float(np.mean(peaks_post)) if peaks_post else np.nan,
            "chi_mean": float(np.mean(chi_hat)) if chi_hat else np.nan,
            "chi_max": float(np.max(chi_hat)) if chi_hat else np.nan,
            "rho_mean": float(np.mean(rho_hat)) if rho_hat else np.nan,
        })
        print(f"  {tag}: chi_mean={float(np.mean(chi_hat)):.2e}  "
              f"chi_max={float(np.max(chi_hat)):.2e}  "
              + (f"rho={float(np.mean(rho_hat)):.3f}"
                 if rho_hat else "rho=N/A"))
    df = pd.DataFrame(rows)
    df.to_csv(OUT / "summary.csv", index=False)
    print(f"\nwrote {OUT/'summary.csv'}")
    print(df.round(4).to_string(index=False))


if __name__ == "__main__":
    main()
