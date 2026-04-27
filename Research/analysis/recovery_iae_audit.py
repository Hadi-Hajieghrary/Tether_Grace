#!/usr/bin/env python3
"""A3: Recovery-time and IAE audit for V3, V4, V5, V6.

For each fault event, compute:
  - peak position-error norm (during the 3-second post-fault window)
  - peak altitude sag
  - recovery time t_rec(epsilon=0.05 m) — first time after fault
    at which |e_p(t)| stays under epsilon for 0.3 s continuously
  - IAE_post — integral of |e_p(t)| over one pendulum period
    after the fault event

Outputs:
  output/recovery_iae/summary.csv
"""
from __future__ import annotations
import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
from plot_capability_demo import num_drones, tracking_error  # noqa

CAP = Path("/workspaces/Tether_Grace/output/capability_demo")
OUT = Path("/workspaces/Tether_Grace/output/recovery_iae")
OUT.mkdir(parents=True, exist_ok=True)

TAU_PEND = 2.24
EPS_REC_3D = 0.35      # m, declared in to do.txt
EPS_REC_ALT = 0.020    # m, tighter altitude-only recovery
HOLD = 0.30            # s

VARIANT_FAULTS = {
    "V3_single_wind": [12.0],
    "V4_dual_5s_wind": [12.0, 17.0],
    "V5_dual_10s_wind": [12.0, 22.0],
    "V6_dual_5s_fullstack": [12.0, 17.0],
}


def _load(tag: str) -> pd.DataFrame | None:
    csv = CAP / tag / f"scenario_{tag}.csv"
    return pd.read_csv(csv) if csv.exists() else None


def _recovery_time(t_arr, e_arr, eps, hold):
    if len(t_arr) < 2:
        return float("nan")
    dt = max(np.median(np.diff(t_arr)), 1e-4)
    n_hold = max(1, int(hold / dt))
    for i in range(len(e_arr) - n_hold):
        if np.all(e_arr[i:i + n_hold] < eps):
            return float(t_arr[i] - t_arr[0])
    return float("nan")


def _recovery_metrics(df: pd.DataFrame, t_fault: float,
                      next_fault: float | None) -> dict:
    t = df["time"].values
    e3d = tracking_error(df)  # |e_p(t)| in metres (3D)
    e_alt = np.abs(df["ref_z"].values - df["payload_z"].values)
    # post-fault window: until next fault, or t_fault + 8 s, or end.
    t_end = next_fault if next_fault is not None else t_fault + 8.0
    post = (t >= t_fault) & (t <= t_end)
    peak_e = float(np.max(e3d[post]))
    peak_sag_mm = float(np.max(e_alt[post])) * 1000

    # Recovery times
    t_post = t[post]
    e3d_post = e3d[post]
    e_alt_post = e_alt[post]
    rec_3d = _recovery_time(t_post, e3d_post, EPS_REC_3D, HOLD)
    rec_alt = _recovery_time(t_post, e_alt_post, EPS_REC_ALT, HOLD)

    # IAE over one pendulum period (use min of pendulum or
    # pre-next-fault gap to avoid contamination).
    iae_horizon = min(TAU_PEND, (t_end - t_fault))
    iae_window = (t >= t_fault) & (t <= t_fault + iae_horizon)
    iae = float(np.trapezoid(e3d[iae_window], t[iae_window])) \
        if iae_window.any() else float("nan")
    return {"peak_e_m": peak_e, "peak_sag_mm": peak_sag_mm,
            "t_rec_3d_s": rec_3d, "t_rec_alt_s": rec_alt,
            "iae_post_ms": iae}


def main():
    rows = []
    for tag, faults in VARIANT_FAULTS.items():
        df = _load(tag)
        if df is None:
            continue
        for k, tf in enumerate(faults):
            next_tf = faults[k + 1] if k + 1 < len(faults) else None
            m = _recovery_metrics(df, tf, next_tf)
            rows.append({"variant": tag.split("_")[0],
                         "fault_idx": k + 1,
                         "t_fault_s": tf,
                         **m})
            print(f"  {tag} f{k+1}@{tf}s: "
                  f"peak_e={m['peak_e_m']*1000:.1f}mm  "
                  f"peak_sag={m['peak_sag_mm']:.1f}mm  "
                  f"t_rec_3D={m['t_rec_3d_s']:.2f}s  "
                  f"t_rec_alt={m['t_rec_alt_s']:.2f}s  "
                  f"IAE={m['iae_post_ms']:.3f}m·s")
    df = pd.DataFrame(rows)
    df.to_csv(OUT / "summary.csv", index=False)
    print(f"\nwrote {OUT/'summary.csv'}")


if __name__ == "__main__":
    main()
