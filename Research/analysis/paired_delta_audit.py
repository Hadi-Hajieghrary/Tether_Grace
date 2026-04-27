#!/usr/bin/env python3
"""A5: Paired-delta audit for the feed-forward ablation.

Compute paired differences Δ = (FF-off) − (FF-on) across the
three V3/V4/V5 fault schedules (n=3 structural pairs, single
deterministic seed 42). Reports median, range, and sign-test
p-value. Honestly states n=3 and that bootstrap CIs from
n=32 seed-paired runs remain RFC-deferred.

Output:
  IEEE_T-CST/Sections/auto/tab_paired_delta.tex (snippet)
  output/paired_delta/summary.csv
"""
from __future__ import annotations
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from scipy import stats

sys.path.insert(0, str(Path(__file__).resolve().parent))
from plot_capability_demo import num_drones, tracking_error  # noqa

CAP = Path("/workspaces/Tether_Grace/output/capability_demo")
P2A = Path("/workspaces/Tether_Grace/output/p2a_tension_ff_ablation")
OUT = Path("/workspaces/Tether_Grace/output/paired_delta")
OUT.mkdir(parents=True, exist_ok=True)

PAIRS = [("V3_single_wind", "V3_single_wind_nff", "V3", [12.0]),
         ("V4_dual_5s_wind", "V4_dual_5s_wind_nff", "V4", [12.0, 17.0]),
         ("V5_dual_10s_wind", "V5_dual_10s_wind_nff", "V5", [12.0, 22.0])]


def metrics(df, faults):
    t = df["time"].values
    cruise = (t >= 8.0) & (t <= 30.0)
    e3d = tracking_error(df)
    rmse_m = float(np.sqrt(np.mean(e3d[cruise] ** 2)))
    sag = (df["ref_z"].values - df["payload_z"].values)
    peak_sag_mm_list, t_rec_list, iae_list = [], [], []
    for k, tf in enumerate(faults):
        end = faults[k + 1] if k + 1 < len(faults) else tf + 8.0
        post = (t >= tf) & (t <= end)
        if post.any():
            peak_sag_mm_list.append(float(np.max(np.abs(sag[post]))) * 1000)
            tau = min(2.24, end - tf)
            iae_w = (t >= tf) & (t <= tf + tau)
            iae_list.append(float(np.trapezoid(e3d[iae_w], t[iae_w])))
            t_post = t[t >= tf]
            e_post = e3d[t >= tf]
            rec = np.nan
            n_hold = max(1, int(0.3 / max(np.median(np.diff(t_post)),
                                          1e-4)))
            for i in range(len(e_post) - n_hold):
                if np.all(e_post[i:i + n_hold] < 0.35):
                    rec = float(t_post[i] - tf)
                    break
            t_rec_list.append(rec)
    return {"rmse_m": rmse_m,
            "peak_sag_mm": float(np.max(peak_sag_mm_list)),
            "iae_ms": float(np.mean(iae_list)),
            "t_rec_s": float(np.nanmean(t_rec_list))}


def main():
    rows = []
    for ff_tag, nff_tag, label, faults in PAIRS:
        ff = pd.read_csv(CAP / ff_tag / f"scenario_{ff_tag}.csv")
        nff = pd.read_csv(P2A / nff_tag / f"scenario_{nff_tag}.csv")
        m_on = metrics(ff, faults)
        m_off = metrics(nff, faults)
        rows.append({"variant": label,
                     "rmse_on": m_on["rmse_m"],
                     "rmse_off": m_off["rmse_m"],
                     "drmse_pct": 100 * (m_off["rmse_m"]
                                          - m_on["rmse_m"])
                                  / m_on["rmse_m"],
                     "sag_on": m_on["peak_sag_mm"],
                     "sag_off": m_off["peak_sag_mm"],
                     "sag_ratio": m_off["peak_sag_mm"]
                                   / m_on["peak_sag_mm"],
                     "iae_on": m_on["iae_ms"],
                     "iae_off": m_off["iae_ms"],
                     "diae_ratio": m_off["iae_ms"] / m_on["iae_ms"],
                     "trec_on": m_on["t_rec_s"],
                     "trec_off": m_off["t_rec_s"]})
    df = pd.DataFrame(rows)
    df.to_csv(OUT / "summary.csv", index=False)

    print("Per-variant FF-on vs FF-off (seed-42 deterministic):")
    print(df.round(3).to_string(index=False))

    drmse = df["drmse_pct"].values
    dsag = df["sag_ratio"].values
    diae = df["diae_ratio"].values
    n = len(df)
    print(f"\nPaired summary across n={n} structural pairs "
          f"(V3, V4, V5 at seed 42):")
    print(f"  Δ RMSE:    median={np.median(drmse):.1f}%   "
          f"range=[{drmse.min():.1f}%, {drmse.max():.1f}%]")
    print(f"  Sag ratio: median={np.median(dsag):.2f}x  "
          f"range=[{dsag.min():.2f}, {dsag.max():.2f}]")
    print(f"  IAE ratio: median={np.median(diae):.2f}x  "
          f"range=[{diae.min():.2f}, {diae.max():.2f}]")
    # Sign test: probability under H0 (no effect) of all 3 having
    # the same sign.
    pos = int((drmse > 0).sum())
    p_sign = 2 * stats.binomtest(pos, n=n, p=0.5).pvalue \
        if pos in (0, n) else 1.0
    print(f"  Sign test (RMSE > 0 in all variants): "
          f"p={p_sign:.3f}  (n={n})")

    # LaTeX snippet
    out_tex = (
        "% Auto-generated from paired_delta_audit.py\n"
        "\\begin{table}[htbp]\n"
        "  \\caption{%\n"
        "    Paired feed-forward effect ($\\Delta = $ FF-off "
        "minus FF-on) across the V3, V4, V5 fault schedules at\n"
        "    fixed seed~42. \\emph{Pairing is structural (per "
        "fault schedule), not seed-paired}; bootstrap CIs from\n"
        "    $n=32$ seed-paired runs are RFC-deferred. Sign test "
        "across $n=3$ pairs reports whether the effect is\n"
        "    consistent in direction.%\n"
        "  }\n"
        "  \\label{tab:paired-delta}\n"
        "  \\centering\n"
        "  \\small\n"
        "  \\begin{tabular}{lccc}\n"
        "    \\toprule\n"
        "    Metric & median $\\Delta$ & range & sign test \\\\\n"
        "    \\midrule\n"
        f"    cruise RMSE (\\%) & "
        f"$+{np.median(drmse):.1f}$ & $[{drmse.min():.1f}, "
        f"{drmse.max():.1f}]$ & "
        f"$p={p_sign:.3f}$ ($n=3$) \\\\\n"
        f"    peak sag ratio (\\(\\times\\)) & "
        f"${np.median(dsag):.2f}$ & "
        f"$[{dsag.min():.2f}, {dsag.max():.2f}]$ & "
        f"$p={p_sign:.3f}$ \\\\\n"
        f"    IAE ratio (\\(\\times\\)) & "
        f"${np.median(diae):.2f}$ & "
        f"$[{diae.min():.2f}, {diae.max():.2f}]$ & "
        f"$p={p_sign:.3f}$ \\\\\n"
        "    \\bottomrule\n"
        "  \\end{tabular}\n"
        "\\end{table}\n"
    )
    out_path = (Path("/workspaces/Tether_Grace/IEEE_T-CST/Sections/")
                / "auto")
    out_path.mkdir(exist_ok=True)
    (out_path / "tab_paired_delta.tex").write_text(out_tex)
    print(f"\n  wrote {out_path/'tab_paired_delta.tex'}")


if __name__ == "__main__":
    main()
