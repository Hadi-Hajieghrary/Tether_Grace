#!/usr/bin/env python3
"""P2-A analysis: tension feed-forward ablation.

Compares the archived capability-demo V3/V4/V5 runs (T_ff enabled)
against the P2-A companion runs under --disable-tension-ff.  Produces:

    - Side-by-side tracking error per scenario.
    - Side-by-side peak per-rope tension per scenario.
    - Side-by-side load-share sigma_T per scenario.
    - Per-fault recovery-metrics comparison table.
    - Headline summary CSV.

Output: output/p2a_tension_ff_ablation/_summary/
"""
from __future__ import annotations

import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ieee_style import (  # noqa: E402
    COLORS, FAULT_STYLE, SINGLE_COL, DOUBLE_COL,
    annotate_faults, setup_style,
)
from plot_capability_demo import (  # noqa: E402
    num_drones, peak_tension, sigma_T, tracking_error,
)
from plot_review_augmentation import (  # noqa: E402
    first_fault_per_rope,
)

setup_style()

# (label, FF-on archived tag, FF-off P2-A tag).
PAIRS = [
    ("V3 single fault",
     "V3_single_wind",
     "V3_single_wind_nff"),
    ("V4 dual 5 s",
     "V4_dual_5s_wind",
     "V4_dual_5s_wind_nff"),
    ("V5 dual 10 s",
     "V5_dual_10s_wind",
     "V5_dual_10s_wind_nff"),
]

FF_ROOT = Path("/workspaces/Tether_Grace/output/capability_demo")
NFF_ROOT = Path("/workspaces/Tether_Grace/output/p2a_tension_ff_ablation")


def _load(root: Path, tag: str) -> pd.DataFrame | None:
    csv = root / tag / f"scenario_{tag}.csv"
    if not csv.exists():
        print(f"  MISSING: {csv}")
        return None
    return pd.read_csv(csv)


def _plot_side_by_side(ff_df, nff_df, metric_fn, ylabel, tag, label, out):
    fig, ax = plt.subplots(figsize=DOUBLE_COL)
    for df, col, lab in [(ff_df, COLORS[0], "$T_i^{ff}=T_i$ (baseline)"),
                         (nff_df, COLORS[1], "$T_i^{ff}=0$ (ablation)")]:
        if df is None:
            continue
        N = num_drones(df)
        t = df["time"].values
        m = metric_fn(df, N) if metric_fn.__code__.co_argcount == 2 \
            else metric_fn(df)
        ax.plot(t, m, color=col, lw=1.0, label=lab)
    if ff_df is not None:
        annotate_faults(ax, first_fault_per_rope(ff_df, num_drones(ff_df)))
    ax.set_xlabel("t [s]")
    ax.set_ylabel(ylabel)
    ax.set_title(f"{label}: FF on vs off", fontsize=9)
    ax.legend(loc="upper right", fontsize=7)
    ax.grid(True, linewidth=0.3, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out, dpi=180)
    plt.close(fig)


def main() -> None:
    out = NFF_ROOT / "_summary"
    out.mkdir(parents=True, exist_ok=True)

    headline_rows = []

    for label, ff_tag, nff_tag in PAIRS:
        print(f"\n--- {label}: {ff_tag} vs {nff_tag}")
        ff = _load(FF_ROOT, ff_tag)
        nff = _load(NFF_ROOT, nff_tag)

        if ff is None or nff is None:
            continue

        # Tracking error
        _plot_side_by_side(
            ff, nff,
            tracking_error,
            r"$\|e_p\|$ [m]",
            ff_tag,
            label,
            out / f"p2a_trackerr_{ff_tag}.png")

        # Peak tension
        _plot_side_by_side(
            ff, nff,
            peak_tension,
            r"peak $T_i$ [N]",
            ff_tag,
            label,
            out / f"p2a_peakT_{ff_tag}.png")

        # sigma_T
        _plot_side_by_side(
            ff, nff,
            sigma_T,
            r"$\sigma_T$ [N]",
            ff_tag,
            label,
            out / f"p2a_sigmaT_{ff_tag}.png")

        # Payload altitude sag
        fig, ax = plt.subplots(figsize=DOUBLE_COL)
        for df, col, lab in [(ff, COLORS[0], "FF on"),
                             (nff, COLORS[1], "FF off")]:
            t = df["time"].values
            sag = df["payload_z"].values - df["ref_z"].values
            ax.plot(t, sag, color=col, lw=1.0, label=lab)
        annotate_faults(ax, first_fault_per_rope(ff, num_drones(ff)))
        ax.axhline(0, color="black", linestyle=":", lw=0.5)
        ax.set_xlabel("t [s]")
        ax.set_ylabel(r"$z_L - z_\text{ref}$ [m]")
        ax.set_title(f"{label}: payload altitude tracking", fontsize=9)
        ax.legend(loc="upper right", fontsize=7)
        ax.grid(True, linewidth=0.3, alpha=0.3)
        fig.tight_layout()
        fig.savefig(out / f"p2a_altsag_{ff_tag}.png", dpi=180)
        plt.close(fig)

        # Summary metrics
        for df, mode in [(ff, "FF_on"), (nff, "FF_off")]:
            # Exclude the first 0.5 s transient.
            mask = df["time"].values >= 0.5
            d = df[mask]
            N = num_drones(d)
            e = tracking_error(d)
            tp = peak_tension(d, N)
            sT = sigma_T(d, N)
            headline_rows.append({
                "scenario": ff_tag,
                "mode": mode,
                "rms_err_m":     round(float(np.sqrt(np.mean(e * e))), 4),
                "peak_err_m":    round(float(np.max(e)), 4),
                "peak_T_N":      round(float(np.max(tp)), 2),
                "mean_T_N":      round(float(np.mean(tp)), 2),
                "rms_sigma_T_N": round(float(np.sqrt(np.mean(sT * sT))), 3),
                "max_altsag_m":  round(float(
                    np.min(d["payload_z"].values - d["ref_z"].values)), 4),
            })

    df_out = pd.DataFrame(headline_rows)
    df_out.to_csv(out / "p2a_summary.csv", index=False)
    print("\n=== P2-A ablation summary ===")
    print(df_out.to_string(index=False))
    print(f"\nAll P2-A outputs under: {out}")


if __name__ == "__main__":
    main()
