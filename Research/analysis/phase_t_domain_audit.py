#!/usr/bin/env python3
"""Phase T Domain Audit.

Computes the largest feasible T_min such that
    min_{i in S(t)} T_i(t) >= T_min > 0
holds on every post-pickup tick of the canonical demonstrated
missions (V1-V6).  Also measures the QP-active-set transition rate
to verify the C0 doctrine D1 (single-active-regime) is tenable.

Gate outcome:
    PASS  -> T_min > 0 found; contract's Omega_tau is non-empty on
             demonstrated missions.
    FAIL  -> some survivor cable goes slack post-pickup; kill the
             Reduction theorem and fall back to empirical claim.

Output: output/phase_t/domain_audit_report.csv
        output/phase_t/domain_audit_figs/*.png
        output/phase_t/domain_audit.log
"""
from __future__ import annotations

import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ieee_style import COLORS, FAULT_STYLE, DOUBLE_COL, setup_style
from plot_capability_demo import num_drones
from plot_review_augmentation import first_fault_per_rope

setup_style()

CAMPAIGN = Path("/workspaces/Tether_Grace/output/capability_demo")
OUT = Path("/workspaces/Tether_Grace/output/phase_t")
OUT.mkdir(parents=True, exist_ok=True)
(OUT / "domain_audit_figs").mkdir(parents=True, exist_ok=True)

T_PICKUP = 1.0  # per contract: end of smoothstep ramp

VARIANTS = [
    "V1_nominal_nowind", "V2_nominal_wind", "V3_single_wind",
    "V4_dual_5s_wind", "V5_dual_10s_wind", "V6_dual_5s_fullstack",
]


def survivor_min_tension(df: pd.DataFrame, t_min_cutoff: float) -> np.ndarray:
    """min_i T_i(t) over surviving cables; cables are 'surviving' if
    they have not yet been severed."""
    N = num_drones(df)
    t = df["time"].values
    faults = dict(first_fault_per_rope(df, N))  # rope_idx -> fault_time
    T = np.stack([df[f"tension_{i}"].values for i in range(N)], axis=1)
    # Mask severed cables with +inf so they do not influence min.
    mask = np.ones_like(T, dtype=bool)
    for rope_idx, t_f in faults.items():
        mask[t >= t_f, rope_idx] = False
    T_masked = np.where(mask, T, np.inf)
    min_over_survivors = T_masked.min(axis=1)
    return t, min_over_survivors


def qp_active_set_transitions(df: pd.DataFrame) -> dict:
    """Count ticks where any QP active-set bit changes vs previous tick."""
    N = num_drones(df)
    bits = []
    for i in range(N):
        for b in ("ax_lo", "ax_hi", "ay_lo", "ay_hi", "az_lo", "az_hi"):
            col = f"act_{b}_{i}"
            if col in df.columns:
                bits.append(df[col].values)
    if not bits:
        return {"any_active_frac": 0.0, "transition_rate": 0.0,
                "any_transition_frac": 0.0}
    bit_stack = np.stack(bits, axis=1)
    # Active any tick.
    any_active = bit_stack.any(axis=1)
    # Transition: diff from previous tick is any bit flip.
    transitions = np.abs(np.diff(bit_stack.astype(int), axis=0)).any(axis=1)
    return {
        "any_active_frac":     float(np.mean(any_active)),
        "any_transition_frac": float(np.mean(transitions)),
    }


def main():
    log_path = OUT / "domain_audit.log"
    rows = []
    with open(log_path, "w") as log:
        def tee(msg):
            print(msg)
            log.write(msg + "\n")
        tee("=" * 72)
        tee("Phase T — Domain Audit Report")
        tee("=" * 72)
        tee(f"Generated: {pd.Timestamp.utcnow().isoformat()}")
        tee(f"Post-pickup cutoff: t >= {T_PICKUP} s")
        tee("")

        for tag in VARIANTS:
            csv = CAMPAIGN / tag / f"scenario_{tag}.csv"
            if not csv.exists():
                tee(f"{tag}: MISSING; skipping")
                continue
            df = pd.read_csv(csv)
            post = df["time"].values >= T_PICKUP
            t_all, Tmin_all = survivor_min_tension(df, T_PICKUP)
            Tmin_post = Tmin_all[post]
            # Replace +inf (all-severed) with nan so statistics are
            # taken over survivor-populated ticks only.
            Tmin_post_finite = Tmin_post[np.isfinite(Tmin_post)]

            qp = qp_active_set_transitions(df[post])

            if len(Tmin_post_finite) > 0:
                Tmin_floor = float(np.min(Tmin_post_finite))
                Tmin_p1    = float(np.percentile(Tmin_post_finite, 1))
                Tmin_mean  = float(np.mean(Tmin_post_finite))
            else:
                Tmin_floor = Tmin_p1 = Tmin_mean = float("nan")

            frac_below_05 = float(np.mean(Tmin_post_finite < 0.5)) \
                if len(Tmin_post_finite) else float("nan")
            frac_below_10 = float(np.mean(Tmin_post_finite < 1.0)) \
                if len(Tmin_post_finite) else float("nan")

            row = {
                "variant":                tag,
                "ticks_post":             int(np.sum(post)),
                "survivor_populated":     int(len(Tmin_post_finite)),
                "Tmin_floor_N":           round(Tmin_floor, 4),
                "Tmin_p1_N":              round(Tmin_p1, 4),
                "Tmin_mean_N":            round(Tmin_mean, 4),
                "frac_below_0p5N":        round(frac_below_05, 4),
                "frac_below_1p0N":        round(frac_below_10, 4),
                "qp_any_active_frac":     round(qp["any_active_frac"], 4),
                "qp_transition_frac":     round(qp["any_transition_frac"], 4),
            }
            rows.append(row)
            tee(f"{tag:<24} "
                f"floor={Tmin_floor:.2f}N  p1={Tmin_p1:.2f}N  "
                f"mean={Tmin_mean:.2f}N  "
                f"frac<0.5N={frac_below_05*100:.2f}%  "
                f"frac<1.0N={frac_below_10*100:.2f}%  "
                f"qp_active={qp['any_active_frac']*100:.1f}%  "
                f"qp_trans={qp['any_transition_frac']*100:.3f}%")

            # Figure: min_i T_i(t) for this variant.
            fig, ax = plt.subplots(figsize=DOUBLE_COL)
            ax.plot(t_all, Tmin_all, color=COLORS[0], lw=0.9,
                    label=r"$\min_i T_i(t)$ (survivors only)")
            faults = first_fault_per_rope(df, num_drones(df))
            for rope_idx, tf in faults:
                ax.axvline(tf, **FAULT_STYLE)
            ax.axvline(T_PICKUP, color="grey", linestyle=":", lw=0.6,
                       label="pickup end")
            ax.axhline(0.5, color="red", linestyle="--", lw=0.5,
                       label="$T_\\min = 0.5$ N candidate")
            ax.axhline(1.0, color="orange", linestyle="--", lw=0.5,
                       label="$T_\\min = 1.0$ N candidate")
            ax.set_xlabel("t [s]")
            ax.set_ylabel(r"$\min_{i\in\mathcal{S}(t)} T_i$ [N]")
            ax.set_title(f"{tag}: survivor-minimum tension", fontsize=9)
            ax.set_ylim(bottom=-0.5)
            ax.legend(fontsize=6, loc="lower right")
            ax.grid(True, linewidth=0.3, alpha=0.3)
            fig.tight_layout()
            fig.savefig(OUT / "domain_audit_figs" / f"{tag}_Tmin.png",
                        dpi=180)
            plt.close(fig)

        tee("")
        tee("-" * 72)
        tee("Gate evaluation")
        tee("-" * 72)

        demonstrated = [r for r in rows if r["variant"] in
                        {"V3_single_wind", "V4_dual_5s_wind",
                         "V5_dual_10s_wind"}]
        if not demonstrated:
            tee("No demonstrated missions in CSV set; gate UNDETERMINED.")
            return
        floors = [r["Tmin_floor_N"] for r in demonstrated]
        global_floor = min(floors)
        tee(f"Global survivor-floor across demonstrated missions (V3, V4, V5): "
            f"{global_floor:.3f} N")

        # Proposed T_min candidates.
        for T_candidate in (0.5, 1.0, 2.0, 3.0, 5.0):
            violators = [r["variant"] for r in demonstrated
                         if r["Tmin_floor_N"] < T_candidate]
            status = "PASS" if not violators else "FAIL"
            tee(f"T_min = {T_candidate:.1f} N: {status} "
                f"({'violators: ' + ', '.join(violators) if violators else 'all pass'})")

        # Headline recommendation.
        if global_floor > 0.0:
            rec = min(0.5, 0.5 * global_floor)  # safety margin
            tee("")
            tee(f"*** GATE PASS ***  Recommended T_min = {rec:.2f} N "
                f"(half the worst demonstrated floor).")
            tee("    Contract's Omega_tau is non-empty; proceed to Reduction phase.")
        else:
            tee("")
            tee("*** GATE FAIL *** survivor minimum reached 0 N on "
                "some demonstrated mission post-pickup.")
            tee("    Kill the Reduction theorem; replace Prop 5.2 "
                "with an empirical claim backed by P2-A data.")

        # QP active-set transition summary.
        tee("")
        tee("QP active-set transition rate (D1-doctrine check):")
        trans_rates = [r["qp_transition_frac"] for r in demonstrated]
        tee(f"  Demonstrated missions:  range "
            f"{min(trans_rates)*100:.3f}% - {max(trans_rates)*100:.3f}%")
        if max(trans_rates) < 0.01:
            tee("  D1 single-active-regime doctrine TENABLE "
                "(< 1% transition rate).")
        else:
            tee("  D1 single-active-regime doctrine QUESTIONABLE "
                "(>= 1% transition rate); consider D2 doctrine "
                "or narrower theorem domain.")

    pd.DataFrame(rows).to_csv(OUT / "domain_audit_report.csv",
                              index=False)
    print(f"\nReport: {OUT / 'domain_audit_report.csv'}")
    print(f"Figures: {OUT / 'domain_audit_figs'}")
    print(f"Log: {log_path}")


if __name__ == "__main__":
    main()
