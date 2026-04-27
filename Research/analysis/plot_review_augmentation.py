#!/usr/bin/env python3
"""Reviewer-response Phase 1 augmentation.

Regenerates, **from the existing** capability-demo CSV logs:

    - Fault-window zoom panels (trackerr, altitude, tensions, thrust) at
      [t_f - 0.5, t_f + 2] for every detected fault event.
    - Normalised load-share plots T_i / sum_j T_j per variant.
    - Constraint-activity plots: thrust margin (f_max - thrust_cmd) and
      tilt margin (theta_max - |tilt|) per drone per variant.
    - Per-fault recovery metrics table: peak recovery error, IAE over
      [t_f, t_f+2], settling time, max payload altitude sag, time to
      tension-redistribution, max sigma_T during recovery.

No new simulation runs are performed; all outputs come from the
archived CSVs under `output/capability_demo/`.
"""
from __future__ import annotations

import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ieee_style import (  # noqa: E402
    COLORS, FAULT_STYLE, SINGLE_COL,
    annotate_faults, detect_faults, setup_style, trim_start,
)
from plot_capability_demo import (  # noqa: E402
    VARIANTS, load_variant, num_drones, peak_tension, sigma_T, tracking_error,
)

setup_style()

# Actuator envelope — from A1_parameters table.
F_MAX = 150.0        # N per-drone thrust ceiling
THETA_MAX = 0.6      # rad tilt ceiling

# Zoom window around fault events (seconds).
ZOOM_BEFORE = 0.5
ZOOM_AFTER = 2.0

# Recovery-metric IAE / settling window.
RECOV_WINDOW_S = 2.0

# Settling: ||e_p|| stays below max(1.2 * pre_fault_rms, SETTLE_FLOOR_M)
# for SETTLE_DWELL_S seconds continuously.
SETTLE_DWELL_S = 0.5
SETTLE_MULT = 1.2
SETTLE_FLOOR_M = 0.4


# Helpers.
def first_fault_per_rope(df: pd.DataFrame, N: int) -> list[tuple[int, float]]:
    """Return [(rope_idx, t_fault_s)] for each rope whose tension drops
    permanently below 0.2 N, excluding brief transient slack events
    (e.g. rope whip during trajectory turnarounds).

    A "permanent" fault is declared when the tension stays below the
    threshold for at least ``DWELL_S`` seconds continuously AND the
    rope had previously been loaded (T > 5 N sustained).  Ordered by
    fault time.
    """
    DWELL_S = 0.5
    LOAD_THRESH_N = 5.0
    LOAD_DWELL_S = 0.1

    faults = []
    t = df["time"].values
    dt = np.median(np.diff(t)) if len(t) > 1 else 1e-3
    dwell_n = max(5, int(round(DWELL_S / dt)))
    load_n = max(5, int(round(LOAD_DWELL_S / dt)))

    for i in range(N):
        T = df[f"tension_{i}"].values

        # Find earliest sustained-loaded index; skip first tick to
        # avoid the t=0 zero-initialisation.
        loaded_idx = None
        for k in range(1, len(T) - load_n):
            if (T[k:k + load_n] > LOAD_THRESH_N).all():
                loaded_idx = k
                break
        if loaded_idx is None:
            continue

        # From there, find the first index where T stays below 0.2 N
        # for >= DWELL_S.
        below = T < 0.2
        for k in range(loaded_idx + load_n, len(T) - dwell_n):
            if below[k:k + dwell_n].all():
                faults.append((i, float(t[k])))
                break

    faults.sort(key=lambda pair: pair[1])
    return faults


def window_mask(t: np.ndarray, t_lo: float, t_hi: float) -> np.ndarray:
    return (t >= t_lo) & (t <= t_hi)


def plot_fault_zoom(variants: dict, out_dir: Path) -> None:
    """Four PNGs per fault event: trackerr, altitude, tensions, thrust."""
    out_dir.mkdir(parents=True, exist_ok=True)
    for tag, label in VARIANTS:
        df = variants.get(tag)
        if df is None:
            continue
        N = num_drones(df)
        faults = first_fault_per_rope(df, N)
        if not faults:
            continue
        t = df["time"].values
        e = tracking_error(df)
        for k, (rope_idx, t_f) in enumerate(faults, start=1):
            lo, hi = t_f - ZOOM_BEFORE, t_f + ZOOM_AFTER
            m = window_mask(t, lo, hi)
            tag_k = f"{tag}_fault{k}"

            # (a) tracking error
            fig, ax = plt.subplots(figsize=SINGLE_COL)
            ax.plot(t[m], e[m], color=COLORS[0], lw=1.2)
            ax.axvline(t_f, **FAULT_STYLE)
            ax.set_xlabel("t [s]")
            ax.set_ylabel(r"$\|e_p\|$ [m]")
            ax.set_title(f"{tag_k}: rope {rope_idx} @ t={t_f:.1f}s", fontsize=8)
            ax.grid(True, linewidth=0.3, alpha=0.3)
            fig.tight_layout()
            fig.savefig(out_dir / f"rev_zoom_{tag_k}_trackerr.png", dpi=180)
            plt.close(fig)

            # (b) payload altitude vs reference
            fig, ax = plt.subplots(figsize=SINGLE_COL)
            ax.plot(t[m], df["payload_z"].values[m], color=COLORS[0],
                    lw=1.2, label="payload z")
            ax.plot(t[m], df["ref_z"].values[m], color="black",
                    linestyle="--", lw=0.8, label="reference")
            ax.axvline(t_f, **FAULT_STYLE)
            ax.set_xlabel("t [s]")
            ax.set_ylabel("z [m]")
            ax.legend(fontsize=7, loc="lower right")
            ax.grid(True, linewidth=0.3, alpha=0.3)
            fig.tight_layout()
            fig.savefig(out_dir / f"rev_zoom_{tag_k}_altitude.png", dpi=180)
            plt.close(fig)

            # (c) per-rope tensions
            fig, ax = plt.subplots(figsize=SINGLE_COL)
            for i in range(N):
                ax.plot(t[m], df[f"tension_{i}"].values[m], lw=0.9,
                        color=COLORS[(i + 1) % len(COLORS)],
                        label=f"rope {i}")
            ax.axvline(t_f, **FAULT_STYLE)
            ax.axhline(100.0, color="red", linestyle="--", lw=0.5)
            ax.set_xlabel("t [s]")
            ax.set_ylabel(r"$T_i$ [N]")
            ax.legend(ncol=2, fontsize=6, loc="upper right")
            ax.grid(True, linewidth=0.3, alpha=0.3)
            fig.tight_layout()
            fig.savefig(out_dir / f"rev_zoom_{tag_k}_tensions.png", dpi=180)
            plt.close(fig)

            # (d) thrust commands
            fig, ax = plt.subplots(figsize=SINGLE_COL)
            for i in range(N):
                col = f"thrust_cmd_{i}"
                if col not in df.columns:
                    continue
                ax.plot(t[m], df[col].values[m], lw=0.8,
                        color=COLORS[(i + 1) % len(COLORS)],
                        label=f"drone {i}")
            ax.axvline(t_f, **FAULT_STYLE)
            ax.axhline(F_MAX, color="red", linestyle="--", lw=0.5)
            ax.set_xlabel("t [s]")
            ax.set_ylabel("thrust cmd [N]")
            ax.legend(ncol=2, fontsize=6, loc="upper right")
            ax.grid(True, linewidth=0.3, alpha=0.3)
            fig.tight_layout()
            fig.savefig(out_dir / f"rev_zoom_{tag_k}_thrust.png", dpi=180)
            plt.close(fig)


def plot_load_share(variants: dict, out_dir: Path) -> None:
    """Normalised per-rope share T_i / sum_j T_j per variant.  Reveals
    load-redistribution dynamics more clearly than absolute tensions."""
    out_dir.mkdir(parents=True, exist_ok=True)
    for tag, label in VARIANTS:
        df = variants.get(tag)
        if df is None:
            continue
        N = num_drones(df)
        t = df["time"].values
        T = np.stack([df[f"tension_{i}"].values for i in range(N)], axis=1)
        tot = T.sum(axis=1)
        tot[tot <= 1e-6] = np.nan  # avoid div-by-zero during ground-hold
        share = T / tot[:, None]

        fig, ax = plt.subplots(figsize=SINGLE_COL)
        for i in range(N):
            ax.plot(t, share[:, i], lw=0.9,
                    color=COLORS[(i + 1) % len(COLORS)],
                    label=f"rope {i}")
        ax.axhline(1.0 / N, color="black", linestyle=":", lw=0.6,
                   label=f"equal share 1/N={1.0/N:.2f}")
        annotate_faults(ax, [(r, tf) for r, tf in
                             first_fault_per_rope(df, N)])
        ax.set_xlabel("t [s]")
        ax.set_ylabel(r"normalised share $T_i/\sum_j T_j$")
        ax.set_ylim(-0.05, 1.0)
        ax.legend(ncol=2, fontsize=6, loc="upper right")
        ax.grid(True, linewidth=0.3, alpha=0.3)
        fig.tight_layout()
        fig.savefig(out_dir / f"rev_loadshare_{tag}.png", dpi=180)
        plt.close(fig)


def plot_constraint_activity(variants: dict, out_dir: Path) -> None:
    """Thrust margin (f_max - thrust_cmd) and tilt margin (theta_max -
    |tilt|) per drone per variant."""
    out_dir.mkdir(parents=True, exist_ok=True)
    for tag, label in VARIANTS:
        df = variants.get(tag)
        if df is None:
            continue
        N = num_drones(df)
        t = df["time"].values

        # (a) thrust margin
        fig, ax = plt.subplots(figsize=SINGLE_COL)
        for i in range(N):
            col = f"thrust_cmd_{i}"
            if col not in df.columns:
                continue
            margin = F_MAX - df[col].values
            ax.plot(t, margin, lw=0.7,
                    color=COLORS[(i + 1) % len(COLORS)],
                    label=f"drone {i}")
        ax.axhline(0.0, color="red", linestyle="--", lw=0.5,
                   label="saturation")
        annotate_faults(ax, first_fault_per_rope(df, N))
        ax.set_xlabel("t [s]")
        ax.set_ylabel(r"$f_\text{max} - f_\text{cmd}$ [N]")
        ax.legend(ncol=2, fontsize=6, loc="lower right")
        ax.grid(True, linewidth=0.3, alpha=0.3)
        fig.tight_layout()
        fig.savefig(out_dir / f"rev_thrustmargin_{tag}.png", dpi=180)
        plt.close(fig)

        # (b) tilt margin
        fig, ax = plt.subplots(figsize=SINGLE_COL)
        for i in range(N):
            col = f"tilt_mag_{i}"
            if col not in df.columns:
                continue
            margin = THETA_MAX - df[col].values
            ax.plot(t, margin, lw=0.7,
                    color=COLORS[(i + 1) % len(COLORS)],
                    label=f"drone {i}")
        ax.axhline(0.0, color="red", linestyle="--", lw=0.5,
                   label="saturation")
        annotate_faults(ax, first_fault_per_rope(df, N))
        ax.set_xlabel("t [s]")
        ax.set_ylabel(r"$\theta_\text{max} - |\theta|$ [rad]")
        ax.legend(ncol=2, fontsize=6, loc="lower right")
        ax.grid(True, linewidth=0.3, alpha=0.3)
        fig.tight_layout()
        fig.savefig(out_dir / f"rev_tiltmargin_{tag}.png", dpi=180)
        plt.close(fig)


def compute_recovery_metrics(variants: dict) -> pd.DataFrame:
    """One row per (variant, fault_event)."""
    rows = []
    for tag, label in VARIANTS:
        df = variants.get(tag)
        if df is None:
            continue
        N = num_drones(df)
        t = df["time"].values
        e = tracking_error(df)
        z_sag = df["payload_z"].values - df["ref_z"].values
        sT = sigma_T(df, N)

        faults = first_fault_per_rope(df, N)
        for k, (rope_idx, t_f) in enumerate(faults, start=1):
            # pre-fault RMS — from BURN_IN to fault
            pre_mask = (t >= 0.5) & (t < t_f)
            pre_rms = float(np.sqrt(np.mean(e[pre_mask] ** 2))) \
                if pre_mask.any() else float("nan")

            # Recovery window
            rec_mask = (t >= t_f) & (t <= t_f + RECOV_WINDOW_S)
            t_rec = t[rec_mask]
            e_rec = e[rec_mask]
            z_rec = z_sag[rec_mask]
            sT_rec = sT[rec_mask]

            peak_err_rec = float(np.max(e_rec)) if len(e_rec) else float("nan")
            iae_rec = float(np.trapezoid(e_rec, t_rec)) if len(e_rec) > 1 \
                else float("nan")
            max_sag = float(np.min(z_rec)) if len(z_rec) else float("nan")
            max_sigmaT_rec = float(np.max(sT_rec)) if len(sT_rec) \
                else float("nan")

            # Settling time: earliest t_s >= t_f such that ||e_p|| stays
            # below threshold for >= SETTLE_DWELL_S seconds continuously.
            threshold = max(SETTLE_MULT * pre_rms, SETTLE_FLOOR_M)
            settle_s = _settling_time(t_rec, e_rec, threshold, SETTLE_DWELL_S)
            settle_rel = (settle_s - t_f) if settle_s is not None \
                else float("inf")

            # Time to tension redistribution: earliest time after t_f
            # when sigma_T first rises to at least 80% of its
            # post-fault steady-state mean (window [t_f+1, t_f+2]).
            late_mask = (t >= t_f + 1.0) & (t <= t_f + RECOV_WINDOW_S)
            if late_mask.any():
                sT_post_mean = float(np.mean(sT[late_mask]))
                sT_pre_mean = float(np.mean(sT[pre_mask])) \
                    if pre_mask.any() else 0.0
                # Proportional threshold above pre-fault baseline.
                target = sT_pre_mean + 0.8 * (sT_post_mean - sT_pre_mean)
                idx_f = int(np.searchsorted(t, t_f, side="left"))
                idx_hi = int(np.searchsorted(t, t_f + RECOV_WINDOW_S,
                                             side="right"))
                hit = np.where(sT[idx_f:idx_hi] >= target)[0]
                redist_rel = float(t[idx_f + hit[0]] - t_f) \
                    if len(hit) else float("inf")
            else:
                redist_rel = float("nan")

            rows.append({
                "variant":           tag,
                "fault_index":       k,
                "rope":              rope_idx,
                "t_fault_s":         round(t_f, 3),
                "pre_fault_rms_m":   round(pre_rms, 4),
                "peak_err_rec_m":    round(peak_err_rec, 4),
                "iae_rec_m_s":       round(iae_rec, 4),
                "max_payload_sag_m": round(max_sag, 4),
                "max_sigmaT_rec_N":  round(max_sigmaT_rec, 3),
                "settle_time_s":     round(settle_rel, 3),
                "t_redistribute_s":  round(redist_rel, 3),
            })
    return pd.DataFrame(rows)


def _settling_time(t: np.ndarray, x: np.ndarray, threshold: float,
                   dwell_s: float) -> float | None:
    """First t[i] such that max(x[i:j]) <= threshold for t[j]-t[i] >= dwell."""
    if len(t) == 0:
        return None
    # Forward scan: maintain the earliest index where the window dips.
    for i in range(len(t)):
        j = i
        while j < len(t) and t[j] - t[i] < dwell_s:
            if x[j] > threshold:
                break
            j += 1
        else:
            # inner loop exhausted without break AND j reached dwell
            if j < len(t) and t[j] - t[i] >= dwell_s and \
                    np.max(x[i:j + 1]) <= threshold:
                return float(t[i])
        # if breaked out of inner loop, continue
    # fallback: scan from the last sample
    if np.all(x[-max(1, int(dwell_s / 0.01)):] <= threshold):
        return float(t[-int(dwell_s / 0.01)])
    return None


def _settling_time_around(t: np.ndarray, x: np.ndarray, center: float,
                          band: float, dwell_s: float) -> float | None:
    """First t[i] such that |x[k] - center| <= band for all k in the
    dwell window starting at i."""
    if len(t) == 0:
        return None
    for i in range(len(t)):
        j = i
        while j < len(t) and t[j] - t[i] < dwell_s:
            if abs(x[j] - center) > band:
                break
            j += 1
        else:
            if j < len(t) and t[j] - t[i] >= dwell_s and \
                    np.max(np.abs(x[i:j + 1] - center)) <= band:
                return float(t[i])
    return None


def main() -> None:
    root = Path(sys.argv[1] if len(sys.argv) > 1
                else "/workspaces/Tether_Grace/output/capability_demo")
    out = root / "_summary" / "review"
    out.mkdir(parents=True, exist_ok=True)

    variants = {tag: load_variant(root, tag) for tag, _ in VARIANTS}
    loaded = [t for t, d in variants.items() if d is not None]
    missing = [t for t, d in variants.items() if d is None]
    print(f"Loaded {len(loaded)}/{len(VARIANTS)} variants")
    if missing:
        print(f"  MISSING: {missing}")

    print("\n--- Phase 1 augmentation ---")
    print("  Fault-window zoom panels ...")
    plot_fault_zoom(variants, out)
    print("  Normalised load-share ...")
    plot_load_share(variants, out)
    print("  Constraint-activity (thrust/tilt margins) ...")
    plot_constraint_activity(variants, out)
    print("  Per-fault recovery metrics ...")
    metrics = compute_recovery_metrics(variants)
    metrics.to_csv(out / "recovery_metrics.csv", index=False)
    print(metrics.to_string(index=False))
    print(f"\nAll reviewer-response artefacts under: {out}")


if __name__ == "__main__":
    main()
