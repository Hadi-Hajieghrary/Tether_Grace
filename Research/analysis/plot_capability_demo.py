#!/usr/bin/env python3
"""Analysis plots for the six-variant capability-demonstration campaign.

Reads the six scenario CSVs produced by `run_capability_demo.sh`
and emits a comparative figure suite + a single-line summary table.

Usage
-----
    python3 plot_capability_demo.py <demo_root>

where <demo_root> defaults to `output/capability_demo/`. Outputs land
under `<demo_root>/_summary/`.
"""
from __future__ import annotations

import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ieee_style import (  # noqa: E402
    COLORS, DOUBLE_COL, DOUBLE_COL_TALL, SINGLE_COL,
    annotate_faults, detect_faults, setup_style, trim_start,
)

setup_style()

VARIANTS = [
    ("V1_nominal_nowind",     "V1 nominal, no wind"),
    ("V2_nominal_wind",       "V2 nominal, 4 m/s wind"),
    ("V3_single_wind",        "V3 single fault, wind"),
    ("V4_dual_5s_wind",       "V4 dual Δt=5 s, wind"),
    ("V5_dual_10s_wind",      "V5 dual Δt=10 s, wind"),
    ("V6_dual_5s_fullstack",  "V6 dual Δt=5 s, fullstack"),
]


def load_variant(root: Path, tag: str) -> pd.DataFrame | None:
    path = root / tag / f"scenario_{tag}.csv"
    if not path.exists():
        return None
    return pd.read_csv(path)


def tracking_error(df: pd.DataFrame) -> np.ndarray:
    err = df[["ref_x", "ref_y", "ref_z"]].values - df[
        ["payload_x", "payload_y", "payload_z"]
    ].values
    return np.linalg.norm(err, axis=1)


def peak_tension(df: pd.DataFrame, N: int) -> np.ndarray:
    cols = [f"tension_{i}" for i in range(N)]
    return df[cols].max(axis=1).values


def sigma_T(df: pd.DataFrame, N: int) -> np.ndarray:
    cols = [f"tension_{i}" for i in range(N)]
    # Running std of per-rope tensions across drones.
    return df[cols].std(axis=1).values


def num_drones(df: pd.DataFrame) -> int:
    n = 0
    while f"quad{n}_x" in df.columns:
        n += 1
    return n


def plot_tracking_error_overlay(variants, out_path: Path) -> None:
    fig, ax = plt.subplots(figsize=DOUBLE_COL)
    for k, (tag, label) in enumerate(VARIANTS):
        df = variants.get(tag)
        if df is None:
            continue
        t = df["time"].values
        e = tracking_error(df)
        ax.plot(t, e, color=COLORS[k % len(COLORS)], lw=1.1, label=label)
    # Overlay fault markers from the hardest case for reference.
    df_v4 = variants.get("V4_dual_5s_wind")
    if df_v4 is not None:
        N = num_drones(df_v4)
        faults = detect_faults(df_v4, N)
        annotate_faults(ax, faults)
    ax.set_xlabel("time [s]")
    ax.set_ylabel(r"payload tracking error $\|e_p\|$ [m]")
    ax.legend(ncol=2, fontsize=7, loc="upper left")
    ax.grid(True, linewidth=0.3, alpha=0.3)
    fig.savefig(out_path)
    plt.close(fig)


def plot_peak_tension_overlay(variants, out_path: Path) -> None:
    fig, ax = plt.subplots(figsize=DOUBLE_COL)
    for k, (tag, label) in enumerate(VARIANTS):
        df = variants.get(tag)
        if df is None:
            continue
        N = num_drones(df)
        t = df["time"].values
        tp = peak_tension(df, N)
        ax.plot(t, tp, color=COLORS[k % len(COLORS)], lw=1.1, label=label)
    ax.axhline(100.0, color="red", linestyle="--", lw=0.8,
               label=r"$T_\text{max}$ ceiling = 100 N")
    df_v4 = variants.get("V4_dual_5s_wind")
    if df_v4 is not None:
        N = num_drones(df_v4)
        annotate_faults(ax, detect_faults(df_v4, N))
    ax.set_xlabel("time [s]")
    ax.set_ylabel(r"peak rope tension $\max_j T_j$ [N]")
    ax.legend(ncol=2, fontsize=7, loc="upper left")
    ax.grid(True, linewidth=0.3, alpha=0.3)
    fig.savefig(out_path)
    plt.close(fig)


def plot_sigmaT_overlay(variants, out_path: Path) -> None:
    fig, ax = plt.subplots(figsize=DOUBLE_COL)
    for k, (tag, label) in enumerate(VARIANTS):
        df = variants.get(tag)
        if df is None:
            continue
        N = num_drones(df)
        t = df["time"].values
        s = sigma_T(df, N)
        ax.plot(t, s, color=COLORS[k % len(COLORS)], lw=1.1, label=label)
    df_v4 = variants.get("V4_dual_5s_wind")
    if df_v4 is not None:
        N = num_drones(df_v4)
        annotate_faults(ax, detect_faults(df_v4, N))
    ax.set_xlabel("time [s]")
    ax.set_ylabel(r"load-share imbalance $\sigma_T$ [N]")
    ax.legend(ncol=2, fontsize=7, loc="upper left")
    ax.grid(True, linewidth=0.3, alpha=0.3)
    fig.savefig(out_path)
    plt.close(fig)


def plot_3d_trajectories(variants, out_path: Path) -> None:
    fig = plt.figure(figsize=DOUBLE_COL_TALL)
    ax = fig.add_subplot(111, projection="3d")
    for k, (tag, label) in enumerate(VARIANTS):
        df = variants.get(tag)
        if df is None:
            continue
        ax.plot(df["payload_x"], df["payload_y"], df["payload_z"],
                color=COLORS[k % len(COLORS)], lw=0.9, label=label)
    # Reference from V1 (same for every run).
    df_v1 = variants.get("V1_nominal_nowind")
    if df_v1 is not None:
        ax.plot(df_v1["ref_x"], df_v1["ref_y"], df_v1["ref_z"],
                color="black", linestyle="--", lw=0.7, label="reference")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.legend(fontsize=6, loc="upper left")
    fig.savefig(out_path)
    plt.close(fig)


def plot_per_variant_story(variants, out_dir: Path) -> None:
    """One six-panel figure per variant — the full narrative at a glance."""
    for tag, label in VARIANTS:
        df = variants.get(tag)
        if df is None:
            continue
        N = num_drones(df)
        t = df["time"].values
        fig, axes = plt.subplots(3, 2, figsize=(8.5, 6.5))
        fig.suptitle(f"{label}  —  payload 15 kg, 5-drone lemniscate3D", fontsize=9)

        # (0,0) tracking error
        ax = axes[0, 0]
        ax.plot(t, tracking_error(df), color=COLORS[0], lw=1.1)
        ax.set_ylabel(r"$\|e_p\|$ [m]")
        ax.set_xlabel("t [s]")
        annotate_faults(ax, detect_faults(df, N))
        ax.grid(True, linewidth=0.3, alpha=0.3)

        # (0,1) payload altitude vs reference
        ax = axes[0, 1]
        ax.plot(t, df["payload_z"], color=COLORS[0], lw=1.1, label="payload z")
        ax.plot(t, df["ref_z"],     color="black", linestyle="--", lw=0.8,
                label="reference")
        ax.set_ylabel("z [m]")
        ax.set_xlabel("t [s]")
        ax.legend(fontsize=6, loc="lower right")
        annotate_faults(ax, detect_faults(df, N))
        ax.grid(True, linewidth=0.3, alpha=0.3)

        # (1,0) per-drone rope tensions
        ax = axes[1, 0]
        for i in range(N):
            ax.plot(t, df[f"tension_{i}"], lw=0.8,
                    color=COLORS[(i + 1) % len(COLORS)],
                    label=f"rope {i}")
        ax.axhline(100.0, color="red", linestyle="--", lw=0.6)
        ax.set_ylabel(r"$T_i$ [N]")
        ax.set_xlabel("t [s]")
        ax.legend(ncol=3, fontsize=6, loc="upper right")
        annotate_faults(ax, detect_faults(df, N))
        ax.grid(True, linewidth=0.3, alpha=0.3)

        # (1,1) load-share imbalance
        ax = axes[1, 1]
        ax.plot(t, sigma_T(df, N), color=COLORS[3], lw=1.1)
        ax.set_ylabel(r"$\sigma_T$ [N]")
        ax.set_xlabel("t [s]")
        annotate_faults(ax, detect_faults(df, N))
        ax.grid(True, linewidth=0.3, alpha=0.3)

        # (2,0) thrust commands
        ax = axes[2, 0]
        for i in range(N):
            if f"thrust_cmd_{i}" not in df.columns:
                continue
            ax.plot(t, df[f"thrust_cmd_{i}"], lw=0.7,
                    color=COLORS[(i + 1) % len(COLORS)],
                    label=f"drone {i}")
        ax.axhline(150.0, color="red", linestyle="--", lw=0.6)
        ax.set_ylabel("thrust cmd [N]")
        ax.set_xlabel("t [s]")
        ax.legend(ncol=3, fontsize=6, loc="upper right")
        annotate_faults(ax, detect_faults(df, N))
        ax.grid(True, linewidth=0.3, alpha=0.3)

        # (2,1) 3-D payload path (bird's-eye XY)
        ax = axes[2, 1]
        ax.plot(df["ref_x"], df["ref_y"], color="black", linestyle="--",
                lw=0.7, label="reference")
        ax.plot(df["payload_x"], df["payload_y"], color=COLORS[0], lw=0.9,
                label="payload")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_aspect("equal")
        ax.legend(fontsize=6, loc="upper right")
        ax.grid(True, linewidth=0.3, alpha=0.3)

        plt.tight_layout(rect=(0, 0, 1, 0.96))
        for ext in ("pdf", "png"):
            fig.savefig(out_dir / f"variant_{tag}.{ext}")
        plt.close(fig)


def plot_per_variant_split(variants, out_dir: Path) -> None:
    """Emit each variant's six panels as independent PNG files.

    Files are named `variant_<tag>_<panel>.png` with
    panel in {trackerr, altitude, tensions, sigmaT, thrust, xy}.
    """
    split_dir = out_dir / "split"
    split_dir.mkdir(parents=True, exist_ok=True)
    for tag, label in VARIANTS:
        df = variants.get(tag)
        if df is None:
            continue
        N = num_drones(df)
        t = df["time"].values
        faults = detect_faults(df, N)

        def _save(name: str, draw):
            fig, ax = plt.subplots(figsize=SINGLE_COL)
            draw(ax)
            annotate_faults(ax, faults)
            ax.grid(True, linewidth=0.3, alpha=0.3)
            fig.tight_layout()
            fig.savefig(split_dir / f"variant_{tag}_{name}.png", dpi=180)
            plt.close(fig)

        # (a) tracking error
        def draw_trackerr(ax):
            ax.plot(t, tracking_error(df), color=COLORS[0], lw=1.1)
            ax.set_ylabel(r"$\|e_p\|$ [m]")
            ax.set_xlabel("t [s]")
        _save("trackerr", draw_trackerr)

        # (b) payload altitude vs reference
        def draw_altitude(ax):
            ax.plot(t, df["payload_z"], color=COLORS[0], lw=1.1,
                    label="payload z")
            ax.plot(t, df["ref_z"], color="black", linestyle="--", lw=0.8,
                    label="reference")
            ax.set_ylabel("z [m]")
            ax.set_xlabel("t [s]")
            ax.legend(fontsize=7, loc="lower right")
        _save("altitude", draw_altitude)

        # (c) per-drone rope tensions
        def draw_tensions(ax):
            for i in range(N):
                ax.plot(t, df[f"tension_{i}"], lw=0.8,
                        color=COLORS[(i + 1) % len(COLORS)],
                        label=f"rope {i}")
            ax.axhline(100.0, color="red", linestyle="--", lw=0.6,
                       label=r"$T_\text{max}=100$ N")
            ax.set_ylabel(r"$T_i$ [N]")
            ax.set_xlabel("t [s]")
            ax.legend(ncol=2, fontsize=6, loc="upper right")
        _save("tensions", draw_tensions)

        # (d) load-share imbalance
        def draw_sigmaT(ax):
            ax.plot(t, sigma_T(df, N), color=COLORS[3], lw=1.1)
            ax.set_ylabel(r"$\sigma_T$ [N]")
            ax.set_xlabel("t [s]")
        _save("sigmaT", draw_sigmaT)

        # (e) thrust commands
        def draw_thrust(ax):
            any_col = False
            for i in range(N):
                if f"thrust_cmd_{i}" not in df.columns:
                    continue
                any_col = True
                ax.plot(t, df[f"thrust_cmd_{i}"], lw=0.7,
                        color=COLORS[(i + 1) % len(COLORS)],
                        label=f"drone {i}")
            if any_col:
                ax.axhline(150.0, color="red", linestyle="--", lw=0.6,
                           label=r"$f_\text{max}=150$ N")
                ax.legend(ncol=2, fontsize=6, loc="upper right")
            ax.set_ylabel("thrust cmd [N]")
            ax.set_xlabel("t [s]")
        _save("thrust", draw_thrust)

        # (f) bird's-eye XY trajectory (no fault markers)
        fig, ax = plt.subplots(figsize=SINGLE_COL)
        ax.plot(df["ref_x"], df["ref_y"], color="black", linestyle="--",
                lw=0.7, label="reference")
        ax.plot(df["payload_x"], df["payload_y"], color=COLORS[0], lw=0.9,
                label="payload")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_aspect("equal")
        ax.legend(fontsize=7, loc="upper right")
        ax.grid(True, linewidth=0.3, alpha=0.3)
        fig.tight_layout()
        fig.savefig(split_dir / f"variant_{tag}_xy.png", dpi=180)
        plt.close(fig)


def summary_row(tag: str, df: pd.DataFrame, t_start: float = 0.5) -> dict:
    df = trim_start(df, t_start)
    N = num_drones(df)
    e = tracking_error(df)
    tp = peak_tension(df, N)
    sT = sigma_T(df, N)
    # Max QP solve time across drones.
    qp_cols = [f"qp_solve_us_{i}" for i in range(N)
               if f"qp_solve_us_{i}" in df.columns]
    qp99 = (np.percentile(df[qp_cols].values, 99) if qp_cols else float("nan"))
    return {
        "variant": tag,
        "rms_err_m":     float(np.sqrt(np.mean(e * e))),
        "peak_err_m":    float(np.max(e)),
        "peak_T_N":      float(np.max(tp)),
        "rms_sigma_T":   float(np.sqrt(np.mean(sT * sT))),
        "qp_p99_us":     float(qp99),
    }


def build_summary(variants, out_path: Path) -> pd.DataFrame:
    rows = []
    for tag, _ in VARIANTS:
        df = variants.get(tag)
        if df is None:
            continue
        rows.append(summary_row(tag, df))
    table = pd.DataFrame(rows)
    table.to_csv(out_path, index=False, float_format="%.3f")
    return table


def main() -> None:
    root = Path(sys.argv[1] if len(sys.argv) > 1
                else "/workspaces/Tether_Grace/output/capability_demo")
    out = root / "_summary"
    out.mkdir(parents=True, exist_ok=True)

    variants = {tag: load_variant(root, tag) for tag, _ in VARIANTS}
    loaded = [t for t, d in variants.items() if d is not None]
    missing = [t for t, d in variants.items() if d is None]
    print(f"Loaded {len(loaded)}/{len(VARIANTS)} variants")
    if missing:
        print(f"  MISSING: {missing}")

    for ext in ("pdf", "png"):
        plot_tracking_error_overlay(variants, out / f"fig_tracking_error.{ext}")
        plot_peak_tension_overlay(variants,   out / f"fig_peak_tension.{ext}")
        plot_sigmaT_overlay(variants,         out / f"fig_sigma_T.{ext}")
        plot_3d_trajectories(variants,        out / f"fig_3d_trajectories.{ext}")

    plot_per_variant_story(variants, out)
    plot_per_variant_split(variants, out)

    summary = build_summary(variants, out / "summary_metrics.csv")
    print("\n=== Campaign summary ===")
    print(summary.to_string(index=False))
    print(f"\nFigures written to: {out}")


if __name__ == "__main__":
    main()
