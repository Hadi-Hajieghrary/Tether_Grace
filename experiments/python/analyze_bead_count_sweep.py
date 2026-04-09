#!/usr/bin/env python3
"""Analyze bead-count sweep results and generate paper-facing artifacts.

Reads sweep outputs from outputs/bead_count_sweep/five_drones/,
computes event-centered fault metrics with clean windows,
and generates the manuscript figure and table.
"""
from __future__ import annotations

import argparse
import json
import shutil
from dataclasses import asdict, dataclass, field
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path("/workspaces/Tether_Grace")
SWEEP_OUTPUT_ROOT = ROOT / "outputs" / "bead_count_sweep" / "five_drones"
FIGURE_DPI = 300
LIFTOFF_HEIGHT_DELTA = 0.05
LIFTOFF_HOLD_TIME = 0.1

plt.rcParams.update({
    "font.family": "serif",
    "font.size": 9,
    "axes.grid": True,
    "grid.alpha": 0.25,
    "grid.linestyle": "--",
    "axes.titlesize": 10,
    "axes.labelsize": 9,
    "legend.fontsize": 7.5,
    "xtick.labelsize": 8,
    "ytick.labelsize": 8,
    "figure.facecolor": "white",
})

# ── Study windows (centralized for tuning) ────────────────────────────────
# These define the time windows for fault-event metrics on the five-drone
# scenario: cable 1 fault at 7.0 s, cable 3 fault at 12.0 s.

NOMINAL_BASELINE_START = 3.0     # after liftoff transient settles
NOMINAL_BASELINE_END = 6.8       # before first fault

FAULT1_TIME = 7.0
FAULT1_PEAK_START = 7.0
FAULT1_PEAK_END = 9.0
FAULT1_RECOVERY_START = 7.0
FAULT1_RECOVERY_END = 11.5

FAULT2_TIME = 12.0
FAULT2_LOCAL_BASELINE_START = 10.5   # clean window after fault 1 recovery
FAULT2_LOCAL_BASELINE_END = 11.8     # before second fault
FAULT2_PEAK_START = 12.0
FAULT2_PEAK_END = 14.0
FAULT2_RECOVERY_START = 12.0
FAULT2_RECOVERY_END = 18.0

# Recovery: error must stay below multiplier × baseline RMSE for this long
RECOVERY_MULTIPLIER = 1.5
RECOVERY_HOLD = 0.5  # seconds


# ── Data structures ───────────────────────────────────────────────────────

@dataclass
class FaultEventMetrics:
    fault_time: float = 0.0
    baseline_rmse_cm: float = 0.0
    peak_error_cm: float = 0.0
    post_fault_rmse_cm: float = 0.0
    recovery_time_s: float = 0.0


@dataclass
class BeadCountMetrics:
    num_beads: int = 0
    wall_time_seconds: float = 0.0
    # Whole-run context
    whole_run_rmse_cm: float = 0.0
    final_load_error_cm: float = 0.0
    peak_load_speed_m_s: float = 0.0
    peak_cable_tension_N: float = 0.0
    # Event-centered
    fault1: FaultEventMetrics = field(default_factory=FaultEventMetrics)
    fault2: FaultEventMetrics = field(default_factory=FaultEventMetrics)

    def to_dict(self) -> dict:
        return asdict(self)


# ── Trajectory reference (mirrors run_fault_schedule_batch.py) ────────────

def build_waypoints(initial_altitude: float, duration: float) -> list[tuple[np.ndarray, float, float]]:
    t1 = 0.08 * duration
    t2 = 0.18 * duration
    t3 = 0.28 * duration
    t4 = 0.38 * duration
    t5 = 0.48 * duration
    t6 = 0.58 * duration
    t7 = 0.68 * duration
    t8 = 0.78 * duration
    t9 = 0.85 * duration
    t10 = 0.90 * duration
    return [
        (np.array([0.0, 0.0, initial_altitude]), 0.0, 1.0),
        (np.array([0.0, 0.0, 2.8]), t1, 0.4),
        (np.array([1.6, 0.5, 3.0]), t2, 0.2),
        (np.array([2.4, 1.4, 3.2]), t3, 0.2),
        (np.array([3.0, 0.0, 3.1]), t4, 0.2),
        (np.array([2.1, -1.4, 2.9]), t5, 0.2),
        (np.array([0.0, 0.0, 3.0]), t6, 0.2),
        (np.array([-1.6, 0.5, 3.1]), t7, 0.2),
        (np.array([-2.6, 1.4, 3.3]), t8, 0.2),
        (np.array([-3.0, 0.0, 3.1]), t9, 0.2),
        (np.array([-2.0, -1.2, 2.9]), t10, 0.2),
        (np.array([0.0, 0.0, 2.6]), duration, 0.0),
    ]


def evaluate_waypoint_trajectory(
    waypoints: list[tuple[np.ndarray, float, float]], time_value: float,
) -> np.ndarray:
    segment_start_time = 0.0
    for index, (position, arrival_time, hold_time) in enumerate(waypoints):
        hold_end_time = arrival_time + hold_time
        if time_value <= arrival_time:
            if index == 0:
                return position.copy()
            previous_position = waypoints[index - 1][0]
            segment_duration = arrival_time - segment_start_time
            if segment_duration > 1e-6:
                alpha = (time_value - segment_start_time) / segment_duration
                return (1.0 - alpha) * previous_position + alpha * position
            return position.copy()
        if time_value <= hold_end_time:
            return position.copy()
        segment_start_time = hold_end_time
    return waypoints[-1][0].copy()


def build_load_reference(time_values: np.ndarray, duration: float) -> np.ndarray:
    waypoints = build_waypoints(initial_altitude=1.2, duration=duration)
    reference = np.zeros((time_values.shape[0], 3), dtype=float)
    for idx, t in enumerate(time_values):
        reference[idx] = evaluate_waypoint_trajectory(waypoints, float(t))
    return reference


def infer_payload_z_offset(
    time_values: np.ndarray,
    base_ref_z: np.ndarray,
    actual_z: np.ndarray,
    first_fault_time: float,
) -> float:
    window_start = min(2.0, max(float(time_values[0]), first_fault_time * 0.25))
    window_end = max(window_start + 0.5, first_fault_time - 0.5)
    mask = (time_values >= window_start) & (time_values <= window_end)
    if not np.any(mask):
        return float(np.median(base_ref_z - actual_z))
    return float(np.median(base_ref_z[mask] - actual_z[mask]))


def infer_payload_liftoff_time(time_values: np.ndarray, actual_z: np.ndarray) -> float:
    initial_height = float(actual_z[0])
    threshold = initial_height + LIFTOFF_HEIGHT_DELTA
    for idx, t in enumerate(time_values):
        if actual_z[idx] < threshold:
            continue
        end_time = float(t) + LIFTOFF_HOLD_TIME
        end_idx = min(int(np.searchsorted(time_values, end_time, side="left")), len(time_values) - 1)
        if np.all(actual_z[idx:end_idx + 1] >= threshold):
            return float(t)
    return float(time_values[0])


def build_payload_reference(
    time_values: np.ndarray, duration: float, actual_z: np.ndarray, first_fault_time: float,
) -> tuple[np.ndarray, float]:
    reference = build_load_reference(time_values, duration)
    z_offset = infer_payload_z_offset(time_values, reference[:, 2], actual_z, first_fault_time)
    reference[:, 2] -= z_offset
    liftoff_time = infer_payload_liftoff_time(time_values, actual_z)
    reference[time_values < liftoff_time, 2] = float(actual_z[0])
    return reference, liftoff_time


# ── Metric computation ────────────────────────────────────────────────────

def window_rmse(errors_cm: np.ndarray, times: np.ndarray, t_start: float, t_end: float) -> float:
    mask = (times >= t_start) & (times < t_end)
    if not np.any(mask):
        return 0.0
    return float(np.sqrt(np.mean(errors_cm[mask] ** 2)))


def window_peak(errors_cm: np.ndarray, times: np.ndarray, t_start: float, t_end: float) -> float:
    mask = (times >= t_start) & (times < t_end)
    if not np.any(mask):
        return 0.0
    return float(np.max(errors_cm[mask]))


def compute_recovery_time(
    errors_cm: np.ndarray,
    times: np.ndarray,
    fault_time: float,
    search_end: float,
    threshold_cm: float,
) -> float:
    """Time from fault until error stays below threshold for RECOVERY_HOLD seconds.

    The search begins only after the error first exceeds the threshold,
    so that the pre-exceedance period is not counted as "already recovered."
    If the error never exceeds the threshold, recovery time is 0.0.
    """
    mask = (times >= fault_time) & (times < search_end)
    indices = np.where(mask)[0]

    # Find the first time the error exceeds the threshold
    exceeded = False
    for idx in indices:
        if not exceeded:
            if errors_cm[idx] >= threshold_cm:
                exceeded = True
            continue
        # Now search for sustained sub-threshold
        if errors_cm[idx] >= threshold_cm:
            continue
        hold_end = times[idx] + RECOVERY_HOLD
        hold_mask = (times >= times[idx]) & (times < hold_end)
        if np.any(hold_mask) and np.all(errors_cm[hold_mask] < threshold_cm):
            return float(times[idx] - fault_time)

    if not exceeded:
        return 0.0  # error never exceeded threshold
    return float(search_end - fault_time)


def compute_metrics_for_run(run_dir: Path, duration: float = 30.0) -> BeadCountMetrics | None:
    """Compute all metrics from a single bead-count run."""
    npz_path = run_dir / "full_drake_recording.npz"
    if not npz_path.exists():
        return None

    data = np.load(npz_path, allow_pickle=True)
    traj = data["trajectories"]
    traj_headers = list(data["trajectory_headers"])
    tension = data["tensions"]
    tension_headers = list(data["tension_headers"])

    time = traj[:, 0]
    load_x = traj[:, traj_headers.index("load_x")]
    load_y = traj[:, traj_headers.index("load_y")]
    load_z = traj[:, traj_headers.index("load_z")]
    load_xyz = np.column_stack([load_x, load_y, load_z])

    load_vx = traj[:, traj_headers.index("load_vx")]
    load_vy = traj[:, traj_headers.index("load_vy")]
    load_vz = traj[:, traj_headers.index("load_vz")]
    load_speed = np.sqrt(load_vx**2 + load_vy**2 + load_vz**2)

    reference, liftoff_time = build_payload_reference(time, duration, load_z, FAULT1_TIME)
    error_vec = load_xyz - reference
    error_cm = 100.0 * np.linalg.norm(error_vec, axis=1)

    # Peak cable tension across all cables
    tension_mag_cols = [
        idx for idx, h in enumerate(tension_headers) if h.endswith("_mag")
    ]
    peak_tension = float(np.max(tension[:, tension_mag_cols])) if tension_mag_cols else 0.0

    # Whole-run metrics (excluding startup/liftoff)
    post_liftoff = time >= max(liftoff_time + 1.0, NOMINAL_BASELINE_START)
    whole_rmse = float(np.sqrt(np.mean(error_cm[post_liftoff] ** 2))) if np.any(post_liftoff) else 0.0
    final_error = float(error_cm[-1])

    # Nominal baseline RMSE
    nominal_rmse = window_rmse(error_cm, time, NOMINAL_BASELINE_START, NOMINAL_BASELINE_END)

    # Fault 1 metrics
    f1 = FaultEventMetrics(fault_time=FAULT1_TIME)
    f1.baseline_rmse_cm = nominal_rmse
    f1.peak_error_cm = window_peak(error_cm, time, FAULT1_PEAK_START, FAULT1_PEAK_END)
    f1.post_fault_rmse_cm = window_rmse(error_cm, time, FAULT1_PEAK_START, FAULT1_RECOVERY_END)
    f1.recovery_time_s = compute_recovery_time(
        error_cm, time, FAULT1_TIME, FAULT1_RECOVERY_END,
        RECOVERY_MULTIPLIER * nominal_rmse,
    )

    # Fault 2 metrics (clean local baseline from inter-fault window)
    f2 = FaultEventMetrics(fault_time=FAULT2_TIME)
    f2.baseline_rmse_cm = window_rmse(error_cm, time, FAULT2_LOCAL_BASELINE_START, FAULT2_LOCAL_BASELINE_END)
    f2.peak_error_cm = window_peak(error_cm, time, FAULT2_PEAK_START, FAULT2_PEAK_END)
    f2.post_fault_rmse_cm = window_rmse(error_cm, time, FAULT2_PEAK_START, FAULT2_RECOVERY_END)
    f2.recovery_time_s = compute_recovery_time(
        error_cm, time, FAULT2_TIME, FAULT2_RECOVERY_END,
        RECOVERY_MULTIPLIER * f2.baseline_rmse_cm,
    )

    # Read wall time from sweep results if available
    wall_time = 0.0
    sweep_results_path = run_dir.parent / "sweep_results.json"
    if sweep_results_path.exists():
        sweep_data = json.loads(sweep_results_path.read_text(encoding="utf-8"))
        for entry in sweep_data.get("completed_runs", []):
            if str(run_dir) == entry.get("output_dir"):
                wall_time = entry.get("wall_time_seconds", 0.0)

    # Extract num_beads from directory name or run manifest
    manifest_path = run_dir / "run_manifest.json"
    if manifest_path.exists():
        run_manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        num_beads = int(run_manifest.get("num_rope_beads", 8))
    else:
        # Fallback: parse from directory name (e.g. "beads_08")
        dir_name = run_dir.name
        try:
            num_beads = int(dir_name.split("_")[1])
        except (IndexError, ValueError):
            num_beads = 8

    return BeadCountMetrics(
        num_beads=num_beads,
        wall_time_seconds=wall_time,
        whole_run_rmse_cm=round(whole_rmse, 3),
        final_load_error_cm=round(final_error, 3),
        peak_load_speed_m_s=round(float(np.max(load_speed)), 3),
        peak_cable_tension_N=round(peak_tension, 3),
        fault1=f1,
        fault2=f2,
    )


# ── Figure generation ─────────────────────────────────────────────────────

def generate_composite_figure(
    sweep_dir: Path,
    metrics_list: list[BeadCountMetrics],
    paper_subset: list[int] | None = None,
) -> Path:
    """Generate the paper-facing composite figure: full horizon + two fault zooms."""
    if paper_subset is None:
        paper_subset = [4, 8, 16]

    # Collect error traces for each bead count in the paper subset
    traces: dict[int, tuple[np.ndarray, np.ndarray]] = {}
    for m in metrics_list:
        if m.num_beads not in paper_subset:
            continue
        run_dir = sweep_dir / f"beads_{m.num_beads:02d}"
        npz_path = run_dir / "full_drake_recording.npz"
        if not npz_path.exists():
            continue
        data = np.load(npz_path, allow_pickle=True)
        traj = data["trajectories"]
        traj_headers = list(data["trajectory_headers"])
        time = traj[:, 0]
        load_xyz = np.column_stack([
            traj[:, traj_headers.index("load_x")],
            traj[:, traj_headers.index("load_y")],
            traj[:, traj_headers.index("load_z")],
        ])
        load_z = load_xyz[:, 2]
        reference, _ = build_payload_reference(time, 30.0, load_z, FAULT1_TIME)
        error_cm = 100.0 * np.linalg.norm(load_xyz - reference, axis=1)
        traces[m.num_beads] = (time, error_cm)

    if not traces:
        print("  No traces available for figure generation.")
        return sweep_dir / "figures" / "bead_discretization_composite.pdf"

    colors = {
        2: "#1b9e77", 4: "#d95f02", 8: "#7570b3",
        12: "#e7298a", 16: "#66a61e", 24: "#e6ab02",
    }
    fig, axes = plt.subplots(1, 3, figsize=(7.16, 2.2), gridspec_kw={"width_ratios": [3, 1, 1]})

    # Panel (a): Full horizon
    ax_full = axes[0]
    for nb in sorted(traces.keys()):
        t, e = traces[nb]
        ax_full.plot(t, e, linewidth=0.8, color=colors.get(nb, "gray"),
                     label=f"$n_b={nb}$", alpha=0.9)
    for ft in [FAULT1_TIME, FAULT2_TIME]:
        ax_full.axvline(ft, color="red", linewidth=0.5, linestyle="--", alpha=0.6)
    ax_full.set_xlabel("Time [s]")
    ax_full.set_ylabel("Load tracking error [cm]")
    ax_full.set_title("(a) Full horizon")
    ax_full.legend(loc="upper right", framealpha=0.85)
    ax_full.set_xlim(0, 30)

    # Panel (b): Fault 1 zoom
    ax_f1 = axes[1]
    for nb in sorted(traces.keys()):
        t, e = traces[nb]
        mask = (t >= 6.5) & (t <= 10.0)
        ax_f1.plot(t[mask], e[mask], linewidth=1.0, color=colors.get(nb, "gray"), alpha=0.9)
    ax_f1.axvline(FAULT1_TIME, color="red", linewidth=0.5, linestyle="--", alpha=0.6)
    ax_f1.set_xlabel("Time [s]")
    ax_f1.set_title("(b) Fault 1 zoom")
    ax_f1.set_xlim(6.5, 10.0)

    # Panel (c): Fault 2 zoom
    ax_f2 = axes[2]
    for nb in sorted(traces.keys()):
        t, e = traces[nb]
        mask = (t >= 11.5) & (t <= 15.0)
        ax_f2.plot(t[mask], e[mask], linewidth=1.0, color=colors.get(nb, "gray"), alpha=0.9)
    ax_f2.axvline(FAULT2_TIME, color="red", linewidth=0.5, linestyle="--", alpha=0.6)
    ax_f2.set_xlabel("Time [s]")
    ax_f2.set_title("(c) Fault 2 zoom")
    ax_f2.set_xlim(11.5, 15.0)

    fig.tight_layout(pad=0.4)
    figure_dir = sweep_dir / "figures"
    figure_dir.mkdir(parents=True, exist_ok=True)

    pdf_path = figure_dir / "bead_discretization_composite.pdf"
    fig.savefig(pdf_path, dpi=FIGURE_DPI, bbox_inches="tight")
    png_path = figure_dir / "bead_discretization_composite.png"
    fig.savefig(png_path, dpi=FIGURE_DPI, bbox_inches="tight")
    plt.close(fig)
    print(f"  Figure saved: {pdf_path}")

    # Copy to paper Figures directory
    paper_fig_dir = ROOT / "IEEE_T-CST" / "Figures" / "bead_discretization"
    paper_fig_dir.mkdir(parents=True, exist_ok=True)
    shutil.copy2(pdf_path, paper_fig_dir / pdf_path.name)
    shutil.copy2(png_path, paper_fig_dir / png_path.name)
    print(f"  Copied to paper: {paper_fig_dir}")

    return pdf_path


# ── Table generation ──────────────────────────────────────────────────────

def generate_latex_table(metrics_list: list[BeadCountMetrics], output_path: Path) -> None:
    """Generate a LaTeX table for the manuscript."""
    lines = [
        r"\begin{table}[t]",
        r"  \centering",
        r"  \caption{Cable discretization sensitivity ($N{=}5$, cascading-fault scenario).}",
        r"  \label{tab:bead_discretization}",
        r"  \setlength{\tabcolsep}{4pt}",
        r"  \begin{tabular}{r c c c c c c}",
        r"    \toprule",
        r"    $n_b$ & \makecell{Fault\,1\\peak [cm]} & \makecell{Fault\,1\\$t_{\mathrm{rec}}$ [s]}"
        r" & \makecell{Fault\,2\\peak [cm]} & \makecell{Fault\,2\\$t_{\mathrm{rec}}$ [s]}"
        r" & \makecell{Whole-run\\RMSE [cm]} & \makecell{Wall\\time [s]} \\",
        r"    \midrule",
    ]
    for m in sorted(metrics_list, key=lambda x: x.num_beads):
        lines.append(
            f"    {m.num_beads} & {m.fault1.peak_error_cm:.1f} & {m.fault1.recovery_time_s:.2f}"
            f" & {m.fault2.peak_error_cm:.1f} & {m.fault2.recovery_time_s:.2f}"
            f" & {m.whole_run_rmse_cm:.1f} & {m.wall_time_seconds:.0f} \\\\"
        )
    lines += [
        r"    \bottomrule",
        r"  \end{tabular}",
        r"\end{table}",
    ]
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"  Table saved: {output_path}")


# ── CSV summary ───────────────────────────────────────────────────────────

def write_csv_summary(metrics_list: list[BeadCountMetrics], output_path: Path) -> None:
    """Write a flat CSV summary of all metrics."""
    headers = [
        "num_beads", "whole_run_rmse_cm", "final_load_error_cm",
        "peak_load_speed_m_s", "peak_cable_tension_N",
        "fault1_baseline_rmse_cm", "fault1_peak_error_cm",
        "fault1_post_fault_rmse_cm", "fault1_recovery_time_s",
        "fault2_baseline_rmse_cm", "fault2_peak_error_cm",
        "fault2_post_fault_rmse_cm", "fault2_recovery_time_s",
        "wall_time_seconds",
    ]
    rows = []
    for m in sorted(metrics_list, key=lambda x: x.num_beads):
        rows.append(",".join([
            str(m.num_beads),
            f"{m.whole_run_rmse_cm:.3f}", f"{m.final_load_error_cm:.3f}",
            f"{m.peak_load_speed_m_s:.3f}", f"{m.peak_cable_tension_N:.3f}",
            f"{m.fault1.baseline_rmse_cm:.3f}", f"{m.fault1.peak_error_cm:.3f}",
            f"{m.fault1.post_fault_rmse_cm:.3f}", f"{m.fault1.recovery_time_s:.3f}",
            f"{m.fault2.baseline_rmse_cm:.3f}", f"{m.fault2.peak_error_cm:.3f}",
            f"{m.fault2.post_fault_rmse_cm:.3f}", f"{m.fault2.recovery_time_s:.3f}",
            f"{m.wall_time_seconds:.1f}",
        ]))
    output_path.write_text(",".join(headers) + "\n" + "\n".join(rows) + "\n", encoding="utf-8")
    print(f"  CSV saved: {output_path}")


# ── Main ──────────────────────────────────────────────────────────────────

def analyze_sweep(
    sweep_dir: Path,
    paper_subset: list[int] | None = None,
) -> list[BeadCountMetrics]:
    """Analyze all completed runs in the sweep directory."""
    print(f"Analyzing sweep: {sweep_dir}")

    # Find all beads_* directories
    run_dirs = sorted(sweep_dir.glob("beads_*"))
    if not run_dirs:
        print("  No run directories found.")
        return []

    metrics_list: list[BeadCountMetrics] = []
    for run_dir in run_dirs:
        m = compute_metrics_for_run(run_dir)
        if m is not None:
            metrics_list.append(m)
            print(f"  beads={m.num_beads:2d}: whole RMSE={m.whole_run_rmse_cm:.1f} cm, "
                  f"F1 peak={m.fault1.peak_error_cm:.1f} cm, "
                  f"F2 peak={m.fault2.peak_error_cm:.1f} cm")
        else:
            print(f"  {run_dir.name}: no data found, skipping")

    if not metrics_list:
        return []

    # Write outputs
    write_csv_summary(metrics_list, sweep_dir / "bead_sweep_metrics.csv")
    json_summary = [m.to_dict() for m in sorted(metrics_list, key=lambda x: x.num_beads)]
    (sweep_dir / "bead_sweep_metrics.json").write_text(
        json.dumps(json_summary, indent=2) + "\n", encoding="utf-8"
    )

    generate_composite_figure(sweep_dir, metrics_list, paper_subset)

    table_dir = sweep_dir / "tables"
    table_dir.mkdir(parents=True, exist_ok=True)
    generate_latex_table(metrics_list, table_dir / "bead_discretization_table.tex")

    # Print convergence summary
    print("\nConvergence summary:")
    sorted_metrics = sorted(metrics_list, key=lambda x: x.num_beads)
    for i in range(1, len(sorted_metrics)):
        prev, curr = sorted_metrics[i - 1], sorted_metrics[i]
        f1_delta = curr.fault1.peak_error_cm - prev.fault1.peak_error_cm
        f2_delta = curr.fault2.peak_error_cm - prev.fault2.peak_error_cm
        print(f"  {prev.num_beads}→{curr.num_beads} beads: "
              f"ΔF1_peak={f1_delta:+.2f} cm, ΔF2_peak={f2_delta:+.2f} cm")

    return metrics_list


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Analyze bead-count sweep and generate paper artifacts."
    )
    parser.add_argument(
        "--sweep-dir",
        type=str,
        default=str(SWEEP_OUTPUT_ROOT),
        help=f"Sweep output directory (default: {SWEEP_OUTPUT_ROOT})",
    )
    parser.add_argument(
        "--paper-subset",
        type=str,
        default="4,8,16",
        help="Comma-separated bead counts to include in the paper figure (default: 4,8,16)",
    )
    args = parser.parse_args()

    paper_subset = [int(x.strip()) for x in args.paper_subset.split(",")]
    analyze_sweep(Path(args.sweep_dir), paper_subset)


if __name__ == "__main__":
    main()
