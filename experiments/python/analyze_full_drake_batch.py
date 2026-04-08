#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import matplotlib
import numpy as np
from experiment_manifest import DEFAULT_FULL_DRAKE_BATCH_MANIFEST, load_scenarios

matplotlib.use("Agg")
import matplotlib.pyplot as plt


ROOT = Path("/workspaces/Tether_Grace")
DEFAULT_BATCH_DIR = ROOT / "outputs" / "full_drake_fault_batch"
PAYLOAD_MASS = 3.0
GRAVITY = 9.81
PICKUP_THRESHOLD = 0.3
FAULT_DETECT_HOLD_TIME = 0.12
FAULT_DETECT_ARMING_TIME = 0.75
FIGURE_DPI = 220
LIFTOFF_HEIGHT_DELTA = 0.05
LIFTOFF_HOLD_TIME = 0.1

plt.rcParams.update(
    {
        "figure.facecolor": "white",
        "axes.facecolor": "#fbfbfb",
        "axes.edgecolor": "#333333",
        "axes.grid": True,
        "grid.alpha": 0.22,
        "grid.linestyle": "--",
        "axes.titlesize": 14,
        "axes.labelsize": 11,
        "legend.fontsize": 9,
        "xtick.labelsize": 10,
        "ytick.labelsize": 10,
    }
)


@dataclass(frozen=True)
class DetectionStats:
    pickup_time: float | None
    armed_time: float | None
    low_tension_start: float | None
    inferred_detection_time: float | None
    detection_latency: float | None
    threshold: float


def scenario_title(name: str) -> str:
    return name.replace("_", " ")


def save_figure(figure: plt.Figure, path: Path) -> None:
    figure.savefig(path, dpi=FIGURE_DPI, bbox_inches="tight", facecolor="white")
    plt.close(figure)


def new_axis_figure(width: float = 11.5, height: float = 4.8) -> tuple[plt.Figure, plt.Axes]:
    return plt.subplots(1, 1, figsize=(width, height), constrained_layout=True)


def mean_defined(values: list[float | None]) -> float:
    defined = [float(value) for value in values if value is not None]
    return float(np.mean(defined)) if defined else float("nan")


def load_csv_matrix(path: Path) -> tuple[list[str], np.ndarray]:
    data = np.genfromtxt(path, delimiter=",", names=True)
    headers = list(data.dtype.names or [])
    if not headers:
        raise RuntimeError(f"No headers found in {path}")
    if data.ndim == 0:
        data = np.array([tuple(data)], dtype=data.dtype)
    matrix = np.column_stack([data[name] for name in headers])
    return headers, matrix


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
    waypoints: list[tuple[np.ndarray, float, float]], time_value: float
) -> tuple[np.ndarray, np.ndarray]:
    segment_start_time = 0.0
    for index, (position, arrival_time, hold_time) in enumerate(waypoints):
        hold_end_time = arrival_time + hold_time
        if time_value <= arrival_time:
            if index == 0:
                return position.copy(), np.zeros(3, dtype=float)
            previous_position, _, _ = waypoints[index - 1]
            segment_duration = arrival_time - segment_start_time
            if segment_duration > 1e-6:
                alpha = (time_value - segment_start_time) / segment_duration
                return (1.0 - alpha) * previous_position + alpha * position, (position - previous_position) / segment_duration
            return position.copy(), np.zeros(3, dtype=float)
        if time_value <= hold_end_time:
            return position.copy(), np.zeros(3, dtype=float)
        segment_start_time = hold_end_time
    return waypoints[-1][0].copy(), np.zeros(3, dtype=float)


def build_load_reference(time_values: np.ndarray, duration: float) -> np.ndarray:
    waypoints = build_waypoints(initial_altitude=1.2, duration=duration)
    reference = np.zeros((time_values.shape[0], 3), dtype=float)
    for index, time_value in enumerate(time_values):
        reference[index], _ = evaluate_waypoint_trajectory(waypoints, float(time_value))
    return reference


def infer_payload_z_offset(
    time_values: np.ndarray,
    base_reference_z: np.ndarray,
    actual_load_z: np.ndarray,
    first_fault_time: float,
) -> float:
    window_start = min(2.0, max(float(time_values[0]), float(first_fault_time) * 0.25))
    window_end = max(window_start + 0.5, float(first_fault_time) - 0.5)
    mask = (time_values >= window_start) & (time_values <= window_end)
    if not np.any(mask):
        fallback_index = min(len(time_values) - 1, 200)
        mask = time_values <= max(float(first_fault_time) - 0.5, float(time_values[fallback_index]))
    if not np.any(mask):
        return float(np.median(base_reference_z - actual_load_z))
    return float(np.median(base_reference_z[mask] - actual_load_z[mask]))


def build_payload_reference(
    time_values: np.ndarray,
    duration: float,
    actual_load_z: np.ndarray,
    first_fault_time: float,
) -> tuple[np.ndarray, float, float]:
    reference = build_load_reference(time_values, duration)
    z_offset = infer_payload_z_offset(time_values, reference[:, 2], actual_load_z, first_fault_time)
    reference[:, 2] -= z_offset
    liftoff_time = infer_payload_liftoff_time(time_values, actual_load_z)
    reference[time_values < liftoff_time, 2] = float(actual_load_z[0])
    return reference, z_offset, liftoff_time


def infer_payload_liftoff_time(time_values: np.ndarray, actual_load_z: np.ndarray) -> float:
    initial_height = float(actual_load_z[0])
    threshold = initial_height + LIFTOFF_HEIGHT_DELTA
    return find_hold_start(time_values, -actual_load_z, -threshold, float(time_values[0]), LIFTOFF_HOLD_TIME) or float(time_values[0])


def find_hold_start(times: np.ndarray, values: np.ndarray, threshold: float, start_time: float, hold_time: float) -> float | None:
    start_index = int(np.searchsorted(times, start_time, side="left"))
    if start_index >= len(times):
                return None
    for index in range(start_index, len(times)):
        if values[index] > threshold:
            continue
        end_time = times[index] + hold_time
        end_index = int(np.searchsorted(times, end_time, side="left"))
        if end_index >= len(times):
            end_index = len(times) - 1
        if np.all(values[index : end_index + 1] <= threshold):
            return float(times[index])
    return None


def active_fault_set(fault_cables: list[int], fault_times: list[float], sample_time: float) -> set[int]:
    return {
        int(faulted_drone)
        for faulted_drone, fault_time in zip(fault_cables, fault_times)
        if float(fault_time) <= sample_time + 1e-9
    }


def nearest_distance_to_healthy(
    traj_headers: list[str], traj: np.ndarray, faulted_drone: int, excluded_drones: set[int], sample_time: float
) -> float | None:
    time = traj[:, 0]
    sample_index = min(int(np.searchsorted(time, sample_time, side="left")), len(time) - 1)
    faulted = traj[sample_index, [
        traj_headers.index(f"drone{faulted_drone}_x"),
        traj_headers.index(f"drone{faulted_drone}_y"),
        traj_headers.index(f"drone{faulted_drone}_z"),
    ]]
    min_distance = float("inf")
    for drone_index in range(sum(1 for header in traj_headers if header.startswith("drone") and header.endswith("_x"))):
        if drone_index == faulted_drone or drone_index in excluded_drones:
            continue
        candidate = traj[sample_index, [
            traj_headers.index(f"drone{drone_index}_x"),
            traj_headers.index(f"drone{drone_index}_y"),
            traj_headers.index(f"drone{drone_index}_z"),
        ]]
        min_distance = min(min_distance, float(np.linalg.norm(candidate - faulted)))
    if not np.isfinite(min_distance):
        return None
    return min_distance


def infer_detection_stats(time: np.ndarray, rope_tension: np.ndarray, fault_time: float, num_quadcopters: int) -> DetectionStats:
    pickup_indices = np.flatnonzero(rope_tension >= PICKUP_THRESHOLD)
    pickup_time = float(time[pickup_indices[0]]) if pickup_indices.size else None
    armed_time = pickup_time + FAULT_DETECT_ARMING_TIME if pickup_time is not None else None
    threshold = max(0.15 * ((PAYLOAD_MASS * GRAVITY) / num_quadcopters), 0.15)
    search_start = max(fault_time, armed_time if armed_time is not None else fault_time)
    low_tension_start = find_hold_start(time, rope_tension, threshold, search_start, FAULT_DETECT_HOLD_TIME)
    detection_time = None if low_tension_start is None else low_tension_start + FAULT_DETECT_HOLD_TIME
    latency = None if detection_time is None else detection_time - fault_time
    return DetectionStats(
        pickup_time=pickup_time,
        armed_time=armed_time,
        low_tension_start=low_tension_start,
        inferred_detection_time=detection_time,
        detection_latency=latency,
        threshold=threshold,
    )


def analyze_scenario(batch_dir: Path, scenario_entry: dict[str, object]) -> tuple[dict[str, object], dict[str, Any]]:
    scenario_dir = batch_dir / str(scenario_entry["scenario"])
    log_dir = Path(str(scenario_entry["log_dir"]))
    run_manifest = json.loads((scenario_dir / "run_manifest.json").read_text(encoding="utf-8"))
    traj_headers, traj = load_csv_matrix(log_dir / "trajectories.csv")
    tension_headers, tension = load_csv_matrix(log_dir / "tensions.csv")

    time = traj[:, 0]
    duration = float(scenario_entry["duration"])
    load_actual = traj[:, [traj_headers.index("load_x"), traj_headers.index("load_y"), traj_headers.index("load_z")]]
    first_fault_time = min(float(value) for value in scenario_entry["fault_times"])
    load_reference, payload_z_offset, payload_liftoff_time = build_payload_reference(time, duration, load_actual[:, 2], first_fault_time)
    load_error = np.linalg.norm(load_actual - load_reference, axis=1)

    fault_events: list[dict[str, object]] = []
    for faulted_drone, fault_time in zip(scenario_entry["fault_cables"], scenario_entry["fault_times"]):
        rope_column = tension_headers.index(f"rope{faulted_drone}_mag")
        rope_tension = tension[:, rope_column]
        detection = infer_detection_stats(time, rope_tension, float(fault_time), int(scenario_entry["num_agents"]))
        excluded_at_fault = active_fault_set(scenario_entry["fault_cables"], scenario_entry["fault_times"], float(fault_time))
        excluded_at_2s = active_fault_set(scenario_entry["fault_cables"], scenario_entry["fault_times"], float(fault_time) + 2.0)
        excluded_final = active_fault_set(scenario_entry["fault_cables"], scenario_entry["fault_times"], float(time[-1]))
        separation_at_fault = nearest_distance_to_healthy(
            traj_headers, traj, int(faulted_drone), excluded_at_fault, float(fault_time)
        )
        separation_2s = nearest_distance_to_healthy(
            traj_headers, traj, int(faulted_drone), excluded_at_2s, float(fault_time) + 2.0
        )
        separation_final = nearest_distance_to_healthy(
            traj_headers, traj, int(faulted_drone), excluded_final, float(time[-1])
        )
        fault_events.append(
            {
                "faulted_drone": int(faulted_drone),
                "fault_time": float(fault_time),
                "pickup_time": detection.pickup_time,
                "armed_time": detection.armed_time,
                "low_tension_start": detection.low_tension_start,
                "inferred_detection_time": detection.inferred_detection_time,
                "detection_latency_s": detection.detection_latency,
                "tension_threshold_n": detection.threshold,
                "nearest_separation_at_fault_m": separation_at_fault,
                "nearest_separation_2s_m": separation_2s,
                "nearest_separation_final_m": separation_final,
                "nearest_healthy_separation_at_fault_m": separation_at_fault,
                "nearest_healthy_separation_2s_m": separation_2s,
                "nearest_healthy_separation_final_m": separation_final,
            }
        )

    summary = {
        "scenario": scenario_entry["scenario"],
        "num_agents": int(scenario_entry["num_agents"]),
        "duration": duration,
        "fault_cables": scenario_entry["fault_cables"],
        "fault_times": scenario_entry["fault_times"],
        "meshcat_html": scenario_entry["meshcat_html"],
        "artifacts": scenario_entry.get("artifacts", {}),
        "inferred_payload_z_offset_m": payload_z_offset,
        "inferred_payload_liftoff_time_s": payload_liftoff_time,
        "run_manifest": run_manifest,
        "load_rmse_m": float(np.sqrt(np.mean(load_error**2))),
        "load_peak_error_m": float(np.max(load_error)),
        "post_first_fault_peak_error_m": float(np.max(load_error[time >= first_fault_time])),
        "final_load_error_m": float(load_error[-1]),
        "load_peak_speed_mps": float(scenario_entry["load_peak_speed"]),
        "fault_events": fault_events,
    }
    plot_bundle = {
        "scenario": scenario_entry["scenario"],
        "num_agents": int(scenario_entry["num_agents"]),
        "time": time,
        "load_error_cm": 100.0 * load_error,
        "fault_times": [float(value) for value in scenario_entry["fault_times"]],
    }
    return summary, plot_bundle


def write_markdown_report(output_dir: Path, scenario_summaries: list[dict[str, object]]) -> None:
    lines = [
        "# Full Drake Batch Comparison Report",
        "",
        "This report is generated only from the full-Drake batch artifacts in outputs/full_drake_fault_batch.",
        "",
        "## Presentation Figures",
        "",
        "- comparison_load_tracking_summary.png: batch-level load-tracking summary.",
        "- comparison_fault_response_timing.png: mean detection latency and final separation summary.",
        "- comparison_healthy_neighbor_clearance.png: batch healthy-neighbor clearance summary.",
        "- comparison_load_speed_envelope.png: batch load-speed summary.",
        "- comparison_load_error_overlay.png: load-tracking error overlay for the full 3/5/7 schedule set.",
        "- comparison_fault_event_separation.png: per-fault healthy-neighbor separation at fault, +2 s, and end of run.",
        "",
        "## Scenario Summary",
        "",
        "| Scenario | Quads | Faults | Load RMSE [cm] | Post-first-fault peak [cm] | Final load error [cm] | Peak load speed [m/s] |",
        "| --- | ---: | --- | ---: | ---: | ---: | ---: |",
    ]
    for summary in scenario_summaries:
        lines.append(
            f"| {summary['scenario']} | {summary['num_agents']} | {summary['fault_cables']} @ {summary['fault_times']} | "
            f"{summary['load_rmse_m'] * 100.0:.2f} | {summary['post_first_fault_peak_error_m'] * 100.0:.2f} | "
            f"{summary['final_load_error_m'] * 100.0:.2f} | {summary['load_peak_speed_mps']:.2f} |"
        )

    lines.extend([
        "",
        "## Fault Detection And Separation",
        "",
        "| Scenario | Faulted drone | Fault time [s] | Detection latency [s] | Healthy separation at fault [m] | Healthy separation +2 s [m] | Healthy final separation [m] |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: |",
    ])
    for summary in scenario_summaries:
        for event in summary["fault_events"]:
            latency = event["detection_latency_s"]
            latency_text = f"{latency:.2f}" if latency is not None else "n/a"
            sep_fault = event["nearest_healthy_separation_at_fault_m"]
            sep_2s = event["nearest_healthy_separation_2s_m"]
            sep_final = event["nearest_healthy_separation_final_m"]
            lines.append(
                f"| {summary['scenario']} | {event['faulted_drone']} | {event['fault_time']:.2f} | "
                f"{latency_text} | {sep_fault:.2f} | {sep_2s:.2f} | {sep_final:.2f} |"
            )

    lines.extend([
        "",
        "## Notes",
        "",
        f"- Pickup trigger threshold is inferred from the controller seam at {PICKUP_THRESHOLD:.2f} N.",
        f"- Detection hold time is {FAULT_DETECT_HOLD_TIME:.2f} s and the post-pickup arming delay is {FAULT_DETECT_ARMING_TIME:.2f} s.",
        "- Detection time here is inferred from tension traces, using the same threshold formula the runner applies to each controller.",
        "- Separation metrics exclude drones whose own cables have already faulted at the sampled time.",
        "- Scenario-level figures are stored under each scenario folder in the figures/ subdirectory.",
    ])
    (output_dir / "comparison_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_batch_readme(output_dir: Path, scenario_summaries: list[dict[str, object]]) -> None:
    lines = [
        "# Full Drake Fault Batch",
        "",
        "This directory contains only regenerated full-Drake multibody outputs for the 3-, 5-, and 7-quadcopter fault schedules.",
        "",
        "## Batch artifacts",
        "",
        "- batch_manifest.json",
        "- comparison_metrics.json",
        "- comparison_report.md",
        "- comparison_scenarios.csv",
        "- comparison_fault_events.csv",
        "- comparison_load_tracking_summary.png",
        "- comparison_fault_response_timing.png",
        "- comparison_healthy_neighbor_clearance.png",
        "- comparison_load_speed_envelope.png",
        "- comparison_load_error_overlay.png",
        "- comparison_fault_event_separation.png",
        "",
        "## Scenario folders",
        "",
    ]
    for summary in scenario_summaries:
        artifacts = summary.get("artifacts", {})
        figure_paths = artifacts.get("figure_paths", {}) if isinstance(artifacts, dict) else {}
        lines.extend(
            [
                f"- {summary['scenario']}",
                f"  - Meshcat replay: {artifacts.get('meshcat_replay_html', summary['meshcat_html'])}",
                f"  - Timeseries archive: {artifacts.get('full_drake_recording_npz', '')}",
                f"  - Figures: {', '.join(sorted(Path(path).name for path in figure_paths.values()))}",
            ]
        )
    (output_dir / "README.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_csv_summaries(output_dir: Path, scenario_summaries: list[dict[str, object]]) -> None:
    scenario_lines = [
        "scenario,num_agents,num_faults,load_rmse_m,post_first_fault_peak_error_m,final_load_error_m,load_peak_speed_mps"
    ]
    event_lines = [
        "scenario,faulted_drone,fault_time_s,detection_latency_s,nearest_healthy_separation_at_fault_m,nearest_healthy_separation_2s_m,nearest_healthy_separation_final_m"
    ]

    for summary in scenario_summaries:
        scenario_lines.append(
            ",".join(
                [
                    str(summary["scenario"]),
                    str(summary["num_agents"]),
                    str(len(summary["fault_events"])),
                    f"{summary['load_rmse_m']:.9f}",
                    f"{summary['post_first_fault_peak_error_m']:.9f}",
                    f"{summary['final_load_error_m']:.9f}",
                    f"{summary['load_peak_speed_mps']:.9f}",
                ]
            )
        )
        for event in summary["fault_events"]:
            event_lines.append(
                ",".join(
                    [
                        str(summary["scenario"]),
                        str(event["faulted_drone"]),
                        f"{event['fault_time']:.9f}",
                        "" if event["detection_latency_s"] is None else f"{event['detection_latency_s']:.9f}",
                        "" if event["nearest_healthy_separation_at_fault_m"] is None else f"{event['nearest_healthy_separation_at_fault_m']:.9f}",
                        "" if event["nearest_healthy_separation_2s_m"] is None else f"{event['nearest_healthy_separation_2s_m']:.9f}",
                        "" if event["nearest_healthy_separation_final_m"] is None else f"{event['nearest_healthy_separation_final_m']:.9f}",
                    ]
                )
            )

    (output_dir / "comparison_scenarios.csv").write_text("\n".join(scenario_lines) + "\n", encoding="utf-8")
    (output_dir / "comparison_fault_events.csv").write_text("\n".join(event_lines) + "\n", encoding="utf-8")


def save_comparison_plot(output_dir: Path, scenario_summaries: list[dict[str, object]]) -> None:
    labels = [str(summary["num_agents"]) for summary in scenario_summaries]
    rmse_cm = [summary["load_rmse_m"] * 100.0 for summary in scenario_summaries]
    peak_cm = [summary["post_first_fault_peak_error_m"] * 100.0 for summary in scenario_summaries]
    final_error_cm = [summary["final_load_error_m"] * 100.0 for summary in scenario_summaries]
    mean_latency = [mean_defined([event["detection_latency_s"] for event in summary["fault_events"]]) for summary in scenario_summaries]
    mean_sep_2s = [
        mean_defined([event["nearest_healthy_separation_2s_m"] for event in summary["fault_events"]])
        for summary in scenario_summaries
    ]
    mean_final_sep = [
        mean_defined([event["nearest_healthy_separation_final_m"] for event in summary["fault_events"]])
        for summary in scenario_summaries
    ]
    min_final_sep = [
        min(float(event["nearest_healthy_separation_final_m"]) for event in summary["fault_events"] if event["nearest_healthy_separation_final_m"] is not None)
        for summary in scenario_summaries
    ]
    peak_speed = [summary["load_peak_speed_mps"] for summary in scenario_summaries]

    x = np.arange(len(labels))
    figure, axis = new_axis_figure()
    axis.bar(x - 0.24, rmse_cm, width=0.24, color="#1f77b4", label="Load RMSE")
    axis.bar(x, peak_cm, width=0.24, color="#ff7f0e", label="Post-fault peak")
    axis.bar(x + 0.24, final_error_cm, width=0.24, color="#2ca02c", label="Final error")
    axis.set_xticks(x)
    axis.set_xticklabels(labels)
    axis.set_xlabel("Quadcopters")
    axis.set_ylabel("Load error [cm]")
    axis.set_title("Load tracking across batch")
    axis.legend(loc="upper right")
    save_figure(figure, output_dir / "comparison_load_tracking_summary.png")

    figure, axis = new_axis_figure()
    axis.plot(x, mean_latency, marker="o", color="#1f77b4", linewidth=2.2, label="Mean detection latency")
    axis.plot(x, mean_final_sep, marker="s", color="#d62728", linewidth=2.2, label="Mean final separation")
    axis.set_xticks(x)
    axis.set_xticklabels(labels)
    axis.set_xlabel("Quadcopters")
    axis.set_title("Fault response timing")
    axis.legend(loc="upper right")
    save_figure(figure, output_dir / "comparison_fault_response_timing.png")

    figure, axis = new_axis_figure()
    axis.bar(x - 0.18, mean_sep_2s, width=0.36, color="#9467bd", label="Mean separation +2 s")
    axis.bar(x + 0.18, min_final_sep, width=0.36, color="#8c564b", label="Minimum final separation")
    axis.set_xticks(x)
    axis.set_xticklabels(labels)
    axis.set_xlabel("Quadcopters")
    axis.set_ylabel("Distance [m]")
    axis.set_title("Healthy-neighbor clearance")
    axis.legend(loc="upper right")
    save_figure(figure, output_dir / "comparison_healthy_neighbor_clearance.png")

    figure, axis = new_axis_figure()
    axis.plot(x, peak_speed, marker="D", color="#17a589", linewidth=2.2, label="Peak load speed")
    axis.set_xticks(x)
    axis.set_xticklabels(labels)
    axis.set_xlabel("Quadcopters")
    axis.set_ylabel("Speed [m/s]")
    axis.set_title("Load speed envelope")
    axis.legend(loc="upper right")
    save_figure(figure, output_dir / "comparison_load_speed_envelope.png")


def save_load_error_overlay(output_dir: Path, plot_bundles: list[dict[str, Any]]) -> None:
    figure, axis = plt.subplots(1, 1, figsize=(13.8, 5.8), constrained_layout=True)
    colors = {3: "#1f77b4", 5: "#ff7f0e", 7: "#2ca02c"}
    for bundle in plot_bundles:
        color = colors.get(int(bundle["num_agents"]), "#444444")
        axis.plot(
            bundle["time"],
            bundle["load_error_cm"],
            color=color,
            linewidth=2.0,
            label=f"{bundle['scenario']}",
        )
        for fault_time in bundle["fault_times"]:
            axis.axvline(float(fault_time), color=color, linestyle="--", linewidth=1.0, alpha=0.6)
    axis.set_title("Load tracking error across 3, 5, and 7 drone fault schedules")
    axis.set_xlabel("Time [s]")
    axis.set_ylabel("Payload error [cm]")
    axis.legend(loc="upper right")
    save_figure(figure, output_dir / "comparison_load_error_overlay.png")


def save_fault_event_separation_plot(output_dir: Path, scenario_summaries: list[dict[str, object]]) -> None:
    labels: list[str] = []
    separation_at_fault: list[float] = []
    separation_2s: list[float] = []
    separation_final: list[float] = []

    for summary in scenario_summaries:
        for event in summary["fault_events"]:
            labels.append(f"{summary['num_agents']}Q-d{event['faulted_drone']}")
            separation_at_fault.append(float(event["nearest_healthy_separation_at_fault_m"]))
            separation_2s.append(float(event["nearest_healthy_separation_2s_m"]))
            separation_final.append(float(event["nearest_healthy_separation_final_m"]))

    x = np.arange(len(labels))
    figure, axis = plt.subplots(1, 1, figsize=(14.0, 6.0), constrained_layout=True)
    axis.plot(x, separation_at_fault, marker="o", linewidth=1.8, color="#7f7f7f", label="At fault")
    axis.plot(x, separation_2s, marker="s", linewidth=1.8, color="#9467bd", label="+2 s")
    axis.plot(x, separation_final, marker="D", linewidth=1.8, color="#d62728", label="Final")
    axis.set_xticks(x)
    axis.set_xticklabels(labels, rotation=20)
    axis.set_ylabel("Healthy-neighbor distance [m]")
    axis.set_xlabel("Fault event")
    axis.set_title("Healthy-neighbor clearance by fault event")
    axis.legend(loc="upper right")
    save_figure(figure, output_dir / "comparison_fault_event_separation.png")


def summarize_manifest_alignment(batch_manifest: dict[str, Any], manifest_path: Path | None) -> dict[str, Any]:
    if manifest_path is None:
        return {"manifest_path": "", "matched": True, "missing_scenarios": [], "unexpected_scenarios": []}

    expected = load_scenarios(manifest_path)
    expected_names = {scenario.name for scenario in expected}
    actual_names = {str(scenario["scenario"]) for scenario in batch_manifest["scenarios"]}
    missing = sorted(expected_names - actual_names)
    unexpected = sorted(actual_names - expected_names)
    return {
        "manifest_path": str(manifest_path),
        "matched": not missing and not unexpected,
        "missing_scenarios": missing,
        "unexpected_scenarios": unexpected,
    }


def regenerate_with_shared_axes(
    batch_dir: Path,
    batch_manifest: dict[str, Any],
    paper_figures_dir: Path,
) -> None:
    """Regenerate per-scenario tracking-error and cable-tension plots with shared y-axis limits."""
    SCENARIO_DIR_MAP = {3: "three_drones", 5: "five_drones", 7: "seven_drones"}
    scenario_data: list[dict[str, Any]] = []
    for entry in batch_manifest["scenarios"]:
        scenario_dir = batch_dir / str(entry["scenario"])
        log_dir = Path(str(entry["log_dir"]))
        traj_headers, traj = load_csv_matrix(log_dir / "trajectories.csv")
        tension_headers, tension = load_csv_matrix(log_dir / "tensions.csv")
        time = traj[:, 0]
        load_xyz = traj[:, [traj_headers.index("load_x"), traj_headers.index("load_y"), traj_headers.index("load_z")]]
        first_fault_time = min(float(v) for v in entry["fault_times"])
        load_ref, _, _ = build_payload_reference(time, float(entry["duration"]), load_xyz[:, 2], first_fault_time)
        load_error_cm = 100.0 * np.linalg.norm(load_xyz - load_ref, axis=1)
        num_agents = int(entry["num_agents"])
        tension_data = {}
        for agent in range(num_agents):
            col = f"rope{agent}_mag"
            if col in tension_headers:
                tension_data[agent] = tension[:, tension_headers.index(col)]
        scenario_data.append({
            "name": str(entry["scenario"]),
            "num_agents": num_agents,
            "time": time,
            "load_error_cm": load_error_cm,
            "tension_data": tension_data,
            "fault_times": [float(v) for v in entry["fault_times"]],
        })

    # Compute shared ranges
    all_error = np.concatenate([d["load_error_cm"] for d in scenario_data])
    error_ylim = (0.0, float(np.ceil(np.max(all_error) / 10.0) * 10.0))
    all_tension = np.concatenate([np.concatenate(list(d["tension_data"].values())) for d in scenario_data])
    tension_ylim = (max(0.0, float(np.floor(np.min(all_tension)))), float(np.ceil(np.max(all_tension))))

    for d in scenario_data:
        dir_name = SCENARIO_DIR_MAP.get(d["num_agents"])
        if dir_name is None:
            continue
        fig_dir = paper_figures_dir / dir_name
        fig_dir.mkdir(parents=True, exist_ok=True)

        # Tracking error
        fig, ax = new_axis_figure()
        ax.plot(d["time"], d["load_error_cm"], color="#d62728", linewidth=2.0, label="tracking error")
        for ft in d["fault_times"]:
            ax.axvline(ft, color="#444444", linestyle=":", linewidth=1.2, alpha=0.7)
        ax.set_ylim(*error_ylim)
        ax.set_xlabel("Time [s]")
        ax.set_ylabel("Error [cm]")
        ax.set_title(f"{scenario_title(d['name'])} payload tracking error")
        ax.legend(loc="upper right")
        save_figure(fig, fig_dir / "payload_tracking_error.png")

        # Cable tensions
        fig, ax = new_axis_figure(height=5.2)
        colors = [plt.get_cmap("tab10")(a % 10) for a in range(d["num_agents"])]
        for agent, tension_vals in d["tension_data"].items():
            ax.plot(d["time"], tension_vals, color=colors[agent], linewidth=1.5, label=f"Cable {agent}")
        for ft in d["fault_times"]:
            ax.axvline(ft, color="#444444", linestyle=":", linewidth=1.2, alpha=0.7)
        ax.set_ylim(*tension_ylim)
        ax.set_xlabel("Time [s]")
        ax.set_ylabel("Tension [N]")
        ax.set_title(f"{scenario_title(d['name'])} cable tensions")
        ax.legend(loc="upper right", ncol=min(4, d["num_agents"]))
        save_figure(fig, fig_dir / "cable_tensions.png")

    print(f"Shared-axis figures regenerated: error y-lim={error_ylim}, tension y-lim={tension_ylim}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Analyze full-Drake batch outputs and generate a comparison report")
    parser.add_argument("--batch-dir", type=Path, default=DEFAULT_BATCH_DIR)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_FULL_DRAKE_BATCH_MANIFEST)
    parser.add_argument("--shared-axes", action="store_true", help="Regenerate per-scenario paper figures with shared y-axis limits")
    parser.add_argument("--paper-figures-dir", type=Path, default=ROOT / "IEEE_T-CST" / "Figures" / "full_drake")
    args = parser.parse_args()

    batch_manifest_path = args.batch_dir / "batch_manifest.json"
    batch_manifest = json.loads(batch_manifest_path.read_text(encoding="utf-8"))
    analysis_results = [analyze_scenario(args.batch_dir, scenario) for scenario in batch_manifest["scenarios"]]
    scenario_summaries = [summary for summary, _ in analysis_results]
    plot_bundles = [plot_bundle for _, plot_bundle in analysis_results]
    scenario_summaries.sort(key=lambda item: item["num_agents"])
    plot_bundles.sort(key=lambda item: item["num_agents"])

    payload = {
        "generator": "experiments/python/analyze_full_drake_batch.py",
        "batch_dir": str(args.batch_dir),
        "manifest_alignment": summarize_manifest_alignment(batch_manifest, args.manifest),
        "scenario_summaries": scenario_summaries,
    }
    (args.batch_dir / "comparison_metrics.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")
    write_markdown_report(args.batch_dir, scenario_summaries)
    write_csv_summaries(args.batch_dir, scenario_summaries)
    write_batch_readme(args.batch_dir, scenario_summaries)
    save_comparison_plot(args.batch_dir, scenario_summaries)
    save_load_error_overlay(args.batch_dir, plot_bundles)
    save_fault_event_separation_plot(args.batch_dir, scenario_summaries)
    print(json.dumps(payload, indent=2))

    if args.shared_axes:
        regenerate_with_shared_axes(args.batch_dir, batch_manifest, args.paper_figures_dir)


if __name__ == "__main__":
    main()