#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib
import numpy as np
from experiment_manifest import ExperimentScenario, load_named_scenario

from topology_invariance_root import compute_fault_metrics, simulate
from topology_invariance_root import Params

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def parse_int_values(raw: str) -> list[int]:
    return [int(part.strip()) for part in raw.split(",") if part.strip()]


def resolve_scenario(args: argparse.Namespace) -> ExperimentScenario | None:
    if not args.scenario:
        return None
    return load_named_scenario(args.scenario, args.manifest)


def aggregate_rows(rows: list[dict[str, float]], mode: str) -> dict[str, float]:
    mode_rows = [row for row in rows if row["mode"] == mode]
    if not mode_rows:
        return {
            "mode": mode,
            "mean_post_fault_rmse": 0.0,
            "max_post_fault_rmse": 0.0,
            "mean_peak_deviation": 0.0,
            "mean_recovery_time": 0.0,
            "mean_rmse_ratio": 0.0,
            "cable_spread": 0.0,
        }

    cable_means: dict[int, float] = {}
    for cable in sorted({int(row["fault_cable"]) for row in mode_rows}):
        cable_rows = [row for row in mode_rows if int(row["fault_cable"]) == cable]
        cable_means[cable] = float(np.mean([row["post_fault_rmse"] for row in cable_rows]))

    return {
        "mode": mode,
        "mean_post_fault_rmse": float(np.mean([row["post_fault_rmse"] for row in mode_rows])),
        "max_post_fault_rmse": float(np.max([row["post_fault_rmse"] for row in mode_rows])),
        "mean_fault_delta": float(np.mean([row["fault_delta_rmse"] for row in mode_rows])),
        "max_fault_delta": float(np.max([row["fault_delta_rmse"] for row in mode_rows])),
        "mean_peak_deviation": float(np.mean([row["peak_deviation"] for row in mode_rows])),
        "mean_recovery_time": float(np.mean([row["recovery_time"] for row in mode_rows])),
        "mean_rmse_ratio": float(np.mean([row["rmse_ratio"] for row in mode_rows])),
        "cable_spread": float(max(cable_means.values()) - min(cable_means.values())) if len(cable_means) > 1 else 0.0,
    }


def aggregate_by_cable(rows: list[dict[str, float]], mode: str) -> list[dict[str, float]]:
    mode_rows = [row for row in rows if row["mode"] == mode]
    result: list[dict[str, float]] = []
    for cable in sorted({int(row["fault_cable"]) for row in mode_rows}):
        cable_rows = [row for row in mode_rows if int(row["fault_cable"]) == cable]
        result.append(
            {
                "mode": mode,
                "fault_cable": float(cable),
                "mean_post_fault_rmse": float(np.mean([row["post_fault_rmse"] for row in cable_rows])),
                "max_post_fault_rmse": float(np.max([row["post_fault_rmse"] for row in cable_rows])),
                "mean_fault_delta": float(np.mean([row["fault_delta_rmse"] for row in cable_rows])),
                "mean_peak_deviation": float(np.mean([row["peak_deviation"] for row in cable_rows])),
                "mean_recovery_time": float(np.mean([row["recovery_time"] for row in cable_rows])),
            }
        )
    return result


def write_csv(rows: list[dict[str, float]], path: Path) -> None:
    fieldnames = [
        "mode",
        "seed",
        "fault_cable",
        "nominal_rmse",
        "post_fault_rmse",
        "pre_fault_rmse",
        "fault_delta_rmse",
        "rmse_ratio",
        "peak_deviation",
        "recovery_time",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def save_summary_figure(output_dir: Path, rows: list[dict[str, float]], aggregates: list[dict[str, float]]) -> None:
    figure, axes = plt.subplots(1, 2, figsize=(13, 5.5))

    modes = [aggregate["mode"] for aggregate in aggregates]
    mean_post = [aggregate["mean_post_fault_rmse"] * 100.0 for aggregate in aggregates]
    mean_peak = [aggregate["mean_peak_deviation"] * 100.0 for aggregate in aggregates]
    spread = [aggregate["cable_spread"] * 100.0 for aggregate in aggregates]

    positions = np.arange(len(modes))
    axes[0].bar(positions - 0.15, mean_post, width=0.3, label="Mean post-fault RMSE")
    axes[0].bar(positions + 0.15, mean_peak, width=0.3, label="Mean peak deviation")
    axes[0].set_xticks(positions)
    axes[0].set_xticklabels([mode.upper() for mode in modes])
    axes[0].set_ylabel("Error [cm]")
    axes[0].set_title("Aggregate Fault Response")
    axes[0].legend()
    axes[0].grid(True, axis="y", alpha=0.3)

    axes[1].bar(positions, spread, width=0.45, color=["#4a90d9", "#5cb85c"][: len(modes)])
    axes[1].set_xticks(positions)
    axes[1].set_xticklabels([mode.upper() for mode in modes])
    axes[1].set_ylabel("Post-fault RMSE spread [cm]")
    axes[1].set_title("Topology-Invariance Spread Across Faulted Cables")
    axes[1].grid(True, axis="y", alpha=0.3)

    figure.tight_layout()
    figure.savefig(output_dir / "fault_matrix_summary.png", dpi=150, bbox_inches="tight")
    plt.close(figure)


def write_markdown_report(
    output_dir: Path,
    aggregates: list[dict[str, float]],
    per_cable: list[dict[str, float]],
    config: dict[str, object],
) -> None:
    lines = [
        "# Fault Matrix Report",
        "",
        "## Configuration",
        "",
        f"- Duration: {config['duration']} s",
        f"- Fault time: {config['fault_time']} s",
        f"- Seeds: {config['seeds']}",
        f"- Faulted cables: {config['fault_cables']}",
        f"- Wind sigma: {config['wind_sigma']}",
        f"- L1 config: {config['l1_config']}",
        "",
        "## Aggregate Summary",
        "",
        "| Mode | Mean post-fault RMSE [cm] | Mean fault delta [cm] | Mean peak [cm] | Spread [cm] |",
        "| --- | ---: | ---: | ---: | ---: |",
    ]

    for aggregate in aggregates:
        lines.append(
            f"| {aggregate['mode'].upper()} | {aggregate['mean_post_fault_rmse'] * 100.0:.2f} | "
            f"{aggregate['mean_fault_delta'] * 100.0:.2f} | {aggregate['mean_peak_deviation'] * 100.0:.2f} | "
            f"{aggregate['cable_spread'] * 100.0:.2f} |"
        )

    lines.extend([
        "",
        "## Per-Cable Summary",
        "",
        "| Mode | Cable | Mean post-fault RMSE [cm] | Mean fault delta [cm] | Mean peak [cm] | Mean recovery [s] |",
        "| --- | ---: | ---: | ---: | ---: | ---: |",
    ])
    for row in per_cable:
        lines.append(
            f"| {row['mode'].upper()} | {int(row['fault_cable'])} | {row['mean_post_fault_rmse'] * 100.0:.2f} | "
            f"{row['mean_fault_delta'] * 100.0:.2f} | {row['mean_peak_deviation'] * 100.0:.2f} | "
            f"{row['mean_recovery_time']:.2f} |"
        )

    (output_dir / "fault_matrix_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description="Run a root-owned CL-vs-L1 fault matrix across seeds and faulted cables")
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--fault-time", type=float, default=4.0)
    parser.add_argument("--seeds", type=str, default="42,43,44")
    parser.add_argument("--fault-cables", type=str, default="0,1,2")
    parser.add_argument("--scenario", type=str, default="", help="Named shared-manifest scenario to reuse")
    parser.add_argument("--manifest", type=Path, default=None, help="Optional shared scenario manifest path")
    parser.add_argument("--wind-sigma", type=float, default=0.5)
    parser.add_argument("--l1-omega", type=float, default=10.0)
    parser.add_argument("--l1-predictor-pole", type=float, default=30.0)
    parser.add_argument("--l1-sigma-max", type=float, default=20.0)
    parser.add_argument("--l1-shared-blend", type=float, default=0.0)
    parser.add_argument("--l1-shared-omega", type=float, default=10.0)
    parser.add_argument("--l1-shared-predictor-pole", type=float, default=20.0)
    parser.add_argument("--l1-shared-sigma-max", type=float, default=10.0)
    parser.add_argument("--l1-shared-ramp-time", type=float, default=0.2)
    parser.add_argument("--l1-shared-allocator", type=str, default="equal")
    parser.add_argument("--l1-shared-load-kp-scale", type=float, default=0.5)
    parser.add_argument("--l1-shared-load-kv-scale", type=float, default=0.5)
    parser.add_argument("--l1-local-lateral-weight", type=float, default=1.0)
    parser.add_argument("--l1-local-projection", type=str, default="uniform_xy")
    parser.add_argument("--l1-desired-geometry-mode", type=str, default="off")
    parser.add_argument("--l1-desired-geometry-ramp-time", type=float, default=0.5)
    parser.add_argument("--skip-plots", action="store_true")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("/workspaces/Tether_Grace/outputs/gpac_fault_tolerance/fault_matrix"),
    )
    args = parser.parse_args()

    output_dir = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)

    scenario = resolve_scenario(args)
    seeds = parse_int_values(args.seeds)
    if scenario is None:
        fault_events = [(float(args.fault_time), cable) for cable in parse_int_values(args.fault_cables)]
        fault_event_payload = None
        duration = args.duration
        params = Params()
    else:
        fault_events = [(float(event.time_seconds), int(event.cable_index)) for event in scenario.fault_events]
        fault_event_payload = [event.to_dict() for event in scenario.fault_events]
        duration = scenario.duration
        params = Params(num_agents=scenario.num_agents, cable_lengths=np.array(scenario.cable_lengths, dtype=float))

    l1_config = {
        "omega": args.l1_omega,
        "predictor_pole": args.l1_predictor_pole,
        "sigma_max": args.l1_sigma_max,
        "shared_blend": args.l1_shared_blend,
        "shared_omega": args.l1_shared_omega,
        "shared_predictor_pole": args.l1_shared_predictor_pole,
        "shared_sigma_max": args.l1_shared_sigma_max,
        "shared_ramp_time": args.l1_shared_ramp_time,
        "shared_allocator": args.l1_shared_allocator,
        "shared_load_kp_scale": args.l1_shared_load_kp_scale,
        "shared_load_kv_scale": args.l1_shared_load_kv_scale,
        "local_lateral_weight": args.l1_local_lateral_weight,
        "local_projection": args.l1_local_projection,
        "desired_geometry_mode": args.l1_desired_geometry_mode,
        "desired_geometry_ramp_time": args.l1_desired_geometry_ramp_time,
    }

    rows: list[dict[str, float]] = []
    for seed in seeds:
        nominal_metrics: dict[str, object] = {}
        for mode in ["cl", "l1"]:
            nominal_run = simulate(
                mode,
                -1,
                1e10,
                duration,
                seed,
                args.wind_sigma,
                l1_config=l1_config if mode == "l1" else None,
                params=params,
            )
            nominal_metrics[mode] = compute_fault_metrics(nominal_run["t"], nominal_run["err"], nominal_run["fault_time"])

        scenario_runs = [(float(args.fault_time), -1)] if scenario is not None else fault_events
        for fault_time, fault_cable in scenario_runs:
            for mode in ["cl", "l1"]:
                run = simulate(
                    mode,
                    fault_cable,
                    fault_time,
                    duration,
                    seed,
                    args.wind_sigma,
                    l1_config=l1_config if mode == "l1" else None,
                    params=params,
                    fault_events=fault_event_payload,
                )
                metrics = compute_fault_metrics(run["t"], run["err"], run["fault_time"])
                nominal_rmse = float(getattr(nominal_metrics[mode], "nominal_rmse"))
                rows.append(
                    {
                        "mode": mode,
                        "seed": float(seed),
                        "fault_cable": float(fault_cable),
                        "nominal_rmse": nominal_rmse,
                        "post_fault_rmse": metrics.post_fault_rmse,
                        "pre_fault_rmse": metrics.pre_fault_rmse,
                        "fault_delta_rmse": metrics.post_fault_rmse - nominal_rmse,
                        "rmse_ratio": metrics.rmse_ratio,
                        "peak_deviation": metrics.peak_deviation,
                        "recovery_time": metrics.recovery_time,
                    }
                )

    aggregates = [aggregate_rows(rows, mode) for mode in ["cl", "l1"]]
    per_cable = aggregate_by_cable(rows, "cl") + aggregate_by_cable(rows, "l1")
    payload = {
        "config": {
            "duration": duration,
            "fault_time": args.fault_time,
            "seeds": seeds,
            "fault_cables": [cable for _, cable in fault_events],
            "fault_times": [time_value for time_value, _ in fault_events],
            "scenario": scenario.name if scenario is not None else "",
            "scenario_fault_profiles": [event["profile"] for event in fault_event_payload] if fault_event_payload is not None else [],
            "num_agents": params.num_agents,
            "cable_lengths": params.cable_lengths.tolist(),
            "wind_sigma": args.wind_sigma,
            "l1_config": l1_config,
        },
        "aggregates": aggregates,
        "per_cable": per_cable,
        "rows": rows,
    }

    (output_dir / "fault_matrix_summary.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")
    write_csv(rows, output_dir / "fault_matrix_summary.csv")
    if not args.skip_plots:
        save_summary_figure(output_dir, rows, aggregates)
    write_markdown_report(output_dir, aggregates, per_cable, payload["config"])

    print("=" * 72)
    print("FAULT MATRIX SUMMARY")
    print("=" * 72)
    for aggregate in aggregates:
        print(
            f"{aggregate['mode'].upper():<4} | mean post={aggregate['mean_post_fault_rmse'] * 100.0:>6.2f} cm"
            f" | mean delta={aggregate['mean_fault_delta'] * 100.0:>6.2f} cm"
            f" | max post={aggregate['max_post_fault_rmse'] * 100.0:>6.2f} cm"
            f" | mean peak={aggregate['mean_peak_deviation'] * 100.0:>6.2f} cm"
            f" | spread={aggregate['cable_spread'] * 100.0:>6.2f} cm"
        )



if __name__ == "__main__":
    main()
