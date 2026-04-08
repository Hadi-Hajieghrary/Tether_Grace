#!/usr/bin/env python3
from __future__ import annotations

import argparse
import itertools
import json
from pathlib import Path

import numpy as np
from experiment_manifest import load_named_scenario

from topology_invariance_root import compute_fault_metrics, simulate
from topology_invariance_root import Params


def parse_float_values(raw: str) -> list[float]:
    return [float(part.strip()) for part in raw.split(",") if part.strip()]


def parse_int_values(raw: str) -> list[int]:
    return [int(part.strip()) for part in raw.split(",") if part.strip()]


def evaluate_mode(
    mode: str,
    seeds: list[int],
    fault_cables: list[int],
    duration: float,
    fault_time: float,
    wind_sigma: float,
    l1_config: dict[str, float] | None,
    params: Params | None = None,
    fault_events: list[dict[str, float | int | str]] | None = None,
) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    params = Params() if params is None else params
    for seed in seeds:
        nominal_run = simulate(
            mode,
            -1,
            1e10,
            duration,
            seed,
            wind_sigma,
            l1_config=l1_config if mode == "l1" else None,
            params=params,
        )
        nominal_metrics = compute_fault_metrics(nominal_run["t"], nominal_run["err"], nominal_run["fault_time"])
        if fault_events is not None:
            fault_specs = [(float(fault_events[0]["time_seconds"]), int(fault_events[0]["cable_index"]), fault_events)]
        else:
            fault_specs = [(fault_time, fault_cable, None) for fault_cable in fault_cables]
        for event_time, fault_cable, event_payload in fault_specs:
            fault_run = simulate(
                mode,
                fault_cable,
                event_time,
                duration,
                seed,
                wind_sigma,
                l1_config=l1_config if mode == "l1" else None,
                params=params,
                fault_events=event_payload,
            )
            metrics = compute_fault_metrics(fault_run["t"], fault_run["err"], fault_run["fault_time"])
            rows.append(
                {
                    "seed": float(seed),
                    "fault_cable": float(fault_cable),
                    "nominal_rmse": nominal_metrics.nominal_rmse,
                    "post_fault_rmse": metrics.post_fault_rmse,
                    "fault_delta_rmse": metrics.post_fault_rmse - nominal_metrics.nominal_rmse,
                    "peak_deviation": metrics.peak_deviation,
                    "recovery_time": metrics.recovery_time,
                    "rmse_ratio": metrics.rmse_ratio,
                }
            )
    return rows


def summarize(rows: list[dict[str, float]]) -> dict[str, float]:
    per_cable = {}
    for cable in sorted({int(row["fault_cable"]) for row in rows}):
        cable_rows = [row for row in rows if int(row["fault_cable"]) == cable]
        per_cable[cable] = float(np.mean([row["post_fault_rmse"] for row in cable_rows]))

    return {
        "mean_post_fault_rmse": float(np.mean([row["post_fault_rmse"] for row in rows])),
        "mean_fault_delta_rmse": float(np.mean([row["fault_delta_rmse"] for row in rows])),
        "mean_peak_deviation": float(np.mean([row["peak_deviation"] for row in rows])),
        "mean_recovery_time": float(np.mean([row["recovery_time"] for row in rows])),
        "mean_rmse_ratio": float(np.mean([row["rmse_ratio"] for row in rows])),
        "cable_spread": float(max(per_cable.values()) - min(per_cable.values())) if len(per_cable) > 1 else 0.0,
    }


def compute_tradeoff_score(
    candidate: dict[str, float],
    rmse_weight: float,
    spread_weight: float,
    peak_weight: float,
) -> float:
    return (
        rmse_weight * candidate["delta_vs_cl"]
        + spread_weight * candidate["spread_delta_vs_cl"]
        + peak_weight * candidate["peak_delta_vs_cl"]
    )


def pareto_front(candidates: list[dict[str, float]]) -> list[dict[str, float]]:
    front: list[dict[str, float]] = []
    for candidate in candidates:
        dominated = False
        for other in candidates:
            if other is candidate:
                continue
            no_worse = (
                other["mean_post_fault_rmse"] <= candidate["mean_post_fault_rmse"]
                and other["cable_spread"] <= candidate["cable_spread"]
                and other["mean_peak_deviation"] <= candidate["mean_peak_deviation"]
            )
            strictly_better = (
                other["mean_post_fault_rmse"] < candidate["mean_post_fault_rmse"]
                or other["cable_spread"] < candidate["cable_spread"]
                or other["mean_peak_deviation"] < candidate["mean_peak_deviation"]
            )
            if no_worse and strictly_better:
                dominated = True
                break
        if not dominated:
            front.append(candidate)

    front.sort(key=lambda item: (item["tradeoff_score"], item["mean_post_fault_rmse"], item["cable_spread"]))
    return front


def main() -> None:
    parser = argparse.ArgumentParser(description="Sweep L1 parameters using a multi-seed, multi-cable fault matrix")
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--fault-time", type=float, default=4.0)
    parser.add_argument("--seeds", type=str, default="42,43,44")
    parser.add_argument("--fault-cables", type=str, default="0,1,2")
    parser.add_argument("--scenario", type=str, default="", help="Named shared-manifest scenario to reuse")
    parser.add_argument("--manifest", type=Path, default=None, help="Optional shared scenario manifest path")
    parser.add_argument("--wind-sigma", type=float, default=0.5)
    parser.add_argument("--omegas", type=str, default="4,5,6,7,8")
    parser.add_argument("--predictor-poles", type=str, default="10,12,14,16,18")
    parser.add_argument("--sigma-maxes", type=str, default="4,5,6,7,8")
    parser.add_argument("--top-count", type=int, default=10)
    parser.add_argument("--rmse-weight", type=float, default=1.0)
    parser.add_argument("--spread-weight", type=float, default=1.0)
    parser.add_argument("--peak-weight", type=float, default=1.0)
    parser.add_argument("--shared-blend", type=float, default=0.0)
    parser.add_argument("--shared-omega", type=float, default=10.0)
    parser.add_argument("--shared-predictor-pole", type=float, default=20.0)
    parser.add_argument("--shared-sigma-max", type=float, default=10.0)
    parser.add_argument("--shared-ramp-time", type=float, default=0.2)
    parser.add_argument("--shared-allocator", type=str, default="equal")
    parser.add_argument("--shared-load-kp-scale", type=float, default=0.5)
    parser.add_argument("--shared-load-kv-scale", type=float, default=0.5)
    parser.add_argument("--local-lateral-weight", type=float, default=1.0)
    parser.add_argument("--local-projection", type=str, default="uniform_xy")
    parser.add_argument("--desired-geometry-mode", type=str, default="off")
    parser.add_argument("--desired-geometry-ramp-time", type=float, default=0.5)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("/workspaces/Tether_Grace/outputs/gpac_fault_tolerance/l1_matrix_sweep"),
    )
    args = parser.parse_args()

    output_dir = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)

    seeds = parse_int_values(args.seeds)
    if args.scenario:
        scenario = load_named_scenario(args.scenario, args.manifest)
        duration = scenario.duration
        fault_cables = [event.cable_index for event in scenario.fault_events]
        fault_events = [event.to_dict() for event in scenario.fault_events]
        params = Params(num_agents=scenario.num_agents, cable_lengths=np.array(scenario.cable_lengths, dtype=float))
    else:
        scenario = None
        duration = args.duration
        fault_cables = parse_int_values(args.fault_cables)
        fault_events = None
        params = Params()
    omegas = parse_float_values(args.omegas)
    predictor_poles = parse_float_values(args.predictor_poles)
    sigma_maxes = parse_float_values(args.sigma_maxes)

    cl_rows = evaluate_mode("cl", seeds, fault_cables, duration, args.fault_time, args.wind_sigma, None, params=params, fault_events=fault_events)
    cl_summary = summarize(cl_rows)

    candidates: list[dict[str, float]] = []
    for omega, predictor_pole, sigma_max in itertools.product(omegas, predictor_poles, sigma_maxes):
        config = {
            "omega": omega,
            "predictor_pole": predictor_pole,
            "sigma_max": sigma_max,
            "shared_blend": args.shared_blend,
            "shared_omega": args.shared_omega,
            "shared_predictor_pole": args.shared_predictor_pole,
            "shared_sigma_max": args.shared_sigma_max,
            "shared_ramp_time": args.shared_ramp_time,
            "shared_allocator": args.shared_allocator,
            "shared_load_kp_scale": args.shared_load_kp_scale,
            "shared_load_kv_scale": args.shared_load_kv_scale,
            "local_lateral_weight": args.local_lateral_weight,
            "local_projection": args.local_projection,
            "desired_geometry_mode": args.desired_geometry_mode,
            "desired_geometry_ramp_time": args.desired_geometry_ramp_time,
        }
        l1_rows = evaluate_mode("l1", seeds, fault_cables, duration, args.fault_time, args.wind_sigma, config, params=params, fault_events=fault_events)
        l1_summary = summarize(l1_rows)
        candidates.append(
            {
                "omega": omega,
                "predictor_pole": predictor_pole,
                "sigma_max": sigma_max,
                "mean_post_fault_rmse": l1_summary["mean_post_fault_rmse"],
                "mean_fault_delta_rmse": l1_summary["mean_fault_delta_rmse"],
                "mean_peak_deviation": l1_summary["mean_peak_deviation"],
                "mean_recovery_time": l1_summary["mean_recovery_time"],
                "mean_rmse_ratio": l1_summary["mean_rmse_ratio"],
                "cable_spread": l1_summary["cable_spread"],
                "delta_vs_cl": l1_summary["mean_post_fault_rmse"] - cl_summary["mean_post_fault_rmse"],
                "spread_delta_vs_cl": l1_summary["cable_spread"] - cl_summary["cable_spread"],
                "peak_delta_vs_cl": l1_summary["mean_peak_deviation"] - cl_summary["mean_peak_deviation"],
            }
        )

    for candidate in candidates:
        candidate["tradeoff_score"] = compute_tradeoff_score(
            candidate,
            args.rmse_weight,
            args.spread_weight,
            args.peak_weight,
        )

    candidates.sort(key=lambda item: (item["mean_post_fault_rmse"], item["cable_spread"], item["mean_peak_deviation"]))
    balanced_candidates = sorted(
        candidates,
        key=lambda item: (item["tradeoff_score"], item["mean_post_fault_rmse"], item["cable_spread"]),
    )
    payload = {
        "config": {
            "duration": duration,
            "scenario": scenario.name if scenario is not None else "",
            "fault_time": args.fault_time,
            "seeds": seeds,
            "fault_cables": fault_cables,
            "fault_times": [event["time_seconds"] for event in fault_events] if fault_events is not None else [args.fault_time],
            "num_agents": params.num_agents,
            "cable_lengths": params.cable_lengths.tolist(),
            "wind_sigma": args.wind_sigma,
            "tradeoff_weights": {
                "rmse": args.rmse_weight,
                "spread": args.spread_weight,
                "peak": args.peak_weight,
            },
            "shared_l1": {
                "blend": args.shared_blend,
                "omega": args.shared_omega,
                "predictor_pole": args.shared_predictor_pole,
                "sigma_max": args.shared_sigma_max,
                "ramp_time": args.shared_ramp_time,
                "allocator": args.shared_allocator,
                "load_kp_scale": args.shared_load_kp_scale,
                "load_kv_scale": args.shared_load_kv_scale,
                "local_lateral_weight": args.local_lateral_weight,
                "local_projection": args.local_projection,
                "desired_geometry_mode": args.desired_geometry_mode,
                "desired_geometry_ramp_time": args.desired_geometry_ramp_time,
            },
        },
        "baseline_cl": cl_summary,
        "candidates": candidates,
        "best_l1": candidates[0] if candidates else None,
        "best_balanced": balanced_candidates[0] if balanced_candidates else None,
        "pareto_front": pareto_front(candidates),
    }
    (output_dir / "l1_matrix_sweep.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")

    print("=" * 72)
    print("L1 MATRIX SWEEP")
    print("=" * 72)
    print(
        f"CL baseline | mean post={cl_summary['mean_post_fault_rmse'] * 100.0:.2f} cm"
        f" | spread={cl_summary['cable_spread'] * 100.0:.2f} cm"
        f" | peak={cl_summary['mean_peak_deviation'] * 100.0:.2f} cm"
    )
    if balanced_candidates:
        best_balanced = balanced_candidates[0]
        print(
            "Best balanced candidate: "
            f"omega={best_balanced['omega']:.1f}, pole={best_balanced['predictor_pole']:.1f}, sigma_max={best_balanced['sigma_max']:.1f}"
            f" | post={best_balanced['mean_post_fault_rmse'] * 100.0:.2f} cm"
            f" | spread={best_balanced['cable_spread'] * 100.0:.2f} cm"
            f" | peak={best_balanced['mean_peak_deviation'] * 100.0:.2f} cm"
            f" | score={best_balanced['tradeoff_score'] * 100.0:.2f}"
        )
    print("Best RMSE candidates:")
    for item in candidates[: args.top_count]:
        print(
            "  "
            f"omega={item['omega']:.1f}, pole={item['predictor_pole']:.1f}, sigma_max={item['sigma_max']:.1f}"
            f" | post={item['mean_post_fault_rmse'] * 100.0:.2f} cm"
            f" | spread={item['cable_spread'] * 100.0:.2f} cm"
            f" | peak={item['mean_peak_deviation'] * 100.0:.2f} cm"
            f" | delta_vs_cl={item['delta_vs_cl'] * 100.0:.2f} cm"
            f" | score={item['tradeoff_score'] * 100.0:.2f}"
        )


if __name__ == "__main__":
    main()
