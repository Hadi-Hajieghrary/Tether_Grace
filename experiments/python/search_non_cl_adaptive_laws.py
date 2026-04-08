#!/usr/bin/env python3
from __future__ import annotations

import argparse
import itertools
import json
from pathlib import Path
from typing import Any

import numpy as np

from experiment_manifest import load_scenarios
from run_l1_matrix_sweep import evaluate_mode, parse_float_values, parse_int_values, summarize
from topology_invariance_root import Params


def candidate_families() -> list[dict[str, Any]]:
    return [
        {
            "family": "local_only",
            "shared_blend": 0.0,
            "shared_allocator": "equal",
            "shared_load_kp_scale": 0.5,
            "shared_load_kv_scale": 0.5,
            "local_lateral_weight": 1.0,
            "local_projection": "uniform_xy",
            "desired_geometry_mode": "off",
            "desired_geometry_ramp_time": 0.5,
            "shared_ramp_time": 0.2,
            "shared_omega": 8.0,
            "shared_predictor_pole": 16.0,
            "shared_sigma_max": 8.0,
        },
        {
            "family": "shared_equal_only",
            "shared_blend": 1.0,
            "shared_allocator": "equal",
            "shared_load_kp_scale": 0.5,
            "shared_load_kv_scale": 0.5,
            "local_lateral_weight": 0.0,
            "local_projection": "uniform_xy",
            "desired_geometry_mode": "off",
            "desired_geometry_ramp_time": 0.5,
            "shared_ramp_time": 0.2,
            "shared_omega": 6.0,
            "shared_predictor_pole": 14.0,
            "shared_sigma_max": 4.0,
        },
        {
            "family": "shared_z_weighted",
            "shared_blend": 1.0,
            "shared_allocator": "z-weighted",
            "shared_load_kp_scale": 0.65,
            "shared_load_kv_scale": 0.65,
            "local_lateral_weight": 0.0,
            "local_projection": "uniform_xy",
            "desired_geometry_mode": "off",
            "desired_geometry_ramp_time": 0.5,
            "shared_ramp_time": 0.2,
            "shared_omega": 6.0,
            "shared_predictor_pole": 14.0,
            "shared_sigma_max": 4.0,
        },
        {
            "family": "hybrid_shared_local",
            "shared_blend": 0.5,
            "shared_allocator": "z-weighted",
            "shared_load_kp_scale": 0.5,
            "shared_load_kv_scale": 0.5,
            "local_lateral_weight": 0.2,
            "local_projection": "uniform_xy",
            "desired_geometry_mode": "off",
            "desired_geometry_ramp_time": 0.5,
            "shared_ramp_time": 0.2,
            "shared_omega": 4.0,
            "shared_predictor_pole": 12.0,
            "shared_sigma_max": 2.0,
        },
        {
            "family": "geometry_only",
            "shared_blend": 0.0,
            "shared_allocator": "equal",
            "shared_load_kp_scale": 0.5,
            "shared_load_kv_scale": 0.5,
            "local_lateral_weight": 0.15,
            "local_projection": "uniform_xy",
            "desired_geometry_mode": "healthy_centered_renormalized",
            "desired_geometry_ramp_time": 0.2,
            "shared_ramp_time": 0.2,
            "shared_omega": 4.0,
            "shared_predictor_pole": 12.0,
            "shared_sigma_max": 2.0,
        },
        {
            "family": "geometry_shared",
            "shared_blend": 0.5,
            "shared_allocator": "z-weighted",
            "shared_load_kp_scale": 0.5,
            "shared_load_kv_scale": 0.5,
            "local_lateral_weight": 0.15,
            "local_projection": "uniform_xy",
            "desired_geometry_mode": "healthy_centered_renormalized",
            "desired_geometry_ramp_time": 0.2,
            "shared_ramp_time": 0.2,
            "shared_omega": 3.0,
            "shared_predictor_pole": 12.0,
            "shared_sigma_max": 1.5,
        },
    ]


def compute_tradeoff_score(row: dict[str, Any]) -> float:
    return float(row["mean_delta_vs_cl_rmse_m"] + 0.5 * row["mean_delta_vs_cl_peak_m"] + 0.25 * row["mean_delta_vs_cl_spread_m"])


def write_markdown(output_dir: Path, payload: dict[str, Any]) -> None:
    baseline_rows = payload["baseline_cl_by_scenario"]
    lines = [
        "# Non-CL Adaptive Law Search",
        "",
        f"- Candidate families tested: {payload['candidate_family_count']}",
        f"- Total candidates tested: {payload['candidate_count']}",
        f"- Robust winners: {len(payload['robust_winners'])}",
        "",
        "## CL Baselines",
        "",
        "| Scenario | N | CL post [cm] | CL peak [cm] | CL spread [cm] |",
        "| --- | ---: | ---: | ---: | ---: |",
    ]
    for row in baseline_rows:
        lines.append(
            f"| {row['scenario']} | {row['num_agents']} | {row['mean_post_fault_rmse'] * 100.0:.2f} | {row['mean_peak_deviation'] * 100.0:.2f} | {row['cable_spread'] * 100.0:.2f} |"
        )

    lines.extend([
        "",
        "## Best Balanced Candidate",
        "",
    ])
    best = payload.get("best_balanced_candidate")
    if best is not None:
        lines.extend([
            f"- Family: {best['family']}",
            f"- omega={best['omega']:.2f}, predictor_pole={best['predictor_pole']:.2f}, sigma_max={best['sigma_max']:.2f}",
            f"- Mean delta vs CL post-fault RMSE: {best['mean_delta_vs_cl_rmse_m'] * 100.0:.2f} cm",
            f"- Mean delta vs CL peak deviation: {best['mean_delta_vs_cl_peak_m'] * 100.0:.2f} cm",
            "",
            "| Scenario | Delta post vs CL [cm] | Delta peak vs CL [cm] | Delta spread vs CL [cm] |",
            "| --- | ---: | ---: | ---: |",
        ])
        for row in best["scenario_metrics"]:
            lines.append(
                f"| {row['scenario']} | {row['delta_vs_cl_rmse_m'] * 100.0:.2f} | {row['delta_vs_cl_peak_m'] * 100.0:.2f} | {row['delta_vs_cl_spread_m'] * 100.0:.2f} |"
            )

    (output_dir / "non_cl_adaptive_search_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description="Systematically test structured non-CL adaptive laws against CL across a shared scenario manifest")
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--seeds", type=str, default="42,43,44")
    parser.add_argument("--wind-sigma", type=float, default=0.5)
    parser.add_argument("--omegas", type=str, default="3,5")
    parser.add_argument("--predictor-poles", type=str, default="10,14")
    parser.add_argument("--sigma-maxes", type=str, default="1.5,4")
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()

    args.output_dir.mkdir(parents=True, exist_ok=True)
    seeds = parse_int_values(args.seeds)
    omegas = parse_float_values(args.omegas)
    predictor_poles = parse_float_values(args.predictor_poles)
    sigma_maxes = parse_float_values(args.sigma_maxes)
    scenarios = load_scenarios(args.manifest)

    baseline_by_scenario: list[dict[str, Any]] = []
    baseline_lookup: dict[str, dict[str, Any]] = {}
    scenario_metadata: dict[str, tuple[list[int], float, Params, list[dict[str, Any]]]] = {}
    for scenario in scenarios:
        fault_cables = [int(event.cable_index) for event in scenario.fault_events]
        fault_events = [event.to_dict() for event in scenario.fault_events]
        params = Params(num_agents=scenario.num_agents, cable_lengths=np.array(scenario.cable_lengths, dtype=float))
        cl_rows = evaluate_mode("cl", seeds, fault_cables, scenario.duration, scenario.fault_events[0].time_seconds, args.wind_sigma, None, params=params, fault_events=fault_events)
        cl_summary = summarize(cl_rows)
        row = {
            "scenario": scenario.name,
            "num_agents": scenario.num_agents,
            **cl_summary,
        }
        baseline_by_scenario.append(row)
        baseline_lookup[scenario.name] = row
        scenario_metadata[scenario.name] = (fault_cables, scenario.duration, params, fault_events)

    candidates: list[dict[str, Any]] = []
    for family in candidate_families():
        for omega, predictor_pole, sigma_max in itertools.product(omegas, predictor_poles, sigma_maxes):
            scenario_metrics = []
            for scenario in scenarios:
                fault_cables, duration, params, fault_events = scenario_metadata[scenario.name]
                config = {
                    "omega": omega,
                    "predictor_pole": predictor_pole,
                    "sigma_max": sigma_max,
                    "shared_blend": family["shared_blend"],
                    "shared_omega": family["shared_omega"],
                    "shared_predictor_pole": family["shared_predictor_pole"],
                    "shared_sigma_max": family["shared_sigma_max"],
                    "shared_ramp_time": family["shared_ramp_time"],
                    "shared_allocator": family["shared_allocator"],
                    "shared_load_kp_scale": family["shared_load_kp_scale"],
                    "shared_load_kv_scale": family["shared_load_kv_scale"],
                    "local_lateral_weight": family["local_lateral_weight"],
                    "local_projection": family["local_projection"],
                    "desired_geometry_mode": family["desired_geometry_mode"],
                    "desired_geometry_ramp_time": family["desired_geometry_ramp_time"],
                }
                l1_rows = evaluate_mode("l1", seeds, fault_cables, duration, scenario.fault_events[0].time_seconds, args.wind_sigma, config, params=params, fault_events=fault_events)
                l1_summary = summarize(l1_rows)
                cl_summary = baseline_lookup[scenario.name]
                scenario_metrics.append(
                    {
                        "scenario": scenario.name,
                        "num_agents": scenario.num_agents,
                        "l1_mean_post_fault_rmse_m": l1_summary["mean_post_fault_rmse"],
                        "l1_mean_peak_deviation_m": l1_summary["mean_peak_deviation"],
                        "l1_cable_spread_m": l1_summary["cable_spread"],
                        "delta_vs_cl_rmse_m": l1_summary["mean_post_fault_rmse"] - cl_summary["mean_post_fault_rmse"],
                        "delta_vs_cl_peak_m": l1_summary["mean_peak_deviation"] - cl_summary["mean_peak_deviation"],
                        "delta_vs_cl_spread_m": l1_summary["cable_spread"] - cl_summary["cable_spread"],
                    }
                )

            candidate = {
                "family": family["family"],
                "omega": omega,
                "predictor_pole": predictor_pole,
                "sigma_max": sigma_max,
                "structural_config": family,
                "scenario_metrics": scenario_metrics,
                "mean_delta_vs_cl_rmse_m": float(np.mean([row["delta_vs_cl_rmse_m"] for row in scenario_metrics])),
                "mean_delta_vs_cl_peak_m": float(np.mean([row["delta_vs_cl_peak_m"] for row in scenario_metrics])),
                "mean_delta_vs_cl_spread_m": float(np.mean([row["delta_vs_cl_spread_m"] for row in scenario_metrics])),
                "scenarios_beating_cl_on_rmse": int(sum(1 for row in scenario_metrics if row["delta_vs_cl_rmse_m"] < 0.0)),
                "scenarios_beating_cl_on_peak": int(sum(1 for row in scenario_metrics if row["delta_vs_cl_peak_m"] < 0.0)),
            }
            candidate["tradeoff_score"] = compute_tradeoff_score(candidate)
            candidates.append(candidate)

    candidates.sort(key=lambda item: (item["tradeoff_score"], item["mean_delta_vs_cl_rmse_m"], item["mean_delta_vs_cl_peak_m"]))
    robust_winners = [
        candidate
        for candidate in candidates
        if all(row["delta_vs_cl_rmse_m"] <= 0.0 and row["delta_vs_cl_peak_m"] <= 0.0 for row in candidate["scenario_metrics"])
    ]

    payload = {
        "generator": "experiments/python/search_non_cl_adaptive_laws.py",
        "manifest": str(args.manifest),
        "seeds": seeds,
        "wind_sigma": args.wind_sigma,
        "candidate_family_count": len(candidate_families()),
        "candidate_count": len(candidates),
        "baseline_cl_by_scenario": baseline_by_scenario,
        "best_balanced_candidate": candidates[0] if candidates else None,
        "best_rmse_candidate": min(candidates, key=lambda item: item["mean_delta_vs_cl_rmse_m"]) if candidates else None,
        "robust_winners": robust_winners,
        "top_candidates": candidates[:10],
    }

    (args.output_dir / "non_cl_adaptive_search.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")
    write_markdown(args.output_dir, payload)
    print(json.dumps(payload, indent=2))


if __name__ == "__main__":
    main()