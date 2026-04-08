#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib
import numpy as np
from experiment_manifest import DEFAULT_FULL_DRAKE_BATCH_MANIFEST, load_named_scenario
from topology_invariance_root import Params, simulate

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def expected_mass_share(load_mass: float, healthy_count: int) -> float:
    return load_mass / max(healthy_count, 1)


def corruption_duration(time: np.ndarray, theta_mean: np.ndarray, target: float, start_time: float, tolerance: float) -> float | None:
    start_index = int(np.searchsorted(time, start_time, side="left"))
    for index in range(start_index, len(time)):
        if np.all(np.abs(theta_mean[index:] - target) <= tolerance):
            return float(time[index] - start_time)
    return None


def bias_duration(time: np.ndarray, signal: np.ndarray, reference: np.ndarray, start_time: float, tolerance: float) -> float | None:
    start_index = int(np.searchsorted(time, start_time, side="left"))
    exceed_indices = np.where(np.abs(signal[start_index:] - reference[start_index:]) > tolerance)[0]
    if exceed_indices.size == 0:
        return 0.0
    return float(time[start_index + exceed_indices[-1]] - start_time)


def window_mean_abs_error(time: np.ndarray, signal: np.ndarray, reference: np.ndarray, start_time: float, window_seconds: float) -> float:
    start_index = int(np.searchsorted(time, start_time, side="left"))
    end_index = int(np.searchsorted(time, start_time + window_seconds, side="left"))
    if end_index <= start_index:
        return 0.0
    return float(np.mean(np.abs(signal[start_index:end_index] - reference[start_index:end_index])))


def history_refill_time(time: np.ndarray, history_mean: np.ndarray, start_time: float, target_fraction: float = 0.95) -> float | None:
    start_index = int(np.searchsorted(time, start_time, side="left"))
    pre_fault_level = float(np.max(history_mean[: max(start_index, 1)]))
    if pre_fault_level <= 1e-9:
        return None
    target = target_fraction * pre_fault_level
    for index in range(start_index, len(time)):
        if history_mean[index] >= target:
            return float(time[index] - start_time)
    return None


def parse_str_values(raw: str) -> list[str]:
    return [part.strip() for part in raw.split(",") if part.strip()]


def mean_or_none(values: list[float | None]) -> float | None:
    finite_values = [float(value) for value in values if value is not None and np.isfinite(value)]
    if not finite_values:
        return None
    return float(np.mean(finite_values))


def run_variant(name: str, scenario_name: str, manifest: Path, seed: int, wind_sigma: float, forgetting_factor: float, flush_on_fault: bool) -> dict:
    scenario = load_named_scenario(scenario_name, manifest)
    params = Params(num_agents=scenario.num_agents, cable_lengths=np.array(scenario.cable_lengths, dtype=float))
    run = simulate(
        "cl",
        duration=scenario.duration,
        seed=seed,
        wind_sigma=wind_sigma,
        params=params,
        fault_events=[event.to_dict() for event in scenario.fault_events],
        cl_config={
            "forgetting_factor": forgetting_factor,
            "flush_on_fault": flush_on_fault,
        },
    )
    time = run["t"]
    theta_mean = np.mean(run["theta_agents"], axis=1)
    history_mean = np.mean(run["cl_history_sizes"], axis=1)
    replay_mean = np.mean(run["cl_replay_terms"], axis=1)
    first_fault_time = float(run["fault_time"])
    final_healthy = max(int(np.sum(run["cable_health"][-1] > 1e-6)), 1)
    post_target = expected_mass_share(params.load_mass, final_healthy)
    duration = corruption_duration(time, theta_mean, post_target, first_fault_time, tolerance=0.1 * post_target)
    return {
        "name": name,
        "time": time,
        "theta_mean": theta_mean,
        "history_mean": history_mean,
        "replay_mean": replay_mean,
        "first_fault_time": first_fault_time,
        "post_target_theta": post_target,
        "corruption_duration_s": duration,
        "history_refill_time_s": history_refill_time(time, history_mean, first_fault_time),
    }


def save_plot(output_dir: Path, scenario_name: str, variants: list[dict]) -> None:
    figure, axes = plt.subplots(3, 1, figsize=(11.5, 10.0), constrained_layout=True)
    for variant in variants:
        axes[0].plot(variant["time"], variant["theta_mean"], linewidth=1.8, label=variant["name"])
        axes[1].plot(variant["time"], variant["history_mean"], linewidth=1.8, label=variant["name"])
        axes[2].plot(variant["time"], variant["replay_mean"], linewidth=1.8, label=variant["name"])

    fault_time = variants[0]["first_fault_time"]
    post_target = variants[0]["post_target_theta"]
    axes[0].axvline(fault_time, color="#d62728", linestyle="--", linewidth=1.2)
    axes[0].axhline(post_target, color="#333333", linestyle=":", linewidth=1.2, label="post-fault target")
    axes[1].axvline(fault_time, color="#d62728", linestyle="--", linewidth=1.2)
    axes[2].axvline(fault_time, color="#d62728", linestyle="--", linewidth=1.2)

    axes[0].set_title("CL mean mass-share estimate")
    axes[0].set_ylabel("theta")
    axes[1].set_title("Mean CL history size")
    axes[1].set_ylabel("samples")
    axes[2].set_title("Mean CL replay term")
    axes[2].set_ylabel("replay")
    axes[2].set_xlabel("Time [s]")
    for axis in axes:
        axis.grid(True, alpha=0.3)
        axis.legend(loc="upper right")

    figure.savefig(output_dir / f"cl_history_corruption_{scenario_name}.png", dpi=180, bbox_inches="tight")
    plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser(description="Analyze reduced-order concurrent-learning history corruption under manifest-driven fault schedules")
    parser.add_argument("--scenario", type=str, default="three_drones", help="Deprecated single-scenario alias; use --scenarios for broader analysis")
    parser.add_argument("--scenarios", type=str, default="three_drones,five_drones,seven_drones")
    parser.add_argument("--manifest", type=Path, default=DEFAULT_FULL_DRAKE_BATCH_MANIFEST)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--wind-sigma", type=float, default=0.5)
    parser.add_argument("--forgetting-factor", type=float, default=0.95)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()

    args.output_dir.mkdir(parents=True, exist_ok=True)
    scenario_names = parse_str_values(args.scenarios)
    if not scenario_names:
        scenario_names = [args.scenario]

    theta_tolerance = 0.05
    replay_tolerance = 1.0

    scenario_results = []
    aggregate_variants: dict[str, list[dict[str, float | None]]] = {
        "baseline": [],
        "forgetting": [],
        "flush": [],
        "forgetting_flush": [],
    }
    for scenario_name in scenario_names:
        variants = [
            run_variant("baseline", scenario_name, args.manifest, args.seed, args.wind_sigma, forgetting_factor=1.0, flush_on_fault=False),
            run_variant("forgetting", scenario_name, args.manifest, args.seed, args.wind_sigma, forgetting_factor=args.forgetting_factor, flush_on_fault=False),
            run_variant("flush", scenario_name, args.manifest, args.seed, args.wind_sigma, forgetting_factor=1.0, flush_on_fault=True),
            run_variant("forgetting_flush", scenario_name, args.manifest, args.seed, args.wind_sigma, forgetting_factor=args.forgetting_factor, flush_on_fault=True),
        ]
        save_plot(args.output_dir, scenario_name, variants)

        flush_variant = next(variant for variant in variants if variant["name"] == "flush")
        scenario_payload_variants = []
        for variant in variants:
            if variant["name"] == "flush":
                variant["theta_bias_duration_vs_flush_s"] = 0.0
                variant["theta_bias_mean_abs_first_second_vs_flush"] = 0.0
                variant["theta_bias_peak_abs_vs_flush"] = 0.0
                variant["replay_bias_duration_vs_flush_s"] = 0.0
                variant["replay_bias_mean_abs_first_second_vs_flush"] = 0.0
                variant["replay_bias_peak_abs_vs_flush"] = 0.0
            else:
                variant["theta_bias_duration_vs_flush_s"] = bias_duration(
                    variant["time"],
                    variant["theta_mean"],
                    flush_variant["theta_mean"],
                    variant["first_fault_time"],
                    theta_tolerance,
                )
                variant["theta_bias_mean_abs_first_second_vs_flush"] = window_mean_abs_error(
                    variant["time"],
                    variant["theta_mean"],
                    flush_variant["theta_mean"],
                    variant["first_fault_time"],
                    1.0,
                )
                variant["theta_bias_peak_abs_vs_flush"] = float(np.max(np.abs(variant["theta_mean"] - flush_variant["theta_mean"])))
                variant["replay_bias_duration_vs_flush_s"] = bias_duration(
                    variant["time"],
                    variant["replay_mean"],
                    flush_variant["replay_mean"],
                    variant["first_fault_time"],
                    replay_tolerance,
                )
                variant["replay_bias_mean_abs_first_second_vs_flush"] = window_mean_abs_error(
                    variant["time"],
                    variant["replay_mean"],
                    flush_variant["replay_mean"],
                    variant["first_fault_time"],
                    1.0,
                )
                variant["replay_bias_peak_abs_vs_flush"] = float(np.max(np.abs(variant["replay_mean"] - flush_variant["replay_mean"])))

            summary_variant = {
                "name": variant["name"],
                "first_fault_time": variant["first_fault_time"],
                "post_target_theta": variant["post_target_theta"],
                "corruption_duration_s": variant["corruption_duration_s"],
                "history_refill_time_s": variant["history_refill_time_s"],
                "theta_bias_duration_vs_flush_s": variant["theta_bias_duration_vs_flush_s"],
                "theta_bias_mean_abs_first_second_vs_flush": variant["theta_bias_mean_abs_first_second_vs_flush"],
                "theta_bias_peak_abs_vs_flush": variant["theta_bias_peak_abs_vs_flush"],
                "replay_bias_duration_vs_flush_s": variant["replay_bias_duration_vs_flush_s"],
                "replay_bias_mean_abs_first_second_vs_flush": variant["replay_bias_mean_abs_first_second_vs_flush"],
                "replay_bias_peak_abs_vs_flush": variant["replay_bias_peak_abs_vs_flush"],
            }
            scenario_payload_variants.append(summary_variant)
            aggregate_variants[variant["name"]].append(summary_variant)

        scenario_results.append(
            {
                "scenario": scenario_name,
                "variants": scenario_payload_variants,
            }
        )

    aggregate_payload = []
    for name, rows in aggregate_variants.items():
        aggregate_payload.append(
            {
                "name": name,
                "mean_corruption_duration_s": mean_or_none([row["corruption_duration_s"] for row in rows]),
                "mean_history_refill_time_s": mean_or_none([row["history_refill_time_s"] for row in rows]),
                "mean_theta_bias_duration_vs_flush_s": mean_or_none([row["theta_bias_duration_vs_flush_s"] for row in rows]),
                "mean_theta_bias_mean_abs_first_second_vs_flush": mean_or_none([row["theta_bias_mean_abs_first_second_vs_flush"] for row in rows]),
                "mean_replay_bias_duration_vs_flush_s": mean_or_none([row["replay_bias_duration_vs_flush_s"] for row in rows]),
            }
        )

    payload = {
        "generator": "experiments/python/analyze_cl_history_corruption.py",
        "scenarios": scenario_names,
        "manifest": str(args.manifest),
        "theta_bias_tolerance": theta_tolerance,
        "replay_bias_tolerance": replay_tolerance,
        "scenario_results": scenario_results,
        "aggregate_variants": aggregate_payload,
    }
    (args.output_dir / "cl_history_corruption.json").write_text(json.dumps(payload, indent=2, allow_nan=False), encoding="utf-8")
    print(json.dumps(payload, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()