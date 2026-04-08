#!/usr/bin/env python3
from __future__ import annotations

import argparse
import itertools
import json
from pathlib import Path

from topology_invariance_root import compute_fault_metrics, simulate


def parse_values(raw: str) -> list[float]:
    return [float(part.strip()) for part in raw.split(",") if part.strip()]


def main() -> None:
    parser = argparse.ArgumentParser(description="Sweep L1 tuning parameters for the root-owned topology-invariance experiment")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--fault-time", type=float, default=5.0)
    parser.add_argument("--fault-cable", type=int, default=1)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--wind-sigma", type=float, default=0.5)
    parser.add_argument("--omegas", type=str, default="10,15,20,25,30")
    parser.add_argument("--predictor-poles", type=str, default="30,40,50,60")
    parser.add_argument("--sigma-maxes", type=str, default="10,15,20,25")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("/workspaces/Tether_Grace/outputs/gpac_fault_tolerance/l1_tuning_sweep"),
    )
    args = parser.parse_args()

    output_dir = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)

    omegas = parse_values(args.omegas)
    predictor_poles = parse_values(args.predictor_poles)
    sigma_maxes = parse_values(args.sigma_maxes)

    baseline = simulate(
        "cl",
        args.fault_cable,
        args.fault_time,
        args.duration,
        args.seed,
        args.wind_sigma,
    )
    baseline_metrics = compute_fault_metrics(baseline["t"], baseline["err"], baseline["fault_time"])

    results: list[dict[str, float]] = []
    for omega, predictor_pole, sigma_max in itertools.product(omegas, predictor_poles, sigma_maxes):
        l1_run = simulate(
            "l1",
            args.fault_cable,
            args.fault_time,
            args.duration,
            args.seed,
            args.wind_sigma,
            l1_config={
                "omega": omega,
                "predictor_pole": predictor_pole,
                "sigma_max": sigma_max,
            },
        )
        metrics = compute_fault_metrics(l1_run["t"], l1_run["err"], l1_run["fault_time"])
        results.append(
            {
                "omega": omega,
                "predictor_pole": predictor_pole,
                "sigma_max": sigma_max,
                "post_fault_rmse": metrics.post_fault_rmse,
                "pre_fault_rmse": metrics.pre_fault_rmse,
                "rmse_ratio": metrics.rmse_ratio,
                "peak_deviation": metrics.peak_deviation,
                "recovery_time": metrics.recovery_time,
                "cl_post_fault_rmse": baseline_metrics.post_fault_rmse,
                "delta_vs_cl": metrics.post_fault_rmse - baseline_metrics.post_fault_rmse,
            }
        )

    results.sort(key=lambda item: (item["post_fault_rmse"], item["peak_deviation"], item["recovery_time"]))
    payload = {
        "baseline_cl": baseline_metrics.to_dict(),
        "best_l1": results[0] if results else None,
        "grid": results,
    }
    (output_dir / "l1_tuning_sweep.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")

    print("=" * 72)
    print("L1 TUNING SWEEP")
    print("=" * 72)
    print(f"CL baseline post-fault RMSE: {baseline_metrics.post_fault_rmse * 100.0:.2f} cm")
    print("Best candidates:")
    for item in results[:5]:
        print(
            "  "
            f"omega={item['omega']:.1f}, pole={item['predictor_pole']:.1f}, sigma_max={item['sigma_max']:.1f}"
            f" | post={item['post_fault_rmse'] * 100.0:.2f} cm"
            f" | ratio={item['rmse_ratio']:.3f}"
            f" | peak={item['peak_deviation'] * 100.0:.2f} cm"
            f" | recovery={item['recovery_time']:.2f} s"
        )


if __name__ == "__main__":
    main()