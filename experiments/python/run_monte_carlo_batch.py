#!/usr/bin/env python3
"""Monte Carlo batch wrapper for the full-Drake fault runner.

Runs a single scenario N times with different random seeds (for wind
turbulence) and aggregates statistics across trials.

Usage:
    python3 run_monte_carlo_batch.py --scenario three_drones --trials 20 --enable-wind
    python3 run_monte_carlo_batch.py --manifest ../manifests/matched_single_fault.json \
        --scenario matched_n3_1fault --trials 10 --enable-wind --wind-sigma 0.8
"""
from __future__ import annotations

import argparse
import csv
import json
import subprocess
from pathlib import Path
from typing import Sequence

import numpy as np

ROOT = Path("/workspaces/Tether_Grace")
OUTPUT_ROOT = ROOT / "outputs" / "monte_carlo"
BUILD_EXE = ROOT / "build" / "full_drake_fault_runner"


def load_scenario(manifest_path: Path, scenario_name: str) -> dict:
    """Load a named scenario from a JSON manifest."""
    with open(manifest_path, encoding="utf-8") as f:
        data = json.load(f)
    for s in data["scenarios"]:
        if s["name"] == scenario_name:
            return s
    raise KeyError(f"Scenario '{scenario_name}' not found in {manifest_path}")


def build_command(scenario: dict, output_dir: Path, seed: int,
                  enable_wind: bool, wind_sigma: float | None,
                  wind_mean: list[float] | None) -> list[str]:
    """Build the C++ runner command for a single trial."""
    cmd = [
        str(BUILD_EXE),
        "--num-quads", str(scenario["num_agents"]),
        "--duration", str(scenario["duration"]),
        "--output-dir", str(output_dir),
        "--cable-lengths", ",".join(f"{v:.3f}" for v in scenario["cable_lengths"]),
        "--headless",
    ]

    fault_events = scenario.get("fault_events", [])
    if fault_events:
        cmd += ["--fault-cables", ",".join(str(e["cable_index"]) for e in fault_events)]
        cmd += ["--fault-times", ",".join(f'{e["time_seconds"]:.3f}' for e in fault_events)]

    # Physics overrides from scenario
    for key, flag in [("max_thrust", "--max-thrust"), ("payload_mass", "--payload-mass"),
                      ("tension_kp", "--tension-kp"), ("position_kp", "--position-kp"),
                      ("position_kd", "--position-kd"), ("position_ki", "--position-ki")]:
        if key in scenario and scenario[key] is not None:
            cmd += [flag, str(scenario[key])]

    if scenario.get("enable_eso"):
        cmd += ["--enable-eso"]

    # Wind and seed
    cmd += ["--seed", str(seed)]
    if enable_wind:
        cmd += ["--enable-wind"]
    if wind_sigma is not None:
        cmd += ["--wind-sigma", str(wind_sigma)]
    if wind_mean is not None:
        cmd += ["--wind-mean"] + [str(v) for v in wind_mean]

    return cmd


def extract_metrics(trial_dir: Path) -> dict[str, float] | None:
    """Extract metrics from a completed trial's trajectory log."""
    traj_files = sorted(trial_dir.glob("logs/trajectory_*.csv"))
    if not traj_files:
        return None

    # Read trajectory CSV
    data = np.genfromtxt(traj_files[0], delimiter=",", names=True, dtype=float)
    if data.ndim == 0:
        data = np.array([tuple(data)], dtype=data.dtype)

    time = data["time"]

    # Find load columns
    load_cols = [n for n in data.dtype.names if n.startswith("load_")]
    if len(load_cols) < 3:
        return None

    load_x = data[load_cols[0]]
    load_y = data[load_cols[1]]
    load_z = data[load_cols[2]]

    # Find reference columns
    ref_cols = [n for n in data.dtype.names if n.startswith("ref_load") or n.startswith("reference_load")]
    if len(ref_cols) >= 3:
        ref_x, ref_y, ref_z = data[ref_cols[0]], data[ref_cols[1]], data[ref_cols[2]]
    else:
        # No reference in CSV — compute from waypoints
        return None

    error = np.sqrt((load_x - ref_x)**2 + (load_y - ref_y)**2 + (load_z - ref_z)**2)
    rmse = float(np.sqrt(np.mean(error**2)))
    peak_error = float(np.max(error))
    final_error = float(error[-1])

    # Post-fault metrics (assume first fault at earliest fault time)
    manifest_path = trial_dir / "run_manifest.json"
    first_fault_time = None
    if manifest_path.exists():
        with open(manifest_path) as f:
            manifest = json.load(f)
        fault_times = manifest.get("fault_times", [])
        if fault_times:
            first_fault_time = min(fault_times)

    post_fault_peak = peak_error
    if first_fault_time is not None:
        post_fault_mask = time >= first_fault_time
        if np.any(post_fault_mask):
            post_fault_peak = float(np.max(error[post_fault_mask]))

    return {
        "rmse_m": rmse,
        "peak_error_m": peak_error,
        "post_fault_peak_m": post_fault_peak,
        "final_error_m": final_error,
    }


def run_mc_batch(scenario: dict, scenario_name: str, num_trials: int,
                 enable_wind: bool, wind_sigma: float | None,
                 wind_mean: list[float] | None) -> list[dict]:
    """Run num_trials trials of a scenario with different seeds."""
    batch_dir = OUTPUT_ROOT / scenario_name
    batch_dir.mkdir(parents=True, exist_ok=True)

    results = []
    for trial_idx in range(num_trials):
        seed = trial_idx
        trial_dir = batch_dir / f"trial_{trial_idx:03d}"
        trial_dir.mkdir(parents=True, exist_ok=True)

        print(f"  Trial {trial_idx+1}/{num_trials} (seed={seed})...", flush=True)
        cmd = build_command(scenario, trial_dir, seed, enable_wind, wind_sigma, wind_mean)
        subprocess.run(cmd, check=True, cwd=ROOT)

        metrics = extract_metrics(trial_dir)
        if metrics is not None:
            metrics["seed"] = seed
            metrics["trial_dir"] = str(trial_dir)
            results.append(metrics)
            print(f"    RMSE={metrics['rmse_m']*100:.1f}cm  peak={metrics['peak_error_m']*100:.1f}cm  "
                  f"final={metrics['final_error_m']*100:.1f}cm")
        else:
            print(f"    WARNING: could not extract metrics from {trial_dir}")

    return results


def write_mc_summary(scenario_name: str, results: list[dict], batch_dir: Path) -> None:
    """Write MC summary CSV and statistics JSON."""
    if not results:
        print("No results to summarize.")
        return

    # CSV with per-trial metrics
    csv_path = batch_dir / "mc_trials.csv"
    keys = ["seed", "rmse_m", "peak_error_m", "post_fault_peak_m", "final_error_m"]
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
        writer.writeheader()
        for r in results:
            writer.writerow(r)

    # Statistics
    metric_keys = ["rmse_m", "peak_error_m", "post_fault_peak_m", "final_error_m"]
    stats: dict[str, dict[str, float]] = {}
    for key in metric_keys:
        values = np.array([r[key] for r in results])
        stats[key] = {
            "mean": float(np.mean(values)),
            "std": float(np.std(values)),
            "min": float(np.min(values)),
            "max": float(np.max(values)),
            "median": float(np.median(values)),
            "n_trials": len(values),
        }

    stats_path = batch_dir / "mc_statistics.json"
    with open(stats_path, "w", encoding="utf-8") as f:
        json.dump({"scenario": scenario_name, "statistics": stats}, f, indent=2)

    print(f"\n=== Monte Carlo Summary: {scenario_name} ({len(results)} trials) ===")
    for key in metric_keys:
        s = stats[key]
        print(f"  {key}: mean={s['mean']*100:.2f}cm  std={s['std']*100:.2f}cm  "
              f"[{s['min']*100:.2f}, {s['max']*100:.2f}]cm")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scenario", required=True, help="Scenario name from manifest")
    parser.add_argument("--manifest", type=Path,
                        default=ROOT / "experiments" / "manifests" / "full_drake_reference_batch.json",
                        help="Path to scenario manifest JSON")
    parser.add_argument("--trials", type=int, default=10, help="Number of MC trials")
    parser.add_argument("--enable-wind", action="store_true", help="Enable Dryden wind disturbance")
    parser.add_argument("--wind-sigma", type=float, default=None, help="Wind turbulence intensity")
    parser.add_argument("--wind-mean", type=float, nargs=3, default=None, help="Mean wind [x y z] m/s")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if not BUILD_EXE.exists():
        raise FileNotFoundError(f"Expected built executable at {BUILD_EXE}")

    scenario = load_scenario(args.manifest, args.scenario)
    print(f"Running {args.trials} MC trials for scenario '{args.scenario}'")
    if args.enable_wind:
        print(f"  Wind: sigma={args.wind_sigma or 0.5}, mean={args.wind_mean or [1.0, 0.5, 0.0]}")

    results = run_mc_batch(scenario, args.scenario, args.trials,
                           args.enable_wind, args.wind_sigma, args.wind_mean)

    batch_dir = OUTPUT_ROOT / args.scenario
    write_mc_summary(args.scenario, results, batch_dir)


if __name__ == "__main__":
    main()
