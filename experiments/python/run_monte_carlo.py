#!/usr/bin/env python3
"""Monte Carlo campaign: run N_MC randomized simulations per scenario,
varying fault timing, cable lengths, and wind seeds. Extracts
distributional statistics (mean, std, 95th percentile) for paper tables."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import numpy as np

ROOT = Path("/workspaces/Tether_Grace")
BUILD_EXE = ROOT / "build" / "full_drake_fault_runner"
MC_OUTPUT = ROOT / "outputs" / "monte_carlo"

sys.path.insert(0, str(ROOT / "experiments" / "python"))
from run_fault_schedule_batch import build_payload_reference, load_csv_matrix


@dataclass
class MCScenario:
    name: str
    num_agents: int
    duration: float
    nominal_cable_lengths: list[float]
    fault_cables: list[int]
    nominal_fault_times: list[float]
    enable_eskf: bool = False
    enable_wind: bool = False


SCENARIOS = [
    MCScenario(
        name="mc_n3_1fault",
        num_agents=3,
        duration=30.0,
        nominal_cable_lengths=[1.00, 1.08, 1.16],
        fault_cables=[0],
        nominal_fault_times=[7.0],
    ),
    MCScenario(
        name="mc_n7_3fault",
        num_agents=7,
        duration=30.0,
        nominal_cable_lengths=[0.98, 1.01, 1.04, 1.07, 1.10, 1.13, 1.16],
        fault_cables=[1, 3, 5],
        nominal_fault_times=[7.0, 12.0, 14.0],
    ),
    MCScenario(
        name="mc_n3_eskf_wind",
        num_agents=3,
        duration=30.0,
        nominal_cable_lengths=[1.00, 1.08, 1.16],
        fault_cables=[0],
        nominal_fault_times=[7.0],
        enable_eskf=True,
        enable_wind=True,
    ),
]


def randomize_scenario(
    scenario: MCScenario, rng: np.random.Generator, run_idx: int,
) -> tuple[list[float], list[float], int]:
    """Return (cable_lengths, fault_times, seed) with randomization."""
    # Cable lengths: ±5% Gaussian perturbation
    cable_lengths = [
        float(np.clip(L + rng.normal(0, 0.05 * L), 0.5, 2.0))
        for L in scenario.nominal_cable_lengths
    ]
    # Fixed fault times — match deterministic canonical scenarios exactly
    fault_times = list(scenario.nominal_fault_times)
    # Seed for wind randomization
    seed = int(rng.integers(1, 100000))
    return cable_lengths, fault_times, seed


def run_single(
    scenario: MCScenario,
    cable_lengths: list[float],
    fault_times: list[float],
    seed: int,
    output_dir: Path,
) -> Path:
    """Run one Drake simulation and return the log directory."""
    output_dir.mkdir(parents=True, exist_ok=True)
    cmd = [
        str(BUILD_EXE),
        "--num-quads", str(scenario.num_agents),
        "--duration", str(scenario.duration),
        "--output-dir", str(output_dir),
        "--cable-lengths", ",".join(f"{v:.3f}" for v in cable_lengths),
        "--fault-cables", ",".join(str(c) for c in scenario.fault_cables),
        "--fault-times", ",".join(f"{t:.3f}" for t in fault_times),
        "--headless",
        "--seed", str(seed),
    ]
    if scenario.enable_wind:
        cmd += ["--enable-wind", "--wind-mean", "2.0", "0.0", "0.0", "--wind-sigma", "1.0"]
    if scenario.enable_eskf:
        cmd += ["--enable-eskf", "--gps-noise", "0.02", "--baro-noise", "0.3"]
    subprocess.run(cmd, check=True, cwd=ROOT, capture_output=True)
    return output_dir


def extract_metrics(output_dir: Path, fault_times: list[float], duration: float) -> dict:
    """Extract RMSE, peak, final from a completed run."""
    manifest = json.loads((output_dir / "run_manifest.json").read_text())
    log_dir = Path(manifest["log_dir"])
    h, traj = load_csv_matrix(log_dir / "trajectories.csv")
    time = traj[:, 0]
    load_xyz = np.column_stack([
        traj[:, h.index("load_x")],
        traj[:, h.index("load_y")],
        traj[:, h.index("load_z")],
    ])
    first_ft = min(fault_times) if fault_times else duration
    ref, _, _ = build_payload_reference(time, duration, load_xyz[:, 2], first_ft)
    error = np.linalg.norm(load_xyz - ref, axis=1)
    post = time >= first_ft
    return {
        "rmse": float(np.sqrt(np.mean(error**2))),
        "peak": float(np.max(error[post])) if np.any(post) else float(np.max(error)),
        "final": float(error[-1]),
    }


def run_campaign(scenario: MCScenario, n_mc: int, max_parallel: int) -> list[dict]:
    """Run n_mc simulations for one scenario, return list of metric dicts."""
    rng = np.random.default_rng(seed=42)
    base_dir = MC_OUTPUT / scenario.name
    base_dir.mkdir(parents=True, exist_ok=True)

    results = []
    # Run sequentially (Drake sims are CPU-bound and already use 100% of one core)
    for i in range(n_mc):
        cable_lengths, fault_times, seed = randomize_scenario(scenario, rng, i)
        run_dir = base_dir / f"run_{i:03d}"
        print(f"  [{scenario.name}] Run {i+1}/{n_mc}: "
              f"fault_times={fault_times}, seed={seed}", flush=True)
        try:
            run_single(scenario, cable_lengths, fault_times, seed, run_dir)
            metrics = extract_metrics(run_dir, fault_times, scenario.duration)
            metrics["run"] = i
            metrics["fault_times"] = fault_times
            metrics["cable_lengths"] = cable_lengths
            metrics["seed"] = seed
            results.append(metrics)
        except Exception as e:
            print(f"    FAILED: {e}", flush=True)

    # Save results
    (base_dir / "mc_results.json").write_text(
        json.dumps(results, indent=2), encoding="utf-8"
    )
    return results


def summarize(results: list[dict], name: str) -> dict:
    """Compute distributional statistics."""
    if not results:
        return {}
    rmse = np.array([r["rmse"] for r in results]) * 100  # cm
    peak = np.array([r["peak"] for r in results]) * 100
    final = np.array([r["final"] for r in results]) * 100
    summary = {
        "scenario": name,
        "n_runs": len(results),
        "rmse_mean": float(np.mean(rmse)),
        "rmse_std": float(np.std(rmse)),
        "peak_mean": float(np.mean(peak)),
        "peak_std": float(np.std(peak)),
        "peak_95th": float(np.percentile(peak, 95)),
        "final_mean": float(np.mean(final)),
        "final_std": float(np.std(final)),
    }
    return summary


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--n-mc", type=int, default=30, help="Runs per scenario")
    parser.add_argument("--scenario", type=str, default=None,
                        help="Run only this scenario (mc_n3_1fault, mc_n7_3fault, mc_n3_eskf_wind)")
    args = parser.parse_args()

    MC_OUTPUT.mkdir(parents=True, exist_ok=True)
    all_summaries = []

    for scenario in SCENARIOS:
        if args.scenario and scenario.name != args.scenario:
            continue
        print(f"\n=== {scenario.name} ({args.n_mc} runs) ===", flush=True)
        results = run_campaign(scenario, args.n_mc, max_parallel=1)
        summary = summarize(results, scenario.name)
        all_summaries.append(summary)
        print(f"  RMSE: {summary['rmse_mean']:.2f} ± {summary['rmse_std']:.2f} cm")
        print(f"  Peak: {summary['peak_mean']:.2f} ± {summary['peak_std']:.2f} cm "
              f"(95th: {summary['peak_95th']:.2f} cm)")
        print(f"  Final: {summary['final_mean']:.2f} ± {summary['final_std']:.2f} cm")

    (MC_OUTPUT / "mc_summaries.json").write_text(
        json.dumps(all_summaries, indent=2), encoding="utf-8"
    )
    print(f"\nResults saved to {MC_OUTPUT}")


if __name__ == "__main__":
    main()
