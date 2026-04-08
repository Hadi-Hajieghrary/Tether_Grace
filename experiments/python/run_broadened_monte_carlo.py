#!/usr/bin/env python3
"""Broadened Monte Carlo campaign: randomizes cable lengths, fault timing,
payload mass, and PID gains.

Extends run_monte_carlo.py with additional parameter perturbations.
Supports two modes:
  --mode broadened:  all parameters randomized simultaneously (Campaign A)
  --mode isolation:  one parameter at a time (Campaign B sensitivity analysis)
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

ROOT = Path("/workspaces/Tether_Grace")
BUILD_EXE = ROOT / "build" / "full_drake_fault_runner"
MC_OUTPUT = ROOT / "outputs" / "monte_carlo_broadened"

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


@dataclass
class RandomizedParams:
    """Parameters for a single MC run after randomization."""
    cable_lengths: list[float]
    fault_times: list[float]
    seed: int
    payload_mass: float = 3.0
    position_kp: float = 8.0
    position_kd: float = 8.0


SCENARIOS = [
    MCScenario(
        name="broad_n3_1fault",
        num_agents=3,
        duration=30.0,
        nominal_cable_lengths=[1.00, 1.08, 1.16],
        fault_cables=[0],
        nominal_fault_times=[7.0],
    ),
    MCScenario(
        name="broad_n7_3fault",
        num_agents=7,
        duration=30.0,
        nominal_cable_lengths=[0.98, 1.01, 1.04, 1.07, 1.10, 1.13, 1.16],
        fault_cables=[1, 3, 5],
        nominal_fault_times=[7.0, 12.0, 14.0],
    ),
    MCScenario(
        name="broad_n3_eskf_wind",
        num_agents=3,
        duration=30.0,
        nominal_cable_lengths=[1.00, 1.08, 1.16],
        fault_cables=[0],
        nominal_fault_times=[7.0],
        enable_eskf=True,
        enable_wind=True,
    ),
]

# For Campaign B isolation, use only the N=3 single fault scenario
ISOLATION_SCENARIO = SCENARIOS[0]


def randomize_broadened(
    scenario: MCScenario, rng: np.random.Generator,
) -> RandomizedParams:
    """Randomize ALL parameters simultaneously (Campaign A)."""
    # Cable lengths: ±5% Gaussian
    cable_lengths = [
        float(np.clip(L + rng.normal(0, 0.05 * L), 0.5, 2.0))
        for L in scenario.nominal_cable_lengths
    ]
    # Fault timing: ±2s uniform, enforce minimum 2s inter-fault spacing
    fault_times = []
    for i, t_nom in enumerate(scenario.nominal_fault_times):
        t = t_nom + rng.uniform(-2.0, 2.0)
        t = max(t, 2.0)  # don't fault before t=2s
        if i > 0:
            t = max(t, fault_times[-1] + 2.0)  # minimum inter-fault spacing
        fault_times.append(float(t))
    # Payload mass: ±10% uniform
    payload_mass = float(3.0 * (1 + rng.uniform(-0.10, 0.10)))
    # PID gains: ±10% uniform
    kp_scale = 1.0 + rng.uniform(-0.10, 0.10)
    kd_scale = 1.0 + rng.uniform(-0.10, 0.10)
    seed = int(rng.integers(1, 100000))

    return RandomizedParams(
        cable_lengths=cable_lengths,
        fault_times=fault_times,
        seed=seed,
        payload_mass=payload_mass,
        position_kp=float(8.0 * kp_scale),
        position_kd=float(8.0 * kd_scale),
    )


def randomize_isolated(
    scenario: MCScenario, rng: np.random.Generator,
    vary: str,
) -> RandomizedParams:
    """Randomize only ONE parameter category (Campaign B)."""
    params = RandomizedParams(
        cable_lengths=list(scenario.nominal_cable_lengths),
        fault_times=list(scenario.nominal_fault_times),
        seed=int(rng.integers(1, 100000)),
    )
    if vary == "cable_length":
        params.cable_lengths = [
            float(np.clip(L + rng.normal(0, 0.05 * L), 0.5, 2.0))
            for L in scenario.nominal_cable_lengths
        ]
    elif vary == "payload_mass":
        params.payload_mass = float(3.0 * (1 + rng.uniform(-0.10, 0.10)))
    elif vary == "pid_gains":
        kp_scale = 1.0 + rng.uniform(-0.10, 0.10)
        kd_scale = 1.0 + rng.uniform(-0.10, 0.10)
        params.position_kp = float(8.0 * kp_scale)
        params.position_kd = float(8.0 * kd_scale)
    elif vary == "fault_timing":
        fault_times = []
        for i, t_nom in enumerate(scenario.nominal_fault_times):
            t = t_nom + rng.uniform(-2.0, 2.0)
            t = max(t, 2.0)
            if i > 0:
                t = max(t, fault_times[-1] + 2.0)
            fault_times.append(float(t))
        params.fault_times = fault_times
    elif vary == "all":
        return randomize_broadened(scenario, rng)
    return params


def run_single(
    scenario: MCScenario,
    params: RandomizedParams,
    output_dir: Path,
) -> Path:
    """Run one Drake simulation."""
    output_dir.mkdir(parents=True, exist_ok=True)
    cmd = [
        str(BUILD_EXE),
        "--num-quads", str(scenario.num_agents),
        "--duration", str(scenario.duration),
        "--output-dir", str(output_dir),
        "--cable-lengths", ",".join(f"{v:.3f}" for v in params.cable_lengths),
        "--fault-cables", ",".join(str(c) for c in scenario.fault_cables),
        "--fault-times", ",".join(f"{t:.3f}" for t in params.fault_times),
        "--headless",
        "--seed", str(params.seed),
        "--payload-mass", f"{params.payload_mass:.3f}",
        "--position-kp", f"{params.position_kp:.3f}",
        "--position-kd", f"{params.position_kd:.3f}",
    ]
    if scenario.enable_wind:
        cmd += ["--enable-wind", "--wind-mean", "2.0", "0.0", "0.0",
                "--wind-sigma", "1.0"]
    if scenario.enable_eskf:
        cmd += ["--enable-eskf", "--gps-noise", "0.02", "--baro-noise", "0.3"]
    subprocess.run(cmd, check=True, cwd=ROOT, capture_output=True)
    return output_dir


def extract_metrics(
    output_dir: Path, fault_times: list[float], duration: float,
) -> dict:
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
        "peak": float(np.max(error[post])) if np.any(post) else float(
            np.max(error)),
        "final": float(error[-1]),
    }


def run_campaign(
    scenario: MCScenario, n_mc: int, randomize_fn,
) -> list[dict]:
    """Run n_mc simulations, return list of metric dicts."""
    rng = np.random.default_rng(seed=42)
    base_dir = MC_OUTPUT / scenario.name
    base_dir.mkdir(parents=True, exist_ok=True)

    results = []
    for i in range(n_mc):
        params = randomize_fn(scenario, rng)
        run_dir = base_dir / f"run_{i:03d}"
        print(f"  [{scenario.name}] Run {i+1}/{n_mc}: "
              f"mass={params.payload_mass:.2f}kg "
              f"kp={params.position_kp:.2f} kd={params.position_kd:.2f} "
              f"ft={params.fault_times}", flush=True)
        try:
            run_single(scenario, params, run_dir)
            metrics = extract_metrics(
                run_dir, params.fault_times, scenario.duration)
            metrics["run"] = i
            metrics["payload_mass"] = params.payload_mass
            metrics["position_kp"] = params.position_kp
            metrics["position_kd"] = params.position_kd
            metrics["fault_times"] = params.fault_times
            metrics["cable_lengths"] = params.cable_lengths
            metrics["seed"] = params.seed
            results.append(metrics)
        except Exception as e:
            print(f"    FAILED: {e}", flush=True)

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
    return {
        "scenario": name,
        "n_runs": len(results),
        "rmse_mean": float(np.mean(rmse)),
        "rmse_std": float(np.std(rmse)),
        "peak_mean": float(np.mean(peak)),
        "peak_std": float(np.std(peak)),
        "peak_95th": float(np.percentile(peak, 95)),
    }


def main():
    parser = argparse.ArgumentParser(description="Broadened Monte Carlo")
    parser.add_argument("--n-mc", type=int, default=30)
    parser.add_argument("--mode", choices=["broadened", "isolation"],
                        default="broadened")
    parser.add_argument("--scenario", type=str, default=None)
    args = parser.parse_args()

    MC_OUTPUT.mkdir(parents=True, exist_ok=True)
    all_summaries = []

    if args.mode == "broadened":
        # Campaign A: all parameters randomized, 30 runs per scenario
        for scenario in SCENARIOS:
            if args.scenario and scenario.name != args.scenario:
                continue
            print(f"\n=== {scenario.name} (broadened, {args.n_mc} runs) ===")
            results = run_campaign(
                scenario, args.n_mc,
                lambda s, r: randomize_broadened(s, r),
            )
            summary = summarize(results, scenario.name)
            all_summaries.append(summary)
            print(f"  RMSE: {summary['rmse_mean']:.2f} ± "
                  f"{summary['rmse_std']:.2f} cm")
            print(f"  Peak: {summary['peak_mean']:.2f} ± "
                  f"{summary['peak_std']:.2f} cm "
                  f"(95th: {summary['peak_95th']:.2f} cm)")

    elif args.mode == "isolation":
        # Campaign B: one parameter at a time, N=3 single fault only
        scenario = ISOLATION_SCENARIO
        for vary in ["cable_length", "payload_mass", "pid_gains",
                     "fault_timing", "all"]:
            iso_name = f"iso_{vary}"
            iso_scenario = MCScenario(
                name=iso_name,
                num_agents=scenario.num_agents,
                duration=scenario.duration,
                nominal_cable_lengths=scenario.nominal_cable_lengths,
                fault_cables=scenario.fault_cables,
                nominal_fault_times=scenario.nominal_fault_times,
            )
            n = min(args.n_mc, 10)  # 10 runs per isolation factor
            print(f"\n=== {iso_name} ({n} runs) ===")
            results = run_campaign(
                iso_scenario, n,
                lambda s, r, v=vary: randomize_isolated(s, r, v),
            )
            summary = summarize(results, iso_name)
            all_summaries.append(summary)
            if summary:
                print(f"  RMSE: {summary['rmse_mean']:.2f} ± "
                      f"{summary['rmse_std']:.2f} cm")

    (MC_OUTPUT / "broadened_mc_summaries.json").write_text(
        json.dumps(all_summaries, indent=2), encoding="utf-8"
    )
    print(f"\nResults saved to {MC_OUTPUT}")


if __name__ == "__main__":
    main()
