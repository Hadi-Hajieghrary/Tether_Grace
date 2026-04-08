#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import matplotlib
import numpy as np

from fault_metrics_root import compute_fault_metrics
from topology_invariance_root import Params, simulate

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def parse_float_values(raw: str) -> list[float]:
    return [float(part.strip()) for part in raw.split(",") if part.strip()]


def save_recording(path: Path, run: dict[str, Any]) -> None:
    arrays = {key: value for key, value in run.items() if isinstance(value, np.ndarray)}
    np.savez_compressed(path, **arrays)


def summary_metrics(run: dict[str, Any]) -> dict[str, float]:
    metrics = compute_fault_metrics(run["t"], run["err"], run["fault_time"])
    return metrics.to_dict()


def plot_load_tracking(output_dir: Path, results: dict[str, dict[str, Any]], fault_time: float) -> None:
    figure, axes = plt.subplots(2, 2, figsize=(15, 10))

    l1_run = results["l1_snap"]
    for axis_index, label in enumerate(["x", "y", "z"]):
        row = axis_index // 2
        col = axis_index % 2
        axes[row, col].plot(l1_run["t"], l1_run["load_d"][:, axis_index], label=f"desired {label}", lw=1.4)
        axes[row, col].plot(l1_run["t"], l1_run["load_p"][:, axis_index], label=f"actual {label}", lw=1.0)
        axes[row, col].axvline(fault_time, color="r", ls="--", lw=1.0)
        axes[row, col].set_ylabel("Position [m]")
        axes[row, col].set_title(f"Load {label.upper()} Tracking")
        axes[row, col].grid(True, alpha=0.3)
        axes[row, col].legend(fontsize=8)

    axes[1, 1].plot(results["cl_snap"]["t"], results["cl_snap"]["err"] * 100.0, label="CL error", lw=1.1)
    axes[1, 1].plot(results["l1_snap"]["t"], results["l1_snap"]["err"] * 100.0, label="L1 error", lw=1.1)
    axes[1, 1].axvline(fault_time, color="r", ls="--", lw=1.0)
    axes[1, 1].set_ylabel("Error norm [cm]")
    axes[1, 1].set_title("Payload Tracking Error")
    axes[1, 1].grid(True, alpha=0.3)
    axes[1, 1].legend(fontsize=8)

    for axis in axes[1, :]:
        axis.set_xlabel("Time [s]")

    figure.tight_layout()
    figure.savefig(output_dir / "five_agent_load_tracking.png", dpi=150, bbox_inches="tight")
    plt.close(figure)


def plot_quad_signals(output_dir: Path, run: dict[str, Any], fault_time: float) -> None:
    figure, axes = plt.subplots(2, 2, figsize=(15, 10))
    quad_error = np.linalg.norm(run["quad_p"] - run["quad_d"], axis=2)
    quad_force_norm = np.linalg.norm(run["quad_force"], axis=2)
    quad_wind_norm = np.linalg.norm(run["quad_wind"], axis=2)
    quad_speed = np.linalg.norm(run["quad_v"], axis=2)

    for agent in range(quad_error.shape[1]):
        label = f"Quad {agent}"
        axes[0, 0].plot(run["t"], quad_error[:, agent], lw=0.9, label=label)
        axes[0, 1].plot(run["t"], quad_force_norm[:, agent], lw=0.9, label=label)
        axes[1, 0].plot(run["t"], quad_wind_norm[:, agent], lw=0.9, label=label)
        axes[1, 1].plot(run["t"], quad_speed[:, agent], lw=0.9, label=label)

    titles = [
        "Per-Quad Position Error Norm",
        "Per-Quad Commanded Force Norm",
        "Per-Quad Wind Force Norm",
        "Per-Quad Speed Norm",
    ]
    ylabels = ["Error [m]", "Force [N]", "Wind force [N]", "Speed [m/s]"]
    for axis, title, ylabel in zip(axes.ravel(), titles, ylabels):
        axis.axvline(fault_time, color="r", ls="--", lw=1.0)
        axis.set_title(title)
        axis.set_ylabel(ylabel)
        axis.set_xlabel("Time [s]")
        axis.grid(True, alpha=0.3)

    axes[0, 0].legend(fontsize=8, ncol=2)
    figure.tight_layout()
    figure.savefig(output_dir / "five_agent_quad_signals.png", dpi=150, bbox_inches="tight")
    plt.close(figure)


def plot_cable_and_controller_signals(output_dir: Path, run: dict[str, Any], fault_time: float, fault_cable: int) -> None:
    figure, axes = plt.subplots(2, 2, figsize=(15, 10))
    local_sigma_norm = np.linalg.norm(run["local_sigma"], axis=2)
    shared_allocation_norm = np.linalg.norm(run["shared_allocations"], axis=2)
    adaptive_force_norm = np.linalg.norm(run["adaptive_force"], axis=2)

    for agent in range(run["tensions"].shape[1]):
        label = f"Cable {agent}"
        if agent == fault_cable:
            label += " faulted"
        axes[0, 0].plot(run["t"], run["tensions"][:, agent], lw=1.0, label=label)
        axes[0, 1].plot(run["t"], local_sigma_norm[:, agent], lw=1.0, label=f"Quad {agent}")
        axes[1, 0].plot(run["t"], shared_allocation_norm[:, agent], lw=1.0, label=f"Quad {agent}")
        axes[1, 1].plot(run["t"], adaptive_force_norm[:, agent], lw=1.0, label=f"Quad {agent}")

    axes[0, 0].set_title("Cable Tensions")
    axes[0, 0].set_ylabel("Tension [N]")
    axes[0, 1].set_title("Local L1 Sigma Norm")
    axes[0, 1].set_ylabel("Sigma [m/s^2]")
    axes[1, 0].set_title("Shared Adaptive Allocation Norm")
    axes[1, 0].set_ylabel("Force [N]")
    axes[1, 1].set_title("Applied Adaptive Force Norm")
    axes[1, 1].set_ylabel("Force [N]")

    for axis in axes.ravel():
        axis.axvline(fault_time, color="r", ls="--", lw=1.0)
        axis.set_xlabel("Time [s]")
        axis.grid(True, alpha=0.3)

    axes[0, 0].legend(fontsize=8, ncol=2)
    figure.tight_layout()
    figure.savefig(output_dir / "five_agent_cable_controller_signals.png", dpi=150, bbox_inches="tight")
    plt.close(figure)


def plot_xy_overview(output_dir: Path, run: dict[str, Any], fault_time: float) -> None:
    figure, axes = plt.subplots(1, 2, figsize=(15, 6))
    axes[0].plot(run["load_d"][:, 0], run["load_d"][:, 1], label="Desired load", lw=1.4)
    axes[0].plot(run["load_p"][:, 0], run["load_p"][:, 1], label="Actual load", lw=1.0)
    fault_index = int(np.searchsorted(run["t"], fault_time))
    fault_index = min(max(fault_index, 0), len(run["t"]) - 1)
    axes[0].scatter(run["load_p"][fault_index, 0], run["load_p"][fault_index, 1], color="r", label="Fault instant")
    axes[0].set_title("Payload XY Trajectory")
    axes[0].set_xlabel("x [m]")
    axes[0].set_ylabel("y [m]")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(fontsize=8)

    final_quad_positions = run["quad_p"][-1]
    final_quad_desired = run["quad_d"][-1]
    axes[1].scatter(final_quad_desired[:, 0], final_quad_desired[:, 1], marker="x", s=80, label="Desired quad positions")
    axes[1].scatter(final_quad_positions[:, 0], final_quad_positions[:, 1], marker="o", s=50, label="Actual quad positions")
    for agent in range(final_quad_positions.shape[0]):
        axes[1].plot(
            [final_quad_desired[agent, 0], final_quad_positions[agent, 0]],
            [final_quad_desired[agent, 1], final_quad_positions[agent, 1]],
            lw=0.8,
            color="0.4",
        )
        axes[1].text(final_quad_positions[agent, 0], final_quad_positions[agent, 1], str(agent), fontsize=9)
    axes[1].set_title("Final Quad XY Positions")
    axes[1].set_xlabel("x [m]")
    axes[1].set_ylabel("y [m]")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(fontsize=8)

    figure.tight_layout()
    figure.savefig(output_dir / "five_agent_xy_overview.png", dpi=150, bbox_inches="tight")
    plt.close(figure)


def build_summary(
    params: Params,
    results: dict[str, dict[str, Any]],
    fault_time: float,
    fault_cable: int,
    l1_config: dict[str, float],
) -> dict[str, Any]:
    return {
        "num_agents": params.num_agents,
        "cable_lengths": params.cable_lengths.tolist(),
        "formation_radius": params.formation_radius,
        "fault_time": fault_time,
        "fault_cable": fault_cable,
        "modes": {name: summary_metrics(run) for name, run in results.items()},
        "l1_config": l1_config,
        "recorded_signals": {
            "load": ["load_p", "load_v", "load_d", "load_vd", "load_a_cmd", "load_err_vec", "load_wind", "err"],
            "quad": ["quad_p", "quad_v", "quad_d", "quad_vd", "quad_force", "quad_accel_cmd", "quad_wind"],
            "cable": ["tensions", "cable_directions", "cable_health", "cable_force_comp"],
            "controller": [
                "feedback_force",
                "adaptive_force",
                "local_sigma",
                "local_compensation",
                "shared_sigma",
                "shared_compensation",
                "shared_force",
                "shared_allocations",
            ],
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Run and record a five-agent suspended-load fault simulation")
    parser.add_argument("--num-agents", type=int, default=5)
    parser.add_argument("--cable-lengths", type=str, default="1.04,1.08,1.13,1.10,1.06")
    parser.add_argument("--formation-radius", type=float, default=0.85)
    parser.add_argument("--duration", type=float, default=12.0)
    parser.add_argument("--fault-time", type=float, default=6.0)
    parser.add_argument("--fault-cable", type=int, default=2)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--wind-sigma", type=float, default=0.5)
    parser.add_argument("--l1-omega", type=float, default=3.5)
    parser.add_argument("--l1-predictor-pole", type=float, default=17.0)
    parser.add_argument("--l1-sigma-max", type=float, default=2.0)
    parser.add_argument("--l1-shared-blend", type=float, default=0.5)
    parser.add_argument("--l1-shared-omega", type=float, default=3.0)
    parser.add_argument("--l1-shared-predictor-pole", type=float, default=12.0)
    parser.add_argument("--l1-shared-sigma-max", type=float, default=1.5)
    parser.add_argument("--l1-shared-ramp-time", type=float, default=0.2)
    parser.add_argument("--l1-shared-allocator", type=str, default="z-weighted")
    parser.add_argument("--l1-shared-load-kp-scale", type=float, default=0.5)
    parser.add_argument("--l1-shared-load-kv-scale", type=float, default=0.5)
    parser.add_argument("--l1-local-lateral-weight", type=float, default=0.15)
    parser.add_argument("--l1-local-projection", type=str, default="uniform_xy")
    parser.add_argument("--l1-desired-geometry-mode", type=str, default="healthy_centered_renormalized")
    parser.add_argument("--l1-desired-geometry-ramp-time", type=float, default=0.2)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("/workspaces/Tether_Grace/outputs/gpac_fault_tolerance/five_agent_recording"),
    )
    args = parser.parse_args()

    cable_lengths = parse_float_values(args.cable_lengths)
    params = Params(
        num_agents=args.num_agents,
        cable_lengths=np.array(cable_lengths, dtype=float),
        formation_radius=args.formation_radius,
    )
    if not 0 <= args.fault_cable < params.num_agents:
        raise ValueError(f"fault cable index {args.fault_cable} must be in [0, {params.num_agents - 1}]")

    output_dir = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)

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

    results = {
        "cl_snap": simulate(
            "cl",
            fault_cable=args.fault_cable,
            fault_time=args.fault_time,
            duration=args.duration,
            seed=args.seed,
            wind_sigma=args.wind_sigma,
            params=params,
        ),
        "l1_snap": simulate(
            "l1",
            fault_cable=args.fault_cable,
            fault_time=args.fault_time,
            duration=args.duration,
            seed=args.seed,
            wind_sigma=args.wind_sigma,
            l1_config=l1_config,
            params=params,
        ),
    }

    save_recording(output_dir / "cl_snap_recording.npz", results["cl_snap"])
    save_recording(output_dir / "l1_snap_recording.npz", results["l1_snap"])

    plot_load_tracking(output_dir, results, args.fault_time)
    plot_quad_signals(output_dir, results["l1_snap"], args.fault_time)
    plot_cable_and_controller_signals(output_dir, results["l1_snap"], args.fault_time, args.fault_cable)
    plot_xy_overview(output_dir, results["l1_snap"], args.fault_time)

    summary = build_summary(params, results, args.fault_time, args.fault_cable, l1_config)
    (output_dir / "five_agent_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")

    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()