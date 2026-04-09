#!/usr/bin/env python3
"""Bead-count discretization sweep for the five-drone cascading-fault scenario.

Runs the canonical five-drone scenario with varying cable bead counts
to study the effect of cable discretization on load dynamics.
"""
from __future__ import annotations

import argparse
import json
import subprocess
import time as time_mod
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from experiment_manifest import ExperimentScenario, FaultEvent, load_named_scenario


ROOT = Path("/workspaces/Tether_Grace")
BUILD_EXE = ROOT / "build" / "full_drake_fault_runner"
SWEEP_OUTPUT_ROOT = ROOT / "outputs" / "bead_count_sweep" / "five_drones"

# Pilot grid of bead counts
DEFAULT_BEAD_COUNTS = [2, 4, 8, 12, 16]

# Bead counts that also get meshcat replay artifacts
REPLAY_BEAD_COUNTS = {8}


def get_canonical_five_drone_scenario() -> ExperimentScenario:
    """Load the canonical five-drone scenario from the manifest."""
    try:
        return load_named_scenario("five_drones")
    except KeyError:
        return ExperimentScenario(
            name="five_drones",
            num_agents=5,
            duration=30.0,
            cable_lengths=[1.00, 1.05, 1.10, 1.14, 1.18],
            fault_events=[
                FaultEvent(cable_index=1, time_seconds=7.0),
                FaultEvent(cable_index=3, time_seconds=12.0),
            ],
            description="Five-drone sequential-snap full-Drake reference scenario.",
            tags=["full_drake", "reference_batch"],
        )


@dataclass
class BeadRunResult:
    num_beads: int
    output_dir: Path
    wall_time_seconds: float
    success: bool
    error_message: str = ""


def compute_rope_parameters_for_logging(
    num_beads: int,
    rope_length: float,
    rope_total_mass: float = 0.2,
    payload_mass: float = 3.0,
    num_quadcopters: int = 5,
    max_stretch_percentage: float = 0.15,
    max_bead_radius: float = 0.012,
) -> dict[str, object]:
    """Mirror the C++ rope parameter computation for manifest logging."""
    gravity = 9.81
    avg_rope_length = rope_length
    load_per_rope = (payload_mass * gravity) / num_quadcopters
    effective_rope_stiffness = load_per_rope / (avg_rope_length * max_stretch_percentage)
    num_segments = num_beads + 1
    segment_stiffness = effective_rope_stiffness * num_segments

    # Cable-level damping (matches the invariant formula in full_drake_fault_runner.cc)
    reference_num_segments = 9  # 8 beads + 1
    reference_stiffness = 300.0
    reference_damping = 15.0
    reference_segment_stiffness = effective_rope_stiffness * reference_num_segments
    reference_segment_damping = reference_damping * (
        reference_segment_stiffness / reference_stiffness
    ) ** 0.5
    cable_damping = reference_segment_damping / reference_num_segments
    segment_damping = cable_damping * num_segments

    bead_mass = rope_total_mass / num_beads
    segment_rest_length = rope_length / num_segments
    raw_bead_radius = 0.49 * segment_rest_length
    bead_radius = min(raw_bead_radius, max_bead_radius)
    bead_radius_capped = raw_bead_radius > max_bead_radius

    return {
        "num_beads": num_beads,
        "num_segments": num_segments,
        "rope_length": rope_length,
        "rope_total_mass": rope_total_mass,
        "bead_mass": round(bead_mass, 6),
        "bead_radius": round(bead_radius, 6),
        "bead_radius_capped": bead_radius_capped,
        "segment_rest_length": round(segment_rest_length, 6),
        "segment_stiffness": round(segment_stiffness, 4),
        "segment_damping": round(segment_damping, 6),
        "effective_cable_stiffness": round(effective_rope_stiffness, 4),
        "effective_cable_damping": round(cable_damping, 6),
    }


def run_single_bead_count(
    scenario: ExperimentScenario,
    num_beads: int,
    output_dir: Path,
    generate_replay: bool = False,
) -> BeadRunResult:
    """Run the Drake executable for a single bead count."""
    output_dir.mkdir(parents=True, exist_ok=True)

    command = [
        str(BUILD_EXE),
        "--num-quads", str(scenario.num_agents),
        "--duration", str(scenario.duration),
        "--output-dir", str(output_dir),
        "--cable-lengths", ",".join(f"{v:.3f}" for v in scenario.cable_lengths),
        "--headless",
        "--num-rope-beads", str(num_beads),
    ]
    if generate_replay:
        command += ["--record-meshcat-html"]
    if scenario.fault_cables:
        command += [
            "--fault-cables", ",".join(str(v) for v in scenario.fault_cables),
            "--fault-times", ",".join(f"{v:.3f}" for v in scenario.fault_times),
        ]
    # Forward any physics overrides from the scenario
    if scenario.max_thrust is not None:
        command += ["--max-thrust", str(scenario.max_thrust)]
    if scenario.payload_mass is not None:
        command += ["--payload-mass", str(scenario.payload_mass)]
    if scenario.tension_kp is not None:
        command += ["--tension-kp", str(scenario.tension_kp)]
    if scenario.disable_tension_ff:
        command += ["--disable-tension-ff"]
    if scenario.position_kp is not None:
        command += ["--position-kp", str(scenario.position_kp)]
    if scenario.position_kd is not None:
        command += ["--position-kd", str(scenario.position_kd)]
    if scenario.position_ki is not None:
        command += ["--position-ki", str(scenario.position_ki)]

    print(f"  Running {num_beads}-bead simulation → {output_dir}")
    start = time_mod.monotonic()
    try:
        subprocess.run(command, check=True, cwd=ROOT, capture_output=True, text=True)
        elapsed = time_mod.monotonic() - start
        print(f"  ✓ {num_beads} beads completed in {elapsed:.1f}s")
        return BeadRunResult(
            num_beads=num_beads,
            output_dir=output_dir,
            wall_time_seconds=elapsed,
            success=True,
        )
    except subprocess.CalledProcessError as exc:
        elapsed = time_mod.monotonic() - start
        msg = f"Exit code {exc.returncode}: {exc.stderr[-500:]}" if exc.stderr else f"Exit code {exc.returncode}"
        print(f"  ✗ {num_beads} beads FAILED after {elapsed:.1f}s: {msg}")
        return BeadRunResult(
            num_beads=num_beads,
            output_dir=output_dir,
            wall_time_seconds=elapsed,
            success=False,
            error_message=msg,
        )


def load_csv_matrix(path: Path) -> tuple[list[str], np.ndarray]:
    """Load a CSV file with headers into a numpy matrix."""
    data = np.genfromtxt(path, delimiter=",", names=True, dtype=float)
    headers = list(data.dtype.names) if data.dtype.names else []
    if not headers:
        raise RuntimeError(f"No headers found in {path}")
    if data.ndim == 0:
        data = np.array([tuple(data)], dtype=data.dtype)
    matrix = np.column_stack([data[name] for name in headers])
    return headers, matrix


def archive_run_data(run_dir: Path) -> Path | None:
    """Compress raw CSV logs into a single NPZ archive."""
    manifest_path = run_dir / "run_manifest.json"
    if not manifest_path.exists():
        return None
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    log_dir = Path(manifest["log_dir"])

    traj_path = log_dir / "trajectories.csv"
    tension_path = log_dir / "tensions.csv"
    if not traj_path.exists() or not tension_path.exists():
        return None

    traj_headers, traj = load_csv_matrix(traj_path)
    tension_headers, tension = load_csv_matrix(tension_path)

    arrays: dict[str, np.ndarray] = {
        "trajectories": traj,
        "trajectory_headers": np.array(traj_headers, dtype=object),
        "tensions": tension,
        "tension_headers": np.array(tension_headers, dtype=object),
    }

    control_path = log_dir / "control_efforts.csv"
    if control_path.exists():
        ctrl_headers, ctrl = load_csv_matrix(control_path)
        arrays["controls"] = ctrl
        arrays["control_headers"] = np.array(ctrl_headers, dtype=object)

    npz_path = run_dir / "full_drake_recording.npz"
    np.savez_compressed(npz_path, **arrays)
    return npz_path


def run_sweep(
    bead_counts: list[int],
    output_root: Path,
    runtime_kill_seconds: float = 3600.0,
) -> list[BeadRunResult]:
    """Run the full bead-count sweep."""
    scenario = get_canonical_five_drone_scenario()
    print(f"Bead-count sweep: {bead_counts}")
    print(f"Scenario: {scenario.name}, N={scenario.num_agents}, T={scenario.duration}s")
    print(f"Cable lengths: {scenario.cable_lengths}")
    print(f"Faults: cables {scenario.fault_cables} at times {scenario.fault_times}")
    print(f"Output root: {output_root}")
    print()

    # Write the sweep manifest before running
    sweep_manifest: dict[str, object] = {
        "study": "bead_count_discretization",
        "scenario_name": scenario.name,
        "bead_counts": bead_counts,
        "invariants": {
            "cable_lengths": scenario.cable_lengths,
            "fault_cables": scenario.fault_cables,
            "fault_times": scenario.fault_times,
            "duration": scenario.duration,
            "num_agents": scenario.num_agents,
            "rope_total_mass": 0.2,
            "simulation_timestep": 2e-4,
            "note": "Cable-level stiffness and damping held invariant across bead counts",
        },
        "rope_parameters_per_bead_count": {},
    }

    # Log derived rope parameters for each bead count
    avg_cable = sum(scenario.cable_lengths) / len(scenario.cable_lengths)
    for nb in bead_counts:
        sweep_manifest["rope_parameters_per_bead_count"][str(nb)] = (
            compute_rope_parameters_for_logging(nb, avg_cable)
        )

    output_root.mkdir(parents=True, exist_ok=True)
    (output_root / "sweep_manifest.json").write_text(
        json.dumps(sweep_manifest, indent=2) + "\n", encoding="utf-8"
    )

    results: list[BeadRunResult] = []
    for nb in bead_counts:
        run_dir = output_root / f"beads_{nb:02d}"
        generate_replay = nb in REPLAY_BEAD_COUNTS
        result = run_single_bead_count(scenario, nb, run_dir, generate_replay)
        results.append(result)

        if result.success:
            archive_run_data(run_dir)

        if result.wall_time_seconds > runtime_kill_seconds:
            print(f"\n  ⚠ Runtime ({result.wall_time_seconds:.0f}s) exceeded kill threshold "
                  f"({runtime_kill_seconds:.0f}s). Stopping sweep.")
            break

    # Write results summary
    results_summary = {
        "completed_runs": [
            {
                "num_beads": r.num_beads,
                "output_dir": str(r.output_dir),
                "wall_time_seconds": round(r.wall_time_seconds, 2),
                "success": r.success,
                "error_message": r.error_message,
            }
            for r in results
        ],
    }
    (output_root / "sweep_results.json").write_text(
        json.dumps(results_summary, indent=2) + "\n", encoding="utf-8"
    )

    successful = sum(1 for r in results if r.success)
    failed = sum(1 for r in results if not r.success)
    print(f"\nSweep complete: {successful} succeeded, {failed} failed out of {len(results)} runs.")
    return results


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Bead-count discretization sweep for the five-drone scenario."
    )
    parser.add_argument(
        "--bead-counts",
        type=str,
        default=",".join(str(b) for b in DEFAULT_BEAD_COUNTS),
        help=f"Comma-separated bead counts (default: {','.join(str(b) for b in DEFAULT_BEAD_COUNTS)})",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default=str(SWEEP_OUTPUT_ROOT),
        help=f"Output directory (default: {SWEEP_OUTPUT_ROOT})",
    )
    parser.add_argument(
        "--runtime-kill",
        type=float,
        default=3600.0,
        help="Kill threshold per run in seconds (default: 3600)",
    )
    args = parser.parse_args()

    bead_counts = [int(x.strip()) for x in args.bead_counts.split(",")]
    output_root = Path(args.output_dir)

    if not BUILD_EXE.exists():
        raise FileNotFoundError(
            f"Drake executable not found at {BUILD_EXE}. "
            "Run: cmake --build build --target full_drake_fault_runner"
        )

    run_sweep(bead_counts, output_root, args.runtime_kill)


if __name__ == "__main__":
    main()
