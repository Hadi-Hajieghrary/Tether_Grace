#!/usr/bin/env python3
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import subprocess
import sys
import time
from pathlib import Path
from typing import Callable

from experiment_manifest import DEFAULT_FULL_DRAKE_BATCH_MANIFEST, load_scenarios


ROOT = Path("/workspaces/Tether_Grace")
PYTHON = Path(sys.executable)
DEFAULT_OUTPUT_DIR = ROOT / "outputs" / "gpac_fault_tolerance" / "unknowns_status_pack"
DEFAULT_FULL_DRAKE_BATCH_DIR = ROOT / "outputs" / "full_drake_fault_batch"
DEFAULT_DEGRADATION_MANIFEST = ROOT / "experiments" / "manifests" / "reduced_order_degradation_extended.json"
DEFAULT_SCALING_MANIFEST = ROOT / "experiments" / "manifests" / "reduced_order_topology_scaling_extended.json"
STAGE_STATUS_FILE = "run_unknowns_status_pack_stage_status.json"
STAGE_ORDER = [
    "full_drake_analysis",
    "reference_reduced_order",
    "reduced_vs_drake",
    "u2",
    "u3",
    "u4_scaling",
    "u5",
    "u6",
    "status_pack",
    "run_manifest",
]
PROFILE_NAMES = ["report", "smoke", "no-u6"]

DEFAULT_U6_CONFIG = {
    "l1_omega": 3.5,
    "l1_predictor_pole": 17.0,
    "l1_sigma_max": 2.0,
    "l1_shared_blend": 0.5,
    "l1_shared_omega": 3.0,
    "l1_shared_predictor_pole": 12.0,
    "l1_shared_sigma_max": 1.5,
    "l1_shared_ramp_time": 0.2,
    "l1_shared_allocator": "z-weighted",
    "l1_shared_load_kp_scale": 0.5,
    "l1_shared_load_kv_scale": 0.5,
    "l1_local_lateral_weight": 0.15,
    "l1_local_projection": "uniform_xy",
    "l1_desired_geometry_mode": "healthy_centered_renormalized",
    "l1_desired_geometry_ramp_time": 0.2,
}


def run_command(command: list[str]) -> None:
    result = subprocess.run(command, check=False, cwd=ROOT, capture_output=True, text=True)
    if result.returncode == 0:
        return

    stdout_tail = "\n".join(result.stdout.splitlines()[-40:])
    stderr_tail = "\n".join(result.stderr.splitlines()[-40:])
    raise RuntimeError(
        "Command failed:\n"
        f"{' '.join(command)}\n\n"
        f"exit code: {result.returncode}\n\n"
        f"stdout tail:\n{stdout_tail}\n\n"
        f"stderr tail:\n{stderr_tail}"
    )


def load_stage_status(output_dir: Path) -> dict[str, object]:
    path = output_dir / STAGE_STATUS_FILE
    if not path.exists():
        return {"generator": "experiments/python/run_unknowns_status_pack.py", "stages": {}}
    return json.loads(path.read_text(encoding="utf-8"))


def timestamp_utc() -> str:
    return datetime.now(timezone.utc).isoformat()


def write_stage_status(output_dir: Path, stage: str, status: str, detail: str = "", started_at: str | None = None, finished_at: str | None = None, duration_seconds: float | None = None) -> None:
    payload = load_stage_status(output_dir)
    stages = payload.setdefault("stages", {})
    stages[stage] = {
        "status": status,
        "detail": detail,
    }
    if started_at is not None:
        stages[stage]["started_at_utc"] = started_at
    if finished_at is not None:
        stages[stage]["finished_at_utc"] = finished_at
    if duration_seconds is not None:
        stages[stage]["duration_seconds"] = duration_seconds
    (output_dir / STAGE_STATUS_FILE).write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def apply_profile_defaults(args: argparse.Namespace) -> None:
    profile = args.profile
    if profile == "smoke":
        args.reference_reduced_order_seeds = "42"
        args.degradation_seeds = "42"
        args.u6_seeds = "42,43"
    elif profile == "no-u6":
        args.stages = [stage for stage in STAGE_ORDER if stage != "u6"] if args.stages is None else args.stages


def parse_stage_list(raw: str) -> list[str]:
    stages = [part.strip() for part in raw.split(",") if part.strip()]
    unknown = [stage for stage in stages if stage not in STAGE_ORDER]
    if unknown:
        raise ValueError(f"Unknown stages: {unknown}. Valid stages are {STAGE_ORDER}")
    return stages


def should_run_stage(stage: str, enabled_stages: set[str] | None) -> bool:
    return enabled_stages is None or stage in enabled_stages


def script_path(name: str) -> Path:
    return ROOT / "experiments" / "python" / name


def run_python_script(script_name: str, *args: str) -> None:
    command = [str(PYTHON), str(script_path(script_name)), *args]
    run_command(command)


def regenerate_full_drake_batch(args: argparse.Namespace) -> None:
    runner_args = ["--manifest", str(args.full_drake_manifest)]
    if args.refresh_full_drake:
        runner_args.insert(0, "--clean")
    if args.render_only_full_drake:
        runner_args.insert(0, "--render-only")
    if args.refresh_full_drake or args.render_only_full_drake:
        run_python_script("run_fault_schedule_batch.py", *runner_args)


def analyze_full_drake_batch(batch_dir: Path, manifest: Path) -> Path:
    run_python_script(
        "analyze_full_drake_batch.py",
        "--batch-dir",
        str(batch_dir),
        "--manifest",
        str(manifest),
    )
    return batch_dir / "comparison_metrics.json"


def output_exists(path: Path | list[Path]) -> bool:
    if isinstance(path, list):
        return all(item.exists() for item in path)
    return path.exists()


def maybe_run(output_dir: Path, stage: str, enabled_stages: set[str] | None, resume: bool, outputs: Path | list[Path], builder: Callable[[], Path | list[Path]]) -> Path | list[Path]:
    if not should_run_stage(stage, enabled_stages):
        if not output_exists(outputs):
            raise FileNotFoundError(f"Stage '{stage}' was skipped but required output is missing: {outputs}")
        write_stage_status(output_dir, stage, "skipped", "Stage skipped by --stages and required outputs already exist", finished_at=timestamp_utc())
        return outputs

    if resume and output_exists(outputs):
        write_stage_status(output_dir, stage, "resumed", "Existing outputs detected and reused", finished_at=timestamp_utc(), duration_seconds=0.0)
        return outputs

    started_at = timestamp_utc()
    stage_start = time.perf_counter()
    write_stage_status(output_dir, stage, "running", "", started_at=started_at)
    try:
        result = builder()
    except Exception as exc:
        write_stage_status(
            output_dir,
            stage,
            "failed",
            str(exc),
            started_at=started_at,
            finished_at=timestamp_utc(),
            duration_seconds=time.perf_counter() - stage_start,
        )
        raise
    write_stage_status(
        output_dir,
        stage,
        "completed",
        "",
        started_at=started_at,
        finished_at=timestamp_utc(),
        duration_seconds=time.perf_counter() - stage_start,
    )
    return result


def build_reduced_order_reference_set(output_dir: Path, manifest: Path, seeds: str) -> list[Path]:
    reduced_paths: list[Path] = []
    reduced_root = output_dir / "reduced_order_reference"
    for scenario in load_scenarios(manifest):
        scenario_output = reduced_root / scenario.name
        run_python_script(
            "run_fault_matrix.py",
            "--scenario",
            scenario.name,
            "--manifest",
            str(manifest),
            "--seeds",
            seeds,
            "--output-dir",
            str(scenario_output),
        )
        reduced_paths.append(scenario_output / "fault_matrix_summary.json")
    return reduced_paths


def build_reduced_vs_drake(output_dir: Path, reduced_paths: list[Path], full_drake_metrics: Path) -> Path:
    comparison_dir = output_dir / "reduced_vs_full_drake"
    run_python_script(
        "compare_reduced_order_to_full_drake.py",
        "--reduced-order",
        *[str(path) for path in reduced_paths],
        "--full-drake",
        str(full_drake_metrics),
        "--output-dir",
        str(comparison_dir),
    )
    return comparison_dir / "reduced_vs_full_drake.json"


def build_u2(output_dir: Path, manifest: Path) -> Path:
    result_path = output_dir / "topology_condition_validation" / "full_drake_reference_conditions.json"
    run_python_script(
        "validate_topology_invariance_conditions.py",
        "--manifest",
        str(manifest),
        "--output",
        str(result_path),
    )
    return result_path


def build_u3(output_dir: Path, manifest: Path, seed: int, wind_sigma: float, forgetting_factor: float) -> Path:
    result_dir = output_dir / "cl_history_corruption"
    run_python_script(
        "analyze_cl_history_corruption.py",
        "--manifest",
        str(manifest),
        "--seed",
        str(seed),
        "--wind-sigma",
        str(wind_sigma),
        "--forgetting-factor",
        str(forgetting_factor),
        "--output-dir",
        str(result_dir),
    )
    return result_dir / "cl_history_corruption.json"


def build_u4_scaling(output_dir: Path, manifest: Path, seeds: str, reduced_vs_drake_path: Path) -> list[Path]:
    scaling_root = output_dir / "topology_scaling_reference"
    outputs: list[Path] = []
    for scenario in load_scenarios(manifest):
        scenario_output = scaling_root / scenario.name
        run_python_script(
            "run_fault_matrix.py",
            "--scenario",
            scenario.name,
            "--manifest",
            str(manifest),
            "--seeds",
            seeds,
            "--output-dir",
            str(scenario_output),
        )
        outputs.append(scenario_output / "fault_matrix_summary.json")
    analysis_dir = output_dir / "topology_scaling_analysis"
    run_python_script(
        "analyze_topology_scaling.py",
        "--reduced-order",
        *[str(path) for path in outputs],
        "--matched-full-drake",
        str(reduced_vs_drake_path),
        "--output-dir",
        str(analysis_dir),
    )
    outputs.append(analysis_dir / "topology_scaling_analysis.json")
    return outputs


def build_u5(output_dir: Path, manifest: Path, seeds: str) -> list[Path]:
    degradation_paths: list[Path] = []
    degradation_root = output_dir / "degradation"
    for scenario in load_scenarios(manifest):
        scenario_output = degradation_root / scenario.name
        run_python_script(
            "run_fault_matrix.py",
            "--scenario",
            scenario.name,
            "--manifest",
            str(manifest),
            "--seeds",
            seeds,
            "--output-dir",
            str(scenario_output),
        )
        degradation_paths.append(scenario_output / "fault_matrix_summary.json")
    return degradation_paths


def build_u6(output_dir: Path, manifest: Path, seeds: str, wind_sigma: float) -> Path:
    result_dir = output_dir / "u6_non_cl_search"
    run_python_script(
        "search_non_cl_adaptive_laws.py",
        "--manifest",
        str(manifest),
        "--seeds",
        seeds,
        "--wind-sigma",
        str(wind_sigma),
        "--output-dir",
        str(result_dir),
    )
    return result_dir / "non_cl_adaptive_search.json"


def build_status_pack(
    output_dir: Path,
    reduced_vs_drake_path: Path,
    topology_conditions_path: Path,
    cl_history_path: Path,
    topology_scaling_path: Path,
    degradation_paths: list[Path],
    u6_search_path: Path,
) -> None:
    run_python_script(
        "build_unknowns_status_report.py",
        "--reduced-vs-drake",
        str(reduced_vs_drake_path),
        "--topology-conditions",
        str(topology_conditions_path),
        "--cl-history",
        str(cl_history_path),
        "--topology-scaling",
        str(topology_scaling_path),
        "--degradation",
        *[str(path) for path in degradation_paths],
        "--non-cl-search",
        str(u6_search_path),
        "--output-dir",
        str(output_dir),
    )


def write_run_manifest(
    output_dir: Path,
    args: argparse.Namespace,
    reduced_paths: list[Path],
    topology_scaling_paths: list[Path],
    degradation_paths: list[Path],
    full_drake_metrics: Path,
    reduced_vs_drake_path: Path,
    topology_conditions_path: Path,
    cl_history_path: Path,
    u4_scaling_path: Path,
    u6_search_path: Path,
) -> None:
    payload = {
        "generator": "experiments/python/run_unknowns_status_pack.py",
        "python": str(PYTHON),
        "profile": args.profile,
        "full_drake_manifest": str(args.full_drake_manifest),
        "degradation_manifest": str(args.degradation_manifest),
        "scaling_manifest": str(args.scaling_manifest),
        "full_drake_batch_dir": str(args.full_drake_batch_dir),
        "refresh_full_drake": args.refresh_full_drake,
        "render_only_full_drake": args.render_only_full_drake,
        "resume": args.resume,
        "stages": list(args.stages) if args.stages is not None else list(STAGE_ORDER),
        "reference_reduced_order_seeds": args.reference_reduced_order_seeds,
        "degradation_seeds": args.degradation_seeds,
        "u3_seed": args.u3_seed,
        "u3_wind_sigma": args.u3_wind_sigma,
        "u3_forgetting_factor": args.u3_forgetting_factor,
        "u6_seeds": args.u6_seeds,
        "u6_wind_sigma": args.u6_wind_sigma,
        "artifacts": {
            "full_drake_metrics": str(full_drake_metrics),
            "reference_reduced_order_summaries": [str(path) for path in reduced_paths],
            "reduced_vs_full_drake": str(reduced_vs_drake_path),
            "topology_conditions": str(topology_conditions_path),
            "cl_history": str(cl_history_path),
            "topology_scaling_summaries": [str(path) for path in topology_scaling_paths[:-1]],
            "topology_scaling_analysis": str(u4_scaling_path),
            "degradation_summaries": [str(path) for path in degradation_paths],
            "u6_non_cl_search": str(u6_search_path),
            "unknowns_status_json": str(output_dir / "unknowns_status.json"),
            "unknowns_status_report": str(output_dir / "unknowns_status_report.md"),
        },
    }
    (output_dir / "run_unknowns_status_pack_manifest.json").write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Regenerate the README unknowns U1-U6 status pack from root-owned experiment and analysis scripts")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--full-drake-batch-dir", type=Path, default=DEFAULT_FULL_DRAKE_BATCH_DIR)
    parser.add_argument("--full-drake-manifest", type=Path, default=DEFAULT_FULL_DRAKE_BATCH_MANIFEST)
    parser.add_argument("--degradation-manifest", type=Path, default=DEFAULT_DEGRADATION_MANIFEST)
    parser.add_argument("--scaling-manifest", type=Path, default=DEFAULT_SCALING_MANIFEST)
    parser.add_argument("--profile", type=str, choices=PROFILE_NAMES, default="report", help="Preset run profile: report, smoke, or no-u6")
    parser.add_argument("--quick", action="store_true", help="Deprecated alias for --profile smoke")
    parser.add_argument("--resume", action="store_true", help="Skip stages whose expected outputs already exist")
    parser.add_argument("--stages", type=str, help=f"Comma-separated subset of stages to run. Valid stages: {','.join(STAGE_ORDER)}")
    parser.add_argument("--refresh-full-drake", action="store_true", help="Rerun the full-Drake batch before analysis")
    parser.add_argument("--render-only-full-drake", action="store_true", help="Regenerate full-Drake artifacts from existing logs before analysis")
    parser.add_argument("--reference-reduced-order-seeds", type=str, default="42")
    parser.add_argument("--degradation-seeds", type=str, default="42")
    parser.add_argument("--u3-seed", type=int, default=42)
    parser.add_argument("--u3-wind-sigma", type=float, default=0.5)
    parser.add_argument("--u3-forgetting-factor", type=float, default=0.95)
    parser.add_argument("--u6-seeds", type=str, default="42,43,44,45,46,47,48,49")
    parser.add_argument("--u6-wind-sigma", type=float, default=0.5)
    args = parser.parse_args()
    if args.refresh_full_drake and args.render_only_full_drake:
        raise ValueError("--refresh-full-drake and --render-only-full-drake cannot be used together")
    args.stages = parse_stage_list(args.stages) if args.stages else None
    if args.quick:
        args.profile = "smoke"
    apply_profile_defaults(args)
    return args


def main() -> None:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    enabled_stages = set(args.stages) if args.stages is not None else None

    regenerate_full_drake_batch(args)
    full_drake_metrics = maybe_run(
        args.output_dir,
        "full_drake_analysis",
        enabled_stages,
        args.resume,
        args.full_drake_batch_dir / "comparison_metrics.json",
        lambda: analyze_full_drake_batch(args.full_drake_batch_dir, args.full_drake_manifest),
    )
    reduced_outputs = [
        args.output_dir / "reduced_order_reference" / scenario.name / "fault_matrix_summary.json"
        for scenario in load_scenarios(args.full_drake_manifest)
    ]
    reduced_paths = maybe_run(
        args.output_dir,
        "reference_reduced_order",
        enabled_stages,
        args.resume,
        reduced_outputs,
        lambda: build_reduced_order_reference_set(args.output_dir, args.full_drake_manifest, args.reference_reduced_order_seeds),
    )
    reduced_vs_drake_path = maybe_run(
        args.output_dir,
        "reduced_vs_drake",
        enabled_stages,
        args.resume,
        args.output_dir / "reduced_vs_full_drake" / "reduced_vs_full_drake.json",
        lambda: build_reduced_vs_drake(args.output_dir, list(reduced_paths), Path(full_drake_metrics)),
    )
    topology_conditions_path = maybe_run(
        args.output_dir,
        "u2",
        enabled_stages,
        args.resume,
        args.output_dir / "topology_condition_validation" / "full_drake_reference_conditions.json",
        lambda: build_u2(args.output_dir, args.full_drake_manifest),
    )
    cl_history_path = maybe_run(
        args.output_dir,
        "u3",
        enabled_stages,
        args.resume,
        args.output_dir / "cl_history_corruption" / "cl_history_corruption.json",
        lambda: build_u3(args.output_dir, args.full_drake_manifest, args.u3_seed, args.u3_wind_sigma, args.u3_forgetting_factor),
    )
    scaling_outputs = [
        args.output_dir / "topology_scaling_reference" / scenario.name / "fault_matrix_summary.json"
        for scenario in load_scenarios(args.scaling_manifest)
    ] + [args.output_dir / "topology_scaling_analysis" / "topology_scaling_analysis.json"]
    scaling_paths = maybe_run(
        args.output_dir,
        "u4_scaling",
        enabled_stages,
        args.resume,
        scaling_outputs,
        lambda: build_u4_scaling(args.output_dir, args.scaling_manifest, args.reference_reduced_order_seeds, Path(reduced_vs_drake_path)),
    )
    degradation_outputs = [
        args.output_dir / "degradation" / scenario.name / "fault_matrix_summary.json"
        for scenario in load_scenarios(args.degradation_manifest)
    ]
    degradation_paths = maybe_run(
        args.output_dir,
        "u5",
        enabled_stages,
        args.resume,
        degradation_outputs,
        lambda: build_u5(args.output_dir, args.degradation_manifest, args.degradation_seeds),
    )
    u6_search_path = maybe_run(
        args.output_dir,
        "u6",
        enabled_stages,
        args.resume,
        args.output_dir / "u6_non_cl_search" / "non_cl_adaptive_search.json",
        lambda: build_u6(
            args.output_dir,
            args.scaling_manifest,
            args.u6_seeds,
            args.u6_wind_sigma,
        ),
    )
    maybe_run(
        args.output_dir,
        "status_pack",
        enabled_stages,
        args.resume,
        [args.output_dir / "unknowns_status.json", args.output_dir / "unknowns_status_report.md"],
        lambda: (
            build_status_pack(
                args.output_dir,
                Path(reduced_vs_drake_path),
                Path(topology_conditions_path),
                Path(cl_history_path),
                Path(list(scaling_paths)[-1]),
                list(degradation_paths),
                Path(u6_search_path),
            )
            or [args.output_dir / "unknowns_status.json", args.output_dir / "unknowns_status_report.md"]
        ),
    )
    maybe_run(
        args.output_dir,
        "run_manifest",
        enabled_stages,
        args.resume,
        args.output_dir / "run_unknowns_status_pack_manifest.json",
        lambda: (
            write_run_manifest(
                args.output_dir,
                args,
                list(reduced_paths),
                list(scaling_paths),
                list(degradation_paths),
                Path(full_drake_metrics),
                Path(reduced_vs_drake_path),
                Path(topology_conditions_path),
                Path(cl_history_path),
                Path(list(scaling_paths)[-1]),
                Path(u6_search_path),
            )
            or (args.output_dir / "run_unknowns_status_pack_manifest.json")
        ),
    )


if __name__ == "__main__":
    main()