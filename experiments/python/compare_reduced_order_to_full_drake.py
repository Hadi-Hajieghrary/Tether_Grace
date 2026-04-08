#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def reduced_summary_by_name(paths: list[Path]) -> dict[str, dict[str, Any]]:
    mapping: dict[str, dict[str, Any]] = {}
    for path in paths:
        payload = load_json(path)
        config = payload.get("config", {})
        scenario_name = str(config.get("scenario", "")).strip()
        if not scenario_name:
            scenario_name = f"num_agents_{int(config.get('num_agents', 0))}"
        mapping[scenario_name] = payload
    return mapping


def summarize_comparison(reduced_payload: dict[str, Any], full_payload: dict[str, Any]) -> dict[str, Any]:
    reduced_aggregates = {row["mode"]: row for row in reduced_payload.get("aggregates", [])}
    full_summary = full_payload
    reduced_cl = reduced_aggregates.get("cl", {})
    reduced_l1 = reduced_aggregates.get("l1", {})
    return {
        "scenario": str(full_summary["scenario"]),
        "num_agents": int(full_summary["num_agents"]),
        "reduced_order": {
            "scenario": reduced_payload.get("config", {}).get("scenario", ""),
            "cl_mean_post_fault_rmse_m": float(reduced_cl.get("mean_post_fault_rmse", 0.0)),
            "l1_mean_post_fault_rmse_m": float(reduced_l1.get("mean_post_fault_rmse", 0.0)),
            "cl_mean_peak_deviation_m": float(reduced_cl.get("mean_peak_deviation", 0.0)),
            "l1_mean_peak_deviation_m": float(reduced_l1.get("mean_peak_deviation", 0.0)),
            "cl_cable_spread_m": float(reduced_cl.get("cable_spread", 0.0)),
            "l1_cable_spread_m": float(reduced_l1.get("cable_spread", 0.0)),
        },
        "full_drake": {
            "load_rmse_m": float(full_summary["load_rmse_m"]),
            "post_first_fault_peak_error_m": float(full_summary["post_first_fault_peak_error_m"]),
            "final_load_error_m": float(full_summary["final_load_error_m"]),
            "load_peak_speed_mps": float(full_summary["load_peak_speed_mps"]),
        },
    }


def write_report(output_dir: Path, comparisons: list[dict[str, Any]]) -> None:
    lines = [
        "# Reduced-Order vs Full-Drake Comparison",
        "",
        "| Scenario | N | Reduced CL post [cm] | Reduced L1 post [cm] | Drake load RMSE [cm] | Drake post-first-fault peak [cm] |",
        "| --- | ---: | ---: | ---: | ---: | ---: |",
    ]
    for item in comparisons:
        reduced = item["reduced_order"]
        full = item["full_drake"]
        lines.append(
            f"| {item['scenario']} | {item['num_agents']} | {reduced['cl_mean_post_fault_rmse_m'] * 100.0:.2f} | "
            f"{reduced['l1_mean_post_fault_rmse_m'] * 100.0:.2f} | {full['load_rmse_m'] * 100.0:.2f} | {full['post_first_fault_peak_error_m'] * 100.0:.2f} |"
        )
    (output_dir / "reduced_vs_full_drake_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description="Compare manifest-matched reduced-order summaries against full-Drake batch metrics")
    parser.add_argument("--reduced-order", type=Path, nargs="+", required=True)
    parser.add_argument("--full-drake", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()

    args.output_dir.mkdir(parents=True, exist_ok=True)
    reduced_map = reduced_summary_by_name(args.reduced_order)
    full_payload = load_json(args.full_drake)
    comparisons: list[dict[str, Any]] = []
    missing_reduced: list[str] = []

    for summary in full_payload.get("scenario_summaries", []):
        scenario_name = str(summary["scenario"])
        reduced_payload = reduced_map.get(scenario_name)
        if reduced_payload is None:
            missing_reduced.append(scenario_name)
            continue
        comparisons.append(summarize_comparison(reduced_payload, summary))

    payload = {
        "generator": "experiments/python/compare_reduced_order_to_full_drake.py",
        "comparisons": comparisons,
        "missing_reduced_summaries": missing_reduced,
    }
    (args.output_dir / "reduced_vs_full_drake.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")
    write_report(args.output_dir, comparisons)
    print(json.dumps(payload, indent=2))


if __name__ == "__main__":
    main()