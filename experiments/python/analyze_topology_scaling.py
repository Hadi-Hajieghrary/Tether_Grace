#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def find_mode(payload: dict[str, Any], mode: str) -> dict[str, Any]:
    for row in payload.get("aggregates", []):
        if row.get("mode") == mode:
            return row
    return {}


def monotone_nonincreasing(values: list[float]) -> bool:
    return all(curr <= prev + 1e-9 for prev, curr in zip(values, values[1:]))


def slope_per_agent(x_values: list[int], y_values: list[float]) -> float | None:
    if len(x_values) < 2:
        return None
    coefficients = np.polyfit(np.array(x_values, dtype=float), np.array(y_values, dtype=float), 1)
    return float(coefficients[0])


def build_reduced_rows(paths: list[Path]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for path in paths:
        payload = load_json(path)
        config = payload.get("config", {})
        cl_row = find_mode(payload, "cl")
        l1_row = find_mode(payload, "l1")
        rows.append(
            {
                "scenario": str(config.get("scenario", path.parent.name)),
                "num_agents": int(config.get("num_agents", 0)),
                "cl_mean_post_fault_rmse_m": float(cl_row.get("mean_post_fault_rmse", 0.0)),
                "cl_mean_peak_deviation_m": float(cl_row.get("mean_peak_deviation", 0.0)),
                "cl_cable_spread_m": float(cl_row.get("cable_spread", 0.0)),
                "l1_mean_post_fault_rmse_m": float(l1_row.get("mean_post_fault_rmse", 0.0)),
                "l1_mean_peak_deviation_m": float(l1_row.get("mean_peak_deviation", 0.0)),
                "l1_cable_spread_m": float(l1_row.get("cable_spread", 0.0)),
            }
        )
    return sorted(rows, key=lambda item: int(item["num_agents"]))


def build_full_drake_rows(path: Path | None) -> list[dict[str, Any]]:
    if path is None:
        return []
    payload = load_json(path)
    rows = []
    for comparison in payload.get("comparisons", []):
        rows.append(
            {
                "scenario": str(comparison["scenario"]),
                "num_agents": int(comparison["num_agents"]),
                "reduced_cl_post_fault_rmse_m": float(comparison["reduced_order"]["cl_mean_post_fault_rmse_m"]),
                "full_drake_load_rmse_m": float(comparison["full_drake"]["load_rmse_m"]),
                "full_to_reduced_cl_ratio": float(comparison["full_drake"]["load_rmse_m"]) / max(float(comparison["reduced_order"]["cl_mean_post_fault_rmse_m"]), 1e-9),
            }
        )
    return sorted(rows, key=lambda item: int(item["num_agents"]))


def write_markdown(output_dir: Path, reduced_rows: list[dict[str, Any]], full_drake_rows: list[dict[str, Any]], diagnostics: dict[str, Any]) -> None:
    lines = [
        "# Topology Scaling Analysis",
        "",
        "## Reduced-Order Scaling",
        "",
        f"- CL post-fault RMSE monotone with N: {str(diagnostics['reduced_order']['cl_post_monotone_with_n']).lower()}",
        f"- CL peak monotone with N: {str(diagnostics['reduced_order']['cl_peak_monotone_with_n']).lower()}",
        f"- CL spread monotone with N: {str(diagnostics['reduced_order']['cl_spread_monotone_with_n']).lower()}",
        "",
        "| Scenario | N | CL post [cm] | CL peak [cm] | CL spread [cm] | L1 post [cm] |",
        "| --- | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in reduced_rows:
        lines.append(
            f"| {row['scenario']} | {row['num_agents']} | {row['cl_mean_post_fault_rmse_m'] * 100.0:.2f} | {row['cl_mean_peak_deviation_m'] * 100.0:.2f} | {row['cl_cable_spread_m'] * 100.0:.2f} | {row['l1_mean_post_fault_rmse_m'] * 100.0:.2f} |"
        )

    if full_drake_rows:
        lines.extend([
            "",
            "## Matched Full-Drake Trend",
            "",
            f"- Full-Drake matched RMSE monotone with N: {str(diagnostics['matched_full_drake']['full_drake_monotone_with_n']).lower()}",
            "",
            "| Scenario | N | Reduced CL post [cm] | Full Drake load RMSE [cm] | Ratio |",
            "| --- | ---: | ---: | ---: | ---: |",
        ])
        for row in full_drake_rows:
            lines.append(
                f"| {row['scenario']} | {row['num_agents']} | {row['reduced_cl_post_fault_rmse_m'] * 100.0:.2f} | {row['full_drake_load_rmse_m'] * 100.0:.2f} | {row['full_to_reduced_cl_ratio']:.2f} |"
            )

    (output_dir / "topology_scaling_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description="Analyze reduced-order topology scaling and compare the trend against matched full-Drake references")
    parser.add_argument("--reduced-order", type=Path, nargs="+", required=True)
    parser.add_argument("--matched-full-drake", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()

    args.output_dir.mkdir(parents=True, exist_ok=True)
    reduced_rows = build_reduced_rows(args.reduced_order)
    full_drake_rows = build_full_drake_rows(args.matched_full_drake)

    reduced_agents = [int(row["num_agents"]) for row in reduced_rows]
    reduced_cl_post = [float(row["cl_mean_post_fault_rmse_m"]) for row in reduced_rows]
    reduced_cl_peak = [float(row["cl_mean_peak_deviation_m"]) for row in reduced_rows]
    reduced_cl_spread = [float(row["cl_cable_spread_m"]) for row in reduced_rows]
    full_drake_series = [float(row["full_drake_load_rmse_m"]) for row in full_drake_rows]

    payload = {
        "generator": "experiments/python/analyze_topology_scaling.py",
        "reduced_order_rows": reduced_rows,
        "matched_full_drake_rows": full_drake_rows,
        "reduced_order": {
            "cl_post_monotone_with_n": monotone_nonincreasing(reduced_cl_post),
            "cl_peak_monotone_with_n": monotone_nonincreasing(reduced_cl_peak),
            "cl_spread_monotone_with_n": monotone_nonincreasing(reduced_cl_spread),
            "cl_post_slope_m_per_agent": slope_per_agent(reduced_agents, reduced_cl_post),
            "cl_peak_slope_m_per_agent": slope_per_agent(reduced_agents, reduced_cl_peak),
            "cl_spread_slope_m_per_agent": slope_per_agent(reduced_agents, reduced_cl_spread),
            "cl_post_improvement_first_to_last_m": float(reduced_cl_post[0] - reduced_cl_post[-1]) if len(reduced_cl_post) >= 2 else None,
        },
        "matched_full_drake": {
            "full_drake_monotone_with_n": monotone_nonincreasing(full_drake_series) if full_drake_series else False,
            "full_drake_rmse_slope_m_per_agent": slope_per_agent([int(row["num_agents"]) for row in full_drake_rows], full_drake_series),
            "worst_full_to_reduced_ratio": max((float(row["full_to_reduced_cl_ratio"]) for row in full_drake_rows), default=None),
        },
    }

    (args.output_dir / "topology_scaling_analysis.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")
    write_markdown(args.output_dir, reduced_rows, full_drake_rows, payload)
    print(json.dumps(payload, indent=2))


if __name__ == "__main__":
    main()