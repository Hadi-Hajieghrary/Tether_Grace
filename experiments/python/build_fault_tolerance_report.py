#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path


def cm_to_m(value_cm: float | None) -> float | None:
    if value_cm is None:
        return None
    return value_cm / 100.0


def candidate_meets_constraints(
    candidate: dict,
    max_spread_delta_m: float | None,
    max_peak_delta_m: float | None,
    min_rmse_gain_m: float | None,
) -> bool:
    if max_spread_delta_m is not None and candidate["spread_delta_vs_cl"] > max_spread_delta_m:
        return False
    if max_peak_delta_m is not None and candidate["peak_delta_vs_cl"] > max_peak_delta_m:
        return False
    if min_rmse_gain_m is not None and (-candidate["delta_vs_cl"]) < min_rmse_gain_m:
        return False
    return True


def rank_scenarios(rows: list[dict], metric_key: str) -> list[dict]:
    grouped: dict[tuple[float, float], dict[str, dict]] = {}
    for row in rows:
        key = (row["seed"], row["fault_cable"])
        grouped.setdefault(key, {})[row["mode"]] = row

    ranked: list[dict] = []
    for (seed, fault_cable), values in grouped.items():
        cl_row = values.get("cl")
        l1_row = values.get("l1")
        if cl_row is None or l1_row is None:
            continue
        ranked.append(
            {
                "seed": seed,
                "fault_cable": fault_cable,
                "cl": cl_row[metric_key],
                "l1": l1_row[metric_key],
                "delta": l1_row[metric_key] - cl_row[metric_key],
                "winner": "L1" if l1_row[metric_key] < cl_row[metric_key] else "CL",
            }
        )

    ranked.sort(key=lambda item: (item["delta"], item["seed"], item["fault_cable"]))
    return ranked


def summarize_candidate_table(candidates: list[dict], limit: int) -> list[str]:
    lines = [
        "| omega | pole | sigma max | Mean post-fault RMSE [cm] | Spread [cm] | Peak [cm] | Delta vs CL [cm] | Tradeoff score [cm] |",
        "| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for candidate in candidates[:limit]:
        lines.append(
            f"| {candidate['omega']:.1f} | {candidate['predictor_pole']:.1f} | {candidate['sigma_max']:.1f} | "
            f"{candidate['mean_post_fault_rmse'] * 100.0:.2f} | {candidate['cable_spread'] * 100.0:.2f} | "
            f"{candidate['mean_peak_deviation'] * 100.0:.2f} | {candidate['delta_vs_cl'] * 100.0:.2f} | "
            f"{candidate.get('tradeoff_score', 0.0) * 100.0:.2f} |"
        )
    return lines


def summarize_constraints(args: argparse.Namespace) -> list[str]:
    lines = ["## Constraints", ""]
    lines.append(
        f"- Minimum RMSE gain vs CL: {args.min_rmse_gain_cm:.2f} cm" if args.min_rmse_gain_cm is not None else "- Minimum RMSE gain vs CL: none"
    )
    lines.append(
        f"- Maximum spread penalty vs CL: {args.max_spread_delta_cm:.2f} cm" if args.max_spread_delta_cm is not None else "- Maximum spread penalty vs CL: none"
    )
    lines.append(
        f"- Maximum peak delta vs CL: {args.max_peak_delta_cm:.2f} cm" if args.max_peak_delta_cm is not None else "- Maximum peak delta vs CL: none"
    )
    return lines


def summarize_ranked_scenarios(title: str, rows: list[dict], limit: int) -> list[str]:
    lines = [
        title,
        "",
        "| Seed | Cable | CL [cm] | L1 [cm] | L1-CL [cm] | Winner |",
        "| ---: | ---: | ---: | ---: | ---: | --- |",
    ]
    for row in rows[:limit]:
        lines.append(
            f"| {int(row['seed'])} | {int(row['fault_cable'])} | {row['cl'] * 100.0:.2f} | "
            f"{row['l1'] * 100.0:.2f} | {row['delta'] * 100.0:.2f} | {row['winner']} |"
        )
    return lines


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def main() -> None:
    parser = argparse.ArgumentParser(description="Build a markdown comparison report from root-owned CL-vs-L1 experiment artifacts")
    parser.add_argument("--matrix-sweep", type=Path, required=True)
    parser.add_argument("--fault-matrix", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--top-candidates", type=int, default=5)
    parser.add_argument("--top-scenarios", type=int, default=10)
    parser.add_argument("--min-rmse-gain-cm", type=float)
    parser.add_argument("--max-spread-delta-cm", type=float)
    parser.add_argument("--max-peak-delta-cm", type=float)
    args = parser.parse_args()

    matrix_sweep = load_json(args.matrix_sweep)
    fault_matrix = load_json(args.fault_matrix)

    best_l1 = matrix_sweep.get("best_l1")
    best_balanced = matrix_sweep.get("best_balanced")
    candidates = matrix_sweep.get("candidates", [])
    pareto = matrix_sweep.get("pareto_front", [])
    max_spread_delta_m = cm_to_m(args.max_spread_delta_cm)
    max_peak_delta_m = cm_to_m(args.max_peak_delta_cm)
    min_rmse_gain_m = cm_to_m(args.min_rmse_gain_cm)
    feasible_candidates = [
        candidate
        for candidate in candidates
        if candidate_meets_constraints(candidate, max_spread_delta_m, max_peak_delta_m, min_rmse_gain_m)
    ]
    feasible_candidates.sort(
        key=lambda item: (item.get("tradeoff_score", 0.0), item["mean_post_fault_rmse"], item["spread_delta_vs_cl"])
    )
    aggregates = {row["mode"]: row for row in fault_matrix.get("aggregates", [])}
    cl = aggregates.get("cl")
    l1 = aggregates.get("l1")
    rows = fault_matrix.get("rows", [])
    ranked_post_fault = rank_scenarios(rows, "post_fault_rmse")
    ranked_peak = rank_scenarios(rows, "peak_deviation")

    lines = [
        "# CL vs L1 Fault-Tolerance Report",
        "",
        "## Inputs",
        "",
        f"- Matrix sweep: {args.matrix_sweep}",
        f"- Fault matrix: {args.fault_matrix}",
        "",
        "## Best L1 Candidate",
        "",
    ]

    if best_l1 is None:
        lines.append("- No L1 candidate was found in the supplied sweep.")
    else:
        lines.extend(
            [
                f"- omega: {best_l1['omega']}",
                f"- predictor pole: {best_l1['predictor_pole']}",
                f"- sigma max: {best_l1['sigma_max']}",
                f"- mean post-fault RMSE: {best_l1['mean_post_fault_rmse'] * 100.0:.2f} cm",
                f"- cable spread: {best_l1['cable_spread'] * 100.0:.2f} cm",
                f"- delta vs CL: {best_l1['delta_vs_cl'] * 100.0:.2f} cm",
            ]
        )

    if candidates:
        lines.extend(
            [
                "",
                "## Top L1 Candidates",
                "",
                *summarize_candidate_table(candidates, args.top_candidates),
            ]
        )

        if args.min_rmse_gain_cm is not None or args.max_spread_delta_cm is not None or args.max_peak_delta_cm is not None:
                lines.extend(["", *summarize_constraints(args), ""])
                lines.append("## Best Feasible Candidate")
                lines.append("")
                if feasible_candidates:
                    best_feasible = feasible_candidates[0]
                    lines.extend(
                            [
                                    f"- omega: {best_feasible['omega']}",
                                    f"- predictor pole: {best_feasible['predictor_pole']}",
                                    f"- sigma max: {best_feasible['sigma_max']}",
                                    f"- mean post-fault RMSE: {best_feasible['mean_post_fault_rmse'] * 100.0:.2f} cm",
                                    f"- RMSE gain vs CL: {(-best_feasible['delta_vs_cl']) * 100.0:.2f} cm",
                                    f"- spread penalty vs CL: {best_feasible['spread_delta_vs_cl'] * 100.0:.2f} cm",
                                    f"- peak delta vs CL: {best_feasible['peak_delta_vs_cl'] * 100.0:.2f} cm",
                                    f"- tradeoff score: {best_feasible.get('tradeoff_score', 0.0) * 100.0:.2f} cm-equivalent",
                            ]
                    )
                    lines.extend(["", "## Feasible Candidates", "", *summarize_candidate_table(feasible_candidates, args.top_candidates)])
                else:
                    lines.append("- No candidate satisfied the supplied constraints.")

    if best_balanced is not None:
        lines.extend(
            [
                "",
                "## Best Balanced Candidate",
                "",
                f"- omega: {best_balanced['omega']}",
                f"- predictor pole: {best_balanced['predictor_pole']}",
                f"- sigma max: {best_balanced['sigma_max']}",
                f"- mean post-fault RMSE: {best_balanced['mean_post_fault_rmse'] * 100.0:.2f} cm",
                f"- cable spread: {best_balanced['cable_spread'] * 100.0:.2f} cm",
                f"- mean peak deviation: {best_balanced['mean_peak_deviation'] * 100.0:.2f} cm",
                f"- tradeoff score: {best_balanced.get('tradeoff_score', 0.0) * 100.0:.2f} cm-equivalent",
            ]
        )

    if pareto:
        lines.extend(
            [
                "",
                "## Pareto Front",
                "",
                *summarize_candidate_table(pareto, min(args.top_candidates, len(pareto))),
            ]
        )

    lines.extend(
        [
            "",
            "## Aggregate Comparison",
            "",
            "| Mode | Mean post-fault RMSE [cm] | Mean fault delta [cm] | Mean peak [cm] | Spread [cm] |",
            "| --- | ---: | ---: | ---: | ---: |",
        ]
    )

    for key, row in [("CL", cl), ("L1", l1)]:
        if row is None:
            continue
        lines.append(
            f"| {key} | {row['mean_post_fault_rmse'] * 100.0:.2f} | {row['mean_fault_delta'] * 100.0:.2f} | "
            f"{row['mean_peak_deviation'] * 100.0:.2f} | {row['cable_spread'] * 100.0:.2f} |"
        )

    if ranked_post_fault:
        lines.extend(["", *summarize_ranked_scenarios("## Scenario Ranking by Post-Fault RMSE", ranked_post_fault, args.top_scenarios)])

    if ranked_peak:
        lines.extend(["", *summarize_ranked_scenarios("## Scenario Ranking by Peak Deviation", ranked_peak, args.top_scenarios)])

    lines.extend(["", "## Recommendation", ""])
    if cl is not None and l1 is not None:
        if l1["mean_post_fault_rmse"] < cl["mean_post_fault_rmse"]:
            lines.append("- L1 currently outperforms CL on mean post-fault RMSE in the supplied matrix results.")
        else:
            lines.append("- CL currently remains better than the tested L1 candidate on mean post-fault RMSE.")
        if l1["mean_peak_deviation"] < cl["mean_peak_deviation"]:
            lines.append("- L1 currently improves peak deviation on the supplied matrix results.")
        else:
            lines.append("- CL currently remains better or equal on peak deviation.")
        if l1["cable_spread"] < cl["cable_spread"]:
            lines.append("- L1 shows better topology-invariance spread than CL on the supplied matrix results.")
        else:
            lines.append("- CL currently remains better or equal on topology-invariance spread.")
    else:
        lines.append("- The supplied artifacts did not contain both CL and L1 aggregate summaries.")

    args.output.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"Wrote report to {args.output}")


if __name__ == "__main__":
    main()
