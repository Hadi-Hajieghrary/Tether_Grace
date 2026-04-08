#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


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


def summarize_table(candidates: list[dict], limit: int) -> list[str]:
    lines = [
        "| omega | pole | sigma max | RMSE [cm] | RMSE gain [cm] | Spread penalty [cm] | Peak delta [cm] | Tradeoff [cm] |",
        "| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for candidate in candidates[:limit]:
        lines.append(
            f"| {candidate['omega']:.2f} | {candidate['predictor_pole']:.2f} | {candidate['sigma_max']:.2f} | "
            f"{candidate['mean_post_fault_rmse'] * 100.0:.2f} | {(-candidate['delta_vs_cl']) * 100.0:.2f} | "
            f"{candidate['spread_delta_vs_cl'] * 100.0:.2f} | {candidate['peak_delta_vs_cl'] * 100.0:.2f} | "
            f"{candidate.get('tradeoff_score', 0.0) * 100.0:.2f} |"
        )
    return lines


def main() -> None:
    parser = argparse.ArgumentParser(description="Select L1 candidates from a matrix sweep using explicit RMSE, spread, and peak constraints")
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--top-count", type=int, default=10)
    parser.add_argument("--sort-by", choices=["rmse", "spread", "peak", "tradeoff"], default="tradeoff")
    parser.add_argument("--min-rmse-gain-cm", type=float)
    parser.add_argument("--max-spread-delta-cm", type=float)
    parser.add_argument("--max-peak-delta-cm", type=float)
    args = parser.parse_args()

    payload = load_json(args.input)
    candidates = payload.get("candidates", [])
    max_spread_delta_m = cm_to_m(args.max_spread_delta_cm)
    max_peak_delta_m = cm_to_m(args.max_peak_delta_cm)
    min_rmse_gain_m = cm_to_m(args.min_rmse_gain_cm)

    feasible = [
        candidate
        for candidate in candidates
        if candidate_meets_constraints(candidate, max_spread_delta_m, max_peak_delta_m, min_rmse_gain_m)
    ]

    sort_map = {
        "rmse": lambda item: (item["mean_post_fault_rmse"], item["cable_spread"], item["mean_peak_deviation"]),
        "spread": lambda item: (item["spread_delta_vs_cl"], item["mean_post_fault_rmse"], item["peak_delta_vs_cl"]),
        "peak": lambda item: (item["peak_delta_vs_cl"], item["mean_post_fault_rmse"], item["spread_delta_vs_cl"]),
        "tradeoff": lambda item: (item.get("tradeoff_score", 0.0), item["mean_post_fault_rmse"], item["spread_delta_vs_cl"]),
    }
    feasible.sort(key=sort_map[args.sort_by])

    lines = [
        "# L1 Candidate Selection Report",
        "",
        f"- Input sweep: {args.input}",
        f"- Sort mode: {args.sort_by}",
        f"- Feasible candidates: {len(feasible)} / {len(candidates)}",
        "",
        *summarize_constraints(args),
        "",
    ]
    if feasible:
        best = feasible[0]
        lines.extend(
            [
                "## Best Feasible Candidate",
                "",
                f"- omega: {best['omega']}",
                f"- predictor pole: {best['predictor_pole']}",
                f"- sigma max: {best['sigma_max']}",
                f"- mean post-fault RMSE: {best['mean_post_fault_rmse'] * 100.0:.2f} cm",
                f"- RMSE gain vs CL: {(-best['delta_vs_cl']) * 100.0:.2f} cm",
                f"- spread penalty vs CL: {best['spread_delta_vs_cl'] * 100.0:.2f} cm",
                f"- peak delta vs CL: {best['peak_delta_vs_cl'] * 100.0:.2f} cm",
                f"- tradeoff score: {best.get('tradeoff_score', 0.0) * 100.0:.2f} cm-equivalent",
                "",
                "## Feasible Candidates",
                "",
                *summarize_table(feasible, args.top_count),
            ]
        )
    else:
        lines.extend([
            "## Best Feasible Candidate",
            "",
            "- No candidate satisfied the supplied constraints.",
        ])

    args.output.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"Wrote selection report to {args.output}")


if __name__ == "__main__":
    main()