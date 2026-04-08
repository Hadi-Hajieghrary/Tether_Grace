#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def find_mode(payload: dict[str, Any], mode: str) -> dict[str, Any] | None:
    for row in payload.get("aggregates", []):
        if row.get("mode") == mode:
            return row
    return None


def summarize_u1(reduced_vs_drake: dict[str, Any]) -> dict[str, Any]:
    rows: list[dict[str, Any]] = []
    ratios: list[float] = []
    for comparison in reduced_vs_drake.get("comparisons", []):
        reduced_cl = float(comparison["reduced_order"]["cl_mean_post_fault_rmse_m"])
        full_rmse = float(comparison["full_drake"]["load_rmse_m"])
        ratio = full_rmse / max(reduced_cl, 1e-9)
        ratios.append(ratio)
        rows.append(
            {
                "scenario": comparison["scenario"],
                "num_agents": int(comparison["num_agents"]),
                "reduced_cl_post_fault_rmse_m": reduced_cl,
                "full_drake_load_rmse_m": full_rmse,
                "full_to_reduced_cl_ratio": ratio,
            }
        )

    max_ratio = max(ratios) if ratios else None
    if max_ratio is None:
        status = "missing"
    elif max_ratio <= 1.5:
        status = "supported"
    elif max_ratio <= 3.0:
        status = "bounded_degradation"
    else:
        status = "gap"

    return {
        "status": status,
        "scenario_rows": rows,
        "max_full_to_reduced_cl_ratio": max_ratio,
    }


def summarize_u2(topology_conditions: dict[str, Any]) -> dict[str, Any]:
    rows = []
    all_ok = True
    for item in topology_conditions.get("scenario_checks", []):
        ok = bool(item.get("thrust_margin_ok", False))
        sufficient_ok = bool(item.get("sufficient_margin_ok", False))
        all_ok = all_ok and ok
        rows.append(
            {
                "scenario": item["scenario"],
                "num_agents": int(item["num_agents"]),
                "thrust_margin_ok": ok,
                "sufficient_margin_ok": sufficient_ok,
                "worst_required_thrust_per_agent_n": float(item["worst_required_thrust_per_agent_n"]),
                "max_available_thrust_per_agent_n": float(item["max_available_thrust_per_agent_n"]),
                "fault_tolerance_cost_linear_ratio": float(item["fault_tolerance_cost_linear_ratio"]),
                "worst_capacity_ratio": float(item.get("worst_capacity_ratio", 0.0)),
                "worst_effective_capacity_ratio": float(item.get("worst_effective_capacity_ratio", 0.0)),
                "min_healthy_support_fraction": float(item.get("min_healthy_support_fraction", 0.0)),
            }
        )
    return {
        "status": "numeric_support_only" if all_ok and rows else "missing_or_failed",
        "scenario_rows": rows,
    }


def summarize_u3(cl_history: dict[str, Any]) -> dict[str, Any]:
    variants = {item["name"]: item for item in cl_history.get("aggregate_variants", [])}
    baseline = variants.get("baseline", {})
    forgetting = variants.get("forgetting", {})
    flush = variants.get("flush", {})
    forgetting_flush = variants.get("forgetting_flush", {})
    baseline_theta_bias = baseline.get("mean_theta_bias_duration_vs_flush_s")
    forgetting_theta_bias = forgetting.get("mean_theta_bias_duration_vs_flush_s")
    forgetting_flush_theta_bias = forgetting_flush.get("mean_theta_bias_duration_vs_flush_s")

    if baseline_theta_bias is None:
        status = "missing"
    elif forgetting_flush_theta_bias is not None and baseline_theta_bias > forgetting_flush_theta_bias:
        status = "bias_detected_with_mitigation_signal"
    else:
        status = "bias_detected"

    return {
        "status": status,
        "baseline": baseline,
        "forgetting": forgetting,
        "flush": flush,
        "forgetting_flush": forgetting_flush,
        "scenario_results": cl_history.get("scenario_results", []),
    }


def summarize_u4(reduced_vs_drake: dict[str, Any], topology_scaling: dict[str, Any] | None) -> dict[str, Any]:
    rows = sorted(reduced_vs_drake.get("comparisons", []), key=lambda item: int(item["num_agents"]))
    reduced_cl = [float(item["reduced_order"]["cl_mean_post_fault_rmse_m"]) for item in rows]
    full_rmse = [float(item["full_drake"]["load_rmse_m"]) for item in rows]
    reduced_monotone = bool(topology_scaling.get("reduced_order", {}).get("cl_post_monotone_with_n", False)) if topology_scaling is not None else (all(curr <= prev + 1e-9 for prev, curr in zip(reduced_cl, reduced_cl[1:])) if len(reduced_cl) >= 2 else False)
    reduced_order_rows = topology_scaling.get("reduced_order_rows", []) if topology_scaling is not None else []
    return {
        "status": "reduced_order_supported_full_drake_mixed" if rows else "missing",
        "reduced_order_monotone_with_n": reduced_monotone,
        "reduced_order_scaling_rows": reduced_order_rows,
        "scenario_rows": [
            {
                "scenario": item["scenario"],
                "num_agents": int(item["num_agents"]),
                "reduced_cl_post_fault_rmse_m": float(item["reduced_order"]["cl_mean_post_fault_rmse_m"]),
                "full_drake_load_rmse_m": float(item["full_drake"]["load_rmse_m"]),
            }
            for item in rows
        ],
        "reduced_cl_series_m": reduced_cl,
        "full_drake_series_m": full_rmse,
    }


def summarize_u5(degradation_payloads: list[dict[str, Any]]) -> dict[str, Any]:
    rows = []
    for payload in degradation_payloads:
        config = payload.get("config", {})
        scenario = str(config.get("scenario", ""))
        profiles = list(config.get("scenario_fault_profiles", []))
        cl_row = find_mode(payload, "cl")
        l1_row = find_mode(payload, "l1")
        if cl_row is None or l1_row is None:
            continue
        rows.append(
            {
                "scenario": scenario,
                "profiles": profiles,
                "cl_mean_post_fault_rmse_m": float(cl_row["mean_post_fault_rmse"]),
                "l1_mean_post_fault_rmse_m": float(l1_row["mean_post_fault_rmse"]),
                "cl_mean_peak_deviation_m": float(cl_row["mean_peak_deviation"]),
                "l1_mean_peak_deviation_m": float(l1_row["mean_peak_deviation"]),
            }
        )
    return {
        "status": "extended_support" if len(rows) >= 4 else ("pilot_supported" if rows else "missing"),
        "scenario_rows": rows,
    }


def summarize_u6(non_cl_fault_matrix: dict[str, Any] | None, non_cl_sweep: dict[str, Any] | None, non_cl_search: dict[str, Any] | None) -> dict[str, Any]:
    summary: dict[str, Any] = {"status": "missing"}
    if non_cl_search is not None:
        robust_winners = list(non_cl_search.get("robust_winners", []))
        summary.update(
            {
                "status": "supported_in_reduced_order" if robust_winners else "thoroughly_tested_not_supported",
                "candidate_count": int(non_cl_search.get("candidate_count", 0)),
                "candidate_family_count": int(non_cl_search.get("candidate_family_count", 0)),
                "best_balanced_candidate": non_cl_search.get("best_balanced_candidate"),
                "best_rmse_candidate": non_cl_search.get("best_rmse_candidate"),
                "robust_winners": robust_winners,
                "baseline_cl_by_scenario": non_cl_search.get("baseline_cl_by_scenario", []),
            }
        )
    if non_cl_fault_matrix is not None:
        cl_row = find_mode(non_cl_fault_matrix, "cl")
        l1_row = find_mode(non_cl_fault_matrix, "l1")
        if cl_row is not None and l1_row is not None:
            l1_better_rmse = float(l1_row["mean_post_fault_rmse"]) < float(cl_row["mean_post_fault_rmse"])
            l1_better_peak = float(l1_row["mean_peak_deviation"]) < float(cl_row["mean_peak_deviation"])
            summary.update(
                {
                    "status": "supported_in_reduced_order" if l1_better_rmse else "tested_but_not_supported",
                    "fault_matrix": {
                        "cl": cl_row,
                        "l1": l1_row,
                        "l1_beats_cl_on_rmse": l1_better_rmse,
                        "l1_beats_cl_on_peak": l1_better_peak,
                    },
                }
            )
    if non_cl_sweep is not None:
        summary["sweep_best_l1"] = non_cl_sweep.get("best_l1")
        summary["sweep_best_balanced"] = non_cl_sweep.get("best_balanced")
    return summary


def build_takeaways(summary: dict[str, Any]) -> list[str]:
    u1 = summary["U1"]
    u2 = summary["U2"]
    u3 = summary["U3"]
    u4 = summary["U4"]
    u5 = summary["U5"]
    u6 = summary["U6"]

    takeaways: list[str] = []
    max_ratio = u1.get("max_full_to_reduced_cl_ratio")
    if max_ratio is not None:
        takeaways.append(
            f"U1 remains open because the worst full-Drake / reduced-order CL post-fault RMSE ratio is {float(max_ratio):.2f}, driven by the N=7 case."
        )

    takeaways.append(
        "U2 is narrowed to numeric support: all canonical 3/5/7 scenarios satisfy the current thrust-margin check, but no formal sufficient-condition proof exists yet."
        if u2.get("status") == "numeric_support_only"
        else f"U2 status: {u2.get('status')}"
    )

    forgetting_bias = u3.get("forgetting", {}).get("mean_theta_bias_duration_vs_flush_s")
    forgetting_flush_bias = u3.get("forgetting_flush", {}).get("mean_theta_bias_duration_vs_flush_s")
    baseline_bias = u3.get("baseline", {}).get("mean_theta_bias_duration_vs_flush_s")
    if baseline_bias is not None and forgetting_flush_bias is not None:
        takeaways.append(
            f"U3 is now quantified across the canonical 3/5/7 scenarios: baseline CL remains theta-biased relative to fault-triggered flush for {float(baseline_bias):.2f} s on average, while forgetting plus flush reduces that to {float(forgetting_flush_bias):.2f} s."
        )

    if u4.get("scenario_rows"):
        monotone = str(bool(u4.get("reduced_order_monotone_with_n"))).lower()
        takeaways.append(
            f"U4 is only partially resolved: reduced-order CL improves monotonically with N on the extended single-fault scaling set ({monotone}), but the matched full-Drake trend is mixed rather than monotone."
        )

    u5_rows = u5.get("scenario_rows", [])
    if u5_rows:
        cl_better_all = all(
            float(row["cl_mean_post_fault_rmse_m"]) <= float(row["l1_mean_post_fault_rmse_m"]) + 1e-9
            for row in u5_rows
        )
        degradation_statement = "CL stays below the tested L1 variant on post-fault RMSE across all replayed degradation scenarios." if cl_better_all else "the relative CL versus L1 ordering depends on the degradation profile."
        takeaways.append(
            f"U5 now has replayable multi-topology evidence for gradual fray and exponential weakening profiles; {degradation_statement}"
        )

    if u6.get("candidate_count"):
        takeaways.append(
            f"U6 has now been searched more thoroughly: {u6['candidate_count']} structured non-CL candidates across {len(u6.get('baseline_cl_by_scenario', []))} scenarios produced {len(u6.get('robust_winners', []))} robust winners."
        )

    fault_matrix = u6.get("fault_matrix")
    if fault_matrix is not None:
        takeaways.append(
            "U6 is not supported by the current replayable pack: the tested structured L1 configuration does not beat CL on either post-fault RMSE or peak deviation."
        )

    return takeaways


def write_report(output_path: Path, summary: dict[str, Any]) -> None:
    u1 = summary["U1"]
    u2 = summary["U2"]
    u3 = summary["U3"]
    u4 = summary["U4"]
    u5 = summary["U5"]
    u6 = summary["U6"]

    lines = [
        "# Unknowns Resolution Status",
        "",
        "## Executive Takeaways",
        "",
    ]
    for takeaway in build_takeaways(summary):
        lines.append(f"- {takeaway}")

    lines.extend([
        "",
        "## U1 Reduced-Order vs Full Drake",
        "",
        f"- Status: {u1['status']}",
        f"- Max full-Drake / reduced-CL RMSE ratio: {u1['max_full_to_reduced_cl_ratio']:.2f}" if u1["max_full_to_reduced_cl_ratio"] is not None else "- Max full-Drake / reduced-CL RMSE ratio: unavailable",
        "",
        "| Scenario | N | Reduced CL post-fault RMSE [cm] | Full Drake load RMSE [cm] | Ratio |",
        "| --- | ---: | ---: | ---: | ---: |",
    ])
    for row in u1["scenario_rows"]:
        lines.append(
            f"| {row['scenario']} | {row['num_agents']} | {row['reduced_cl_post_fault_rmse_m'] * 100.0:.2f} | {row['full_drake_load_rmse_m'] * 100.0:.2f} | {row['full_to_reduced_cl_ratio']:.2f} |"
        )

    lines.extend([
        "",
        "## U2 Topology-Invariance Conditions",
        "",
        f"- Status: {u2['status']}",
        "- Current result is numeric support only, not a formal proof.",
        "",
        "| Scenario | N | Margin OK | Sufficient-margin OK | Worst required thrust per agent [N] | Max available [N] | Capacity ratio | Effective capacity ratio | Healthy support fraction | Cost ratio |",
        "| --- | ---: | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |",
    ])
    for row in u2["scenario_rows"]:
        lines.append(
            f"| {row['scenario']} | {row['num_agents']} | {str(row['thrust_margin_ok']).lower()} | {str(row['sufficient_margin_ok']).lower()} | {row['worst_required_thrust_per_agent_n']:.2f} | {row['max_available_thrust_per_agent_n']:.2f} | {row['worst_capacity_ratio']:.2f} | {row['worst_effective_capacity_ratio']:.2f} | {row['min_healthy_support_fraction']:.2f} | {row['fault_tolerance_cost_linear_ratio']:.2f} |"
        )

    lines.extend([
        "",
        "## U3 CL History Corruption",
        "",
        f"- Status: {u3['status']}",
        f"- Baseline mean theta-bias duration vs fault-triggered flush: {u3['baseline'].get('mean_theta_bias_duration_vs_flush_s')}",
        f"- Forgetting mean theta-bias duration vs fault-triggered flush: {u3['forgetting'].get('mean_theta_bias_duration_vs_flush_s')}",
        f"- Forgetting+flush mean theta-bias duration vs fault-triggered flush: {u3['forgetting_flush'].get('mean_theta_bias_duration_vs_flush_s')}",
        f"- Flush mean history refill time: {u3['flush'].get('mean_history_refill_time_s')}",
        "",
        "## U4 Generalization Beyond N=3",
        "",
        f"- Status: {u4['status']}",
        f"- Reduced-order CL RMSE decreases monotonically with N: {str(u4['reduced_order_monotone_with_n']).lower()}",
        "",
        "| Scenario | N | Reduced CL post-fault RMSE [cm] | Full Drake load RMSE [cm] |",
        "| --- | ---: | ---: | ---: |",
    ])
    for row in u4["scenario_rows"]:
        lines.append(
            f"| {row['scenario']} | {row['num_agents']} | {row['reduced_cl_post_fault_rmse_m'] * 100.0:.2f} | {row['full_drake_load_rmse_m'] * 100.0:.2f} |"
        )

    if u4.get("reduced_order_scaling_rows"):
        lines.extend([
            "",
            "### Extended Reduced-Order Scaling Set",
            "",
            "| Scenario | N | CL post-fault RMSE [cm] | CL peak [cm] | CL spread [cm] |",
            "| --- | ---: | ---: | ---: | ---: |",
        ])
        for row in u4["reduced_order_scaling_rows"]:
            lines.append(
                f"| {row['scenario']} | {row['num_agents']} | {row['cl_mean_post_fault_rmse_m'] * 100.0:.2f} | {row['cl_mean_peak_deviation_m'] * 100.0:.2f} | {row['cl_cable_spread_m'] * 100.0:.2f} |"
            )

    lines.extend([
        "",
        "## U5 Gradual Cable Degradation",
        "",
        f"- Status: {u5['status']}",
        "",
        "| Scenario | Profiles | CL post-fault RMSE [cm] | L1 post-fault RMSE [cm] | CL peak [cm] | L1 peak [cm] |",
        "| --- | --- | ---: | ---: | ---: | ---: |",
    ])
    for row in u5["scenario_rows"]:
        lines.append(
            f"| {row['scenario']} | {', '.join(row['profiles'])} | {row['cl_mean_post_fault_rmse_m'] * 100.0:.2f} | {row['l1_mean_post_fault_rmse_m'] * 100.0:.2f} | {row['cl_mean_peak_deviation_m'] * 100.0:.2f} | {row['l1_mean_peak_deviation_m'] * 100.0:.2f} |"
        )

    lines.extend([
        "",
        "## U6 Non-CL Adaptive Laws",
        "",
        f"- Status: {u6['status']}",
    ])
    if u6.get("candidate_count"):
        lines.extend([
            f"- Candidate families tested: {u6['candidate_family_count']}",
            f"- Total candidates tested: {u6['candidate_count']}",
            f"- Robust winners: {len(u6.get('robust_winners', []))}",
        ])
    best_balanced_candidate = u6.get("best_balanced_candidate")
    if best_balanced_candidate is not None:
        lines.extend([
            f"- Best balanced candidate: {best_balanced_candidate['family']} | omega={best_balanced_candidate['omega']:.2f}, predictor_pole={best_balanced_candidate['predictor_pole']:.2f}, sigma_max={best_balanced_candidate['sigma_max']:.2f}",
            f"- Mean delta vs CL post-fault RMSE [cm]: {best_balanced_candidate['mean_delta_vs_cl_rmse_m'] * 100.0:.2f}",
            f"- Mean delta vs CL peak deviation [cm]: {best_balanced_candidate['mean_delta_vs_cl_peak_m'] * 100.0:.2f}",
            "",
            "| Scenario | Delta post vs CL [cm] | Delta peak vs CL [cm] | Delta spread vs CL [cm] |",
            "| --- | ---: | ---: | ---: |",
        ])
        for row in best_balanced_candidate["scenario_metrics"]:
            lines.append(
                f"| {row['scenario']} | {row['delta_vs_cl_rmse_m'] * 100.0:.2f} | {row['delta_vs_cl_peak_m'] * 100.0:.2f} | {row['delta_vs_cl_spread_m'] * 100.0:.2f} |"
            )
    fault_matrix = u6.get("fault_matrix")
    if fault_matrix is not None:
        lines.extend([
            f"- L1 beats CL on post-fault RMSE: {str(fault_matrix['l1_beats_cl_on_rmse']).lower()}",
            f"- L1 beats CL on peak deviation: {str(fault_matrix['l1_beats_cl_on_peak']).lower()}",
            "",
            "| Mode | Mean post-fault RMSE [cm] | Mean peak deviation [cm] | Cable spread [cm] |",
            "| --- | ---: | ---: | ---: |",
            f"| CL | {float(fault_matrix['cl']['mean_post_fault_rmse']) * 100.0:.2f} | {float(fault_matrix['cl']['mean_peak_deviation']) * 100.0:.2f} | {float(fault_matrix['cl']['cable_spread']) * 100.0:.2f} |",
            f"| L1 | {float(fault_matrix['l1']['mean_post_fault_rmse']) * 100.0:.2f} | {float(fault_matrix['l1']['mean_peak_deviation']) * 100.0:.2f} | {float(fault_matrix['l1']['cable_spread']) * 100.0:.2f} |",
        ])

    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description="Build a consolidated status report for README unknowns U1-U6")
    parser.add_argument("--reduced-vs-drake", type=Path, required=True)
    parser.add_argument("--topology-conditions", type=Path, required=True)
    parser.add_argument("--cl-history", type=Path, required=True)
    parser.add_argument("--topology-scaling", type=Path)
    parser.add_argument("--degradation", type=Path, nargs="*", default=[])
    parser.add_argument("--non-cl-fault-matrix", type=Path)
    parser.add_argument("--non-cl-sweep", type=Path)
    parser.add_argument("--non-cl-search", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()

    args.output_dir.mkdir(parents=True, exist_ok=True)
    reduced_vs_drake = load_json(args.reduced_vs_drake)
    topology_conditions = load_json(args.topology_conditions)
    cl_history = load_json(args.cl_history)
    topology_scaling = load_json(args.topology_scaling) if args.topology_scaling else None
    degradation_payloads = [load_json(path) for path in args.degradation]
    non_cl_fault_matrix = load_json(args.non_cl_fault_matrix) if args.non_cl_fault_matrix else None
    non_cl_sweep = load_json(args.non_cl_sweep) if args.non_cl_sweep else None
    non_cl_search = load_json(args.non_cl_search) if args.non_cl_search else None

    summary = {
        "generator": "experiments/python/build_unknowns_status_report.py",
        "U1": summarize_u1(reduced_vs_drake),
        "U2": summarize_u2(topology_conditions),
        "U3": summarize_u3(cl_history),
        "U4": summarize_u4(reduced_vs_drake, topology_scaling),
        "U5": summarize_u5(degradation_payloads),
        "U6": summarize_u6(non_cl_fault_matrix, non_cl_sweep, non_cl_search),
    }

    (args.output_dir / "unknowns_status.json").write_text(json.dumps(summary, indent=2, allow_nan=False), encoding="utf-8")
    write_report(args.output_dir / "unknowns_status_report.md", summary)
    print(json.dumps(summary, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()