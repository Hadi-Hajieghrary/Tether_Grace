#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from experiment_manifest import DEFAULT_FULL_DRAKE_BATCH_MANIFEST, load_scenarios


def main() -> None:
    parser = argparse.ArgumentParser(description="Validate topology-invariance scenario assumptions against root-owned sufficient-condition checks")
    parser.add_argument("--manifest", type=Path, default=DEFAULT_FULL_DRAKE_BATCH_MANIFEST)
    parser.add_argument("--quad-mass", type=float, default=1.5)
    parser.add_argument("--load-mass", type=float, default=3.0)
    parser.add_argument("--gravity", type=float, default=9.81)
    parser.add_argument("--max-thrust-per-agent", type=float, default=3.5 * 1.5 * 9.81)
    parser.add_argument("--wind-force-max", type=float, default=0.0)
    parser.add_argument("--sigma-bar-nom", type=float, default=8.0)
    parser.add_argument("--sigma-bar-ft", type=float, default=12.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    scenario_checks = []
    for scenario in load_scenarios(args.manifest):
        healthy_count = scenario.num_agents
        worst_required = 0.0
        worst_capacity_ratio = float("inf")
        worst_effective_capacity_ratio = float("inf")
        min_support_fraction = 1.0
        thrust_margin_ok = True
        sufficient_margin_ok = True
        event_checks = []
        cable_lengths = np.array(scenario.cable_lengths, dtype=float)
        healthy_mask = np.ones_like(cable_lengths, dtype=bool)
        for event in scenario.fault_events:
            if event.post_fault_scale <= 1e-6:
                healthy_count = max(healthy_count - 1, 1)
                if 0 <= event.cable_index < healthy_mask.size:
                    healthy_mask[event.cable_index] = False
            required_thrust = (args.quad_mass + args.load_mass / healthy_count) * args.gravity + args.wind_force_max
            worst_required = max(worst_required, required_thrust)
            event_ok = required_thrust <= args.max_thrust_per_agent
            thrust_margin_ok = thrust_margin_ok and event_ok
            capacity_ratio = args.max_thrust_per_agent / max(required_thrust, 1e-9)
            effective_capacity_ratio = capacity_ratio / max(args.sigma_bar_ft / max(args.sigma_bar_nom, 1e-9), 1e-9)
            support_fraction = healthy_count / max(scenario.num_agents, 1)
            healthy_lengths = cable_lengths[healthy_mask]
            length_cv = float(np.std(healthy_lengths) / max(np.mean(healthy_lengths), 1e-9)) if healthy_lengths.size else 0.0
            worst_capacity_ratio = min(worst_capacity_ratio, capacity_ratio)
            worst_effective_capacity_ratio = min(worst_effective_capacity_ratio, effective_capacity_ratio)
            min_support_fraction = min(min_support_fraction, support_fraction)
            event_sufficient = event_ok and effective_capacity_ratio >= 1.0 and healthy_count >= 2
            sufficient_margin_ok = sufficient_margin_ok and event_sufficient
            event_checks.append(
                {
                    "cable_index": event.cable_index,
                    "time_seconds": event.time_seconds,
                    "profile": event.profile,
                    "post_fault_scale": event.post_fault_scale,
                    "healthy_count_after_event": healthy_count,
                    "required_thrust_per_agent_n": required_thrust,
                    "thrust_margin_ok": event_ok,
                    "capacity_ratio": capacity_ratio,
                    "effective_capacity_ratio": effective_capacity_ratio,
                    "healthy_support_fraction": support_fraction,
                    "healthy_length_cv": length_cv,
                    "sufficient_margin_ok": event_sufficient,
                }
            )

        scenario_checks.append(
            {
                "scenario": scenario.name,
                "num_agents": scenario.num_agents,
                "thrust_margin_ok": thrust_margin_ok,
                "sufficient_margin_ok": sufficient_margin_ok,
                "worst_required_thrust_per_agent_n": worst_required,
                "max_available_thrust_per_agent_n": args.max_thrust_per_agent,
                "fault_tolerance_cost_linear_ratio": args.sigma_bar_ft / args.sigma_bar_nom,
                "worst_capacity_ratio": worst_capacity_ratio,
                "worst_effective_capacity_ratio": worst_effective_capacity_ratio,
                "min_healthy_support_fraction": min_support_fraction,
                "event_checks": event_checks,
            }
        )

    payload = {
        "generator": "experiments/python/validate_topology_invariance_conditions.py",
        "manifest": str(args.manifest),
        "assumptions": {
            "quad_mass": args.quad_mass,
            "load_mass": args.load_mass,
            "gravity": args.gravity,
            "max_thrust_per_agent": args.max_thrust_per_agent,
            "wind_force_max": args.wind_force_max,
            "sigma_bar_nom": args.sigma_bar_nom,
            "sigma_bar_ft": args.sigma_bar_ft,
        },
        "scenario_checks": scenario_checks,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(json.dumps(payload, indent=2))


if __name__ == "__main__":
    main()