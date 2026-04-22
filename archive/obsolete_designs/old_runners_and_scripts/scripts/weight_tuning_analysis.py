#!/usr/bin/env python3
"""
Weight Tuning Analysis for Decentralized Optimal Controller

This script systematically explores the weight space (w1, w2, w3, w4) to:
1. Measure objective function values under different weights
2. Compute Pareto frontier
3. Generate sensitivity analysis plots
4. Recommend optimal weight settings

Usage:
    python3 weight_tuning_analysis.py --output results/

Expected output:
    - weight_sweep_results.csv: Full sweep results
    - pareto_frontier.csv: Pareto-optimal configurations
    - sensitivity_plots.pdf: Matplotlib figures
    - recommended_weights.txt: Best weight settings
"""

import argparse
import csv
import json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.gridspec import GridSpec
from pathlib import Path
from typing import Dict, List, Tuple
import subprocess
import pandas as pd


class WeightTuningAnalyzer:
    """Analyze weight trade-offs for multi-objective MPC."""

    def __init__(self, output_dir: str):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

        # Store results
        self.results = []
        self.pareto_frontier = []

    def generate_weight_combinations(
        self,
        w1_range: Tuple[float, float, int],
        w2_range: Tuple[float, float, int],
        w3_range: Tuple[float, float, int],
        w4_range: Tuple[float, float, int],
    ) -> List[Dict[str, float]]:
        """
        Generate weight combinations for exploration.

        Args:
            w1_range: (min, max, num_points) for trajectory weight
            w2_range: (min, max, num_points) for stability weight
            w3_range: (min, max, num_points) for effort weight
            w4_range: (min, max, num_points) for tension balance weight

        Returns:
            List of weight dictionaries
        """
        w1_vals = np.linspace(*w1_range)
        w2_vals = np.linspace(*w2_range)
        w3_vals = np.linspace(*w3_range)
        w4_vals = np.linspace(*w4_range)

        combinations = []

        # Sweep: keep priority ratios, vary overall magnitude
        for w1 in w1_vals:
            for w2 in w2_vals:
                for w3 in w3_vals:
                    for w4 in w4_vals:
                        combinations.append({
                            "w_trajectory": float(w1),
                            "w_stability": float(w2),
                            "w_effort": float(w3),
                            "w_tension_balance": float(w4),
                        })

        print(f"Generated {len(combinations)} weight combinations")
        return combinations

    def simulate_with_weights(
        self,
        weights: Dict[str, float],
        simulation_duration: float = 10.0,
        test_executable: str = "./build/decentralized_mpc_test",
    ) -> Dict[str, float]:
        """
        Run simulation with given weights and extract metrics.

        For Phase 1, we'll estimate metrics from weight magnitudes
        (actual simulation data would replace this).

        Args:
            weights: Dictionary with w_trajectory, w_stability, w_effort, w_tension_balance
            simulation_duration: Duration of simulation in seconds
            test_executable: Path to compiled test executable

        Returns:
            Dictionary with J_traj, J_stab, J_eff, J_tension metrics
        """

        # Phase 1: Analytical estimation based on weight values
        # (Replace with actual simulation results once test harness is running)

        w1 = weights["w_trajectory"]
        w2 = weights["w_stability"]
        w3 = weights["w_effort"]
        w4 = weights["w_tension_balance"]

        # Estimated trade-off: higher weights on one term → lower cost on that term
        # This is a placeholder model; replace with actual simulation data

        # Normalize by typical magnitudes
        J_traj_typical = 0.02  # m² error
        J_stab_typical = 0.5   # (m/s²)²
        J_eff_typical = 5.0    # normalized
        J_tension_typical = 2.0  # N²

        # Inverse relationship: higher weight → lower cost on that objective
        J_traj = J_traj_typical / (1.0 + w1 / 10.0)
        J_stab = J_stab_typical / (1.0 + w2 / 1.0)
        J_eff = J_eff_typical / (1.0 + w3 / 0.1)
        J_tension = J_tension_typical / (1.0 + w4 / 0.01)

        # Add small random noise (simulation variability)
        noise_scale = 0.01
        J_traj += np.random.normal(0, noise_scale * J_traj)
        J_stab += np.random.normal(0, noise_scale * J_stab)
        J_eff += np.random.normal(0, noise_scale * J_eff)
        J_tension += np.random.normal(0, noise_scale * J_tension)

        return {
            "J_trajectory": float(J_traj),
            "J_stability": float(J_stab),
            "J_effort": float(J_eff),
            "J_tension_balance": float(J_tension),
            "J_total": float(w1 * J_traj + w2 * J_stab + w3 * J_eff +
                           w4 * J_tension),
        }

    def compute_pareto_frontier(self):
        """
        Identify Pareto-optimal configurations.

        A configuration is Pareto-optimal if no other configuration
        dominates it in all objectives.
        """

        objectives = [
            "J_trajectory",
            "J_stability",
            "J_effort",
            "J_tension_balance",
        ]

        pareto = []
        for config in self.results:
            is_dominated = False

            for other_config in self.results:
                # Check if other_config dominates config
                dominates_all = True
                for obj in objectives:
                    if other_config[obj] >= config[obj]:
                        dominates_all = False
                        break

                if dominates_all:
                    is_dominated = True
                    break

            if not is_dominated:
                pareto.append(config)

        self.pareto_frontier = pareto
        print(f"Pareto frontier contains {len(pareto)} configurations")

    def run_sweep(
        self,
        w1_range: Tuple[float, float, int],
        w2_range: Tuple[float, float, int],
        w3_range: Tuple[float, float, int],
        w4_range: Tuple[float, float, int],
    ):
        """Run full weight sweep."""

        combinations = self.generate_weight_combinations(
            w1_range, w2_range, w3_range, w4_range
        )

        print("\nRunning weight sweep...")
        for i, weights in enumerate(combinations):
            metrics = self.simulate_with_weights(weights)
            result = {**weights, **metrics}
            self.results.append(result)

            if (i + 1) % 10 == 0:
                print(f"  Completed {i + 1}/{len(combinations)} simulations")

        # Save full results to CSV
        csv_file = self.output_dir / "weight_sweep_results.csv"
        if self.results:
            with open(csv_file, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=self.results[0].keys())
                writer.writeheader()
                writer.writerows(self.results)
            print(f"Saved results to {csv_file}")

    def save_pareto_frontier(self):
        """Save Pareto frontier to CSV."""
        pareto_file = self.output_dir / "pareto_frontier.csv"
        if self.pareto_frontier:
            with open(pareto_file, "w", newline="") as f:
                writer = csv.DictWriter(
                    f, fieldnames=self.pareto_frontier[0].keys()
                )
                writer.writeheader()
                writer.writerows(self.pareto_frontier)
            print(f"Saved Pareto frontier to {pareto_file}")

    def find_recommended_weights(self) -> Dict[str, float]:
        """
        Find recommended weights using lexicographic ordering heuristic.

        Prioritize:
        1. Lowest J_trajectory (highest priority)
        2. Among those, lowest J_stability
        3. Among those, lowest J_effort
        4. Among those, lowest J_tension_balance
        """

        if not self.pareto_frontier:
            if not self.results:
                return {}
            self.compute_pareto_frontier()

        candidates = sorted(
            self.pareto_frontier,
            key=lambda x: (x["J_trajectory"], x["J_stability"], x["J_effort"],
                          x["J_tension_balance"]),
        )

        if candidates:
            best = candidates[0]
            return {
                "w_trajectory": best["w_trajectory"],
                "w_stability": best["w_stability"],
                "w_effort": best["w_effort"],
                "w_tension_balance": best["w_tension_balance"],
            }

        return {}

    def plot_sensitivity_analysis(self):
        """Generate sensitivity plots."""

        if not self.results:
            print("No results to plot")
            return

        df = pd.DataFrame(self.results)

        # Create multi-panel figure
        fig = plt.figure(figsize=(14, 10))
        gs = GridSpec(3, 2, figure=fig, hspace=0.35, wspace=0.3)

        objectives = [
            ("J_trajectory", "Trajectory Error"),
            ("J_stability", "Load Stability"),
            ("J_effort", "Control Effort"),
            ("J_tension_balance", "Tension Balance"),
        ]

        # Panel 1: Scatter: w_trajectory vs J_trajectory
        ax1 = fig.add_subplot(gs[0, 0])
        ax1.scatter(df["w_trajectory"], df["J_trajectory"], alpha=0.5, s=20)
        ax1.set_xlabel("w_trajectory")
        ax1.set_ylabel("J_trajectory (m²)")
        ax1.set_title("Trajectory Weight vs Error")
        ax1.grid(True, alpha=0.3)

        # Panel 2: Scatter: w_stability vs J_stability
        ax2 = fig.add_subplot(gs[0, 1])
        ax2.scatter(df["w_stability"], df["J_stability"], alpha=0.5, s=20,
                   color="orange")
        ax2.set_xlabel("w_stability")
        ax2.set_ylabel("J_stability (m²/s⁴)")
        ax2.set_title("Stability Weight vs Cost")
        ax2.grid(True, alpha=0.3)

        # Panel 3: Scatter: w_effort vs J_effort
        ax3 = fig.add_subplot(gs[1, 0])
        ax3.scatter(df["w_effort"], df["J_effort"], alpha=0.5, s=20,
                   color="green")
        ax3.set_xlabel("w_effort")
        ax3.set_ylabel("J_effort (normalized)")
        ax3.set_title("Effort Weight vs Cost")
        ax3.grid(True, alpha=0.3)

        # Panel 4: Scatter: w_tension vs J_tension
        ax4 = fig.add_subplot(gs[1, 1])
        ax4.scatter(df["w_tension_balance"], df["J_tension_balance"],
                   alpha=0.5, s=20, color="red")
        ax4.set_xlabel("w_tension_balance")
        ax4.set_ylabel("J_tension (N²)")
        ax4.set_title("Tension Balance Weight vs Cost")
        ax4.grid(True, alpha=0.3)

        # Panel 5: Pareto frontier (2D projection: J_trajectory vs J_effort)
        ax5 = fig.add_subplot(gs[2, :])
        if self.pareto_frontier:
            pareto_df = pd.DataFrame(self.pareto_frontier)
            ax5.scatter(pareto_df["J_trajectory"], pareto_df["J_effort"],
                       s=100, marker="*", color="darkblue", label="Pareto",
                       edgecolors="black", linewidth=2)
        ax5.scatter(df["J_trajectory"], df["J_effort"], alpha=0.2, s=10,
                   label="All configs")
        ax5.set_xlabel("J_trajectory (m²) - Higher priority")
        ax5.set_ylabel("J_effort (normalized)")
        ax5.set_title("Pareto Frontier (Trajectory vs Effort)")
        ax5.legend()
        ax5.grid(True, alpha=0.3)

        plt.suptitle(
            "Weight Tuning Sensitivity Analysis\n"
            "Decentralized Optimal Controller",
            fontsize=14,
            fontweight="bold",
        )

        plot_file = self.output_dir / "sensitivity_plots.pdf"
        plt.savefig(plot_file, dpi=150, bbox_inches="tight")
        print(f"Saved plots to {plot_file}")
        plt.close()

    def generate_recommendation_report(self):
        """Generate final recommendation report."""

        recommended = self.find_recommended_weights()

        if not recommended:
            print("No weights to recommend (no results)")
            return

        report_file = self.output_dir / "recommended_weights.txt"
        with open(report_file, "w") as f:
            f.write("=" * 60 + "\n")
            f.write("DECENTRALIZED OPTIMAL CONTROLLER - WEIGHT TUNING RESULTS\n")
            f.write("=" * 60 + "\n\n")

            f.write("RECOMMENDED WEIGHTS (Lexicographic Ordering)\n")
            f.write("-" * 60 + "\n")
            f.write(
                f"w_trajectory (priority 1):     {recommended['w_trajectory']:.4f}\n"
            )
            f.write(
                f"w_stability (priority 2):      {recommended['w_stability']:.4f}\n"
            )
            f.write(f"w_effort (priority 3):         {recommended['w_effort']:.4f}\n")
            f.write(
                f"w_tension_balance (priority 4): {recommended['w_tension_balance']:.4f}\n"
            )

            f.write("\nPriority Ratios\n")
            f.write("-" * 60 + "\n")
            f.write(
                f"w_stability / w_trajectory:     {recommended['w_stability'] / recommended['w_trajectory']:.4f}\n"
            )
            f.write(
                f"w_effort / w_stability:        {recommended['w_effort'] / recommended['w_stability']:.4f}\n"
            )
            f.write(
                f"w_tension / w_effort:          {recommended['w_tension_balance'] / recommended['w_effort']:.4f}\n"
            )

            f.write("\nGUIDENCE FOR USE IN Code\n")
            f.write("-" * 60 + "\n")
            f.write("Add to DecentralizedOptimalController::Config:\n\n")
            f.write(
                f"config.w_trajectory = {recommended['w_trajectory']:.1f};\n"
            )
            f.write(f"config.w_stability = {recommended['w_stability']:.1f};\n")
            f.write(f"config.w_effort = {recommended['w_effort']:.2f};\n")
            f.write(
                f"config.w_tension_balance = {recommended['w_tension_balance']:.3f};\n"
            )

            f.write("\nAlternative Weights for Different Scenarios\n")
            f.write("-" * 60 + "\n")

            # Conservative (prioritize smoothness)
            f.write("\nCONSERVATIVE (Smooth, stable recovery from faults):\n")
            conservative_ratio = 20.0
            w1_cons = 100.0
            w2_cons = w1_cons / conservative_ratio
            w3_cons = w2_cons / conservative_ratio
            w4_cons = w3_cons / conservative_ratio
            f.write(f"  w_trajectory = {w1_cons:.1f}\n")
            f.write(f"  w_stability = {w2_cons:.1f}\n")
            f.write(f"  w_effort = {w3_cons:.2f}\n")
            f.write(f"  w_tension_balance = {w4_cons:.3f}\n")

            # Aggressive (prioritize trajectory)
            f.write("\nAGGRESSIVE (Fast trajectory tracking, higher effort):\n")
            aggressive_ratio = 5.0
            w1_agg = 100.0
            w2_agg = w1_agg / aggressive_ratio
            w3_agg = w2_agg / aggressive_ratio
            w4_agg = w3_agg / aggressive_ratio
            f.write(f"  w_trajectory = {w1_agg:.1f}\n")
            f.write(f"  w_stability = {w2_agg:.1f}\n")
            f.write(f"  w_effort = {w3_agg:.2f}\n")
            f.write(f"  w_tension_balance = {w4_agg:.3f}\n")

        print(f"Saved recommendation to {report_file}")


def main():
    parser = argparse.ArgumentParser(
        description="Weight tuning analysis for Decentralized Optimal Controller"
    )
    parser.add_argument(
        "--output",
        default="weight_tuning_results",
        help="Output directory for results",
    )
    parser.add_argument(
        "--quick",
        action="store_true",
        help="Quick sweep (fewer combinations)",
    )

    args = parser.parse_args()

    analyzer = WeightTuningAnalyzer(args.output)

    # Define weight ranges to explore
    if args.quick:
        # Quick sweep: 2 points per dimension
        w1_range = (10, 100, 2)
        w2_range = (1, 10, 2)
        w3_range = (0.1, 1, 2)
        w4_range = (0.01, 0.1, 2)
    else:
        # Full sweep: 5 points per dimension
        w1_range = (10, 100, 5)
        w2_range = (1, 10, 5)
        w3_range = (0.1, 1, 5)
        w4_range = (0.01, 0.1, 5)

    # Run weight sweep
    analyzer.run_sweep(w1_range, w2_range, w3_range, w4_range)

    # Compute Pareto frontier
    analyzer.compute_pareto_frontier()

    # Save results
    analyzer.save_pareto_frontier()

    # Generate plots
    analyzer.plot_sensitivity_analysis()

    # Generate recommendation
    analyzer.generate_recommendation_report()

    print(f"\n✓ Analysis complete. Results saved to {args.output}/")


if __name__ == "__main__":
    main()
