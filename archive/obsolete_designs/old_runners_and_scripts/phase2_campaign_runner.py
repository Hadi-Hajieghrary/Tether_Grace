#!/usr/bin/env python3
"""
Phase 2 Research Campaign Runner
Executes 10 scenarios with automated data collection and analysis
"""

import subprocess
import json
import os
import sys
import time
from pathlib import Path
from datetime import datetime
import numpy as np

class Phase2Campaign:
    """Orchestrates Phase 2 research campaign execution"""

    SCENARIOS = {
        "S1": {
            "name": "Single Drone (Phase 1 Regression)",
            "description": "One drone with dual-MPC, validates Phase 1 baseline",
            "duration": 10.0,
            "sever_0": -1.0,
            "sever_1": -1.0,
            "notes": "Establishes Phase 1 regression baseline"
        },
        "S2": {
            "name": "Two Drones Nominal",
            "description": "Both drones active, shared reference, no faults",
            "duration": 10.0,
            "sever_0": -1.0,
            "sever_1": -1.0,
            "notes": "Validates implicit load sharing under nominal conditions"
        },
        "S3": {
            "name": "Implicit Coordination Validation",
            "description": "Two drones nominal - analyzes thrust symmetry",
            "duration": 10.0,
            "sever_0": -1.0,
            "sever_1": -1.0,
            "notes": "Verifies symmetric thrust distribution despite decentralized control"
        },
        "S4": {
            "name": "Drone 0 Cable Severance (t=3s)",
            "description": "Drone 0 cable fails at 30% through trajectory",
            "duration": 10.0,
            "sever_0": 3.0,
            "sever_1": -1.0,
            "notes": "Early fault detection: rope tension → 0, MPC rebalances"
        },
        "S5": {
            "name": "Drone 0 Cable Severance (t=5s)",
            "description": "Drone 0 cable fails at 50% through trajectory",
            "duration": 10.0,
            "sever_0": 5.0,
            "sever_1": -1.0,
            "notes": "Mid-flight fault: tests dynamic load rebalancing"
        },
        "S6": {
            "name": "Drone 0 Cable Severance (t=7s)",
            "description": "Drone 0 cable fails at 70% through trajectory",
            "duration": 10.0,
            "sever_0": 7.0,
            "sever_1": -1.0,
            "notes": "Late fault: validates sustained single-drone recovery"
        },
        "S7": {
            "name": "Both Cables Severed (Sequential)",
            "description": "Drone 0 fails at t=3s, Drone 1 at t=6s",
            "duration": 10.0,
            "sever_0": 3.0,
            "sever_1": 6.0,
            "notes": "Progressive failure: validates two-stage fault response"
        },
        "S8": {
            "name": "Wind Disturbance (Future)",
            "description": "Horizontal wind gust injection at t=5s",
            "duration": 10.0,
            "sever_0": -1.0,
            "sever_1": -1.0,
            "notes": "Requires wind disturbance system integration (Phase 2.5+)"
        },
        "S9": {
            "name": "Measurement Noise (Future)",
            "description": "Gaussian noise on cable tension measurements",
            "duration": 10.0,
            "sever_0": -1.0,
            "sever_1": -1.0,
            "notes": "Requires noise injection system (Phase 2.5+)"
        },
        "S10": {
            "name": "Concurrent Failures (Future)",
            "description": "Both cables fail simultaneously at t=5s",
            "duration": 10.0,
            "sever_0": 5.0,
            "sever_1": 5.0,
            "notes": "Complete system failure: validates safe shutdown behavior"
        }
    }

    def __init__(self, executable_path: str, output_dir: str = "campaign_results"):
        """Initialize campaign runner

        Args:
            executable_path: Path to decentralized_mpc_phase25_test
            output_dir: Directory for results
        """
        self.executable = Path(executable_path)
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)
        self.results = {}

        if not self.executable.exists():
            raise FileNotFoundError(f"Executable not found: {executable_path}")

    def run_scenario(self, scenario_key: str, timeout: float = 180.0) -> dict:
        """Run a single scenario

        Args:
            scenario_key: S1, S2, ..., S10
            timeout: Maximum execution time in seconds

        Returns:
            dict with execution results
        """
        if scenario_key not in self.SCENARIOS:
            raise ValueError(f"Unknown scenario: {scenario_key}")

        scenario = self.SCENARIOS[scenario_key]
        print(f"\n{'='*70}")
        print(f"Running {scenario_key}: {scenario['name']}")
        print(f"{'='*70}")
        print(f"Duration: {scenario['duration']}s")
        print(f"Drone 0 severance: {scenario['sever_0']}s")
        print(f"Drone 1 severance: {scenario['sever_1']}s")

        # Build command
        cmd = [str(self.executable), "--duration", str(scenario['duration'])]
        if scenario['sever_0'] >= 0:
            cmd.extend(["--sever-0", str(scenario['sever_0'])])
        if scenario['sever_1'] >= 0:
            cmd.extend(["--sever-1", str(scenario['sever_1'])])

        # Run scenario
        start_time = time.time()
        try:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=timeout,
                cwd=self.executable.parent
            )
            elapsed = time.time() - start_time

            success = result.returncode == 0
            output = result.stdout + result.stderr

            return {
                "scenario": scenario_key,
                "success": success,
                "elapsed_time": elapsed,
                "output": output,
                "returncode": result.returncode
            }
        except subprocess.TimeoutExpired:
            elapsed = time.time() - start_time
            print(f"  ⚠ TIMEOUT after {elapsed:.1f}s")
            return {
                "scenario": scenario_key,
                "success": False,
                "elapsed_time": elapsed,
                "output": "",
                "returncode": -1,
                "timeout": True
            }
        except Exception as e:
            print(f"  ✗ ERROR: {e}")
            return {
                "scenario": scenario_key,
                "success": False,
                "elapsed_time": 0,
                "output": str(e),
                "returncode": -1
            }

    def run_campaign(self, scenarios=None, timeout=180.0):
        """Run full campaign

        Args:
            scenarios: List of scenario keys to run (default: all)
            timeout: Timeout per scenario
        """
        if scenarios is None:
            scenarios = [f"S{i}" for i in range(1, 11)]

        campaign_start = datetime.now()
        print(f"\n{'='*70}")
        print(f"PHASE 2 RESEARCH CAMPAIGN")
        print(f"Started: {campaign_start.isoformat()}")
        print(f"Scenarios: {len(scenarios)}")
        print(f"{'='*70}")

        for scenario_key in scenarios:
            result = self.run_scenario(scenario_key, timeout)
            self.results[scenario_key] = result

            if result['success']:
                print(f"  ✓ PASS ({result['elapsed_time']:.1f}s)")
            else:
                print(f"  ✗ FAIL")
                if 'timeout' in result:
                    print(f"     Timeout after {result['elapsed_time']:.1f}s")
                else:
                    print(f"     Return code: {result['returncode']}")

        campaign_end = datetime.now()
        duration = (campaign_end - campaign_start).total_seconds()

        # Save results
        self.save_results(campaign_start, campaign_end)

        # Print summary
        self.print_summary()

    def save_results(self, start_time, end_time):
        """Save campaign results to JSON"""
        summary = {
            "campaign_start": start_time.isoformat(),
            "campaign_end": end_time.isoformat(),
            "total_scenarios": len(self.results),
            "passed": sum(1 for r in self.results.values() if r['success']),
            "failed": sum(1 for r in self.results.values() if not r['success']),
            "scenarios": {}
        }

        for scenario_key, result in self.results.items():
            summary["scenarios"][scenario_key] = {
                "name": self.SCENARIOS[scenario_key]['name'],
                "success": result['success'],
                "elapsed_time_seconds": result['elapsed_time'],
                "returncode": result['returncode']
            }

        results_file = self.output_dir / "campaign_results.json"
        with open(results_file, 'w') as f:
            json.dump(summary, f, indent=2)

        print(f"\nResults saved to: {results_file}")

    def print_summary(self):
        """Print campaign summary"""
        total = len(self.results)
        passed = sum(1 for r in self.results.values() if r['success'])
        failed = total - passed

        print(f"\n{'='*70}")
        print("CAMPAIGN SUMMARY")
        print(f"{'='*70}")
        print(f"Total:  {total}")
        print(f"Passed: {passed} ({100*passed/total:.0f}%)")
        print(f"Failed: {failed} ({100*failed/total:.0f}%)")
        print(f"\nScenario Results:")
        print(f"{'-'*70}")

        for scenario_key in sorted(self.results.keys()):
            result = self.results[scenario_key]
            status = "✓ PASS" if result['success'] else "✗ FAIL"
            scenario_name = self.SCENARIOS[scenario_key]['name']
            time_str = f"{result['elapsed_time']:.1f}s"
            print(f"  {scenario_key:3} {scenario_name:40} {status:8} {time_str:>8}")

        print(f"{'='*70}\n")


if __name__ == "__main__":
    # Default paths
    executable = "/workspaces/Tether_Grace/Research/cpp/build/decentralized_mpc_phase25_test"
    output_dir = "/workspaces/Tether_Grace/Research/campaign_results"

    # Parse arguments
    if len(sys.argv) > 1:
        executable = sys.argv[1]
    if len(sys.argv) > 2:
        output_dir = sys.argv[2]

    # Run campaign
    try:
        campaign = Phase2Campaign(executable, output_dir)

        # Run only S1-S7 for now (S8-S10 require additional features)
        campaign.run_campaign(scenarios=[f"S{i}" for i in range(1, 8)], timeout=120.0)

    except Exception as e:
        print(f"Campaign failed: {e}", file=sys.stderr)
        sys.exit(1)
