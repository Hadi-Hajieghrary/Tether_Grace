# 4-Quadcopter Decentralized Load Transport Scenario

## Overview

A complete simulation of 4 quadcopters cooperatively transporting a 6 kg payload using
decentralized MPC control with implicit load sharing. Three fault scenarios exercise the
system's graceful degradation under cable failures.

## Configuration

| Parameter | Value |
|-----------|-------|
| Drones | 4 (arranged in a square) |
| Quad mass | 1.5 kg each |
| Payload mass | 6.0 kg |
| Rope length | 1.0 m per rope |
| Rope stiffness | 200 N/m |
| Rope damping | 15 N·s/m |
| Bead count | 8 per rope (32 total) |
| Control rate | 50 Hz decentralized MPC |
| Thrust limit | 4× hover per quad = 58.86 N |
| Reference point | p_ref = [0, 0, 2.0] m |

## Three Scenarios

### Scenario A — Nominal Multi-Drone Coordination
- All 4 drones active
- No cable faults
- Purpose: Validate symmetric thrust distribution (each drone carries ~1.5 kg of the 6 kg payload)
- Expected per-drone thrust at steady state: ~29.4 N

### Scenario B — Single-Drone Failure (Graceful Degradation)
- Drone 0's cable severs at t = duration/2
- Remaining 3 drones rebalance: each carries ~2.0 kg
- Expected post-failure thrust: ~19.6 N per remaining drone

### Scenario C — Dual-Drone Failure (System Limits)
- Drone 0 fails at t = 30% duration
- Drone 1 fails at t = 60% duration
- Remaining 2 drones must sustain: each carries ~3.0 kg
- Tests thrust-limit saturation behavior

## Files

| File | Purpose |
|------|---------|
| [Research/cpp/src/decentralized_mpc_four_drones_test_main.cc](Research/cpp/src/decentralized_mpc_four_drones_test_main.cc) | 4-drone test harness |
| [Research/cpp/include/quad_plant_state_splitter.h](Research/cpp/include/quad_plant_state_splitter.h) | Splits plant state → 4 drones × 4 measurements |
| [Research/cpp/src/quad_plant_state_splitter.cc](Research/cpp/src/quad_plant_state_splitter.cc) | State splitter implementation |
| [Research/cpp/src/force_combiner.cc](Research/cpp/src/force_combiner.cc) | Combines spatial forces from 4 rope systems |
| [Research/cpp/src/tension_communicator.cc](Research/cpp/src/tension_communicator.cc) | Broadcasts tension measurements to all drones |
| [Research/run_four_drones_scenarios.sh](Research/run_four_drones_scenarios.sh) | Runs all 3 scenarios |
| [Research/four_drones_meshcat_replay.py](Research/four_drones_meshcat_replay.py) | Meshcat visualization replay |

## How to Run

### 1. Build the executable
```bash
cd /workspaces/Tether_Grace/Research/cpp/build
cmake ..
make decentralized_mpc_four_drones_test -j4
```

### 2. Run all 3 scenarios
```bash
cd /workspaces/Tether_Grace/Research
./run_four_drones_scenarios.sh 3.0   # 3 seconds per scenario
```

This writes three CSV files to `four_drones_replays/`:
- `scenario_A_nominal.csv`
- `scenario_B_single_fault.csv`
- `scenario_C_dual_fault.csv`

Each CSV contains per-sample data for quad positions, payload position, all bead positions,
and thrust commands.

### 3. View the simulation in Meshcat

Each scenario produces a **standalone HTML file** with the full Meshcat animation embedded.
No server is needed — just open the file in any browser.

```bash
# Open one of the HTML replays in a browser:
#   four_drones_replays/scenario_A_nominal.html
#   four_drones_replays/scenario_B_single_fault.html
#   four_drones_replays/scenario_C_dual_fault.html
```

**To view:**
1. Open the `.html` file in Chrome/Firefox/Safari.
2. The scene loads with 4 colored quads (red/green/blue/yellow), orange payload, and rope beads.
3. In the Meshcat UI (top right): open the **Animations** panel → press ▶ Play.

The recording is client-side — you can scrub the timeline, pause, rotate the camera.

### 4. Standalone / Single Run

```bash
./cpp/build/decentralized_mpc_four_drones_test \
    --duration 5.0 \
    --sever-0 2.0 \          # Drone 0 cable severs at t=2s (optional)
    --sever-1 -1 \           # -1 = no sever for drone 1
    --scenario my_scenario \
    --output my_replay.csv \
    --html my_replay.html    # Standalone Meshcat HTML for viewing
```

## Known Limitations / Future Work

- **MPC tuning**: The solver currently oscillates between full thrust and zero — the cost
  weights (`w_trajectory`, `w_effort`) need retuning for stable reference tracking.
- **Load estimator convergence**: Initial load position is far from the converged estimate.
  Warm-starting the estimator would reduce the initial transient.
- **Initial rope state**: Bodies are placed at initial positions but velocities are zero,
  so the rope dynamics must settle during the first few timesteps.
- **MPC reference tracking**: Payload currently falls rather than climbing to `z=+2m`.
  Needs cost-weight retuning for stable reference tracking.
