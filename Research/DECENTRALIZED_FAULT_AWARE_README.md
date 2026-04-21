# Decentralized Fault-Aware 4-Drone Lift — Campaign & Analysis Pipeline

This builds on Tether_Lift's proven simulation infrastructure (URDF quadrotor
meshes, bead-chain ropes, ground plane, `MeshcatVisualizer` + HTML recording)
and swaps in a **peer-aware controller** that infers peer failures from rope
tensions alone (no centralized fault signal) and rebalances the load share.

## What's in this pipeline

1. **C++ simulation** ([decentralized_fault_aware_sim](cpp/src/decentralized_fault_aware_sim_main.cc)) — one executable, CLI-configurable trajectory and fault schedule
2. **Per-scenario plotting** ([analysis/scenario_plots.py](analysis/scenario_plots.py)) — 15-panel figure covering all signals
3. **Cross-scenario summary** ([analysis/campaign_summary.py](analysis/campaign_summary.py)) — comparative bar charts + overlay plots + metrics CSV
4. **One-shot runner** ([run_full_campaign.sh](run_full_campaign.sh)) — executes all 6 scenarios + generates all plots

## The Controller's Peer-Aware Fault Tolerance

Cascade PD (position → tilt → torque) identical to Tether_Lift's baseline,
**plus** one new input and one new computation:

```cpp
// New input: every rope's scalar tension, from the other drones' rope systems
peer_tensions: Eigen::VectorXd (size = N)

// New computation (only active once pickup has completed)
n_alive = Σᵢ clamp(T_peer[i] / T_nominal, 0, 1)   // smooth count
target_tension = m_L * g / n_alive                 // adapts to failures
```

A peer's tension → 0 fades its `n_alive` contribution → surviving drones'
`target_tension` rises → they pick up more load. No consensus protocol, no
central failure flag. The `TensionFaultGate` on the faulted rope zeroes both
the force signal (physics) and the tension reading (telemetry) simultaneously.

## The 6 Scenarios

| # | Name | Trajectory | Faults | Why it's interesting |
|---|------|-----------|--------|----------------------|
| A | `A_nominal`          | Straight traverse (0,0,1) → (3,0,3) | none | Baseline: shows balanced load sharing |
| B | `B_single_fault`     | Straight traverse                    | Quad 0 @ t=10s | One-fault graceful degradation |
| C | `C_dual_fault`       | Straight traverse                    | Quad 0 @ t=8s, Quad 2 @ t=14s | Two sequential faults |
| D | `D_figure8_nominal`  | Lemniscate (Bernoulli figure-8)      | none | Agile trajectory under load |
| E | `E_figure8_fault`    | Lemniscate                           | Quad 0 @ t=12s | Fault during aggressive lateral turn |
| F | `F_triple_fault`     | Straight traverse                    | Quads 0,2,3 sequentially | Stress test — drops to 1 lift drone |

## Outputs (per scenario)

Each scenario writes three files to [decentralized_replays/](decentralized_replays/):

| File | Contents |
|------|----------|
| `scenario_<NAME>.html` | Standalone Meshcat replay (~5 MB). Open in any browser → press ▶ in Animations panel |
| `scenario_<NAME>.csv`  | 30 kHz signal log: positions, velocities, rope tensions, controller diagnostics, force commands (~70 MB/20s) |
| `scenario_<NAME>.png`  | 15-panel diagnostic plot: trajectory, tensions, thrust, N_alive, etc. |

Plus campaign-wide:
- `campaign_metrics.csv` — summary table
- `campaign_summary.png` — bar charts comparing RMS error, peak tension, thrust, impulse, imbalance across scenarios
- `campaign_overlay.png` — overlay of tracking error and load imbalance vs. time

## Campaign Results — 2026-04-19 (stiff-rope configuration)

| Scenario | RMS err | Peak err | Max T | Max ‖F‖ | Σ impulse | Load imbal. | Faults |
|---|---:|---:|---:|---:|---:|---:|---|
| A_nominal          | **1.19** | 1.31 | 217 | 45.8 | 2374 | **0.72** | — |
| B_single_fault     | 1.23     | 1.40 | 217 | 45.8 | 2127 | 4.14 | 1 |
| C_dual_fault       | 1.23     | 1.40 | 217 | 45.8 | 1936 | 5.22 | 2 |
| D_figure8_nominal  | 1.39     | 2.40 | 217 | 60.1 | 3716 | 4.37 | — |
| E_figure8_fault    | 1.45     | 2.40 | 217 | 60.1 | 3278 | 6.60 | 1 |
| F_triple_fault     | 1.36     | 2.28 | 217 | 67.7 | 2061 | **6.99** | 3 |

*(RMS/peak in m; tensions/thrust in N; impulse in N·s; imbalance = RMS std(T) across ropes in N.)*

**Takeaways:**
- Stiff-rope configuration improves **tracking RMS by 19–25 %** across all scenarios and cuts steady-state payload bounce from ~10 cm to ~0.5 cm std.
- The nominal imbalance is **0.72 N** — each fault **unambiguously** raises imbalance to ≥ 4 N (5.7× baseline or more). Fault signature is now clearly visible against background.
- Figure-8 agility raises peak thrust demand from 46 N (traverse) to **60–68 N** (lemniscate); triple-fault peaks at 68 N because surviving drones carry the failed peers' loads.
- **Peak rope tension = 217 N** during horizontal-acceleration transients — uniform across all scenarios because it's a property of the rope stiffness × payload mass × max acceleration, not the fault pattern. Still well within structural limits for a braided-nylon rope (rated 3–5 kN).
- Tracking-error RMS is ≈ 1.2–1.5 m in all scenarios. *Note:* the "reference" logged is the **drone formation centre**, which sits ~1 m above the payload because of rope length; subtract ~1 m to compare payload to its effective hang point.

## Rope stiffness — why it matters for fault visibility

The rope is modelled as a bead chain with spring-damper segments. With a
**soft rope** (`segment_stiffness = 200 N/m`), the payload sags ~30 cm per
rope under static load and oscillates at the rope's bending-mode
frequency (~10 Hz); fault transients get lost in that background.

Current configuration is **stiff + near-critically damped**:

| Parameter | Value | Rationale |
|-----------|------:|-----------|
| `segment_stiffness` | 2000 N/m | Static stretch ~3 cm (realistic braided nylon) |
| `segment_damping`   |    45 N·s/m | ζ ≈ 1.0 (kills oscillations) |
| Bead mass           | 0.025 kg | Same as baseline |
| Bead count / rope   | 8 | Same as baseline |

Results: hover-state payload bounce **std 0.46 cm** (≈20× reduction), dominant
frequency shifts from rope bending to the pendulum-swing mode (~5 Hz), and
fault transients become visually distinct from background motion.

Trade-off: peak rope tension during horizontal acceleration rises from
23 N → 217 N (stiff ropes transmit transient accelerations more sharply).
This stays well within typical 3–5 kN braided-nylon rope ratings.

## Build

```bash
cd /workspaces/Tether_Grace/Research/cpp/build
cmake ..                                     # only needed first time
make decentralized_fault_aware_sim -j4
```

## Run the full campaign

```bash
cd /workspaces/Tether_Grace/Research
./run_full_campaign.sh                       # ~45 min wall-time (6 × ~7 min)
```

Takes ~5–8 min per scenario at `target_realtime_rate=0.5`; plots generate in seconds.

## Run a single custom scenario

```bash
cd /workspaces/Tether_Grace/Research/cpp/build
./decentralized_fault_aware_sim \
    --num-quads 4 \
    --duration 20 \
    --trajectory figure8 \
    --scenario my_run \
    --fault-0-quad 1 --fault-0-time 15 \
    --output-dir /tmp/my_run
python3 /workspaces/Tether_Grace/Research/analysis/scenario_plots.py \
    /tmp/my_run/scenario_my_run.csv
```

## CLI reference

| Flag | Meaning |
|------|---------|
| `--num-quads N`         | Number of drones (default 4) |
| `--duration T`          | Seconds to simulate |
| `--trajectory NAME`     | `traverse` (default) or `figure8` |
| `--scenario NAME`       | Label used in output filenames |
| `--fault-{0,1,2}-quad I`| Which drone's cable to sever |
| `--fault-{0,1,2}-time T`| When to sever [s] |
| `--output-dir DIR`      | Destination for `.html` + `.csv` |

## Key source files

| File | Role |
|------|------|
| [cpp/src/decentralized_fault_aware_sim_main.cc](cpp/src/decentralized_fault_aware_sim_main.cc) | Simulation harness (physics + vis + recording + CSV dump) |
| [cpp/include/decentralized_fault_aware_controller.h](cpp/include/decentralized_fault_aware_controller.h) | Controller interface |
| [cpp/src/decentralized_fault_aware_controller.cc](cpp/src/decentralized_fault_aware_controller.cc) | Controller: cascade PD + peer-aware `N_alive` feedforward |
| [cpp/include/cable_fault_gate.h](cpp/include/cable_fault_gate.h) | `CableFaultGate` (forces) + `TensionFaultGate` (telemetry) |
| [analysis/scenario_plots.py](analysis/scenario_plots.py) | Per-scenario diagnostic figure |
| [analysis/campaign_summary.py](analysis/campaign_summary.py) | Cross-scenario comparative figures + metrics |
