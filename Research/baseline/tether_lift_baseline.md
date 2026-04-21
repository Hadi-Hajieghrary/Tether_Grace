# Tether_Lift Baseline Reconstruction

## Purpose

This note records the verified baseline that new work in [../..](../..) should inherit from [../../Tether_Lift](../../Tether_Lift).

The key distinction is simple:

- The repository contains a broad research story.
- The live baseline executable is materially narrower than that story.

New work in `Research/` should build on the live baseline first, then decide deliberately which dormant branches to revive.

## Repository Boundary

### First-party research scope

- [../../Tether_Lift/README.md](../../Tether_Lift/README.md)
- [../../Tether_Lift/Research](../../Tether_Lift/Research)
- [../../Tether_Lift/IEEE_IROS_2026](../../Tether_Lift/IEEE_IROS_2026)
- [../../Tether_Lift/outputs](../../Tether_Lift/outputs)

### Dependency boundary

- [../../Tether_Lift/drake](../../Tether_Lift/drake) is a vendored dependency tree.

It matters for API semantics and runtime behavior, but it is not the research contribution that this knowledge base is trying to preserve.

## Canonical Entry Points

### Live runtime entry points

1. [../../Tether_Lift/Research/cpp/src/main.cc](../../Tether_Lift/Research/cpp/src/main.cc)
2. [../../Tether_Lift/Research/scripts/run_monte_carlo.sh](../../Tether_Lift/Research/scripts/run_monte_carlo.sh)
3. [../../Tether_Lift/Research/scripts/generate_figures.py](../../Tether_Lift/Research/scripts/generate_figures.py)
4. [../../Tether_Lift/Research/scripts/generate_video.py](../../Tether_Lift/Research/scripts/generate_video.py)

### Dormant or secondary entry points

1. [../../Tether_Lift/Research/scripts/quad_rope_lift.py](../../Tether_Lift/Research/scripts/quad_rope_lift.py)
2. [../../Tether_Lift/Research/cpp/gpac/src/main_gpac.cc](../../Tether_Lift/Research/cpp/gpac/src/main_gpac.cc)

## What The Baseline Actually Runs

### Runtime summary

The active baseline executable is `quad_rope_lift`, built from [../../Tether_Lift/Research/cpp/CMakeLists.txt](../../Tether_Lift/Research/cpp/CMakeLists.txt) and wired in [../../Tether_Lift/Research/cpp/src/main.cc](../../Tether_Lift/Research/cpp/src/main.cc).

The live Drake diagram contains:

- a `MultibodyPlant` plus `SceneGraph`
- `N` quadcopters as free rigid bodies
- one spherical payload
- one bead-chain rope model per quad via `RopeForceSystem`
- one `QuadcopterLiftController` per quad
- GPS, IMU, and barometer sensors
- one `PositionVelocityEstimator` per quad
- one `DecentralizedLoadEstimator` per quad
- wind disturbance plus `WindForceApplicator`
- Meshcat visualization and replay export
- `SimulationDataLogger` plus CSV sinks

### Control reality

The live controller is the tension-aware cascaded PD controller in [../../Tether_Lift/Research/cpp/src/quadcopter_controller.cc](../../Tether_Lift/Research/cpp/src/quadcopter_controller.cc).

Its defining behaviors are:

- piecewise-linear waypoint tracking
- small-angle roll/pitch commands from horizontal position error
- attitude PD on roll, pitch, and yaw
- pickup-phase tension ramping and thrust compensation
- equal nominal load-share target `payload_weight / N`

### Estimation reality

The baseline creates estimators, but it does not close the default controller loop through them.

- `enable_estimation = true`
- `use_estimated_in_controller = false`

Practical meaning:

- GPS and other sensors are simulated and logged.
- `PositionVelocityEstimator` and `DecentralizedLoadEstimator` run.
- The controller still uses plant truth by default.

### Logging reality

The baseline produces honest evidence mainly for:

- trajectories
- cable tensions
- control effort
- GPS measurements
- estimator outputs
- IMU measurements
- barometer measurements
- attitude traces
- wind disturbance
- configuration text
- Meshcat HTML replay

The default run does not produce baseline evidence for GPAC layers primarily because those systems are never instantiated or connected in the active Drake diagram. The configuration flags `log_gpac_signals = false` and `log_rope_states = false` simply reflect that structural absence in the default runtime.

## Module Maturity Matrix

| Module or subsystem | Role | Runtime status | Carry-forward meaning |
|---|---|---|---|
| `RopeForceSystem` | Bead-chain tether dynamics | Live | Core baseline physics; safe starting point, but potentially expensive and stiff |
| `rope_utils` | Rope parameter sizing and slack initialization | Live | Important for setup and sign conventions |
| `QuadcopterLiftController` | Baseline lift controller | Live | This is the actual nominal controller, not GPAC |
| `PositionVelocityEstimator` | GPS-based quad state estimate | Live observer branch | Useful baseline estimator, but not closed into control |
| `DecentralizedLoadEstimator` | Local load-state estimate from cable geometry | Live observer branch | Valuable extension seam; baseline downstream usage is limited |
| `WindDisturbance` + `WindForceApplicator` | Disturbance injection | Live | Reusable for robustness experiments |
| `SimulationDataLogger` | CSV evidence backbone | Live | Should be preserved and improved, not discarded |
| `TrajectoryVisualizer` | Reference/trail visualization | Live | Useful presentation layer; not core control logic |
| `EskfEstimator` | 15-state inertial fusion | Compiled, not wired into default run | Candidate for future activation |
| `GPACLoadTrackingController` | Layer-1 load-centric / anti-swing path | Compiled, not wired | Research branch, not baseline fact |
| `GPACQuadcopterController` | Layer-2 geometric SO(3) control | Compiled, not wired | Research branch, not baseline fact |
| `ConcurrentLearningEstimator` | CL adaptation branch | Present, not active in baseline runtime | Needs explicit integration and evidence |
| `ExtendedStateObserver` | ESO disturbance observer | Present, not active in baseline runtime | Candidate for future disturbance feedforward work |
| `GPACCbfSafetyFilter` | Safety filter | Present, not active in baseline runtime | Current implementation is not a full QP-based layer |
| Load-centric pipeline (`LoadTrackingController`, `RequiredForceComputer`, `ForceAllocationSystem`, `DroneTrajectoryMapper`, `ExtendedLoadTrajectoryGenerator`) | More structured future controller decomposition | Compiled but dormant in default path | Strong candidate for successor architecture |
| `Research/cpp/gpac/*` | Separate GPAC harness | Dormant / disconnected from active build path | Treat as prototype scaffold, not baseline executable |
| `Research/scripts/quad_rope_lift.py` | Earlier 2D single-quad prototype | Secondary / historical | Useful intuition, not the multi-quad baseline |

## Paper Story Versus Baseline Reality

The repository contains a richer GPAC manuscript in [../../Tether_Lift/IEEE_IROS_2026](../../Tether_Lift/IEEE_IROS_2026), but the active baseline executable does not currently realize the full story described there.

The safest baseline interpretation is:

- The repository preserves both a live legacy runtime and a more ambitious but only partially integrated GPAC branch.
- The paper is valuable as a design target and theory reference.
- The paper should not be treated as a direct description of what the default executable runs today.

Use [paper_vs_code_matrix.md](paper_vs_code_matrix.md) before carrying any claim from manuscript to new work.

## Reproducibility Notes

### What is reproducible today

- The live baseline executable can be built from [../../Tether_Lift/Research/cpp](../../Tether_Lift/Research/cpp).
- Short or long headless runs can be executed with an explicit `--output-dir` override.
- The committed outputs under [../../Tether_Lift/outputs](../../Tether_Lift/outputs) provide historical evidence for baseline trajectories and summary metrics.

### Known reproducibility pain points

1. Default output paths are pinned to `/workspaces/Tether_Lift`, not the current workspace layout.
2. `--headless` does not actually disable Meshcat initialization or HTML export.
3. The figure-generation pipeline mixes empirical plots with synthetic or hard-coded paper-support artifacts.
4. There is no single manifest linking manuscript claims, figure IDs, committed outputs, and exact runtime configurations.

### Workspace-safe baseline build pattern

```bash
cd /workspaces/Tether_Grace/Tether_Lift/Research/cpp
mkdir -p build
cd build
cmake .. -G Ninja -DCMAKE_PREFIX_PATH=/opt/drake
ninja
./quad_rope_lift --headless --duration 5 --num-quads 3 --output-dir /workspaces/Tether_Grace/Research/outputs/baseline_smoke_test
```

## Baseline Constraints New Work Must Respect

1. Do not assume the GPAC stack is active until a successor executable wires and logs it explicitly.
2. Do not assume the paper's headline metrics map cleanly to the committed baseline outputs without a provenance table.
3. Do not treat the vendored Drake source as research content; treat it as dependency behavior.
4. Do not use the default output-path assumptions from Tether_Lift in new work.
5. Do not erase the current logging backbone; future work should extend it with better provenance rather than replace it blindly.

## Immediate Extension Implications

For new work in [../cpp](../cpp) and [../scripts](../scripts), the most robust strategy is:

1. Reproduce the live baseline honestly.
2. Add a successor executable with explicit module wiring and a machine-readable run manifest.
3. Introduce dormant GPAC, ESKF, or load-centric modules one layer at a time, with logging added before claiming performance.
4. Keep a strict separation between empirical figures and illustrative or synthetic figures.