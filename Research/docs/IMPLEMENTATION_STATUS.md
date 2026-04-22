# Implementation Status

Single source of truth for the active code tree. Every source file
under `Research/cpp/` and every script under `Research/analysis/` or
`Research/scripts/` is listed here with its purpose, its theoretical
reference, and — where applicable — which artefact under `output/`
it produces. If a file is not listed here, it is not part of the
build; anything historical lives under [`../../archive/`](../../archive).

## High-level diagram

```
         reference trajectory (waypoint interpolator)
                   │  p_ref(t), v_ref(t)
                   ▼
   ┌───────────────────────────┐           ┌──────────────────┐
   │   FormationCoordinator    │◄──────────┤   FaultDetector  │
   │  (dynamic slot offsets)   │  fault_id │  (tension-below- │
   └──────────────┬────────────┘           │   threshold latch)│
                  │ 3N offsets              └──────▲────────────┘
                  ▼                                │
  ┌────────────────────────────────────────────────┴──────────┐
  │      Per-drone controller  (one of two implementations)   │
  │   DecentralizedLocalController   (QP + optional L1)       │
  │   MpcLocalController             (N-step QP with          │
  │                                   tension-ceiling rows)   │
  └──────────────────────────────┬────────────────────────────┘
                                 │ spatial force
                                 ▼
           ┌────────────────────────────────────────┐
           │    Drake MultibodyPlant + SceneGraph    │
           │   quadrotor URDFs + bead-chain ropes    │
           │   fault gates + Meshcat visualiser      │
           └────────────────────────────────────────┘
```

## C++ source

| File | Role |
|------|------|
| [`cpp/src/decentralized_fault_aware_sim_main.cc`](../cpp/src/decentralized_fault_aware_sim_main.cc) | Simulator entry point. Builds the diagram, parses CLI flags, instantiates controllers, wires logging, drives the simulation. |
| [`cpp/src/decentralized_local_controller.cc`](../cpp/src/decentralized_local_controller.cc) · [`cpp/include/decentralized_local_controller.h`](../cpp/include/decentralized_local_controller.h) | Baseline controller. Cascade outer-loop PD, anti-swing feed-forward, per-tick 3-variable QP, Lee-style attitude inner loop, optional L1 adaptive augmentation. |
| [`cpp/src/mpc_local_controller.cc`](../cpp/src/mpc_local_controller.cc) · [`cpp/include/mpc_local_controller.h`](../cpp/include/mpc_local_controller.h) | Receding-horizon MPC replacement. Error-state double-integrator prediction, condensed Φ/Ω, DARE terminal cost, linearised tension-ceiling constraints with soft slack. |
| [`cpp/src/cl_param_estimator.cc`](../cpp/src/cl_param_estimator.cc) · [`cpp/include/cl_param_estimator.h`](../cpp/include/cl_param_estimator.h) | Concurrent-learning observer for (m_L / N, k_rope). Diagnostic channel only; wired when `--adaptive` is passed. |
| [`cpp/include/fault_detector.h`](../cpp/include/fault_detector.h) | Header-only latched detector that declares drone `i` faulted once its tension has stayed below threshold for `detect_duration`. |
| [`cpp/include/formation_coordinator.h`](../cpp/include/formation_coordinator.h) | Header-only supervisor that, given a latched fault id, emits dynamic formation offsets with a quintic smoothstep transition. |
| [`cpp/include/controller_utils.h`](../cpp/include/controller_utils.h) | Shared helpers: slot and payload-reference interpolation, scalar DARE, LQR gain, closed-loop spectral radius. |
| [`cpp/src/fault_aware_rope_visualizer.cc`](../cpp/src/fault_aware_rope_visualizer.cc) · [`cpp/include/fault_aware_rope_visualizer.h`](../cpp/include/fault_aware_rope_visualizer.h) | Meshcat polyline of each rope chord; hides the severed rope after fault time. |
| [`cpp/include/cable_fault_gate.h`](../cpp/include/cable_fault_gate.h) | Zeroes the tension feeding the rope's physics after the fault time (physical fault). |
| [`cpp/include/rope_segment_tension_probe.h`](../cpp/include/rope_segment_tension_probe.h) | Per-segment rope-tension logger. |
| [`cpp/include/safe_hover_controller.h`](../cpp/include/safe_hover_controller.h) · [`cpp/include/control_mode_switcher.h`](../cpp/include/control_mode_switcher.h) | Post-fault autopilot and mode-selector for the faulted drone. |
| [`cpp/include/meshcat_fault_hider.h`](../cpp/include/meshcat_fault_hider.h) | Periodic-update helper that walks the Meshcat scene to hide a severed rope's visual assets. |

Build: `cmake -S cpp -B cpp/build -DCMAKE_BUILD_TYPE=Release && cmake --build cpp/build`. Single executable: `decentralized_fault_aware_sim`.

## Command-line reference

| Flag | Default | Description |
|------|---------|-------------|
| `--num-quads N` | 4 | Number of drones. Tested for N ∈ {4, 5, 6}. |
| `--duration T` | 20 | Simulation wall-time (s). |
| `--output-dir path` | `./decentralized_replays` | Destination for CSV log + Meshcat HTML replay. |
| `--scenario name` | `A_nominal` | Informational tag used in the replay filename. |
| `--trajectory {traverse,figure8,lemniscate3d}` | `traverse` | Reference trajectory class. |
| `--fault-k-quad i` · `--fault-k-time t` | none | Inject fault `k ∈ {0,1,2}` on drone `i` at time `t`. |
| `--controller {baseline,mpc}` | `baseline` | Per-drone controller. |
| `--mpc-horizon N` | 5 | MPC prediction horizon (ticks). |
| `--mpc-tension-max N` | 100 | MPC tension ceiling (N). |
| `--l1-enabled` | off | Enables the L1 adaptive augmentation inside the baseline controller. |
| `--adaptive` | off | Runs the CL observer in parallel (diagnostic). |
| `--reshaping-enabled` | off | Instantiates FaultDetector + FormationCoordinator. |
| `--payload-mass kg` | 3.0 | Override the payload mass (robustness sweeps). |

## Python analysis

| Script | Role | Input | Output |
|--------|------|-------|--------|
| [`analysis/ieee/plot_publication.py`](../analysis/ieee/plot_publication.py) | Full 12-figure suite per campaign. | per-scenario CSVs under `output/<campaign>/08_source_data/` | `output/<campaign>/09_publication_figures/` |
| [`analysis/ieee/plot_scenario.py`](../analysis/ieee/plot_scenario.py) | Per-scenario 10-PNG set. | one CSV | per-scenario subfolder |
| [`analysis/ieee/plot_comparison.py`](../analysis/ieee/plot_comparison.py) | Cross-scenario grouped bar charts. | scenario-level summary metrics | `07_cross_scenario_comparison/` |
| [`analysis/ieee/plot_ablation.py`](../analysis/ieee/plot_ablation.py) | Cross-configuration ablation bars + summary heat-map. | `output/ablation/summary_metrics.csv` | per-config `ablation_figures/` |
| [`analysis/ieee/plot_system.py`](../analysis/ieee/plot_system.py) | Renders the architecture block-diagram SVG/PDF. | nothing | `00_system_architecture/` |
| [`analysis/ieee/ieee_style.py`](../analysis/ieee/ieee_style.py) | matplotlib rc defaults + figure sizes. | — | — |
| [`analysis/ieee/fill_results_placeholders.py`](../analysis/ieee/fill_results_placeholders.py) | Numeric substitution pass over the LaTeX results section. | metrics CSV + LaTeX template | filled LaTeX |
| [`analysis/ieee/write_readmes.py`](../analysis/ieee/write_readmes.py) | Generates per-scenario README snippets. | metrics CSV | per-scenario READMEs |

## Campaign runners

| Script | What it runs |
|--------|--------------|
| [`scripts/run_5drone_campaign.sh`](../scripts/run_5drone_campaign.sh) | Five-drone A/B/C/D sweep under the lemniscate3d reference. |
| [`scripts/run_mc_interfault_sweep.sh`](../scripts/run_mc_interfault_sweep.sh) | Monte-Carlo sweep over inter-fault separation (5-drone). |
| [`scripts/run_ablation_campaign.sh`](../scripts/run_ablation_campaign.sh) | Four-configuration ablation matrix (baseline, +L1, +MPC, fullstack). |
| [`scripts/run_transactions_campaign.sh`](../scripts/run_transactions_campaign.sh) | Full IEEE-Transactions Monte-Carlo matrix (494 runs, 6 configs × 5 scenarios × 5 seeds + robustness sweeps). |

## Where numbers in the paper come from

| Artefact | Source |
|----------|--------|
| LaTeX manuscript draft | [`docs/latex/tether_grace_reference.pdf`](latex/tether_grace_reference.pdf) built from [`docs/latex/tether_grace_reference.tex`](latex/tether_grace_reference.tex). |
| Methods section | [`docs/latex/methods_section.tex`](latex/methods_section.tex). |
| Results section | [`docs/latex/results_section.tex`](latex/results_section.tex), populated by `fill_results_placeholders.py`. |
| Pre-registered experiment plan | [`docs/preregistration.md`](preregistration.md). |
| Formal proofs | [`docs/latex/supplementary.tex`](latex/supplementary.tex). |

Every number in the manuscript maps to an `experiment_id` in
[`docs/preregistration.md`](preregistration.md); the same id labels the
corresponding row of `output/<campaign>/publication_metrics.csv`.

## Output tree layout

```
output/
└── 5drone_baseline_campaign/
    ├── 01_scenario_A_nominal/         per-scenario plots
    ├── 02_scenario_B_single_fault/
    ├── 03_scenario_C_dual_5sec/
    ├── 04_scenario_D_dual_10sec/
    ├── 07_cross_scenario_comparison/  cross-scenario grouped bars
    ├── 08_source_data/                raw per-tick CSVs (gitignored)
    ├── 09_publication_figures/        F01..F12 suite
    ├── monte_carlo/                   inter-fault Δt sweep
    └── README.md                      campaign summary
```

When the Transactions campaign runs, its output lands alongside in
`output/transactions_campaign/` with the same structure plus one
sub-folder per controller configuration.
