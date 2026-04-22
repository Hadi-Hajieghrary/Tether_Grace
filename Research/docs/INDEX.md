# Documentation Index — Tether_Grace

Everything you might want to read, grouped by *why* you would open it.

## 0. Canonical code map (read first)

| Document | Read it to learn… |
|----------|-------------------|
| [`IMPLEMENTATION_STATUS.md`](IMPLEMENTATION_STATUS.md) | **Single source of truth.** Every active `.cc`/`.h`/`.py`/`.sh` file in the tree, what it does, which CLI flag turns it on, and which theory doc matches it. Start here. |
| [`EXTENSION_PLAN.md`](EXTENSION_PLAN.md) | Master roadmap: three extensions (L1, MPC, formation reshape), current implementation status, cross-extension dependencies. |

## 1. Paper-facing summaries

| Document | Read it to learn… |
|----------|-------------------|
| [`../output/Tether_Grace/README.md`](../../output/Tether_Grace/README.md) | What the 4-drone campaign archive contains, headline numbers, how to reproduce. |
| [`../../output/Tether_Grace_5drone/README.md`](../../output/Tether_Grace_5drone/README.md) | 5-drone lemniscate-3D campaign (A/B/C/D) + Monte-Carlo sweep results. |
| [`../README.md`](../README.md) | One-page concise overview of the workspace (code + docs boundary, entry points, where history lives). |
| [`../DECENTRALIZED_FAULT_AWARE_README.md`](../DECENTRALIZED_FAULT_AWARE_README.md) | High-level design narrative: scope, controller, scenarios, system diagram, parameter table, campaign results. |

## 1a. Pedagogical / teaching material

| Document | Target reader |
|----------|---------------|
| [`case_study_tether_grace.md`](case_study_tether_grace.md) | Graduate robotics / controls student — self-contained case study covering problem framing → multibody & rope model → fully-local QP controller → fault-tolerance mechanism → Drake simulation framework → six-scenario campaign → exercises and further reading. Every numerical claim is evidence-tagged. |

## 2. Theory — equations mapped to code

Every doc uses KaTeX-compatible math and tags numbered equations with
the `file:line` of the implementing code. Each extension doc's header
now carries a one-block "Status: Implemented" pointer to the active
class + CLI flag + empirical result.

### 2a. Baseline (4 docs)

| Document | What it derives |
|----------|-----------------|
| [`theory/theory_decentralized_local_controller.md`](theory/theory_decentralized_local_controller.md) | Cascade PD + per-step QP + anti-swing, constraints, saturation, stability discussion. |
| [`theory/theory_rope_dynamics.md`](theory/theory_rope_dynamics.md) | 8-bead Kelvin-Voigt rope model, effective $EA/L$, catenary drape, $\omega_n$ stability bound, `RopeSegmentTensionProbe`. |
| [`theory/theory_fault_model.md`](theory/theory_fault_model.md) | Four simultaneous fault gates (physical, telemetry, visual, supervisory), emergent surviving-drone load pickup. |
| [`theory/theory_figure8_trajectory.md`](theory/theory_figure8_trajectory.md) | Bernoulli lemniscate parametric form, waypoint interpolation, velocity/acceleration analysis, bandwidth sizing. |

### 2b. Extensions (3 docs) — *all implemented as of 2026-04-22*

| Document | What it adds | Active in |
|----------|--------------|-----------|
| [`theory/theory_l1_extension.md`](theory/theory_l1_extension.md) | L1 adaptive outer loop: state predictor, Lyapunov $P$, $\sigma$-projection, LP filter $\omega_c$. | `DecentralizedLocalController::CalcL1Update` (flag `--l1-enabled`) |
| [`theory/theory_mpc_extension.md`](theory/theory_mpc_extension.md) | Receding-horizon QP with linearised hard tension-ceiling constraint, DARE terminal cost, soft slack fallback. | `MpcLocalController` (flag `--controller=mpc`) |
| [`theory/theory_reshaping_extension.md`](theory/theory_reshaping_extension.md) | Closed-form equiangular reassignment for N=4 → M=3 (π/6 rotation); quintic $C^2$ smoothstep transition; collision-safety proof; fault-ID broadcast only. | `FaultDetector` + `FormationCoordinator` (flag `--reshaping-enabled`) |

## 3. Results — scenario-by-scenario

Two independent campaigns validate the controller at different fleet sizes.

### 3a. Original 4-drone campaign

Each scenario folder under [`../../output/Tether_Grace/`](../../output/Tether_Grace/)
contains 10 IEEE-style PNGs, the HTML Meshcat replay, and a `README.md`
that explains every figure.

| Scenario | Folder | Claim evidenced |
|----------|--------|-----------------|
| **S1** nominal traverse           | [`01_scenario_S1_nominal_traverse`](../../output/Tether_Grace/01_scenario_S1_nominal_traverse) | Baseline; 0.16 m RMS payload track. |
| **S2** single cable fault         | [`02_scenario_S2_cruise_fault`](../../output/Tether_Grace/02_scenario_S2_cruise_fault) | Graceful degradation, tracking barely degraded. |
| **S3** dual sequential fault      | [`03_scenario_S3_dual_sequential`](../../output/Tether_Grace/03_scenario_S3_dual_sequential) | Two failures compose additively. |
| **S4** figure-8 nominal           | [`04_scenario_S4_figure8_nominal`](../../output/Tether_Grace/04_scenario_S4_figure8_nominal) | Agile trajectory, pendulum-lag-limited 0.43 m RMS. |
| **S5** fault during figure-8      | [`05_scenario_S5_figure8_fault`](../../output/Tether_Grace/05_scenario_S5_figure8_fault) | Fault mid-manoeuvre, no extra RMS cost. |
| **S6** triple sequential fault    | [`06_scenario_S6_triple_stress`](../../output/Tether_Grace/06_scenario_S6_triple_stress) | Stress limit — one drone alone carrying 3 kg. |
| comparison bars + overlays        | [`07_cross_scenario_comparison`](../../output/Tether_Grace/07_cross_scenario_comparison) | Cross-scenario metric summary. |

### 3b. 5-drone dynamic-lemniscate campaign — *with Phase-A/B refinements*

A harder stress test with **N = 5 drones** on a 3-D lemniscate. The
2026-04-21 refresh re-ran the full campaign with the following
improvements (see
[`../DECENTRALIZED_FAULT_AWARE_README.md`](../DECENTRALIZED_FAULT_AWARE_README.md)
§ 7.5 for details):

* pre-tensioned hover-equilibrium initial conditions (no pickup snap-taut)
* smoothstep ($C^1$) pickup ramp co-phased across $T_{\text{ff}}$ and
  tension-feedback
* 13-signal controller diagnostics (QP active-set + solve-time + thrust /
  tilt envelope occupancy)
* per-segment tension probe ($T_{i,j}(t)$) for the rope-waterfall figure
* full 12-figure IEEE publication suite
  ([`09_publication_figures/`](../../output/Tether_Grace_5drone/09_publication_figures/))

Output at
[`../../output/Tether_Grace_5drone/`](../../output/Tether_Grace_5drone/).

| Scenario | Folder | Demonstrates |
|----------|--------|--------------|
| **A** 5-drone nominal         | [`01_scenario_A_nominal`](../../output/Tether_Grace_5drone/01_scenario_A_nominal) | Steady-motion baseline by t=8 s |
| **B** single fault @ 15 s     | [`02_scenario_B_single_fault`](../../output/Tether_Grace_5drone/02_scenario_B_single_fault) | Mid-trajectory rebalancing |
| **C** dual, 5 s apart (15, 20) | [`03_scenario_C_dual_5sec`](../../output/Tether_Grace_5drone/03_scenario_C_dual_5sec) | Compound failure before settlement |
| **D** dual, 10 s apart (15, 25)| [`04_scenario_D_dual_10sec`](../../output/Tether_Grace_5drone/04_scenario_D_dual_10sec) | Inter-fault settlement; contrast vs C |
| comparison bars + overlays     | [`07_cross_scenario_comparison`](../../output/Tether_Grace_5drone/07_cross_scenario_comparison) | Key finding: peak-T jumps from 271 N (10 s apart) to 344 N (5 s apart) |

## 4. Code — where the math lives

| Theory concept | Implementing file |
|----------------|-------------------|
| Per-drone local QP controller | [`../cpp/src/decentralized_local_controller.cc`](../cpp/src/decentralized_local_controller.cc) |
| Simulation harness (plant, rope systems, visualisation, fault wiring) | [`../cpp/src/decentralized_fault_aware_sim_main.cc`](../cpp/src/decentralized_fault_aware_sim_main.cc) |
| Rope bead-chain spring-damper | [`../../Tether_Lift/Research/cpp/src/rope_force_system.cc`](../../Tether_Lift/Research/cpp/src/rope_force_system.cc) (Tether_Lift baseline) |
| Cable + tension fault gates | [`../cpp/include/cable_fault_gate.h`](../cpp/include/cable_fault_gate.h) |
| Safe-hover supervisor | [`../cpp/include/safe_hover_controller.h`](../cpp/include/safe_hover_controller.h), [`../cpp/include/control_mode_switcher.h`](../cpp/include/control_mode_switcher.h) |
| Meshcat rope-line fault hider | [`../cpp/src/fault_aware_rope_visualizer.cc`](../cpp/src/fault_aware_rope_visualizer.cc) |
| IEEE-style plotting pipeline | [`../analysis/ieee/`](../analysis/ieee/) |
| Per-rope per-segment tension observer | [`../cpp/include/rope_segment_tension_probe.h`](../cpp/include/rope_segment_tension_probe.h) |
| Publication-grade figure suite (12 figs + 2 supplementary) | [`../analysis/ieee/plot_publication.py`](../analysis/ieee/plot_publication.py) |
| Monte-Carlo inter-fault-gap sweep | [`../run_mc_interfault_sweep.sh`](../run_mc_interfault_sweep.sh) |
| Waypoint interpolation | `ComputePayloadReference` inside [`../cpp/src/decentralized_local_controller.cc`](../cpp/src/decentralized_local_controller.cc) |

## 5. Archives (provenance only)

| Folder | Contents |
|--------|----------|
| [`../archive_obsolete_designs/`](../archive_obsolete_designs/) | Six superseded intermediate designs (Phase-1 single-drone MPC through peer-aware controller), each with a `NOTES.txt`. |
| [`../archive_session_notes_2026-04/`](../archive_session_notes_2026-04/) | Dated 2026-04 session planning / progress notes + superseded LaTeX drafts. Kept for provenance; see `NOTES.txt` inside. |
| [`../../output/Tether_Grace/archive_prior_campaigns/`](../../output/Tether_Grace/archive_prior_campaigns/) | ≈ 509 MB of earlier-phase campaign outputs (HTML replays, CSVs, logs). |
| [`../../output/Tether_Grace/archive_phase_docs/`](../../output/Tether_Grace/archive_phase_docs/) | Research-planning markdown from earlier sessions. |

## 6. Reproducing the archive

One command regenerates everything under `output/Tether_Grace/`:

```bash
cd /workspaces/Tether_Grace/Research
./run_ieee_campaign.sh
```

The campaign is deterministic — no random seeds — so every run
reproduces the same scenario outputs byte-for-byte (modulo solver
tolerance).
