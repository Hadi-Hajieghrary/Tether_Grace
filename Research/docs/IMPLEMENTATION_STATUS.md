# Implementation Status — Tether_Grace

*Single source of truth for "what is in the active codebase right now."*

Last refreshed: 2026-04-22 after the Phase-A → Phase-H sprint.

Every row below points at the **active code** (paths relative to
`Research/`) and its companion **theory doc**. If a file is not listed
here, it is not part of the current active tree — look under
`archive_obsolete_designs/` or `archive_session_notes_2026-04/` instead.

---

## Executive map

```
    ┌───────────────────────────┐
    │   Reference trajectory    │  (hard-coded waypoints +
    │   (waypoint interpolator) │   Bernoulli lemniscate / figure-8)
    └─────────────┬─────────────┘
                  │ p_ref(t), v_ref(t)
                  ▼
           ┌──────────────┐              ┌──────────────────┐
           │ Formation-   │◄──fault_id──┤   FaultDetector  │ Phase-H
           │ Coordinator  │              │   (observer)     │
           └──────┬───────┘              └───────▲──────────┘
                  │                               │
                  │ 3N dynamic formation offsets  │ N scalar tensions
                  ▼                               │
     ┌─────────────────────────────────────────────────────┐
     │      Per-drone fully-local controller                │
     │                                                       │
     │    ┌───────────────────────────────────────────┐     │
     │    │  DecentralizedLocalController (baseline)  │     │
     │    │      single-step QP + L1 adaptive         │     │
     │    │    (Phase-F: L1 inside; CL estimator      │     │
     │    │     optional diagnostic observer)         │     │
     │    └──────────────────────────┬────────────────┘     │
     │   OR                          │                       │
     │    ┌──────────────────────────▼────────────────┐     │
     │    │  MpcLocalController  (Phase-G)            │     │
     │    │     N-step QP + linearised tension rows   │     │
     │    │     + Aj−I + T_dot feed-forward           │     │
     │    └──────────────────────────┬────────────────┘     │
     └──────────────────────────────┬┴─────────────────────┘
                                    │ spatial force
                                    ▼
           ┌──────────────────────────────────────┐
           │  Drake MultibodyPlant + SceneGraph   │
           │  5-drone URDF + 9-segment bead chain │
           │  + four-gate fault model             │
           └──────────────────────────────────────┘
```

---

## Active code — every file accounted for

### C++ (Research/cpp/)

| File | Role | Theory doc |
|---|---|---|
| [`src/decentralized_fault_aware_sim_main.cc`](../cpp/src/decentralized_fault_aware_sim_main.cc) | Drake diagram builder, hover-equilibrium IC solver, CLI (`--controller`, `--l1-enabled`, `--adaptive`, `--reshaping-enabled`), CSV log writer. | — (it's the harness) |
| [`include/decentralized_local_controller.h`](../cpp/include/decentralized_local_controller.h) + [`src/decentralized_local_controller.cc`](../cpp/src/decentralized_local_controller.cc) | **Phase-A/B/C baseline controller**: cascade PD + single-step QP + anti-swing + pickup ramp + `initial_pretensioned` latch + expanded 13-signal diagnostics port. Now also carries the **Phase-F L1 adaptive outer loop** as an in-controller addition (11 `Params` fields, `CalcL1Update` method, 1-line `a_target.z() += u_ad` injection, 5-scalar `l1_state` output). | [theory_decentralized_local_controller.md](theory/theory_decentralized_local_controller.md), [theory_l1_extension.md](theory/theory_l1_extension.md) |
| [`include/mpc_local_controller.h`](../cpp/include/mpc_local_controller.h) + [`src/mpc_local_controller.cc`](../cpp/src/mpc_local_controller.cc) | **Phase-G receding-horizon MPC**. Drop-in replacement controller (same port signatures). Pre-computes $\Phi$, $\Omega$, constant Hessian $H$, DARE terminal cost $P_f$. Per-tick: reference-tracking cost $w_t\|U−a_\mathrm{target}\|^2 + w_e\|U\|^2$, linearised tension rows $C_T\,U \le d_T$ with $d_T$ using the $(A^j − I)\cdot x$ free-response term and a $\dot T_{\text{meas}}$ feed-forward, soft slack for feasibility. | [theory_mpc_extension.md](theory/theory_mpc_extension.md) |
| [`include/controller_utils.h`](../cpp/include/controller_utils.h) | Shared header: payload/slot-reference interpolator, scalar 2×2 DARE solver, LQR gain + spectral-radius helpers. Re-used by both controllers. | n/a |
| [`include/cl_param_estimator.h`](../cpp/include/cl_param_estimator.h) + [`src/cl_param_estimator.cc`](../cpp/src/cl_param_estimator.cc) | **Phase-F concurrent-learning observer**. Per-drone, two-channel CL (payload-EOM mass share + rope-axial stiffness). Disabled by default; enabled with `--adaptive`. Secondary diagnostic only; the primary adaptive action is L1 inside the baseline controller. | theory_l1_extension.md §6 (channel K) |
| [`include/fault_detector.h`](../cpp/include/fault_detector.h) | **Phase-H fault detector**. Header-only LeafSystem; latches `fault_id` once a drone's tension stays below 0.5 N for 100 ms. | [theory_reshaping_extension.md](theory/theory_reshaping_extension.md) §E |
| [`include/formation_coordinator.h`](../cpp/include/formation_coordinator.h) | **Phase-H formation-reshaping supervisor**. Closed-form 30°-toward-gap assignment for N=4 → M=3 + quintic smoothstep $h(\tau)=10\tau^3-15\tau^4+6\tau^5$ + 5-s transition. Single `fault_id` input, 3N-vector of dynamic formation offsets output. | [theory_reshaping_extension.md](theory/theory_reshaping_extension.md) §B-C |
| [`include/rope_segment_tension_probe.h`](../cpp/include/rope_segment_tension_probe.h) | Observer that exposes per-segment tension $T_{i,j}(t)$ of all $N_{\text{seg}}$ rope segments. Used to produce the F04 tension waterfall figure. Uses a cached plant context. | [theory_rope_dynamics.md](theory/theory_rope_dynamics.md) §9 |
| [`include/cable_fault_gate.h`](../cpp/include/cable_fault_gate.h) | Two `LeafSystem`s: `CableFaultGate` (zeros rope spatial forces post-fault), `TensionFaultGate` (zeros 4-scalar tension telemetry post-fault). Gates 1 and 2 of the four-gate fault model. | [theory_fault_model.md](theory/theory_fault_model.md) |
| [`include/fault_aware_rope_visualizer.h`](../cpp/include/fault_aware_rope_visualizer.h) + [`src/fault_aware_rope_visualizer.cc`](../cpp/src/fault_aware_rope_visualizer.cc) | Gate 3 — draws the rope polyline in Meshcat and cleanly removes it after fault_time. | theory_fault_model.md |
| [`include/control_mode_switcher.h`](../cpp/include/control_mode_switcher.h) + [`include/safe_hover_controller.h`](../cpp/include/safe_hover_controller.h) | Gate 4 — supervisory switch to safe-hover retreat for the faulted drone. | theory_fault_model.md |
| [`include/meshcat_fault_hider.h`](../cpp/include/meshcat_fault_hider.h) | Small utility that hides Meshcat breadcrumb trails for faulted drones. | n/a |

### Python (Research/analysis/ieee/)

| File | Role |
|---|---|
| [`ieee_style.py`](../analysis/ieee/ieee_style.py) | IEEE-Transactions rcParams, Wong colour-blind-safe palette, shared `BURN_IN_SECONDS = 0.5` constant. |
| [`plot_scenario.py`](../analysis/ieee/plot_scenario.py) | Per-scenario 10-figure pipeline (3-D traj, tracking err, tensions, thrust, etc.). |
| [`plot_comparison.py`](../analysis/ieee/plot_comparison.py) | Cross-scenario bar charts + overlays. |
| [`plot_system.py`](../analysis/ieee/plot_system.py) | System-architecture diagram generator (for `output/…/00_system_architecture/`). |
| [`plot_publication.py`](../analysis/ieee/plot_publication.py) | The **publication figure suite** (F01–F12 + S01–S02) — one PDF + PNG per figure. |
| [`PUBLICATION_FIGURES.md`](../analysis/ieee/PUBLICATION_FIGURES.md) | Reviewer-persona justification for each publication figure. |
| [`write_readmes.py`](../analysis/ieee/write_readmes.py) | Per-scenario README emitter. |
| [`fill_results_placeholders.py`](../analysis/ieee/fill_results_placeholders.py) | Populates the LaTeX `results_section.tex` `\NEW*` macros with real numbers. Uses the shared `BURN_IN_SECONDS`. |

### Runners (Research/)

| Script | What it runs |
|---|---|
| [`run_ieee_campaign.sh`](../run_ieee_campaign.sh) | 4-drone S1–S6 campaign + full figure pipeline. |
| [`run_5drone_campaign.sh`](../run_5drone_campaign.sh) | 5-drone lemniscate-3D A–D campaign + publication figures. |
| [`run_mc_interfault_sweep.sh`](../run_mc_interfault_sweep.sh) | Monte-Carlo inter-fault-gap sweep (6 points, `Δt ∈ {2, 4, 7, 10, 14, 20}` s). |

---

## CLI flag summary (what each turns on)

| Flag | Effect |
|---|---|
| *(default)* | Baseline controller, no extensions. Byte-for-byte reproducible. |
| `--controller=mpc` | Substitute the Phase-G MPC for the single-step QP. |
| `--mpc-horizon N` | MPC horizon length (steps); default 5. Each step is 10 ms. |
| `--mpc-tension-max N` | Hard tension-ceiling in N; default 100. |
| `--l1-enabled` | Enable the Phase-F L1 adaptive outer loop inside the baseline. |
| `--adaptive` | Attach the Phase-F CL observer to every drone (diagnostics only). |
| `--reshaping-enabled` | Instantiate the Phase-H FaultDetector + FormationCoordinator. Works with either controller. |
| `--payload-mass X` | Override the plant's payload mass (useful for testing L1). |

Orthogonality: extensions compose — e.g. `--controller=mpc --l1-enabled --reshaping-enabled` runs MPC + L1 + reshape together.

---

## Archived material (pointer only)

- [`../archive_obsolete_designs/`](../archive_obsolete_designs/) — six superseded prior-phase designs, each with its own `NOTES.txt`.
- [`../archive_session_notes_2026-04/`](../archive_session_notes_2026-04/) — dated session progress notes + superseded LaTeX drafts.
- [`../../output/Tether_Grace/archive_prior_campaigns/`](../../output/Tether_Grace/archive_prior_campaigns/) — ~500 MB of pre-refresh campaign artefacts.
- [`../../output/Tether_Grace/archive_phase_docs/`](../../output/Tether_Grace/archive_phase_docs/) — research-planning markdown from earlier sessions.

---

## Verifying the build

```bash
cd /workspaces/Tether_Grace/Research/cpp
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target decentralized_fault_aware_sim -j4
# →  Single executable:  build/decentralized_fault_aware_sim
```

The executable is the *only* binary in the active tree. Everything else
(scripts, Python, LaTeX) depends on that one C++ target.

---

## Status of each extension

| Phase | Extension | Status | Evidence |
|---|---|---|---|
| A | Hover-equilibrium IC + smoothstep pickup | ✅ Implemented | See comparison table in `DECENTRALIZED_FAULT_AWARE_README.md` § 7.5 |
| B | Per-segment tension probe + 13-signal diagnostics | ✅ Implemented | All figures use the extended CSV schema. |
| C | Publication figure suite (F01–F12 + S01–S02) | ✅ Implemented | `analysis/ieee/plot_publication.py` produces the full set. |
| D | Monte-Carlo inter-fault sweep | ✅ Implemented | `run_mc_interfault_sweep.sh` generated 6 runs; F10 scatter figure shows both fault-window and full-run peak-T envelopes. |
| E | Consolidated documentation | ✅ Implemented | You're reading it. |
| F | L1 adaptive outer loop | ✅ Implemented; **−16 % payload tracking-error** under 30 % $m_L$ bias | `--l1-enabled` |
| F (CL) | CL two-channel parameter estimator | ✅ Implemented; used as diagnostics | `--adaptive` |
| G | Receding-horizon MPC + hard tension ceiling | ✅ Implemented; matches baseline at $T \le 100$ N, physically caps real rope T under ceiling when ceiling is tight; slack fallback guarantees feasibility | `--controller=mpc` |
| G-2 | Richer tension linearisation (Aj−I + $\dot T$ feed-forward) | ✅ Implemented | Same flag; included in the MPC's per-tick constraint build |
| H | Formation-reshaping supervisor (closed-form 30° + quintic) | ✅ Implemented; drones verified to rotate to $(60°, 180°, 300°)$ after single fault; **tension-reduction claim is scenario-dependent and needs joint tuning with the descent profile** | `--reshaping-enabled` |
