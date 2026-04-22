# Tether_Lift Baseline Knowledge Base

This directory is the verified baseline map for the research inherited from [../../Tether_Lift](../../Tether_Lift).

Its purpose is to answer four questions before any new work starts:

1. What does Tether_Lift actually run today?
2. Which modules exist in code but are not active in the baseline executable?
3. Which paper or README claims are safe to inherit, and which ones require caution?
4. Where should new work in `Research/` start if we want to extend the baseline honestly?

## Documents

- [tether_lift_baseline.md](tether_lift_baseline.md): baseline inventory, runtime architecture, module maturity, reproducibility, and extension constraints.
- [paper_vs_code_matrix.md](paper_vs_code_matrix.md): claim-by-claim matrix separating live implementation facts from manuscript or README narrative.
- [research_agenda.md](research_agenda.md): practical next-step agenda for extending the baseline into new research territory.

## Baseline In One Sentence

The live Tether_Lift baseline is a Drake-based multi-quad cooperative lift simulator with bead-chain ropes, a tension-aware cascaded PD controller, estimator logging, wind disturbance, Meshcat replay, and CSV export; the richer GPAC, ESKF, concurrent-learning, and CBF stack exists in the repository but is not wired into the default runtime path.

## What Tether_Grace (This Repository) Has Verified

> **Architecture note (updated 2026-04-21):** The active controller is
> `DecentralizedLocalController` (fully-local cascade PD + QP per drone,
> `Research/cpp/src/decentralized_local_controller.cc`). The GPAC
> stack described below was an intermediate design phase; it has been
> superseded and is preserved in
> [`archive_obsolete_designs/`](../archive_obsolete_designs/) for
> provenance. The references to `output/gpac/` and `GPACQuadcopterController`
> below describe that prior state and are **not current**.

The `Research/` folder in this workspace has activated and validated
the **decentralised fault-aware QP lift** system as of 2026-04-21.
The confirmed working state is documented in
[`../DECENTRALIZED_FAULT_AWARE_README.md`](../DECENTRALIZED_FAULT_AWARE_README.md):

- **Fully-local `DecentralizedLocalController`** — cascade PD outer
  loop + per-step 3-variable QP + attitude inner loop, per drone, with
  no peer-state communication.
- **Cable-cut fault injection** (`CableFaultGate`) works: rope forces
  zero at fault time, surviving drones stabilise within ~one pendulum
  period.
- **Degraded-mode safe hover** (`SafeHoverController` +
  `ControlModeSwitcher`) works for the faulted drone.
- **Six 4-drone scenarios** (S1–S6) and **four 5-drone scenarios**
  (A–D) fully executed, logged, and analysed.
- **Pre-tensioned, free-fall-free initial conditions** and
  **smoothstep pickup ramp** implemented and validated.

---
*(Legacy section retained for provenance — describes the GPAC phase, not the active system)*

- **GPAC position and attitude control** (`GPACQuadcopterController`) was wired and running with verified numerical gains.
- **Cable-cut fault injection** (`CableFaultGate`) works: rope forces zero at fault time, no numerical instability.
- **Degraded-mode safe hover** (`SafeHoverController` + `ControlModeSwitcher`) works: faulted drone reaches its retreat position within 5 s.
- **Formation hold**: surviving 3 drones hold altitude within ±2 cm of target with redistributed tensions after the fault.
- **Adaptive load estimation** runs; convergence analysis is in progress.
- **ESKF sensor stack** runs but is not yet closed into the controller (plant truth is used). ESKF-to-controller closure is identified as the next integration step.

The key validated run (GPAC era, superseded) was `output/gpac/n4_cable_cut_t10_v21`. See [../README.md](../README.md) for the current verified result table.