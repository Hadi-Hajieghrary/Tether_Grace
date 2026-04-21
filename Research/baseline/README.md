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

The `Research/` folder in this workspace has activated and validated the GPAC stack as of April 2026. The following is the confirmed working state:

- **GPAC position and attitude control** (`GPACQuadcopterController`) is wired and running with verified numerical gains.
- **Cable-cut fault injection** (`CableFaultGate`) works: rope forces zero at fault time, no numerical instability.
- **Degraded-mode safe hover** (`SafeHoverController` + `ControlModeSwitcher`) works: faulted drone reaches its retreat position within 5 s.
- **Formation hold**: surviving 3 drones hold altitude within ±2 cm of target with redistributed tensions after the fault.
- **Adaptive load estimation** runs; convergence analysis is in progress.
- **ESKF sensor stack** runs but is not yet closed into the controller (plant truth is used). ESKF-to-controller closure is identified as the next integration step.

The key validated run is `output/gpac/n4_cable_cut_t10_v21`. See [../README.md](../README.md) for the verified result table.