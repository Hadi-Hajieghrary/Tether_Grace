# Tether_Lift Baseline Knowledge Base

Orientation material for the upstream Tether_Lift baseline, which is
vendored here as a git submodule at [`../../Tether_Lift`](../../Tether_Lift).
The active controller in this workspace replaces the baseline's
control stack; Tether_Lift is read-only and its bead-chain rope model,
URDF assets, and visualiser are reused verbatim.

## Documents

- [`tether_lift_baseline.md`](tether_lift_baseline.md) — baseline inventory, runtime architecture, module maturity, reproducibility, and extension constraints.
- [`paper_vs_code_matrix.md`](paper_vs_code_matrix.md) — claim-by-claim matrix separating live implementation facts from manuscript or README narrative.
- [`research_agenda.md`](research_agenda.md) — long-term extension agenda.

## Baseline in one sentence

A Drake-based multi-quadrotor cooperative-lift simulator with
bead-chain ropes, a tension-aware cascade PD controller, estimator
logging, wind disturbance, Meshcat replay, and CSV export. A richer
GPAC / ESKF / concurrent-learning / CBF stack exists in the upstream
repository but is not wired into its default runtime path.

## What this workspace inherits vs. replaces

- **Inherited verbatim:** the bead-chain Kelvin–Voigt rope, the URDF
  quadrotor, the ground-plane and Meshcat wiring, the tension
  measurement port, the file-layout conventions.
- **Replaced:** the per-drone control stack. `DecentralizedLocalController`
  (and its MPC variant and L1 / reshape extensions) supersedes the
  centralised PD/GPAC controllers in Tether_Lift; see
  [`../docs/IMPLEMENTATION_STATUS.md`](../docs/IMPLEMENTATION_STATUS.md).

Earlier experiments that mixed in the upstream GPAC controller are
preserved under [`../../archive/obsolete_designs/`](../../archive/obsolete_designs/)
for historical reference; they are not part of the active build.
