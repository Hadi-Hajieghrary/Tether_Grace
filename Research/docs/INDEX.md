# Documentation Index

Grouped by why you would open each document.

## Start here

- [`IMPLEMENTATION_STATUS.md`](IMPLEMENTATION_STATUS.md) — code map. Every file in `cpp/`, `analysis/`, `scripts/` is listed with its purpose, CLI flag, theory reference, and where its output flows.
- [`../README.md`](../README.md) — orientation for the `Research/` tree (directory layout, build/run, paper pipeline).

## Theory

| Document | Derivation |
|----------|------------|
| [`theory/theory_decentralized_local_controller.md`](theory/theory_decentralized_local_controller.md) | Cascade PD + single-step QP, anti-swing, attitude inner loop. |
| [`theory/theory_rope_dynamics.md`](theory/theory_rope_dynamics.md) | 8-bead Kelvin–Voigt rope; effective axial stiffness; catenary drape. |
| [`theory/theory_fault_model.md`](theory/theory_fault_model.md) | Four-gate fault model (physical, telemetry, visual, supervisory). |
| [`theory/theory_figure8_trajectory.md`](theory/theory_figure8_trajectory.md) | Bernoulli lemniscate parameterisation and bandwidth analysis. |
| [`theory/theory_l1_extension.md`](theory/theory_l1_extension.md) | L1 adaptive outer loop: predictor, projection, LP filter, stability bounds. |
| [`theory/theory_mpc_extension.md`](theory/theory_mpc_extension.md) | Receding-horizon MPC; condensed prediction; linearised tension rows; DARE terminal cost. |
| [`theory/theory_reshaping_extension.md`](theory/theory_reshaping_extension.md) | Closed-form reshape for N = 4 → M = 3 with quintic smoothstep and collision certificate. |

## Design and engineering

- [`design_narrative.md`](design_narrative.md) — high-level architectural narrative.
- [`case_study_tether_grace.md`](case_study_tether_grace.md) — self-contained graduate case study (problem framing → model → controller → fault tolerance → simulation → campaign).

## Experiment pre-registration

- [`preregistration.md`](preregistration.md) — frozen hypotheses, experiment matrix, acceptance criteria. Committed before the Monte-Carlo campaign runs and not edited thereafter.

## Manuscript

- [`latex/tether_grace_reference.tex`](latex/tether_grace_reference.tex) — master LaTeX source (manuscript + appendices).
- [`latex/methods_section.tex`](latex/methods_section.tex) · [`latex/results_section.tex`](latex/results_section.tex) — chapter files included by the master.
- [`latex/supplementary.tex`](latex/supplementary.tex) — formal proofs (Theorems 1–6, Propositions 1–9) and extended figures.
- [`latex/tether_grace_reference.pdf`](latex/tether_grace_reference.pdf) — latest build.

## Simulation results

- [`../../output/5drone_baseline_campaign/README.md`](../../output/5drone_baseline_campaign/README.md) — five-drone A/B/C/D baseline campaign.
- `../../output/transactions_campaign/` *(created by* [`../scripts/run_transactions_campaign.sh`](../scripts/run_transactions_campaign.sh)*)* — full IEEE-Transactions Monte-Carlo matrix.

## Archives

All superseded material lives under [`../../archive/`](../../archive) — see its `README.md` for the layout.
