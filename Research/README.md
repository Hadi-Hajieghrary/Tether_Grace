# Tether_Grace — Research

Workspace for the control-systems research built on top of the
Tether_Lift baseline. The active contribution is a
**decentralised, fully-local, QP-based fault-aware controller** for
4- and 5-drone cooperative payload lift, with three composable
extensions, its C++ simulation, a 12-figure IEEE publication pipeline,
and a printable LaTeX reference.

> **For the authoritative "what is in the codebase right now" map,
> see** [`docs/IMPLEMENTATION_STATUS.md`](docs/IMPLEMENTATION_STATUS.md).
> For the roadmap status see [`docs/EXTENSION_PLAN.md`](docs/EXTENSION_PLAN.md).

## Implemented controllers + extensions

| Layer | What | CLI flag | Theory doc |
|---|---|---|---|
| Baseline | cascade PD + single-step QP + anti-swing + smoothstep pickup + per-segment tension probe + 13-signal diagnostics | (default) | [`docs/theory/theory_decentralized_local_controller.md`](docs/theory/theory_decentralized_local_controller.md) |
| Phase-F | L1 adaptive outer loop (in-controller) | `--l1-enabled` | [`docs/theory/theory_l1_extension.md`](docs/theory/theory_l1_extension.md) |
| Phase-F (CL) | Two-channel concurrent-learning observer (diagnostics) | `--adaptive` | [`docs/theory/theory_l1_extension.md`](docs/theory/theory_l1_extension.md) §6 |
| Phase-G | Receding-horizon MPC with hard tension-ceiling | `--controller=mpc` | [`docs/theory/theory_mpc_extension.md`](docs/theory/theory_mpc_extension.md) |
| Phase-H | FaultDetector + FormationCoordinator supervisor | `--reshaping-enabled` | [`docs/theory/theory_reshaping_extension.md`](docs/theory/theory_reshaping_extension.md) |

All extensions compose orthogonally — running
`--controller=mpc --l1-enabled --reshaping-enabled` produces the full
stack.

## Boundaries

| Folder | Role |
|--------|------|
| [`../Tether_Lift`](../Tether_Lift) | Inherited baseline repo — **read-only**, never modified. |
| `./` (this folder) | Active code + design documentation for the new controller. |
| [`../output/Tether_Grace/`](../output/Tether_Grace/) | All simulation outputs, replays, plots, campaign metrics. Nothing source-code here. |

## Active contents

| Path | What it is |
|------|------------|
| [`cpp/`](cpp/) | Drake C++ sim; one executable target `decentralized_fault_aware_sim`. |
| [`cpp/src/`](cpp/src/) + [`cpp/include/`](cpp/include/) | 5 `.cc` + 12 `.h` files — see [`docs/IMPLEMENTATION_STATUS.md`](docs/IMPLEMENTATION_STATUS.md) for the file-by-file map. |
| [`analysis/ieee/`](analysis/ieee/) | IEEE-Transactions figure pipeline + `plot_publication.py` (12-figure suite) + `PUBLICATION_FIGURES.md` + `fill_results_placeholders.py`. |
| [`run_ieee_campaign.sh`](run_ieee_campaign.sh), [`run_5drone_campaign.sh`](run_5drone_campaign.sh), [`run_mc_interfault_sweep.sh`](run_mc_interfault_sweep.sh) | End-to-end campaign runners. |
| [`docs/IMPLEMENTATION_STATUS.md`](docs/IMPLEMENTATION_STATUS.md) | **Single source of truth** for which file does what. |
| [`docs/EXTENSION_PLAN.md`](docs/EXTENSION_PLAN.md) | Cross-extension roadmap + status table. |
| [`docs/INDEX.md`](docs/INDEX.md) | Navigation hub for docs / output / theory. |
| [`docs/theory/`](docs/theory/) | 7 theory markdown files (baseline + L1 + MPC + reshaping + rope dynamics + fault model + figure-8). |
| [`docs/case_study_tether_grace.md`](docs/case_study_tether_grace.md) | Graduate-level self-contained case study. |
| [`docs/latex/`](docs/latex/) | Printable PDF reference + `methods_section.tex` + `results_section.tex`. |
| [`DECENTRALIZED_FAULT_AWARE_README.md`](DECENTRALIZED_FAULT_AWARE_README.md) | Historical design narrative — the "why" behind every choice. |
| [`baseline/`](baseline/) | Orientation notes on the Tether_Lift baseline. |

## Design documentation (read in order)

1. [`docs/IMPLEMENTATION_STATUS.md`](docs/IMPLEMENTATION_STATUS.md) — code map (fastest onboarding).
2. [`DECENTRALIZED_FAULT_AWARE_README.md`](DECENTRALIZED_FAULT_AWARE_README.md) — architectural narrative.
2. [`baseline/tether_lift_baseline.md`](baseline/tether_lift_baseline.md), [`baseline/paper_vs_code_matrix.md`](baseline/paper_vs_code_matrix.md) — context on the inherited Tether_Lift codebase.

## Where history lives

During development this controller went through several design iterations
(Phase 1 single-drone MPC → Phase 2 two-drone → Phase 2.5 inter-drone
comms → box-only 4-drone → peer-aware → fully-local QP → Phase-F L1 →
Phase-G MPC → Phase-H reshape). Superseded material is preserved for
provenance, never deleted:

- **Source code of prior-phase designs:** [`archive_obsolete_designs/`](archive_obsolete_designs/)
  (Phase 1/2/2.5/4/peer-aware; each sub-folder has a `NOTES.txt`).
- **Prior campaign outputs (~0.5 GB of replays, CSVs, logs):**
  [`../output/Tether_Grace/archive_prior_campaigns/`](../output/Tether_Grace/archive_prior_campaigns/).
- **Early research-planning docs:**
  [`../output/Tether_Grace/archive_phase_docs/`](../output/Tether_Grace/archive_phase_docs/).
- **Session-level 2026-04 planning/progress notes + superseded LaTeX drafts:**
  [`archive_session_notes_2026-04/`](archive_session_notes_2026-04/)
  (one `NOTES.txt` file explaining each archived doc).

All active code and docs are in the top-level `Research/` tree, with
[`docs/IMPLEMENTATION_STATUS.md`](docs/IMPLEMENTATION_STATUS.md) as
the canonical entry point.

## Reproducing the campaigns

```bash
# 1. build
cd Research/cpp
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target decentralized_fault_aware_sim -j4

# 2. pick a campaign
cd ..
./run_ieee_campaign.sh           # 4-drone S1-S6  → output/Tether_Grace/
./run_5drone_campaign.sh         # 5-drone A-D   → output/Tether_Grace_5drone/
./run_mc_interfault_sweep.sh     # MC Δt-sweep   → .../monte_carlo/

# 3. run ablations (composable)
./run_5drone_campaign.sh --l1-enabled               # +L1 adaptive
./run_5drone_campaign.sh --controller=mpc           # +MPC
./run_5drone_campaign.sh --reshaping-enabled        # +formation reshape
./run_5drone_campaign.sh --controller=mpc --l1-enabled --reshaping-enabled
```

Each runner ends with the full figure pipeline (12 publication figures
+ 2 supplementary, plus per-scenario 10-figure sets). See
[`docs/IMPLEMENTATION_STATUS.md`](docs/IMPLEMENTATION_STATUS.md) for
the CLI-flag reference.

## Design intent

This folder exists to keep the control-systems story honest and executable:

- **baseline physics inherited deliberately** from Tether_Lift, never copied or forked
- **one active target** per entry point — intermediate-design code lives in `archive_obsolete_designs/`, not as disabled CMake blocks
- **simulation outputs separated from source code** — `output/Tether_Grace/` holds every artefact; nothing heavyweight lives here
- **documentation tracks the code path that actually runs** — old design docs are archived, not kept in the main README

Future additions should follow the same principle.
