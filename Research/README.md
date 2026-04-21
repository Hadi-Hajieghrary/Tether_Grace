# Tether_Grace — Research

Workspace for the new control-systems research built on top of the
Tether_Lift baseline. The current active contribution is a
**decentralised, fully-local, QP-based fault-aware controller** for 4-drone
cooperative payload lift, with its C++ simulation and its IEEE-paper
figure pipeline.

## Boundaries

| Folder | Role |
|--------|------|
| [`../Tether_Lift`](../Tether_Lift) | Inherited baseline repo — **read-only**, never modified. |
| `./` (this folder) | Active code + design documentation for the new controller. |
| [`../output/Tether_Grace/`](../output/Tether_Grace/) | All simulation outputs, replays, plots, campaign metrics. Nothing source-code here. |

## Active contents

| Path | What it is |
|------|------------|
| [`cpp/`](cpp/) | Drake-based C++ sim. Single active executable target `decentralized_fault_aware_sim` (see [cpp/CMakeLists.txt](cpp/CMakeLists.txt)). |
| [`cpp/src/decentralized_local_controller.cc`](cpp/src/decentralized_local_controller.cc) | The fully-local QP controller — **the paper's contribution**. |
| [`cpp/src/decentralized_fault_aware_sim_main.cc`](cpp/src/decentralized_fault_aware_sim_main.cc) | Simulation harness: plant, rope model, Meshcat, logging. |
| [`cpp/src/fault_aware_rope_visualizer.cc`](cpp/src/fault_aware_rope_visualizer.cc) | Rope polyline that cleanly hides after a cable fault. |
| [`analysis/ieee/`](analysis/ieee/) | IEEE Transactions-style figure pipeline (one PNG per plot). |
| [`run_ieee_campaign.sh`](run_ieee_campaign.sh) | One-shot runner: builds → runs 6 scenarios → generates figures + READMEs. |
| [`DECENTRALIZED_FAULT_AWARE_README.md`](DECENTRALIZED_FAULT_AWARE_README.md) | Full design document — architecture, scenarios, parameters, results. |
| [`baseline/`](baseline/) | Orientation notes on the Tether_Lift baseline. |

## Design documentation (read these)

1. [`DECENTRALIZED_FAULT_AWARE_README.md`](DECENTRALIZED_FAULT_AWARE_README.md) — principal design document for the active system.
2. [`baseline/tether_lift_baseline.md`](baseline/tether_lift_baseline.md), [`baseline/paper_vs_code_matrix.md`](baseline/paper_vs_code_matrix.md) — context on the inherited Tether_Lift codebase.

## Where history lives

During development this controller went through several design iterations
(Phase 1 single-drone MPC → Phase 2 two-drone → Phase 2.5 inter-drone
comms → box-only 4-drone → peer-aware → fully-local QP). The obsolete
source files and runners are preserved in-repo for provenance:

- **Source code of superseded designs:** [`archive_obsolete_designs/`](archive_obsolete_designs/) (each phase has its own sub-folder with a `NOTES.txt`).
- **Prior campaign outputs (~0.5 GB of replays, CSVs, logs):** [`../output/Tether_Grace/archive_prior_campaigns/`](../output/Tether_Grace/archive_prior_campaigns/).
- **Research-planning docs from earlier sessions** (PHASE_*.md, delivery manifests): [`../output/Tether_Grace/archive_phase_docs/`](../output/Tether_Grace/archive_phase_docs/).

## Reproducing the active campaign

```bash
# 1. build
cd Research/cpp
cmake -B build -DCMAKE_BUILD_TYPE=Release
make -C build decentralized_fault_aware_sim -j4

# 2. run all 6 scenarios + generate IEEE figures & per-scenario READMEs
cd ..
./run_ieee_campaign.sh

# Outputs land under ../output/Tether_Grace/ (one sub-folder per scenario + a
# cross-scenario comparison folder + an archive folder).
```

## Design intent

This folder exists to keep the control-systems story honest and executable:

- **baseline physics inherited deliberately** from Tether_Lift, never copied or forked
- **one active target** per entry point — intermediate-design code lives in `archive_obsolete_designs/`, not as disabled CMake blocks
- **simulation outputs separated from source code** — `output/Tether_Grace/` holds every artefact; nothing heavyweight lives here
- **documentation tracks the code path that actually runs** — old design docs are archived, not kept in the main README

Future additions should follow the same principle.
