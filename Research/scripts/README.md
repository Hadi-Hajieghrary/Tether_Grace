# Campaign Runners

Shell scripts that wrap the
[`decentralized_fault_aware_sim`](../cpp/README.md) executable in the
canonical flag combinations used in the research programme. Each
script writes into its own output directory under `../../output/`,
leaves the raw CSVs and Meshcat replays behind, and (where applicable)
invokes the Python analysis pipeline at
[`../analysis/`](../analysis/) to emit summary figures.

## Map

| Script | Purpose | Wall-time (8-core) | Output |
|--------|---------|--------------------|--------|
| [`run_capability_demo.sh`](run_capability_demo.sh) | Six-variant capability demonstration on a 10 kg lemniscate-3D reference with 4 m/s Dryden wind. Cleanly separates nominal (V1), wind (V2), single-fault (V3), dual-fault at 5 s spacing (V4) and 10 s spacing (V5), and full-stack (V6) responses. Headline table `summary_metrics.csv`. | ~2 h | `output/capability_demo/` |
| [`run_p2a_tension_ff_ablation.sh`](run_p2a_tension_ff_ablation.sh) | Phase-2 ablation of the local feed-forward identity `T_ff = T_i` via `--disable-tension-ff`. Runs V3/V4/V5 with feed-forward disabled; analysed against the archived capability-demo runs. Positive result: +34–39% RMS error without FF, 2.6–3.1× altitude sag. | ~1 h (3 runs) | `output/p2a_tension_ff_ablation/` |
| [`run_p2b_mass_mismatch.sh`](run_p2b_mass_mismatch.sh) | Payload-mass mismatch sweep for the L1 layer. 4 masses × 3 modes (FF-on, FF-off, FF-off+L1). Positive result: L1 recovers ~40% of altitude sag when FF is disabled. | ~1.5 h (12 runs) | `output/p2b_mass_mismatch/` |
| [`run_p2c_mpc_ceiling_sweep.sh`](run_p2c_mpc_ceiling_sweep.sh) | MPC tension-ceiling sweep at 60/70/80/90/100 N plus one baseline reference. Null result in current implementation: MPC peak tension equals baseline peak at every ceiling. | ~1.5 h (6 runs) | `output/p2c_mpc_ceiling_sweep/` |
| [`run_p2d_period_sweep.sh`](run_p2d_period_sweep.sh) | Lemniscate-period sweep for the reshape supervisor. 3 periods × {noreshape, reshape}. Null result on this schedule: reshape and no-reshape produce identical peak tensions. | ~1.5 h (6 runs) | `output/p2d_period_sweep/` |
| [`run_p2e_wind_seed_sweep.sh`](run_p2e_wind_seed_sweep.sh) | 5 winds × 3 seeds box-plot sweep. **Deferred — not executed in this revision.** Filed for a future robustness-across-seeds revision. | ~4 h (15 runs) | would populate `output/p2e_wind_seed_sweep/` |
| [`run_phase2_chain.sh`](run_phase2_chain.sh) | Sequential runner invoking P2-C → P2-D → P2-B → P2-E. Orchestrates the phase-2 stress-campaign slate from a single entry point; logs progress to `/tmp/phase2_chain_progress.log`. | sum of sub-campaigns | — |
| [`phase2_watcher.sh`](phase2_watcher.sh) | Companion to the chain runner: watches `/tmp/phase2_chain_progress.log` and, as each sub-campaign completes, runs the corresponding `plot_p2X_*.py` + `update_p2X_report.py` without waiting for the chain to finish. Intended to be launched alongside the chain. | polls; no sim cost | — |
| [`run_5drone_campaign.sh`](run_5drone_campaign.sh) | Single-configuration five-drone A/B/C/D sweep on the 3-D lemniscate reference, plus the 12-figure publication suite. Pre-P2 runner kept for reproducibility of the original case study. | ~1 h | `output/5drone_baseline_campaign/` |
| [`run_interfault_sweep.sh`](run_interfault_sweep.sh) | Deterministic sweep over inter-fault gap Δt ∈ {2, 4, 7, 10, 14, 20} s with the first fault pinned at t = 15 s. | ~25 min | `output/5drone_baseline_campaign/inter_fault_sweep/` |
| [`run_ablation_campaign.sh`](run_ablation_campaign.sh) | Quick-turn ablation: 4 controller configurations × 4 fault scenarios (16 runs). Pre-P2 runner; superseded by the narrower P2-A/B/C/D campaigns but retained for cross-config sanity checks. | ~4–8 h | `output/ablation/` |
| [`run_transactions_campaign.sh`](run_transactions_campaign.sh) | The 80-run parametric sweep pre-registered for the IEEE T-CST submission. Awaits `--actuator-max` flag in the harness. | ~2.5 h | `output/transactions_campaign/` |

All scripts depend on exactly one artefact — the built simulator
binary — and one Python environment for plotting.

## Script lineage

The runners split by era:

- **Capability demo + Phase-2 stress campaigns** (the current canonical
  suite): `run_capability_demo.sh` plus the five P2-* campaigns and
  the chain + watcher. These populate the empirical evidence for
  the 104-page reproducibility report in
  [`/workspaces/Tether_Grace/report/`](../../report/).
- **Pre-P2 campaigns** (retained for continuity): `run_5drone_campaign.sh`,
  `run_interfault_sweep.sh`, `run_ablation_campaign.sh`,
  `run_transactions_campaign.sh`. These populated the earlier case
  study and the pre-registered IEEE matrix. Still runnable; outputs
  write into separate directories so they do not collide with the
  current suite.

## Prerequisites

1. **Built simulator.** Every script hard-wires
   `EXE = Research/cpp/build/decentralized_fault_aware_sim`. Build it first:
   ```bash
   cmake -S Research/cpp -B Research/cpp/build -DCMAKE_BUILD_TYPE=Release
   cmake --build Research/cpp/build --target decentralized_fault_aware_sim -j8
   ```
2. **Python 3.10+** with `numpy`, `pandas`, `matplotlib`, `scipy`, and
   `pyyaml`. The devcontainer at the repo root has these pre-installed.
3. **Write access** to `../../output/`. The scripts create subdirectories
   under it; they never write under `archive/` or into the source tree.

No environment variables are required for default behaviour; some
scripts accept `EXE` and `ROOT` overrides (documented inline).

## `run_capability_demo.sh` (canonical)

Six variants on the 10 kg lemniscate-3D reference. The canonical
entry point for the report's Chapter 11 and for the Phase-T
Domain-Audit gate evaluation.

| Variant | Fault schedule | Wind | Controller |
|---|---|---|---|
| V1 nominal no-wind | — | 0 m/s | baseline |
| V2 nominal wind | — | 4 m/s Dryden seed 42 | baseline |
| V3 single wind | fault @ 12 s | 4 m/s seed 42 | baseline |
| V4 dual 5 s wind | faults @ 12, 17 s | 4 m/s seed 42 | baseline |
| V5 dual 10 s wind | faults @ 12, 22 s | 4 m/s seed 42 | baseline |
| V6 dual 5 s full-stack | faults @ 12, 17 s | 4 m/s seed 42 | MPC + L1 + reshape |

## Phase-2 stress campaigns

Each P2-* runner targets a specific evidence-audit row in the report's
§12 Discussion. See that table for claim-to-campaign mapping. The P2-E
wind × seed sweep was pre-designed but deliberately not executed in
this revision; its row in the audit is flagged strength B rather than A.

## Pre-P2 runners (retained for reproducibility)

See the original entries below — these scripts still work and their
documentation is unchanged from the earlier revision.

### `run_5drone_campaign.sh`

Runs the headline four-scenario sweep (A/B/C/D) and the 12-figure
publication suite. Flags after the script name are forwarded to every
simulator invocation, so extensions layer on top of the scenario
matrix:

```bash
./run_5drone_campaign.sh                               # baseline only
./run_5drone_campaign.sh --l1-enabled                  # +L1
./run_5drone_campaign.sh --controller=mpc              # +MPC
./run_5drone_campaign.sh --controller=mpc --l1-enabled --reshaping-enabled
                                                        # full stack
```

### `run_interfault_sweep.sh`

Fixes the first fault on drone 0 at t = 15 s and moves the second
fault across Δt ∈ {2, 4, 7, 10, 14, 20} s. Six deterministic runs.

### `run_ablation_campaign.sh`

Four configurations × four fault scenarios (16 runs). A quick-turn
cross-config sanity tool:

```bash
./run_ablation_campaign.sh              # all four configs
./run_ablation_campaign.sh baseline     # only the baseline column
./run_ablation_campaign.sh mpc fullstack
```

### `run_transactions_campaign.sh`

The 80-run pre-registered matrix for the IEEE T-CST submission,
specified in
[`../../archive/docs_2026_04_23/preregistration.md`](../../archive/docs_2026_04_23/preregistration.md). Sections:

| Section | Runs | What it exercises |
|--------|------|-------------------|
| `ablation` | 30 | 6 configurations × 5 scenarios (primary headline table) |
| `param` | 10 | Payload-mass sweep m_L ∈ {2.0, 2.5, 3.0, 3.5, 3.9} kg, baseline vs fullstack |
| `actuator` | 4 | Actuator-ceiling sweep (awaits `--actuator-max` in harness) |
| `competitor` | 0 | Reserved; activates when the four competitor LeafSystems land |
| `long` | 6 | 300-second stress runs on A / C / E for baseline and fullstack |

```bash
./run_transactions_campaign.sh                 # every section
./run_transactions_campaign.sh ablation        # one section only
./run_transactions_campaign.sh --resume        # skip runs whose manifest.yaml exists
./run_transactions_campaign.sh --parallel 4    # four worker processes
```

## Cross-reference

- Simulator CLI and output format:
  [`../cpp/README.md`](../cpp/README.md).
- Analysis scripts consumed by these runners:
  [`../analysis/`](../analysis/).
- Canonical report built from these runs:
  [`/workspaces/Tether_Grace/report/main.pdf`](../../report/main.pdf).
- Experiment traceability:
  [`../../archive/docs_2026_04_23/preregistration.md`](../../archive/docs_2026_04_23/preregistration.md) and
  [report Appendix A4 (CLI reference)](../../report/sections/A4_cli_reference.tex).
- Phase-T theorem contracts and Domain-Audit findings:
  [`../../report/companions/theory/theorem_contracts_phase_T.md`](../../report/companions/theory/theorem_contracts_phase_T.md).
