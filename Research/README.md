# Research

Active source tree for the Tether_Grace controller stack and its
companion analysis pipeline. The canonical algorithmic and
mathematical specification is the 104-page reproducibility report at
[`/workspaces/Tether_Grace/report/`](../report/); this README is the
map of the source tree that produces it.

## Layout

```
Research/
├── cpp/                             ── C++ simulator + controller stack
│   ├── CMakeLists.txt
│   ├── README.md                    ── cpp-subtree reference
│   ├── include/                     ── public headers
│   └── src/                         ── implementations + simulator main
├── analysis/                        ── Python analysis + plotting pipeline
│   ├── README.md                    ── analysis-script map
│   ├── plot_capability_demo.py      ── Chapter 11 figures
│   ├── plot_review_augmentation.py  ── fault-zoom / load-share / margins
│   ├── plot_p2{a,b,c,d,e}_*.py      ── Phase-2 stress-campaign plotters
│   ├── update_p2{a,b,c,d,e}_*.py    ── LaTeX subfile writers for the report
│   ├── phase_t_domain_audit_v11.py  ── Phase-T v1.1 gate criterion (canonical)
│   └── phase_t_domain_audit.py      ── Phase-T v1.0 gate (historical; frozen)
├── scripts/                         ── campaign runners
│   └── README.md
└── tests/                           ── Python verification tests
    └── README.md
```

Previously a `docs/` subtree sat alongside these; as of the April-2026
consolidation, it has been archived at
[`/workspaces/Tether_Grace/archive/docs_2026_04_23/`](../archive/docs_2026_04_23/).
The two documents the report still actively cites
(`theorem_contracts_phase_T.md`, `theory_baseline_stability.md`)
moved into
[`/workspaces/Tether_Grace/report/companions/theory/`](../report/companions/theory/)
alongside the LaTeX sources.

## The controller stack

| Layer | Source | CLI flag | Report chapter |
|-------|--------|----------|----------------|
| Baseline QP | [`cpp/src/decentralized_local_controller.cc`](cpp/src/decentralized_local_controller.cc) | (default) | [Chapter 5](../report/sections/05_baseline_controller.tex) |
| L1 adaptive outer loop | same file | `--l1-enabled` | [Chapter 6](../report/sections/06_l1_adaptive.tex) |
| CL observer | [`cpp/src/cl_param_estimator.cc`](cpp/src/cl_param_estimator.cc) | `--adaptive` | Report Ch 6 §6 |
| Receding-horizon MPC | [`cpp/src/mpc_local_controller.cc`](cpp/src/mpc_local_controller.cc) | `--controller=mpc` | [Chapter 7](../report/sections/07_mpc_extension.tex) |
| Formation reshape | [`cpp/include/fault_detector.h`](cpp/include/fault_detector.h) + [`cpp/include/formation_coordinator.h`](cpp/include/formation_coordinator.h) | `--reshaping-enabled` | [Chapter 8](../report/sections/08_reshape_extension.tex) |

The layers compose independently; every combination is a valid
controller configuration and is exercised by the campaign matrix.

## Building

```bash
cmake -S cpp -B cpp/build -DCMAKE_BUILD_TYPE=Release
cmake --build cpp/build --target decentralized_fault_aware_sim -j8
```

The build assumes Drake is on the `CMAKE_PREFIX_PATH` (the devcontainer
sets this automatically). A single executable target is produced:
`decentralized_fault_aware_sim`.

## Running a scenario

```bash
./cpp/build/decentralized_fault_aware_sim \
    --num-quads 5 \
    --trajectory lemniscate3d \
    --duration 40 \
    --scenario A_nominal \
    --output-dir /tmp/run
```

The authoritative CLI-flag reference is
[report Appendix A4](../report/sections/A4_cli_reference.tex).
Canonical flag combinations used in the published campaigns are
composed by the shell wrappers under
[`scripts/`](scripts/README.md).

## Theoretical guarantees (Phase-T)

The baseline's system-level behaviour is backed by two explicit
theorems on a slack-excursion-bounded domain
$\Omega_\tau^\text{dwell}$:

- **Reduction Theorem** (report Chapter 2A) — justifies
  $k_\text{eff} = k_s/N_s$ with an $O(\delta + \eta)$ error budget.
- **Baseline Practical-ISS Theorem** (report Chapter 5B) — explicit
  bounded-recovery envelope for the decentralised closed loop
  under finite spaced fault events.

The theorem contracts and Domain-Audit findings live at
[`../report/companions/theory/theorem_contracts_phase_T.md`](../report/companions/theory/theorem_contracts_phase_T.md);
the full proof document is
[`../report/companions/theory/theory_baseline_stability.md`](../report/companions/theory/theory_baseline_stability.md).

## Where the numbers in the paper come from

Numerical claims trace to two artefact trees:

1. **The canonical reproducibility report** at
   [`../report/`](../report/) — consumes `output/capability_demo/` and
   `output/p2{a,b,c,d}/` via the analysis pipeline under
   [`analysis/`](analysis/README.md).
2. **The archived IEEE-T-CST journal draft** at
   [`../archive/docs_2026_04_23/latex/`](../archive/docs_2026_04_23/latex/)
   — mapped to the pre-registration at
   [`../archive/docs_2026_04_23/preregistration.md`](../archive/docs_2026_04_23/preregistration.md)
   and fed by `run_transactions_campaign.sh`. The journal draft is
   preserved for future revival; it is not the active deliverable.
