# Tether_Grace

Decentralised, fault-tolerant cooperative payload lift with cable-suspended
quadrotors. The repository contains a Drake-based C++ simulator, the
controller stack (baseline QP, L1 adaptive outer loop, receding-horizon
MPC with tension-ceiling, and formation-reshape supervisor), an analysis
pipeline in Python, a tracked LaTeX manuscript and supplementary PDF, and
a campaign harness that produces the published numbers end-to-end.

## Top-level layout

| Path | Role |
|------|------|
| [`Research/`](Research) | All active source, documentation, analysis scripts, and manuscripts. |
| [`Tether_Lift/`](Tether_Lift) | Upstream baseline, vendored as a read-only git submodule. Not modified here. |
| [`output/`](output) | Simulation artefacts produced by the campaign runner. READMEs are tracked; raw CSVs, Meshcat replays, and per-tick logs are gitignored. |
| [`archive/`](archive) | Provenance of superseded code, documentation, and campaigns. See [`archive/README.md`](archive/README.md). |

Entry points for a new reader:

1. [`report/main.pdf`](report/main.pdf) — reproducibility report, 104 pages, with the theorem-backed revision. The canonical mathematical and algorithmic specification.
2. [`Research/README.md`](Research/README.md) — orientation for the active source tree; each subtree has its own README (cpp/, analysis/, scripts/, tests/).
3. [`report/companions/`](report/companions/) — two markdown documents the report actively cites: the Phase-T theorem contract plus Domain-Audit findings, and the full lemma-by-lemma proof document.

## Theoretical guarantees (report Chapters 2A and 5B)

The revision upgrades the previous quasi-static justification of the
baseline's emergent-fault-tolerance mechanism with two explicit theorems
on a disciplined, slack-excursion-bounded domain:

- **Reduction Theorem** ([report Ch 2A](report/sections/02a_reduction.tex)) — on
  the forward-invariant domain `Ω_τ^dwell` (slack events ≤ 40 ms,
  duty cycle ≤ 2.5%), the bead-chain plant is uniformly *O*(δ + η)-close
  to a reduced-order taut-cable model parameterised by
  *k*<sub>eff</sub> = *k*<sub>s</sub> / *N*<sub>s</sub>, with δ the
  dimensionless time-scale separation
  ω<sub>pend</sub> / ω<sub>rope</sub>. Justifies the effective-stiffness
  identity used throughout the rest of the report.
- **Baseline Practical-ISS Theorem** ([report Ch 5B](report/sections/05b_baseline_stability.tex)) —
  on `Ω_τ^dwell ∩ Ω_QP,active`, the decentralised baseline closed loop
  satisfies an explicit hybrid practical-ISS envelope under bounded
  disturbance, bounded parameter mismatch, and a finite number of
  cable-severance events separated by a dwell time τ<sub>d</sub> ≥
  τ<sub>pend</sub>. Replaces the previous edition's quasi-static
  Proposition 5.2 with a formal bounded-recovery statement.

**Scope limits** (stated up front in the introduction and repeated in
the conclusion): single-active-regime QP doctrine (D1); rapid
multi-fault scenarios out of scope; slack events longer than 40 ms out
of scope. Full proofs are in
[`report/companions/theory/theory_baseline_stability.md`](report/companions/theory/theory_baseline_stability.md);
the contract and domain-audit artefacts are in
[`report/companions/theory/theorem_contracts_phase_T.md`](report/companions/theory/theorem_contracts_phase_T.md).

## Reproducing the published numbers

```bash
# Build
cmake -S Research/cpp -B Research/cpp/build -DCMAKE_BUILD_TYPE=Release
cmake --build Research/cpp/build --target decentralized_fault_aware_sim -j8

# Run a single scenario (5-drone lemniscate, 40 s, dual 10-s fault)
./Research/cpp/build/decentralized_fault_aware_sim \
    --num-quads 5 --trajectory lemniscate3d --duration 40 \
    --scenario D_dual_10sec \
    --fault-0-quad 0 --fault-0-time 15 \
    --fault-1-quad 2 --fault-1-time 25 \
    --output-dir output/5drone_baseline_campaign/08_source_data

# Or run the full A/B/C/D sweep and publication-figure build
./Research/scripts/run_5drone_campaign.sh

# Or run the six-variant capability demonstration (heavy payload +
# wind + compound faults, ≈ 2 h wall-time).
./Research/scripts/run_capability_demo.sh
python3 Research/analysis/plot_capability_demo.py
```

The pre-registered IEEE-T-CST parametric-sweep campaign (80 runs) is
executed by
[`Research/scripts/run_transactions_campaign.sh`](Research/scripts/run_transactions_campaign.sh);
expect roughly 2.5 wall-hours on an 8-core host. The frozen
experiment specification and the journal-manuscript LaTeX live under
[`archive/docs_2026_04_23/`](archive/docs_2026_04_23/) (the IEEE
journal product is a parallel deliverable to the reproducibility
report and was archived in the April-2026 consolidation).

## Dependencies

- **Drake** ≥ 1.32 (built against upstream master via the devcontainer).
- **Eigen** 3.4.
- **OSQP / osqp-eigen** (supplied by the Drake vendor bundle).
- **Python 3.10** with `numpy`, `pandas`, `matplotlib`, `scipy`, `pyyaml`.

A reproducible devcontainer lives in [`.devcontainer/`](.devcontainer); it
is a symlink into the Tether_Lift submodule.

## Citing

If you use this code, please cite the reproducibility report at
[`report/main.pdf`](report/main.pdf). The archived IEEE-T-CST journal
draft is at
[`archive/docs_2026_04_23/latex/`](archive/docs_2026_04_23/latex/) and
may be revived for submission in a future pass.

## License

Apache 2.0. See [`LICENSE`](LICENSE).
