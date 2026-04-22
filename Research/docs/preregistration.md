# Experiment Pre-Registration

This document is committed before the IEEE-Transactions Monte-Carlo
campaign runs and is treated as read-only thereafter. Every numerical
claim in the manuscript is traceable to exactly one `experiment_id`
listed below and to exactly one row of
`output/transactions_campaign/publication_metrics.csv`. If a claim
cannot be traced back to this file, it is withdrawn.

## Hypotheses

| ID | Hypothesis | Rejected if |
|----|------------|-------------|
| H1 | The L1 adaptive outer loop reduces payload-tracking RMS error by ≥ 10 % relative to the baseline under matched-mass bias, across all fault topologies. | Mean reduction < 10 % (BH-corrected `p` ≥ 0.05) on Scenarios A–E. |
| H2 | The MPC with `T_max = 100 N` produces zero *unslacked* tension-ceiling violations across the primary ablation matrix. | Any unslacked violation is observed in the 150 ablation runs. |
| H3 | The full stack (L1 + MPC + reshape) strictly Pareto-dominates the baseline on (tracking RMS, peak tension, `σ_T`) in ≥ 80 % of scenarios. | Dominance holds in fewer than 4 of 5 scenarios. |
| H4 | The MPC p99 solve time stays below 500 µs for `N_p = 5` and below 1 ms for `N_p = 10` on the x86_64 reference platform. | Measured p99 exceeds the budget on any configuration. |
| H5 | The formation-reshape reduces post-fault peak tension by ≥ 20 % relative to the same controller without reshape on scenarios satisfying the geometric sufficient condition stated in the reshape theory doc §F. | Reduction < 20 % on the eligible subset. |
| H6 | The closed-loop L1-augmented system satisfies the ISS bound derived in Supplementary Theorem 1 for every adversarial disturbance in the bounded-energy class. | Any disturbance realisation drives RMS error above the analytical bound R*. |

## Experiment matrix

### Primary ablation (P-#) — 150 runs

Six configurations × five scenarios × five seeds. Fixed nominal
payload mass (3.0 kg), nominal rope stiffness (25 000 N/m), no wind,
nominal sensor noise, strong-envelope actuators (20 N/quad).

| Axis | Levels |
|------|--------|
| Configuration | `baseline`, `+L1`, `+MPC(N_p=5)`, `+MPC(N_p=10)`, `fullstack`, `fullstack(T_max=60)` |
| Scenario | `A_nominal`, `B_single_15s`, `C_dual_5s`, `D_dual_10s`, `E_cascade_3fault` |
| Seed | `{1, 2, 3, 4, 5}` |

### Parametric robustness sweeps (R-#)

One-axis-at-a-time; `fullstack` vs. `baseline` only; 5 seeds per cell.

| Sweep | Levels | Runs |
|-------|--------|------|
| Payload mass `m_L` | `{2.0, 2.5, 3.0, 3.5, 3.9}` kg | 50 |
| Rope stiffness `k_s` | `{15, 20, 25, 30, 40}` × 10³ N/m | 50 |
| Wind | `{none, 3 m/s Dryden, 6 m/s Dryden}` | 30 |
| IMU noise | `{nominal, ×4}` | 20 |
| Actuator ceiling | `{20, 12}` N/quad | 20 |

### Adversarial worst-case (A-#) — 20 runs

Bounded-energy disturbance class; 10 realisations × `fullstack` +
`baseline`. Disturbance precomputed offline via the H∞ surrogate
described in `Research/docs/theory/adversarial_disturbance.md`.

### Competitor head-to-head (C-#) — 100 runs

5 scenarios × 4 competitors × 5 seeds. Competitors:

| ID | Reference | Implementation |
|----|-----------|----------------|
| C-1 | Centralised 3N-variable QP (genie-knowledge upper bound). | `CentralizedController` |
| C-2 | Consensus-ADMM distributed MPC — Wang & Ong, T-CST 2017. | `ConsensusADMMController` |
| C-3 | GPAC baseline (the Tether_Lift predecessor stack). | `GPACBaselineController` |
| C-4 | Distributed LQR — Borrelli–Keviczky–Balas, T-CST 2009. | `DistributedLQRController` |

### Communication-robustness (K-#) — 36 runs

Applies only to the reshape broadcast path.

| Axis | Levels |
|------|--------|
| End-to-end delay | `{0, 100, 500}` ms |
| Packet-drop rate | `{0, 0.1, 0.3}` |
| Seed | `{1, 2, 3}` |

### Ultra-long horizon stress (L-#) — 18 runs

3 scenarios × {`baseline`, `fullstack`} × 3 seeds × 300 s duration.

**Grand total: 494 runs.** At 15 min/run parallelised over 8 cores,
wall time ≈ 15.5 h.

## Acceptance criteria

| AC | Criterion |
|----|-----------|
| AC1 | The `baseline` configuration reproduces the 5-drone-baseline campaign within 1 % on RMS tracking, peak tension, and `σ_T`. |
| AC2 | H1, H3 accepted at BH-corrected p < 0.05 and Cohen's d > 0.5. |
| AC3 | H2 — zero unslacked ceiling violations across all 150 ablation runs. |
| AC4 | H4 — solve-time p99 budget met on the reference platform. |
| AC5 | H5 — ≥ 20 % peak-tension reduction on the eligible subset. |
| AC6 | H6 — empirical RMS stays within the analytical ISS bound on all 20 adversarial runs. |
| AC7 | Every published number resolves to a row of `publication_metrics.csv` with matching `experiment_id`. |

A failed acceptance criterion is reported as-is; it is not silently
dropped or re-parameterised.

## Statistical protocol

- Bootstrap 95 % confidence intervals, 10 000 resamples.
- Paired Wilcoxon signed-rank tests, seed-paired across configurations.
- Multiple-comparison correction: Benjamini–Hochberg, FDR = 0.05.
- Effect sizes: Cohen's d reported alongside p-values.

## Reproducibility package

- Git SHA, binary SHA256, CMake cache hash, CPU model, and memory size
  are written into `manifest.yaml` in every run's output folder by
  `Research/cpp/include/run_manifest.h`.
- The full Monte-Carlo campaign and its analysis scripts run inside a
  pinned devcontainer image; the image digest is recorded in the
  campaign-level `manifest.yaml`.
- The post-campaign tarball (≈ 2 GB) is archived to Zenodo with a DOI
  cross-referenced from the manuscript.

## Change log

This file must not be edited while simulations are in flight. Changes
permitted before campaign launch are recorded here. After launch, any
amendment requires a new pre-registration file with a new id.
