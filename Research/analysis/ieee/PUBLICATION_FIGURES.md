# Publication-grade figure suite — reviewer-motivated justification

This document lists every figure produced by
[`plot_publication.py`](plot_publication.py), cross-references it to
the specialist-reviewer persona that requested it, and states in one
sentence the scientific claim the figure is expected to support for a
**highly skeptical** IEEE Transactions reviewer.

The seven personas consulted:

* **[A] codebase-scrutinizer** — implementation integrity, runtime cost,
  numerical conditioning
* **[B] Codebase Theorist / Technical Auditor** — mathematical
  soundness, closed-loop stability
* **[C] design-explainer** — conceptual clarity, block-diagram
  self-containedness
* **[D] drake-expert** — simulation fidelity, integrator and
  contact-solver behaviour
* **[E] IEEE Technical Report Writer** — readability, single-page
  absorption, ablation evidence
* **[F] Scientific Visualization Architect** — information density,
  frequency-domain diagnostics
* **[G] Robotics Adversarial Auditor** — worst-case envelopes,
  Monte-Carlo evidence

## Figure manifest

| ID  | File (PDF + PNG) | Reviewers | Scientific claim |
|-----|------------------|-----------|------------------|
| F01 | `F01_architecture.*`                  | C, E       | The controller is fully local; no peer-tension arrow crosses drone boundaries. |
| F02 | `F02_3d_<scenario>.*` (per scenario)  | C, E, G    | Payload tracks the 3-D reference within the ±ε tube, and the faulted drone departs the formation. |
| F03 | `F03_tracking_error_overlay.*`        | B, E       | $\|e_p\|$ is bounded across all scenarios; fault injection adds only a transient, not a steady-state bias. |
| F04 | `F04_waterfall_<scenario>.*`          | D, G       | Segment tensions propagate physically up the rope; post-fault the severed rope's segments go to zero and the surviving ropes absorb the load. |
| F05 | `F05_sigmaT_overlay.*`                | B, E, G    | Load-sharing imbalance $\sigma_T(t)$ rises monotonically with the number of compound faults, as predicted by the emergent-rebalancing theory. |
| F06 | `F06_spectrogram_<scenario>.*`        | F          | Fault events appear as broadband vertical stripes in the STFT of $\sigma_T(t)$ — uniquely detectable from one scalar signal. |
| F07 | `F07_activeset_<scenario>.*`          | A, E       | The QP is *unconstrained* during nominal cruise; saturation is transient and bounded to the pickup/landing windows. |
| F08 | `F08_thrust_tilt_<scenario>.*`        | A, D       | Thrust envelope $\|F\|\le F_{\max}$ and tilt envelope $\theta\le\theta_{\max}$ are respected in closed-loop. |
| F09 | `F09_pole_locus.*`                    | B          | The closed-loop payload-vertical subsystem is asymptotically stable for every $N_{\text{alive}}\in\{1,\dots,5\}$. |
| F10 | `F10_mc_scatter.*`                    | G          | Peak rope tension under compound failures scales smoothly and boundedly with the inter-fault gap $\Delta t$ — no singular configurations. |
| F11 | `F11_metrics_heatmap.*`               | E          | The controller is fault-tolerant across the six headline metrics (RMS err, peak err, peak $T$, peak $\|F\|$, RMS $\sigma_T$). |
| F12 | `F12_grouped_bars.*`                  | E, G       | Bootstrap confidence intervals for RMS $\|e_p\|$ — nominal vs. faulted scenarios are statistically indistinguishable. |
| S01 | `S01_pickup_stretch_<scenario>.*`     | D          | The bead-chain starts taut at hover equilibrium; no snap-taut impulse at pickup. |
| S02 | `S02_settling_ecdf.*`                 | F, G       | Empirical CDF of post-fault settling time; the 95th percentile is bounded. |

## Reading order for a 2-minute skim

Open the figures in this order — the reader will form the complete
argument in two minutes, without needing to read a line of text:

1. **F01** — architecture: locality is visual.
2. **F02** (scenario C) — 3-D trajectory with the severed rope visually
   disconnecting during flight.
3. **F03** — tracking error: not affected by fault.
4. **F05** — $\sigma_T(t)$: peer-free rebalancing.
5. **F04** (scenario C) — waterfall showing rope segment-by-segment
   relaxation after fault.
6. **F11** — metric heatmap: numbers tell the story.
7. **F10** — Monte-Carlo worst-case envelope.

## Reproducibility

All figures are generated deterministically from the per-scenario CSVs:

```bash
python3 plot_publication.py <campaign_root> <output_dir>
```

The per-scenario CSVs (`scenario_*.csv` under `08_source_data/`) contain
every signal needed: plant state (drones + payload), reference
trajectory, per-rope scalar tensions, per-rope per-segment tensions
(observer), per-drone control vector, and 13-signal per-drone
diagnostics (including QP solve time and active-set flags).

## Publication targets

The figures are sized for the IEEE Transactions two-column layout:

* single column: 3.5 in × 2.3 in
* double column: 7.16 in × 2.8 in (or 4.2 in tall)

rendered at 300 dpi in both PNG and PDF. Fonts: serif (Computer
Modern / STIX). Colour palette: Wong (colour-blind safe), extended
to six drones.
