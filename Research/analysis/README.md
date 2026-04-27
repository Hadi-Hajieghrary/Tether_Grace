# Analysis Pipeline

Python scripts that consume the CSV artefacts produced by the
[campaign runners](../scripts/README.md) and emit summary tables,
publication figures, and LaTeX subfiles that the report ingests via
`\input`.

## Scripts by role

### Styling and shared helpers

| File | Role |
|---|---|
| [`ieee_style.py`](ieee_style.py) | Matplotlib rcParams, colour palette (Wong colour-blind safe), fault annotation helper, trim-startup-transient helper, canonical fault detector used by every downstream script. |
| [`statistics.py`](statistics.py) | Common aggregates (RMS, IAE, percentile helpers) used by the per-campaign analyses. |
| [`build_tables.py`](build_tables.py) | Assembles LaTeX tables from per-run `publication_metrics.csv` artefacts. |
| [`write_readmes.py`](write_readmes.py) | Emits per-scenario READMEs into the output tree. |

### Capability-demo analysis (Chapter 11 of the report)

| File | Role |
|---|---|
| [`plot_capability_demo.py`](plot_capability_demo.py) | Reads the six capability-demo CSVs and produces the four overlay figures (tracking error, peak tension, $\sigma_T$, 3-D trajectories), the six per-variant "story" plots, and the headline `summary_metrics.csv`. Single command regenerates every figure in Chapter 11. |
| [`plot_review_augmentation.py`](plot_review_augmentation.py) | Re-analyses the archived capability-demo CSVs to produce fault-window zooms, normalised load-share plots, constraint-activity (thrust/tilt margin) plots, and the per-fault recovery-metrics table used in Chapter 11's later sections. No new simulation required; uses only the archived logs. |

### Phase-T analysis (Chapter 2A Domain Audit)

| File | Role |
|---|---|
| [`phase_t_domain_audit_v11.py`](phase_t_domain_audit_v11.py) | **Canonical.** Executes the v1.1 admissibility criterion (max slack-run ≤ 40 ms; duty cycle ≤ 2.5%; QP active-set-transition fraction) against archived missions. Produces `output/phase_t/domain_audit_v11_report.csv`. |
| [`phase_t_domain_audit.py`](phase_t_domain_audit.py) | **Historical.** The v1.0 script with the stricter "all-taut $T_\min > 0$" criterion that failed the gate on all six missions. Retained as the evidence record for `theorem_contracts_phase_T.md §12`; do not run for new analyses. |

### Phase-2 campaign analyses

Every Phase-2 campaign has a `plot_*` script (produces PNG figures +
summary CSV in `output/p2X/_summary/`) and a companion
`update_*_report.py` script (copies the PNGs into
`report/Figures/` and writes the LaTeX subfiles that the report
ingests).

| Campaign | Plotter | Report updater |
|---|---|---|
| P2-A (tension FF ablation) | [`plot_p2a_ff_ablation.py`](plot_p2a_ff_ablation.py) | [`update_p2a_report.py`](update_p2a_report.py) |
| P2-B (mass mismatch) | [`plot_p2b_mass_mismatch.py`](plot_p2b_mass_mismatch.py) | [`update_p2b_report.py`](update_p2b_report.py) |
| P2-C (MPC ceiling sweep) | [`plot_p2c_mpc_ceiling.py`](plot_p2c_mpc_ceiling.py) | [`update_p2c_report.py`](update_p2c_report.py) |
| P2-D (period sweep for reshape) | [`plot_p2d_period_sweep.py`](plot_p2d_period_sweep.py) | [`update_p2d_report.py`](update_p2d_report.py) |
| P2-E (wind × seed; deferred) | [`plot_p2e_wind_seed.py`](plot_p2e_wind_seed.py) | [`update_p2e_report.py`](update_p2e_report.py) |

### Pre-P2 publication pipeline (retained for the IEEE draft)

| File | Role |
|---|---|
| [`plot_scenario.py`](plot_scenario.py) | Per-scenario time-series plots (A/B/C/D). |
| [`plot_comparison.py`](plot_comparison.py) | Cross-scenario comparison plots. |
| [`plot_system.py`](plot_system.py) | System-level 3-D payload/drone trajectories. |
| [`plot_publication.py`](plot_publication.py) | Assembles the 12-figure IEEE publication suite. Consumed by `run_5drone_campaign.sh`. |
| [`plot_ablation.py`](plot_ablation.py), [`plot_ablation_ci.py`](plot_ablation_ci.py) | Cross-configuration ablation bars; consumed by `run_ablation_campaign.sh`. |
| [`fill_results_placeholders.py`](fill_results_placeholders.py) | Fills IEEE-T-CST `<<token>>` placeholders in the archived journal draft at `../../archive/docs_2026_04_23/latex/ieee_t_cst.tex` from `publication_metrics.csv`. |
| [`PUBLICATION_FIGURES.md`](PUBLICATION_FIGURES.md) | Per-figure reviewer justification and scientific-claim trail for the IEEE publication suite. |

## Typical invocation sequence

After a campaign runner finishes, the analysis is a single command:

```bash
# Capability demo
python3 Research/analysis/plot_capability_demo.py
python3 Research/analysis/plot_review_augmentation.py

# Phase-2 campaign (example: P2-A)
python3 Research/analysis/plot_p2a_ff_ablation.py
python3 Research/analysis/update_p2a_report.py

# Phase-T domain audit (canonical v1.1)
python3 Research/analysis/phase_t_domain_audit_v11.py
```

`phase2_watcher.sh` automates the plot-and-update step for each
P2-X campaign as soon as its CSV artefacts land.

## Cross-reference

- Campaign runners that produce the CSVs consumed here:
  [`../scripts/README.md`](../scripts/README.md).
- Simulator CLI and CSV schema:
  [`../cpp/README.md`](../cpp/README.md).
- Canonical report ingesting the figures and tables emitted here:
  [`/workspaces/Tether_Grace/report/main.pdf`](../../report/main.pdf).
- Phase-T contract:
  [`../../report/companions/theory/theorem_contracts_phase_T.md`](../../report/companions/theory/theorem_contracts_phase_T.md).
