# IEEE T-CST Simulation Plan — 2026-04-25

This document specifies the simulation program that extends the
current V1--V6 + P2-A/B/C/D campaign with three Tier-1
non-negotiable analyses (reduction-fidelity audit,
communication-dependency comparison, integrated rescue mission),
four Tier-2 additions, and three Tier-3 supplementary studies.

## Directory layout (under `output/`)

| Path | Contents | Status |
|---|---|---|
| `capability_demo/V{1..6}/` | existing | done |
| `p2a_tension_ff_ablation/` | existing | done |
| `p2b_mass_mismatch/` | existing | done |
| `p2c_mpc_ceiling_sweep/` | existing | done |
| `p2d_period_sweep/` | existing | done |
| `dwell_sweep/dwell_{0p50,0p75,1p00,1p25,1p50,2p00,3p00}/` | E7 | running |
| `reshape_binding/reshape_T6_{off,on}/` | E8 | done, NF3 null |
| `reduction_fidelity/` | Tier 1A post-processing artefacts | **to build** |
| `comm_delay/{local_d0,centralized_d{0,20,50,100,200,500},observer_d{20,50,100,200,500}}/` | Tier 1B | **needs new code** |
| `rescue_mission/{full_stack,baseline,ff_off,l1_only,mpc_only,reshape_only}/` | Tier 1C | **needs new code** |
| `slack_domain/T{4,5,6,8,10,12}s_m{10,12,14,16,18}kg/` | Tier 2E | **to launch** |
| `l1_gain_map/gamma{0.1..2.0}/` | Tier 2F | partial |
| `sensor_imperfection/` | Tier 2G | **needs new code** |
| `mpc_binding/` | Tier 3H | **to launch** |
| `stratified_faults/seed_{0..99}/` | Tier 3J | **to launch** |

## Per-simulation specification

All sims use the canonical simulator
`Research/cpp/build/decentralized_fault_aware_sim`. The 188-column
CSV schema (`time, quad{i}_{x,y,z,vx,vy,vz}, payload_*, ref_*,
tension_{0..4}, swing_speed_{0..4}, qp_cost_{0..4},
qp_solve_us_{0..4}, T_ff_{0..4}, thrust_cmd_{0..4}, tilt_mag_{0..4},
swing_offset_{0..4}, act_a{x,y,z}_{lo,hi}_{0..4}`) is sufficient for
Tier 1A, 2D, 2E, 2F, 3H, 3J. Tier 1B and 2G require new log channels;
Tier 1C requires mission-phase annotations.

### Tier 1A — Reduction Fidelity Audit
**Purpose.** Defend Thm 1 (taut-cable reduction) by showing the
reduced scalar tension model tracks the Drake Kelvin--Voigt truth
within $O(\delta + \eta_{\max})$.

**Method.** Post-processing only. For each of the six capability
missions V1--V6:
1. Read `scenario_V{k}.csv`.
2. For each rope $i$, treat `tension_i(t)` as the KV truth $T_i^{\mathrm{KV}}$.
3. Compute the lumped prediction $T_i^{\mathrm{lumped}}$ from the
   reduction-theorem formula: at taut equilibrium,
   $T_i^{\mathrm{lumped}} \approx \frac{1}{|\mathcal{S}(t)|}(m_L g +
   m_L a_{\mathrm{ref},z}) + \text{horizontal geometry term}$.
4. Compute error $\varepsilon_i = |T_i^{\mathrm{KV}} - T_i^{\mathrm{lumped}}|$
   per sample; aggregate RMS, 95th pct, peak per rope per variant.
5. Correlate error with slack duty cycle bucketed per 1-second window.

**Artefacts.**
- `output/reduction_fidelity/V{k}_audit.csv` — per-sample columns
  `time, rope, T_KV, T_lumped, eps, slack_flag`.
- `output/reduction_fidelity/summary.csv` — per-variant per-rope
  `rms_eps, p95_eps, peak_eps`.

**Figures.**
- `fig_reduction_fidelity_V4_overlay.png` — $T_i^{\mathrm{KV}}$ vs
  $T_i^{\mathrm{lumped}}$ for V4, surviving ropes, solid vs dashed.
- `fig_reduction_error_cdf.png` — empirical CDF of $\varepsilon_i$
  aggregated across V1--V6, with $O(\delta + \eta_{\max})$ vertical
  guide.
- `fig_reduction_error_vs_slack.png` — scatter of $\varepsilon_i$
  versus slack duty cycle (per 1-s window).

**Metrics.** RMS/P95/peak tension-model error per rope and per
variant.

**Claim support.** B4 (direct, replaces the current indirect
gate-based argument).

### Tier 1B — Communication-Dependency Comparison *(REQUIRES NEW CODE)*
**Purpose.** Defend the strict-locality innovation against
centralized and observer-based baselines.

**Code required.**
1. `centralized_qp_controller.{h,cc}` — global force-allocation
   QP run by a single supervisor; distributes thrust targets to
   all drones. CLI: `--controller centralized`.
2. Delay model: buffer centralized commands by
   `--comm-delay-ms D` in the command path.
3. `observer_reassign_controller.{h,cc}` — per-drone tension-loss
   detector with `--detect-delay-ms` holdoff; on detection,
   re-distributes feed-forward across surviving drones.
4. Log additions: `comm_queue_depth`, `detected_fault_time_i`,
   `reassign_active_i`.

**Sweep.** Delay $d \in \{0, 20, 50, 100, 200, 500\}$ ms on V4
schedule, payload 10 kg, Dryden seed 42. 17 runs total (proposed
local + 5×centralized + 5×observer + baseline sanity-checks).

**Figures.**
- `fig_comm_peak_sag_vs_delay.png` — peak sag vs delay per controller.
- `fig_comm_recovery_time_vs_delay.png`.
- `fig_comm_peak_tension_vs_delay.png`.
- `fig_comm_timeline_V4.png` — fault → detect → reassign → recover
  timeline.
- `fig_comm_thrust_response.png` — fault-centered thrust comparison
  across all controllers.

**Metrics.** Peak sag, recovery-time to within 10 mm, peak rope
tension, actuator-saturation fraction, mission-success flag.

**Claim support.** C2 innovation (local-information sufficiency)
elevated from rhetorical to quantitative.

**Scope.** ~3--5 engineer-days plus 17 sim runs (~6 h compute).

### Tier 1C — WITHDRAWN
A mission orchestrator that schedules phases, mass changes, or
controller reconfiguration violates the paper's strict-locality
thesis. Compound-mission demonstration is provided by the
existing V4/V5/V6 dual-fault campaigns under their existing
configuration (multiple faults, wind, extension stack) without
any external scheduler. No orchestrator-based deliverable is
pursued.

### Tier 2D — Dwell-Time Boundary Sweep (in progress)
Already running; 2 of 7 points done at time of writing. Adds
fault-phase sweep as a future extension.

### Tier 2E — Slack-Domain Boundary Phase Diagram
**Purpose.** Show where Thm 1's hypotheses break and the theorem
stops applying.

**Sweep.** $T_{\mathrm{ref}} \in \{4, 5, 6, 8, 10, 12\}$ s ×
$m_L \in \{10, 12, 14, 16, 18\}$ kg = 30 grid points. V4 fault
schedule, Dryden seed 42.

**Existing CLI.** `--lemniscate-period`, `--payload-mass`
(already present).

**Figures.**
- `fig_phase_pass_fail.png` — binary pass/fail phase diagram
  (acceptance thresholds from §VI).
- `fig_phase_slack_duty_contours.png`.
- `fig_phase_qp_transition_contours.png`.
- `fig_phase_max_slack_run_contours.png`.
- `fig_phase_theorem_boundary_overlay.png` — empirical failure
  boundary vs theorem-domain hypotheses H1/H3.

**Metrics.** For each grid point: pass/fail, RMSE, peak sag,
slack-duty %, max slack-run ms, QP transition %.

**Claim support.** X9 (operating boundary characterisation)
extended to a 2-axis empirical boundary map.

**Scope.** 30 runs × 22 min = 11 h serial compute (or 5 h with 2
parallel streams).

### Tier 2F — L1 Stability Map *(PARTIAL CODE)*
**Purpose.** Validate Proposition 1 ($\Gamma < 2/(T_s p_{22})$).

**Code required.**
- CLI addition: `--l1-gain <float>` to override the compiled-in
  $\Gamma$.

**Sweep.** $\Gamma/\Gamma^\star \in \{0.1, 0.25, 0.5, 0.75, 1.0,
1.25, 1.5, 1.75, 2.0\}$ × $m_L \in \{2.5, 3.0, 3.5, 3.9\}$ kg =
36 runs. No faults, fault-free cruise.

**Figures.**
- `fig_l1_rmse_vs_gamma.png`.
- `fig_l1_stability_map.png` — stable/unstable gain phase map.
- `fig_l1_adaptive_correction.png` — $u_{\mathrm{ad}}(t)$ at a
  representative operating point.

**Scope.** ~0.5 engineer-day for CLI wiring + 36 runs (~13 h
compute).

### Tier 2G — Sensor Imperfection Heatmap *(REQUIRES NEW CODE)*
**Purpose.** Test $T_i^{\mathrm{ff}} = T_i$ robustness under
realistic tension sensing.

**Code required.** Tension-sensor model with `--tension-delay-ms`,
`--tension-noise-std`, `--tension-bias`, `--tension-quant-N`,
`--tension-dropout-rate`.

**Scope.** ~1--2 engineer-days + ~40-sample heatmap grid × 22 min.

### Tier 3H — MPC-Binding Scenario
**Purpose.** Drive MPC layer into actually binding regime (current
NF2 null).

**Method.** Heavier payload (25 kg), faster reference ($T=6$ s),
tighter ceiling (40 N), dual faults. No new code.

**Runs.** Paired (MPC on/off) at the driven regime + 3 ceiling values.

**Figures.** Per-rope tension with ceiling overlay, constraint
violation integral, QP cost spike.

**Scope.** 4 runs (~90 min compute).

### Tier 3I — Reshape-Binding Scenario (done — NF3 null confirmed)
Aggressive T=6 s reshape probe completed; 0.0% measured benefit,
linearised 25.8% prediction not observed empirically. Results
documented in `§VI.F` and `rem:reshape-null`.

### Tier 3J — Stratified Random Fault-Schedule Sampling
**Purpose.** Show fault schedules aren't hand-picked.

**Method.** Sample 100 admissible schedules: uniform over
$(t_1 \in [8, 20]$ s, $\Delta t \in [\tau_{\mathrm{pend}}, 10]$ s,
$i_1 \in \{0..4\}$, $i_2 \ne i_1)$.

**Figures.** Empirical RMSE distribution, worst/median/best
trajectories, 95th-percentile envelope.

**Scope.** 100 runs × 22 min = 37 h serial compute (or 18 h with 2
streams).

## §VI integration plan

- §VI.A.1 (new) — Reduction-Fidelity Audit: Tier 1A.
- §VI.B replaces current V1--V6 narrative with confidence intervals
  (requires n=32 seeded sweeps, secondary to Tier 1).
- §VI.C: Fault progression (existing).
- §VI.D: FF ablation (existing).
- §VI.E: Dwell-time boundary sweep (existing scaffolding; fills
  when Tier 2D completes).
- §VI.F (expanded) — Mass mismatch + L1 stability map (Tier 2F).
- §VI.G (new) — Communication-Dependency Comparison: Tier 1B.
- §VI.H (existing, extended): Extensions + Reshape binding null.
- §VI.I (new) — Integrated Rescue Mission: Tier 1C. Paper
  centerpiece.
- §VI.J (existing, renamed) — Failure and Operating Boundary Suite
  + phase diagram: Tier 2E + existing H2 probe.
- §VI.K — V4 Signal Atlas (existing).
- §VI.L — Reproducibility and Claim-to-Evidence Map (existing).

## Session execution priority

Given the ~3-5-day engineering scope for Tier 1B, ~1-2 days for 1C,
and ~2 days for 2G, **the current session can complete
Tier 1A, 2E, 3H, and 3J plus finish Tier 2D (already running)**.
Tiers 1B, 1C, 2F, 2G are RFCs for follow-up engineering sessions.
