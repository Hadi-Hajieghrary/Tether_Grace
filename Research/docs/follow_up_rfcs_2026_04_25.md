# Follow-Up RFCs — Tier 1B / 1C / 2F / 2G

The simulation plan (`simulation_plan_2026_04_25.md`) marks four
items as requiring non-trivial new code that is out of scope for
the current session. This document specifies each as an
implementation RFC for follow-up work.

## RFC-1B: Communication-Dependency Comparison

### Goal
Defend the strict-locality innovation by comparing the proposed
controller against centralized-QP and observer-based-reassignment
baselines under realistic communication and detection delays.

### New code
1. `Research/cpp/include/centralized_qp_controller.h` and
   `.cc`. A single-process force-allocation QP that:
   - Receives full plant state $x_{\mathcal{N}}, x_L$ each tick.
   - Solves a global QP for thrust assignments $f_i$ that
     minimizes the weighted sum of altitude-tracking error,
     formation error, and tension imbalance.
   - Applies a configurable buffer (round-trip delay
     $d_\mathrm{comm}$) before the per-drone command takes
     effect.
2. `observer_reassign_controller.h` and `.cc`. Per-drone:
   - Local fault detector: declares peer fault when `tension_j`
     drops below threshold for $\tau_\mathrm{det}$ continuously
     (`--detect-delay-ms`).
   - On detection, redistributes feed-forward across the
     observer-reported surviving set (instead of the structural
     identity $T_i^\mathrm{ff}=T_i$).
3. CLI additions to `decentralized_fault_aware_sim_main.cc`:
   `--controller {local,centralized,observer}`,
   `--comm-delay-ms <int>`, `--detect-delay-ms <int>`.
4. New CSV columns: `comm_queue_depth`, `detected_fault_time_i`,
   `reassign_active_i`.

### Sweeps
- delay $d \in \{0, 20, 50, 100, 200, 500\}$ ms
- 17 paired runs at V4 schedule, payload 10 kg, Dryden seed 42.
- Output: `output/comm_delay/{local_d0, centralized_d{0..500},
   observer_d{20..500}}/`.

### Figures
- `fig_comm_peak_sag_vs_delay.png`
- `fig_comm_recovery_time_vs_delay.png`
- `fig_comm_peak_tension_vs_delay.png`
- `fig_comm_timeline_V4.png` (fault → detect → reassign → recover)
- `fig_comm_thrust_response.png`

### Effort
~3--5 engineer-days for new controllers + delay model + 17 sim
runs (~6 h compute).

### Risk
Centralized-QP performance at $d=0$ may match local controller
because both leverage the same physics; the comparison's
sharpness emerges only at $d\!\geq\!50$ ms. Mitigation: include
a single-fault scenario where the centralized solution
analytically dominates absent delay, then show delay degradation
crosses the proposed local controller.

## RFC-1C: WITHDRAWN --- orchestrator out of scope

A central orchestrator that schedules mission phases, mass
changes, or controller reconfiguration violates the paper's
strict-locality thesis: each drone executes its local
controller without notification, supervisor, or shared
scheduler. Any compound-mission demonstration must therefore be
achieved through the configuration knobs the existing
decentralized simulator already exposes
(`--lemniscate-period`, `--payload-mass`,
`--fault-{0,1,2}-{quad,time}`, `--wind-speed`,
`--reshaping-enabled`, etc.). Within that constraint, the
existing V4/V5/V6 dual-fault scenarios already serve as
compound demonstrations: dual fault sequence, wind, full
extension stack, all without any phase-scheduling supervisor.

A richer single-run compound demonstration on the canonical
trajectory family is preserved in the V6 + dwell-sweep
combination already in §VI. No orchestrator-based RFC is
pursued.

## RFC-2F: L1 Stability Map

### Goal
Validate Proposition 1's $\Gamma < 2/(T_s p_{22})$ closed-form
gain bound.

### New code
- CLI: `--l1-gain <float>` to override the compiled-in $\Gamma$.

### Sweep
$\Gamma/\Gamma^\star \in \{0.1, 0.25, 0.5, 0.75, 1.0, 1.25, 1.5,
1.75, 2.0\}$ × $m_L \in \{2.5, 3.0, 3.5, 3.9\}$ kg = 36 runs,
fault-free cruise.

### Figures
- `fig_l1_rmse_vs_gamma.png`
- `fig_l1_stability_map.png` (stable/unstable phase map)
- `fig_l1_adaptive_correction.png`

### Effort
~0.5 engineer-day + 36 runs (~13 h compute).

## RFC-2G: Sensor Imperfection Heatmap

### Goal
Test $T_i^\mathrm{ff}=T_i$ robustness under realistic
non-idealities (delay, noise, bias, quantization, dropout).

### New code
Tension-sensor model: `tension_sensor.h/cc`. Per-drone
configurable corruption pipeline:
- `--tension-delay-ms <int>` — pure transport delay
- `--tension-noise-std <float>` — additive Gaussian noise
- `--tension-bias <float>` — constant offset
- `--tension-quant-N <int>` — N-bit ADC quantization
- `--tension-dropout-rate <float>` — random sample dropouts

### Sweep
Two-axis heatmap, e.g., delay × noise: $d \in \{0,5,10,20,50\}$ ms
× $\sigma \in \{0,0.5,1,2,5\}$ N = 25 grid points × paired
controllers.

### Figures
- `fig_sensor_peak_sag_heatmap.png`
- `fig_sensor_rmse_heatmap.png`
- `fig_sensor_max_tolerable_delay.png`
- `fig_sensor_true_vs_measured_thrust.png`

### Effort
~1--2 engineer-days for sensor model + 50+ runs (~18 h compute).

## RFC-WP5: Active fault-tolerance baseline (head-to-head)

### Goal
Provide the comparative evidence that converts the architectural
locality claim from rhetoric into measurable advantage.

### What "active baseline" means here
A faithful instance of the **detect-classify-reconfigure**
architectural class. The implementation does not need to
reproduce any specific paper bit-by-bit; it needs to instantiate
the structural commitment the paper argues against. Natural
candidate: per-drone tension-residual observer with detection
threshold and dwell, plus a reassignment law that
redistributes feed-forward shares across the observer-reported
surviving set, parameterised by detection-delay
$\tau_{\mathrm{det}}$.

### New code (in scope as comparison infrastructure, not as
part of the proposed system)
1. `observer_reassign_controller.h/cc` --- per-drone tension-loss
   detector + reassignment law. Signature: same input/output as
   `decentralized_local_controller`, but feed-forward share is
   $T_i^{\mathrm{ff}} = (m_L g)/|\widehat{\mathcal{S}}(t)|$ where
   $\widehat{\mathcal{S}}$ is the observer-reported surviving set.
2. CLI: `--controller observer-reassign`,
   `--detect-delay-ms <int>`, `--detect-threshold-N <float>`.
3. Logged additions: `detected_fault_time_i`,
   `reassign_active_i`, `reassign_share_i`.

### Sweep
- Detection delay $\tau_{\mathrm{det}} \in \{0, 20, 50, 100,
  200\}$ ms (the proposed strict-local controller is the
  $\tau_{\mathrm{det}}=0$ idealised reference).
- V3, V4, V5 schedules; seed 42.
- 15 runs total ($\sim$5.5 h compute).

### Figures
- `fig_compare_peak_sag_vs_delay.png`
- `fig_compare_recovery_time_vs_delay.png`
- `fig_compare_peak_tension_vs_delay.png`
- `fig_compare_timeline_V4.png` (fault $\to$ detect $\to$
  reassign $\to$ recover for the active baseline; instantaneous
  for the proposed controller)

### Effort
~3--4 engineer-days for the observer/reassignment controller;
1 day for the sweep + post-processing.

### Risk
The active baseline at $\tau_{\mathrm{det}}=0$ may match the
proposed controller because both leverage the same physics; the
comparison's sharpness emerges only at $\tau_{\mathrm{det}} \ge
20$ ms. This is exactly the point: the comparison is between
\emph{architectures}, and the cost of the
detection-classification timeline is the architecture-level
quantity to measure.

## RFC-WP6.1: Parallel reduced-order simulator + trajectory-level reduction audit

### Goal
Replace the indirect domain-gate argument for Theorem 1 with a
direct trajectory-level comparison between the bead-chain truth
model (Drake, current sim) and a separate reduced-rope model
running the same controller.

### New code
A standalone reduced-rope plant that replaces each cable with a
massless scalar spring at the lumped effective stiffness from
Section II-C. The same `DecentralizedLocalController` instances
run in both. Input: trajectory references and seed; output: per-
sample payload and drone states.

### Sweep
V3 and V4 schedules in both simulators; seed 42; 30 s each.
4 runs total.

### Figures
- `fig_reduction_truth_vs_lumped_payload_V4.png` --- payload
  trajectory overlay (Drake KV, lumped) for V4.
- `fig_reduction_trajectory_envelope_V4.png` --- trajectory
  deviation $\|\xi_{\mathrm{KV}}(t) - \xi_{\mathrm{lumped}}(t)\|$
  vs the analytical $O(\delta + \eta_{\max})$ envelope.

### Effort
~1 week to stand up the reduced-rope plant and validate against
a no-fault sanity case; 1 day for sweeps; 1 day for figures.

### Outcome
This is the audit that converts the reduction-fidelity claim B4
from indirect (gates pass) to direct (trajectories agree).

## Sequencing recommendation

The remaining RFCs in priority order:

1. **RFC-WP5 (active-FT baseline comparison)** --- highest
   external-critique payoff. Without it, the architectural
   locality claim is rhetorical. The centralized/observer-based
   baselines are the *alternatives the paper improves on*, not
   alternative implementations of our system; their
   implementation is in scope as comparison infrastructure.
2. **RFC-WP6.1 (parallel reduced-order simulator)** --- closes
   the most consequential evidence gap a senior reviewer will
   identify. Direct trajectory-level audit replaces the indirect
   domain-gate argument.
3. **RFC-2F (L1 stability map)** --- in execution this session.
4. **RFC-1B (legacy comm-delay)** --- subsumed by RFC-WP5.
5. **RFC-2G (Sensor Imperfection)** --- secondary;
   hardware-credibility evidence.

**RFC-1C remains withdrawn** as out of scope (orchestrator
violates strict-decentralized thesis).
