# Advanced Improvement Plan — Tether_Grace Research System

**Date:** 2026-04-21
**Scope:** Code quality, implementation correctness, documentation, theoretical backbone, and publication evidence.
**Status of immediate fixes:** See [DOC_AUDIT_AND_ENRICHMENT_PLAN.md](DOC_AUDIT_AND_ENRICHMENT_PLAN.md) for D1–D10 resolution table.

All items below are improvements **beyond** the immediate discrepancy fixes.
Severity is rated: 🔴 Critical / 🟠 High / 🟡 Medium / 🟢 Low.

---

## Track A — Code Quality and Correctness

### A1 🔴 QP solver: replace with closed-form clip
**Problem:** Drake `MathematicalProgram` + `OsqpSolver` is used for a
3-variable box-constrained QP that has a closed-form solution
(see §11 of `theory_decentralized_local_controller.md`).
**Risk:** Solver call latency is currently 0.1 ms — acceptable at
2 ms timestep — but any future reduction in dt or increase in N_drones
will become a bottleneck.
**Action:** Replace `MathematicalProgram` in
`decentralized_local_controller.cc:L227–245` with the 6-clip
closed-form (Eqs. 12–13). Keep the Drake formulation in a
`#ifdef TETHER_USE_QP_SOLVER` guard for regression testing.
**Files:** `cpp/src/decentralized_local_controller.cc:L227–245`
**Effort:** 1 day.

### A2 🟠 Header defaults vs. runtime values divergence prevention
**Problem fixed:** D6 corrected `swing_kd=2.0→0.8`, `w_swing=0.8→0.3`,
`w_effort=0.03→0.02`. But the divergence was not caught for months.
**Action:** Add a static assert or runtime assertion in
`decentralized_local_controller.cc` constructor that warns when any
runtime override deviates from the header default by more than a
configured tolerance. Alternatively, remove the defaults entirely and
require explicit construction from `main.cc`.
**Files:** `cpp/include/decentralized_local_controller.h`,
`cpp/src/decentralized_fault_aware_sim_main.cc:L556–578`
**Effort:** 0.5 day.

### A3 🟠 Burn-in alignment guard in fill_results_placeholders.py
**Problem fixed:** D8 fixed the missing `trim_start` call. But nothing
prevents a future developer from changing the `T_BURN_IN` constant
in `fill_results_placeholders.py` without updating `ieee_style.py`
(or vice versa).
**Action:** Define a single `BURN_IN_SECONDS = 0.5` constant in
`ieee_style.py` and import it in `fill_results_placeholders.py`.
Remove the independent `T_BURN_IN = 0.5` literal.
**Files:** `analysis/ieee/ieee_style.py`, `analysis/ieee/fill_results_placeholders.py`
**Effort:** 30 min.

### A4 🟡 Stale-citation CI script
**Problem:** Seven citation discrepancies (D7 class) accumulated because
`.cc:L<n>` anchors in Markdown are not checked against actual file
line numbers. D7 was corrected manually but will drift again.
**Action:** Create `Research/scripts/check_doc_anchors.sh`:
```bash
#!/usr/bin/env bash
# Extract all .cc#L<n> and .h#L<n> anchors from Research/docs/**/*.md
# and verify that the referenced line in the source file contains
# the expected symbol keyword (extract from the table cell context).
# Exit non-zero if any anchor is more than 5 lines off.
grep -rn '\(\.cc\|\.h\)#L[0-9]' Research/docs/theory/*.md \
  | while IFS=: read file lineno match; do
      src=$(echo "$match" | grep -oP '(?<=\().*?(?=\.cc)').cc
      lnum=$(echo "$match" | grep -oP 'L\K[0-9]+')
      # ... verify src line lnum is non-empty
    done
```
**Files:** `Research/scripts/check_doc_anchors.sh` (new)
**Effort:** 1 day.

### A5 🟡 Rope parameters in a single header
**Problem:** `num_rope_beads`, `k_s`, `c_s`, `L_seg`, `formation_radius`
are hardcoded as magic numbers in `decentralized_fault_aware_sim_main.cc`.
The theory docs and header must be updated manually when any parameter
changes.
**Action:** Create `cpp/include/rope_parameters.h` with a `RopeParams`
struct. Reference it in both `main.cc` and the `RopeSegmentTensionProbe`.
**Files:** `cpp/include/rope_parameters.h` (new), `cpp/src/decentralized_fault_aware_sim_main.cc`
**Effort:** 0.5 day.

### A6 🟢 Pickup-ramp smoothstep: unit test
**Problem:** The Hermite-cubic pickup ramp (Eq. 8b in theory) is verified
only visually from the pickup-stretch time series figure. No unit test
checks that $\alpha(0)=0$, $\alpha(1)=1$, $\dot\alpha(0)=0$,
$\dot\alpha(1)=0$.
**Action:** Add a Python unit test `Research/scripts/test_pickup_ramp.py`
verifying boundary values.
**Effort:** 0.5 day.

---

## Track B — Documentation Quality

### B1 🔴 Fill 5-drone metrics table in README §8.3
**Problem:** The §8.3 table added in this session has placeholder `—`
cells. These must be populated before submission.
**Action:** Run `./run_5drone_campaign.sh` + `fill_results_placeholders.py`
and paste the resulting LaTeX macro values into the README table.
**Files:** `DECENTRALIZED_FAULT_AWARE_README.md §8.3`
**Effort:** 30 min + campaign runtime (~60 min).

### B2 🟠 Fault model theory document
**Problem:** `docs/theory/theory_fault_model.md` is referenced in the
navigation table but has not been verified/confirmed to exist.
**Action:** Check existence; if missing, create it with the CableFaultGate
model: zero-force after fault, latch mechanism, SafeHoverController
fall-through, ControlModeSwitcher transition diagram.
**Files:** `docs/theory/theory_fault_model.md`
**Effort:** 2 days.

### B3 🟠 Link §9 (RopeSegmentTensionProbe) in docs navigation table
**Problem:** The new §9 in `theory_rope_dynamics.md` is not listed in
the `DECENTRALIZED_FAULT_AWARE_README.md` navigation table.
**Action:** Update §9 "Where to go next" table with a row for tension
probe usage.
**Files:** `DECENTRALIZED_FAULT_AWARE_README.md §9`
**Effort:** 5 min.

### B4 🟡 Cross-link theory docs to each other
**Problem:** Theory documents exist but cross-link sparsely. A reader of
`theory_rope_dynamics.md` never reaches `theory_decentralized_local_controller.md`
except via the README.
**Action:** Add a "Related documents" footer section to each theory doc
pointing to the other two.
**Files:** All four `docs/theory/*.md` files
**Effort:** 1 hour.

### B5 🟢 Notation consistency pass
**Problem:** The four theory docs use slightly different notation:
- $N_s$ vs. $N_{\text{seg}}$ vs. $N_s$
- $f_i$ vs. $u_i$ for thrust
- $\bar{\mathbf{p}}_L$ vs. $\mathbf{p}_L^{\text{ref}}$ for payload ref
**Action:** Pick one notation per symbol and update all four docs.
Maintain a `docs/notation_glossary.md` as the authoritative table.
**Files:** All four theory docs + new `docs/notation_glossary.md`
**Effort:** 2 hours.

---

## Track C — Theoretical Backbone (Advanced Contributions)

### C1 🟠 Input-to-state stability (ISS) argument for inner/outer cascade
**Current state:** §9 of `theory_decentralized_local_controller.md`
gives empirical validation only.
**Proposed contribution:** Linearise the outer-loop position error dynamics
around the formation-slot equilibrium. Show the linearised system is ISS
with disturbance input = pendulum-induced rope horizontal force. Bound
the L∞ gain from rope force to position error in terms of
$K_p^{xy}, K_d^{xy}, m_i$.
**Expected result:** A sufficient condition on rope stiffness for ISS:
$k_{\text{eff}} > m_i (K_p^{xy} + K_d^{xy}/T_{\text{step}}) / (N L_{\text{chord}})$
— which is satisfied by 2778 N/m with a large margin.
**Effort:** 2–3 days (analytical, no new code).

### C2 🟡 QP active-set occupancy as a safety index
**Current state:** Active-set bits are logged in the 13-signal diagnostic
port but not published as a metric.
**Proposed contribution:** Define a *saturation index*
$\Gamma(t) = |\mathcal{A}(t)| / 6$ (fraction of the 6 box-constraint
faces active at time $t$). A trajectory is "safe" iff
$\sup_t \Gamma(t) < 1$. Compute this for all scenarios and publish as
a saturation-timeline figure. High $\Gamma$ near fault events is a
predictive precursor to performance degradation.
**Effort:** 1 day (analysis script + figure).

### C3 🟡 Closed-loop pole locus vs. $N_{\text{alive}}$
**Current state:** The 12-figure suite includes a "closed-loop pole locus
vs. $N_{\text{alive}}$" figure (mentioned in §7.5 of README), but the
theory behind it is not documented.
**Proposed contribution:** Derive the 2-DoF closed-loop poles of the
$(z, \dot{z})$ subsystem as $N$ drones drop out one by one (1 fault at
a time). The tension feed-forward $T_{\text{ff}} \leftarrow T_{\text{meas}}$
effectively rescales the effective gravity seen by the drone; show the
pole trajectory stays in the left-half plane for $N \ge 2$.
**Effort:** 1 day.

### C4 🟢 Pendulum resonance avoidance — trajectory design rule
**Current state:** §12.4 of `theory_decentralized_local_controller.md`
derives $\omega_p \approx 2.84$ rad/s and notes $\omega_f \ll \omega_p$
for the figure-8.
**Proposed contribution:** Formalise as a design rule: given rope length
$L_{\text{chord}}$, the maximum safe trajectory frequency is
$\omega_f^{\max} = \omega_p / \text{SR}$ where SR ≥ 3 is a
spectral-separation ratio. For the nominal geometry
$\omega_f^{\max} \approx 0.95$ rad/s ↔ minimum safe period $T_{\min}
\approx 6.6$ s. The figure-8 uses $T = 16$ s (safety factor 2.4×);
lemniscate3d uses $T = 12$ s (factor 1.8×). Publish as a contour map
in $(a, T)$ space.
**Effort:** 0.5 day.

### C5 🟢 Bead-chain axial wave speed and pickup ramp design criterion
**Current state:** The smoothstep ramp duration $\tau_p = 1$ s is chosen
by inspection. No criterion ties it to the rope dynamics.
**Proposed contribution:** The 5 Hz axial mode of the 9-segment bead chain
(period $T_{\text{axial}} = 0.2$ s) sets a minimum ramp duration:
the ramp must be ≥ $5T_{\text{axial}} = 1$ s to avoid exciting the
resonance. This explains the observed value and provides a formula for
future rope configurations.
**Effort:** 0.5 day.

---

## Track D — Publication Evidence

### D-E1 🔴 Populate 5-drone metric table (B1, reiterated as D-level)
### D-E2 🟠 Generate and commit all 12 publication figures
**Status:** `plot_publication.py` exists; figures have not been committed
to the repo or the paper's `Figures/` folder.
**Action:** Run the pipeline end-to-end, export to
`IEEE_IROS_2026/Figures/`, update `Main.tex` to reference the correct
filenames.

### D-E3 🟠 Monte-Carlo inter-fault-gap — include results in paper
**Status:** `run_mc_interfault_sweep.sh` exists and produces
`output/Tether_Grace_5drone/09_publication_figures/fig_mc_*.pdf`.
The paper §IV does not reference this adversarial sweep.
**Action:** Add a paragraph citing the sweep result (peak tension <
$T_{\max}$ for all $\Delta t \ge 2$ s) as robustness evidence.

### D-E4 🟡 Baseline comparison table
**Status:** Paper and README compare against no external baseline.
**Action:** Add a 2-row comparison table to the paper: row 1 = naive
load-sharing (each drone always targets $m_L g / N$ regardless of
rope tension), row 2 = proposed system. Compute via a new `naive`
scenario in `run_ieee_campaign.sh`.

---

## Track E — Infrastructure

### E1 🟡 Reproducibility: pin Drake version in CMake
**Status:** `CMakeLists.txt` fetches `drake@master`. This means results
are not reproducible after an upstream Drake commit.
**Action:** Pin `drake@<specific-hash>` or use the Drake APT pinned
pre-built at the same hash used for the submission runs.

### E2 🟡 Continuous integration (CI)
**Status:** No CI. All tests are manual `make && ./run_...sh` invocations.
**Action:** Add a minimal `.github/workflows/ci.yml` that:
1. Builds `decentralized_fault_aware_sim` in Release mode.
2. Runs `./check_doc_anchors.sh` (from A4).
3. Runs S1 for 5 s, checks payload RMS < 0.5 m.

### E3 🟢 Dockerfile for reproducible paper environment
**Status:** The dev container exists (Tether_Lift/DevContainers/).
**Action:** Add a `Dockerfile.paper` that installs the pinned Drake,
Python deps, and LaTeX, allowing one-command paper build:
`docker run --rm tether_grace paper_build`.

---

## Implementation Priority Order

| Priority | Item | Track | Effort | Rationale |
|---|---|---|---|---|
| 1 | D5+D6 fixes already done | A | Done | Critical correctness |
| 2 | B1: Populate 5-drone metrics | B | 1.5 h | Needed before submission |
| 3 | A3: Single BURN_IN constant | A | 30 min | Prevents future D8-class drift |
| 4 | A4: CI anchor-lint script | A | 1 day | Prevents future D7-class drift |
| 5 | C1: ISS argument | C | 2–3 days | Elevates paper claim strength |
| 6 | D-E2: Commit publication figures | D | 1 h | Required for paper |
| 7 | D-E3: Monte-Carlo in paper | D | 1 h | Strong evidence |
| 8 | C2: QP saturation index | C | 1 day | New analytic metric |
| 9 | A1: QP closed-form impl | A | 1 day | Efficiency, clarity |
| 10 | B2: theory_fault_model.md | B | 2 days | Missing theory doc |
| 11 | C3: Pole locus derivation | C | 1 day | Supports §12 claim |
| 12 | D-E4: Baseline comparison | D | 2 days | Reviewers will ask |
| 13 | E1: Pin Drake version | E | 1 h | Reproducibility |
| 14 | E2: CI workflow | E | 1 day | Long-term maintenance |
| 15 | B5: Notation glossary | B | 2 h | Paper polish |

---

## Evidence Status Summary

| Claim | Evidence Status |
|---|---|
| `k_eff = 25000/9 ≈ 2778 N/m` | **Observed** (`main.cc:L372`) |
| S5 fault time = 20 s | **Observed** (`run_ieee_campaign.sh:L55`) |
| QP tilt budget $c_{xy} = g\tan(0.6) \approx 6.71$ m/s² | **Observed** (auditor-verified: 9.81×tan(0.6 rad)=6.71; previous value 7.4 was corrected) |
| Payload pendulum $\omega_p ≈ 2.84$ rad/s | **Supported Inference** (Eq. 15, verified geometry) |
| QP closed-form equivalence | **Supported Inference** (Eqs. 12–13, derivation) |
| Fault sub-mm z-sag (0.9 mm) | **Supported Inference** (Eq. 16, verified params) |
| 5-drone scenario A–D metrics | **Unverified** (placeholders pending campaign run) |
| ISS of cascade under rope disturbance | **Unverified** (proposed C1 work) |
| Peak tension < $T_{\max}$ for all $\Delta t \ge 2$ s | **Unverified** (pending MC sweep) |
| Steady-state error ≤ 5 cm at hold waypoints | **Observed** (empirical, S1 logs) |
