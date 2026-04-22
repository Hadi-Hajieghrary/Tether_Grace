# Documentation Audit and Enrichment Plan
**Audit date:** 2026-04-21  
**Scope:** `/workspaces/Tether_Grace/Research/` — all theory docs, README files, source headers, runner scripts  
**Method:** Codebase Theorist deep audit → Adversarial Auditor independent challenge → direct file verification for all disputes  
**Status:** 10 confirmed discrepancies; 11 enrichment actions (4 paper-facing critical)

---

## 1. Confirmed Discrepancies

| ID | Location | Stated Claim | Ground Truth | Severity |
|---|---|---|---|---|
| **D1a** | `docs/theory/theory_rope_dynamics.md` §4 Eq.(4) | $k_{\text{eff}} = 25000/8 = 3125$ N/m | Code `main.cc:L372`: $25000/(8+1)=2778$ N/m. 8 beads → 9 segments | **High — paper-facing math error** |
| **D1b** | Same doc §7 table | Static stretch = 2.4 mm | Correct with 2778 N/m: $7.4/2778 = 2.7$ mm | Medium |
| **D1c** | Same doc §4 vs §6 | §4 uses denominator 8; §6 silently uses 9 | Internal inconsistency within one document | High |
| **D2** | `DECENTRALIZED_FAULT_AWARE_README.md` §3 | "S5 drone 0 @ t=14 s" | `run_ieee_campaign.sh:L55`: `--fault-0-time 20` | **High — wrong by 6 s** |
| **D4** | `docs/theory/theory_figure8_trajectory.md` §7 | "Only two shapes: traverse + figure8" | `lemniscate3d` branch at `main.cc:L131–180`; used for all 5-drone scenarios | **High — entire campaign undocumented** |
| **D5** | `Research/baseline/README.md` | "GPACQuadcopterController is wired and running" | Active controller is `DecentralizedLocalController`; GPAC superseded | **High — misleads onboarding** |
| **D6** | `cpp/include/decentralized_local_controller.h` Params defaults | `w_swing=0.8, w_effort=0.03, swing_kd=2.0` | Runtime: 0.3 / 0.02 / 0.8 (`main.cc:L573–578`) | Medium |
| **D7** | All `.cc` line citations in `theory_decentralized_local_controller.md` | e.g. `measured_tension` at `.cc:L108` | Actual: L129 (+21). `a_target`: cited L176, actual L197 (+21). `thrust`: cited L230, actual L279 (+49). Attitude loop: cited L246, actual L301 (+55). **Only `.cc:L35` latch init is accurate.** | Medium — all hyperlinks broken |
| **D8** | `DECENTRALIZED_FAULT_AWARE_README.md` §6 metrics | RMS/Peak values with no methodology note | Values use `ieee_style.trim_start(t_start=0.5)`. `fill_results_placeholders.py` does NOT call trim_start → LaTeX macros diverge from README table | Medium |
| **D9** | `theory_decentralized_local_controller.md` §9 | "steady-state error ≤ 5 cm" | True only at hold waypoints (t>22s, peak=50mm). During traverse: RMS=255mm, peak=302mm | Medium |
| **D10** | `DECENTRALIZED_FAULT_AWARE_README.md` §6 "Peak\|F\|=150N" | Appears to be measured peak rope force | Is the `max_thrust` actuator ceiling, not a measured quantity | Low |

---

## 2. Enrichment Actions

### Priority 1 — Paper-facing correctness (fix before any submission)

#### F1. Fix `theory_rope_dynamics.md` §4 Eq.(4) and §7 table  
**Files:** `Research/docs/theory/theory_rope_dynamics.md`  
**Change:**
- Eq.(4): Replace $k_s/N_{\text{seg}} = 25000/8 = 3125$ with $k_s/(N_{\text{beads}}+1) = 25000/9 \approx 2778$ N/m
- Add notation clarification: "$N_{\text{beads}}=8$ intermediate beads; $N_{\text{seg}} = N_{\text{beads}}+1 = 9$ springs in series"
- §7 table: update static stretch from 2.4 mm → 2.7 mm ($= 7.4/2778$)
- Make §4 and §6 consistent — both must use denominator 9

#### F2. Fix `DECENTRALIZED_FAULT_AWARE_README.md` §3 S5 fault time  
**File:** `Research/DECENTRALIZED_FAULT_AWARE_README.md`  
**Change:**
- Row S5: "drone 0 @ t=14 s" → "drone 0 @ t=20 s"
- Add annotation: "fault injected at 1.25 × T period (20/16)"
- Source: `run_ieee_campaign.sh:L55` (`--fault-0-time 20`)

#### F3. Qualify "≤ 5 cm steady-state error" in `theory_decentralized_local_controller.md` §9  
**File:** `Research/docs/theory/theory_decentralized_local_controller.md`  
**Change:**
- Replace bare "≤ 5 cm" with: "≤ 5 cm at hold waypoints (e.g. final landing hold, t > 22 s for S1); RMS during active traverse is 0.26 m, peak 0.30 m, limited by pendulum-lag dynamics"

#### F4. Add burn-in footnote to `DECENTRALIZED_FAULT_AWARE_README.md` §6 and theory §9  
**Files:** `Research/DECENTRALIZED_FAULT_AWARE_README.md`, `Research/docs/theory/theory_decentralized_local_controller.md`  
**Change:**
- Add to §6 table header: "All metrics computed for $t \geq 0.5$ s (startup transient excluded via `ieee_style.trim_start(t_start=0.5)`)"
- Mirror this note in theory doc §9 metrics paragraph

---

### Priority 2 — Stale documentation removal / replacement

#### F5. Overhaul `Research/baseline/README.md`  
**File:** `Research/baseline/README.md`  
**Change:**
- Remove entire "What Tether_Grace Has Verified" section describing GPAC as active
- Replace with: "As of 2026-04-21 the active controller is `DecentralizedLocalController` (fully-local QP per drone, `Research/cpp/src/decentralized_local_controller.cc`). GPAC was an intermediate design; retained in `archive_obsolete_designs/` for provenance only."
- Remove all references to `output/gpac/n4_cable_cut_t10_v21` (path does not exist)

#### F6. Update all `.cc` line citations in `theory_decentralized_local_controller.md`  
**File:** `Research/docs/theory/theory_decentralized_local_controller.md`  
**Reference table** (old → new):

| Cited | Context | Correct |
|---|---|---|
| `.cc:L108` | `measured_tension` | `.cc:L129` |
| `.cc:L116` | `p_self` | `.cc:L137` |
| `.cc:L140–147` | anti-swing slot correction | `.cc:L161–168` |
| `.cc:L149–155` | PD tracking accel | `.cc:L171–176` |
| `.cc:L176–215` | QP formulation block | `.cc:L197–245` |
| `.cc:L211–212` | tilt box constraint | `.cc:L240–241` |
| `.cc:L216` | `a_d_opt` | `.cc:L251` |
| `.cc:L230` | `double thrust =` | `.cc:L279` |
| `.cc:L238` | thrust clamp | `.cc:L290` |
| `.cc:L246–276` | attitude inner loop | `.cc:L301–320` |
| `.cc:L262–264` | torque clamp | `.cc:L316–318` |
| `.cc:L33–37` | pickup latch init | ✓ accurate (keep) |

**Root cause:** A 21-line diagnostics port block was inserted at `ctrl.cc:L57–71` after the theory doc was written.  
**Maintenance recommendation:** Add a CI step: scan `.md` files for `#L[0-9]+` anchors and verify the cited line still contains the expected symbol name.

---

### Priority 3 — Missing documentation

#### F7. Add §5 "lemniscate3d Trajectory" to `theory_figure8_trajectory.md`  
**File:** `Research/docs/theory/theory_figure8_trajectory.md`  
**Add:**
- 3-D Bernoulli lemniscate with vertical oscillation: $z(\varphi) = z_{\text{high}} + A_z\sin\varphi$
- Parameters: $a=3$ m, $A_z = 0.35$ m, $T = 12$ s (more dynamic than 2-D figure-8 at T=16 s)
- Parameter table matching §4 format
- Note: used exclusively for all 5-drone campaign scenarios
- Update §7 "Only two shapes" → "Three shapes: traverse, figure8, lemniscate3d"

#### F8. Add §8 "5-Drone Lemniscate Campaign" to `DECENTRALIZED_FAULT_AWARE_README.md`  
**File:** `Research/DECENTRALIZED_FAULT_AWARE_README.md`  
**Add:**
- Entry point: `run_5drone_campaign.sh`
- N=5 symmetric formation geometry
- Four scenarios: A nominal lemniscate3d 40s; B single fault drone0 t=15s; C dual fault drone0+2 (5s gap); D dual fault drone0+2 (10s gap)
- Trajectory: lemniscate3d, T=12s, Az=0.35m
- Output path: `output/Tether_Grace_5drone/`

#### F9. Fix `decentralized_local_controller.h` Params struct defaults  
**File:** `Research/cpp/include/decentralized_local_controller.h`  
**Change:** Update defaults to match runtime experiment values:
- `w_swing = 0.8` → `w_swing = 0.3`
- `w_effort = 0.03` → `w_effort = 0.02`
- `swing_kd = 2.0` → `swing_kd = 0.8`

This makes `main.cc` overrides a no-op and removes the false default signal.

#### F10. Standardize burn-in in `analysis/ieee/fill_results_placeholders.py`  
**File:** `Research/analysis/ieee/fill_results_placeholders.py`  
**Change:**
- Add `df = trim_start(df)` call (matching `ieee_style.py:L88`) to each CSV load
- This aligns generated LaTeX macro values with the README §6 table values

**Also:** Add docstring to `trim_start()` in `ieee_style.py`:
```python
"""Remove the first t_start seconds (default 0.5 s) of data.
All published RMS/Peak/T_peak metrics in DECENTRALIZED_FAULT_AWARE_README.md §6
and theory_decentralized_local_controller.md §9 are computed with this cutoff."""
```

#### F11. Add §9 "RopeSegmentTensionProbe" to `theory_rope_dynamics.md`  
**File:** `Research/docs/theory/theory_rope_dynamics.md`  
**Add:**
- Document the observer-only `RopeSegmentTensionProbe` (`rope_segment_tension_probe.h`)
- Output shape: $N_{\text{drones}} \times N_{\text{seg}}$ tension matrix per timestep
- Role: provides per-segment tensions $T_{i,j}(t)$ for figure F04 (tension waterfall plots)
- Note this is a logging-only system; it does not feed back into the controller

---

## 3. Evidence Basis

All findings are **Observed** (directly verified in source code via `grep -n` and file reads). No claims are based solely on documentation.

Key physics verification:
$$k_{\text{rope}}^{\text{eff}} = \frac{k_s}{N_{\text{beads}}+1} = \frac{25{,}000}{9} \approx 2778 \text{ N/m}$$
$$\Delta L_{\text{hover}} = \frac{m_L g / N_{\text{drones}}}{k_{\text{eff}}} = \frac{7.4}{2778} \approx 2.7 \text{ mm}$$

Code confirmation at `main.cc:L500`:
```cpp
const int num_rope_segments = num_rope_beads + 1;  // N_beads + 1 segments
```

---

## 4. Open Risks

1. **LaTeX macro divergence** — `fill_results_placeholders.py` currently generates incorrect metric values for the paper (no burn-in applied)
2. **No CI guard on doc-to-code anchors** — D7 will recur after the next non-trivial code insertion
3. **5-drone results exist but have zero theory backing** — not reproducible without F7/F8
4. **D6 header mismatch** — silent failure for any external controller instantiation (unit tests, Python bindings)

---

*Generated by Tether Research Squad audit pipeline, 2026-04-21*
