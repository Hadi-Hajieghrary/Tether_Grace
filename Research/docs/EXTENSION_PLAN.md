# Advanced Extension Plan — Tether_Grace

> **Status (2026-04-22):** ✅ **All three extensions implemented** in
> Phase-F (L1) / Phase-G+G-2 (MPC) / Phase-H (formation reshaping).
> See [`IMPLEMENTATION_STATUS.md`](IMPLEMENTATION_STATUS.md) for the
> active-code map. This plan remains as the *roadmap of record*.
>
> **Related theory docs:**
> - [theory_l1_extension.md](theory/theory_l1_extension.md) — ✅ Implemented
> - [theory_mpc_extension.md](theory/theory_mpc_extension.md) — ✅ Implemented (+ G-2 correction notes)
> - [theory_reshaping_extension.md](theory/theory_reshaping_extension.md) — ✅ Implemented (tension-reduction benefit is scenario-dependent; pending lemniscate re-run to quantify)

---

## Overview

Three mutually-compatible extensions to the baseline `DecentralizedLocalController` system.
Each extension is self-contained and can be run alone or composed with the others via
independent CLI flags.

| Ext | Name | Status | Files | CLI flag |
|-----|------|--------|-------|----------|
| E1 | L1 Adaptive Outer Loop | ✅ Implemented (Phase-F) | in-place in `decentralized_local_controller.*` | `--l1-enabled` |
| E2 | Receding-Horizon MPC with Tension Ceiling | ✅ Implemented (Phase-G, G-2) | `mpc_local_controller.{h,cc}`, `controller_utils.h` | `--controller=mpc` (+`--mpc-horizon`, `--mpc-tension-max`) |
| E3 | Formation Re-Shaping After Fault | ✅ Implemented (Phase-H) | `fault_detector.h`, `formation_coordinator.h` | `--reshaping-enabled` |

**Execution order (taken):** E1 → E2 → G-2 follow-up → E3, per the phase-by-phase user-review
checkpoints. All extensions compose orthogonally: e.g. `--controller=mpc --l1-enabled --reshaping-enabled`
runs the full stack (L1 inside MPC, with the supervisor re-slotting survivors on fault).

---

## Extension 1 — L1 Adaptive Outer Loop

### 1.1 Problem

The baseline hard-codes `pickup_target_tension_nominal = 7.36 N = m̂_L·g/N` and
`rope_drop` computed from a nominal $k_\text{eff}$. Unknown payload mass or rope stiffness
creates a persistent altitude error the PD loop cannot reject.

### 1.2 Solution Architecture

A discrete L1 adaptive controller running at 500 Hz:

1. **State predictor** (Eqs. 5.4a–b in theory doc): replicates the altitude error dynamics
   using the current estimated disturbance $\hat\sigma + u_\text{ad}$.
2. **Adaptive law** (Eq. 5.5): gradient step on the Lyapunov function to drive prediction
   error $\tilde{\mathbf{x}}_z = \hat{\mathbf{x}}_z - \mathbf{x}_z$ to zero.
3. **LP filter** (Eq. 5.7): $u_\text{ad}[k+1] = \alpha\,u_\text{ad}[k] + (1-\alpha)\hat\sigma[k+1]$
   with $\alpha = e^{-0.05} \approx 0.9512$.
4. **k_eff estimator** (Eq. 6.1): gradient update from measured tension vs. spring model.
5. **Injection**: single additive `a_target.z() -= u_ad` before the QP (§7.6).

### 1.3 Key Parameters

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| $\Gamma$ | 2000 | Per-step gain = 0.084, time constant = 24 ms; $\ll 47\,500$ stability limit |
| $\omega_c$ | 25 rad/s | Separation ratio $25/8.16 = 3.06$ (minimum 3) |
| $[\sigma_\min,\sigma_\max]$ | $[-5, 5]$ m/s² | Covers $\Delta m_L = \pm 3$ kg |
| $\Gamma_k$ | 5000 | Stiffness estimator (empirical; excitation-dependent) |
| $[\hat{k}_\text{eff,min},\hat{k}_\text{eff,max}]$ | $[500, 5000]$ N/m | Covers 0.2×–1.8× nominal |

### 1.4 Verified Mathematical Facts

| Fact | Value | Verification |
|------|-------|--------------|
| $A_m$ eigenvalues | $-5.37,\,-18.63$ | Analytic (Observed) |
| Lyapunov $p_{12}, p_{22}, p_{11}$ | $0.005,\;0.02104,\;2.224$ | Analytic (Observed) |
| $\sigma_\text{max}$ at $\Delta m_L = 3$ kg | $4.905$ m/s² | Analytic (Observed) |
| Gain stability bound | $\Gamma < 47\,500$ | Euler discretisation (Supported Inference) |

### 1.5 Implementation Scope

- **Files modified:** `decentralized_local_controller.h` (+11 Params fields, +1 private member),
  `decentralized_local_controller.cc` (+`CalcL1Update` method, +1 line in `CalcControlForce`)
- **New code:** ~80 lines total; no new files
- **Backward compatibility:** `l1_enabled = false` (default) — zero runtime change to existing scenarios

### 1.6 Test Scenarios

| ID | Configuration | Pass Criterion |
|----|--------------|----------------|
| L1-1 | Traverse, $m_L=4.5$ kg (50% over) | Steady-state $|e_z| < 2$ cm; $\hat\sigma \to -2.45$ m/s² in < 0.5 s |
| L1-2 | Figure-8, $k_\text{seg}=12500$ N/m (50% under) | $|\hat{k}_\text{eff} - 1389| < 15\%$ in < 5 s |
| L1-3 | Traverse + drone0 fault at 20 s, $m_L=3.6$ kg | $\hat\sigma$ re-converges after fault transient within < 0.1 s |

---

## Extension 2 — Receding-Horizon MPC with Hard Tension-Ceiling Constraint

### 2.1 Problem

The single-step QP has no prediction horizon and cannot prevent tension spikes arising from
aggressive trajectory tracking. Under compound faults, the surviving drone may encounter
rope tension transients exceeding the design limit ($T_\text{max,safe} = 100$ N).

### 2.2 Solution Architecture

Drop-in `MpcLocalController` class (same port signatures as `DecentralizedLocalController`):

1. **Error state:** $\mathbf{x}_i = [e_p; e_v] \in \mathbb{R}^6$, ZOH double integrator
   $A = I_3 \otimes [[1, T_s];[0,1]]$, $B = I_3 \otimes [[T_s^2/2];[T_s]]$.
2. **Condensed prediction:** Constant $\Phi \in \mathbb{R}^{6N_p\times 6}$ and
   $\Omega \in \mathbb{R}^{6N_p\times 3N_p}$ pre-computed at construction.
3. **Tension linearisation:** $T_i(k+j) \approx T_{i,\text{nom}} + k_\text{eff}\,\hat{n}_i^\top e_{p,i}$
   where $\hat{n}_i$ is the unit vector from payload to drone nominal position.
4. **QP (per step):** Constant Hessian $H = \Omega^\top\bar{Q}\Omega + \bar{R}$;
   per-step gradient $f = 2\Omega^\top\bar{Q}\Phi x_i(k)$ and tension RHS $d_T$.
   Only $(f, d_T)$ are updated per step; $H$ and $A_\text{ineq}$ are hot-started in OSQP.
5. **Terminal cost** $P_f$: DARE solution for $(A_s, B_s)$ with $q_p=100, q_v=10, r=1$.

### 2.3 Key Parameters

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| $N_p$ | 5 (baseline), 10 (evaluation) | 10 ms / 20 ms look-ahead; see §6.1 of theory doc |
| $q_p, q_v, r$ | 100, 10, 1 | Same as baseline QP weights |
| $T_\text{max,safe}$ | 100 N | Per rope hard ceiling |
| Tension slack $s_j$ | $\geq 0$, penalty $\rho = 10^4$ | Ensures feasibility at fault onset |

### 2.4 Corrected Geometry (Auditor Fix)

> **Critical correction applied after adversarial audit.**

The tension model must use the **attachment-to-attachment** chord, not the
body-centre-to-payload distance. Key corrected values:

| Quantity | Wrong (body-centre + $L_\text{rest}$) | Correct |
|----------|---------------------------------------|---------|
| Horizontal span | $r_f = 0.8$ m | $r_\text{eff} = 0.7\,r_f = 0.56$ m |
| Vertical span | $L_\text{rest} = 1.25$ m | $dz_\text{att} = \text{rope\_drop} - dz_q = 1.121$ m |
| Chord $d_\text{nom}$ | $\sqrt{0.64+1.5625} = 1.484$ m | $\sqrt{0.3136+1.2567} = 1.253$ m |
| Stretch $\delta_\text{nom}$ | 0.234 m | 0.003 m |
| $T_\text{nom}$ | 650 N (infeasible!) | **8.23 N** ✓ |

For implementation, the corrected approach uses an **effective rest length**
referenced to body-centre positions:
$$
L_\text{eff,body} = d_\text{body} - \delta_\text{nom}
= \sqrt{r_f^2 + \text{rope\_drop}^2} - 0.003 \approx 1.448\text{ m}
$$
so that `T = k_eff * max(d_body - L_eff_body, 0)` gives $T_\text{nom} = 8.23$ N
at hover. `L_eff_body` is computed once at construction from the hover fixed point.

### 2.5 Verified Mathematical Facts

| Fact | Value | Verification |
|------|-------|--------------|
| ZOH $B$ matrix entry $B_{00}$ | $T_s^2/2 = 2\times10^{-6}$ m | Analytic (Observed) |
| DARE eigenvalue $\|\lambda\|$ | $\approx 0.990$ (not 0.980; $\|\lambda\|^2 = \det = 0.980$) | Algebraic check (Supported Inference) |
| Tension spike timing at 0.5 m/s | $\sim$7 ms = 3.5 steps | Kinematic (Supported Inference) |
| $T_\text{nom}$ at hover | 8.23 N | Fixed-point iteration (Observed) |

### 2.6 Compute Budget

| Horizon | Decision vars | OSQP time (est.) | Budget |
|---------|---------------|------------------|--------|
| $N_p = 5$ | 20 | < 0.3 ms | 2 ms |
| $N_p = 10$ | 40 | < 0.6 ms | 2 ms |

### 2.7 Implementation Scope

- **New files:** `Research/cpp/include/mpc_local_controller.h`,
  `Research/cpp/src/mpc_local_controller.cc`
- **Dependency:** OSQP (or Eigen-based active-set fallback for small $N_p$)
- **CLI:** `--mpc-horizon N` in `decentralized_fault_aware_sim_main.cc`
- **Port signatures:** identical to `DecentralizedLocalController`

---

## Extension 3 — Formation Re-Shaping After Fault

### 3.1 Problem

After a drone fault, the surviving drones occupy asymmetric slots. Under dynamic loading
(lemniscate3d inner loop), the asymmetric formation creates tension asymmetry: the drone
nearest the gap carries disproportionate load. Moving survivors to equiangular slots minimises
$\max_i T_i$ — but requires knowing the fault identity (minimal peer communication).

### 3.2 Solution Architecture

A `FormationCoordinator` LeafSystem receives a fault announcement (scalar integer, -1 = no fault),
broadcasts new target slots to all surviving drones via an output port, and each drone transitions
its slot assignment with a quintic smoothstep over $T_\text{trans} = 5$ s.

### 3.3 Key Result: Optimal Angular Assignment for N=4→M=3

> **Corrected from naive 90° assumption.**

For $N=4$ with one fault, the equiangular new formation has $M=3$ drones at 120° spacing.
The optimal assignment minimises $\sum_i |\phi'_i - \phi_i|$ by choosing the rotation offset
$\phi_0$ of the new formation to minimise the max single-drone travel:

**Pattern:** the drone **opposite** the gap stays fixed ($\Delta\phi = 0$);
the two **adjacent** survivors each rotate $\mathbf{30° = \pi/6}$ toward the gap.

| $j^*$ (faulted) | Fixed drone | Two rotating drones | $\Delta\phi$ per rotating drone |
|-----------------|-------------|---------------------|---------------------------------|
| 0 (at $0°$) | drone at $180°$ (stays at $180°$) | $90°\to60°$, $270°\to300°$ | $30°$ |
| 1 (at $90°$) | drone at $270°$ (stays at $270°$) | $0°\to330°$, $180°\to210°$ | $30°$ |
| 2 (at $180°$) | drone at $0°$ (stays at $0°$) | $90°\to120°$, $270°\to240°$ | $30°$ |
| 3 (at $270°$) | drone at $90°$ (stays at $90°$) | $0°\to30°$, $180°\to150°$ | $30°$ |

All four cases are identical by symmetry: $\Delta\phi_\text{max} = \pi/6$ (not $\pi/2$).

### 3.4 Quintic Smooth Transition

$$
h(\tau) = 10\tau^3 - 15\tau^4 + 6\tau^5, \quad \tau = (t - t_\text{fault}) / T_\text{trans}
$$

$$
\phi_i(\tau) = \phi_{i,\text{old}} + (\phi'_i - \phi_{i,\text{old}})\,h(\tau)
$$

Maximum slot speed (peak of $h'$: $15/8$ at $\tau = 1/2$):
$$
|\dot{s}|_\text{max}
= r_f \cdot \Delta\phi_\text{max} \cdot \frac{15/8}{T_\text{trans}}
= 0.8 \times \frac{\pi}{6} \times \frac{15}{8 \times 5} = \mathbf{0.157\text{ m/s}}
$$

This is 3× slower than a naive 90° assignment would require (0.471 m/s) and well within
the anti-swing damper capability.

### 3.5 Tension Reduction

For the lemniscate3d inner loop (dominant centripetal loading), equiangular reshaping gives:

| | Asymmetric (old slots) | Equiangular (new slots) |
|---|---|---|
| $T_\text{max}$ | 19.88 N | 14.74 N |
| $T_\text{min}$ | ~0 N (near slack) | 10.26 N |
| **Reduction** | — | **−25.8%** (Weak Inference) |

The reduction formula is derived from quasi-static centripetal loading; full simulation
verification is needed.

### 3.6 Collision Safety

Minimum inter-drone horizontal chord during transition (equiangular $M=3$ at $r_f = 0.8$ m):
$$
d_\text{min} = 2 \times 0.8 \times \sin(45°) = 1.131\text{ m} \gg d_\text{safe} = 0.5\text{ m}
$$
**No full peer comm needed** for the $N=4\to M=3$ case; only the fault-ID broadcast.

### 3.7 Key Verified Facts

| Fact | Value | Verification |
|------|-------|--------------|
| $\Delta\phi_\text{max}$ (optimal assignment) | $\pi/6 = 30°$ | Minimax optimization (Supported Inference) |
| Max slot speed | 0.157 m/s | Analytic with quintic (Observed) |
| Collision safety margin | 1.131 m > 0.5 m | Analytic (Observed) |
| Tension reduction % | 25.8% | Quasi-static derivation (Weak Inference) |

### 3.8 Implementation Scope

- **New files:**
  - `Research/cpp/include/formation_coordinator.h` (~80 lines)
  - `Research/cpp/src/formation_coordinator.cc` (~200 lines)
- **Modified:**
  - `decentralized_local_controller.h`: add optional `new_slot_override_port` (6D)
  - `decentralized_fault_aware_sim_main.cc`: wire `FormationCoordinator`, add `--reshaping-enabled`
- **Communication model:** ONE broadcast (fault-ID $j^*$); each survivor independently computes
  new $\phi'_i$ using the 6-permutation optimal assignment.
- **Deliberately breaks fully-local property:** state as paper contribution.

### 3.9 Test Scenarios

| ID | Configuration | Pass Criterion |
|----|--------------|----------------|
| RS-1 | Traverse, drone0 fault at $t=8$ s, $T_\text{trans}=5$ s | $T_\text{max}$ drops after reshaping complete |
| RS-2 | Lemniscate3d, sequential faults at 15 s and 25 s | $T_\text{max} \leq 100$ N at all times |
| RS-3 | Second fault during reshaping transition | Graceful fallback (abort, hover at current slots) |

---

## Cross-Extension Dependencies and Composition

```
E1 (L1 Adaptive)
  ↓ provides online m̂_L, k̂_eff
E2 (MPC) — uses m̂_L to update QP stage cost weights
            k̂_eff updates L_eff_body in tension linearisation

E3 (Reshaping) — independent of E1/E2 by design
               — after reshape, new slot geometry requires E2 to recompute
                 nominal tension vector n̂_i at new φ'_i (automatic if n̂_i
                 is recomputed from current slot each MPC step)

Composition E1 + E2 + E3:
  - L1 u_ad injected before MPC QP (same single-line change)
  - MPC tension constraint uses L_eff_body from E1's k̂_eff
  - Reshaping updates slot references that both E1 predictor and E2 use
  - No circular dependencies
```

### Interference Risks

1. **E1 + fault ($N$ changes):** $\sigma_\text{true}$ jumps when $N$ decreases. L1 re-converges
   in ~24 ms (well within 0.5 s fault transient).
2. **E2 tension constraint at fault onset:** Tension spikes are exactly what E2 prevents;
   slack variable $s_j \geq 0$ handles infeasibility at fault onset.
3. **E3 transition + E2 horizon:** During reshaping, the nominal trajectory (slots moving)
   causes non-zero $e_p$ by construction. The MPC treats this as a planned disturbance;
   the tension constraint remains active to prevent overload during the sweep.

---

## Implementation Order and Milestones

### Phase 1: E1 — L1 Adaptive (est. 2–3 days)

| Step | Task |
|------|------|
| 1.1 | Add 11 Params fields to header |
| 1.2 | Implement `CalcL1Update` (~60 lines) |
| 1.3 | Inject `u_ad` in `CalcControlForce` (1 line) |
| 1.4 | Add CLI flags |
| 1.5 | Run L1-1, L1-2, L1-3 test scenarios |
| 1.6 | Record results in README §8 |

### Phase 2: E2 — MPC (est. 5–7 days)

| Step | Task |
|------|------|
| 2.1 | Set up OSQP dependency in `CMakeLists.txt` |
| 2.2 | Implement `MpcLocalController` class (Phi, Omega pre-compute) |
| 2.3 | Implement tension constraint builder (§4.3 of theory doc) |
| 2.4 | Add CLI `--mpc-horizon` |
| 2.5 | Validate on S1 (no fault), S5 (single fault), S6 (triple fault) |
| 2.6 | Compute budget profiling (verify <0.6 ms at $N_p=10$) |

### Phase 3: E3 — Formation Reshaping (est. 4–5 days)

| Step | Task |
|------|------|
| 3.1 | Implement `FormationCoordinator` (input: fault_id, output: new slots for N drones) |
| 3.2 | Wire quintic transition engine |
| 3.3 | Add optional `new_slot_override_port` to controller header |
| 3.4 | Wire in `main.cc` behind `--reshaping-enabled` flag |
| 3.5 | Run RS-1, RS-2, RS-3 |
| 3.6 | Measure $T_\text{max}$ with and without reshaping on RS-2 (lemniscate3d) |

### Phase 4: Integration and Paper Results (est. 3–4 days)

| Step | Task |
|------|------|
| 4.1 | Compose E1 + E3 (L1 + reshaping): MC sweep on S5-equivalent scenarios |
| 4.2 | Compose E2 + E3 (MPC + reshaping): verify $T \leq 100$ N during transition |
| 4.3 | Generate publication figures (waterfall plots, tension time-series, Monte Carlo) |
| 4.4 | Update IEEE paper §4 (extensions) with verified numbers |

---

## Evidence Status — All Major Claims

| Claim | Source | Status |
|-------|--------|--------|
| L1 $A_m$ eigenvalues $-5.37, -18.63$ | Analytic | **Observed** |
| Lyapunov $P$ entries | Analytic | **Observed** |
| L1 $\sigma_\text{max} = 4.905$ m/s² | Analytic | **Observed** |
| L1 gain stability $\Gamma < 47\,500$ | Euler bound | **Supported Inference** |
| L1 $\mathcal{L}_1$ condition | Partial analytic | **Weak Inference** |
| MPC $T_\text{nom} = 8.23$ N at hover | Fixed-point iteration | **Observed** |
| MPC ZOH $B$ matrix | Analytic | **Observed** |
| MPC DARE $\|\lambda\| \approx 0.990$ | Algebraic (corrected from 0.980) | **Supported Inference** |
| MPC $L_\text{eff,body} = 1.448$ m | From hover fixed point | **Observed** |
| MPC tension spike timing 7 ms | Kinematic | **Supported Inference** |
| Reshaping $\Delta\phi_\text{max} = \pi/6$ | Minimax derivation | **Supported Inference** |
| Reshaping max slot speed 0.157 m/s | Quintic + $\pi/6$ | **Observed** |
| Reshaping collision safety 1.131 m > 0.5 m | Analytic | **Observed** |
| Reshaping $-25.8\%$ tension reduction | Quasi-static | **Weak Inference** |
| E1 + E2 + E3 composability | Structural analysis | **Supported Inference** |

---

## Open Risks and Missing Artifacts

| Risk | Impact | Mitigation |
|------|--------|-----------|
| L1 filter margin borderline ($\omega_c/\omega_n^z = 3.06$) | Possible oscillation at $\omega_c$ | Test with $\omega_c = 40$ rad/s; extend margin if needed |
| L1 post-fault re-convergence | $\hat\sigma$ may over-correct transiently | L1-3 test scenario specifically targets this |
| MPC OSQP not yet in build | E2 implementation blocked | Add OSQP CMake dependency before Phase 2 |
| MPC warm-start correctness | Sub-optimal solutions if not shifted correctly | Unit test shift logic separately |
| Reshaping $-25.8\%$ claim simulation-unverified | Paper claim unsupported | RS-2 scenario directly measures this |
| E3 second-fault during transition (RS-3) | Formation may be geometrically invalid | Fallback: freeze at current slot positions |
| No simulation data for any extension | All three designs are design-only | All three require implementation and verification before paper submission |

---

## Recommended Next Actions (Priority Order)

1. **[E1, P0]** Implement L1 adaptive loop — lowest risk, highest evidence gain per effort.
2. **[E3, P1]** Implement `FormationCoordinator` — strongest paper novelty claim; requires only
   standard C++ with no new dependencies.
3. **[E2, P2]** Add OSQP dependency and implement `MpcLocalController` — highest implementation
   effort but provides quantified tension safety guarantee.
4. **[All, P3]** Monte Carlo sweep over $m_L \in [2, 5]$ kg and $k_\text{seg} \in [12500, 37500]$
   N/m with E1 enabled to generate Figure F07 (adaptive robustness).
5. **[All, P4]** Joint MC sweep with E2 + E3: count tension violations with/without extensions
   to generate the key safety table for the IEEE paper.

---

*See individual theory docs for full equations, pseudocode, and per-extension checklists.*
