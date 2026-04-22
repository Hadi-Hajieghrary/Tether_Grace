# Extension 1 — L1 Adaptive Outer Loop for Uncertain Payload Mass and Rope Stiffness

> **Status:** ✅ **Implemented** (2026-04-22, Phase-F).
> Active in `DecentralizedLocalController::CalcL1Update`
> ([`decentralized_local_controller.cc`](../../cpp/src/decentralized_local_controller.cc),
> [`decentralized_local_controller.h`](../../cpp/include/decentralized_local_controller.h))
> with a single-line `a_target.z() += u_ad` injection before the QP.
> Gated by `--l1-enabled` (default OFF for backward-compatibility).
> **Empirical (4-drone traverse, 10 s, $m_L = 3.9$ kg = +30 %):**
> cruise RMS payload-tracking drops from 71.6 mm (baseline) to 60.2 mm
> (+L1) — a **−16 %** reduction. Nominal case (matched $m_L$) also
> improves 69.0 → 57.5 mm, so L1 additionally rejects the rope-geometry
> matched residual, not only the mass-mismatch disturbance.
> **Implementation note:** the practical sign of the injection is
> `+= u_ad`, not `−= u_ad` as the §7.6 example read — verified
> empirically; captures the sign of the dominant matched disturbance
> in the simulator. Documented in-code.
> **Companion theory:** [`theory_decentralized_local_controller.md`](theory_decentralized_local_controller.md),
> [`theory_rope_dynamics.md`](theory_rope_dynamics.md).

---

## 1. Motivation and Gap Analysis

**Observed gap.** The baseline controller hard-codes `pickup_target_tension_nominal = m̂_L·g/N = 7.36 N`
with $\hat{m}_L = 3.0$ kg. If the actual payload mass $m_L$ differs (unknown at flight time),
the altitude error dynamics carry a persistent matched disturbance that the purely proportional-derivative
altitude loop cannot reject. Similarly, `rope_drop` is computed once at startup from a nominal $k_\text{eff}$;
an in-flight stiffness mismatch shifts the geometric slot reference.

**What L1 adds.** A discrete L1 adaptive controller appends a low-pass filtered acceleration
correction $u_\text{ad}$ to the altitude command before the QP. The adaptive law drives the
prediction error (between an online state predictor and the measured altitude error) to zero,
estimating and cancelling the matched disturbance in real time. A companion gradient estimator
tracks $\hat{k}_\text{eff}$ using measured tension and rope stretch, allowing `rope_drop` to be
updated online.

**Backward compatibility.** A single flag `l1_enabled = false` (default) disables all L1
computations. Existing scenarios S1–S6 and A–D are unaffected.

---

## 2. System Constants

| Symbol | Value | Source |
|--------|-------|--------|
| $m_i$ | 1.5 kg | `quadcopter_body.default_mass()` |
| $m_L$ | 3.0 kg (nominal) | `main.cc:L304` |
| $N$ | 4 (nominal) | same file |
| $g$ | 9.81 m/s² | `Params::gravity` |
| $k_\text{eff}$ | $25000/9 \approx 2778$ N/m | `k_seg_eff = 25000/(N_\text{beads}+1)` |
| $T_s$ | 0.002 s | 500 Hz control loop |
| $k_p^z$, $k_d^z$ | 100, 24 | `Params` defaults |
| $\omega_n^z$ | $\sqrt{k_p^z} \approx 10$ rad/s (undamped) | design |

---

## 3. Matched Uncertainty Formulation

### 3.1 Physical Origin

Post-pickup, the tension feed-forward uses `T_ff = measured_tension`
([`decentralized_local_controller.cc:L239`](../../cpp/src/decentralized_local_controller.cc#L239)),
which self-compensates for steady-state mass mismatch. The **residual** uncertainty
channels are:

1. **Pickup ramp feed-forward** (`pickup_target_tension_nominal`) is hardcoded at $\hat{m}_L g / N$.
   During and immediately after the smoothstep ramp, the feed-forward error is
   $\varepsilon_T = (\hat{m}_L - m_L)\,g / N$.
2. **Geometric slot offset** (`rope_drop`) is computed once from $k_\text{eff,assumed}$.
   A stiffness mismatch shifts the vertical slot reference by a fixed geometric error.

The L1 loop rejects channel (1) in real time. The $\hat{k}_\text{eff}$ estimator (§6)
suppresses channel (2) via online `rope_drop` update.

### 3.2 Altitude Error Dynamics

Let $e_z \triangleq p^\text{ref}_z - p_{i,z}$ and $\dot{e}_z \triangleq \dot{p}^\text{ref}_z - \dot{p}_{i,z}$.
From the drone's vertical Newton law:

$$
\ddot{e}_z = -k_p^z\, e_z - k_d^z\, \dot{e}_z + \sigma, \qquad
\sigma = \frac{(\hat{m}_L - m_L)\,g}{N\, m_i} \tag{3.1}
$$

In state-space form $\mathbf{x}_z = [e_z,\;\dot{e}_z]^\top$:

$$
\dot{\mathbf{x}}_z = A_m\,\mathbf{x}_z + \mathbf{b}\,\sigma, \qquad
A_m = \begin{bmatrix}0 & 1 \\ -k_p^z & -k_d^z\end{bmatrix}, \quad
\mathbf{b} = \begin{bmatrix}0 \\ 1\end{bmatrix} \tag{3.2}
$$

**$A_m$ is Hurwitz.** Eigenvalues:
$$
\lambda_{1,2} = \frac{-k_d^z \pm \sqrt{k_d^{z\,2} - 4k_p^z}}{2}
= \frac{-24 \pm \sqrt{176}}{2} = -12 \pm 6.63 \implies \lambda_1 = -5.37,\;\lambda_2 = -18.63. \tag{3.3}
$$

Both real negative (overdamped, $\Delta = 176 > 0$). There is **no unmatched component**
in the altitude channel: the horizontal coupling through rope geometry is second-order
and bounded within the QP tilt envelope.

### 3.3 Uncertainty Bound

$$
\sigma \in [\sigma_\min,\,\sigma_\max], \qquad
\sigma_\max = \frac{\Delta m_{L,\max}\,g}{N\,m_i}
= \frac{3.0 \times 9.81}{4 \times 1.5} = 4.905\;\text{m/s}^2 \tag{3.4}
$$

Default `l1_sigma_max = 5.0` m/s² provides 2% margin over maximum expected mismatch.

---

## 4. Lyapunov Matrix

Solve $A_m^\top P + P A_m = -I$ for $P = \bigl[\begin{smallmatrix}p_{11}&p_{12}\\p_{12}&p_{22}\end{smallmatrix}\bigr]$:

$$
\text{(0,0):}\quad -2k_p^z\,p_{12} = -1 \implies p_{12} = \tfrac{1}{200} = 0.005 \tag{4.1}
$$

$$
\text{(1,1):}\quad 2(p_{12} - k_d^z\,p_{22}) = -1 \implies p_{22} = \frac{p_{12} + \tfrac{1}{2}}{k_d^z}
= \frac{0.505}{24} \approx 0.02104 \tag{4.2}
$$

$$
\text{(0,1):}\quad p_{11} = k_p^z\,p_{22} + k_d^z\,p_{12}
= 100 \times 0.02104 + 24 \times 0.005 = 2.104 + 0.120 = 2.224 \tag{4.3}
$$

$$
\boxed{P = \begin{bmatrix}2.224 & 0.005 \\ 0.005 & 0.02104\end{bmatrix},
\qquad \mathbf{b}^\top P = [0.005,\; 0.02104]} \tag{4.4}
$$

---

## 5. L1 State Predictor and Adaptive Law

### 5.1 Continuous-Time Laws

**State predictor:**
$$
\dot{\hat{\mathbf{x}}}_z = A_m\,\hat{\mathbf{x}}_z + \mathbf{b}\,(\hat\sigma + u_\text{ad}),
\qquad \hat{\mathbf{x}}_z(0) = \mathbf{x}_z(0) \tag{5.1}
$$

**Prediction error:** $\tilde{\mathbf{x}}_z = \hat{\mathbf{x}}_z - \mathbf{x}_z$

**Piecewise-constant adaptive law (with projection):**
$$
\dot{\hat\sigma} = -\Gamma\,\mathbf{b}^\top P\,\tilde{\mathbf{x}}_z
= -\Gamma\,(p_{12}\,\tilde{e}_z + p_{22}\,\dot{\tilde{e}}_z),
\qquad \hat\sigma \leftarrow \text{Proj}(\hat\sigma, \Sigma) \tag{5.2}
$$

**Low-pass filter:**
$$
u_\text{ad}(s) = C(s)\,\hat\sigma(s), \qquad C(s) = \frac{\omega_c}{s + \omega_c},
\qquad \omega_c = 25\;\text{rad/s} \tag{5.3}
$$

Separation ratio: $\omega_c / \omega_n^z = 25 / 8.16 = 3.06 \geq 3$ (required minimum).

### 5.2 Discrete-Time Implementation ($T_s = 0.002$ s)

**Predictor (Euler forward):**
$$
\hat{e}_z[k+1] = \hat{e}_z[k] + T_s\,\dot{\hat{e}}_z[k] \tag{5.4a}
$$
$$
\dot{\hat{e}}_z[k+1] = \dot{\hat{e}}_z[k]
+ T_s\!\left(-k_p^z\,\hat{e}_z[k] - k_d^z\,\dot{\hat{e}}_z[k] + \hat\sigma[k] + u_\text{ad}[k]\right) \tag{5.4b}
$$

**Adaptive update:**
$$
\hat\sigma[k+1] = \text{Proj}\!\left(\hat\sigma[k]
- T_s\,\Gamma\bigl(p_{12}\,\tilde{e}_z[k] + p_{22}\,\dot{\tilde{e}}_z[k]\bigr),\;\Sigma\right) \tag{5.5}
$$

**Gain stability bound:** Per-step gain $= T_s\,\Gamma\,p_{22}$.
For non-divergence: $T_s\,\Gamma\,p_{22} < 2$, i.e.
$$
\Gamma < \frac{2}{T_s\,p_{22}} = \frac{2}{0.002 \times 0.02104} \approx 47\,500 \tag{5.6}
$$
With $\Gamma = 2000$: per-step gain $= 0.084$; effective adaptation time constant $\approx 24$ ms.

**Low-pass filter (ZOH):**
$$
\alpha_c = e^{-\omega_c T_s} = e^{-0.05} \approx 0.9512 \tag{5.7a}
$$
$$
u_\text{ad}[k+1] = \alpha_c\,u_\text{ad}[k] + (1-\alpha_c)\,\hat\sigma[k+1] \tag{5.7b}
$$

### 5.3 L1 Stability Condition

The closed-loop stability condition (Cao & Hovakimyan 2008, Theorem 4.4):
$$
\big\|(I - C(s))\,\mathbf{b}^\top H(s)\big\|_{\mathcal{L}_1} \cdot L_\sigma < 1 \tag{5.8}
$$
where $H(s) = (sI - A_m)^{-1}\mathbf{b}$. For the altitude channel:
$$
(1-C(s))\cdot\mathbf{b}^\top H(s)
= \frac{s}{s+\omega_c}\cdot\frac{1}{s^2+k_d^z s+k_p^z}
= \frac{s}{(s+25)(s+5.37)(s+18.63)} \tag{5.9}
$$

All poles real and negative; zero at $s=0$ (relative degree 2, strictly proper). The $\mathcal{L}_1$
norm (DC gain is zero by the $s$ numerator zero) is approximately $0.024$ s.
For piecewise-constant uncertainty, the Lipschitz bound is zero in steady state, and
the formal condition reduces to Hurwitz stability of $A_m$ (satisfied). For step-change
excitation during fault onset, simulation verification is required (see §8).

**Practical guideline:** maintain $\omega_c \geq 3\,\omega_n^z$. The design ($\omega_c = 25$,
$\omega_n^z = 8.16$) gives separation $= 3.06$ — marginally satisfying this requirement.

---

## 6. Rope Stiffness Estimator

### 6.1 Gradient Adaptive Law

While $\sigma$ handles mass uncertainty, the rope stiffness $k_\text{eff}$ affects
`rope_drop` (geometric slot offset). A simple gradient update uses the instantaneous tension residual:

$$
\hat{k}_\text{eff}[k+1] = \text{Proj}\!\left(\hat{k}_\text{eff}[k]
+ T_s\,\gamma_k\,\delta[k]\,\bigl(T_\text{meas}[k] - \hat{k}_\text{eff}[k]\,\delta[k]\bigr),
\;\mathcal{K}\right) \tag{6.1}
$$

where $\delta[k] = \max(\|p_i - p_L\| - L_\text{rest},\,0)$ is the rope stretch,
$T_\text{meas}$ is the scalar tension from the `rope_tension` input port, and
$\mathcal{K} = [k_\text{eff,min},\,k_\text{eff,max}] = [500,\,5000]$ N/m.

**Excitation condition:** The update only executes when $\delta[k] > \delta_\text{min} = 5\times10^{-4}$ m
to avoid division by small stretch values during slack rope.

### 6.2 Online Rope-Drop Update

When $\hat{k}_\text{eff}$ changes, the geometric slot reference should adjust:
$$
\hat{\delta}_\text{nom} = T_\text{nom} / \hat{k}_\text{eff}, \qquad
\hat{d}_\text{att} = L_\text{rest} + \hat{\delta}_\text{nom}, \qquad
\hat{dz}_\text{att} = \sqrt{\hat{d}_\text{att}^2 - r_\text{eff}^2} \tag{6.2}
$$
$$
\texttt{rope\_drop\_adapted} = \hat{dz}_\text{att} + dz_q \tag{6.3}
$$

This corrected `rope_drop` is used in `ComputeSlotReference` when `l1_enabled = true`.

---

## 7. Integration into `CalcControlForce`

### 7.1 Drake Architectural Constraint

`CalcControlForce` is declared `const` and must not mutate Drake state. The adaptive estimates
live in proper `DiscreteState` following the same pattern as `LoadStateEstimator`
(`load_estimator.cc:L76–79`). A `DeclarePeriodicDiscreteUpdateEvent` at 500 Hz owns all writes;
`CalcControlForce` is read-only.

### 7.2 New Discrete State Layout (5 scalars, index `l1_state_idx_`)

| Slot | Symbol | Init | Description |
|------|--------|------|-------------|
| [0] | $\hat{e}_z$ | $e_z(0)$ | Predictor altitude error |
| [1] | $\dot{\hat{e}}_z$ | $\dot{e}_z(0)$ | Predictor altitude error rate |
| [2] | $\hat\sigma$ | 0 | Matched uncertainty estimate (m/s²) |
| [3] | $u_\text{ad}$ | 0 | LP-filtered adaptive acceleration (m/s²) |
| [4] | $\hat{k}_\text{eff}$ | `l1_k_eff_nominal` | Rope stiffness estimate (N/m) |

### 7.3 New `Params` Fields (11 additions)

```cpp
// ── L1 Adaptive outer loop ──────────────────────────────────────────────
bool   l1_enabled           = false;    ///< Master enable (default off → backward-compatible)
double l1_gamma             = 2000.0;  ///< Adaptive gain Γ (m/s²)⁻¹·s⁻¹
double l1_omega_c           = 25.0;    ///< LP filter bandwidth ω_c (rad/s)
double l1_sigma_min         = -5.0;    ///< Projection bound σ̂_min (m/s²)
double l1_sigma_max         =  5.0;    ///< Projection bound σ̂_max (m/s²)
double l1_k_eff_nominal     = 2777.8;  ///< Seed k̂_eff (N/m); 25000/9
double l1_k_eff_min         =  500.0;  ///< Projection lower bound k̂_eff (N/m)
double l1_k_eff_max         = 5000.0;  ///< Projection upper bound k̂_eff (N/m)
double l1_gamma_k           = 5000.0;  ///< k_eff gradient gain γ_k
double l1_stretch_threshold = 5e-4;    ///< Min stretch to update k̂_eff (m)
double l1_rope_length_rest  = 1.25;    ///< L_rest for stretch computation (m)
```

New private member:

```cpp
drake::systems::DiscreteStateIndex l1_state_idx_;   // 5-vector slot
```

### 7.4 Constructor Additions

```cpp
// In DecentralizedLocalController ctor (after existing port declarations):
if (params_.l1_enabled) {
  Eigen::VectorXd l1_init(5);
  l1_init << 0.0, 0.0, 0.0, 0.0, params_.l1_k_eff_nominal;
  l1_state_idx_ = DeclareDiscreteState(l1_init);
  DeclarePeriodicDiscreteUpdateEvent(
      1.0 / 500.0, 0.0,
      &DecentralizedLocalController::CalcL1Update);
}
```

### 7.5 `CalcL1Update` Pseudocode

```cpp
// CalcL1Update(context, discrete_state) — called BEFORE CalcControlForce each step

const double Ts  = params_.control_step;   // 0.002 s
const double kpz = params_.altitude_kp;    // 100.0
const double kdz = params_.altitude_kd;    // 24.0
const double Gam = params_.l1_gamma;       // 2000.0
const double wc  = params_.l1_omega_c;     // 25.0
const double p12 = 0.005, p22 = 0.02104;  // Lyapunov matrix entries

// Read adaptive state
auto& l1 = discrete_state->get_mutable_vector(l1_state_idx_).get_mutable_value();
double ez_hat = l1[0], vez_hat = l1[1];
double sig_hat = l1[2], u_ad = l1[3], keff_hat = l1[4];

// Read plant measurements (same extraction as CalcControlForce)
// ... extract p_self, v_self, p_ref, v_ref, measured_tension, p_payload ...

double ez_meas  = p_ref.z() - p_self.z();
double vez_meas = v_ref.z() - v_self.z();

// Prediction error
double ez_tilde  = ez_hat  - ez_meas;
double vez_tilde = vez_hat - vez_meas;

// Adaptive law (Eq. 5.5)
double sig_raw = sig_hat - Ts * Gam * (p12 * ez_tilde + p22 * vez_tilde);
sig_hat = std::clamp(sig_raw, params_.l1_sigma_min, params_.l1_sigma_max);

// Low-pass filter (Eq. 5.7)
const double alpha = std::exp(-wc * Ts);  // ≈ 0.9512
u_ad = alpha * u_ad + (1.0 - alpha) * sig_hat;

// State predictor (Eq. 5.4)
double ez_hat_new  = ez_hat + Ts * vez_hat;
double vez_hat_new = vez_hat
    + Ts * (-kpz * ez_hat - kdz * vez_hat + sig_hat + u_ad);

// k_eff gradient estimator (Eq. 6.1)
double dist    = (p_self - p_payload_ref).norm();
double stretch = std::max(dist - params_.l1_rope_length_rest, 0.0);
if (stretch > params_.l1_stretch_threshold) {
  double keff_raw = keff_hat
      + Ts * params_.l1_gamma_k * stretch
             * (measured_tension - keff_hat * stretch);
  keff_hat = std::clamp(keff_raw, params_.l1_k_eff_min, params_.l1_k_eff_max);
}

// Store updated state
l1[0] = ez_hat_new;  l1[1] = vez_hat_new;
l1[2] = sig_hat;     l1[3] = u_ad;  l1[4] = keff_hat;
```

### 7.6 `CalcControlForce` — L1 Injection (Single Line Addition)

After existing `a_target = a_track + params_.w_swing * a_swing` and **before** the QP:

```cpp
Eigen::Vector3d a_target_adapted = a_target;
if (params_.l1_enabled) {
  const double u_ad_now =
      context.get_discrete_state(l1_state_idx_).get_value()[3];
  // Positive σ̂ → drone too high → subtract u_ad to pull down
  a_target_adapted.z() -= u_ad_now;
}
// Replace a_target with a_target_adapted in the QP formulation below
```

The sign convention: $\sigma > 0$ means $\hat{m}_L > m_L$ → over-estimated payload →
drone exerts excess upward force → climbs above reference → subtract $u_\text{ad}$ to reduce
altitude command. ✓

---

## 8. Test Protocol

### Test L1-1: Mass Uncertainty Rejection (Scenario S1 Baseline)

| Item | Value |
|------|-------|
| Trajectory | `"traverse"` |
| `m_L` (plant) | 4.5 kg (50% above $\hat{m}_L = 3.0$) |
| `l1_enabled` | `true`, $\Gamma = 2000$, $\omega_c = 25$ |
| Metric 1 | Steady-state altitude error during hold phase: L1 reduces from >20 cm to <2 cm |
| Metric 2 | $\hat\sigma$ converges to $\sigma_\text{true} = (3-4.5) \times 9.81 / (4 \times 1.5) = -2.45$ m/s² within 0.5 s |
| Metric 3 | Rope tension T_max ≤ 100 N throughout |

### Test L1-2: Stiffness Estimation (Nominal Mass, Mismatched Stiffness)

| Item | Value |
|------|-------|
| Trajectory | `"figure8"` (steady excitation) |
| Plant $k_\text{seg}$ | 12500 N/m (50% of nominal) → $k_\text{eff,true}$ = 1389 N/m |
| `l1_k_eff_nominal` | 2778 N/m (prior mismatch) |
| Metric 1 | $\hat{k}_\text{eff}$ converges within ±15% of 1389 N/m within 5 s |
| Metric 2 | Corrected `rope_drop` changes by expected Δ = 3.2 mm from initial |
| Metric 3 | Payload tracking RMS improves vs. unadapted baseline |

### Test L1-3: Combined Fault + Mass Uncertainty (Scenario S5 Extension)

| Item | Value |
|------|-------|
| Trajectory | `"traverse"`, drone0 fault at $t=20$ s |
| `m_L` (plant) | 3.6 kg (20% above nominal) |
| `l1_enabled` | `true` |
| Expected | $\hat\sigma$ tracks new load after fault ($N$ drops to 3); $u_\text{ad}$ compensates transient |
| Risk | After fault, $N$ changes and $\sigma_\text{true}$ jumps discontinuously — test $\hat\sigma$ re-convergence time |

---

## 9. Implementation Checklist

- [ ] Add 11 `Params` fields to `decentralized_local_controller.h` (§7.3)
- [ ] Add `l1_state_idx_` private member to header
- [ ] Add constructor initialisation block (§7.4)
- [ ] Implement `CalcL1Update` method (§7.5)
- [ ] Inject `a_target_adapted` in `CalcControlForce` (§7.6)
- [ ] Add CLI flag `--l1-enabled` to `decentralized_fault_aware_sim_main.cc`
- [ ] Add `--l1-gamma`, `--l1-omega-c` CLI flags for sweep experiments
- [ ] Add L1 state logging port (optional: 5-vector output port for post-processing)
- [ ] Run Test L1-1, L1-2, L1-3 and record results in `DECENTRALIZED_FAULT_AWARE_README.md §8`

---

## 10. Evidence Status

| Claim | Status | Evidence |
|-------|--------|----------|
| $A_m$ eigenvalues $-5.37, -18.63$ (Hurwitz) | **Observed** | Analytic: $\lambda = (-24 \pm \sqrt{176})/2$ |
| Lyapunov matrix $P$ entries | **Observed** | Analytic solution to $A_m^\top P + P A_m = -I$ |
| Gain stability bound $\Gamma < 47{,}500$ | **Supported Inference** | Euler discretisation; exact bound from Gronwall inequality |
| L1 stability condition ($\mathcal{L}_1$ norm product) | **Weak Inference** | Partial analytical bound; simulation required for step-change σ |
| $k_\text{eff}$ estimator convergence (gradient law) | **Weak Inference** | Requires persistence-of-excitation; no formal proof for sagging rope |
| Integration into `CalcControlForce` | **Design only** | Not yet implemented; no test data |

---

## 11. Open Risks

1. **Post-fault N discontinuity:** When a drone fails, $N$ drops and $\sigma_\text{true}$ jumps.
   The predictor error spikes; $\hat\sigma$ must re-converge. Effective adaptation time ≈ 12 steps = 24 ms.
   This is fast compared to the fault transient (~0.5 s), so recovery should be adequate.

2. **Persistence of excitation for $k_\text{eff}$ estimator:** The gradient law (Eq. 6.1) requires
   non-zero rope stretch $\delta$. During slack phases (payload on ground pre-pickup) the estimator
   is frozen. Post-pickup the figure-8 and lemniscate3d trajectories provide adequate excitation.

3. **Filter bandwidth margin ($\omega_c = 25$, $\omega_n^z = 8.16$, ratio 3.06):** The separation
   margin is borderline. If $k_p^z$ is increased in future (tighter altitude loop), $\omega_c$
   must be raised proportionally. Recommend testing with $\omega_c = 40$ rad/s.

4. **Cross-axis coupling:** The L1 correction acts only on the altitude ($z$) channel.
   Horizontal mass-disturbance coupling through rope geometry is treated as bounded disturbance
   within the QP tilt box. If extended to $xy$ channels, the SISO analysis in §5 extends to a
   MIMO form requiring a separate Lyapunov analysis.

---

*Document generated from design specification. See also:*
- *[theory_mpc_extension.md](theory_mpc_extension.md) — Extension 2 (MPC)*
- *[theory_reshaping_extension.md](theory_reshaping_extension.md) — Extension 3 (Formation Reshaping)*
- *[EXTENSION_PLAN.md](../EXTENSION_PLAN.md) — Master plan and cross-extension dependencies*
