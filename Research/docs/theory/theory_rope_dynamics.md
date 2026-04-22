# Theory — Rope (Bead-Chain Spring-Damper) Model

> Companion to the rope physics implemented in
> [`Tether_Lift/Research/cpp/src/rope_force_system.cc`](../../../Tether_Lift/Research/cpp/src/rope_force_system.cc)
> and the parameter declarations in the harness
> [`Research/cpp/src/decentralized_fault_aware_sim_main.cc`](../../cpp/src/decentralized_fault_aware_sim_main.cc).

## 1. Summary

Each rope connecting a drone to the payload is modelled as a chain of
$N_{\text{beads}} = 8$ intermediate point-mass beads connected by
$N_{\text{seg}} = N_{\text{beads}} + 1 = 9$ tension-only
Kelvin–Voigt spring-damper segments. Tension-only means the segment
exerts a restoring force only when its current length exceeds its rest
length; when slack, it exerts zero force. The model reproduces the
dominant dynamic behaviour of a real braided-polymer sling — static
stretch, pendulum swing, catenary sag, and the brief transient when a
formerly slack rope goes taut — without the numerical stiffness of
an inextensible constraint. Parameters are chosen to match a
**6 mm aramid-core (Dyneema / Technora) lift sling** with effective
end-to-end stiffness $EA/L \approx 2.8$ kN/m.

> **Notation:** $N_{\text{beads}} = 8$ denotes the number of intermediate
> point-mass beads; $N_{\text{seg}} = N_{\text{beads}} + 1 = 9$ denotes
> the number of spring-damper segments between the 10 total attachment
> nodes (1 drone + 8 beads + 1 payload). The code uses `num_rope_beads`
> and `num_rope_segments` respectively (`main.cc:L325, L500`).

## 2. Geometry and state

A rope connects drone body $B_0$ (attachment point $\mathbf{a}_0$ in
drone frame) to payload body $B_{N_{\text{seg}}}$ (attachment point
$\mathbf{a}_{N_{\text{seg}}}$ in payload frame), threaded through
$N_{\text{beads}} = 8$ intermediate bead bodies
$B_1,\dots,B_{N_{\text{beads}}}$. Each bead has

- mass $m_b = 25\text{ g}$,
- inertia $J_b = \frac{2}{5} m_b r_b^2 I_{3\times 3}$ (solid sphere, $r_b = 0.012$ m),
- 6-DOF floating base (so the plant's generalised state includes each
  bead's pose).

Let $\mathbf{x}_k$, $k = 0,\ldots,N_{\text{seg}}$, denote the world
positions of the $N_{\text{seg}} + 1 = 10$ attachment points (drone,
$N_{\text{beads}} = 8$ beads, payload). The $k$-th segment connects
$\mathbf{x}_{k-1}$ to $\mathbf{x}_k$, has rest length

$$
L_{\text{seg}} = \frac{L}{N_{\text{seg}}} = \frac{1.25}{9} \approx 0.139\text{ m}, \qquad L = 1.25\text{ m}, \tag{1}
$$

and instantaneous length $\ell_k = \|\mathbf{x}_k - \mathbf{x}_{k-1}\|$.

## 3. Force law per segment

Let $\hat{\mathbf{u}}_k = (\mathbf{x}_k - \mathbf{x}_{k-1})/\ell_k$ be
the unit vector along segment $k$, and
$\dot{\ell}_k = \hat{\mathbf{u}}_k^{\!\top}(\dot{\mathbf{x}}_k - \dot{\mathbf{x}}_{k-1})$
its rate of change. Define the stretch
$\Delta_k = \ell_k - L_{\text{seg}}$. The tension carried by segment
$k$ is

$$
\boxed{ \;
T_k \;=\; \max\!\Big(\,0,\; k_s\,\Delta_k \;+\; c_s\,\dot{\ell}_k \,\Big) ,
\;}\tag{2}
$$

with $k_s = 25\,000$ N/m, $c_s = 60$ N·s/m — the tension-only clamp
captures the fundamental physical property that a cable cannot push.
The force the segment exerts on its two endpoints is

$$
\mathbf{F}_{\text{on }\mathbf{x}_{k-1}} = +T_k\,\hat{\mathbf{u}}_k, \qquad
\mathbf{F}_{\text{on }\mathbf{x}_{k}}   = -T_k\,\hat{\mathbf{u}}_k . \tag{3}
$$

The drone-side rope-tension signal used by the controller is
$T_i \equiv T_1$ (the top segment). Source implementation:
[`rope_force_system.cc`](../../../Tether_Lift/Research/cpp/src/rope_force_system.cc).

## 4. End-to-end behaviour

A rope under static tension $T$ (well below its elastic limit) stretches
by $\Delta L \approx T / k_{\text{rope}}^{\text{eff}}$. For a chain of
$N_{\text{seg}}$ identical Kelvin–Voigt springs in **series**, the
effective end-to-end stiffness is

$$
k_{\text{rope}}^{\text{eff}} \;=\; \frac{k_s}{N_{\text{seg}}} \;=\; \frac{k_s}{N_{\text{beads}}+1} \;=\; \frac{25\,000}{9} \;\approx\; 2\,778\text{ N/m}. \tag{4}
$$

> **Code:** `k_seg_eff = segment_stiffness / (num_rope_beads + 1)`
> at `decentralized_fault_aware_sim_main.cc:L372`; confirmed by
> `num_rope_segments = num_rope_beads + 1` at `main.cc:L500`.

At cruise, the per-rope static tension is approximately
$m_L g / N = 3 \cdot 9.81 / 4 \approx 7.4$ N, giving a static stretch of

$$
\Delta L_{\text{hover}} \;\approx\; \frac{7.4}{2778} \;\approx\; 2.7\text{ mm} ,
$$

i.e. **effectively rigid** at cruise (match with real low-creep
aramid-core sling behaviour).

The effective damping coefficient end-to-end follows the same series
rule, $c_{\text{rope}}^{\text{eff}} = c_s / N_{\text{seg}} = 60/9 \approx 6.7$ N·s/m.
The damping ratio of a segment (bead vs. its own spring-damper) is

$$
\zeta \;=\; \frac{c_s}{2\sqrt{k_s\,m_b}} \;=\; \frac{60}{2\sqrt{25\,000\cdot 0.025}} \;\approx\; 1.2 , \tag{5}
$$

i.e. **slightly over-damped**, so bead oscillations decay monotonically
without ringing.

## 5. Numerical stability of the explicit integrator

The simulation uses Drake's discrete-time `MultibodyPlant` with
$\Delta t = 2\times 10^{-4}$ s and an implicit-Euler integration step.
The fastest dynamic mode of the bead chain is the single-segment
oscillation,

$$
\omega_n^{\text{seg}} \;=\; \sqrt{\frac{k_s}{m_b}} \;=\; \sqrt{\frac{25\,000}{0.025}} \;\approx\; 10^3\text{ rad/s}, \tag{6}
$$

corresponding to a natural period of $2\pi/\omega_n^{\text{seg}} \approx
6.3$ ms. Stability of an explicit-Euler step requires
$\Delta t \lesssim 2/\omega_n^{\text{seg}} \approx 2$ ms. Drake's
implicit-Euler allows somewhat larger steps, but the chosen
$\Delta t = 0.2$ ms leaves a **~10× margin**, which is the budget used
to absorb the stiffer transient when a rope suddenly goes taut.

## 6. Static hover equilibrium — rope_drop derivation

The physical rope does not run between body centres but between *attachment
points*: the drone attachment is $|q_{\text{attach}}|=9$ cm below the
drone body centre; the payload attachment is $0.3\,r_f$ in from the
payload body centre (same radial direction). This gives effective
attachment-frame dimensions

$$
r_{\text{eff}} \;=\; 0.7\,r_f \;=\; 0.56\text{ m},
\qquad
\Delta z_{\text{attach}} \;=\; z_d - z_L - |q_{\text{attach}}|. \tag{7}
$$

At static hover, the rope chord must exceed its rest length $L$ by a
small elastic stretch $\delta$ that carries the payload's share of
gravity through the rope:

$$
T \;=\; \frac{m_L g\,L_{\text{chord}}}{N\,\Delta z_{\text{attach}}},
\qquad
\delta \;=\; \frac{T}{k_{\text{eff}}},
\qquad
L_{\text{chord}} \;=\; L + \delta,
\qquad
\Delta z_{\text{attach}} \;=\; \sqrt{L_{\text{chord}}^2 - r_{\text{eff}}^2}. \tag{7a}
$$

Fixed-point iteration with $k_{\text{eff}}=k_s/N_{\text{seg}}=
25\,000/9\approx 2778$ N/m, $N=4$, $m_L=3$ kg, $r_f=0.8$ m, $L=1.25$ m
converges (in under five iterations) to
$\delta\approx 3.0$ mm, $T\approx 8.2$ N, and
$\Delta z_{\text{attach}}\approx 1.120$ m, giving the drone-centre-to-
payload-centre vertical offset that the controller uses for its slot
reference

$$
\mathrm{rope\_drop} \;=\; \Delta z_{\text{attach}} + |q_{\text{attach}}|
\;\approx\; 1.120 + 0.09
\;=\; 1.211 \text{ m}. \tag{8}
$$

This hover-equilibrium solution is computed online at sim start-up at
[`decentralized_fault_aware_sim_main.cc:L360–389`](../../cpp/src/decentralized_fault_aware_sim_main.cc#L360)
and simultaneously used to (i) set the controller's `rope_drop`
parameter and (ii) spawn each drone at the static fixed point so that
the payload is suspended in air with the bead chain pre-tensioned at
$T\approx 8$ N from $t=0$ (no snap-taut, no pickup free-fall).

## 7. Parameter choice rationale

The three knobs $k_s, c_s, N_{\text{seg}}$ are chosen so that:

| Requirement | Satisfied by |
|---|---|
| Static stretch ≤ 5 mm at nominal load | $k_{\text{rope}}^{\text{eff}} = k_s/9 \approx 2.78$ kN/m → 2.7 mm ✓ |
| No oscillatory ringing after a rope-taut transient | $\zeta \approx 1.2$ ✓ |
| Realistic material match (aramid-core 6 mm sling) | $EA/L \approx 3$ kN/m is the advertised dynamic-loading figure of Dyneema/Technora braided ropes of that gauge ✓ |
| Explicit-integrator stable at $\Delta t = 0.2$ ms | $\omega_n^{\text{seg}} \approx 10^3$ rad/s gives a 10× margin ✓ |
| Catenary drape within ~30 % of the ideal taut-rope geometry | $N_{\text{beads}} = 8$ intermediate beads ($N_{\text{seg}} = 9$ segments) is the empirical sweet spot (earlier experiments with $N_{\text{beads}} = 4$ were too rigid, $N_{\text{beads}} = 16$ too soft) |

Earlier phases of the project used $k_s = 200$ N/m (far too compliant —
30 cm static stretch, 10 Hz rope-bending oscillations that hid fault
transients) and $k_s = 2\,000$ N/m (still too soft). The final 25 kN/m
setting was the result of explicit calibration against the observed
payload-bounce frequency — details in the project cleanup notes.

## 8. Cable-severance semantics

A severed cable is modelled by **two simultaneous gating actions**:

1. **`CableFaultGate`** ([`cable_fault_gate.h`](../../cpp/include/cable_fault_gate.h))
   replaces the rope's full spatial-force vector with zero for
   $t \ge t_{\text{fault}}$. This removes the rope from the
   multibody-plant dynamics. The beads continue to exist as inert
   bodies (they fall to the ground under gravity).
2. **`TensionFaultGate`** ([`cable_fault_gate.h`](../../cpp/include/cable_fault_gate.h))
   zeroes the 4-D tension-telemetry output for the same drone, so that
   the scalar tension signal read by the drone's controller is
   exactly 0 after $t_{\text{fault}}$. This is the mechanism by which
   the failed drone's own feed-forward collapses to zero gravity-only
   thrust, without any peer signalling.

See [`theory_fault_model.md`](theory_fault_model.md) for the full
fault-response behaviour.

## 8bis. Rope-length uncertainty — baseline vs active harness

The Tether_Lift baseline (`Tether_Lift/Research/cpp/src/main.cc:230–276`)
samples each rope's rest length from an independent Gaussian at
simulation-configuration time,
$L_i \sim \mathcal{N}(\mu_i, \sigma_i^2)$, with per-drone
$(\mu_i, \sigma_i)$ fixed at 1.0 ± 0.05 m, 1.1 ± 0.08 m, 0.95 ± 0.06 m
for the 3-drone default. The sampled value is what enters
`RopeForceSystem`. Neither the baseline's `QuadcopterLiftController`
nor its `DecentralizedLoadEstimator` sees the sampled length — the
estimator accepts only the nominal mean, so the baseline is **blind**
to rope-length uncertainty.

The Tether_Grace active harness (this document's subject) uses a
**single deterministic** $L = 1.25$ m for every drone
(`decentralized_fault_aware_sim_main.cc:258`), removing the
uncertainty from the campaign so that scenario outcomes reflect
controller and fault behaviour alone. The controller class
`DecentralizedLocalController` has no feedback-path dependence on
rope length: `rope_length` / `rope_drop` appear only in the
reference-slot computation
(`decentralized_local_controller.cc:91, 95`). The vertical closed
loop feed-forwards the *measured* rope tension
(`.cc:192`), which renders the vertical dynamics independent of any
rope-length error — a static slot bias of $O(\sigma_L)$ would
remain, but no dynamic tracking failure. **[D]**

A Monte-Carlo sweep parameterising $\sigma_L$ inside this harness is
an obvious student-sized extension; see
[`case_study_tether_grace.md`](../case_study_tether_grace.md) §9.3
open item L6.

## 9. What the model does NOT capture

- **Bending stiffness.** Each bead is a point mass; there is no
  cross-segment moment. Real braided rope has a small bending
  stiffness; here it is zero.
- **Hysteretic / viscoelastic creep.** The Kelvin-Voigt model has pure
  linear spring + linear dashpot. A real polymer rope loses a fraction
  of the stored elastic energy per cycle; here damping is frequency-linear.
- **Axial strain-rate saturation.** If an agent were to command an
  acceleration that would instantaneously require $\dot{\ell}\gg 1$ m/s,
  the spring-damper force grows without bound; a real rope would slip
  or break. We do not model breaking; tensions are log-limited by the
  controller's thrust saturation.

## 10. Code-to-theory map

| Eq. | Code |
|:-:|---|
| (1) | [`decentralized_fault_aware_sim_main.cc:L257–274`](../../cpp/src/decentralized_fault_aware_sim_main.cc#L257) (parameter declarations) |
| (2) | [`rope_force_system.cc`](../../../Tether_Lift/Research/cpp/src/rope_force_system.cc) (per-segment Kelvin-Voigt + tension-only clamp) |
| (3) | idem (force distribution to endpoint bodies via `ExternallyAppliedSpatialForce`) |
| (4) | derivation of $k_{\text{rope}}^{\text{eff}}$ from series elasticity |
| (5) | damping-ratio check, Eq. (5) |
| (6) | stability bound, no explicit code (design calculation) |
| (7)–(8) | `rope_drop` computed from the hover-equilibrium fixed-point iteration at [`decentralized_fault_aware_sim_main.cc:L360–389`](../../cpp/src/decentralized_fault_aware_sim_main.cc#L360); for the nominal parameters $\mathrm{rope\_drop}\approx 1.211$ m. |
| §8 | [`cable_fault_gate.h`](../../cpp/include/cable_fault_gate.h), [`fault_aware_rope_visualizer.cc`](../../cpp/src/fault_aware_rope_visualizer.cc) |
| §9 | [`rope_segment_tension_probe.h`](../../cpp/include/rope_segment_tension_probe.h) |

---

## 9. RopeSegmentTensionProbe — observer-only tension instrument

### 9.1 Purpose

The bead-chain model gives each rope segment a Kelvin-Voigt tension
$T_{i,j}(t)$ (Eq. 2), but the `rope_force_system.cc` baseline
outputs only the **end-to-end** rope force on the drone and payload.
The `RopeSegmentTensionProbe` is an **observer-only** `LeafSystem`
(no feedback, no plant modification) that reconstructs the full
$N_{\text{drones}} \times N_{\text{seg}}$ tension matrix at every
simulation tick without changing the Tether_Lift baseline.

### 9.2 Architecture

```
    plant_state_port  →  [ RopeSegmentTensionProbe ]  →  tension_matrix_port
                                                         (N_drones × N_seg floats)
```

The probe reads the plant's generalised state $\mathbf{q}, \dot{\mathbf{q}}$
from the `plant_state_port` (a continuous abstract-valued input port).
It queries the `MultibodyPlant` kinematics to reconstruct the
position and velocity of every bead and endpoint, then recomputes
Eq. (2) per-segment to build the tension matrix. This is a pure
observation step — the same arithmetic as the `rope_force_system`
forward pass, with no side-effects on the plant state.

### 9.3 Output

| Port | Shape | Units | Description |
|---|---|---|---|
| `tension_matrix` | $N_d \times N_s$ | N | $T_{i,j}(t)$ for drone $i$, segment $j$ |

where $N_d \in \{4, 5\}$ and $N_s = N_{\text{beads}} + 1 = 9$ (for
the nominal bead configuration).

### 9.4 Applications

The tension matrix enables:

1. **Bead-chain tension waterfall (Figure F04).** Plot
   $T_{i,j}(t)$ for each drone $i$ as a 9-row waterfall — shows the
   axial wave propagation after a fault event (the tension step
   travels from the payload attachment up to the drone attachment at
   the longitudinal wave speed $\sqrt{k_s L_{\text{seg}}/\rho_{\text{seg}}}$).
2. **$\sigma_T(t)$ STFT spectrograms.** The standard deviation
   $\sigma_T(t) = \mathrm{std}_{j}(T_{i,j})$ tracks axial mode
   excitation — spectral content near the axial natural frequency
   $\omega_{\text{axial}} = \sqrt{k_s / m_{\text{bead}}}$ identifies
   resonance onset.
3. **Fault detection (future work).** An unexpected drop in
   $T_{i,0}$ (bottom segment tension near the payload) to near-zero
   is a clear signature of rope cut or payload separation.

### 9.5 Limitations

- The probe computes the Kelvin-Voigt tension analytically; it does
  not read the tension cached inside the `rope_force_system` integrator
  state. In the discrete-step plant (2×10⁻⁴ s), the one-step lag
  between the plant's rope-force evaluation and the probe's kinematics
  evaluation introduces a $\le 2\times 10^{-4}$ s delay in the
  reconstructed tension — negligible for the 12 Hz publication figures.
- Segment tensions are **not observable from the drone actuator side**
  in the controller's local model; the probe is strictly a
  simulation analysis tool.

> **Code reference:**
> [`cpp/include/rope_segment_tension_probe.h`](../../cpp/include/rope_segment_tension_probe.h)
