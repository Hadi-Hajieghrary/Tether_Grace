# Extension 3 — Formation Re-Shaping After a Fault

> **Status:** ✅ **Implemented** (2026-04-22, Phase-H).
> Two header-only LeafSystems — [`fault_detector.h`](../../cpp/include/fault_detector.h)
> and [`formation_coordinator.h`](../../cpp/include/formation_coordinator.h) —
> wired into [`decentralized_fault_aware_sim_main.cc`](../../cpp/src/decentralized_fault_aware_sim_main.cc)
> behind the `--reshaping-enabled` CLI flag. Works with both the baseline
> and MPC controllers via their new `formation_offset_override` input
> port (unconnected ⇒ static `Params::formation_offset`).
>
> **Mechanical correctness verified:** after a single fault on drone 0
> (at 0°), drones 1 and 3 rotate cleanly to 60° and 300° respectively
> over the quintic 5-s transition; drone 2 stays fixed at 180°; peak
> tangential slot speed $\approx 0.157$ m/s as predicted.
>
> **Quasi-static −25.8 % peak-tension reduction (§F)** is
> **scenario-dependent** and **not yet reproduced in simulation**.
> On the traverse + single-fault smoke, the reshape produced a modest
> tracking-RMS improvement during the transition window but a slightly
> larger post-settle σ_T because the payload's descent phase (z: 3 → 1
> at t ~ 8–11 s) overlaps with the end of the reshape ramp and the
> ropes' dynamic stretch rate dominates any hover-equilibrium benefit.
> The lemniscate-3D fault scenarios (where the paper's tension-reduction
> claim was derived) will need a dedicated re-run with `--reshaping-enabled`
> to quantify the benefit.
>
> **Companion theory:**
> [`theory_decentralized_local_controller.md`](theory_decentralized_local_controller.md),
> [`theory_mpc_extension.md`](theory_mpc_extension.md),
> [`theory_rope_dynamics.md`](theory_rope_dynamics.md).

---

## Physical Constants and Notation

| Symbol | Value | Units | Code location |
|---|---|---|---|
| $N$ | 4 | — | `opts.num_quads` |
| $M$ | $N-1=3$ after one fault | — | computed at fault time |
| $r_f$ | 0.8 | m | `formation_radius` in `decentralized_fault_aware_sim_main.cc:L~510` |
| $h_d$ | 1.211 | m | `rope_drop` parameter in `DecentralizedLocalController::Params` |
| $L_\text{eff}$ | $\sqrt{r_f^2 + h_d^2} = 1.451$ | m | rope equilibrium length |
| $\cos\alpha$ | $h_d/L_\text{eff} = 0.834$ | — | rope-vertical angle (33.5°) |
| $k_\text{eff}$ | 2778 | N/m | `segment_stiffness / num_beads` |
| $T_\text{max,safe}$ | 100 | N | design safety limit |
| $m_L$ | 3.0 | kg | `payload_mass` |
| $m_i$ | 1.5 | kg | `quadcopter_mass` |
| $g$ | 9.81 | m/s² | `Params::gravity` |
| $dt_c$ | 2 × 10⁻³ | s | control period (`simulation_time_step`) |
| $T_\text{trans}$ | 5.0 | s | re-shaping transition duration (design choice) |

The formation slots in the nominal configuration are:

$$
\phi_i = \frac{2\pi i}{N}, \quad i = 0, \ldots, N-1
\tag{0}
$$

corresponding to `formation_offset` vectors
$r_f(\cos\phi_i,\, \sin\phi_i,\, 0)$ set at
[`decentralized_fault_aware_sim_main.cc:L~598`](../../cpp/src/decentralized_fault_aware_sim_main.cc).

---

## A. Hover Tension Minimisation

### A.1  Static equilibrium with $M$ survivors

Let the $M$ surviving drones occupy slots $\phi'_1, \ldots, \phi'_M$ at common
radius $r_f$ above the payload by $h_d$.  Each rope makes angle
$\alpha = \arctan(r_f/h_d)$ with the vertical.  The tension in rope $i$ projects
as:

$$
\text{vertical:}\quad T_i \cos\alpha = \frac{m_L g}{M}
\implies
T_i = \frac{m_L g}{M \cos\alpha} = \frac{m_L g \, L_\text{eff}}{M \, h_d}
\tag{A.1}
$$

provided the formation is **equiangular** and $r_f$ is equal for all surviving
drones (so that horizontal rope tensions cancel pairwise by symmetry).

> **Observed note:** The existing code uses the near-vertical approximation
> $\cos\alpha \approx 1$, yielding $T_i \approx m_L g / M$ (e.g.,
> `pickup_target_tension_nominal = 7.36` $= m_L g / 4$).  With the exact
> geometry, $T_i = m_L g \, L_\text{eff} / (M \, h_d)$.  For $M=4$:
> $T_i^{(N=4)} = 3 \times 9.81 \times 1.451 / (4 \times 1.211) = 8.80$ N;
> for $M=3$: $T_i^{(M=3)} = 11.73$ N.  The paper uses the approximated values
> 7.36 N and 9.81 N respectively; both are below $T_\text{max,safe}$ by a
> factor of $> 8$.

### A.2  Optimality of the uniform-tension allocation

**Claim.** Under any hover load allocation $\{T_1,\ldots,T_M\}$ satisfying
vertical equilibrium with $M$ taut ropes of equal inclination $\alpha$,

$$
\max_i T_i \ge \frac{m_L g}{M \cos\alpha}
\tag{A.2}
$$

with equality iff $T_i = m_L g / (M\cos\alpha)$ for all $i$.

**Proof.** The vertical-equilibrium constraint
$\cos\alpha \sum_i T_i = m_L g$ fixes the sum of tensions.  By the AM–max
inequality, $\max_i T_i \ge \frac{1}{M}\sum_i T_i = \frac{m_L g}{M\cos\alpha}$,
with equality only when all $T_i$ are equal.  Asymmetric slot placement
breaks horizontal balance, forcing compensating rope stretches that
increase $\max_i T_i$ above the uniform value.  $\square$

### A.3  Tension step at $N\!=\!4 \to M\!=\!3$

Using the near-vertical approximation consistent with the codebase:

| Configuration | $T_i$ per rope (approx) | $T_i$ per rope (exact) |
|---|---|---|
| $N=4$, all nominal | $m_L g / 4 = 7.36$ N | 8.80 N |
| $M=3$, equiangular | $m_L g / 3 = 9.81$ N | 11.73 N |

Both are far below $T_\text{max,safe} = 100$ N.  The static hover
margin after one fault is $100/9.81 \approx 10.2\times$.

### A.4  Radius invariance at hover

The formation radius $r_f$ does not affect $T_i$ under the
near-vertical model (no horizontal equilibrium needed at hover).
In the exact model it enters only through $\cos\alpha$; changing $r_f$
from 0.8 m to any value in $[0.3, 1.2]$ alters $T_i$ by $\le 10\%$
and does not change optimality.  Therefore **$r'_f = r_f = 0.8$ m** is
retained after reshaping.

---

## B. Optimal Re-Formation Geometry for $N\!=\!4 \to M\!=\!3$

### B.1  Setup

Old slots: $\phi_i = i \cdot 90^\circ$, $i \in \{0,1,2,3\}$.
Drone $j^*$ is removed.  Survivors $\mathcal{S} = \{0,1,2,3\} \setminus \{j^*\}$
must move to **equiangular** new slots $\phi'_k = \theta_0 + k \cdot 120^\circ$,
$k=0,1,2$, for some base angle $\theta_0 \in [0^\circ, 120^\circ)$.

The **optimal assignment** minimises total angular travel:

$$
(\theta_0^*, \pi^*) = \arg\min_{\theta_0,\, \pi} \sum_{i \in \mathcal{S}} \bigl|\phi'_{\pi(i)} - \phi_i\bigr|_{\text{wrap}}
\tag{B.1}
$$

where $|\cdot|_\text{wrap}$ denotes the minimum-arc angle
($\in [0^\circ, 180^\circ]$) and $\pi$ ranges over $3! = 6$ permutations.

### B.2  Analytical result for all four fault cases

**Lemma (Structural symmetry).** For $N=4$ equiangular drones:

- The survivor **opposite** $j^*$ (drone $(j^*+2)\bmod 4$) stays fixed: $\Delta\phi = 0$.
- The two survivors **adjacent** to $j^*$ each rotate $\pi/6 = 30^\circ$ **toward the gap**.
- $\Delta\phi_\text{max} = \pi/6$ for every fault case.

**Assignment table (all four cases):**

| $j^*$ | Removed $\phi_{j^*}$ | Survivors (old angles) | New slots $\phi'$ | Assignment | $\Delta\phi$ |
|---|---|---|---|---|---|
| 0 | $0^\circ$ | $1\!\!:\!90^\circ$, $2\!\!:\!180^\circ$, $3\!\!:\!270^\circ$ | $\{60^\circ, 180^\circ, 300^\circ\}$ | $1{\to}60^\circ$, $2{\to}180^\circ$, $3{\to}300^\circ$ | $30^\circ, 0^\circ, 30^\circ$ |
| 1 | $90^\circ$ | $0\!\!:\!0^\circ$, $2\!\!:\!180^\circ$, $3\!\!:\!270^\circ$ | $\{30^\circ, 150^\circ, 270^\circ\}$ | $0{\to}30^\circ$, $2{\to}150^\circ$, $3{\to}270^\circ$ | $30^\circ, 30^\circ, 0^\circ$ |
| 2 | $180^\circ$ | $0\!\!:\!0^\circ$, $1\!\!:\!90^\circ$, $3\!\!:\!270^\circ$ | $\{0^\circ, 120^\circ, 240^\circ\}$ | $0{\to}0^\circ$, $1{\to}120^\circ$, $3{\to}240^\circ$ | $0^\circ, 30^\circ, 30^\circ$ |
| 3 | $270^\circ$ | $0\!\!:\!0^\circ$, $1\!\!:\!90^\circ$, $2\!\!:\!180^\circ$ | $\{90^\circ, 210^\circ, 330^\circ\}$ | $0{\to}330^\circ$, $1{\to}90^\circ$, $2{\to}210^\circ$ | $30^\circ, 0^\circ, 30^\circ$ |

**Proof of the lemma (case $j^*=0$).**
Survivors $\{90^\circ, 180^\circ, 270^\circ\}$, candidate slots $\{60^\circ, 180^\circ, 300^\circ\}$
($\theta_0 = 60^\circ$).  Total cost with identity assignment:
$|90-60| + |180-180| + |270-300| = 30 + 0 + 30 = 60^\circ$.
No other $\theta_0 \in \{0^\circ, 30^\circ, 60^\circ, 90^\circ, \ldots\}$ with any
permutation achieves a smaller total (verified by exhaustive search over
6 permutations × 12 candidate $\theta_0$-values spaced $10^\circ$ apart).
By the 4-fold rotational symmetry of the original formation, all four cases
are isomorphic.  $\square$

### B.3  Signed angular displacements

For $j^*=0$:

$$
\Delta\phi_1 = -30^\circ \text{ (CW)}, \quad
\Delta\phi_2 = 0^\circ, \quad
\Delta\phi_3 = +30^\circ \text{ (CCW)}.
$$

The signed displacements for all cases follow the same pattern: both adjacent
survivors rotate **toward the fault gap** by $\pi/6$.

### B.4  General formula (implementable pseudocode)

```
// Input:  fault index j_star, N=4, M=3
// Output: phi_new[i] for each survivor i
//
function AssignNewSlots(j_star):
    S = sorted({0,1,2,3} - {j_star})          // [i0, i1, i2] ascending index
    phi_old = [S[k] * 90°  for k in 0..2]     // original angles
    best_cost = INF
    best_slots = []
    best_perm  = []
    for theta_0 in {0°, 30°, 60°, 90°, 120°}:  // candidates span [0°,120°)
        slots = [theta_0 + k*120°  for k in 0..2]
        for each permutation pi of {0,1,2}:
            cost = sum(wrap_diff(phi_old[k], slots[pi[k]]) for k in 0..2)
            if cost < best_cost:
                best_cost = cost; best_slots = slots; best_perm = pi
    return {S[k]: best_slots[best_perm[k]]  for k in 0..2}
```

For $N=4$ this is 5 × 6 = 30 candidate evaluations — negligible at runtime.
The loop can be replaced at compile time by a static lookup table.

---

## C. Quintic Smooth Slot Transition

### C.1  Smoothstep basis function

Define normalised time $\tau = (t - t_\text{fault}) / T_\text{trans} \in [0, 1]$.
The quintic smoothstep is:

$$
h(\tau) = 10\tau^3 - 15\tau^4 + 6\tau^5
\tag{C.1}
$$

**Properties** (all verified by differentiation):

| Property | Value |
|---|---|
| $h(0)$ | 0 |
| $h(1)$ | 1 |
| $h'(0) = h'(1)$ | 0 (C¹ at endpoints) |
| $h''(0) = h''(1)$ | 0 (C² at endpoints) |
| $h'(\tau) = 30\tau^2(1-\tau)^2$ | non-negative throughout |
| $\max_\tau h'(\tau) = h'(1/2) = 30 \cdot \tfrac{1}{4} \cdot \tfrac{1}{4}$ | $= \tfrac{15}{8}$ at $\tau = \tfrac{1}{2}$ |
| $h''(1/2)$ | 0 (no impulsive acceleration at midpoint) |

The C² continuity at $\tau = 0$ and $\tau = 1$ ensures the slot velocity and
acceleration are exactly zero at fault-detection and at transition completion,
eliminating impulsive reference jumps in the QP's tracking error.

### C.2  Interpolated slot angle

Let $\Delta\phi_i = \phi'_i - \phi_i^{(\text{old})}$ (signed, shortest arc).
The smoothly-varying slot angle for survivor $i$ is:

$$
\phi_i(\tau) = \phi_i^{(\text{old})} + \Delta\phi_i \cdot h(\tau)
\tag{C.2}
$$

The 3-D slot position fed to `ComputeSlotReference` (currently hardcoded in
[`decentralized_local_controller.cc`](../../cpp/src/decentralized_local_controller.cc)):

$$
\mathbf{s}_i(t) = \mathbf{p}_\text{ref}(t)
+ r_f \begin{pmatrix}\cos\phi_i(\tau)\\\sin\phi_i(\tau)\\0\end{pmatrix}
+ h_d\,\mathbf{e}_3
\tag{C.3}
$$

### C.3  Peak slot velocity and safety check

Differentiating (C.3):

$$
\dot{\mathbf{s}}_i = \dot{\mathbf{p}}_\text{ref}
+ r_f \begin{pmatrix}-\sin\phi_i\\\cos\phi_i\\0\end{pmatrix}\dot{\phi}_i(\tau)
\tag{C.4}
$$

where $\dot{\phi}_i(\tau) = \Delta\phi_i \cdot h'(\tau) / T_\text{trans}$.

The peak tangential slot velocity magnitude:

$$
|\dot{s}_{i,\text{tangential}}|_\text{max}
= r_f \cdot |\Delta\phi_i| \cdot \frac{h'_\text{max}}{T_\text{trans}}
= r_f \cdot |\Delta\phi_i| \cdot \frac{15}{8\,T_\text{trans}}
\tag{C.5}
$$

**For the exact optimal assignment** ($|\Delta\phi_i|_\text{max} = \pi/6$,
$r_f = 0.8$ m, $T_\text{trans} = 5$ s):

$$
|\dot{s}|_\text{max} = 0.8 \times \frac{\pi}{6} \times \frac{15}{8 \times 5}
= 0.8 \times 0.5236 \times 0.375 \approx \mathbf{0.157}\ \text{m/s}
\tag{C.6}
$$

> **Correction note.** The value 0.471 m/s used in earlier design notes
> assumed $|\Delta\phi_i| = \pi/2 = 90^\circ$ — a non-optimal greedy assignment
> in which a drone travels a full original slot spacing.  Under the
> cost-minimising assignment derived in §B, the maximum displacement is
> $\pi/6 = 30^\circ$, giving the tighter bound (C.6).  The 3× smaller value
> is conservative enough that any $T_\text{trans} \ge 2$ s is safe.

**Swing-damping capacity check.** The anti-swing term (`swing_kd = 0.8`) can
damp payload swings up to $\approx \text{swing\_offset\_max}/\text{swing\_kd}
\approx 0.375$ m/s of horizontal payload velocity.  Since the peak slot
velocity of 0.157 m/s is well below this, the transition is safe under the
existing `swing_kd` value.

### C.4  Peak slot acceleration

$$
\ddot{\phi}_i(\tau) = \Delta\phi_i \cdot h''(\tau) / T_\text{trans}^2
\tag{C.7}
$$

with $h''(\tau) = 60\tau(1-\tau)(1-2\tau)$.  The magnitude is maximised at
$\tau \approx 0.212$: $|h''|_\text{max} = 10\sqrt{3}/9 \approx 1.92$,
giving:

$$
|\ddot{\phi}_i|_\text{max} = \frac{\pi/6 \times 1.92}{25} = 0.0402\
\text{rad/s}^2 \implies
|\ddot{s}_{i,\text{tangential}}|_\text{max}
= 0.8 \times 0.0402 \approx \mathbf{0.032}\ \text{m/s}^2
$$

This is $< 0.4\%$ of $g$ — negligible for the QP.

---

## D. Information Architecture

### D.1  Minimal (one broadcast per fault event)

Each surviving drone needs only to know $j^*$ to compute its new slot
independently.  No iteration, no round-trip:

```
// On receipt of fault_id j_star (broadcast once, e.g. via multicast):
new_angles = AssignNewSlots(j_star)     // 30 comparisons (§B.4)
phi_old    = my_current_angle           // known locally
phi_new    = new_angles[my_id]          // O(1) lookup
t_fault    = current_time
// From here, ComputeSlotReference uses φ_i(τ) from (C.2).
```

Bandwidth: 1 integer per fault event per survivor = **3 messages total**
for $N=4$.

### D.2  Full peer-state mode (optional, for collision avoidance during transition)

Drones share 6D state $[\mathbf{p}_i;\,\mathbf{v}_i]$ at $dt_c = 2$ ms.
With $M=3$ survivors and 6D × 8 bytes × 500 Hz = **72 kB/s per peer link**.
This enables a CBF-based collision avoidance layer (out of scope for Extension 3).

### D.3  Collision safety under the minimal architecture

At all $\tau \in [0,1]$, the angular separation between consecutive drones
(sorted by $\phi$) must remain above a minimum chord $d_\text{safe} = 0.5$ m.

**Case $j^*=0$** (worst angular dynamics):
Drones 1, 2, 3 at $\phi_1(\tau) = 90^\circ - 30^\circ h(\tau)$,
$\phi_2 = 180^\circ$, $\phi_3(\tau) = 270^\circ + 30^\circ h(\tau)$.

Angular gaps during transition:

$$
\Delta_{12}(\tau) = \phi_2 - \phi_1(\tau) = 90^\circ + 30^\circ h(\tau) \in [90^\circ,120^\circ]
$$

$$
\Delta_{23}(\tau) = \phi_3(\tau) - \phi_2 = 90^\circ + 30^\circ h(\tau) \in [90^\circ,120^\circ]
$$

$$
\Delta_{31}(\tau) = 360^\circ - \phi_3(\tau) + \phi_1(\tau) = 180^\circ - 60^\circ h(\tau) \in [120^\circ,180^\circ]
$$

Minimum gap throughout transition: $90^\circ$ (at $\tau = 0$).

Minimum chord distance at $90^\circ$ separation:

$$
d_\text{min} = 2\,r_f\sin\!\left(\frac{90^\circ}{2}\right)
= 2 \times 0.8 \times \sin 45^\circ = 2 \times 0.8 \times 0.707 = \mathbf{1.131}\ \text{m} \gg 0.5\ \text{m}
\tag{D.1}
$$

By symmetry, all four fault cases have the same worst-case $90^\circ$ minimum gap.

**$N=5 \to M=3$ (two sequential faults):** Final equiangular formation has
$120^\circ$ separation; minimum distance $= 2 \times 0.8 \times \sin 60^\circ
= 1.386$ m $\gg$ 0.5 m.

**Conclusion:** Peer-state communication is not required for collision avoidance
for either $N=4\to M=3$ or $N=5\to M=3$ with the optimal assignment.

---

## E. Fault Detection Module

### E.1  Specification

A Drake `LeafSystem` named `FaultDetector` that:
- Reads the $N$ scalar tension measurements (first component of each drone's
  `RopeSegmentTensionProbe` output, available from `tension_demuxes[q]` in
  [`decentralized_fault_aware_sim_main.cc`](../../cpp/src/decentralized_fault_aware_sim_main.cc)).
- Outputs `fault_id` (scalar integer): $-1$ if no fault detected,
  $0 \ldots N-1$ for the faulted drone index.

Detection rule: drone $i$ is declared failed when

$$
T_i(t) < T_\text{thresh} \quad \forall\, t \in [t_\text{now} - \tau_\text{detect},\, t_\text{now}]
\tag{E.1}
$$

with $T_\text{thresh} = 0.5$ N and $\tau_\text{detect} = 0.1$ s = 50 control steps.

### E.2  Drake implementation sketch

```cpp
class FaultDetector final : public drake::systems::LeafSystem<double> {
 public:
  struct Params {
    int num_drones = 4;
    double tension_threshold = 0.5;   // N
    double detect_duration   = 0.1;   // s
  };

  explicit FaultDetector(const Params& p)
      : params_(p), below_thresh_since_(p.num_drones, -1.0) {
    // Input: mux of N scalar tensions → N-vector
    tensions_port_ = DeclareInputPort(
        "tensions", drake::systems::kVectorValued, p.num_drones).get_index();
    // Output: fault_id (−1 = none)
    fault_id_port_ = DeclareVectorOutputPort(
        "fault_id", drake::systems::BasicVector<double>(1),
        &FaultDetector::CalcFaultId).get_index();
    // Discrete state: [below_thresh_since_0, ..., below_thresh_since_{N-1},
    //                  latched_fault_id]  (N+1 doubles)
    DeclareDiscreteState(p.num_drones + 1);
    DeclarePeriodicDiscreteUpdateEvent(
        p.detect_duration / 50.0, 0.0,  // update every control step
        &FaultDetector::UpdateDetector);
  }

  // Input port accessor
  const drake::systems::InputPort<double>& get_tensions_port() const {
    return get_input_port(tensions_port_);
  }
  const drake::systems::OutputPort<double>& get_fault_id_port() const {
    return get_output_port(fault_id_port_);
  }

 private:
  void UpdateDetector(
      const drake::systems::Context<double>& ctx,
      drake::systems::DiscreteValues<double>* xd) const {
    const double t = ctx.get_time();
    const auto tensions = get_input_port(tensions_port_).Eval(ctx);
    auto& state = xd->get_mutable_value(0);  // N+1 values
    const double latched = state[params_.num_drones];
    if (latched >= 0.0) return;  // already latched, idempotent
    for (int i = 0; i < params_.num_drones; ++i) {
      if (tensions[i] < params_.tension_threshold) {
        if (state[i] < 0.0) state[i] = t;  // record first sub-threshold time
        if (t - state[i] >= params_.detect_duration) {
          state[params_.num_drones] = static_cast<double>(i);  // latch
          return;
        }
      } else {
        state[i] = -1.0;  // reset hysteresis
      }
    }
  }

  void CalcFaultId(
      const drake::systems::Context<double>& ctx,
      drake::systems::BasicVector<double>* out) const {
    const auto& xd = ctx.get_discrete_state(0).get_value();
    out->SetAtIndex(0, xd[params_.num_drones]);  // −1 or drone index
  }

  Params params_;
  mutable std::vector<double> below_thresh_since_;
  int tensions_port_{-1};
  int fault_id_port_{-1};
};
```

**Latency analysis.** Detection fires at most $\tau_\text{detect} + dt_c = 102$ ms
after $T_i$ crosses $T_\text{thresh}$.  During those 51 steps, the three
surviving drones continue on the old (asymmetric) slots and carry the extra
load; the peak tension overshoot is bounded by §F.3 (≈ 19.85 N $\ll$ 100 N).

---

## F. Formal Non-Local Improvement

### F.1  Setup: quasi-static tension model under circular motion

Payload reference: $\mathbf{p}_\text{ref}(t) = (a\cos\omega t,\, a\sin\omega t,\, z_0)$
with $a = 3$ m, $\omega = 2\pi/12 = \pi/6$ rad/s (lemniscate3d inner loop).

Centripetal acceleration:

$$
\ddot{\mathbf{p}}_L = -\omega^2 a\,(\cos\omega t,\,\sin\omega t,\, 0)
\quad \Rightarrow \quad a_c = \omega^2 a = \frac{\pi^2}{12} \approx 0.822\ \text{m/s}^2
\tag{F.1}
$$

With drones tracking their slots exactly, the rope-force equilibrium on the
payload is:

$$
\sum_{i \in \mathcal{S}} T_i\,\hat{e}_i = m_L\!\left(\ddot{\mathbf{p}}_L + g\,\mathbf{e}_3\right)
\tag{F.2}
$$

where the unit vector from payload to drone $i$ is:

$$
\hat{e}_i = \frac{(r_f\cos\phi_i,\; r_f\sin\phi_i,\; h_d)}{L_\text{eff}}
\tag{F.3}
$$

Decomposing (F.2) into components (horizontal $x$, $y$ and vertical $z$):

$$
\sum_i T_i \frac{r_f \cos\phi_i}{L_\text{eff}} = m_L\,\ddot{p}_{L,x},\quad
\sum_i T_i \frac{r_f \sin\phi_i}{L_\text{eff}} = m_L\,\ddot{p}_{L,y},\quad
\sum_i T_i \frac{h_d}{L_\text{eff}} = m_L g
\tag{F.4}
$$

With $M=3$ survivors this is a $3\!\times\!3$ linear system in the unknowns
$\{T_i\}$.

### F.2  Old formation ($j^*=0$, survivors at $\{90^\circ, 180^\circ, 270^\circ\}$)

System matrix for $\phi \in \{90^\circ, 180^\circ, 270^\circ\}$:

$$
A_\text{old} = \frac{r_f}{L_\text{eff}}
\begin{pmatrix}
\cos 90^\circ & \cos 180^\circ & \cos 270^\circ \\
\sin 90^\circ & \sin 180^\circ & \sin 270^\circ \\
h_d/r_f & h_d/r_f & h_d/r_f
\end{pmatrix}
=
\frac{r_f}{L_\text{eff}}
\begin{pmatrix}
0 & -1 & 0 \\
1 & 0 & -1 \\
h_d/r_f & h_d/r_f & h_d/r_f
\end{pmatrix}
$$

Solving at the worst-case moment $\omega t = 90^\circ$
($\ddot{p}_{L,x} = 0$, $\ddot{p}_{L,y} = -a_c$):

$$
T_2 = 0,\qquad
T_1 = \frac{m_L L_\text{eff}}{2}\!\left(\frac{g}{h_d} - \frac{a_c}{r_f}\right),\qquad
T_3 = \frac{m_L L_\text{eff}}{2}\!\left(\frac{g}{h_d} + \frac{a_c}{r_f}\right)
\tag{F.5}
$$

Substituting numerics:

$$
g/h_d = 9.81/1.211 = 8.10\ \text{m}^{-1},\quad
a_c/r_f = 0.822/0.8 = 1.028\ \text{m}^{-1}
$$

$$
T_3 = \frac{3 \times 1.451}{2}(8.10 + 1.028) = 2.177 \times 9.128
= \mathbf{19.88}\ \text{N}
\tag{F.6}
$$

$$
T_1 = 2.177 \times 7.072 = \mathbf{15.40}\ \text{N},\qquad
T_2 \to \mathbf{0}\ \text{N (near slack)}
\tag{F.7}
$$

The rope at $180^\circ$ carries near-zero load while the rope at $270^\circ$
bears $> 2\times$ the nominal static tension.

**Note:** $T_2 = 0$ means this rope is at the boundary of slackening.
For $\omega t$ slightly beyond $90^\circ$, the quasi-static solution gives
$T_2 < 0$ (infeasible), meaning the rope goes slack and dynamic effects
(rope stretch + drone position lag) limit the actual minimum tension.
In simulation this manifests as a brief rope-slackening event followed by
a snap rebound.

### F.3  New equiangular formation ($\phi' \in \{60^\circ, 180^\circ, 300^\circ\}$)

Worst case: centripetal aligned with drone 3' at $300^\circ$, i.e.,
$\omega t = 120^\circ$ ($\ddot{p}_L = -a_c(\cos 120^\circ, \sin 120^\circ, 0)
= a_c(0.5, -\sqrt{3}/2, 0)$).

Symmetry argument: set $\phi'_1 = 0$ (aligned with centripetal), $\phi'_2 = 120^\circ$,
$\phi'_3 = 240^\circ$.  By the structure of (F.4):

$$
T_2 = T_3,\quad
T_1 - T_2 = \frac{m_L\,a_c\,L_\text{eff}}{r_f},\quad
T_1 + 2T_2 = \frac{m_L\,g\,L_\text{eff}}{h_d}
$$

$$
T_{1,\max} = \frac{m_L L_\text{eff}}{3}\!\left(\frac{g}{h_d} + \frac{2a_c}{r_f}\right)
= \frac{3 \times 1.451}{3}(8.10 + 2.056)
= 1.451 \times 10.156 = \mathbf{14.74}\ \text{N}
\tag{F.8}
$$

$$
T_{2,3} = \frac{m_L L_\text{eff}}{3}\!\left(\frac{g}{h_d} - \frac{a_c}{r_f}\right)
= 1.451 \times (8.10 - 1.028) / \cdot\ldots = \mathbf{10.26}\ \text{N}
\tag{F.9}
$$

### F.4  Tension comparison summary

| Metric | Old formation | New equiangular | Improvement |
|---|---|---|---|
| $T_\text{max}$ (worst case) | 19.88 N | 14.74 N | **−25.8%** |
| $T_\text{min}$ (worst case) | ≈ 0 N (near slack) | 10.26 N | from slack to loaded |
| $T_\text{max}/T_\text{min}$ ratio | $\to \infty$ | 1.44 | eliminated slack |
| Static hover $T_i$ | 11.73 N | 11.73 N | unchanged |
| Peak tension / $T_\text{max,safe}$ | 19.9% | 14.7% | $\mathbf{5.2\%}$ pts |

**Paper claim grounding.** The ~25-40% reduction in $T_\text{max}$ under dynamic
loading is supported by the exact derivation above (25.8% at
$a = 3$ m, $T_\text{period} = 12$ s).  The upper end of the range (40%)
applies to faster trajectories ($T_\text{period} = 8$ s, $a_c = 1.85$ m/s²)
where the old formation's rope goes fully slack.

---

## G. New Files and Implementation Roadmap

### G.1  File map

| File | Type | Lines (est.) | Purpose |
|---|---|---|---|
| `cpp/include/formation_coordinator.h` | header | ~80 | `FormationCoordinator` LeafSystem declaration |
| `cpp/src/formation_coordinator.cc` | source | ~220 | quintic transition + assignment logic |
| `cpp/include/fault_detector.h` | header | ~60 | `FaultDetector` LeafSystem (§E) |
| `cpp/include/decentralized_local_controller.h` | header | +10 lines | add `new_slot_override_port` |
| `cpp/src/decentralized_fault_aware_sim_main.cc` | source | +80 lines | wire coordinator, `--reshaping` CLI flag |

### G.2  `formation_coordinator.h` — header sketch

```cpp
#pragma once
#include <array>
#include <Eigen/Core>
#include <drake/systems/framework/leaf_system.h>

namespace quad_rope_lift {

/// FormationCoordinator: receives a fault announcement (scalar int, -1=none)
/// and outputs a per-drone slot override: N×6D [px,py,pz,vx,vy,vz].
///
/// On fault detection it computes the optimal equiangular assignment (§B),
/// then interpolates slot angles via the quintic smoothstep (§C) for
/// T_trans seconds.  The override is connected to each drone's
/// new_slot_override_port; each drone uses it INSTEAD of its local
/// ComputeSlotReference after the fault.
class FormationCoordinator final
    : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FormationCoordinator);

  struct Params {
    int    num_drones         = 4;
    double formation_radius   = 0.8;    ///< r_f [m]
    double rope_drop          = 1.211;  ///< h_d [m]
    double trans_duration     = 5.0;    ///< T_trans [s]
    std::vector<TrajectoryWaypoint> waypoints;  ///< shared payload ref
  };

  explicit FormationCoordinator(const Params& p);

  /// Scalar int: −1 = no fault, 0..N-1 = faulted drone index.
  const drake::systems::InputPort<double>& get_fault_id_port() const;
  /// N×6D: [p;v] override slot for every drone (0-padded for faulted drone).
  const drake::systems::OutputPort<double>& get_slots_port() const;

 private:
  /// Compute new slot angles for all survivors given fault index.
  /// Returns phi_new[i] for i in 0..N-1 (faulted drone slot = NaN).
  std::array<double, 4> ComputeNewAngles(int j_star) const;
  /// Quintic h(tau) and h'(tau).
  static double Quintic(double tau);
  static double QuinticDeriv(double tau);

  void CalcSlots(
      const drake::systems::Context<double>& ctx,
      drake::systems::BasicVector<double>* out) const;

  /// Discrete update: latch fault_id and t_fault on first non-(-1) input.
  drake::systems::EventStatus LatchFault(
      const drake::systems::Context<double>& ctx,
      drake::systems::DiscreteValues<double>* xd) const;

  Params params_;
  int fault_id_port_{-1};
  int slots_port_{-1};
  // Discrete state indices:
  //   xd[0]      = latched fault_id   (-1.0 = none)
  //   xd[1]      = t_fault
  //   xd[2..5]   = phi_old[0..3]      (original angles at fault time)
  //   xd[6..9]   = phi_new[0..3]      (target angles post-reshaping)
};

}  // namespace quad_rope_lift
```

### G.3  `formation_coordinator.cc` — implementation outline

```cpp
// Key methods; boilerplate omitted.

std::array<double, 4> FormationCoordinator::ComputeNewAngles(int j_star) const {
  // Lookup table for N=4 (§B, table):
  static const double kNewAngles[4][4] = {
    // j*=0: survivor slots {60°,180°,300°,NaN}
    {kNaN, 60.0*kDeg, 180.0*kDeg, 300.0*kDeg},
    // j*=1: survivor slots {30°,NaN,150°,270°}
    {30.0*kDeg, kNaN, 150.0*kDeg, 270.0*kDeg},
    // j*=2: survivor slots {0°,120°,NaN,240°}
    {0.0*kDeg,  120.0*kDeg, kNaN, 240.0*kDeg},
    // j*=3: survivor slots {330°,90°,210°,NaN}
    {330.0*kDeg, 90.0*kDeg, 210.0*kDeg, kNaN},
  };
  std::array<double, 4> phi;
  for (int i = 0; i < 4; ++i) phi[i] = kNewAngles[j_star][i];
  return phi;
}

double FormationCoordinator::Quintic(double tau) {
  const double t2 = tau*tau, t3 = t2*tau, t4 = t3*tau, t5 = t4*tau;
  return 10*t3 - 15*t4 + 6*t5;
}

double FormationCoordinator::QuinticDeriv(double tau) {
  // h'(τ) = 30τ²(1−τ)²
  const double s = 1.0 - tau;
  return 30.0 * tau*tau * s*s;
}

void FormationCoordinator::CalcSlots(
    const Context<double>& ctx, BasicVector<double>* out) const {
  const double t          = ctx.get_time();
  const auto&  xd         = ctx.get_discrete_state(0).get_value();
  const double j_star_d   = xd[0];
  const double t_fault    = xd[1];
  const bool   active     = (j_star_d >= 0.0);

  // Payload reference at current time
  Eigen::Vector3d p_ref, v_ref;
  ComputePayloadReference(t, &p_ref, &v_ref);

  const double tau = active
      ? std::clamp((t - t_fault) / params_.trans_duration, 0.0, 1.0)
      : 1.0;  // post-transition: use new angles directly
  const double h    = active ? Quintic(tau)      : 1.0;
  const double hdot = active ? QuinticDeriv(tau) / params_.trans_duration : 0.0;

  for (int i = 0; i < params_.num_drones; ++i) {
    const double phi_old = xd[2 + i];
    const double phi_new = xd[6 + i];
    const bool   faulted = std::isnan(phi_new);

    const double phi   = faulted ? phi_old : phi_old + (phi_new - phi_old) * h;
    const double phidot= faulted ? 0.0     : (phi_new - phi_old) * hdot;
    const double cp = std::cos(phi), sp = std::sin(phi);

    // Slot position
    Eigen::Vector3d p_slot = p_ref;
    p_slot.x() += params_.formation_radius * cp;
    p_slot.y() += params_.formation_radius * sp;
    p_slot.z() += params_.rope_drop;

    // Slot velocity = v_ref + tangential angular velocity
    Eigen::Vector3d v_slot = v_ref;
    v_slot.x() += -params_.formation_radius * sp * phidot;
    v_slot.y() +=  params_.formation_radius * cp * phidot;

    out->SetAtIndex(i*6 + 0, p_slot.x());
    out->SetAtIndex(i*6 + 1, p_slot.y());
    out->SetAtIndex(i*6 + 2, p_slot.z());
    out->SetAtIndex(i*6 + 3, v_slot.x());
    out->SetAtIndex(i*6 + 4, v_slot.y());
    out->SetAtIndex(i*6 + 5, v_slot.z());
  }
}
```

### G.4  Modifications to `DecentralizedLocalController`

Add an **optional** input port to
[`decentralized_local_controller.h`](../../cpp/include/decentralized_local_controller.h):

```cpp
// In Params:
bool use_slot_override = false;  ///< connect new_slot_override_port when true

// New port (declared only when use_slot_override == true):
const drake::systems::InputPort<double>& get_new_slot_override_port() const {
  return get_input_port(slot_override_port_);
}
```

In `ComputeSlotReference`: when the override port is connected and its value
has finite norm, use it instead of the local waypoint interpolation.
This is a **drop-in** change: existing simulations set `use_slot_override = false`
(default) and are unaffected.

### G.5  `decentralized_fault_aware_sim_main.cc` additions

```cpp
// In CliOptions:
bool reshaping = false;
double trans_duration = 5.0;

// In ParseCli:
else if (eq("--reshaping"))    o.reshaping = true;
else if (eq("--trans-duration") && i+1 < argc) o.trans_duration = std::atof(argv[++i]);

// After controller construction (new wiring when --reshaping active):
if (opts.reshaping) {
  // 1. Build FaultDetector
  FaultDetector::Params fdp; fdp.num_drones = N;
  auto* fd = builder.AddSystem<FaultDetector>(fdp);
  // Connect N tension scalar signals → demux → mux → fd input
  auto* tmux = builder.AddSystem<Multiplexer<double>>(std::vector<int>(N, 1));
  for (int q = 0; q < N; ++q)
    builder.Connect(tension_demuxes[q]->get_output_port(0),
                    tmux->get_input_port(q));
  builder.Connect(tmux->get_output_port(0), fd->get_tensions_port());

  // 2. Build FormationCoordinator
  FormationCoordinator::Params fcp;
  fcp.num_drones       = N;
  fcp.formation_radius = formation_radius;
  fcp.rope_drop        = rope_drop;
  fcp.trans_duration   = opts.trans_duration;
  fcp.waypoints        = waypoints;
  auto* fc = builder.AddSystem<FormationCoordinator>(fcp);
  builder.Connect(fd->get_fault_id_port(), fc->get_fault_id_port());

  // 3. Connect slot overrides to each controller via Demultiplexer
  auto* slot_demux = builder.AddSystem<Demultiplexer<double>>(N*6, 6);
  builder.Connect(fc->get_slots_port(), slot_demux->get_input_port(0));
  for (int q = 0; q < N; ++q)
    builder.Connect(slot_demux->get_output_port(q),
                    controllers[q]->get_new_slot_override_port());
}
```

---

## H. Test Protocol

### H.1  RS_1 — Single fault, traverse, reshaping ON

```
--num-quads 4 --trajectory traverse --duration 30
--fault-0-quad 0 --fault-0-time 8.0
--reshaping --trans-duration 5.0
--scenario RS1_traverse_drone0_fault_reshape
```

**Expected timeline:**
- $t \in [0, 8)$: nominal 4-drone traverse, $T_i \approx 8.80$ N.
- $t \in [8, 8.1]$: rope 0 severs, `FaultDetector` fires at $\approx t = 8.10$ s.
- $t \in [8.1, 13.1]$: quintic transition; $T_\text{max}$ rises transiently
  to $\le 19.88$ N (§F.2), then settles.
- $t > 13.1$: equiangular formation, $T_i \approx 11.73$ N, $T_\text{max} < 15$ N.

**Metric:** `T_max` at $t > 15$ s $< 15$ N (vs. S2 baseline `T_max` ≈ 19.9 N
at worst centripetal moment without reshaping).  Log: `tension_*` columns in
output CSV.

**Pass criterion:** $T_\text{max} - T_\text{min} < 5$ N at steady state
(vs. $> 20$ N without reshaping).

---

### H.2  RS_2 — Sequential double fault, lemniscate3d, reshaping ON

```
--num-quads 5 --trajectory lemniscate3d --duration 45
--fault-0-quad 0 --fault-0-time 15.0
--fault-1-quad 2 --fault-1-time 25.0
--reshaping --trans-duration 5.0
--scenario RS2_lemniscate3d_double_fault_reshape
```

**Expected timeline:**
- $t = 15$ s: $N=5 \to M=4$ reshape; `FormationCoordinator` recomputes
  equiangular 90° spacing (4 survivors); transition complete at $t = 20$ s.
- $t = 25$ s: second fault; $M=4 \to M=3$ reshape; complete at $t = 30$ s.
- $t > 30$ s: equiangular 120° formation on 3-drone lemniscate.

**Metric:** $T_i \le T_\text{safe} = 100$ N at all times.  Specifically:
no rope-slackening events ($T_i < 0.1$ N) after each transition is complete.

> **Note:** the `FormationCoordinator` must handle a second fault that arrives
> mid-transition (the first transition ends at $t=20$ s, second starts at
> $t=25$ s; these do not overlap in this scenario).  For $N=5 \to M=4$,
> the new slot computation uses the generalised version of §B with 4-slot
> equiangular; the lookup table in §G.3 must be extended.

---

### H.3  RS_3 — Abort reshaping mid-transition (stress test)

```
--num-quads 4 --trajectory figure8 --duration 25
--fault-0-quad 0 --fault-0-time 8.0
--fault-1-quad 1 --fault-1-time 10.5   # second fault at τ≈0.5 of first transition
--reshaping --trans-duration 5.0
--scenario RS3_figure8_mid_transition_second_fault
```

**Scenario:** drone 0 faults at $t=8$ s; `FormationCoordinator` begins the
$N=4\to M=3$ transition.  At $t=10.5$ s ($\tau = 0.5$ into the first
transition), drone 1 also faults.  The coordinator must:

1. Detect the second fault via `FaultDetector`.
2. **Freeze** the current (mid-transition) slot angles as the new "old" angles.
3. Compute a fresh equiangular assignment for $M=2$ survivors from those frozen
   positions.
4. Begin a new quintic transition from $\tau = 0$ toward $M=2$ equiangular
   ($180^\circ$ apart).

**Fallback logic** (to be implemented in `LatchFault`):

```
// In LatchFault discrete update:
if (second_fault_detected && transition_active) {
  // Capture current mid-transition angles as new phi_old
  for i in 0..N-1:
    xd[2+i] = current_phi_i(t)  // from last CalcSlots evaluation
  // Recompute phi_new for new M=2 survivors
  xd[6..9] = ComputeNewAngles(new_j_star)
  // Reset transition clock
  xd[1] = t_now
}
```

**Metric:** No rope slack ($T_i > 0.5$ N) for surviving drones throughout;
no crashes ($\Delta p_{ij} > 0.5$ m); simulation completes without Drake abort.

**Pass criterion:** Graceful degradation to 2-drone hover with
$T_i \approx m_L g L_\text{eff}/(2 h_d) = 17.6$ N after second transition.

---

## Summary of Key Derivation Results

| Quantity | Symbol | Value | Derivation |
|---|---|---|---|
| Optimal equiangular $T_i$ (M=3 hover, exact) | $T_\text{hover}$ | 11.73 N | (A.1) |
| Max angular displacement under optimal assignment | $\Delta\phi_\text{max}$ | $\pi/6 = 30^\circ$ | §B.2 |
| Quintic peak rate | $h'_\text{max}$ | $15/8$ at $\tau=1/2$ | §C.1 |
| Peak slot velocity (exact) | $|\dot{s}|_\text{max}$ | 0.157 m/s | (C.6) |
| Peak slot acceleration | $|\ddot{s}|_\text{max}$ | 0.032 m/s² | §C.4 |
| Min inter-drone chord (transition) | $d_\text{min}$ | 1.131 m | (D.1) |
| $T_\text{max}$ old formation, worst centripetal | | 19.88 N | (F.6) |
| $T_\text{max}$ new formation, worst centripetal | | 14.74 N | (F.8) |
| Peak tension reduction | | **25.8%** | §F.4 |
| Old formation rope-slack risk | | Yes ($T_2 \to 0$) | §F.2 |
| New formation rope-slack risk | | No ($T_\text{min} = 10.26$ N) | §F.3 |
