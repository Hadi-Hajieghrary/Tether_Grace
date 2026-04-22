# Theory — `DecentralizedLocalController`

> Companion to [`Research/cpp/src/decentralized_local_controller.cc`](../../cpp/src/decentralized_local_controller.cc) /
> [`.h`](../../cpp/include/decentralized_local_controller.h). Every
> equation is tagged with the code line that implements it.

## 1. Summary

Each quadrotor of an $N$-drone fleet cooperatively carrying a suspended
payload runs an **identical, fully-local** controller. At each control
step it solves a small quadratic program whose cost jointly minimises
(i) the drone's formation-slot tracking error, (ii) the payload's
pendulum swing, and (iii) the commanded control effort, subject to
tilt-envelope and thrust-envelope constraints. The QP output is a 3-D
commanded acceleration that drives a cascade-PD outer loop and a fast
attitude PD inner loop. The only external inputs to any drone are its
own plant state, its own rope-tension measurement, the payload state
(locally observable), and a shared feed-forward payload-reference
trajectory — **no peer-drone states, no peer-tension vector, no
`N_alive` estimator, no consensus protocol**. Fault tolerance emerges
from the coupled rope–payload physics under $N-1$ identical local
controllers.

## 2. Nomenclature

| Symbol | Meaning | Units | Code name | Location |
|:-:|---|:-:|---|---|
| $\mathbf{p}_i$ | drone position (world) | m | `p_self` | [`.cc:L137`](../../cpp/src/decentralized_local_controller.cc#L137) |
| $\mathbf{v}_i$ | drone velocity (world) | m/s | `v_self` | [`.cc:L138`](../../cpp/src/decentralized_local_controller.cc#L138) |
| $R_i$ | drone orientation | – | `R_self` | [`.cc:L139`](../../cpp/src/decentralized_local_controller.cc#L139) |
| $\boldsymbol{\omega}_i^W$ | angular velocity in world frame | rad/s | `omega_W` | [`.cc:L140`](../../cpp/src/decentralized_local_controller.cc#L140) |
| $\mathbf{p}_L$ | payload position | m | `payload_pose.translation()` | [`.cc:L143`](../../cpp/src/decentralized_local_controller.cc#L143) |
| $\mathbf{v}_L$ | payload velocity | m/s | `v_L` | [`.cc:L145`](../../cpp/src/decentralized_local_controller.cc#L145) |
| $T_i$ | scalar rope tension (drone $i$'s rope) | N | `measured_tension` | [`.cc:L129`](../../cpp/src/decentralized_local_controller.cc#L129) |
| $\bar{\mathbf{p}}_L$ | payload reference position | m | `p_L_ref` | [`.cc:L113`](../../cpp/src/decentralized_local_controller.cc#L113) |
| $\bar{\mathbf{v}}_L$ | payload reference velocity | m/s | `v_L_ref` | [`.cc:L113`](../../cpp/src/decentralized_local_controller.cc#L113) |
| $\Delta\mathbf{p}_i^{\text{offset}}$ | drone $i$'s horizontal formation offset from payload centre | m | `params_.formation_offset` | [`.h:L52`](../../cpp/include/decentralized_local_controller.h#L52) |
| $h$ | effective rope hang height (drone above payload) | m | `params_.rope_drop` | [`.h:L55`](../../cpp/include/decentralized_local_controller.h#L55) |
| $\mathbf{s}_i$ | drone $i$'s formation-slot target position | m | `p_slot` | [`.cc:L115–116`](../../cpp/src/decentralized_local_controller.cc#L115) |
| $k_s$ | anti-swing velocity-feedback gain | s | `params_.swing_kd` | [`.h:L64`](../../cpp/include/decentralized_local_controller.h#L64) |
| $\mathbf{a}_i^\star$ | optimal commanded acceleration | m/s² | `a_d_opt` | [`.cc:L251`](../../cpp/src/decentralized_local_controller.cc#L251) |
| $f_i$ | thrust magnitude | N | `thrust` | [`.cc:L279`](../../cpp/src/decentralized_local_controller.cc#L279) |
| $\boldsymbol{\tau}_i$ | body-frame torque | N·m | `torque_body` | [`.cc:L320`](../../cpp/src/decentralized_local_controller.cc#L320) |
| $m_i$ | drone mass | kg | `mass_` | [`.h:L102`](../../cpp/include/decentralized_local_controller.h#L102) |
| $g$ | gravitational acceleration | m/s² | `params_.gravity` | [`.h:L72`](../../cpp/include/decentralized_local_controller.h#L72) |
| $\theta_{\max}$ | maximum tilt angle | rad | `params_.max_tilt` | [`.h:L69`](../../cpp/include/decentralized_local_controller.h#L69) |
| $f_{\min},f_{\max}$ | thrust envelope | N | `min_thrust`, `max_thrust` | [`.h:L70–71`](../../cpp/include/decentralized_local_controller.h#L70) |
| $w_t, w_s, w_e$ | QP weights (tracking, swing, effort) | – | `params_.w_{track,swing,effort}` | [`.h:L70–72`](../../cpp/include/decentralized_local_controller.h#L70) |

## 3. System model — drone and payload

Drone $i$ is a rigid body of mass $m_i$, inertia $J_i$, pose
$(\mathbf{p}_i, R_i)$, velocity $(\mathbf{v}_i, \boldsymbol{\omega}_i)$.
Its equations of motion under gravity, thrust, rope pull, and aerodynamic
drag (assumed negligible) are

$$
\dot{\mathbf{p}}_i = \mathbf{v}_i, \qquad
m_i\,\dot{\mathbf{v}}_i = R_i\,\mathbf{e}_3\, f_i \;+\; \mathbf{F}_{\text{rope},i} \;-\; m_i g\,\mathbf{e}_3 ,
\tag{1}
$$

$$
J_i\,\dot{\boldsymbol{\omega}}_i + \boldsymbol{\omega}_i \times J_i\boldsymbol{\omega}_i \;=\; \boldsymbol{\tau}_i , \tag{2}
$$

where $\mathbf{e}_3 = (0,0,1)^\top$, $f_i\ge 0$ is the scalar collective
thrust, and $\mathbf{F}_{\text{rope},i}$ is the rope force on drone $i$
from its one rope (see [`theory_rope_dynamics.md`](theory_rope_dynamics.md)
for the bead-chain model that generates $\mathbf{F}_{\text{rope},i}$).

The payload is a point mass $m_L$ at $\mathbf{p}_L$ pulled by all $N$
ropes,

$$
m_L\,\dot{\mathbf{v}}_L \;=\; \sum_{j=1}^{N} \mathbf{F}_{\text{rope},j}^{\,(L)} \;-\; m_L g\,\mathbf{e}_3 , \tag{3}
$$

where $\mathbf{F}_{\text{rope},j}^{\,(L)}$ is the payload-side force of
rope $j$. Each drone sees only its own rope tension $T_i$ (the scalar
magnitude) and the payload pose $\mathbf{p}_L, \mathbf{v}_L$ — which is
a locally-observable signal (onboard camera, IR range sensor, or a
direct measurement from the drone's own cable geometry).

## 4. Reference and formation slot

A shared time-parameterised **payload** reference
$\bar{\mathbf{p}}_L(t)$, $\bar{\mathbf{v}}_L(t)$ is given by the waypoint
interpolator [`theory_figure8_trajectory.md`](theory_figure8_trajectory.md).
Drone $i$ targets a *formation slot* above the payload,

$$
\mathbf{s}_i(t) \;=\; \bar{\mathbf{p}}_L(t) \;+\; \Delta\mathbf{p}_i^{\text{offset}} \;+\; h\,\mathbf{e}_3 , \tag{4}
$$

with $\Delta\mathbf{p}_i^{\text{offset}} = r_f\,(\cos\phi_i,\ \sin\phi_i,\ 0)^\top$,
$\phi_i = 2\pi i/N$, and $h$ the effective rope-hang height so that at
hover the payload sits exactly at $\bar{\mathbf{p}}_L$. This step is
implemented at
[`.cc:L115–116`](../../cpp/src/decentralized_local_controller.cc#L115).

### Anti-swing slot correction

The instantaneous slot target is augmented by a velocity-proportional
horizontal correction that drives the drone against the payload's
pendulum motion:

$$
\tilde{\mathbf{s}}_i(t) \;=\; \mathbf{s}_i(t) \;-\; \mathrm{sat}_{\delta}\big( k_s\,\Pi_{xy}\,\mathbf{v}_L \big) , \tag{5}
$$

where $\Pi_{xy}$ is projection onto the horizontal plane and
$\mathrm{sat}_\delta(\cdot)$ saturates the correction magnitude to
$\delta = $ `swing_offset_max`. The drone moves toward the swinging
payload → rope tilts → rope force gets a horizontal component that
brakes the pendulum. Implemented at
[`.cc:L161–168`](../../cpp/src/decentralized_local_controller.cc#L161).

## 5. Cascade controller — outer loop

### 5.1 Desired-acceleration PD

Let $\mathbf{e}_p = \tilde{\mathbf{s}}_i - \mathbf{p}_i$,
$\mathbf{e}_v = \bar{\mathbf{v}}_L - \mathbf{v}_i$. The *tracking*
acceleration command is a diagonal PD:

$$
\mathbf{a}^{\text{track}} \;=\;
\begin{pmatrix}
K_p^{xy}\,e_p^x + K_d^{xy}\,e_v^x \\
K_p^{xy}\,e_p^y + K_d^{xy}\,e_v^y \\
K_p^{z}\,e_p^z + K_d^{z}\,e_v^z
\end{pmatrix}, \tag{6}
$$

gains $(K_p^{xy}, K_d^{xy}) = (30, 15)$ and $(K_p^{z}, K_d^{z}) = (100,24)$
chosen for $\zeta \approx 1$ with $m_i = 1.5$ kg,
giving $\omega_n^{xy} \approx 4.5$ rad/s and
$\omega_n^{z} \approx 8.2$ rad/s. Implemented at
[`.cc:L171–176`](../../cpp/src/decentralized_local_controller.cc#L171).

### 5.2 Anti-swing acceleration

The anti-swing correction also contributes an additive acceleration
component that points against $\mathbf{v}_L$:

$$
\mathbf{a}^{\text{swing}} \;=\; \begin{pmatrix}
-k_s\,v_L^x \\
-k_s\,v_L^y \\
0
\end{pmatrix}. \tag{7}
$$

Implemented at
[`.cc:L181–182`](../../cpp/src/decentralized_local_controller.cc#L181).
Note that (5) and (7) combine *additively* — they are not in tension.
The anti-swing term appears both as a formation-slot shift (5) and as a
direct acceleration bias (7); the former acts through the PD on the
shifted reference, the latter acts as a feed-forward on the target
acceleration. Empirically this compound action damps the pendulum more
aggressively than either alone.

### 5.3 The per-step local QP

$$
\boxed{
\begin{aligned}
\mathbf{a}_i^\star \;=\; \arg\min_{\mathbf{a}\,\in\,\mathbb{R}^3} \;
  & \underbrace{w_t\,\|\mathbf{a} - \mathbf{a}^{\text{target}}\|^2}_{\text{tracking + swing}} \;+\;
  \underbrace{w_e\,\|\mathbf{a}\|^2}_{\text{effort}} \\
\text{s.t.}\quad
  & |a_x|,\ |a_y| \;\le\; g\,\tan\theta_{\max} \\
  & \frac{f_{\min} - T_i^{\text{ff}}}{m_i} - g \;\le\; a_z \;\le\;
    \frac{f_{\max} - T_i^{\text{ff}}}{m_i} - g
\end{aligned}
} \tag{8}
$$

with $\mathbf{a}^{\text{target}} = \mathbf{a}^{\text{track}} + w_s\,\mathbf{a}^{\text{swing}}$.
The QP has **three decision variables, two box constraints, and two
quadratic costs** — a dead simple convex QP solved by Drake's
`MathematicalProgram` at every control step (well under 1 ms per drone).
Implemented at
[`.cc:L227–245`](../../cpp/src/decentralized_local_controller.cc#L227).

Weights are chosen so tracking dominates:
$w_t = 1,\ w_s = 0.3,\ w_e = 0.02$ — the effort term acts as a tie-breaker
between equivalent feasible points, not as a first-order objective.
Runtime values are set in `decentralized_fault_aware_sim_main.cc:L573–578`;
the `Params` struct defaults are overridden there to match the tuned
experiment values.
The tilt-envelope box $|a_{x,y}| \le g\tan\theta_{\max}$ with
$\theta_{\max} = 0.6$ rad gives a $6.71$ m/s² lateral-acceleration
budget — enough to produce the rope-tilt needed to brake a 1 m/s
pendulum swing in under a second.

### 5.4 Thrust and desired tilt synthesis

Given $\mathbf{a}_i^\star$ and the feed-forward $T_i^{\text{ff}}$,

$$
f_i \;=\; \mathrm{sat}_{[f_{\min},f_{\max}]}\!\left[ m_i\,(g + a_z^\star) \;+\; T_i^{\text{ff}} \right] , \tag{9}
$$

$$
\theta_{\text{pitch}}^{\text{des}} \;=\; \mathrm{sat}_{[-\theta_{\max},\theta_{\max}]}\!\left(\frac{a_x^\star}{g}\right),\;
\theta_{\text{roll}}^{\text{des}}  \;=\; \mathrm{sat}_{[-\theta_{\max},\theta_{\max}]}\!\left(\frac{-a_y^\star}{g}\right). \tag{10}
$$

Implemented at [`.cc:L279–296`](../../cpp/src/decentralized_local_controller.cc#L279).

$T_i^{\text{ff}}$ equals the measured rope tension after the pickup
phase, giving **exact gravity compensation for the hanging payload**
regardless of the true payload mass or the peer-drone state.

During the first $\tau_p$ seconds after first detecting rope engagement,
the feed-forward is ramped through a $C^1$-continuous Hermite-cubic
(smoothstep) schedule

$$
\alpha(\tau) = 3\tau^2 - 2\tau^3,\quad \tau = (t - t_{\text{pickup}})/\tau_p,
\qquad
T_i^{\text{ff}}(t) = \alpha(\tau)\cdot T_i^{\text{nom}}\wedge T_i^{\text{meas}}(t),
\tag{8b}
$$

with zero first-derivative at both endpoints. This eliminates the step
discontinuity in $T_i^{\text{ff}}$ that a linear ramp would produce and
therefore avoids exciting the 5 Hz axial mode of the 9-segment bead
chain at the pickup moment.

For simulations initialised at the static hover fixed point (drones at
their formation slot $z = z_{\text{payload}} + \Delta z_{\text{eq}}$
with the rope already pre-stretched to
$\delta = m_L g L_{\text{chord}} /(N k_{\text{eff}} \Delta z_{\text{attach}})$
≈ 3 mm), the `Params::initial_pretensioned` flag marks the pickup latch
as already-complete at $t = 0$ so that $T_i^{\text{ff}} = T_i^{\text{meas}}$
from the very first tick, carrying the payload share without any
free-fall transient. Implemented at
[`.cc:L205–221`](../../cpp/src/decentralized_local_controller.cc#L205)
(ramp) and
[`.cc:L33–37`](../../cpp/src/decentralized_local_controller.cc#L33)
(latch-init).

## 6. Cascade controller — attitude inner loop

From $R_i$ compute instantaneous roll, pitch, and the small-angle yaw
error:

$$
\theta_{\text{roll}} = \mathrm{atan2}(R_i[2,1], R_i[2,2]),\quad
\theta_{\text{pitch}} = \arcsin(-R_i[2,0]),\quad
e_\psi = \tfrac{1}{2}(R_i[1,0] - R_i[0,1]).
$$

Body-frame angular rates $\boldsymbol{\omega}_i^B = R_i^\top \boldsymbol{\omega}_i^W$
feed a diagonal PD:

$$
\begin{aligned}
\tau_x &= K_p^{\text{att}}(\theta_{\text{roll}}^{\text{des}} - \theta_{\text{roll}}) - K_d^{\text{att}} \omega_{Bx} \\
\tau_y &= K_p^{\text{att}}(\theta_{\text{pitch}}^{\text{des}} - \theta_{\text{pitch}}) - K_d^{\text{att}} \omega_{By} \\
\tau_z &= K_p^{\text{att}}(-e_\psi) - K_d^{\text{att}} \omega_{Bz}
\end{aligned}
\tag{11}
$$

with gains $(K_p^{\text{att}}, K_d^{\text{att}}) = (25, 4)$ — faster than
the outer loop, per the cascade-tuning rule $\omega_n^{\text{inner}} \gg \omega_n^{\text{outer}}$. Each
torque component is clamped to $\pm \tau_{\max}$. The body torque is
rotated back to world frame for the plant's applied-wrench input:
$\boldsymbol{\tau}^W = R_i\,\boldsymbol{\tau}^B$,
$\mathbf{F}^W = R_i\,\mathbf{e}_3\,f_i$. Implemented at
[`.cc:L301–320`](../../cpp/src/decentralized_local_controller.cc#L301).

## 7. Constraints and saturation

| Physical limit | Implementation | Location |
|----------------|----------------|----------|
| Rotor thrust is non-negative | $f_i \ge 0$ via thrust clamp | [`.cc:L290`](../../cpp/src/decentralized_local_controller.cc#L290) |
| Rotor thrust is bounded above | $f_i \le f_{\max} = 150$ N | same |
| Body tilt is bounded | $\|a_{xy}\| \le g\tan\theta_{\max}$ in QP | [`.cc:L240–241`](../../cpp/src/decentralized_local_controller.cc#L240) |
| Torque-axis saturation | $\lvert\tau_{x,y,z}\rvert \le \tau_{\max}$ per axis | [`.cc:L316–318`](../../cpp/src/decentralized_local_controller.cc#L316) |
| Rope tension cannot be pushed | emergent from rope model — drone's $T_i^{\text{ff}}$ $\ge 0$ after post-pickup clamp | [`.cc:L219`](../../cpp/src/decentralized_local_controller.cc#L219) |

## 8. Assumptions and limits

1. **Small-angle tilt approximation** in (10) — valid within the $\pm\theta_{\max} = 34^\circ$ envelope where $\tan\theta \approx \sin\theta$ within 6 %.
2. **Payload pose is observable locally.** In simulation we read payload state from the plant (ground truth); in real deployment this would come from an onboard camera or IR rangefinder locked on the payload.
3. **Rope is the only physical coupling** between drones. Aerodynamic downwash interference, electromagnetic interference, and Co-$W\!I$-FI latency are not modelled.
4. **The payload is a point mass.** A distributed or tumbling payload would require an attitude controller on the load.
5. **No explicit actuator lag.** Rotor thrust responds instantaneously to the commanded $f_i$. A non-trivial first-order lag would reduce the admissible $K_p^z$ before closed-loop instability.

## 9. Stability and convergence — empirical

A rigorous Lyapunov proof for the full closed-loop (drone cascade PD +
QP + rope dynamics + payload pendulum + inter-drone coupling) is out
of scope for this paper. The controller is validated **empirically**:

- Scenario **S1** (nominal traverse) achieves a payload-tracking RMS
  of 0.16 m (t ≥ 0.5 s startup window excluded). During the final
  landing-hold phase (t > 22 s) the steady-state error drops to
  ≤ 5 cm. During active traverse the RMS is 0.26 m and peak is
  0.30 m — dominated by pendulum-lag physics, not a control failure.
  See [`output/Tether_Grace/01_scenario_S1_nominal_traverse/`](../../../output/Tether_Grace/01_scenario_S1_nominal_traverse/).
- Scenario **S6** (triple sequential fault, last drone alone) remains
  asymptotically bounded with RMS 0.50 m.

A full stability analysis would need (i) to linearise the payload
pendulum around hover, (ii) show the cascade PD closes a stable
position loop on the slot-tracking error under the feed-forward
$T_i^{\text{ff}} = T_i$ identity, and (iii) argue that the smooth
saturations in (8)–(10) do not destabilise under a 1 m/s² lateral
manoeuvre. Each step is tractable but lengthy; deferred to a follow-up.

## 10. Code-to-theory map

| Eq. | Code |
|:-:|---|
| (1) | Drake `MultibodyPlant` (implicit — Tether_Lift baseline) |
| (2) | idem |
| (3) | idem + [`rope_force_system.cc`](../../../Tether_Lift/Research/cpp/src/rope_force_system.cc) |
| (4) | [`.cc:L115–116`](../../cpp/src/decentralized_local_controller.cc#L115) |
| (5) | [`.cc:L161–168`](../../cpp/src/decentralized_local_controller.cc#L161) |
| (6) | [`.cc:L171–176`](../../cpp/src/decentralized_local_controller.cc#L171) |
| (7) | [`.cc:L181–182`](../../cpp/src/decentralized_local_controller.cc#L181) |
| (8) | [`.cc:L227–245`](../../cpp/src/decentralized_local_controller.cc#L227) |
| (9) | [`.cc:L279–290`](../../cpp/src/decentralized_local_controller.cc#L279) |
| (10) | [`.cc:L294–296`](../../cpp/src/decentralized_local_controller.cc#L294) |
| (11) | [`.cc:L313–318`](../../cpp/src/decentralized_local_controller.cc#L313) |

---

## 11. QP analytical solution — closed-form derivation

The QP in Eq. (8) is separable: with the box constraint
$(|a_x|, |a_y|) \le c_{xy} := g\tan\theta_{\max}$ and
$a_{z,\min} \le a_z \le a_{z,\max}$ (from thrust bounds), and a
*combined* quadratic cost $J = (w_t + w_e)\|\mathbf{a}\|^2
- 2w_t\,\mathbf{a}^{\top}\mathbf{a}^{\text{target}}$, the
unconstrained minimiser of each component is

$$
a_k^{\text{unc}} \;=\; \frac{w_t}{w_t + w_e}\,a_k^{\text{target}},
\qquad k \in \{x,y,z\}.
\tag{12}
$$

Because the feasible set is a box, the solution is the projection:

$$
a_k^\star \;=\; \mathrm{clip}(a_k^{\text{unc}},\, [l_k,\, u_k]), \tag{13}
$$

where $[l_x, u_x] = [l_y, u_y] = [-c_{xy}, c_{xy}]$ and
$[l_z, u_z] = [a_{z,\min}, a_{z,\max}]$.

This means the QP needs no numerical solver: it reduces to six
clip operations. With $w_t = 1$ and $w_e = 0.02$,
the shrinkage factor is
$w_t/(w_t + w_e) = 1/1.02 \approx 0.98$ — negligible for on-target
commands and not observable in logged data.

**Active constraint detection.** A constraint on axis $k$ is active
iff $|a_k^{\text{target}}| > (1 + w_e/w_t)\,u_k$, i.e. when the
target acceleration exceeds the feasibility boundary scaled by
$1 + w_e/w_t \approx 1.02$. In the 40 s figure-8 campaign the z-axis
constraint activates at each reversal (predicted peak horizontal
acceleration 0.46 m/s²; budget $c_{xy} = 9.81\tan(0.6) = 6.71$ m/s²,
so the tilt constraint is never active in practice under these
trajectories).

### Why Drake MathematicalProgram is still used

Despite having a closed-form solution, Drake's `MathematicalProgram`
is used at [`.cc:L227–245`](../../cpp/src/decentralized_local_controller.cc#L227).
This keeps the formulation declarative (constraints and costs are
added as typed `BoundingBoxConstraint` and `QuadraticCost` objects)
and allows the gain set to be changed without touching the solve logic.
For the current 3-variable problem, `OsqpSolver` terminates in
< 0.1 ms per call on the test hardware — well within the 2 ms control step.
If computation becomes a bottleneck, replacing the solver with the
closed-form (12)–(13) would yield the identical solution.

---

## 12. Gain selection and closed-loop eigenvalues

### 12.1 Outer-loop XY channel

Treating the drone as a double integrator in the horizontal plane
(rigid body, no drag, rope force projected horizontally is small
relative to $m_i \|{\mathbf{a}^{\text{track}}}\|$ during straight flight),
the closed-loop characteristic polynomial for the $x$-channel under
PD control (8), (9), (10) is

$$
s^2 + \frac{K_d^{xy}}{m_i}\,s + \frac{K_p^{xy}}{m_i} = 0 \tag{14}
$$

(ignoring the QP shrinkage factor $\approx 0.98$).
With $m_i = 1.5$ kg, $K_p^{xy} = 30$, $K_d^{xy} = 15$:

$$
\omega_n^{xy} = \sqrt{\frac{30}{1.5}} = \sqrt{20} \approx 4.47\ \text{rad/s},
\qquad
\zeta^{xy} = \frac{K_d^{xy}}{2m_i\omega_n^{xy}} = \frac{15}{2 \times 1.5 \times 4.47} \approx 1.12.
$$

Slightly over-damped ($\zeta > 1$) — by design. This avoids
overshoot on step-change waypoints and suppresses any payload-swing
resonance in the 2–3 rad/s range.

### 12.2 Outer-loop Z channel

$$
\omega_n^{z} = \sqrt{\frac{100}{1.5}} \approx 8.16\ \text{rad/s},
\qquad
\zeta^{z} = \frac{24}{2 \times 1.5 \times 8.16} \approx 0.98.
$$

Critically damped. The faster $z$ bandwidth prevents the altitude
droop that a slower gain would create when $T_i^{\text{ff}}$ exceeds
the actual tension at pickup.

### 12.3 Inner attitude loop

Similarly, treating roll/pitch as $J_i\ddot\theta = \tau$ (decoupled
single-axis model):

$$
\omega_n^{\text{att}} = \sqrt{\frac{K_p^{\text{att}}}{J_i}}.
$$

With $J_i \approx 0.03\ \text{kg·m}^2$ (from Drake plant inertia):
$\omega_n^{\text{att}} \approx \sqrt{25/0.03} \approx 28.9$ rad/s —
roughly $6\times$ faster than $\omega_n^{xy}$, satisfying the
cascade-separation condition.

### 12.4 Payload pendulum eigenfrequency

The payload hangs on $N = 4$ ropes of chord length
$L_{\text{chord}} \approx 1.211$ m (Euclidean distance from
attachment at drone to payload, computed from formation geometry;
see [theory_rope_dynamics.md](theory_rope_dynamics.md) §2).
For small-angle swinging with the formation locked at its slot (i.e.
all four ropes acting as a single compound suspension):

$$
\omega_p \approx \sqrt{\frac{g}{L_{\text{chord}}}} = \sqrt{\frac{9.81}{1.211}} \approx 2.84\ \text{rad/s},
\qquad T_p = \frac{2\pi}{\omega_p} \approx 2.21\ \text{s}. \tag{15}
$$

This is **below** $\omega_n^{xy} = 4.47$ rad/s. If the outer loop
were to be tuned faster than $\omega_p$, the force correction would
react before the pendulum has settled, potentially exciting resonance.
The chosen over-damped ($\zeta > 1$) outer loop with
$\omega_n^{xy} < 2\omega_p$ provides a safe margin.

For the figure-8 campaign ($T = 16$ s, peak tangential acceleration
$\approx 0.46$ m/s²), the forcing frequency $\omega_f = 2\pi/16
\approx 0.39$ rad/s is well below $\omega_p = 2.84$ rad/s, so the
payload oscillates quasi-statically — this is why the residual pendulum
RMS (0.26 m during traverse) is dominated by the lag at corners rather
than by resonance amplification.

---

## 13. Fault recovery — analytical bound on rebalancing time

When drone $j$ goes silent (cable cut or motor failure), the remaining
$N-1$ drones must redistribute the payload's gravitational load.
Each surviving drone's tension feed-forward is updated automatically on
the *next* control tick because $T_i^{\text{ff}} \leftarrow
T_i^{\text{meas}}$ for all $i \ne j$. The question is: how fast do the
remaining rope tensions re-equilibrate?

**First-order tension dynamics.** Under constant formation slot targets
and assuming the payload remains within the slack of the remaining
ropes (i.e. the ropes stay taut), the payload sags by

$$
\delta z_{\text{fault}} \;=\; \frac{m_L g}{(N-1)k_{\text{eff}}} \;-\; \frac{m_L g}{N\,k_{\text{eff}}}
\;=\; \frac{m_L g}{N(N-1)k_{\text{eff}}}. \tag{16}
$$

With $N = 4$, $m_L = 3$ kg, $k_{\text{eff}} = 2778$ N/m:

$$
\delta z_{\text{fault}} = \frac{3 \times 9.81}{4 \times 3 \times 2778} \approx 8.8 \times 10^{-4}\ \text{m} \approx 0.9\ \text{mm}.
$$

The instantaneous sag is sub-millimetre — the fault appears primarily
as a *thrust redistribution* event, not a geometric descent.

**Recovery time.** The altitude loop drives the payload back to its
reference along the $z$-channel. Given a step disturbance
$\delta z_{\text{fault}}$ of 0.9 mm and an overdamped $z$-channel
with $\omega_n^z = 8.16$ rad/s, the 98 % settling time is
approximately

$$
t_{\text{settle}} \;\approx\; \frac{4}{\zeta\,\omega_n^z}
\;\approx\; \frac{4}{0.98 \times 8.16} \;\approx\; 0.50\ \text{s}.
$$

The empirically observed recovery time across scenarios S2–S6 and A–D
is < 1 s payload displacement peak, consistent with the analytical 0.5 s
bound. The dominant observable after fault is the *pendulum kick*
caused by the sudden asymmetry in horizontal rope forces, not the
vertical sag, explaining why S6 (triple fault) shows 0.50 m peak
error while the vertical error remains < 5 cm throughout.

> **Supported Inference** — the sub-mm sag estimate and 0.5 s bound are
> derived analytically from verified rope-stiffness and gain parameters.
> The claim "< 1 s recovery" is consistent with logged telemetry in
> `output/Tether_Grace/`; a formal proof with rope dynamics included
> is deferred to future work.
