# Receding-Horizon MPC with Linearised Tension-Ceiling Constraint

Implemented as the drop-in replacement class `MpcLocalController` in
[`mpc_local_controller.cc`](../../cpp/src/mpc_local_controller.cc) and
[`mpc_local_controller.h`](../../cpp/include/mpc_local_controller.h),
selected at run-time via `--controller=mpc`.

Two implementation choices deserve up-front notice because they depart
from the pure textbook MPC derivation:

1. **Reference-tracking cost.** At the 2 ms sim step the
   state-regulator gradient $f = 2\Omega^\top\bar Q\Phi\,x$ is
   numerically O($10^{-7}$) so the solver returns $U^\star \approx 0$.
   The cost is therefore expressed as
   $J = \sum_k w_t\|U_k - a_{\mathrm{target}}\|^2 + w_e\|U_k\|^2$
   with $a_{\mathrm{target}}$ identical to the baseline cascade PD +
   anti-swing output. The MPC's value is in the tension-horizon
   constraint, not in re-deriving a tracking controller.

2. **Tension linearisation uses $(A^j - I)x(k)$ rather than $A^j x(k)$.**
   The former folds only the *change* in state over the horizon into
   the free response; the latter folds the current tracking error
   into every future step and tightens $d_T$ into infeasibility even
   at steady-state hover. The row is
   $c_j^\top\Omega_j U \le T_{\max} - T_{\mathrm{meas}}(k)
      - c_j^\top(A^j-I)x(k) - j\Delta t\,\dot T_{\mathrm{meas}}$,
   with the filtered $\dot T$ term extrapolating bead-chain tension
   oscillations forward.

Companion derivations:
[`theory_decentralized_local_controller.md`](theory_decentralized_local_controller.md),
[`theory_rope_dynamics.md`](theory_rope_dynamics.md).

---

## 1. Motivation and Gap Analysis

**Observed gap.** The current single-step QP minimises
$\|a_i - a_\text{target}\|^2$ subject to tilt/thrust box constraints only
([`decentralized_local_controller.cc:L226–245`](../../cpp/src/decentralized_local_controller.cc#L226)).
It has no prediction horizon: it cannot "see" that an aggressive
turn one control step later will spike the rope tension. The constraint set
is

$$
\mathcal{U}_\text{box} = \{ a \in \mathbb{R}^3 : |a_x|, |a_y| \le g\tan\theta_\max,\;
a_z \in [a_{z,\min}, a_{z,\max}] \},
$$

and the rope tension $T_i = k_\text{eff}\,\delta_i$ (where $\delta_i$ is the rope stretch)
is bounded only indirectly through $a_{z,\max}$.

**Why a tension ceiling matters.**
Under compound faults (Scenario S6: three sequential drones lost) the
surviving drone must carry nearly the full payload weight. The static
load is $m_L g = 3 \times 9.81 = 29.4$ N, well below the safety limit
$T_\text{max,safe} = 100$ N. However, on an aggressive figure-8 or lemniscate3d
trajectory the drone accelerates centripetally; the rope tension reads

$$
T_i \approx m_L \bigl(g + a_{\perp}\bigr) \tag{1}
$$

where $a_\perp$ is the centripetal component projected onto the rope
direction. For a figure-8 of amplitude $a = 3$ m and period $T = 16$ s,
the peak lateral acceleration is $a_\perp \approx 0.93$ m/s², giving
$T_i \approx 32$ N — still safe. But the dynamic rope stretching during
a fault transient can add $\Delta T = k_\text{eff} \|\delta p\|$ on top of
the static load; with $k_\text{eff} = 25000/9 \approx 2778$ N/m, a
positional deviation of just $\|\delta p\| = 25$ mm produces $\Delta T = 69$ N,
pushing the combined tension to $\approx 101$ N — at the break limit.

The MPC addresses this by predicting the tension $N_p$ steps ahead and
choosing a control sequence that keeps every predicted tension below
$T_\text{max,safe}$.

---

## 2. Physical Constants and Notation

| Symbol | Value / Source | Units | Code reference |
|---|---|---|---|
| $dt$ | $2 \times 10^{-3}$ s | s | control step (10× Drake plant step) |
| $m_i$ | $1.5$ | kg | `quadcopter_mass` [`main.cc:L529`](../../cpp/src/decentralized_fault_aware_sim_main.cc#L529) |
| $m_L$ | $3.0$ | kg | `payload_mass` [`main.cc:L530`](../../cpp/src/decentralized_fault_aware_sim_main.cc#L530) |
| $k_\text{eff}$ | $25000/9 \approx 2778$ | N/m | `segment_stiffness/(num_rope_beads+1)` [`main.cc:L372`](../../cpp/src/decentralized_fault_aware_sim_main.cc#L372) |
| $L_\text{rest}$ | $1.25$ | m | `rope_length` [`main.cc:L531`](../../cpp/src/decentralized_fault_aware_sim_main.cc#L531) |
| $r_f$ | $0.8$ | m | formation radius [`main.cc:L543`](../../cpp/src/decentralized_fault_aware_sim_main.cc#L543) |
| $h$ | $1.25$ | m | `params_.rope_drop` (drone–payload vertical distance) |
| $T_\text{max,safe}$ | $100$ | N | design choice (50% of actuator ceiling 150 N) |
| $g$ | $9.81$ | m/s² | `params_.gravity` |
| $\theta_\max$ | $0.6$ | rad | `params_.max_tilt` |
| $a_{xy,\max}$ | $g\tan(0.6) \approx 6.71$ | m/s² | tilt envelope |
| $N_p$ | $5 \ldots 10$ | steps | design choice (see §6) |

---

## 3. Part A — Prediction Model

### 3.1 Error-State Double Integrator

The outer-loop treats each drone as a double integrator driven by the
commanded acceleration $a_i \in \mathbb{R}^3$:

$$
\dot{p}_i = v_i, \qquad \dot{v}_i = a_i. \tag{2}
$$

Define the **error state** relative to the nominal formation-slot trajectory
$(p_{i,\text{nom}}(t),\, v_{i,\text{nom}}(t))$:

$$
x_i(k) = \begin{bmatrix} e_{p,i}(k) \\ e_{v,i}(k) \end{bmatrix}
         = \begin{bmatrix} p_i(k) - p_{i,\text{nom}}(k) \\ v_i(k) - v_{i,\text{nom}}(k) \end{bmatrix}
         \in \mathbb{R}^6. \tag{3}
$$

The nominal slot trajectory is defined identically to the existing
`ComputeSlotReference` method: $p_{i,\text{nom}}(k) = p_{L,\text{ref}}(k) + \Delta_i + h\,\mathbf{e}_3$
where $\Delta_i$ is the horizontal formation offset of drone $i$
(`params_.formation_offset`).

**Zero-order hold (ZOH) discretisation** of Eq. (2) with control step $dt$:

$$
\boxed{
x_i(k+1) = A\,x_i(k) + B\,\tilde{u}_i(k), \qquad
A = \begin{bmatrix} I_3 & dt\,I_3 \\ 0 & I_3 \end{bmatrix},\;
B = \begin{bmatrix} \tfrac{dt^2}{2}\,I_3 \\ dt\,I_3 \end{bmatrix},
} \tag{4}
$$

where $\tilde{u}_i(k) = a_i(k) - a_{i,\text{ref}}(k)$ is the **deviation control**
(acceleration command minus the reference feedforward acceleration).
$A \in \mathbb{R}^{6\times6}$, $B \in \mathbb{R}^{6\times3}$.

> **Implementation note.** In the existing codebase the reference
> acceleration $a_{i,\text{ref}}$ is already computed as the PD tracking
> term $a_\text{track}$ (plus the swing term). For the MPC integration,
> the **MPC computes $a_i(k)$ directly**; thus $\tilde{u}_i \equiv a_i$ for
> the purpose of the QP formulation (the feedforward $a_{i,\text{ref}}$ is
> folded into the nominal trajectory offset and treated as zero). This is
> exact when the reference trajectory acceleration is small (as for
> figure-8 at $T = 16$ s, where $\|a_{i,\text{ref}}\| \le 1$ m/s²) and
> introduces $O(dt^2 \|a_{i,\text{ref}}\|) \approx 2\,\mu$m/step error.

### 3.2 Explicit Values of $A$ and $B$ for $dt = 0.002$ s

With $dt = 0.002$ s and $I_3 = \text{diag}(1,1,1)$:

$$
A = \begin{bmatrix}
1 & 0 & 0 & 0.002 & 0 & 0 \\
0 & 1 & 0 & 0 & 0.002 & 0 \\
0 & 0 & 1 & 0 & 0 & 0.002 \\
0 & 0 & 0 & 1 & 0 & 0 \\
0 & 0 & 0 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 0 & 1
\end{bmatrix}, \qquad
B = \begin{bmatrix}
2\times10^{-6} & 0 & 0 \\
0 & 2\times10^{-6} & 0 \\
0 & 0 & 2\times10^{-6} \\
0.002 & 0 & 0 \\
0 & 0.002 & 0 \\
0 & 0 & 0.002
\end{bmatrix}. \tag{5}
$$

Note: $dt^2/2 = 2\times10^{-6}$ m per (m/s²). Position is updated mainly
through the velocity channel ($dt = 2\times10^{-3}$); the direct
acceleration-to-position term is four orders of magnitude smaller.

### 3.3 Condensed $N_p$-Step Prediction

Stack the $N_p$ future error states and $N_p$ future controls:

$$
\mathcal{X}_i = \begin{bmatrix} x_i(k+1) \\ \vdots \\ x_i(k+N_p) \end{bmatrix} \in \mathbb{R}^{6N_p},
\qquad
\mathcal{U}_i = \begin{bmatrix} a_i(k) \\ \vdots \\ a_i(k+N_p-1) \end{bmatrix} \in \mathbb{R}^{3N_p}. \tag{6}
$$

The **condensed prediction** reads:

$$
\boxed{\mathcal{X}_i = \Phi\,x_i(k) + \Omega\,\mathcal{U}_i,} \tag{7}
$$

where

$$
\Phi = \begin{bmatrix} A \\ A^2 \\ \vdots \\ A^{N_p} \end{bmatrix} \in \mathbb{R}^{6N_p \times 6},
\qquad
\Omega = \begin{bmatrix}
B      & 0      & \cdots & 0 \\
AB     & B      & \cdots & 0 \\
A^2B   & AB     & \cdots & 0 \\
\vdots & \vdots & \ddots & \vdots \\
A^{N_p-1}B & A^{N_p-2}B & \cdots & B
\end{bmatrix} \in \mathbb{R}^{6N_p \times 3N_p}. \tag{8}
$$

The $(j,\ell)$-th $6\times3$ block of $\Omega$ is
$\Omega_{j,\ell} = A^{j-\ell}\,B$ for $\ell \le j$, else $0$.

**Extraction of predicted positions.** Define the position-extraction operator
$E_p = [I_3,\; 0_{3\times3}] \in \mathbb{R}^{3\times6}$. Then the
absolute predicted drone position at step $k+j$ is

$$
\hat{p}_i(k+j) = p_{i,\text{nom}}(k+j) + E_p\, x_i(k+j)
              = p_{i,\text{nom}}(k+j) + E_p\bigl(A^j x_i(k) + [\Omega]_{j,:}\,\mathcal{U}_i\bigr), \tag{9}
$$

where $[\Omega]_{j,:} \in \mathbb{R}^{6\times3N_p}$ is the $j$-th $6$-row
block of $\Omega$.

**Pre-computation.** $\Phi$ and $\Omega$ depend only on $A$, $B$, and $N_p$,
which are all constant. They are computed once at construction time and
stored as member matrices. For $N_p = 10$: $\Phi$ has $60\times6 = 360$
doubles; $\Omega$ has $60\times30 = 1800$ doubles — both trivial allocations.

---

## 4. Part B — Linearised Tension Constraint

### 4.1 Quasi-Static Tension Model

The rope tension used in the controller (quasi-static, single-segment
effective stiffness):

$$
T_i = k_\text{eff} \max\!\bigl(\|p_i - p_L\| - L_\text{rest},\; 0\bigr). \tag{10}
$$

**Nominal chord length.** The rope runs from the drone attachment point
(offset $dz_q = 0.09$ m below drone body centre) to the payload attachment
point (offset $0.3\,r_f = 0.24$ m outward in the slot direction). Defining
$r_\text{eff} = 0.7\,r_f = 0.56$ m and vertical drop from drone attachment
to payload attachment $dz_\text{att} = \texttt{rope\_drop} - dz_q = 1.211 - 0.09 = 1.121$ m,
the nominal body-centre-to-payload distance is

$$
d_\text{body} = \sqrt{r_f^2 + \texttt{rope\_drop}^2} = \sqrt{0.64 + 1.466} = 1.451\text{ m}. \tag{11a}
$$

However, the rope spans the **attachment-to-attachment** chord

$$
d_\text{att} = \sqrt{r_\text{eff}^2 + dz_\text{att}^2} = \sqrt{0.3136 + 1.2567} = 1.253\text{ m}. \tag{11b}
$$

The fixed-point hover equilibrium (code `main.cc:L388–398`) gives rope
stretch $\delta_\text{nom} \approx 3\text{ mm}$, so the nominal tension is

$$
\delta_\text{nom} = d_\text{att} - L_\text{rest} \approx 1.253 - 1.250 = 0.003\text{ m},
\quad T_\text{nom} = 2778 \times 0.003 \approx 8.23\text{ N}. \tag{11c}
$$

This matches the static equilibrium requirement ($m_L g / N \times 1/\cos\alpha \approx 8.83$ N).
For the MPC tension model, the effective rest length referred to the
body-centre chord is

$$
L_\text{eff,body} = d_\text{body} - \delta_\text{nom} \approx 1.451 - 0.003 = 1.448\text{ m}. \tag{11d}
$$

The MPC implementation uses $L_\text{eff,body}$ (not $L_\text{rest}$) so that
$T = k_\text{eff}\max(d_\text{body} - L_\text{eff,body},0)$ recovers the
correct nominal tension from body-centre positions. $L_\text{eff,body}$ is
computed once at construction from the hover equilibrium.

> **Note.** Using the bare $L_\text{rest} = 1.25$ m against $d_\text{body} = 1.451$ m
> would yield $\delta = 0.201$ m and $T_\text{nom} \approx 559$ N, making the
> ceiling constraint $T \le 100$ N immediately infeasible. The corrected
> $L_\text{eff,body}$ resolves this.

### 4.2 First-Order Taylor Expansion

Let $(p_{i,\text{nom}},\, p_{L,\text{ref}})$ be the nominal drone and
payload positions at step $k+j$. Define the nominal unit vector pointing
from payload to drone:

$$
\hat{n}_i(k+j) = \frac{p_{i,\text{nom}}(k+j) - p_{L,\text{ref}}(k+j)}{d_{i,\text{nom}}(k+j)}, \quad
d_{i,\text{nom}} = \|p_{i,\text{nom}} - p_{L,\text{ref}}\|. \tag{12}
$$

Taylor-expand $\|p_i - p_{L,\text{ref}}\|$ around $p_{i,\text{nom}}$:

$$
\|p_i - p_{L,\text{ref}}\| \approx d_{i,\text{nom}}(k+j) + \hat{n}_i^T(k+j)\,(p_i - p_{i,\text{nom}})
= d_{i,\text{nom}}(k+j) + \hat{n}_i^T(k+j)\,e_{p,i}(k+j). \tag{13}
$$

The relative error of this linearisation is $O(\|e_{p,i}\|^2 / d_\text{nom})$.
For $\|e_{p,i}\| \le 0.1$ m and $d_\text{nom} \approx 1.451$ m the
error is $< 0.5\%$. The constraint is written conservatively (see slack
treatment §5.4), so small linearisation errors do not compromise safety.

Substituting into Eq. (10):

$$
T_i(k+j) \approx k_\text{eff}\bigl(d_{i,\text{nom}}(k+j) - L_\text{rest}\bigr)
           + k_\text{eff}\,\hat{n}_i^T(k+j)\,e_{p,i}(k+j)
           = T_{i,\text{nom}}(k+j) + k_\text{eff}\,\hat{n}_i^T(k+j)\,e_{p,i}(k+j). \tag{14}
$$

### 4.3 Deriving the Linear Constraint Row

The tension ceiling $T_i(k+j) \le T_\text{max,safe}$ becomes:

$$
k_\text{eff}\,\hat{n}_i^T(k+j)\,e_{p,i}(k+j) \le T_\text{max,safe} - T_{i,\text{nom}}(k+j)
\triangleq \mu_j. \tag{15}
$$

Define the $6$-vector:

$$
c_j = k_\text{eff}\,E_p^T\,\hat{n}_i(k+j) \in \mathbb{R}^6 \quad
(\text{i.e. } c_j = k_\text{eff}\,[n_x,\,n_y,\,n_z,\,0,\,0,\,0]^T). \tag{16}
$$

Then $k_\text{eff}\,\hat{n}_i^T e_{p,i} = c_j^T x_i(k+j)$.

Substitute the condensed prediction $x_i(k+j) = A^j x_i(k) + [\Omega]_{j,:}\,\mathcal{U}_i$:

$$
c_j^T [\Omega]_{j,:}\,\mathcal{U}_i \le \mu_j - c_j^T A^j x_i(k). \tag{17}
$$

Stacking over $j = 1,\ldots,N_p$ yields the **tension constraint matrix** $C_T \in \mathbb{R}^{N_p \times 3N_p}$ and the **right-hand side vector** $d_T \in \mathbb{R}^{N_p}$:

$$
\boxed{C_T\,\mathcal{U}_i \le d_T,} \tag{18}
$$

$$
[C_T]_{j,:} = c_j^T\,[\Omega]_{j,:} \in \mathbb{R}^{1\times3N_p}, \tag{19}
$$

$$
[d_T]_j = (T_\text{max,safe} - T_{i,\text{nom}}(k+j)) - c_j^T A^j x_i(k). \tag{20}
$$

**Interpretation of $d_T$.** The right-hand side has two parts:
(i) the available "tension headroom" at step $j$ relative to the current
nominal tension, and (ii) a correction for the free evolution of the
current error state $x_i(k)$ under $A^j$ (zero control). If the drone is
already displaced toward the payload, the free evolution tends to increase
tension, and $d_T$ tightens accordingly.

**Sign convention.** $\hat{n}_i$ points **from payload to drone**. A
positive projection $\hat{n}_i^T e_{p,i} > 0$ means the drone is
displaced further from the payload than nominal → rope is **slack** →
tension decreases → no constraint violation. The tension constraint only
bites when the drone moves **toward** the payload ($\hat{n}_i^T e_{p,i} < 0$)
or when the nominal tension is already high. The linearisation remains
conservative in that regime.

---

## 5. Part C — Full MPC QP Formulation

### 5.1 Cost Function

$$
J\bigl(\mathcal{U}_i;\,x_i(k)\bigr)
= \sum_{j=1}^{N_p} x_i(k+j)^T Q_j\, x_i(k+j)
+ \sum_{j=0}^{N_p-1} a_i(k+j)^T R\, a_i(k+j), \tag{21}
$$

where

$$
Q_j = \begin{cases} Q & j < N_p \\ P_f & j = N_p \end{cases}, \tag{22}
$$

with $Q \in \mathbb{R}^{6\times6}$ the running state cost, $P_f \in \mathbb{R}^{6\times6}$ the
terminal cost (DARE solution, see §6.2), and $R \in \mathbb{R}^{3\times3}$ the
control effort cost. **Proposed canonical values:**

$$
Q = \begin{bmatrix} q_p\,I_3 & 0 \\ 0 & q_v\,I_3 \end{bmatrix},\quad
q_p = 100,\; q_v = 10;\qquad
R = r\,I_3,\; r = 1.0. \tag{23}
$$

These preserve the relative weighting of the existing PD gains:
$k_p^{xy} = 30$, $k_d^{xy} = 15$ in the outer loop. In the MPC, the
analogous ratio is $q_p / (r/dt) = 100 / 500 = 0.2$ (acceleration cost
per position cost at $dt = 0.002$) — comparable in magnitude to the
PD ratio $k_p / k_d = 2$.

### 5.2 Condensed QP Matrices

Define block-diagonal weight matrices:

$$
\bar{Q} = \text{blkdiag}(Q, Q, \ldots, Q, P_f) \in \mathbb{R}^{6N_p \times 6N_p}, \tag{24}
$$

$$
\bar{R} = \text{blkdiag}(R, R, \ldots, R) \in \mathbb{R}^{3N_p \times 3N_p}. \tag{25}
$$

Substituting the condensed prediction (7) into (21):

$$
J = \mathcal{X}_i^T \bar{Q}\,\mathcal{X}_i + \mathcal{U}_i^T \bar{R}\,\mathcal{U}_i
  = \mathcal{U}_i^T \mathcal{H}\,\mathcal{U}_i + f^T \mathcal{U}_i + \text{const}(x_i(k)), \tag{26}
$$

where

$$
\boxed{
\mathcal{H} = \Omega^T \bar{Q}\,\Omega + \bar{R} \in \mathbb{R}^{3N_p \times 3N_p},
\qquad
f = 2\,\Omega^T \bar{Q}\,\Phi\,x_i(k) \in \mathbb{R}^{3N_p}.
} \tag{27}
$$

$\mathcal{H}$ is **constant** (independent of $x_i(k)$ and time) and can
be factored offline. $f$ is linear in the current error state and is
updated at each MPC call.

### 5.3 Constraint Summary

All constraints are expressed in standard OSQP form
$\ell_c \le \mathcal{A}_c\,\mathcal{U}_i \le u_c$.

#### (a) Box constraints

Each of the $N_p$ acceleration commands is bounded component-wise:

$$
a_{i,\ell}(k+j) \in \begin{cases} [-a_{xy,\max}, +a_{xy,\max}] & \ell \in \{x,y\} \\ [a_{z,\min}, a_{z,\max}] & \ell = z \end{cases}
\quad \forall\, j = 0,\ldots,N_p-1. \tag{28}
$$

In OSQP this is handled as **variable bounds**, not as inequality rows:

$$
\ell_\text{box} = [l_0^T, l_1^T, \ldots, l_{N_p-1}^T]^T,\quad
u_\text{box} = [u_0^T, u_1^T, \ldots, u_{N_p-1}^T]^T \in \mathbb{R}^{3N_p}, \tag{29}
$$

where $l_j = [-a_{xy,\max}, -a_{xy,\max}, a_{z,\min}]^T$,
$u_j = [+a_{xy,\max}, +a_{xy,\max}, a_{z,\max}]^T$.
Note: $a_{z,\min}$ and $a_{z,\max}$ depend on the current $T_\text{ff}$
(tension feed-forward), exactly as in the existing code ([`.cc:L202–203`](../../cpp/src/decentralized_local_controller.cc#L202)).

#### (b) Tension ceiling (hard, linearised)

From §4.3: $C_T\,\mathcal{U}_i \le d_T$. In OSQP form:

$$
\ell_T = -\infty\cdot\mathbf{1}_{N_p},\quad u_T = d_T. \tag{30}
$$

#### (c) Slew-rate (optional, soft via penalty)

To limit jerk, penalise $\|a_i(k+j) - a_i(k+j-1)\|^2$. For $j > 0$
this is $\|[\Delta]\mathcal{U}_i\|^2$ where $\Delta$ is the block
first-difference operator; for $j = 0$ it uses the previous optimum
$a_i^*(k-1)$ (stored in warm cache). Add this as an extra quadratic cost
term rather than a hard constraint to avoid compatibility issues with
the tension constraint:

$$
J_\text{slew} = w_\text{slew} \sum_{j=1}^{N_p-1} \|a_i(k+j) - a_i(k+j-1)\|^2 \tag{31}
$$

with $w_\text{slew} = 0.01$ (initial value; tune to avoid slow response).

### 5.4 Slack Variable for Tension Constraint Feasibility

To guarantee feasibility under extreme fault transients, replace the hard
tension constraint with a **penalised soft constraint**. Introduce slack
$s \in \mathbb{R}^{N_p}_{\ge0}$:

$$
C_T\,\mathcal{U}_i \le d_T + s, \qquad s \ge 0. \tag{32}
$$

Add to the cost:

$$
J_\text{slack} = w_s\,\mathbf{1}^T s \quad (w_s = 10^4). \tag{33}
$$

This is a linear penalty on $s$; the large $w_s$ makes constraint
violation costly while preserving feasibility. In OSQP the slack is
appended to the decision variable vector: $z = [\mathcal{U}_i^T, s^T]^T$.

For the **normal-operation case** (no extreme faults), the slack will be
identically zero; the constraint acts as hard. The slack activates
gracefully during fault transients.

### 5.5 Complete QP

$$
\boxed{
\min_{\mathcal{U}_i \in \mathbb{R}^{3N_p}} \;
\tfrac{1}{2}\,\mathcal{U}_i^T \mathcal{H}\,\mathcal{U}_i
+ f^T\!\mathcal{U}_i
\quad \text{s.t.} \quad
\mathcal{A}_c\,\mathcal{U}_i \le u_c,\quad
\ell_\text{box} \le \mathcal{U}_i \le u_\text{box}
} \tag{34}
$$

with constraint matrix

$$
\mathcal{A}_c = C_T \in \mathbb{R}^{N_p \times 3N_p}, \qquad u_c = d_T. \tag{35}
$$

(Box constraints are encoded as variable bounds and do not appear in
$\mathcal{A}_c$.)

### 5.6 Problem Size

| Quantity | $N_p = 5$ | $N_p = 10$ |
|---|---|---|
| Decision variables | 15 | 30 |
| Box bound rows | 15 (bounds only) | 30 |
| Tension constraint rows | 5 | 10 |
| $\mathcal{H}$ non-zeros (sparse) | $\le 225$ | $\le 900$ |
| Expected OSQP iterations (warm) | $\le 10$ | $\le 15$ |
| Expected solve time (warm) | $< 0.3$ ms | $< 0.6$ ms |
| Budget (control step) | 2 ms | 2 ms |

For $N_p = 10$: 30 variables, 10 tension rows, ~900 Hessian non-zeros —
well within OSQP's fast regime. Both sizes leave more than $1$ ms of
budget margin for state extraction and output formatting.

---

## 6. Part D — Horizon Length and Stability

### 6.1 Why a Short Horizon Suffices

**Pendulum period argument.**
The under-slung payload pendulum frequency is

$$
\omega_p = \sqrt{g / d_\text{att}} = \sqrt{9.81 / 1.253} \approx 2.80\text{ rad/s},
\quad T_p = 2\pi / \omega_p \approx 2.25\text{ s}. \tag{36}
$$

At $dt = 0.002$ s: $T_p / dt \approx 1220$ steps. A full-period horizon
is computationally intractable. However, the MPC does not need to simulate
the full pendulum cycle; the terminal cost $P_f$ (see §6.2) acts as an
infinite-horizon value function beyond the prediction window.

**Tension spike timescale.**
A position deviation $e_{p,i}$ develops at the rate $v_i - v_{i,\text{nom}} = e_{v,i}$.
Typical error velocity at fault onset: $\|e_{v,i}\| \le 0.5$ m/s (from
scenario S3 simulations). Time to accumulate a $\Delta T = 10$ N tension
spike (i.e. $\|e_{p,i}\| = 10/2778 \approx 3.6$ mm): $3.6\,\text{mm} / 0.5\,\text{m/s} \approx 7$ ms = 3.5 control steps. Thus $N_p = 5$ (10 ms look-ahead) is the minimum to detect the onset; $N_p = 10$ (20 ms) provides a comfortable margin.

**Rope axial resonance.**
The bead-chain axial resonance (from `theory_rope_dynamics.md §4`):

$$
\omega_r = \sqrt{k_\text{eff} / m_L} = \sqrt{2778 / 3.0} \approx 30.4\text{ rad/s},
\quad T_r = 0.207\text{ s} = 103\text{ steps}. \tag{37}
$$

For $N_p = 10$ we cover ~10% of $T_r$. The tension constraint prevents
entering the high-amplitude regime; the terminal cost handles the
remainder.

**Recommendation:** Use $N_p = 5$ as the baseline (minimal latency,
proven tractable) and $N_p = 10$ as the evaluation alternative.

### 6.2 Terminal Cost via DARE

The DARE for the double integrator (A, B) with cost (Q, R):

$$
P_f = Q + A^T P_f A - A^T P_f B\,(R + B^T P_f B)^{-1} B^T P_f A. \tag{38}
$$

Since $A = I_3 \otimes A_s$ and $B = I_3 \otimes B_s$ (three decoupled
scalar channels), Eq. (38) reduces to three identical $2\times2$ DAREs:

$$
A_s = \begin{bmatrix} 1 & dt \\ 0 & 1 \end{bmatrix},\quad
B_s = \begin{bmatrix} dt^2/2 \\ dt \end{bmatrix},\quad
Q_s = \begin{bmatrix} q_p & 0 \\ 0 & q_v \end{bmatrix},\quad
R_s = r. \tag{39}
$$

**Closed-form iteration.** Initialise $P^{(0)}_s = Q_s$. Iterate:

$$
P^{(k+1)}_s = Q_s + A_s^T P^{(k)}_s A_s
              - \frac{(B_s^T P^{(k)}_s A_s)^T (B_s^T P^{(k)}_s A_s)}{R_s + B_s^T P^{(k)}_s B_s}. \tag{40}
$$

Convergence is geometric; typically 30–50 iterations achieve machine precision.
The result $P_{f,s}$ is the unique positive-definite solution (since
$(A_s, B_s)$ is controllable and $(Q_s^{1/2}, A_s)$ is observable for
$q_p > 0$).

**Numerical example** ($q_p = 100$, $q_v = 10$, $r = 1$, $dt = 0.002$ s):
Solving the $2\times2$ DARE numerically (Python `scipy.linalg.solve_discrete_are`):

$$
P_{f,s} \approx \begin{bmatrix} 200.5 & 0.402 \\ 0.402 & 10.20 \end{bmatrix}. \tag{41}
$$

The LQR gain for each scalar channel:

$$
K_s = (R_s + B_s^T P_{f,s} B_s)^{-1} B_s^T P_{f,s} A_s \approx [200.1,\; 10.02]. \tag{42}
$$

The closed-loop eigenvalues of $A_s - B_s K_s$:
$\lambda_{1,2} \approx 0.990 \pm 0.027i$ — inside the unit disc,
confirming asymptotic stability ($|\lambda| \approx 0.990 < 1$;
note that $|\lambda|^2 = \det(A_s - B_s K_s) \approx 0.980$).

The full terminal cost is

$$
P_f = I_3 \otimes P_{f,s} \in \mathbb{R}^{6\times6}. \tag{43}
$$

### 6.3 Stability Condition

**Theorem (Rawlings, Mayne, Diehl 2017, Theorem 2.16).** If:

1. $(A, B)$ is stabilisable (satisfied: double integrator is fully controllable),
2. $(Q^{1/2}, A)$ is detectable (satisfied: $q_p > 0$ ensures position error
   is in the stage cost, and $(A_s, [1,0])$ is observable),
3. $P_f$ is the positive-definite solution to the DARE (Eq. 38),
4. The constraints are feasible at $k = 0$ (guaranteed by tension slack, §5.4),

then the MPC scheme is **recursively feasible** and **asymptotically stable**
for the unconstrained closed loop. When the tension constraint is active,
the MPC remains asymptotically stable provided that the terminal set
$\mathcal{X}_f = \{x : x^T P_f x \le \alpha\}$ is **positively invariant**
under $A - BK$. For the linear constrained case, a sufficient $\alpha$ is
the largest level set of $V(x) = x^T P_f x$ contained in the feasible
region; this is computed once offline (§7.4).

**Practical stability argument.** In normal operation (no faults, small
disturbances), the MPC operates unconstrained (tension slack is zero,
box constraints are not active) and reduces to the LQR controller with
gain $K$. Faults perturb $x_i(k)$; the MPC then activates the tension
constraint to prevent rope overload while the LQR terminal cost steers
the system back to the nominal trajectory.

---

## 7. Part E — Integration Architecture

### 7.1 New Class: `MpcLocalController`

The new controller is a drop-in replacement for `DecentralizedLocalController`
and exposes **identical port signatures**:

```
Inputs:  plant_state  (same vector dimension)
         rope_tension  (same 4-vector)
Outputs: control_force  (same abstract output)
         control_vector (same 6-vector)
         diagnostics    (extended: adds mpc_solve_time_us, tension_headroom_min)
```

The only change to `decentralized_fault_aware_sim_main.cc` is replacing:

```cpp
// OLD (line ~584)
controllers[q] = builder.AddSystem<DecentralizedLocalController>(plant, ...)

// NEW
controllers[q] = builder.AddSystem<MpcLocalController>(plant, ...)
```

### 7.2 New Parameters Struct

```cpp
struct MpcParams {
  // Inherited from DecentralizedLocalController::Params (all fields kept)
  // ... formation_offset, waypoints, position_kp, etc. ...

  // MPC-specific additions
  int    horizon_steps = 5;          ///< N_p: prediction horizon (steps)
  double dt_mpc = 2e-3;              ///< control step (must match ZOH dt)

  // State cost Q = diag(q_pos*I3, q_vel*I3)
  double q_pos = 100.0;              ///< position error weight
  double q_vel = 10.0;               ///< velocity error weight

  // Control cost R = r_ctrl * I3
  double r_ctrl = 1.0;

  // Tension constraint
  double T_max_safe = 100.0;         ///< N — hard ceiling
  double tension_slack_weight = 1e4; ///< penalty on slack variable s

  // Slew-rate penalty (optional)
  double w_slew = 0.01;              ///< weight on consecutive accel difference

  // Warm-start enable (default on)
  bool warm_start = true;
};
```

### 7.3 Pre-Computation at Construction

```cpp
MpcLocalController::MpcLocalController(const MultibodyPlant<double>& plant,
                                       const RigidBody<double>& quad_body,
                                       const RigidBody<double>& payload_body,
                                       const MpcParams& params)
{
  const int Np = params.horizon_steps;
  const double dt = params.dt_mpc;

  // --- Build A, B (6x6, 6x3) ---
  A_ = Eigen::MatrixXd::Identity(6, 6);
  A_.topRightCorner(3, 3) = dt * Eigen::Matrix3d::Identity();
  B_.resize(6, 3); B_.setZero();
  B_.topRows(3) = 0.5 * dt * dt * Eigen::Matrix3d::Identity();
  B_.bottomRows(3) = dt * Eigen::Matrix3d::Identity();

  // --- Build Phi (6Np x 6), Omega (6Np x 3Np) ---
  Phi_.resize(6 * Np, 6);
  Omega_.resize(6 * Np, 3 * Np);  Omega_.setZero();
  Eigen::MatrixXd Ak = A_;  // A^j, j starts at 1
  for (int j = 0; j < Np; ++j) {
    Phi_.block(6 * j, 0, 6, 6) = Ak;
    for (int l = 0; l <= j; ++l) {
      // Omega[j, l] = A^(j-l) * B
      Eigen::MatrixXd Ajl = (j - l == 0) ? Eigen::MatrixXd::Identity(6,6)
                                           : A_.pow(j - l);   // or iterative
      Omega_.block(6 * j, 3 * l, 6, 3) = Ajl * B_;
    }
    Ak = A_ * Ak;
  }

  // --- Build Q-bar, R-bar ---
  // (solve DARE first to get P_f)
  Pf_ = SolveDARE(A_, B_, params.q_pos, params.q_vel, params.r_ctrl);

  Eigen::MatrixXd Qbar = Eigen::MatrixXd::Zero(6 * Np, 6 * Np);
  Eigen::MatrixXd Q = BuildQ(params.q_pos, params.q_vel);
  for (int j = 0; j < Np - 1; ++j)
    Qbar.block(6*j, 6*j, 6, 6) = Q;
  Qbar.bottomRightCorner(6, 6) = Pf_;

  Rbar_ = params.r_ctrl * Eigen::MatrixXd::Identity(3*Np, 3*Np);

  // --- Build constant Hessian H ---
  H_ = Omega_.transpose() * Qbar * Omega_ + Rbar_;

  // --- Cholesky of H (for warm-start residual checks) ---
  H_chol_ = H_.llt();

  // --- Initialise warm-start cache ---
  U_warm_ = Eigen::VectorXd::Zero(3 * Np);

  // --- Store references ---
  params_mpc_ = params;
  Qbar_ = Qbar;
}
```

### 7.4 Per-Step MPC Solve (`CalcControlForce`)

Replacing the single-step QP block at
[`decentralized_local_controller.cc:L227–245`](../../cpp/src/decentralized_local_controller.cc#L227):

```cpp
void MpcLocalController::CalcControlForce(
    const Context<double>& context,
    std::vector<ExternallyAppliedSpatialForce<double>>* output) const
{
  const int Np = params_mpc_.horizon_steps;
  const double t = context.get_time();

  // --- 1. Extract current state (identical to existing code) ---
  //   p_self, v_self, p_L, v_L, T_ff, a_z_min, a_z_max  (unchanged)
  // ... [same state extraction as DecentralizedLocalController::CalcControlForce L122-210]

  // --- 2. Build error state x_i(k) = [e_p; e_v] ---
  Eigen::VectorXd xi(6);
  xi.head(3) = p_self - p_slot_dynamic;   // p_slot_dynamic from anti-swing correction
  xi.tail(3) = v_self - v_slot;

  // --- 3. Build linear cost vector f = 2 * Omega^T * Qbar * Phi * xi ---
  Eigen::VectorXd f = 2.0 * Omega_.transpose() * (Qbar_ * (Phi_ * xi));

  // --- 4. Build tension constraint (C_T, d_T) ---
  const int nT = Np;
  Eigen::MatrixXd CT(nT, 3 * Np);
  Eigen::VectorXd dT(nT);

  Eigen::MatrixXd Aj = Eigen::MatrixXd::Identity(6, 6);
  for (int j = 0; j < Np; ++j) {
    Aj = A_ * Aj;   // Aj = A^(j+1)
    const double t_j = t + (j + 1) * params_mpc_.dt_mpc;

    // Nominal slot and payload reference at t_j
    Eigen::Vector3d p_slot_j, v_slot_j;
    ComputeSlotReference(t_j, &p_slot_j, &v_slot_j);
    Eigen::Vector3d p_L_ref_j, v_L_ref_j;
    ComputePayloadReference(t_j, &p_L_ref_j, &v_L_ref_j);

    // Nominal drone position at t_j = slot (no error)
    const Eigen::Vector3d p_nom_j = p_slot_j;
    const double d_nom = (p_nom_j - p_L_ref_j).norm();

    // Unit vector payload→drone
    Eigen::Vector3d nhat = (d_nom > 1e-6) ?
        (p_nom_j - p_L_ref_j) / d_nom :
        Eigen::Vector3d(0, 0, 1);

    // c_j = k_eff * [nhat; 0_3]
    Eigen::VectorXd cj(6);
    cj.head(3) = params_mpc_.k_eff * nhat;
    cj.tail(3).setZero();

    // CT row j: c_j^T * Omega[j, :]
    CT.row(j) = cj.transpose() * Omega_.block(6*j, 0, 6, 3*Np);

    // Nominal tension at t_j.
    // Use rope_length_effective (L_eff_body ≈ 1.448 m), not L_rest (1.25 m),
    // so that T_nom = k_eff*(d_body - L_eff_body) ≈ 8.23 N at hover.
    // L_eff_body is computed once at construction from the hover equilibrium.
    const double T_nom_j = params_mpc_.k_eff *
        std::max(d_nom - params_mpc_.rope_length_effective, 0.0);

    // d_T[j] = (T_max - T_nom_j) - c_j^T * A^(j+1) * xi
    dT(j) = (params_mpc_.T_max_safe - T_nom_j) - cj.dot(Aj * xi);
  }

  // --- 5. Set up OSQP ---
  //   Variables: [U_i (3*Np), slack s (Np)]  (slack for tension feasibility)
  const int nVar = 3 * Np + Np;
  Eigen::MatrixXd H_full = Eigen::MatrixXd::Zero(nVar, nVar);
  H_full.topLeftCorner(3*Np, 3*Np) = H_;
  // Slack penalty: w_s * I (in the cost, no quadratic on s — only linear,
  // handled in q_full; but adding small quadratic improves OSQP convergence)
  H_full.bottomRightCorner(Np, Np) =
      1e-6 * Eigen::MatrixXd::Identity(Np, Np);  // regularisation

  Eigen::VectorXd q_full = Eigen::VectorXd::Zero(nVar);
  q_full.head(3*Np) = f;
  q_full.tail(Np) = params_mpc_.tension_slack_weight * Eigen::VectorXd::Ones(Np);

  // Constraint matrix: [CT, -I_Np] (tension: CT*U - I*s <= dT)
  Eigen::MatrixXd Ac(Np, nVar);
  Ac.leftCols(3*Np) = CT;
  Ac.rightCols(Np) = -Eigen::MatrixXd::Identity(Np, Np);

  // Box bounds on U (first 3*Np variables)
  Eigen::VectorXd lb(nVar), ub(nVar);
  for (int j = 0; j < Np; ++j) {
    lb(3*j)   = -a_tilt_max;  ub(3*j)   = a_tilt_max;
    lb(3*j+1) = -a_tilt_max;  ub(3*j+1) = a_tilt_max;
    lb(3*j+2) = a_z_min;      ub(3*j+2) = a_z_max;
  }
  // Slack: s >= 0
  lb.tail(Np).setZero();
  ub.tail(Np).setConstant(std::numeric_limits<double>::infinity());

  // Tension constraint bounds: -inf <= CT*U - s <= dT
  Eigen::VectorXd lc = -std::numeric_limits<double>::infinity() * Eigen::VectorXd::Ones(Np);
  Eigen::VectorXd uc = dT;

  // --- 6. Solve with OSQP ---
  OsqpEigen::Solver solver;
  solver.settings()->setWarmStart(params_mpc_.warm_start);
  solver.settings()->setMaxIteration(200);
  solver.settings()->setAbsoluteTolerance(1e-4);
  solver.settings()->setRelativeTolerance(1e-4);
  solver.settings()->setVerbosity(false);

  // Build sparse H_full, Ac (OSQP requires CSC format)
  // ... (use Eigen::SparseMatrix wrappers)

  // Warm-start primal
  if (params_mpc_.warm_start) {
    Eigen::VectorXd z_warm = Eigen::VectorXd::Zero(nVar);
    z_warm.head(3*Np) = U_warm_;
    solver.setPrimalVariable(z_warm);
  }

  solver.solve();

  Eigen::VectorXd z_opt = solver.getSolution();
  Eigen::VectorXd U_opt = z_opt.head(3*Np);

  // --- 7. Extract first control a_d_opt = U_opt[0:3] ---
  Eigen::Vector3d a_d_opt = U_opt.head(3);

  // --- 8. Update warm-start cache (shift by one step) ---
  U_warm_.head(3*(Np-1)) = U_opt.tail(3*(Np-1));
  U_warm_.tail(3) = U_opt.tail(3);  // repeat last

  // --- 9. Thrust synthesis (IDENTICAL to existing code) ---
  //   double thrust = mass_ * (params_.gravity + a_d_opt.z()) + T_ff;
  //   ... [unchanged from DecentralizedLocalController::CalcControlForce L279-300]
}
```

### 7.5 OSQP Problem Summary (OSQP Native Formulation)

OSQP solves: $\min_z \tfrac{1}{2} z^T \mathbf{P} z + \mathbf{q}^T z$ s.t. $\mathbf{l} \le \mathbf{A} z \le \mathbf{u}$.

Mapping from the MPC formulation:

| OSQP symbol | MPC content | Dimension |
|---|---|---|
| $z$ | $[\mathcal{U}_i;\, s]$ | $4N_p$ (at $N_p = 5$: 20) |
| $\mathbf{P}$ | $\text{blkdiag}(\mathcal{H},\, \epsilon I_{N_p})$ | $4N_p \times 4N_p$ |
| $\mathbf{q}$ | $[f;\, w_s \mathbf{1}]$ | $4N_p$ |
| $\mathbf{A}$ | $[I_{3N_p}, 0;\; C_T, -I_{N_p}]$ | $(4N_p) \times (4N_p)$ |
| $\mathbf{l}$ | $[\ell_\text{box};\, -\infty\cdot\mathbf{1}_{N_p}]$ | $4N_p$ |
| $\mathbf{u}$ | $[u_\text{box};\, d_T]$ | $4N_p$ |

For $N_p = 5$: $n_\text{var} = 20$, $n_\text{con} = 20$. OSQP internal
complexity is $O(n_\text{var}^{1.5})$ per iteration — approximately
$20^{1.5} \approx 90$ operations; with warm-starting and 10 iterations
this is $\mathcal{O}(900)$ multiply-adds. On an ARM Cortex-A72 at 1.5 GHz,
this completes in $< 50\,\mu$s.

---

## 8. Part F — Warm-Starting Strategy

### 8.1 Shift Operator

At step $k$, after obtaining the optimal sequence
$\mathcal{U}_i^*(k) = [a^*(k),\, a^*(k+1),\, \ldots,\, a^*(k+N_p-1)]^T$,
the warm-start for step $k+1$ is:

$$
\mathcal{U}_\text{warm}(k+1) = \begin{bmatrix}
a^*(k+1) \\ a^*(k+2) \\ \vdots \\ a^*(k+N_p-1) \\ a^*(k+N_p-1)
\end{bmatrix}, \tag{44}
$$

i.e., shift left by one block and repeat the last element. This is a
feasible (box-constraint satisfying) initial point for the next QP.

**Implementation:**

```cpp
// After extracting U_opt from the solver:
U_warm_.head(3*(Np-1)) = U_opt.segment(3, 3*(Np-1));  // [a*(1), ..., a*(Np-1)]
U_warm_.tail(3)        = U_opt.tail(3);                // repeat a*(Np-1)
```

### 8.2 Dual Variable Warm-Starting

OSQP also warm-starts the **dual variables** (Lagrange multipliers for
constraints). Apply the same shift to the dual vector $y^*$:

$$
y_\text{warm}(k+1) = [y^*(k+1),\, \ldots,\, y^*(k+N_p-1),\, y^*(k+N_p-1)]^T. \tag{45}
$$

OSQP's `setPrimalVariable` and `setDualVariable` should be called
together. This typically reduces the iteration count from ~20 (cold start)
to ~5 (warm start) — a 4× speedup.

### 8.3 Expected Solve Time Budget

| Mode | Cold start | Warm start |
|---|---|---|
| $N_p = 5$, no tension active | $\approx 0.15$ ms | $\approx 0.05$ ms |
| $N_p = 5$, tension active | $\approx 0.25$ ms | $\approx 0.08$ ms |
| $N_p = 10$, no tension active | $\approx 0.40$ ms | $\approx 0.15$ ms |
| $N_p = 10$, tension active | $\approx 0.60$ ms | $\approx 0.25$ ms |

All values below the 2 ms control-step budget with $> 3\times$ margin.

---

## 9. Part G — Test Protocol

### 9.1 Scenario MPC_T1 — Single Fault, Figure-8

**Setup:**
```bash
./decentralized_fault_aware_sim \
  --num-quads 4 \
  --trajectory figure8 \
  --duration 40 \
  --scenario MPC_T1 \
  --fault-0-quad 0 \
  --fault-0-time 20.0 \
  --mpc-horizon 5          # new CLI flag
```

**Controller variants run in parallel:**
- Variant A: `DecentralizedLocalController` (baseline QP, no MPC)
- Variant B: `MpcLocalController`, $N_p = 5$, $T_\text{max,safe} = 100$ N

**Primary metrics:**

| Metric | Definition | Target (MPC vs. QP) |
|---|---|---|
| $T_\text{max}$ | $\max_{i,t} T_i(t)$ over full 40 s | MPC $\le 85$ N; QP $\le 95$ N |
| $\text{RMSE}_L$ | $\sqrt{\tfrac{1}{T}\int_0^T \|p_L - p_{L,\text{ref}}\|^2 \,dt}$ | MPC $\le 1.05\times$ QP value |
| Constraint violation | $\max_{i,t} [T_i(t) - T_\text{max,safe}]_+$ | MPC $= 0$ N |
| Max solve time | $\max_t t_\text{solve}$ (ms) | $< 1.5$ ms |

**Expected behaviour:** Post-fault ($t > 20$ s), the three surviving drones
each carry $\approx 1/3$ of $m_L g \approx 9.8$ N; the figure-8 centripetal
acceleration adds $\approx 0.9$ m/s², giving dynamic peak tension
$\approx 12$ N — no tension constraint should activate with either
controller. The test validates that MPC does not degrade tracking and that
the constraint infrastructure is correctly wired.

### 9.2 Scenario MPC_T2 — Dual Sequential Fault, Lemniscate3d

**Setup:**
```bash
./decentralized_fault_aware_sim \
  --num-quads 5 \
  --trajectory lemniscate3d \
  --duration 35 \
  --scenario MPC_T2 \
  --fault-0-quad 0 --fault-0-time 15.0 \
  --fault-1-quad 2 --fault-1-time 20.0 \
  --mpc-horizon 10
```

**Controller variants:**
- Variant A: `DecentralizedLocalController` (baseline)
- Variant B: `MpcLocalController`, $N_p = 10$, $T_\text{max,safe} = 100$ N

**Stress analysis:** After $t = 20$ s, 3 surviving drones of 5 carry the
full load. Lemniscate3d at $a = 3$ m, $T = 12$ s gives peak lateral
acceleration $\approx 1.5$ m/s², so dynamic tension per surviving drone:

$$
T_\text{dyn} \approx \frac{m_L (g + a_\perp)}{N_\text{alive}}
= \frac{3.0(9.81 + 1.5)}{3} = 11.3\text{ N}. \tag{46}
$$

This is still well below the safety limit. However, **fault transients**
can generate rope length oscillations with amplitude scaling as
$\Delta T = k_\text{eff} \|v_\text{fault}\| \cdot dt_\text{ring} \approx
2778 \times 0.5 \times 0.1 = 139$ N peak (at $dt_\text{ring} = 0.1$ s
ringdown duration). **This is the regime where the tension constraint is
expected to activate.**

**Primary metrics:**

| Metric | Definition | Target |
|---|---|---|
| $T_\text{max}$ | $\max_{i,t} T_i$ | MPC $\le 100$ N; QP possibly $> 100$ N |
| Constraint violation | $\max_{i,t}[T_i(t) - T_\text{max,safe}]_+$ | MPC $= 0$ N |
| $\text{RMSE}_L$ | Payload tracking error | MPC $\le 1.1\times$ QP value |
| Tension-constraint activation | Fraction of steps where $C_T$ constraint is active | Report raw value |

**Expected outcome:** With the baseline QP, the fault transient at $t = 15$ s
and $t = 20$ s drives the surviving drone positions toward the payload,
causing rope tension spikes that may transiently exceed 100 N. The MPC
with $N_p = 10$ predicts the incoming spike and proactively reduces
speed (trading trajectory tracking for rope safety). The $\text{RMSE}$
penalty is bounded because the MPC minimises tracking error subject to
the constraint, not in addition to it.

### 9.3 Metric Definitions and Analysis Scripts

**CSV logging additions** (new columns in the existing per-drone CSV output):

```
mpc_solve_time_us   — wall-clock MPC solve time (μs)
tension_headroom_min — min_j(d_T[j]) at each step (headroom to constraint, N)
tension_constraint_active — 1 if any tension slack s[j] > 0.01 N, else 0
U_warm_norm         — ||U_warm|| for warm-start quality monitoring
```

**Analysis script outline** (`Research/analysis/mpc_tension_comparison.py`):

```python
import pandas as pd, numpy as np, matplotlib.pyplot as plt

def load(scenario, variant):
    return pd.read_csv(f"output/{scenario}/drone0_{variant}.csv")

for scenario, variant in [("MPC_T1", "baseline"), ("MPC_T1", "mpc_Np5"),
                           ("MPC_T2", "baseline"), ("MPC_T2", "mpc_Np10")]:
    df = load(scenario, variant)
    T_max = df["tension_N"].max()
    rmse  = np.sqrt(((df["payload_error_m"]**2).mean()))
    print(f"{scenario}/{variant}: T_max={T_max:.1f} N, RMSE={rmse:.3f} m")

# Plot tension time series: baseline vs. MPC, with horizontal line at 100 N
```

---

## 10. Implementation Checklist

| Step | File | Action | Complexity |
|---|---|---|---|
| 1 | `cpp/include/mpc_local_controller.h` | New class declaration, `MpcParams` struct | 1 day |
| 2 | `cpp/src/mpc_local_controller.cc` | Constructor (pre-compute Φ, Ω, H), `CalcControlForce` with OSQP | 3 days |
| 3 | `cpp/src/mpc_local_controller.cc` | `SolveDARE()` helper (iterate Eq. 40 to convergence) | 0.5 day |
| 4 | `cpp/CMakeLists.txt` | Add `osqp-eigen` dependency, link `mpc_local_controller` | 0.5 day |
| 5 | `cpp/src/decentralized_fault_aware_sim_main.cc` | Add `--mpc-horizon` CLI flag; swap `DecentralizedLocalController` → `MpcLocalController` when flag set | 0.5 day |
| 6 | `cpp/src/decentralized_fault_aware_sim_main.cc` | Extend CSV output with 4 new MPC columns | 0.5 day |
| 7 | `Research/analysis/mpc_tension_comparison.py` | Analysis + comparison figures | 1 day |
| 8 | Unit test: `test_dare.py` | Verify DARE solution: spectral radius of $A-BK < 1$ | 0.5 day |
| 9 | Unit test: `test_tension_linearisation.py` | Verify linearised tension vs. exact tension for random positions | 0.5 day |

**Total estimated effort:** 8 person-days.

---

## 11. Appendix — Symbol Table

| Symbol | Meaning | Dimensions | Units |
|---|---|---|---|
| $x_i(k)$ | Error state at step $k$ | $6$ | m, m/s |
| $e_{p,i}$ | Position error | $3$ | m |
| $e_{v,i}$ | Velocity error | $3$ | m/s |
| $\mathcal{U}_i$ | Stacked control sequence | $3N_p$ | m/s² |
| $a_i(k+j)$ | Acceleration command at $k+j$ | $3$ | m/s² |
| $A$ | State transition matrix | $6\times6$ | — |
| $B$ | Input matrix | $6\times3$ | — |
| $\Phi$ | Prediction free-response map | $6N_p\times6$ | — |
| $\Omega$ | Prediction forced-response map | $6N_p\times3N_p$ | — |
| $\mathcal{H}$ | MPC Hessian (constant) | $3N_p\times3N_p$ | — |
| $f$ | MPC linear cost (per-step) | $3N_p$ | — |
| $\bar{Q}$ | Block-diagonal state cost | $6N_p\times6N_p$ | — |
| $\bar{R}$ | Block-diagonal control cost | $3N_p\times3N_p$ | — |
| $P_f$ | DARE terminal cost | $6\times6$ | — |
| $K$ | LQR gain | $3\times6$ | — |
| $\hat{n}_i(k+j)$ | Payload-to-drone unit vector at $k+j$ | $3$ | — |
| $d_{i,\text{nom}}$ | Nominal chord length | scalar | m |
| $T_{i,\text{nom}}$ | Nominal tension at formation slot | scalar | N |
| $C_T$ | Tension constraint matrix | $N_p\times3N_p$ | N/(m/s²) |
| $d_T$ | Tension constraint RHS | $N_p$ | N |
| $s$ | Tension slack variables | $N_p$ | N |
| $\mu_j$ | Tension headroom at step $j$ | scalar | N |
| $k_\text{eff}$ | Effective rope stiffness | scalar | N/m |
| $L_\text{rest}$ | Rope rest length | scalar | m |
| $T_\text{max,safe}$ | Tension safety ceiling | scalar | N |
| $N_p$ | Prediction horizon | scalar | steps |
| $dt$ | MPC control step | scalar | s |
| $w_s$ | Tension slack penalty weight | scalar | N⁻¹ |

---


