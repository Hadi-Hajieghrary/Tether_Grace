# Theory — Cable-Fault Model and Response

> Companion to the fault-related Drake systems in
> [`Research/cpp/include/cable_fault_gate.h`](../../cpp/include/cable_fault_gate.h),
> [`Research/cpp/include/meshcat_fault_hider.h`](../../cpp/include/meshcat_fault_hider.h),
> [`Research/cpp/include/safe_hover_controller.h`](../../cpp/include/safe_hover_controller.h),
> [`Research/cpp/include/control_mode_switcher.h`](../../cpp/include/control_mode_switcher.h),
> [`Research/cpp/src/fault_aware_rope_visualizer.cc`](../../cpp/src/fault_aware_rope_visualizer.cc),
> and the wiring in
> [`Research/cpp/src/decentralized_fault_aware_sim_main.cc`](../../cpp/src/decentralized_fault_aware_sim_main.cc).

## 1. Summary

A cable-severance event at time $t_{\text{fault}}$ is modelled as the
simultaneous activation of four gates:

1. **Physical gate** — rope-force vector to the plant is zeroed.
2. **Telemetry gate** — rope-tension signal to the controller is zeroed.
3. **Visual gate** — rope polyline in the Meshcat replay is erased.
4. **Supervisory gate** — the faulted drone's controller is swapped from
   `DecentralizedLocalController` to `SafeHoverController` so the freed
   quadcopter retreats to a fixed safe-hover point instead of
   continuing to track its (now impossible-to-satisfy) formation slot.

The **surviving** drones' controllers are **untouched** at the moment
of fault — no broadcast, no flag, no state update. Their response is
emergent from (1)+(2): the payload's load redistributes through the
rope-payload physics onto the remaining $N-1$ ropes, each surviving
drone's own rope-tension rises, its `tension_ff` rises correspondingly,
its commanded thrust rises, and the payload stabilises within roughly
one natural period of the system.

## 2. Nomenclature

| Symbol | Meaning | Units | Code name |
|:-:|---|:-:|---|
| $t_{\text{fault}}$ | fault event time | s | `fault_time` / `fault_time_` |
| $\mathbf{F}_{\text{rope},i}$ | spatial force from rope $i$ on drone / payload / beads | N, N·m | `ExternallyAppliedSpatialForce` list |
| $\mathbf{t}_i$ | 4-D tension telemetry $[T_i, f_x, f_y, f_z]$ | N | `tension_data` |
| $\mathbf{p}_i^{\text{safe}}$ | pre-computed safe-hover target for the faulted drone | m | `safe_pos` |
| $r_s$ | safe-hover radial distance | m | $2.5 \times r_f$ (hard-coded) |

## 3. Fault gates — detailed

### 3.1 `CableFaultGate` — physical force

A Drake `LeafSystem` that copies its input abstract-valued force-vector
stream to the output unchanged for $t < t_{\text{fault}}$, and outputs
an empty vector afterwards:

$$
\mathbf{F}^{\text{out}}(t) \;=\;
\begin{cases}
\mathbf{F}^{\text{in}}(t), & t < t_{\text{fault}} \\
\emptyset, & t \ge t_{\text{fault}}
\end{cases} \tag{1}
$$

The output port is piped into the `ExternallyAppliedSpatialForceMultiplexer`
that feeds the `MultibodyPlant`, so after the fault the plant sees no
rope force from this drone's rope on any body (drone, beads, payload).
The intermediate bead bodies continue to exist as inert 6-DOF rigid
bodies and fall under gravity. Implementation:
[`cable_fault_gate.h`](../../cpp/include/cable_fault_gate.h).

### 3.2 `TensionFaultGate` — telemetry

A companion `LeafSystem` that gates the 4-D tension-telemetry signal:

$$
\mathbf{t}_i^{\text{out}}(t) \;=\;
\begin{cases}
\mathbf{t}_i^{\text{in}}(t), & t < t_{\text{fault}} \\
\mathbf{0}_4, & t \ge t_{\text{fault}}
\end{cases} \tag{2}
$$

The output is piped into the `ZeroOrderHold` and then into the drone's
controller `get_tension_input_port`. After the fault, the surviving
drone's own tension signal still flows normally; the **faulted** drone's
signal is clamped to zero (the rope has physically detached, so its
scalar tension would be 0 anyway — this gate just makes that definite
without waiting for the rope dynamics to catch up).

### 3.3 `FaultAwareRopeVisualizer` — Meshcat replay

The on-screen rope polyline for drone $i$ is drawn every frame as a
7-point polyline from drone → beads[0..7] → payload:

$$
\text{polyline}_i(t) \;=\; \big\{\; \mathbf{x}_0(t),\; \mathbf{x}_1(t),\; \dots,\; \mathbf{x}_{N_{\text{seg}}}(t)\; \big\}
$$

For $t \ge t_{\text{fault}}$ the visualiser replaces the polyline with a
two-point zero-alpha segment at the world origin and calls
`meshcat->Delete(path)`, which is recorded into the Meshcat animation
stream so the rope **disappears** at exactly $t_{\text{fault}}$ in the
replay rather than leaving a ghost line at its last taut pose.
Implementation:
[`fault_aware_rope_visualizer.cc`](../../cpp/src/fault_aware_rope_visualizer.cc).

### 3.4 `SafeHoverController` + `ControlModeSwitcher` — supervisory

The faulted drone cannot continue to track a formation slot because the
slot's whole physical justification (being directly above the payload
and holding it up) has just vanished. Two small systems restore a
well-defined behaviour for it:

- `SafeHoverController` ([`.h`](../../cpp/include/safe_hover_controller.h))
  is a simple PD around a fixed world-frame setpoint
  $\mathbf{p}_i^{\text{safe}} = (r_s \cos\phi_i,\ r_s \sin\phi_i,\ 5\text{ m})$,
  with $r_s = 2.5\,r_f$. It outputs a spatial force on the drone body.
- `ControlModeSwitcher`
  ([`.h`](../../cpp/include/control_mode_switcher.h)) is a multiplexer
  with one abstract input per mode and a $t_{\text{fault}}$ switch
  time. Before $t_{\text{fault}}$ it routes the
  `DecentralizedLocalController` output to the plant; after, it routes
  the `SafeHoverController` output.

Wiring at
[`decentralized_fault_aware_sim_main.cc:L484–523`](../../cpp/src/decentralized_fault_aware_sim_main.cc#L484).

## 4. Coupled response of the surviving drones

No drone other than the faulted one receives any explicit fault signal.
The *only* change they see is in the payload's observed motion and in
their own rope's tension. Denote the surviving index set
$\mathcal{S}(t) = \{j : t_{\text{fault},j} > t\}$.

### 4.1 Physics of redistribution

Before the fault, at quasi-hover equilibrium with $N$ drones,

$$
\sum_{j=1}^{N} T_j \;\approx\; m_L\,g , \qquad T_j \approx \frac{m_L g}{N} . \tag{3}
$$

At $t = t_{\text{fault}}$, one rope's tension drops to zero
instantaneously. The payload is now pulled by $N-1$ ropes; the load is
redistributed. By symmetry of the intact formation,

$$
T_j \xrightarrow{\,t > t_{\text{fault}}\,} \frac{m_L g}{N - 1}, \qquad j \in \mathcal{S}. \tag{4}
$$

The redistribution settles at the rate of the coupled rope-payload
dynamics: dominated by the pendulum natural frequency
$\omega_n^{\text{pend}} \sim \sqrt{g/L} \approx 2.8$ rad/s, so the
transient decays over a few hundred milliseconds, not the much faster
rope stiffness timescale of Eq. (6) in
[`theory_rope_dynamics.md`](theory_rope_dynamics.md).

### 4.2 Why the surviving drones track the increased load automatically

In the surviving drone $j$'s controller (Eq. (9) of
[`theory_decentralized_local_controller.md`](theory_decentralized_local_controller.md)),

$$
f_j \;=\; m_j\,(g + a_z^\star) \;+\; \underbrace{T_j^{\text{ff}}}_{= T_j\,\text{(measured)}} . \tag{5}
$$

As the measured $T_j$ rises from $m_L g / N \to m_L g / (N-1)$, the
thrust feed-forward rises with it. The *PD* term $m_j a_z^\star$ has to
absorb only the small altitude error that develops during the transient
($\lesssim 10$ cm in our scenarios), so tracking recovers without the
drone needing to know that a peer has failed.

This is the key physical fact that motivates the fully-local design: the
payload and the rope together implement a perfect one-to-one
signalling channel. A drone feeling its own rope pull harder is
mathematically equivalent to being told "a peer has failed; carry
more load". **We prefer the physics over the signalling.**

## 5. Fault signature in observable quantities

| Signal | Pre-fault | Immediately post-fault | Notes |
|---|---|---|---|
| $T_{i_{\text{fault}}}$ | $m_L g / N$ | $0$ | Zeroed by `TensionFaultGate` (2). |
| $T_j$, $j \in \mathcal{S}$ | $m_L g / N$ | $\to m_L g/(N-1)$ after a transient | Dominant fault signature for surviving drones. |
| $\sigma_T = \mathrm{std}(\mathbf{T})$ | small (≈ 3 N for $N=4$ at cruise) | jumps to ≈ 7 N after a single fault, ≈ 11 N after triple fault | Aggregate metric; see [`fig_compare_imbalance.png`](../../../output/Tether_Grace/07_cross_scenario_comparison/fig_compare_imbalance.png). |
| $\|\mathbf{e}_p\|$ | ≤ 5 cm RMS | transient of ≤ 40 cm, settles back to ≤ 10 cm | See [`fig_overlay_tracking_error.png`](../../../output/Tether_Grace/07_cross_scenario_comparison/fig_overlay_tracking_error.png). |
| Drone position (faulted) | formation slot | retreats to safe-hover via `SafeHoverController` | Visible in the HTML replay. |

## 6. Assumptions and limits

1. **The fault is instantaneous.** Real cable failure is a progressive
   event (yarns break over ≤ 100 ms). A first-order low-pass before the
   fault gate would be a trivial extension.
2. **Only one physical effect per rope.** We do not model partial
   damage, friction spikes, or pulley slippage.
3. **The supervisory switcher knows $t_{\text{fault}}$ a-priori.** In
   deployment an onboard fault-detection estimator (e.g. a CUSUM on
   $T_i$) would set this. See the "UNDOCUMENTED" flag in the Phase-1
   audit report.
4. **The safe-hover setpoint is hard-coded** in the harness, not
   chosen by any optimal policy. Picking an optimal safe-hover point
   (e.g. one that minimises disturbance to the remaining formation) is
   a natural future extension.

## 7. Code-to-theory map

| Aspect | Code |
|---|---|
| Physical force gate (1) | [`cable_fault_gate.h`](../../cpp/include/cable_fault_gate.h) class `CableFaultGate` |
| Telemetry gate (2) | [`cable_fault_gate.h`](../../cpp/include/cable_fault_gate.h) class `TensionFaultGate` |
| Meshcat rope-line hide (§3.3) | [`fault_aware_rope_visualizer.cc`](../../cpp/src/fault_aware_rope_visualizer.cc), `Update(…)` method |
| Supervisory switcher (§3.4) | [`control_mode_switcher.h`](../../cpp/include/control_mode_switcher.h), [`safe_hover_controller.h`](../../cpp/include/safe_hover_controller.h) |
| Wiring at the fault-time | [`decentralized_fault_aware_sim_main.cc:L484–523`](../../cpp/src/decentralized_fault_aware_sim_main.cc#L484) |
| Surviving-drone load pickup (§4) | emergent; no explicit code (feed-forward line [`decentralized_local_controller.cc:L192`](../../cpp/src/decentralized_local_controller.cc#L192)) |
