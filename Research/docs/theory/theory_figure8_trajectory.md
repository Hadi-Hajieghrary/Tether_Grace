# Theory — Waypoint Trajectory and the Figure-8 Reference

> Companion to the trajectory-generation code in
> [`Research/cpp/src/decentralized_fault_aware_sim_main.cc`](../../cpp/src/decentralized_fault_aware_sim_main.cc)
> (function `BuildTrajectory`) and the controller's
> `ComputePayloadReference` method in
> [`Research/cpp/src/decentralized_local_controller.cc`](../../cpp/src/decentralized_local_controller.cc).

## 1. Summary

Three families of time-parameterised trajectories are available for the
shared payload reference: a **"traverse"** waypoint sequence (lift →
horizontal translate → land), a **"figure8"** 2-D Bernoulli lemniscate
for agile manoeuvring under load, and a **"lemniscate3d"** 3-D Bernoulli
lemniscate with a superimposed vertical oscillation for the 5-drone
agility campaign. All three are produced by the single generator
`BuildTrajectory(shape, duration, z_low, z_high)`, stored as a list of
`TrajectoryWaypoint` structs, and resolved at runtime by
`ComputePayloadReference(t)` via first-order hold between successive
waypoints. The payload reference is published as a feed-forward signal
to every drone's controller — there is no feedback through this channel.

## 2. Waypoint semantics

A `TrajectoryWaypoint` is a triple

$$
(w_k,\; t_k,\; h_k) \;=\; (\text{position}_k,\; \text{arrival\_time}_k,\; \text{hold\_time}_k) , \tag{1}
$$

interpreted as "arrive at $w_k$ no later than $t_k$ and hold there for
$h_k$ seconds". The reference at time $t$ is a piecewise-linear
interpolation between consecutive waypoints, with a zero-velocity hold
segment in between:

$$
\bar{\mathbf{p}}_L(t) \;=\;
\begin{cases}
w_k & t \le t_k \text{ and } k = 0 \\
(1-\alpha_k)\,w_{k-1} \;+\; \alpha_k\,w_k , \quad \alpha_k \;=\; \dfrac{t - (t_{k-1} + h_{k-1})}{t_k - (t_{k-1} + h_{k-1})}
    & t_{k-1} + h_{k-1} < t \le t_k \\
w_k & t_k < t \le t_k + h_k \\
w_{K} & t > t_K + h_K \text{ (past the last waypoint)}
\end{cases}
\tag{2}
$$

and

$$
\bar{\mathbf{v}}_L(t) \;=\;
\begin{cases}
\dfrac{w_k - w_{k-1}}{t_k - (t_{k-1} + h_{k-1})} & \text{during transit} \\
\mathbf{0} & \text{during hold or past end}
\end{cases}
\tag{3}
$$

Implementation:
[`decentralized_local_controller.cc:L59–88`](../../cpp/src/decentralized_local_controller.cc#L59).
The reference is a zero-order-continuous, velocity-piecewise-continuous
signal — velocity has step discontinuities at every waypoint transition.

## 3. "Traverse" waypoint sequence

Four waypoints for a simple pickup → lift → translate → land manoeuvre,
with $z_{\text{low}} = 1$ m, $z_{\text{high}} = 3$ m, duration $D$:

| $k$ | $w_k$ | $t_k$ (s) | $h_k$ (s) | Phase |
|:-:|:-:|:-:|:-:|---|
| 0 | $(0, 0, z_{\text{low}})$ | 0.0 | 2.0 | pickup hover |
| 1 | $(0, 0, z_{\text{high}})$ | 5.0 | 2.0 | climb + hold |
| 2 | $(3, 0, z_{\text{high}})$ | $0.60\,D$ | 2.0 | traverse + hold |
| 3 | $(3, 0, z_{\text{low}})$ | $0.90\,D$ | 2.0 | descend + land |

Implementation:
[`decentralized_fault_aware_sim_main.cc` `BuildTrajectory`](../../cpp/src/decentralized_fault_aware_sim_main.cc)
(search `"figure8" ==` to jump to the block; the `else` branch handles
traverse).

## 4. "figure8" — Bernoulli lemniscate

### 4.1 Parametric form

The figure-8 reference is a **Bernoulli lemniscate** parameterised by
$\varphi \in [0, 2\pi)$:

$$
x(\varphi) \;=\; \frac{a\cos\varphi}{1 + \sin^2\varphi}, \qquad
y(\varphi) \;=\; \frac{a\,\sin\varphi\cos\varphi}{1 + \sin^2\varphi} . \tag{4}
$$

With $a = 3$ m, the curve has maximum x-extent $\pm 3$ m, maximum
y-extent $\pm a/(2\sqrt{2}) \approx \pm 1.06$ m, and passes through the
origin at $\varphi = \pi/2$ and $3\pi/2$. Time is mapped as
$\varphi(t) = 2\pi\,(t - t_0) / T$ with period $T = 16$ s, giving an
average speed of about 1 m/s and **peak lateral acceleration of about
1 m/s²** — slow enough that the payload pendulum (eigenfrequency
$\approx 2.8$ rad/s) can track without large lag.

### 4.2 Waypoint schedule for a 40 s figure-8 mission

Two complete lemniscate loops at altitude $z_{\text{high}}$ are book-ended
by a lift-in and a descent:

| Phase | Range | Contents |
|---|---|---|
| Pickup + lift | $t \in [0, 6]$ s | 2 waypoints: $(a,0,z_{\text{low}})$ hold 2 s, $(a,0,z_{\text{high}})$ hold 1 s |
| Cruise | $t \in [6, D-3]$ s | One waypoint per **0.2 s** sampled from (4): $w(t) = (x(\varphi(t-6)),\ y(\varphi(t-6)),\ z_{\text{high}})$, no hold |
| Return to start | $t = D - 2$ | $(a, 0, z_{\text{high}})$ hold 0.5 s |
| Descend | $t = D - 0.1$ | $(a, 0, z_{\text{low}})$ hold 2 s |

The dense 0.2 s sampling during the cruise phase means the linear
interpolation of Eq. (2) is effectively smooth — the maximum
first-derivative discontinuity between successive segments is well
below the controller's tracking bandwidth.

### 4.3 Differential kinematics

A useful quantity for predicting the payload's pendulum-swing response
is the lemniscate's tangential velocity:

$$
\dot x = \frac{a(-\sin\varphi - \sin^3\varphi - 2\cos^2\varphi\sin\varphi)}{(1+\sin^2\varphi)^2}\,\dot\varphi \,,\quad
\dot y = \frac{a(\cos\varphi - \sin^2\varphi\cos\varphi + 2\sin^2\varphi\cos\varphi)}{(1+\sin^2\varphi)^2}\,\dot\varphi \,.
$$

With $\dot\varphi = 2\pi/T$, the speed
$\|\dot{\mathbf{w}}(t)\| = \sqrt{\dot x^2 + \dot y^2}$ oscillates twice
per period; the peak lateral acceleration occurs at the "lobes" (where
$\sin\varphi \approx \pm 1/\sqrt{3}$) at approximately

$$
\|\ddot{\mathbf{w}}\|_{\max} \;\approx\; a \left(\frac{2\pi}{T}\right)^2 \;\approx\; 3\,(0.39)^2 \;\approx\; 0.46\text{ m/s}^2,
$$

well inside the controller's 6.71 m/s² lateral-acceleration authority.

## 5. "lemniscate3d" — 3-D Bernoulli lemniscate with vertical oscillation

### 5.1 Motivation

The 2-D figure-8 (§4) keeps the payload at constant altitude
$z_{\text{high}}$. For the **5-drone agility campaign** a more
demanding reference is needed: one that exercises the altitude control
axis and creates a continuously varying 3-D payload inertial path that
the formation must track throughout the full mission (including under
compound faults). The `"lemniscate3d"` trajectory extends the 2-D
figure-8 with a superimposed vertical sinusoidal oscillation, uses a
shorter period ($T=12$ s vs. $T=16$ s), and retains the same
$(x,y)$-plane lemniscate shape.

### 5.2 Parametric form

The horizontal $(x,y)$ plane coordinates follow the same Bernoulli
lemniscate as §4, Eq. (4). The altitude adds a once-per-period
sinusoidal oscillation:

$$
\boxed{
\begin{aligned}
x(\varphi) &= \frac{a\cos\varphi}{1 + \sin^2\varphi} \\
y(\varphi) &= \frac{a\,\sin\varphi\cos\varphi}{1 + \sin^2\varphi} \\
z(\varphi) &= z_{\text{high}} + A_z\sin\varphi
\end{aligned}
} \tag{5}
$$

with $a = 3$ m, $A_z = 0.35$ m, period $T = 12$ s, and
$\varphi(t) = 2\pi(t - t_0)/T$.

### 5.3 Parameters

| Parameter | Symbol | Value |
|---|:-:|---:|
| Horizontal scale | $a$ | 3 m |
| Vertical oscillation amplitude | $A_z$ | 0.35 m |
| Period | $T$ | 12 s |
| Sample interval (waypoint spacing) | $\Delta t_{\text{wp}}$ | 0.2 s |
| Pickup hold (at $(a,0,z_{\text{low}})$) | $h_0$ | 2.0 s |
| Lift hold (at $(a,0,z_{\text{high}})$) | $h_1$ | 1.0 s |
| Return hold | $h_r$ | 0.5 s |
| Descend hold | $h_f$ | 2.0 s |

### 5.4 Waypoint schedule for a 40 s lemniscate3d mission

| Phase | Time range | Contents |
|---|---|---|
| Pickup + lift | $t \in [0, 6]$ s | 2 waypoints: $(a,0,z_{\text{low}})$ hold 2 s, $(a,0,z_{\text{high}})$ hold 1 s |
| Cruise | $t \in [6, D-3]$ s | One waypoint per 0.2 s sampled from Eq. (5); no hold |
| Return | $t = D-2$ | $(a,0,z_{\text{high}})$ hold 0.5 s |
| Descend | $t = D-0.1$ | $(a,0,z_{\text{low}})$ hold 2 s |

The 0.2 s sampling makes the linear interpolation between waypoints
effectively smooth relative to the controller's 5 ms control period.

### 5.5 Dynamic loading

The altitude oscillation $A_z = 0.35$ m at $T = 12$ s gives a peak
vertical acceleration $\|\ddot{z}\|_{\max} = A_z(2\pi/T)^2
\approx 0.096$ m/s² — well below $g$ and within the altitude PD
authority. Combined with the horizontal lemniscate acceleration
(peak $\approx 0.46$ m/s²), the total vector peak is
$\approx 0.47$ m/s². The shorter period means the
pendulum sees a higher update rate; this slightly increases the
residual swing angle compared to the 2-D figure-8.

### 5.6 Usage

`"lemniscate3d"` is the **exclusive trajectory** for all four 5-drone
campaign scenarios (A–D), executed via `run_5drone_campaign.sh`. It is
not used in any of the 6 four-drone IEEE scenarios (S1–S6).

**Code reference:** `BuildTrajectory` `"lemniscate3d"` branch at
[`decentralized_fault_aware_sim_main.cc:L131–182`](../../cpp/src/decentralized_fault_aware_sim_main.cc#L131).

---

## 6. Why the payload tracks, but with a residual RMS

Even under the above conservative trajectory, the final-campaign S4
(nominal figure-8) achieves an RMS payload-tracking error of **0.43 m**
— about 7× the 0.08 m of the straight-traverse baseline (S1). The
excess error is *not* a controller failure; it is the **physical
inertial lag of the under-slung pendulum** swinging into and out of
each lemniscate lobe. A non-zero swing velocity is the necessary
bandwidth price of agility with an under-slung load; the anti-swing
term (§5.2 of
[`theory_decentralized_local_controller.md`](theory_decentralized_local_controller.md))
reduces but does not eliminate it.

The S5 fault-during-figure-8 scenario (RMS 0.43 m — essentially equal
to S4) confirms that a mid-manoeuvre fault does **not** inflate the
tracking error further: the 3-drone formation continues to track the
lemniscate with the same residual pendulum lag.

## 6. Reference-source `LeafSystem`

At runtime the shared reference is emitted by a small `ReferenceSource`
LeafSystem (defined locally in the sim-main) that publishes two
`BasicVector<double>` output ports — `p_ref` (3-D position) and
`v_ref` (3-D velocity) — both computed from the `ComputeRef` helper
that applies Eq. (2)–(3). The same interpolator is re-used inside the
controller (see above) so that the payload-reference stream logged to
CSV exactly matches the formation target the drones are tracking.

## 7. Limits of the current trajectory model

- **No acceleration continuity.** Velocity has first-order jumps at
  waypoint boundaries; a drone's position PD sees a unit impulse of
  velocity-error on each transition. In practice we hide this by
  making holds long enough for the impulse to decay; a C²-continuous
  interpolator (quintic spline, for example) would be the
  upgrade path.
- **No explicit coupling to pendulum dynamics.** The trajectory is
  designed against the payload's rigid-body motion as if the payload
  were a point mass on a rigid rod; the anti-swing term handles the
  residual.
- **Only three shapes.** A general input would be a parametric
  $\mathbf{w}(t), \dot{\mathbf{w}}(t), \ddot{\mathbf{w}}(t)$ triple;
  currently only position+velocity are used, with acceleration
  implicit in the PD.

## 8. Code-to-theory map

| Eq. | Code |
|:-:|---|
| (1) | [`quadcopter_controller.h:TrajectoryWaypoint`](../../../Tether_Lift/Research/cpp/include/quadcopter_controller.h) (struct definition, Tether_Lift baseline) |
| (2)–(3) | [`decentralized_local_controller.cc:L74–88`](../../cpp/src/decentralized_local_controller.cc#L74) — `ComputePayloadReference` |
| (4) | `BuildTrajectory` "figure8" branch, [`decentralized_fault_aware_sim_main.cc:L183–229`](../../cpp/src/decentralized_fault_aware_sim_main.cc#L183) |
| (5) | `BuildTrajectory` "lemniscate3d" branch, [`decentralized_fault_aware_sim_main.cc:L131–182`](../../cpp/src/decentralized_fault_aware_sim_main.cc#L131) |
| Traverse (§3) | `BuildTrajectory` "traverse" branch (else branch, `main.cc:L230+`) |
| §6 | `ReferenceSource` class defined inside `decentralized_fault_aware_sim_main.cc` |
