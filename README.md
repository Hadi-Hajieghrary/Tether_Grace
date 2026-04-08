# Tether Grace

**Cable-Fault Tolerance in Decentralized Cooperative Aerial Transport:
N-Agnostic Architecture, Evidence, and Open Bounds**

Tether Grace is the research codebase behind an IEEE Transactions on Control
Systems Technology submission that answers a simple question: when a cable
snaps during multi-quadrotor cooperative payload transport, does a
decentralized architecture need to detect the fault? The answer is no. The
N-agnostic PID-plus-gravity-feedforward control law already absorbs cable
faults with no detection, no reconfiguration, and no inter-agent
communication. This repository contains the complete pipeline from theory
to full multibody simulation evidence.

---

## Table of Contents

- [The Problem](#the-problem)
- [The Key Insight](#the-key-insight)
- [Contributions](#contributions)
- [Results at a Glance](#results-at-a-glance)
- [Canonical Simulations](#canonical-simulations)
  - [Three Drones, Single Cable Fault](#three-drones-single-cable-fault)
  - [Five Drones, Two Cascading Cable Faults](#five-drones-two-cascading-cable-faults)
  - [Seven Drones, Three Cascading Cable Faults](#seven-drones-three-cascading-cable-faults)
- [Physical System](#physical-system)
- [Control Architecture](#control-architecture)
- [Topology Invariance Theory](#topology-invariance-theory)
- [Source Code](#source-code)
- [Experiment Pipeline](#experiment-pipeline)
- [Simulation Outputs](#simulation-outputs)
- [The Paper](#the-paper)
- [Build and Run](#build-and-run)
- [Repository Structure](#repository-structure)
- [Relationship to Tether Lift](#relationship-to-tether-lift)
- [Open Problems](#open-problems)
- [License](#license)

---

## The Problem

Multi-quadrotor cooperative cable-suspended payload transport uses N
underactuated vehicles tethered to a shared rigid payload through flexible
cables. The system operates on the configuration manifold
Q = SE(3) x (SE(3) x S^2)^N with 6 + 8N degrees of freedom. When a cable
snaps, three coupled events occur simultaneously:

1. The payload accelerates downward (~3.3 m/s^2 for N=3)
2. The freed agent accelerates upward (~6.5 m/s^2)
3. Every surviving agent faces an instantaneous load-share jump (50% for N=3)

These effects propagate on timescales from sub-10 ms (rigid-body force
transmission) to ~200 ms (cable wave propagation), creating a multi-channel
fault signature that each surviving agent must absorb without centralized
coordination.

The difficulty is compounded by the architecture's defining property: the
**N-agnostic constraint**. Each agent knows only its own mass, inertia,
local sensor measurements, cable tension and direction, and a shared
reference trajectory. It does not know the team size, the payload mass, or
any other agent's state. This eliminates single points of failure but also
blocks model-based fault detection methods such as MMAE (which requires N+1
hypothesis filters). The tension-channel signal-to-noise ratio for a remote
cable break is approximately 1.3 -- indistinguishable from wind gusts.

No prior work addresses cable-fault tolerance in cooperative cable-suspended
aerial transport. The literature on UAV fault tolerance covers actuator and
sensor faults extensively, but cable faults -- which change the mechanical
coupling between agents and payload -- are entirely absent.

---

## The Key Insight

The mechanism is a structural property of N-agnostic control. Because each
agent's PID-plus-gravity-feedforward law uses only local measurements with
fixed gains, the per-agent closed-loop error dynamics are governed by the
same system matrices (A, B) regardless of cable topology:

```
z_dot = A * z + B * d_i(t)

A = [ 0    1    0  ]       B = [ 0 ]
    [-kp  -kd  -ki ]           [ 1 ]
    [ 1    0    0  ]           [ 0 ]
```

where z = [e, e_dot, eta]^T is the error state and d_i(t) is the lumped
disturbance. Cable snaps change the disturbance input d_i(t) but not the
system (A, B). This is **Proposition 1** in the paper -- the topology-invariant
error structure.

This structural property has three immediate consequences:

- A **common Lyapunov function** V(z) = z^T P z serves across all cable
  topologies, requiring no switching and no dwell-time constraints
- A **finite-time transient prediction** yields ~18 cm peak error from a
  single cable snap (observed ~107 cm; the topology-invariant ~5.8x ratio
  across scenarios confirms correct scaling)
- A **closed-form cost ratio** rho_FT = N/(N-k) provides an actuator-sizing
  design criterion

Ablation identifies PID feedback with gravity feedforward as the primary
fault-absorption mechanism. Cable-tension feedforward contributes less than
1% RMSE change. A centralized oracle with perfect fault knowledge improves
over the passive architecture by only 0.7%.

---

## Contributions

**C1 -- Topology-invariant error structure and mechanism identification.**
Proposition 1 proves that the N-agnostic PID-plus-feedforward law produces
per-agent error dynamics identical across all cable topologies. Ablation
confirms PID + gravity feedforward as the primary mechanism. A finite-time
transient prediction and the closed-form cost ratio rho_FT = N/(N-k)
provide design criteria for actuator sizing.

**C2 -- Comprehensive multibody evidence.**
Full-Drake simulation across N in {3, 5, 7} with 1-3 cascading cable-snap
faults under both ideal and sensor-realistic (ESKF + Dryden wind) conditions.
Monte Carlo validation (30 runs per scenario, <3% RMSE variation). Matched
single-fault experiments, fault-count isolation, trajectory generalization,
and controller comparison provide comprehensive deconfounding.

**C3 -- Quantitative design framework.**
A fault-tolerance cost ratio validated against empirical nominal-vs-fault
RMSE ratio, a thrust-margin screening methodology validated by the f_max
phase transition at the theoretical minimum, and a tilt-corrected capacity
analysis for actuator sizing.

---

## Results at a Glance

### Canonical Fault Scenarios

| Scenario | N | Faults | Load RMSE | Peak error | rho_FT |
|----------|---|--------|-----------|------------|--------|
| Three drones, no fault | 3 | 0 | 35.12 cm | 110.57 cm | --- |
| Three drones, 1 fault | 3 | 1 | 48.84 cm | 107.44 cm | 1.39 |
| Five drones, 2 faults | 5 | 2 | 46.37 cm | 112.30 cm | 1.33 |
| Seven drones, 3 faults | 7 | 3 | 43.60 cm | 117.45 cm | 1.26 |

Payload RMSE *decreases* with team size despite absorbing proportionally
more faults. The N=7 scenario absorbs three cascading faults yet achieves
11% lower RMSE than N=3 with one fault.

### Controller Comparison

| Controller | RMSE | Detection? | Communication? |
|-----------|------|------------|----------------|
| Centralized oracle (perfect knowledge) | 49.18 cm | Perfect | Yes |
| Passive GPAC (this work) | 48.84 cm | No | No |
| Threshold-triggered reactive | 49.45 cm | Threshold | No |

The price of decentralization under cable faults is effectively zero.

### Key Ablation Results

- Disabling cable-tension feedforward: RMSE changes by 0.44 cm (0.9%)
- Thrust boundary phase transition: sharp failure at f_max ~45 N,
  coinciding with theoretical minimum; default 150 N provides 3x headroom
- Monte Carlo (30 runs): RMSE std 0.49-1.34 cm (<3% variation)
- Sensor-realistic (ESKF + Dryden wind): uniform ~10% RMSE increase

---

## Canonical Simulations

The three canonical scenarios are the core evidence of the paper. Each is a
full Drake multibody simulation at 5 kHz with bead-chain cable dynamics,
geometric SO(3) attitude control, and PID position tracking on a 30-second
figure-eight trajectory. Every simulation produces an interactive 3D replay
that can be opened in any web browser -- the most direct way to see what
happens when a cable snaps.

---

### Three Drones, Single Cable Fault

[**Open 3D Replay**](outputs/full_drake_fault_batch/three_drones/full_drake_meshcat_replay.html)
 | [Recording](outputs/full_drake_fault_batch/three_drones/full_drake_recording.npz)
 | [Scenario Summary](outputs/full_drake_fault_batch/three_drones/scenario_summary.json)

This is the most stressed scenario in terms of per-agent impact. Three
quadrotors in a triangular formation transport a 3.0 kg payload along a
figure-eight trajectory at 2.5 m altitude. Cable lengths are asymmetric:
1.00, 1.08, and 1.16 m. At t = 7.0 s, cable 0 is instantaneously severed.

**What happens at the moment of fault:** Drone 0 loses all cable tension
within one sample period (5 ms) and begins accelerating upward at ~6.5 m/s^2.
The payload simultaneously drops as it loses one-third of its lift support,
producing a downward acceleration of ~3.3 m/s^2. Drones 1 and 2, still
connected, experience an instantaneous 50% load-share jump -- from carrying
m_L/3 = 1.0 kg each to m_L/2 = 1.5 kg each.

**What happens next:** Drone 0 detects its own cable loss via the tension
threshold (T < 0.2 N held for 0.12 s) and initiates the two-stage escape
maneuver: climbing 0.9 m and displacing 1.8 m laterally within 1.5 s,
reaching 2.26 m final separation from the nearest healthy agent. Meanwhile,
drones 1 and 2 have no knowledge that anything has changed. Their PID
controllers observe increasing position error as the payload deviates from
the reference, and the integral term begins accumulating to compensate the
increased load share. No adaptation, no mode switch, no communication -- just
PID feedback responding to a larger-than-usual error signal.

**The recovery:** The payload tracking error peaks at 107.44 cm during the
transient, then settles within approximately 2 seconds. The post-fault RMSE
is 48.84 cm, yielding an empirical cost ratio rho_FT = 1.39 against the
nominal 35.12 cm. Both surviving cables show tension surges during load
redistribution, with peaks at 25.7 N (cable 1) and 25.3 N (cable 2) --
well within the 150 N thrust authority. Progressive cable pendulum
oscillation is visible throughout the figure-eight, reflecting near-resonance
between the ~2 s cable pendulum period and the ~3 s trajectory corner interval.

**Why this scenario matters:** This is the hardest per-agent case. Losing
one of three cables means a 50% load increase on each survivor -- the
largest relative jump in the test matrix. If the architecture absorbs this
fault, it can absorb any single fault in a larger team.

| Metric | Value |
|--------|-------|
| Agents | 3 (2 surviving) |
| Cable lengths | 1.00, 1.08, 1.16 m |
| Fault | Cable 0 at t = 7.0 s |
| Post-fault load share increase | 50% |
| Payload RMSE (nominal / fault) | 35.12 / 48.84 cm |
| Peak tracking error | 107.44 cm |
| Empirical rho_FT | 1.39 |
| Detection latency | 0.12 s |
| Freed-agent final separation | 2.26 m |
| Peak cable tension (post-fault) | 25.7 N |
| Thrust headroom | 150 N available, ~40 N used |

**Diagnostic figures** (19 plots in `outputs/full_drake_fault_batch/three_drones/figures/`):

| Figure | What it shows |
|--------|---------------|
| `payload_tracking_error.png` | 3D position error vs time -- the fault transient at t=7 s is the central feature |
| `cable_tensions.png` | Per-cable tension histories -- cable 0 drops to zero, cables 1-2 surge |
| `xy_trajectories.png` | XY plane view of all agent and payload paths -- drone 0 departing visible |
| `payload_xy_trajectory.png` | Payload-only XY path overlaid on reference |
| `payload_x_tracking.png` | X-axis tracking: reference vs actual |
| `payload_y_tracking.png` | Y-axis tracking: reference vs actual |
| `payload_z_tracking.png` | Z-axis tracking -- shows the altitude drop at fault |
| `payload_z_offset.png` | Vertical offset from reference over time |
| `quad_tilt_magnitude.png` | Vehicle tilt angle -- confirms safe margins |
| `quad_force_norm.png` | Per-vehicle thrust magnitude -- shows post-fault increase |
| `quad_formation_deviation.png` | Formation geometry deviation from nominal |
| `quad_altitude.png` | Per-vehicle altitude histories |
| `quad_speed_norm.png` | Per-vehicle speed magnitude |
| `quad_lateral_force_norm.png` | Lateral force component per vehicle |
| `quad_torque_norm.png` | Torque magnitude per vehicle |
| `cable_health_schedule.png` | Cable health multiplier timeline |
| `final_quad_positions.png` | 3D snapshot of final vehicle positions |
| `vertical_motion.png` | Vertical dynamics detail |
| `load_speed_and_total_vertical_force.png` | Payload speed and aggregate vertical force |

---

### Five Drones, Two Cascading Cable Faults

[**Open 3D Replay**](outputs/full_drake_fault_batch/five_drones/full_drake_meshcat_replay.html)
 | [Recording](outputs/full_drake_fault_batch/five_drones/full_drake_recording.npz)
 | [Scenario Summary](outputs/full_drake_fault_batch/five_drones/scenario_summary.json)

Five quadrotors in a pentagonal formation transport the same 3.0 kg payload
along the figure-eight trajectory. Cable lengths are 1.00, 1.05, 1.10, 1.14,
and 1.18 m. Two cables are severed in sequence: cable 1 at t = 7.0 s and
cable 3 at t = 12.0 s, with 5 seconds between faults.

**First fault (t = 7.0 s):** Cable 1 snaps. Drone 1 detects and escapes.
The remaining four agents absorb a 25% load increase (from m_L/5 to m_L/4
per agent). The transient is visually similar to the three-drone case but
less severe -- four agents distribute the shock more effectively than two.

**Second fault (t = 12.0 s):** Cable 3 snaps while the system is still
tracking normally after recovering from the first fault. Drone 3 detects
and escapes. Now three agents remain, each carrying m_L/3 -- a cumulative
67% load increase from the original m_L/5. The second transient is
comparable in magnitude to the first. Both transients produce bounded
payload excursions that settle within 1-2 seconds.

**The cascading nature:** This scenario tests whether the architecture can
handle sequential faults without residual effects from the first fault
compromising the response to the second. Because the PID error dynamics are
topology-invariant (Proposition 1), the second fault encounters exactly
the same (A, B) system as the first -- the system does not accumulate
hidden vulnerability.

**What to look for in the replay:** Watch the pentagonal formation at the
start. At t = 7 s, one agent departs upward. The remaining four continue
tracking with slightly increased oscillation. At t = 12 s, a second agent
departs. The surviving triangle of three agents -- now carrying the same
per-agent load as the post-fault three-drone scenario -- continues tracking
without interruption. The payload trajectory remains visibly smooth through
both transitions.

| Metric | Value |
|--------|-------|
| Agents | 5 (3 surviving) |
| Cable lengths | 1.00, 1.05, 1.10, 1.14, 1.18 m |
| Faults | Cable 1 at t = 7.0 s, cable 3 at t = 12.0 s |
| Cumulative load share increase | 67% |
| Payload RMSE (nominal / fault) | 34.84 / 46.37 cm |
| Peak tracking error | 112.30 cm |
| Empirical rho_FT | 1.33 |
| Detection latency | 0.12 s (both faults) |
| Freed-agent final separation | 2.69 m (drone 1), 4.68 m (drone 3) |
| Peak cable tension (post-fault) | 21.7 N |

---

### Seven Drones, Three Cascading Cable Faults

[**Open 3D Replay**](outputs/full_drake_fault_batch/seven_drones/full_drake_meshcat_replay.html)
 | [Recording](outputs/full_drake_fault_batch/seven_drones/full_drake_recording.npz)
 | [Scenario Summary](outputs/full_drake_fault_batch/seven_drones/scenario_summary.json)

This is the most demanding scenario in the entire test matrix. Seven
quadrotors in a heptagonal formation transport the payload along the
figure-eight. Cable lengths span 0.98 to 1.16 m. Three cables are severed
in rapid succession: cable 1 at t = 7.0 s, cable 3 at t = 12.0 s, and
cable 5 at t = 14.0 s -- with only 2 seconds between the second and third
faults.

**The cascade:** The first fault at t = 7.0 s is the gentlest: six agents
absorb a 17% load increase. The second at t = 12.0 s raises the load share
further. The critical moment is the third fault at t = 14.0 s, which arrives
**before the system has fully recovered from the second fault**. This is the
only scenario where fault transients overlap, testing whether the
topology-invariant error structure holds under compounding perturbations.

**After all three faults:** Drones 1, 3, and 5 have departed. Drones 0, 2,
4, and 6 remain, carrying m_L/4 = 0.75 kg each -- a 75% cumulative load
increase from the original m_L/7. The surviving four-agent formation is
no longer symmetric (alternating agents from a heptagon), yet the N-agnostic
controller requires no knowledge of formation geometry. Each surviving agent
simply tracks its reference position with the same PID gains it has always
used.

**The striking result:** Despite absorbing three cascading faults with
overlapping transients, the N=7 scenario achieves a payload RMSE of
43.60 cm -- **11% lower than the N=3 scenario with only one fault**
(48.84 cm). Larger teams tolerate proportionally more faults while tracking
better, because each fault represents a smaller relative perturbation.
This is the empirical confirmation of the topology-invariance hypothesis.

**What to look for in the replay:** The heptagonal formation is visually
dense at the start. Watch at t = 7 s as the first agent departs -- the
payload barely reacts. At t = 12 s, the second departure is similarly
absorbed. At t = 14 s, the third departure is the most dramatic: it arrives
during a trajectory direction change, compounding the fault transient with
the cable pendulum oscillation. Yet the payload trajectory remains bounded.
After t = 16 s, the four surviving agents settle into a new equilibrium and
continue tracking for the remaining 14 seconds of the simulation.

**This is the paper's Fig. 1** -- the eight-panel snapshot showing the
seven-drone team before and after each cascading fault event.

| Metric | Value |
|--------|-------|
| Agents | 7 (4 surviving) |
| Cable lengths | 0.98, 1.01, 1.04, 1.07, 1.10, 1.13, 1.16 m |
| Faults | Cable 1 at 7.0 s, cable 3 at 12.0 s, cable 5 at 14.0 s |
| Cumulative load share increase | 75% |
| Payload RMSE (nominal / fault) | 34.70 / 43.60 cm |
| Peak tracking error | 117.45 cm |
| Empirical rho_FT | 1.26 |
| Detection latency | 0.12 s (all three faults) |
| Freed-agent final separation | 2.06 m (drone 1), 4.73 m (drone 3), 3.82 m (drone 5) |
| Peak cable tension (post-fault) | 18.4 N |
| Payload peak speed | 3.04 m/s |

**The most stressed freed agent:** Drone 5, faulted at t = 14.0 s (only
2 seconds after the second fault), achieves 1.69 m separation at +2 s and
3.82 m final separation -- safely above the collision threshold despite
the compressed fault schedule.

---

## Physical System

### Quadrotor Model

Each of the N quadrotors (default N=3) is a 6-DOF rigid body:

- Mass: m_Q = 1.5 kg, dimensions 0.30 x 0.30 x 0.10 m
- Thrust: f_i in [0, 150] N, torque |tau_k| <= 10 N*m per axis
- Attitude: geometric SO(3) representation (singularity-free)

Translational dynamics:

```
m_Q * p_ddot_i = -m_Q*g*e3 + f_i*R_i*e3 + F_cable_i + F_wind_i
```

### Payload

Rigid sphere: m_L = 3.0 kg, radius 0.15 m. Ground contact with Coulomb
friction (mu_s=0.5, mu_k=0.3). Cable attachment points at 0.105 m radius.
The payload mass is **unknown to every agent**.

### Bead-Chain Cable Model

Each cable is a chain of n_b = 8 point-mass beads (m_rope = 0.2 kg)
creating 9 tension-only spring-damper segments:

```
tension = { 0                                 if stretch <= 0  (slack)
           { k_s*stretch + c_s*max(0,rate)    if stretch > 0   (taut)
```

Stiffness derived from 15% maximum strain. Cable wave velocity ~7 m/s.
Per-quadrotor cable lengths sampled from Gaussians with up to 19% asymmetry.

### Wind Disturbance

MIL-F-8785C Dryden turbulence (sigma_u = sigma_v = 1.0 m/s, sigma_w = 0.5
m/s, mean wind [2, 0, 0] m/s). Spatial correlation decays exponentially
with l_c = 10 m.

### Sensor Suite

Each agent carries IMU (400 Hz), GPS (10 Hz, 2 cm horizontal noise),
barometer (25 Hz), and cable tension/direction sensor (200 Hz). State
estimation uses a 15-state error-state Kalman filter (ESKF) with
multiplicative quaternion error.

### The N-Agnostic Constraint

Each agent knows **only**: its own mass and inertia, its own ESKF state
estimate, its cable tension and direction, and a shared reference trajectory.
It does **not** know: the number of other agents, the payload mass, any
other agent's state, or whether other agents are connected.

---

## Control Architecture

The baseline GPAC (Geometric Position and Attitude Control) hierarchy is a
four-layer controller per agent. The fault-tolerance evaluation uses
**Layers 1-2 only** to isolate the minimal fault-tolerant mechanism.

### Layer 1: PID Position Control (50 Hz)

Computes desired force from PID feedback, gravity feedforward, cable-tension
feedforward, and anti-swing damping:

```
F_des = m_Q*g*e3 + m_Q*(kp*e_p + kd*e_v + ki*integral(e_p)) + kappa_T*T_i*q_i - k_sw*omega_cable
```

Gains: kp = 8.0, kd = 8.0, ki = 0.15, kappa_T = 0.5, k_sw = 0.5 N*s/rad.
Integral anti-windup at +/-2 m*s per axis.

**Every term is a function of agent i's local measurements only.** When a
cable snaps, the surviving agents' tracking errors update instantaneously
through the physics, and the PID integral term absorbs the increased load
share without adaptation delay.

### Layer 2: Geometric SO(3) Attitude Control (200 Hz)

Singularity-free tracking on SO(3) following Lee et al. (2010):

```
tau_i = -k_R*e_R - k_Omega*e_Omega + Omega x J*Omega
```

Almost-global exponential convergence on {Psi_R < 2} in SO(3).

### Layers 3-4 (Disabled in Reported Runs)

Layer 3 (concurrent learning mass estimator) and Layer 4 (extended state
observer) exist in the Tether_Lift codebase. They are deliberately disabled
in the fault-tolerance evaluation because they introduce history-dependent
terms that break the topology-invariant structure. Their integration is
discussed as future work. An ESO-active evaluation confirms that
absolute post-fault RMSE improves with ESO but rho_FT increases modestly.

### Freed-Agent Escape

When agent i detects its own cable loss (T_i < T_min for 0.12 s debounce),
it executes a two-stage escape maneuver: first waypoint at 1.0 m lateral,
0.7 m climb over 1.2 s; second at 2.5 m lateral, 1.3 m climb over 4.0 s.

---

## Topology Invariance Theory

### Proposition 1: Topology-Independent Error Dynamics

Under the N-agnostic PID-plus-feedforward law with ideal attitude tracking
and no actuator saturation, every agent's closed-loop error dynamics are
governed by the same (A, B) pair regardless of which cables are intact,
the team size N, or the payload mass m_L.

A cable snap changes the **input** d_i(t) but not the **system** (A, B),
because every element of A and B depends only on the PID gains (kp, kd, ki)
-- none of which is a function of N, m_L, cable stiffness, or cable identity.

### Corollary: Common Lyapunov Function

Let P > 0 satisfy A^T P + PA = -Q for some Q > 0. Then V(z) = z^T P z
serves as a common ISS-Lyapunov function for the error dynamics under
**every** cable topology. No switching between Lyapunov functions is
required at cable-snap events.

### Finite-Time Transient Prediction

The standard steady-state ISS gain is operationally vacuous (80 m). A
finite-time bound over the 0.5 s fault transient predicts ~18 cm peak
position error. The observed ~107 cm in full Drake includes unmodeled
elastic cable coupling, attitude-loop delay, and multi-axis interaction.
The ratio alpha ~= 5.8 is **topology-invariant** across scenarios (varying
<10%), confirming the prediction captures correct scaling.

### Fault-Tolerance Cost Ratio

```
rho_FT = N / (N - k)
```

Conservative steady-state bound. Empirical values (1.26-1.39) are lower
because the fault disturbance is transient, not sustained. For actuator
sizing, rho_FT = 1.5 (N=3, k=1) remains appropriate.

### Gap Analysis

Three gaps separate the simplified model from the full bead-chain plant:

1. **Impulsive slack-to-taut transitions** during the elastic transient
2. **Attitude-loop delay** during fast force transients (~5 ms inner-loop
   period vs. ~8 ms force transmission)
3. **State-dependent cable-force feedback** requiring a small-gain
   condition (scalar gamma_ISS = 0.125; requires gamma_cable < 8.0)

The ISS switched-system framework provides a coherent partial resolution.
The formal stability extension to the full nonlinear plant remains the
primary open theoretical problem.

---

## Source Code

The implementation lives in `src/gpac_fault_tolerance/` (20 headers,
21 source files) organized in four architectural layers.

### Core Algorithms (`core/`, Drake-free C++17)

| File | Purpose |
|------|---------|
| `l1_adaptive_core.{h,cc}` | L1-style adaptive disturbance estimator with memoryless piecewise-constant adaptation. Implemented for future integration; not active in reported runs. |
| `cable_snap_profile.{h,cc}` | Parameterized cable fault schedule: per-cable multipliers with instantaneous or ramped transitions. |
| `fault_metrics.{h,cc}` | Computes pre/post-fault RMSE, peak deviation, RMSE ratio, and recovery time from error time series. |
| `desired_geometry_redistributor.{h,cc}` | Post-fault formation redistribution: recenters and rescales surviving agents' offsets to preserve symmetric load distribution. |
| `reference_trajectory_mapper.{h,cc}` | Converts load-level references and redistributed geometry into per-drone trajectory references. |

### Contracts (`contracts/`)

`signal_contracts.h` defines the inter-component vocabulary:
`DisturbanceEstimateSignal`, `DisturbanceCompensationSignal`,
`DisturbanceBoundSignal`, `CableTensionSample`, `CableHealthMultipliers`.
All disturbance signals use acceleration units (m/s^2) to match the GPAC
controller's internal convention.

### Adapter (`adapters/`)

`gpac_seam_adapter.{h,cc}` packages the L1 adaptive core with
GPAC-specific signal policy. Enforces the residual-only rule: the adaptive
signal does not fold in nominal cable-tension compensation, avoiding
double-counting.

### Drake Integration (`drake/`, C++17/20)

Built only with `ENABLE_GPAC_DRAKE_WRAPPERS=ON`. Provides Drake LeafSystem
wrappers for all core algorithms, plus the main simulation executable:

| System | Purpose |
|--------|---------|
| `fault_detecting_gpac_controller` | Extended GPAC controller with PID, geometric SO(3), cable-tension feedforward, anti-swing, integral action, and load-tracking. |
| `fault_detecting_quadcopter_controller` | Cable-fault detection via tension threshold, autonomous escape maneuver with two-stage waypoints. |
| `faultable_rope_force_system` | Bead-chain cable with runtime severance capability. |
| `estimated_state_override` | Routes ESKF estimates into the controller when sensor-realistic mode is active. |
| `full_drake_fault_runner.cc` | Standalone simulation executable assembling the full Drake diagram for N quadrotors with configurable faults, sensing, and wind. |

### Test Suite (`tests/`, 6 files)

Unit tests for every core component using Google Test. All Drake-free,
linked against `libgpac_fault_tolerance_core.a`:

- `l1_adaptive_core_test.cc` -- Saturation bounds, convergence behavior
- `gpac_seam_adapter_test.cc` -- Residual-only rule, safety bound expansion
- `fault_metrics_test.cc` -- Pre/post RMSE, recovery time detection
- `cable_snap_profile_test.cc` -- Ramped and instantaneous fault transitions
- `desired_geometry_redistributor_test.cc` -- Centering, scaling, blending
- `reference_trajectory_mapper_test.cc` -- Composition and flattening

---

## Experiment Pipeline

### Python Experiment Scripts (`experiments/python/`, 25 files)

The Python layer provides reduced-order simulation for rapid parameter
exploration and orchestrates the C++ full-Drake runs.

**Core simulation:**
`topology_invariance_root.py` implements a reduced-order multi-agent
suspended-load simulator with spring-damper cables, wind disturbance,
and CL/L1 controller variants. Used for rapid prototyping before
committing to full Drake simulation.

**Drake batch orchestration:**
- `run_fault_schedule_batch.py` -- Defines all 30+ scenarios, invokes the
  C++ executable, generates per-scenario figures and batch comparisons
- `run_monte_carlo.py` -- Orchestrates 30-run Monte Carlo campaigns
- `run_broadened_monte_carlo.py` -- Extended parameter-sweep Monte Carlo
- `analyze_full_drake_batch.py` -- Post-processes batch results into
  comparison plots, CSV tables, and markdown reports

**Analysis and reporting:**
- `compute_hinf_norms.py` -- H-infinity small-gain verification
- `analyze_cl_history_corruption.py` -- CL bias duration analysis
- `validate_topology_invariance_conditions.py` -- Thrust-margin screening
- `build_fault_tolerance_report.py` -- Consolidated report generation

**Reduced-order sweeps:**
- `run_l1_matrix_sweep.py` -- Multi-seed, multi-cable parameter search
- `run_fault_matrix.py` -- Matched validation across seed/cable matrices
- `search_non_cl_adaptive_laws.py` -- Structured search across 6
  controller families (12 candidates, 0 robust winners vs CL)

---

## Simulation Outputs

All results live in `outputs/` (~1.7 GB). A detailed README with
per-scenario descriptions, paper table mappings, and links to every
interactive 3D replay is in `outputs/README.md`.

### Output Structure

```
outputs/
  full_drake_fault_batch/     30 scenario directories (Tables I-VIII, Figs. 1-5)
  monte_carlo/                Aggregated MC results (Table IX)
  monte_carlo_broadened/      Broadened parameter-sweep MC (Section VI.D)
  eso_n{3,5,7}_{nom,fault}/  ESO evaluation runs (Table VII)
  gain_sched_{moderate,aggressive}/  Gain-scheduled PID baselines (Table IV)
  linearize/                  Scalar model linearization (Section IV)
  hinf_scalar_results.json    H-infinity analysis (Section IV)
  revision_reports/           Methodology documentation (9 steps)
```

### Per-Scenario Artifacts

Each Drake run produces:
- `full_drake_meshcat_replay.html` -- Interactive 3D replay (open in browser)
- `full_drake_recording.npz` -- Full state trajectory (NumPy archive)
- `scenario_summary.json` -- Computed metrics (RMSE, peak, rho_FT)
- `run_manifest.json` -- Experiment parameters
- `figures/` -- 19 auto-generated PNG diagnostic plots
- `logs/<timestamp>/` -- Raw CSV time-series data (trajectories, tensions,
  attitudes, control efforts)

---

## The Paper

The IEEE T-CST manuscript lives in `IEEE_T-CST/` with LaTeX source,
TikZ figures, simulation figures, and bibliography.

```
IEEE_T-CST/
  Main.tex                    Root document
  Sections/                   7 section files + supplementary material
  Figures/                    TikZ diagrams, full-Drake PNGs, baseline PDFs
  References/References.bib   Bibliography (~50 entries)
  IEEEtran.cls, .bst          IEEE template files
```

The paper is organized as:
1. Introduction -- problem, gap, key observation, contributions
2. Related Works -- cooperative transport, UAV FTC, adaptive control,
   safety-critical control, emergent resilience
3. Problem Statement -- system model, N-agnostic constraint, cable-snap
   fault event, FDIR incompatibility, formal problem
4. Fault-Tolerant Architecture -- PID control law, Proposition 1,
   finite-time bound, cost ratio, gap analysis
5. Experimental Methodology -- evidence levels, scenario families, metrics
6. Simulation Results -- canonical scenarios, deconfounding, mechanism
   identification, robustness validation
7. Discussion -- mechanism interpretation, comparison, open problems

To compile:
```bash
cd IEEE_T-CST
pdflatex Main.tex && bibtex Main && pdflatex Main.tex && pdflatex Main.tex
```

---

## Build and Run

### Prerequisites

- C++17 compiler (GCC 9+ or Clang 10+)
- CMake 3.16+
- [Drake](https://drake.mit.edu/) (for full simulation; install at /opt/drake)
- Python 3.8+ with NumPy, SciPy, Matplotlib (for experiments)
- Node.js + Playwright (optional, for MeshCat snapshot capture)

### Build Core (Drake-Free)

```bash
cmake --preset default
cmake --build build
ctest --test-dir build --output-on-failure
```

Builds `libgpac_fault_tolerance_core.a` and 6 unit test executables.

### Build with Drake

```bash
cmake --preset drake
cmake --build build
```

Builds additionally `libgpac_fault_tolerance_drake.a`,
`full_drake_fault_runner`, and `linearize_plant`.

### Run a Simulation

```bash
./build/full_drake_fault_runner \
  --num-quads 3 \
  --fault-schedule "0:7.0" \
  --duration 30 \
  --output-dir outputs/my_run
```

### Run the Full Batch

```bash
python experiments/python/run_fault_schedule_batch.py
```

### Run Monte Carlo

```bash
python experiments/python/run_monte_carlo.py
```

---

## Repository Structure

```
Tether_Grace/
  README.md                        This document
  CMakeLists.txt                   Build configuration (C++17, optional Drake)
  CMakePresets.json                Presets: "default" (core) and "drake" (full)
  .gitmodules                      Tether_Lift submodule reference
  LICENSE                          Apache License 2.0

  src/gpac_fault_tolerance/        C++ implementation (20 .h, 21 .cc)
    contracts/signal_contracts.h     Signal vocabulary types
    core/                            Drake-free algorithms (5 modules)
    adapters/                        GPAC signal translation
    drake/                           Drake LeafSystem wrappers + runner

  tests/                           Unit tests (6 GTest files)

  experiments/                     Python experiment pipeline
    manifests/                       JSON parameter grids (12 files)
    python/                          Experiment scripts (25 .py files)

  IEEE_T-CST/                      Paper manuscript
    Main.tex                         Root LaTeX document
    Sections/                        7 sections + supplementary
    Figures/                         TikZ, PNGs, PDFs, data
    References/                      Bibliography

  outputs/                         Simulation results (~1.7 GB)
    README.md                        Detailed guide with paper mappings
    full_drake_fault_batch/          30 Drake scenarios (623 MB)
    monte_carlo/                     MC summary statistics
    monte_carlo_broadened/           Broadened MC results
    eso_n{3,5,7}_{nom,fault}/       ESO evaluation (6 runs)
    gain_sched_{moderate,aggressive}/ Gain-scheduled baselines
    linearize/                       Scalar linearization
    hinf_scalar_results.json         H-infinity analysis
    revision_reports/                Methodology documentation

  references/                      Theory documents and reference code
    problem_definition.{tex,pdf}     Problem formulation
    fault_bounds_*.{tex,pdf}         Fault bound analysis
    elevation_plan.{tex,pdf}         Research plan
    literature_review.tex            200+ papers surveyed
    *.{h,cc,py}                      Reference implementations
    SETUP.md, INTEGRATION_GUIDE.md   Development guides

  analysis/                        Gap analysis documentation

  Tether_Lift/                     Read-only submodule (GPAC baseline)
    Research/cpp/                     GPAC controller, estimation, sensors
    drake/                           Drake system definitions
    IEEE_IROS_2026/                  Earlier publication

  .github/                         Agent definitions and CI configuration
  .devcontainer -> Tether_Lift/DevContainers/.devcontainer
  package.json                     Playwright dependency (MeshCat capture)
  fix_bib*.py, verify_bib.py       Bibliography management utilities
```

---

## Relationship to Tether Lift

Tether Lift is the predecessor system, developed for IEEE IROS 2026, that
established the GPAC baseline: N-agnostic cooperative lift using only local
measurements. It validated 23.7 +/- 1.5 cm payload tracking RMSE across
a 15-seed Monte Carlo study with the full four-layer hierarchy (PID +
geometric SO(3) + concurrent learning + ESO + CBF safety filter).

Tether Grace asks the next question: does that same architecture also
provide resilience to abrupt cable-topology changes? The answer led to the
present paper.

The boundary is strict:
- **Tether_Lift/** is a read-only git submodule. No modifications are made.
- **All new implementation** lives in the Tether Grace root repository.
- The `full_drake_fault_runner` links against Tether Lift's Drake plant
  definitions and sensor models but adds fault injection, the extended
  controller, and the complete experiment orchestration.

---

## Open Problems

The paper explicitly identifies what is proven, what is empirically
supported, and what remains open:

| Claim | Evidence level |
|-------|---------------|
| Topology-independent error dynamics (A,B) for simplified model | **Proven** (Proposition 1) |
| Finite-time transient prediction (alpha ~= 5.8, topology-invariant) | Analytical + empirical |
| Cost ratio rho_FT = N/(N-k) | Derived, validated |
| Bounded tracking under faults in full nonlinear Drake plant | Empirically supported |
| Small-gain condition for cable-force feedback | Scalar verified (gamma_ISS = 0.125); full-plant open |
| Formal stability for full bead-chain plant | **Open** |

Three specific gaps remain:
1. Bounding the impulsive component of slack-to-taut cable forces
2. Cascaded position/attitude stability under fast force transients
3. Numerical small-gain verification for the full 6+8N dimensional plant

An L1 adaptive controller is implemented in the codebase as an optional
residual-adaptation path. Head-to-head comparison with the PID-only baseline
under ESKF+wind conditions is planned as near-term future work.

Hardware validation with physical multi-quadrotor cable transport and actual
cable-snap events remains the ultimate test.

---

## License

Apache License 2.0. See [LICENSE](LICENSE).
