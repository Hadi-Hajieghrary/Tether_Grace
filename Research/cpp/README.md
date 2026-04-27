# `Research/cpp/` — Simulation and Controller Source

This directory contains the entire Drake-based C++ code for the
Tether_Grace cooperative-lift research programme. A single executable,
`decentralized_fault_aware_sim`, instantiates the multibody plant,
wires in the controllers, drives the fault injection, and writes the
trajectory log plus the Meshcat replay. Everything else in the
workspace — analysis scripts, campaign runners, papers — consumes
what this target produces.

The document is organised as:

1. [System under study](#1-system-under-study)
2. [Theoretical foundations](#2-theoretical-foundations)
3. [Control-stack design](#3-control-stack-design)
4. [Fault model](#4-fault-model)
5. [Implementation notes](#5-implementation-notes)
6. [Build, run, and command-line reference](#6-build-run-and-command-line-reference)
7. [File-by-file source map](#7-file-by-file-source-map)
8. [What the simulator produces](#8-what-the-simulator-produces)
9. [Expected numerical behaviour](#9-expected-numerical-behaviour)
10. [Extending the harness](#10-extending-the-harness)

---

## 1. System under study

$N$ identical quadrotors cooperatively lift a point-mass payload
through $N$ flexible, bead-chain cables. The target `decentralized_fault_aware_sim`
is the forward simulator for this system; it exposes eight CLI flags
that select the fleet size, trajectory shape, payload mass, per-drone
controller, and the fault schedule.

| Parameter | Value | Notes |
|-----------|------:|-------|
| Drone mass $m_i$                | 1.5 kg | URDF quadrotor from the Tether_Lift baseline |
| Drone inertia (on-diag.)        | $\mathrm{diag}(0.0123,\,0.0123,\,0.0224)$ kg·m² | Tether_Lift URDF |
| Payload mass $m_L$              | 3.0 kg (default; `--payload-mass` overrides) | Point sphere, radius 0.15 m |
| Fleet size $N$                  | 4 or 5 (`--num-quads`) | Tested also at $N=6$ |
| Rope rest length $L$            | 1.25 m | 6 mm aramid-core lift sling |
| Rope segment stiffness $k_\text{seg}$ | 25 000 N/m | 8 beads in series → $k_\text{eff}=k_\text{seg}/N_\text{seg}$ with $N_\text{seg}=9$, i.e. **2778 N/m** |
| Rope segment damping $c_\text{seg}$ | 60 N·s/m | $\zeta \approx 1.2$ (slightly over-damped) |
| Actuator ceiling                 | Thrust 150 N, torque 10 N·m, tilt 0.6 rad | Per drone |
| Simulator step $\Delta t$       | $2\times 10^{-4}$ s | Factor-10 safety margin below $\omega_n$ of the bead chain |
| Gravity $g$                     | 9.81 m/s² | |

The formation is equiangular: drone $i$ is placed on a circle of
radius $r_f = 0.8$ m above the payload at angle $\phi_i = 2\pi i/N$.
Three reference trajectories are available:

* `traverse` — a four-waypoint point-to-point manoeuvre, used for
  ramp, cruise, and descent tests.
* `figure8` — a Bernoulli lemniscate in $(x,y)$ of 3 m amplitude with
  a 16 s period (peak lateral acceleration $\approx 0.9$ m/s²).
* `lemniscate3d` — the figure-8 augmented with a 0.35 m vertical
  oscillation at the same angular rate; this is the harder,
  continuously-dynamic reference used in the five-drone campaign.

---

## 2. Theoretical foundations

### 2.1 Payload dynamics

The payload is a rigid point of mass $m_L$. In the world frame,
ignoring air drag,

$$m_L\,\ddot p_L \;=\; \sum_{i=1}^{N} \mathbf T_i \;-\; m_L g\,\hat{\mathbf e}_3,$$

with $\mathbf T_i\in\mathbb R^3$ the tension applied by cable $i$
at the payload-side attachment.

### 2.2 Bead-chain rope model

Each cable is a Kelvin–Voigt bead chain: $N_\text{seg} = 9$ linear
spring–damper segments connecting $N_b = 8$ point-mass beads in
series, with the top bead welded to the drone's attachment frame and
the bottom bead welded to the payload. The per-segment tension law
along unit vector $\hat n_s$ is

$$T_s \;=\; \max\!\bigl(0,\; k_\text{seg}\,\delta_s + c_\text{seg}\,\dot\delta_s\bigr),$$

where $\delta_s$ is the segment stretch relative to its rest length
and the $\max$ enforces **tension-only** behaviour (a rope cannot
push). Because the segments are in series and we observe tension at
the top bead, the drone-side effective stiffness is
$k_\text{eff} = k_\text{seg}/N_\text{seg} = 2778$ N/m.

### 2.3 Per-drone dynamics

Drone $i$ is a rigid body with the usual 6-DoF floating-base
dynamics,

$$m_i\,\ddot p_i = \mathbf F_i - m_i g\,\hat{\mathbf e}_3 - \mathbf T_i,\qquad
I_i\,\dot\omega_i + \omega_i\times I_i\omega_i = \boldsymbol\tau_i,$$

where $\mathbf F_i = f_i\,R_i\hat{\mathbf e}_3$ is the thrust applied
along the body-$z$ axis and $\boldsymbol\tau_i\in\mathbb R^3$ is the
commanded body torque. The commanded tilt is recovered from the
desired horizontal acceleration via the small-angle mapping
$\theta_\text{pitch}=a_x/g,\ \theta_\text{roll}=-a_y/g$.

### 2.4 Hover equilibrium (initial condition)

The simulator spawns the system directly at its static-hover fixed
point so that $t=0$ contains no pickup transient. At equilibrium the
rope chord slightly exceeds $L$ to carry the payload:

$$L_\text{chord} = \sqrt{r_\text{eff}^2 + \Delta z_\text{att}^2},\quad
T_\text{nom} = m_L g\,\frac{L_\text{chord}}{N\,\Delta z_\text{att}},\quad
\delta = T_\text{nom}/k_\text{eff}.$$

Here $r_\text{eff} = 0.7\,r_f$ accounts for the fact that the payload
attachment point sits at $0.3 r_f$ radially inside the payload body,
and $\Delta z_\text{att} = \rho - \Delta z_\text{quad}$ is the
vertical drop between drone and payload *attachment* points (not
bodies). The main file solves the fixed point by a five-iteration
Picard sweep that converges to $\delta\approx 3$ mm at the default
parameters.

### 2.5 Shared feed-forward reference

Every drone is given the same piecewise-polynomial payload reference
$p_L^d(t)$. Its own slot reference is

$$p_{i,\text{slot}}(t) = p_L^d(t) + \Delta_i + \rho\,\hat{\mathbf e}_3,\quad
\Delta_i = r_f\,\bigl(\cos\phi_i,\sin\phi_i,0\bigr),$$

where $\rho$ is the vertical drop. No peer information flows between
drones — they see the same reference and their own local physics.

---

## 3. Control-stack design

The control stack is layered so each layer is independently switchable
by a CLI flag. With all flags off the baseline QP runs stand-alone;
flags can be composed in any combination.

```
  reference p_L^d(t)  ───────────────────────┐
                                             │
        ┌───────────────────────────┐        │
        │ FormationCoordinator      │◄─ fault_id
        │ (dynamic slot offsets)    │        │
        └──────────────┬────────────┘        │
                       │ 3N offsets          │
                       ▼                      ▼
       ┌──────────────────────────────────────────────┐
       │      Per-drone controller (selectable)        │
       │   baseline  — DecentralizedLocalController    │
       │   mpc       — MpcLocalController              │
       │   (both share the thrust-and-tilt synthesis   │
       │    and the attitude PD inner loop)            │
       └──────────────────────┬────────────────────────┘
                              ▼
         Drake MultibodyPlant + SceneGraph
            (beads, URDF drones, payload,
             fault gates, Meshcat)
```

### 3.1 Baseline — single-step QP

The outer loop is a cascade proportional–derivative controller that
produces a desired acceleration

$$a_\text{target} = a_\text{track} + w_\text{swing}\,a_\text{swing},$$

with

$$a_\text{track} = K_p(p_\text{slot}^\text{dyn} - p_i) + K_d(v_\text{slot} - v_i),\quad
a_\text{swing} = -k_\text{swing}\begin{bmatrix}v_{L,x}\\v_{L,y}\\0\end{bmatrix},$$

where the "dynamic slot" is the nominal slot shifted against the
payload swing by $-k_\text{swing}v_L$, saturated in norm. The
accelerations are then projected onto the actuator envelope by a
3-variable convex QP

$$\min_{a\in\mathbb R^3}\ \lVert a - a_\text{target}\rVert_{W_t}^2
   + \lVert a\rVert_{W_e}^2
\ \ \text{s.t.}\ \ |a_x|,|a_y|\le g\tan\theta_\text{max},\
a_z\in[a_{z,\min},a_{z,\max}],$$

solved every sim tick via Drake's `MathematicalProgram` → OSQP wrapper.
The thrust envelope is a function of the measured rope-tension
feed-forward $T_\text{ff}$, which tracks the measured tension after a
$C^1$-smoothstep pickup ramp (zero slope at both endpoints). The QP
solution is mapped to body thrust and desired tilt consumed by a Lee-
style proportional–derivative attitude inner loop with box-saturated
torques.

### 3.2 L1 adaptive outer loop (`--l1-enabled`)

Adds a single scalar correction on the altitude channel,

$$a_\text{target}^z \;\leftarrow\; a_\text{target}^z + u_\text{ad}(t),$$

where $u_\text{ad}$ is the low-pass-filtered output of a Lyapunov-
projected adaptive law running on a second-order state predictor. The
adaptive law tracks the matched altitude-channel disturbance
$\sigma(t)$ arising from unknown payload mass, rope-stiffness drift,
and residual rope-geometry bias; projection keeps $\hat\sigma$ inside
the admissible interval. A companion gradient estimator tracks
$\hat k_\text{eff}$ online whenever the rope is taut. Full derivation
and stability proof are in
[report Chapter 6](../../report/sections/06_l1_adaptive.tex) and the
archived theory note at
[`../../archive/docs_2026_04_23/theory/theory_l1_extension.md`](../../archive/docs_2026_04_23/theory/theory_l1_extension.md).

### 3.3 Receding-horizon MPC (`--controller=mpc`)

The MPC is a drop-in replacement for the baseline; the harness picks
between the two at construction time. Error-state double-integrator
prediction,

$$x_i(k+1) = A\,x_i(k) + B\,U_k,\qquad
A=\begin{bmatrix}I_3 & \Delta t\,I_3\\ 0 & I_3\end{bmatrix},\ \
B=-\begin{bmatrix}\tfrac{\Delta t^2}{2}I_3\\ \Delta t\,I_3\end{bmatrix},$$

is stacked over a horizon of $N_p$ steps using a pre-computed
$\Phi\in\mathbb R^{6N_p\times 6}$ and block-lower-triangular
$\Omega\in\mathbb R^{6N_p\times 3N_p}$, and the QP is solved per tick.
Tension is enforced through a first-order linearisation,

$$T_i(k+j) \;\approx\; T_\text{meas}(k) + c_j^\top(A^j - I)\,x(k)
                      + c_j^\top\,\Omega_{\text{row }j}\,U
                      + j\Delta t\,\dot T_\text{meas}(k),$$

with $c_j = [-k_\text{eff}\hat n(k);\,0]^\top$. The
$T_j \le T_\text{max}$ inequality is enforced with a soft slack
$s_j\ge 0$ and a large penalty $w_s$, guaranteeing recursive
feasibility. The cost is reference-tracking rather than state-regulator
(this matters at $\Delta t = 2$ ms; see the caveat section of the MPC
theory doc). Recursive feasibility and linearisation fidelity are
Theorems 2 and 4 of the supplementary.

### 3.4 Formation-reshape supervisor (`--reshaping-enabled`)

A [`FaultDetector`](include/fault_detector.h) latches the faulted
drone index the first time its rope tension stays below 0.5 N for
100 ms; a [`FormationCoordinator`](include/formation_coordinator.h)
then emits new angular slot offsets computed in closed form. For
$N=4\to M=3$ the drone diametrically opposite the gap stays fixed and
the two drones adjacent to the gap each rotate $\pi/6$ radians
toward it — this is the globally tension-optimal assignment
(Theorem 5). Transitions use the quintic smoothstep
$h(\tau) = 10\tau^3 - 15\tau^4 + 6\tau^5$ over $T_\text{trans} = 5$ s,
which is $C^2$ at both endpoints and keeps the minimum pairwise
chord distance above the safe-clearance threshold throughout
(Proposition 7). Exactly one scalar (`fault_id`) is broadcast per
fault event; no peer-state exchange is introduced.

### 3.5 Concurrent-learning observer (`--adaptive`)

An optional diagnostic channel. The
[`CLParamEstimator`](include/cl_param_estimator.h) runs a two-channel
rank-maximising history-buffer estimator that produces $\hat m_L/N$
and $\hat k_\text{eff}$ from the local payload acceleration and rope
stretch. It does **not** drive the controller — it is wired only to
logging — so it can be used for post-run cross-validation of the
L1 adaptive estimates without closing any additional loop.

---

## 4. Fault model

A cable fault is implemented with four simultaneous "gates" so the
rest of the stack sees a physically and telemetrically consistent
severance:

1. [`CableFaultGate`](include/cable_fault_gate.h) — zeros the spatial
   force emitted by the bead-chain physics model from $t\ge t^\star$.
   The rope ceases to carry load.
2. `TensionFaultGate` (in the harness) — zeros the scalar tension
   signal fed to the controllers, so observers see the fault
   immediately.
3. [`FaultAwareRopeVisualizer`](include/fault_aware_rope_visualizer.h)
   +
   [`MeshcatFaultHider`](include/meshcat_fault_hider.h) — hide the
   severed rope's polyline in Meshcat so the replay is consistent.
4. [`SafeHoverController`](include/safe_hover_controller.h) +
   [`ControlModeSwitcher`](include/control_mode_switcher.h) — put the
   faulted drone in a safe-hover autopilot so it does not try to keep
   tracking the payload.

The harness supports up to three simultaneous faults through
`--fault-0-quad/--fault-0-time`, `--fault-1-…`, and `--fault-2-…`.

---

## 5. Implementation notes

- **Drake discrete-time system composition.** Every controller,
  estimator, gate, and supervisor derives from
  `drake::systems::LeafSystem<double>`, with explicit input and
  output ports and periodic unrestricted-update events as needed.
  Composition is pure `DiagramBuilder::Connect`; there are no
  back-channels between LeafSystems.
- **Matched step sizes.** The physics integrator runs at
  $\Delta t = 2\times 10^{-4}$ s, the baseline controller at the same
  rate, the MPC at its own `dt_mpc = 2\times 10^{-4}$ s, and the L1
  adaptive update on its own periodic event. Everything is
  numerically in-phase.
- **Pickup is handled once.** `initial_pretensioned = true` (default)
  tells both controllers the rope is already taut at the hover
  equilibrium, so $T_\text{ff}$ = measured tension from the first
  tick. If the user spawns the system off-equilibrium the ramp is
  the $C^1$ smoothstep described in §3.1.
- **Per-drone tension measurement.** Each drone only sees *its own*
  scalar tension (plus the 3-vector rope force on its body). The
  harness demultiplexes a single plant-side tension signal into
  per-drone ZOH-filtered channels.
- **OSQP is deterministic.** The same CLI invocation produces the
  same trajectory to within IEEE-754 rounding; no random seeds
  anywhere in the active controller stack.
- **Observability of the payload state.** The controllers read the
  payload pose and velocity directly from the plant, standing in for
  an onboard range-sensor / visual-tracker that is modelled in
  deployment but not in this sim.

The five `.cc` files under `src/` total about 2 500 lines; the fifteen
`.h` files under `include/` another 1 800 lines. All major classes
fit in a single header and, where needed, one implementation file.

---

## 6. Build, run, and command-line reference

### 6.1 Build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target decentralized_fault_aware_sim -j8
```

The `CMakeLists.txt` expects `drake::drake` and `Eigen3::Eigen` on
the CMake prefix path; the devcontainer at the repo root provides
both. A single executable lands at `build/decentralized_fault_aware_sim`.

### 6.2 CLI reference

| Flag | Default | Meaning |
|------|---------|---------|
| `--num-quads N` | 4 | Number of drones in the formation. |
| `--duration T` | 20 | Simulation wall-time in seconds. |
| `--output-dir path` | `./decentralized_replays` | Destination for CSV + Meshcat HTML. |
| `--scenario name` | `A_nominal` | Informational tag written into the replay filename. |
| `--trajectory t` | `traverse` | `traverse`, `figure8`, or `lemniscate3d`. |
| `--fault-k-quad i`, `--fault-k-time t` | — | Inject fault `k∈{0,1,2}` on drone `i` at time `t`. |
| `--payload-mass m` | 3.0 | Overrides the default payload mass (kg) for robustness sweeps. |
| `--controller {baseline,mpc}` | `baseline` | Selects the per-drone controller. |
| `--mpc-horizon N` | 5 | MPC prediction horizon (ticks). |
| `--mpc-tension-max N` | 100 | MPC hard ceiling on per-cable tension (N). |
| `--l1-enabled` | off | Enables the L1 adaptive augmentation. |
| `--adaptive` | off | Runs the concurrent-learning observer alongside (diagnostic). |
| `--reshaping-enabled` | off | Instantiates `FaultDetector` + `FormationCoordinator`. |
| `--wind-speed m/s` | 0 | Mean wind along +x; 0 disables. Value > 0 wires a Dryden turbulence model and an aerodynamic-drag applicator on every drone and the payload. |
| `--wind-seed N` | 42 | Random seed for the turbulence generator; fixed value ⇒ deterministic gust realisation. |

### 6.3 Minimal invocation

```bash
./build/decentralized_fault_aware_sim \
    --num-quads 5 --duration 40 --trajectory lemniscate3d \
    --scenario D_dual_10sec \
    --fault-0-quad 0 --fault-0-time 15 \
    --fault-1-quad 2 --fault-1-time 25 \
    --controller mpc --l1-enabled --reshaping-enabled \
    --output-dir /tmp/tg_fullstack
```

Canonical combinations used in the paper are wrapped by
`../scripts/run_5drone_campaign.sh` and
`../scripts/run_transactions_campaign.sh`.

---

## 7. File-by-file source map

### 7.1 Implementation files (`src/`)

| File | Purpose |
|------|---------|
| `decentralized_fault_aware_sim_main.cc` | Entry point. Parses CLI, builds the MultibodyPlant + SceneGraph, instantiates bead-chain ropes and URDF drones, solves the hover-equilibrium fixed point, wires controllers / fault gates / loggers, runs the Drake simulator, and writes the CSV + Meshcat replay. |
| `decentralized_local_controller.cc` | Baseline cascade-PD + anti-swing + 3-variable QP + Lee-style attitude inner loop + optional L1 adaptive augmentation. |
| `mpc_local_controller.cc` | Error-state receding-horizon MPC with the linearised tension-ceiling constraint, DARE terminal cost, and reference-tracking cost. Same port signature as the baseline. |
| `cl_param_estimator.cc` | Concurrent-learning two-channel observer for $\hat m_L/N$ and $\hat k_\text{eff}$; diagnostic only. |
| `fault_aware_rope_visualizer.cc` | Meshcat polyline of each rope chord; hides the polyline after the fault time so the replay remains consistent with the physics. |

### 7.2 Public headers (`include/`)

Grouped by role.

#### Controllers and utilities

| Header | Purpose |
|--------|---------|
| `decentralized_local_controller.h` | Class, `Params` struct, port interface. |
| `mpc_local_controller.h` | Class, `Params` struct, port interface. |
| `controller_utils.h` | Shared helpers: `ComputeSlotReferenceAt`, `ComputePayloadReferenceAt`, `SolveScalarDARE`, LQR gain, closed-loop spectral-radius check. |

#### Fault subsystem

| Header | Purpose |
|--------|---------|
| `cable_fault_gate.h` | Physical gate — zeroes the cable's spatial-force output at $t^\star$. |
| `fault_detector.h` | Latches `fault_id` from the N-vector of tensions with a time-hysteresis threshold. |
| `fault_aware_rope_visualizer.h` | Polyline visualiser that accepts a fault-id input and suppresses the faulted rope. |
| `meshcat_fault_hider.h` | Helper that walks the Meshcat scene to hide a named asset after a trigger. |
| `safe_hover_controller.h` | Autopilot the faulted drone switches to. |
| `control_mode_switcher.h` | Latching selector between the nominal controller and the safe-hover controller. |

#### Estimator and adaptation

| Header | Purpose |
|--------|---------|
| `cl_param_estimator.h` | Concurrent-learning observer; rank-maximising history buffer; outputs $(\hat m_L/N,\ \hat k_\text{eff})$ + innovation + rank margin + buffer size. |
| `rope_segment_tension_probe.h` | Per-segment tension observer used by the per-rope tension waterfall figure. |

#### Supervisor and formation

| Header | Purpose |
|--------|---------|
| `formation_coordinator.h` | Header-only. On receipt of `fault_id`, emits the 3N-vector of dynamic formation offsets following a $C^2$ quintic smoothstep. |

#### Disturbance and reproducibility scaffolding

| Header | Purpose |
|--------|---------|
| `dryden_wind_model.h` | Header-only Dryden-spectrum turbulence source. Ready for the stochastic-robustness follow-up; not yet wired in the harness. |
| `imu_noise_injector.h` | Band-limited Gaussian noise injector on accel / gyro. Ready; not yet wired. |
| `comm_channel_model.h` | Delay + Bernoulli-drop channel for the reshape-broadcast path. Ready; not yet wired. |
| `run_manifest.h` | Emits `manifest.yaml` per run (git SHA, binary SHA256, platform, CLI, seed, UTC window). Called from the simulator main. |

---

## 8. What the simulator produces

Every run writes two artefacts to `--output-dir`:

1. **`scenario_<name>.csv`** — one row per sim tick (≈ 5 000 rows per
   second of simulation). Columns, in order:
   - `t` — simulation time (s);
   - per drone: position `x,y,z`, velocity `vx,vy,vz` (six columns);
   - payload position and velocity (six columns);
   - reference position and velocity (six columns);
   - per drone: scalar top-segment rope tension (one column);
   - per drone: the 13-element controller diagnostics vector —
     `swing_speed`, `swing_offset_mag`, `qp_cost`,
     `qp_solve_time_us`, `T_ff`, `thrust_cmd`, `tilt_mag`, and six
     active-set indicator bits;
   - per drone: the 6-element control wrench `tau_xyz, f_xyz`;
   - per drone per segment: the bead-chain tension
     $T_{i,j}(t)$;
   - per drone (only when `--adaptive`): CL estimator output
     $(\hat m_L/N,\hat k_\text{eff},\text{innov},\text{rank},\text{hist}\,|\mathcal H|)$;
   - per drone (only when `--l1-enabled`): L1 adaptive state
     $(\hat e_z, \dot{\hat e}_z, \hat\sigma, u_\text{ad}, \hat k_\text{eff})$.

2. **`scenario_<name>.html`** — a self-contained Meshcat replay that
   can be opened in any browser and scrubbed frame-by-frame.

The analysis pipeline under `../analysis/ieee/` consumes these CSVs to
produce the publication figures and the metric summaries that the
paper tables cite.

---

## 9. Expected numerical behaviour

The pre-registered IEEE-Transactions campaign has not yet run, so the
authoritative headline numbers are not yet available. The entries in
this section are **pre-campaign expectations**, not measurements. The
live numbers will appear in
`../../output/transactions_campaign/publication_metrics.csv` once the
campaign completes.

Smoke-test observations on a 3-second four-drone traverse (baseline
default, see the CLI example in §6.3) give a usable regression floor:

| Quantity | Baseline | MPC ($N_p = 5$) |
|----------|---------:|-----------------:|
| QP $p_{99}$ solve time | ≈ 41 µs | ≈ 204 µs |
| DARE $\rho(A - BK)$ | — | 0.969 |

The MPC solve cost is roughly five times the baseline QP on the
reference x86_64 platform; both stay well below the 500 µs budget at
$N_p = 5$. Peak tensions on the smoke test are ≈ 10.8 N per rope
(static payload share, no aggressive manoeuvre).

Qualitative expectations for the full campaign:

- RMS tracking increases monotonically with fault severity, for every
  configuration.
- `+L1` improves tracking under payload-mass bias (matched-disturbance
  rejection on the altitude channel) and rejects the rope-geometry
  residual in the nominal case.
- `+MPC` with a tight ceiling caps peak tension below the
  $T_\text{max}$ bound; with a loose ceiling it preserves baseline
  tracking because the reference-tracking cost reduces to the
  baseline target in the unconstrained regime.
- The reshape supervisor reduces $\sigma_T$ on scenarios where the
  surviving-drone hover-equilibrium asymmetry dominates the dynamic
  rope stretch; on aggressive vertical manoeuvres the benefit
  attenuates.

A campaign run whose numbers depart materially from these qualitative
expectations (RMS > 0.5 m on nominal, QP $p_{99}$ > 2 ms, unslacked
tension-ceiling violations when the MPC flag is set) is a regression
and must be triaged before the data is cited.

---

## 10. Extending the harness

- **Adding a new controller.** Implement a new LeafSystem with the
  same three ports (`plant_state`, `rope_tension`, optional
  `formation_offset_override`) and three outputs (`control_force`,
  `control_vector`, `diagnostics`). Register it in the
  `ctrl_system(q)` selector in
  [`decentralized_fault_aware_sim_main.cc`](src/decentralized_fault_aware_sim_main.cc)
  behind a new CLI string.
- **Wiring the stochastic axes.** The three disturbance headers
  under `include/` (`dryden_wind_model.h`, `imu_noise_injector.h`,
  `comm_channel_model.h`) compile and emit signals. The follow-up
  campaign that uses them needs the harness to insert them between
  the plant and the controller state input, and to expose CLI flags
  (`--wind-speed`, `--imu-noise-scale`, `--comm-delay-ms`,
  `--comm-drop-rate`) for the campaign runner to sweep. The
  `RunManifest` already records the seed so reproducibility is
  preserved.
- **Adding a supervisor.** The `formation_offset_override` port is
  the only back-channel the controllers accept. Any supervisor that
  writes to it must emit a 3-vector per drone; the harness wires up
  a Demultiplexer so the supervisor outputs a single 3N-vector and
  each controller reads its own slice.

Cross-references:

- Canonical algorithmic specification — [report](../../report/main.pdf) and
  [`../../report/sections/`](../../report/sections/).
- Full proofs of the Phase-T theorems — [`../../report/companions/theory/theory_baseline_stability.md`](../../report/companions/theory/theory_baseline_stability.md).
- CLI-flag reference — [report Appendix A4](../../report/sections/A4_cli_reference.tex).
- Archived per-layer theory notes (pre-report) — [`../../archive/docs_2026_04_23/theory/`](../../archive/docs_2026_04_23/theory/).
- Archived IEEE-T-CST draft + supplementary proofs — [`../../archive/docs_2026_04_23/latex/`](../../archive/docs_2026_04_23/latex/).
- Frozen experiment matrix (pre-P2) — [`../../archive/docs_2026_04_23/preregistration.md`](../../archive/docs_2026_04_23/preregistration.md).
- Campaign runners — [`../scripts/`](../scripts/).
- Theorem tests — [`../tests/`](../tests/).
