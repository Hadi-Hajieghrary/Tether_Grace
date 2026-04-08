# Simulation Outputs

This directory contains all simulation results backing the IEEE T-CST paper
*"Cable-Fault Tolerance in Decentralized Cooperative Aerial Transport."*
Every subdirectory maps to one or more tables, figures, or sections in the manuscript.

All results were produced by the `full_drake_fault_runner` executable
(Drake multibody simulation at 5 kHz) or by the Python experiment scripts
in `experiments/python/`. The simulation plant uses bead-chain cable dynamics
(8 beads per cable), geometric SO(3) attitude control at 200 Hz, and PID
position control at 50 Hz.

**Total size:** ~1.7 GB across ~2,900 files.

---

## Directory Overview

```
outputs/
  full_drake_fault_batch/     623 MB   Primary batch: 33 scenarios (Tables I-VIII, Figs. 1-5)
  monte_carlo/                976 MB   Stochastic validation: 120 runs (Table IX)
  monte_carlo_broadened/       14 MB   Broadened parameter sweep: 5 runs (Section VI.D)
  eso_n3_nom/                 2.8 MB   ESO evaluation, N=3 nominal (Table VII)
  eso_n3_fault/               2.8 MB   ESO evaluation, N=3 fault (Table VII)
  eso_n5_nom/                 4.3 MB   ESO evaluation, N=5 nominal (Table VII)
  eso_n5_fault/               4.3 MB   ESO evaluation, N=5 fault (Table VII)
  eso_n7_nom/                 5.8 MB   ESO evaluation, N=7 nominal (Table VII)
  eso_n7_fault/               5.8 MB   ESO evaluation, N=7 fault (Table VII)
  gain_sched_moderate/        2.8 MB   Gain-scheduled PID baseline (Table IV)
  gain_sched_aggressive/      2.8 MB   Gain-scheduled PID variant (Table IV)
  linearize/                   20 KB   Scalar model linearization (Section IV)
  hinf_scalar_results.json    717 B    H-infinity analysis summary (Section IV)
  revision_reports/            56 KB   Methodology documentation (9 revision steps)
```

---

## Per-Run Artifact Structure

Each simulation run produces a standard set of artifacts:

```
<scenario_name>/
  run_manifest.json               Experiment parameters (N, faults, gains, flags)
  scenario_summary.json           Computed metrics (RMSE, peak error, rho_FT, ...)
  full_drake_meshcat_replay.html  Interactive 3D replay (open in browser)
  full_drake_recording.npz        Full state trajectory (NumPy archive)
  figures/                        Auto-generated PNG plots:
    payload_tracking_error.png      Payload position error vs. time
    cable_tensions.png              Cable tension time histories
    xy_trajectories.png             XY plane trajectories (all agents + payload)
    quad_tilt_magnitude.png         Vehicle tilt angle vs. time
    quad_force_norm.png             Thrust magnitude per vehicle
    quad_formation_deviation.png    Formation geometry deviation
    payload_z_offset.png            Vertical offset from reference
    cable_health_schedule.png       Cable snap event timeline
    ... (19 plots total per run)
  logs/<timestamp>/               Raw CSV time-series data:
    trajectories.csv                Position, velocity, reference for all bodies
    tensions.csv                    Cable tension magnitudes
    attitude_data.csv               Quaternions, angular velocities
    control_efforts.csv             Thrust and torque commands
    config.txt                      Full configuration dump
```

ESO and gain-scheduling runs omit the meshcat replay and recording
(they contain only `run_manifest.json` and `logs/`).

---

## Drake 3D Simulation Replays and Recordings

Every scenario in `full_drake_fault_batch/` includes two key artifacts
that capture the full multibody simulation state:

- **`full_drake_meshcat_replay.html`** -- A self-contained, interactive 3D
  replay of the entire simulation. Open this file in any modern web browser
  to watch the quadrotors, cables, and payload in three dimensions. The
  viewer supports play/pause, scrubbing through time, orbiting the camera,
  and zooming. This is the most direct way to *see* what happens when a
  cable snaps: the freed agent accelerates upward, the payload drops, and
  the surviving agents absorb the load. Built on Drake's MeshCat visualizer.

- **`full_drake_recording.npz`** -- A NumPy compressed archive containing the
  full state trajectory at the visualization frame rate. Load with
  `numpy.load('full_drake_recording.npz')` to access position, orientation,
  and cable geometry for every body at every timestep. This is the
  programmatic counterpart to the HTML replay -- use it to produce custom
  plots, generate video frames, or extract specific state snapshots.

### Replay Index: All 33 Scenarios

Below is a complete index of every simulation replay, organized by the
experimental question each demonstrates. All paths are relative to
`outputs/full_drake_fault_batch/`.

---

#### Nominal Baselines (No Faults)

These three replays show the system under normal operation -- all cables
intact, all agents tracking the figure-eight trajectory. They establish
the visual baseline: coordinated formation flight with cable pendulum
oscillations growing near trajectory direction changes. Watch for the
rhythmic tension oscillation that reflects the ~2 s cable pendulum period
vs. the ~3 s trajectory corner interval.

| Scenario | Replay | Recording | What to observe |
|----------|--------|-----------|-----------------|
| **3 drones, no fault** | [replay](full_drake_fault_batch/nominal_n3/full_drake_meshcat_replay.html) | [recording](full_drake_fault_batch/nominal_n3/full_drake_recording.npz) | Three quadrotors in triangular formation tracking a figure-eight with a cable-suspended payload. Nominal pendulum dynamics visible. Baseline for the N=3 fault scenario. |
| **5 drones, no fault** | [replay](full_drake_fault_batch/nominal_n5/full_drake_meshcat_replay.html) | [recording](full_drake_fault_batch/nominal_n5/full_drake_recording.npz) | Five quadrotors in pentagonal formation. Slightly tighter formation geometry. Cable interactions more complex but tracking quality nearly identical to N=3 (~35 cm RMSE). |
| **7 drones, no fault** | [replay](full_drake_fault_batch/nominal_n7/full_drake_meshcat_replay.html) | [recording](full_drake_fault_batch/nominal_n7/full_drake_recording.npz) | Seven quadrotors in heptagonal formation. The most visually dense configuration. Demonstrates that N-agnostic control scales without degradation -- nominal RMSE (34.70 cm) is within 1.2% of N=3. |

---

#### Canonical Fault Scenarios (Paper Table I, Figs. 1-5)

The central demonstration of the paper. Each replay shows the moment of
cable snap, the transient response, and the post-fault recovery. The freed
agent climbs and displaces laterally (escape maneuver), while surviving
agents continue tracking with no explicit fault detection or reconfiguration.
Compare the three to see that larger teams absorb proportionally more faults
with *less* degradation.

| Scenario | Replay | Recording | What to observe |
|----------|--------|-----------|-----------------|
| **3 drones, 1 fault** | [replay](full_drake_fault_batch/three_drones/full_drake_meshcat_replay.html) | [recording](full_drake_fault_batch/three_drones/full_drake_recording.npz) | Cable 0 snaps at t=7.0 s. Watch drone 0 shoot upward while drones 1-2 absorb a 50% load-share jump. The payload dips ~1 m, then the two surviving agents recover tracking within ~2 s. This is the most stressed case: only 2 agents remain. RMSE 48.84 cm. Source for Fig. 2a, Fig. 3a. |
| **5 drones, 2 cascading faults** | [replay](full_drake_fault_batch/five_drones/full_drake_meshcat_replay.html) | [recording](full_drake_fault_batch/five_drones/full_drake_recording.npz) | Cable 1 snaps at t=7.0 s, cable 3 at t=12.0 s. Two sequential fault transients, each producing a bounded payload excursion. After both faults, 3 agents continue tracking (67% load increase). RMSE 46.37 cm -- lower than N=3 despite two faults. Source for Fig. 2b, Fig. 3b. |
| **7 drones, 3 cascading faults** | [replay](full_drake_fault_batch/seven_drones/full_drake_meshcat_replay.html) | [recording](full_drake_fault_batch/seven_drones/full_drake_recording.npz) | Cables 1, 3, 5 snap at t=7.0, 12.0, 14.0 s. The most demanding scenario: three freed agents depart in sequence, and the third fault arrives only 2 s after the second, before full recovery. Four agents remain (75% load increase). RMSE 43.60 cm -- 11% *better* than N=3 with one fault. Source for Figs. 1-4. |

---

#### Matched Single-Fault Experiments (Paper Table II)

These three replays apply the *identical* fault event (cable 0 at t=7.0 s)
across all team sizes. This deconfounds team size from fault count. Compare
them side by side: the initial transient is nearly identical (~107-112 cm
peak), but larger teams recover faster mid-trajectory, producing lower RMSE.

| Scenario | Replay | Recording | What to observe |
|----------|--------|-----------|-----------------|
| **Matched N=3, 1 fault** | [replay](full_drake_fault_batch/matched_n3_1fault/full_drake_meshcat_replay.html) | [recording](full_drake_fault_batch/matched_n3_1fault/full_drake_recording.npz) | Same as canonical 3-drone. 2 surviving agents, capacity ratio 2/3. RMSE 48.84 cm. The baseline for the matched comparison. |
| **Matched N=5, 1 fault** | [replay](full_drake_fault_batch/matched_n5_1fault/full_drake_meshcat_replay.html) | [recording](full_drake_fault_batch/matched_n5_1fault/full_drake_recording.npz) | Same cable 0 fault, but 4 agents remain (capacity 4/5). Watch how the transient is visually similar but mid-trajectory recovery is noticeably faster. RMSE drops to 40.37 cm (17% improvement over N=3). |
| **Matched N=7, 1 fault** | [replay](full_drake_fault_batch/matched_n7_1fault/full_drake_meshcat_replay.html) | [recording](full_drake_fault_batch/matched_n7_1fault/full_drake_recording.npz) | Same cable 0 fault, 6 agents remain (capacity 6/7). The fault is barely visible in the payload trajectory -- each surviving agent absorbs only a 17% load increase. RMSE 38.38 cm (21% improvement over N=3). |

---

#### Fault-Count Isolation at N=7 (Paper Table III)

Holds team size constant at N=7 while increasing the number of faults from
0 to 3. Watch these in sequence to see graceful, incremental degradation:
each additional fault adds ~2-5 cm RMSE without catastrophic failure.

| Scenario | Replay | Recording | What to observe |
|----------|--------|-----------|-----------------|
| **N=7, 0 faults** | [replay](full_drake_fault_batch/nominal_n7/full_drake_meshcat_replay.html) | [recording](full_drake_fault_batch/nominal_n7/full_drake_recording.npz) | Baseline: all 7 cables intact. RMSE 34.70 cm. (Same as nominal N=7 above.) |
| **N=7, 1 fault** | [replay](full_drake_fault_batch/n7_1fault/full_drake_meshcat_replay.html) | [recording](full_drake_fault_batch/n7_1fault/full_drake_recording.npz) | Cable 1 snaps at t=7.0 s. Minimal visible impact on payload -- 6 remaining agents absorb the load easily. RMSE 39.39 cm (+4.7 cm from nominal). |
| **N=7, 2 faults** | [replay](full_drake_fault_batch/n7_2fault/full_drake_meshcat_replay.html) | [recording](full_drake_fault_batch/n7_2fault/full_drake_recording.npz) | Cables 1, 3 snap at t=7.0 and 12.0 s. Two departing agents visible. RMSE 41.29 cm. Peak error highest here (118.21 cm) -- the second fault at t=12 s catches an unfavorable trajectory phase. |
| **N=7, 3 faults** | [replay](full_drake_fault_batch/n7_3fault/full_drake_meshcat_replay.html) | [recording](full_drake_fault_batch/n7_3fault/full_drake_recording.npz) | Cables 1, 3, 5 snap at t=7.0, 12.0, 14.0 s. Three agents depart; four remain. The third fault arrives before full recovery from the second. RMSE 43.60 cm -- only +8.9 cm above nominal despite losing 43% of agents. |

---

#### Controller Comparison (Paper Table IV)

These replays compare the passive GPAC architecture against detection-based
alternatives. The visual differences are subtle -- all controllers achieve
similar tracking -- which is precisely the point: detection and centralized
knowledge provide negligible benefit.

| Scenario | Replay | Recording | What to observe |
|----------|--------|-----------|-----------------|
| **Centralized oracle, N=3** | [replay](full_drake_fault_batch/oracle_n3_1fault/full_drake_meshcat_replay.html) | [recording](full_drake_fault_batch/oracle_n3_1fault/full_drake_recording.npz) | Each surviving agent receives perfect post-fault load share m_L/(N-k) as gravity feedforward. Despite perfect fault knowledge and communication, RMSE is 49.18 cm -- *worse* than passive GPAC (48.84 cm) by 0.7%. Demonstrates that local cable sensing already captures nearly all fault-relevant information. |
| **Centralized oracle, N=7** | [replay](full_drake_fault_batch/oracle_n7_3fault/full_drake_meshcat_replay.html) | [recording](full_drake_fault_batch/oracle_n7_3fault/full_drake_recording.npz) | Oracle with 3 cascading faults. Multi-fault variant of the oracle comparison. Confirms the single-fault finding scales to the stress-test scenario. |
| **Reactive FTC, N=3** | [replay](full_drake_fault_batch/reactive_ftc_n3/full_drake_meshcat_replay.html) | [recording](full_drake_fault_batch/reactive_ftc_n3/full_drake_recording.npz) | Naive threshold-triggered controller: when T_i exceeds a threshold, thrust increases by +30%. Watch for slightly more aggressive lateral oscillation -- the thrust boost amplifies pendulum dynamics. RMSE 49.45 cm (+1.2% vs. passive). |

The gain-scheduled PID baselines (`gain_sched_moderate/`, `gain_sched_aggressive/`)
are in the top-level `outputs/` directory and use the same log format but
without meshcat replays.

---

#### Tension Feedforward Ablation (Paper Table V)

These replays test how much cable-tension feedforward contributes to fault
tolerance. The visual difference between kappa_T=0 (no tension feedforward)
and kappa_T=1.0 (full feedforward) is essentially invisible -- confirming
that PID + gravity feedforward is the primary mechanism.

| Scenario | Replay | Recording | What to observe |
|----------|--------|-----------|-----------------|
| **No tension FF** | [replay](full_drake_fault_batch/tension_ff_off/full_drake_meshcat_replay.html) | [recording](full_drake_fault_batch/tension_ff_off/full_drake_recording.npz) | Cable-tension feedforward completely disabled (kappa_T=0). The system looks identical to the default run. RMSE changes by only 0.44 cm (0.9%). This is the key ablation result. |
| **kappa_T = 0.00** | [replay](full_drake_fault_batch/kappa_t_0.00/full_drake_meshcat_replay.html) | [recording](full_drake_fault_batch/kappa_t_0.00/full_drake_recording.npz) | Sweep point: no tension feedforward. Part of the kappa_T sweep {0, 0.25, 0.5, 1.0}. |
| **kappa_T = 0.25** | [replay](full_drake_fault_batch/kappa_t_0.25/full_drake_meshcat_replay.html) | [recording](full_drake_fault_batch/kappa_t_0.25/full_drake_recording.npz) | Sweep point: 25% tension feedforward. Intermediate between off and default. |
| **kappa_T = 0.50** | [replay](full_drake_fault_batch/kappa_t_0.50/full_drake_meshcat_replay.html) | [recording](full_drake_fault_batch/kappa_t_0.50/full_drake_recording.npz) | Sweep point: 50% tension feedforward (default). Baseline for the ablation. |
| **kappa_T = 1.00** | [replay](full_drake_fault_batch/kappa_t_1.00/full_drake_meshcat_replay.html) | [recording](full_drake_fault_batch/kappa_t_1.00/full_drake_recording.npz) | Sweep point: full tension feedforward. Total RMSE span across the entire sweep is only 1.55 cm. |

---

#### Thrust Saturation Boundary (Paper Table VI)

These replays sweep the maximum available thrust from 150 N down to 35 N.
**This is the most visually dramatic set.** At f_max >= 45 N, the system
looks identical to the default. At f_max = 38 N, the system catastrophically
fails -- the payload plunges and agents lose formation. The sharp phase
transition at ~45 N coincides with the theoretical post-fault hover minimum.

| Scenario | Replay | Recording | What to observe |
|----------|--------|-----------|-----------------|
| **f_max = 150 N** | [replay](full_drake_fault_batch/fmax_150/full_drake_meshcat_replay.html) | [recording](full_drake_fault_batch/fmax_150/full_drake_recording.npz) | Default thrust authority (10.2x hover). Identical to canonical 3-drone. 3x headroom above operational requirement. RMSE 48.84 cm. |
| **f_max = 80 N** | [replay](full_drake_fault_batch/fmax_80/full_drake_meshcat_replay.html) | [recording](full_drake_fault_batch/fmax_80/full_drake_recording.npz) | 5.4x hover. Visually indistinguishable from 150 N. RMSE 48.84 cm (identical). |
| **f_max = 60 N** | [replay](full_drake_fault_batch/fmax_60/full_drake_meshcat_replay.html) | [recording](full_drake_fault_batch/fmax_60/full_drake_recording.npz) | 4.1x hover. Still indistinguishable. RMSE 48.84 cm (identical). |
| **f_max = 45 N** | [replay](full_drake_fault_batch/fmax_45/full_drake_meshcat_replay.html) | [recording](full_drake_fault_batch/fmax_45/full_drake_recording.npz) | 3.1x hover. Right at the operational edge. Barely detectable difference -- very slight increase in post-fault oscillation. RMSE 49.06 cm (+0.4%). |
| **f_max = 38 N** | [replay](full_drake_fault_batch/fmax_38/full_drake_meshcat_replay.html) | [recording](full_drake_fault_batch/fmax_38/full_drake_recording.npz) | **2.6x hover -- FAILURE.** Watch the payload drop dramatically after the cable snap. Agents cannot generate enough thrust to maintain altitude under the increased load share. RMSE explodes to 236.92 cm (4.9x degradation). The phase transition is visually unmistakable. |
| **f_max = 35 N** | [replay](full_drake_fault_batch/fmax_35/full_drake_meshcat_replay.html) | [recording](full_drake_fault_batch/fmax_35/full_drake_recording.npz) | **2.4x hover -- FAILURE.** Even more dramatic. Below the theoretical minimum, the system cannot sustain post-fault flight. RMSE 192.11 cm. |

---

#### Sensor-Realistic Evaluation (Paper Table VIII)

These replays activate the ESKF state estimator (fusing IMU at 400 Hz,
GPS at 10 Hz, barometer at 25 Hz) and the Dryden wind turbulence model.
The visual difference from ideal conditions is subtle -- slightly noisier
trajectories and ~10% higher RMSE -- confirming that the fault-absorption
mechanism is robust to realistic sensing.

| Scenario | Replay | Recording | What to observe |
|----------|--------|-----------|-----------------|
| **ESKF+wind, N=3, 1 fault** | [replay](full_drake_fault_batch/eskf_wind_n3_1fault/full_drake_meshcat_replay.html) | [recording](full_drake_fault_batch/eskf_wind_n3_1fault/full_drake_recording.npz) | Sensor noise and wind perturbations visible as slightly rougher trajectories. The fault transient is somewhat amplified by wind compounding the cable-snap impulse. RMSE 53.68 cm (+9.9% vs. ideal). |
| **ESKF+wind, N=5, 2 faults** | [replay](full_drake_fault_batch/eskf_wind_n5_2fault/full_drake_meshcat_replay.html) | [recording](full_drake_fault_batch/eskf_wind_n5_2fault/full_drake_recording.npz) | Two cascading faults under realistic conditions. Wind and estimation lag compound each fault transient. RMSE 50.14 cm (+8.1%). |
| **ESKF+wind, N=7, 3 faults** | [replay](full_drake_fault_batch/eskf_wind_n7_3fault/full_drake_meshcat_replay.html) | [recording](full_drake_fault_batch/eskf_wind_n7_3fault/full_drake_recording.npz) | The full stress test under realistic conditions: 3 cascading faults + ESKF + Dryden wind. Despite all perturbation sources, the system remains stable. RMSE 48.28 cm (+10.7%). Peak error 131.56 cm. |

---

#### Trajectory Generalization (Supplementary Table)

These replays test the fault-absorption mechanism on non-figure-eight
trajectories to confirm the results are not trajectory-limited.

| Scenario | Replay | Recording | What to observe |
|----------|--------|-----------|-----------------|
| **Hover, N=3, 1 fault** | [replay](full_drake_fault_batch/hover_n3_1fault/full_drake_meshcat_replay.html) | [recording](full_drake_fault_batch/hover_n3_1fault/full_drake_recording.npz) | Station-keeping hover with cable 0 fault at t=7.0 s. No trajectory-induced oscillation -- isolates the pure fault response. RMSE 39.75 cm (19% lower than figure-eight). Peak error only 64.39 cm vs. 107 cm, confirming the figure-eight amplifies post-fault error through cable-pendulum near-resonance. |
| **Hover, N=7, 3 faults** | [replay](full_drake_fault_batch/hover_n7_3fault/full_drake_meshcat_replay.html) | [recording](full_drake_fault_batch/hover_n7_3fault/full_drake_recording.npz) | Seven drones hovering with 3 cascading faults. Dramatically cleaner than the figure-eight version -- each fault produces a brief dip and rapid recovery. RMSE 30.88 cm, peak only 38.04 cm. |
| **Point-to-point, N=3, 1 fault** | [replay](full_drake_fault_batch/p2p_n3_1fault/full_drake_meshcat_replay.html) | [recording](full_drake_fault_batch/p2p_n3_1fault/full_drake_recording.npz) | Straight-line transit from [-2,0,z] to [2,0,z] over 10 s, then hold. Different frequency content from the figure-eight. RMSE 44.98 cm, peak 71.48 cm -- between hover and figure-eight. |
| **Point-to-point, N=7, 3 faults** | [replay](full_drake_fault_batch/p2p_n7_3fault/full_drake_meshcat_replay.html) | [recording](full_drake_fault_batch/p2p_n7_3fault/full_drake_recording.npz) | Seven drones, point-to-point with 3 cascading faults. RMSE 37.66 cm. Confirms fault tolerance holds across trajectory types. |

---

## Full Drake Fault Batch

**Directory:** `full_drake_fault_batch/` (623 MB, 33 scenario subdirectories)

This is the primary experiment batch. It also contains batch-level comparison
artifacts at the top level:

| File | Description |
|------|-------------|
| `batch_manifest.json` | Master manifest listing all 33 scenarios with parameters |
| `comparison_metrics.json` | Cross-scenario aggregated metrics |
| `comparison_report.md` | Human-readable batch summary |
| `comparison_scenarios.csv` | Tabular scenario parameters |
| `comparison_fault_events.csv` | Fault event timeline across scenarios |
| `comparison_load_error_overlay.png` | **Fig. 5** in the paper |
| `comparison_load_tracking_summary.png` | Aggregate tracking summary |
| `comparison_fault_event_separation.png` | Freed-agent clearance distances |
| `comparison_fault_response_timing.png` | Fault response timing overlay |
| `comparison_healthy_neighbor_clearance.png` | Healthy-agent separation distances |
| `comparison_load_speed_envelope.png` | Payload speed envelope |

### Canonical Scenarios -> Table I (`tab:scenario_summary`)

These six runs establish the core result: payload RMSE decreases with team
size despite absorbing proportionally more faults.

| Directory | N | Faults | RMSE | Peak | rho_FT |
|-----------|---|--------|------|------|--------|
| `nominal_n3` | 3 | 0 | 35.12 cm | 110.57 cm | --- |
| `nominal_n5` | 5 | 0 | 34.84 cm | 110.10 cm | --- |
| `nominal_n7` | 7 | 0 | 34.70 cm | 108.65 cm | --- |
| `three_drones` | 3 | 1 (cable 0, t=7.0 s) | 48.84 cm | 107.44 cm | 1.39 |
| `five_drones` | 5 | 2 (cables 1,3, t=7,12 s) | 46.37 cm | 112.30 cm | 1.33 |
| `seven_drones` | 7 | 3 (cables 1,3,5, t=7,12,14 s) | 43.60 cm | 117.45 cm | 1.26 |

The `three_drones`, `five_drones`, and `seven_drones` runs also produce the
per-scenario figures used in the paper:

- **Fig. 1** (`fig:scenario_seven`): Snapshots from `seven_drones` meshcat replay
- **Fig. 2** (`fig:tracking_errors_all`): `payload_tracking_error.png` from each
- **Fig. 3** (`fig:tensions_all`): `cable_tensions.png` from each
- **Fig. 4** (`fig:vehicle_dynamics_seven`): `xy_trajectories.png`,
  `quad_formation_deviation.png`, `quad_tilt_magnitude.png`,
  `quad_force_norm.png` from `seven_drones`

### Matched Single-Fault -> Table II (`tab:matched_single_fault`)

Isolates the effect of team size by applying the *same* fault event
(cable 0 at t=7.0 s) across all three team sizes.

| Directory | N | Capacity | RMSE | Peak | Final |
|-----------|---|----------|------|------|-------|
| `matched_n3_1fault` | 3 | 2/3 | 48.84 cm | 107.44 cm | 42.36 cm |
| `matched_n5_1fault` | 5 | 4/5 | 40.37 cm | 112.09 cm | 53.25 cm |
| `matched_n7_1fault` | 7 | 6/7 | 38.38 cm | 111.15 cm | 49.85 cm |

### Fault-Count Isolation -> Table III (`tab:fault_count_isolation`)

Holds N=7 constant while varying the number of faults k in {0,1,2,3}.

| Directory | k | Surviving | RMSE | Peak |
|-----------|---|-----------|------|------|
| `nominal_n7` | 0 | 7 | 34.70 cm | 108.65 cm |
| `n7_1fault` | 1 | 6 | 39.39 cm | 111.26 cm |
| `n7_2fault` | 2 | 5 | 41.29 cm | 118.21 cm |
| `n7_3fault` | 3 | 4 | 43.60 cm | 117.45 cm |

### Controller Comparison -> Table IV (`tab:baseline_comparison`)

Compares passive GPAC against detection-based alternatives on the
N=3 single-fault scenario.

| Directory | Controller | RMSE | Detection? | Communication? |
|-----------|-----------|------|------------|----------------|
| `oracle_n3_1fault` | Centralized oracle | 49.18 cm | Perfect | Yes |
| `three_drones` | Passive GPAC (this work) | 48.84 cm | No | No |
| `reactive_ftc_n3` | Threshold-triggered (+30%) | 49.45 cm | Threshold | No |

The gain-scheduled PID baselines are in the top-level directories
`gain_sched_moderate/` (1.5x k_p, 1.25x k_d -> 48.96 cm) and
`gain_sched_aggressive/` (variant). `oracle_n7_3fault` provides the
multi-fault oracle comparison.

### Tension Feedforward Ablation -> Table V (`tab:tension_ablation`)

Sweeps the cable-tension feedforward gain kappa_T in {0, 0.25, 0.5, 1.0}
on the N=3 single-fault scenario.

| Directory | kappa_T | RMSE | Peak | Final |
|-----------|---------|------|------|-------|
| `three_drones` | 0.5 (default) | 48.84 cm | 107.44 cm | 42.36 cm |
| `tension_ff_off` | 0 (disabled) | 48.40 cm | 107.50 cm | 43.13 cm |
| `kappa_t_0.00` | 0.00 | sweep | | |
| `kappa_t_0.25` | 0.25 | sweep | | |
| `kappa_t_0.50` | 0.50 | sweep | | |
| `kappa_t_1.00` | 1.00 | sweep | | |

Key finding: total RMSE variation across the full sweep is only 1.55 cm,
confirming PID + gravity feedforward as the primary mechanism.

### Thrust Saturation Boundary -> Table VI (`tab:fmax_boundary`)

Sweeps maximum thrust f_max from 150 N down to 35 N on the N=3
single-fault scenario. Identifies a sharp phase transition at ~45 N.

| Directory | f_max | f_max/f_hover | RMSE | Peak |
|-----------|-------|---------------|------|------|
| `fmax_150` | 150 N | 10.2x | 48.84 cm | 107.44 cm |
| `fmax_80` | 80 N | 5.4x | 48.84 cm | 107.44 cm |
| `fmax_60` | 60 N | 4.1x | 48.84 cm | 107.44 cm |
| `fmax_45` | 45 N | 3.1x | 49.06 cm | 107.66 cm |
| `fmax_38` | 38 N | 2.6x | 236.92 cm | 324.97 cm |
| `fmax_35` | 35 N | 2.4x | 192.11 cm | 250.34 cm |

### Sensor-Realistic Evaluation -> Table VIII (`tab:sensor_realistic`)

Activates ESKF state estimation and Dryden wind turbulence model.

| Directory | N | Faults | RMSE (ESKF+wind) | Delta vs ideal |
|-----------|---|--------|-------------------|----------------|
| `eskf_wind_n3_1fault` | 3 | 1 | 53.68 cm | +9.9% |
| `eskf_wind_n5_2fault` | 5 | 2 | 50.14 cm | +8.1% |
| `eskf_wind_n7_3fault` | 7 | 3 | 48.28 cm | +10.7% |

### Trajectory Generalization -> Supplementary Table (`tab:trajectory_generalization`)

Tests fault absorption across different reference trajectories.

| Directory | Trajectory | N | Faults | RMSE | Peak |
|-----------|-----------|---|--------|------|------|
| `hover_n3_1fault` | Hover (station-keeping) | 3 | 1 | 39.75 cm | 64.39 cm |
| `hover_n7_3fault` | Hover | 7 | 3 | 30.88 cm | 38.04 cm |
| `p2p_n3_1fault` | Point-to-point | 3 | 1 | 44.98 cm | 71.48 cm |
| `p2p_n7_3fault` | Point-to-point | 7 | 3 | 37.66 cm | 50.78 cm |

---

## ESO Evaluation

**Directories:** `eso_n3_nom/`, `eso_n3_fault/`, `eso_n5_nom/`, `eso_n5_fault/`,
`eso_n7_nom/`, `eso_n7_fault/` (26 MB total)

**Paper reference:** Table VII (`tab:eso_evaluation`)

These runs activate the Extended State Observer (Layer 4) to assess whether
the favorable rho_FT values are an artifact of the deliberately constrained
PID-only controller.

| Nominal dir | Fault dir | N | Nominal RMSE | Fault RMSE | rho_FT (ESO) | rho_FT (PID-only) |
|-------------|-----------|---|-------------|------------|--------------|-------------------|
| `eso_n3_nom` | `eso_n3_fault` | 3 | 28.94 cm | 40.43 cm | 1.40 | 1.39 |
| `eso_n5_nom` | `eso_n5_fault` | 5 | 30.50 cm | 44.59 cm | 1.46 | 1.33 |
| `eso_n7_nom` | `eso_n7_fault` | 7 | 33.67 cm | 46.01 cm | 1.37 | 1.26 |

Key finding: ESO reduces nominal RMSE by 17-18% but rho_FT increases
modestly. Absolute post-fault RMSE with ESO (40-46 cm) is *lower* than
PID-only (44-49 cm).

Each directory contains `run_manifest.json` and `logs/<timestamp>/` with
the standard CSV time-series data.

---

## Gain-Scheduled PID Baselines

**Directories:** `gain_sched_moderate/`, `gain_sched_aggressive/` (5.6 MB total)

**Paper reference:** Table IV (`tab:baseline_comparison`)

Standard FTC approach: each agent independently detects elevated cable tension
and increases PID gains.

| Directory | Gains | RMSE |
|-----------|-------|------|
| `gain_sched_moderate` | 1.5x k_p, 1.25x k_d | 48.96 cm |
| `gain_sched_aggressive` | Higher multipliers | variant |

---

## Monte Carlo Validation

**Directory:** `monte_carlo/` (976 MB, 120 runs across 4 families)

**Paper reference:** Table IX (`tab:monte_carlo`)

Each family runs 30 independent realizations with randomized cable lengths
(+/-5% Gaussian), wind seeds, and simulation seeds, using the same fixed
fault schedule as the deterministic canonical runs.

| Subdirectory | N | Faults | Conditions | Runs | RMSE (mean +/- std) |
|-------------|---|--------|------------|------|---------------------|
| `mc_n3_1fault/` | 3 | 1 | Ideal | 30 | 48.84 +/- 0.49 cm |
| `mc_n5_2fault/` | 5 | 2 | Ideal | 30 | 46.40 +/- 0.49 cm |
| `mc_n7_3fault/` | 7 | 3 | Ideal | 30 | 43.95 +/- 1.34 cm |
| `mc_n3_eskf_wind/` | 3 | 1 | ESKF+wind | 30 | 53.62 +/- 0.83 cm |

Each family contains:
- `run_000/` through `run_029/` -- individual run directories with
  `run_manifest.json` and `logs/<timestamp>/` (standard CSV data)
- `mc_results.json` -- per-family aggregated statistics

Top-level summary files:
- `mc_all_results.json` -- combined results across all families
- `mc_summaries.json` -- compact summary for paper table generation

---

## Broadened Monte Carlo

**Directory:** `monte_carlo_broadened/` (14 MB, 5 runs)

**Paper reference:** Section VI.D (Robustness Validation), prose paragraph

Extends the narrow Monte Carlo by additionally randomizing payload mass
(+/-10%), PID gains (k_p, k_d each +/-10%), and fault timing (+/-2 s).

| Subdirectory | Runs | RMSE (mean +/- std) | 95th pctl. peak |
|-------------|------|---------------------|-----------------|
| `broad_n3_1fault/` | 5 | 48.04 +/- 1.29 cm | 109.69 cm |

Standard deviation increases ~2.6x vs narrow MC but all runs remain stable.

Top-level: `broadened_mc_summaries.json` -- compact summary.

---

## Linearization Analysis

**Directory:** `linearize/` (20 KB)

**Paper reference:** Section IV, Proposition 1 and eigenvalue analysis

Contains the scalar per-axis linearization of the PID error dynamics
(Eq. 9-10 in the paper).

| File | Description |
|------|-------------|
| `A_scalar.csv` | 3x3 system matrix A (PID error dynamics) |
| `B_scalar.csv` | 3x1 input matrix B |
| `eigenvalues_scalar.csv` | Eigenvalues of A: -6.83, -1.15, -0.019 |
| `plant_info.json` | Model metadata, gains, ISS gains, predicted peaks |

---

## H-infinity Scalar Results

**File:** `hinf_scalar_results.json` (717 bytes)

**Paper reference:** Section IV (topology invariance, finite-time bound, small-gain)

Key quantities from the scalar transfer function G(s) = C(sI-A)^{-1}B:

| Quantity | Value | Paper location |
|----------|-------|----------------|
| `gamma_ISS` | 0.1253 | H-infinity norm, Eq. 12 |
| `omega_peak_rad_s` | 0.147 rad/s | Peak frequency of G(s) |
| `gamma_FT_0_5s` | 0.0419 | Finite-time gain for tau=0.5 s, Eq. 13 |
| `predicted_peak_cm` | 18.42 cm | Predicted transient peak, Eq. 13 |
| `max_gamma_cable_for_stability` | 7.98 | Small-gain margin: gamma_cable < 8.0 |
| `eigenvalues` | -6.83, -1.15, -0.019 | All real, negative (Hurwitz) |

---

## Revision Reports

**Directory:** `revision_reports/` (56 KB, 9 markdown files)

Documentation of the experimental methodology built up during manuscript
revision. Each file documents one step in the evidence pipeline.

| File | Topic |
|------|-------|
| `step1_cli_flags.md` | CLI flag design for full_drake_fault_runner |
| `step2_matched_experiments.md` | Matched single-fault deconfounding design |
| `step3_kappa_t_ablation.md` | Tension feedforward ablation methodology |
| `step4_fmax_boundary_sweep.md` | Thrust saturation boundary sweep design |
| `step5_wind_adapter.md` | Dryden wind model integration |
| `step6_mc_wrapper.md` | Monte Carlo orchestration wrapper |
| `step7_manuscript_restructuring.md` | Paper restructuring notes |
| `step8_figure_y_axis_consistency.md` | Figure axis standardization |
| `step9_full_sensing_stack.md` | ESKF sensor-realistic stack integration |

---

## Regeneration

All outputs can be regenerated from the source code and experiment scripts:

1. **Build the simulator:**
   ```bash
   cmake --preset drake && cmake --build build
   ```

2. **Run a single scenario:**
   ```bash
   ./build/full_drake_fault_runner --num-quads 3 --fault-schedule "0:7.0" \
     --duration 30 --output-dir outputs/my_run
   ```

3. **Run the full batch:**
   ```bash
   python experiments/python/run_fault_schedule_batch.py
   ```

4. **Run Monte Carlo:**
   ```bash
   python experiments/python/run_monte_carlo.py
   ```

Full regeneration of all outputs requires approximately 24-48 hours of
compute time depending on hardware.

---

## Recovery

All outputs are tracked in git. A safety tag `pre-cleanup-20260408` preserves
the state before the repository cleanup. Any removed file can be recovered:

```bash
git show pre-cleanup-20260408:outputs/path/to/file > restored_file
```
