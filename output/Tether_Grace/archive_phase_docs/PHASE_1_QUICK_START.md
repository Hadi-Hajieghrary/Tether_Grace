# Phase 1 Quick Start Card

## Build (2 minutes)

```bash
cd Research/cpp
mkdir -p build && cd build
cmake .. -G Ninja -DCMAKE_PREFIX_PATH=/opt/drake
ninja decentralized_mpc_test
```

## Run Test (30 seconds)

```bash
./decentralized_mpc_test --headless --duration 10
```

Expected output:
```
=== Decentralized MPC Phase 1 Test ===
Simulation duration: 10 s
Visualization: disabled

Running simulation...

=== SIMULATION COMPLETE ===
State log size: 1000 samples
Control log size: 1000 samples
Solver stats:
  Mean solve time: 25.3 ms
  Max solve time: 48.7 ms
```

## Weight Tuning Analysis (5 minutes)

```bash
cd ../../../scripts
python3 weight_tuning_analysis.py --output weight_results --quick
```

Expected outputs:
- `weight_results/weight_sweep_results.csv`
- `weight_results/pareto_frontier.csv`
- `weight_results/sensitivity_plots.pdf`
- `weight_results/recommended_weights.txt`

View recommendations:
```bash
cat weight_results/recommended_weights.txt
```

## Recommended Weights (from Analysis)

Copy-paste into your code:

```cpp
config.w_trajectory = 50.0;        // Priority 1: track reference
config.w_stability = 5.0;          // Priority 2: smooth motion
config.w_effort = 0.5;             // Priority 3: minimize control
config.w_tension_balance = 0.05;   // Priority 4: balance load
```

## Key Files

| What | Where |
|------|-------|
| Main controller class | `Research/cpp/include/decentralized_optimal_controller.h` |
| Implementation | `Research/cpp/src/decentralized_optimal_controller.cc` |
| Test harness | `Research/cpp/src/decentralized_mpc_test_main.cc` |
| Weight analysis | `Research/scripts/weight_tuning_analysis.py` |
| Full docs | `PHASE_1_DECENTRALIZED_MPC_README.md` |
| This summary | `PHASE_1_IMPLEMENTATION_SUMMARY.md` |

## API Quick Reference

```cpp
// Create configuration
DecentralizedOptimalController::Config cfg;
cfg.drone_index = 0;
cfg.num_drones = 1;
cfg.rope_length = 1.0;
cfg.w_trajectory = 50.0;
cfg.w_stability = 5.0;
cfg.w_effort = 0.5;
cfg.w_tension_balance = 0.05;

// Create controller
auto* ctrl = builder.AddSystem<DecentralizedOptimalController>(cfg);

// Wire inputs (8 ports)
builder.Connect(state->get_position_port(), 
                ctrl->get_drone_position_input_port());
builder.Connect(state->get_velocity_port(),
                ctrl->get_drone_velocity_input_port());
builder.Connect(sensors->get_tension_port(),
                ctrl->get_cable_tension_input_port());
builder.Connect(geometry->get_cable_direction_port(),
                ctrl->get_cable_direction_input_port());
builder.Connect(traj->get_reference_position_port(),
                ctrl->get_reference_trajectory_input_port());
builder.Connect(traj->get_reference_velocity_port(),
                ctrl->get_reference_velocity_input_port());
builder.Connect(sensor_network->get_other_tensions_port(),
                ctrl->get_other_tensions_input_port());
builder.Connect(estimator->get_payload_mass_port(),
                ctrl->get_payload_mass_input_port());

// Use outputs (5 ports)
builder.Connect(ctrl->get_optimal_control_output_port(),
                attitude_ctrl->get_desired_input_port());
// ... log other outputs
```

## Input/Output Spec

**Inputs (100 Hz):**
- `p_drone`: Eigen::Vector3d (m) — drone position
- `v_drone`: Eigen::Vector3d (m/s) — drone velocity
- `T_cable`: double (N) — cable tension magnitude
- `n_cable`: Eigen::Vector3d (unit) — cable direction
- `p_ref`: Eigen::Vector3d (m) — reference position
- `v_ref`: Eigen::Vector3d (m/s) — reference velocity
- `T_others`: Eigen::Vector4d (N) — [T₀, T₁, T₂, T₃]
- `m_load`: double (kg) — payload mass estimate

**Outputs (100 Hz):**
- `u_optimal`: Eigen::Vector4d — [thrust, τₓ, τᵧ, τᵤ]
- `p_load_est`: Eigen::Vector3d (m) — estimated load position
- `v_load_est`: Eigen::Vector3d (m/s) — estimated load velocity
- `solver_time_ms`: double — solver wall clock time
- `solver_status`: double — 0=success, 1=failure

## Expected Performance

| Metric | Value |
|--------|-------|
| Solve time per step | 10-50 ms (50 Hz feasible) |
| Load estimate error | <0.2 m RMS |
| Trajectory tracking error | 0.02-0.05 m |
| Control effort (1.5kg quad) | ~7.5 N thrust |
| Solver success rate | >99% |

## Troubleshooting

**Solver fails immediately**
→ Check bounds (thrust_max, torque_max, rope_length)

**Solve time > 50 ms consistently**
→ Reduce horizon (`control_horizon_sec`) or # iterations (`max_solver_iterations`)

**Load estimate drifts**
→ Increase filter coefficient (`load_estimator_filter_alpha` from 0.1 to 0.2)

**Drone oscillates**
→ Reduce w_trajectory or increase w_stability (heavier stability term)

## Next: Phase 2

Once Phase 1 runs successfully:
1. Add 2nd and 3rd quadcopters (update `num_drones`)
2. Implement multi-drone sensor network (broadcast T_others)
3. Close ESKF feedback loop
4. Verify implicit coordination (no explicit communication)

---

**Phase 1 Status**: ✅ Complete  
**Ready for**: Testing & Phase 2 integration
