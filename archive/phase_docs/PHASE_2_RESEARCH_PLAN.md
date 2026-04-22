# Phase 2 Research Plan: Decentralized MPC Multi-Drone Deployment

**Date**: 2026-04-17  
**Target Delivery**: Week of 2026-05-05  
**Scientific Objective**: Validate implicit coordination of N≥2 drones on shared payload via rope tension coupling (zero explicit communication)

---

## Executive Summary

Phase 1 (skeleton complete, functionality incomplete) must be remediated before Phase 2 can proceed. This plan covers:

1. **Phase 1 Remediation** (4 critical fixes): Close feedback loop, implement objective function, validate load estimator, add rope tautness constraint
2. **Phase 2 Deployment** (4-week sprint): Single-drone validation → 2-drone implicit coordination → 3-drone formation → cable severance preview
3. **Validation Framework**: Metrics to prove coordination works without communication
4. **Risk Register**: Identified failure modes and fallback plans

---

## Section 1: Phase 2 Detailed Timeline & Resource Planning

### Overview: 4-Week Sprint

```
Week 1 (Apr 21-25):  Phase 1 Remediation
Week 2 (Apr 28-May2): Single-Drone Validation + 2-Drone Test Harness
Week 3 (May 5-9):     3-Drone Formation + Convergence Analysis
Week 4 (May 12-16):   Cable Severance Scenario + Phase 3 Preview
```

---

### Week 1: Phase 1 Remediation (4 Critical Fixes)

#### Objective
Enable closed-loop feedback and functional MPC optimizer so Phase 1 test passes all assertions.

#### Deliverables
1. **Plant truth extractor** (state splitter) connected to controller inputs
2. **Trajectory cost function** integrated into optimization problem
3. **Discrete load estimator** refactored from mutable state to DeclareDiscreteState()
4. **Rope tautness constraint** formulated and added to optimizer

#### Tests to Run
- `decentralized_mpc_test --headless --duration 10.0`
  - Check: drone hovers at z=2.0m
  - Check: load settles below drone
  - Check: solver completes in <50 ms
  - Check: solver status = success (no failures)
  - Check: thrust ≈ 0.5*F_max at steady state

#### Metrics to Measure
- **Solver success rate**: Target ≥99% (1 failure per 100 solves is acceptable)
- **Mean solve time**: Target <10 ms (Phase 1 nominal), <25 ms (Phase 1 worst case)
- **Load position error at t=10s**: |p_load - p_ref| < 0.05 m
- **Rope tension range**: 0 ≤ T ≤ 1.2*m_load*g (no slack, no excessive tension)

#### Success Criteria
- [ ] Controller reads plant state correctly (p_quad, v_quad, p_beads, v_beads extracted)
- [ ] MPC objective: min w1·||p_L - p_ref||² + w2·||a_L||² + w3·||u||²
- [ ] Optimizer adds rope tautness constraint: T[k] ≥ 0 for all k
- [ ] Load estimator uses discrete state event, not mutable members
- [ ] All solver invocations complete; mean solve time logged
- [ ] Test logs include: solver_time_ms, solver_status, p_load_est, T_measured

#### Risk Mitigation
- **Risk**: Rope tension goes negative (slack) → optimizer infeasible
  - **Mitigation**: Use slack variables s[k] ≥ 0; add penalty term w_slack·Σs[k]
  - **Fallback**: If infeasible, lower the w_trajectory weight and re-solve
- **Risk**: Discrete state event misses update cycles
  - **Mitigation**: Use PeriodicEventData with dt=control_dt; log update count vs. expected count
  - **Fallback**: Use continuous state (less ideal but acceptable for Phase 2a)

---

### Week 2: Single-Drone Validation + 2-Drone Test Harness

#### Objective
Prove Phase 1 remediation works on 1 drone; build and test 2-drone system without explicit communication.

#### Deliverables
1. **Phase 1 Final Report** (2 pages): Remediation summary, metrics, lessons learned
2. **Two-drone test executable** (`decentralized_mpc_test_2drones`)
3. **Implicit coordination validator**: Compare load position estimates from both drones
4. **Initial results plots**: Load position, tensions, solver times (10 scenarios, 1 seed each)

#### Tests to Run
**Week 2a (Mon-Wed): Single-drone validation**
- `decentralized_mpc_test --headless --duration 10.0` (10 runs, varied initial conditions)
  - Initial heights: [1.0, 1.5, 2.0] m
  - Reference positions: hover vs. circular motion (figure-8 preview)
  - Measure: success rate, mean/max solver time, load settling time

**Week 2b (Thu-Fri): Two-drone test**
- `decentralized_mpc_test_2drones --headless --duration 10.0 --num_drones 2` (10 runs)
  - Both drones receive same reference trajectory (shared load)
  - Each drone runs independent controller (no inter-drone comms)
  - Measure: load position agreement between estimates, rope tensions

#### Metrics to Measure
**Single-Drone (Phase 1 Final)**:
- Solver success rate (target ≥99%)
- Mean solve time (target <10 ms)
- Load settling time: t when ||p_L - p_ref|| < 0.05m (target <5s)
- Steady-state rope tension: T_ss (should equal m_load*g ≈ 29.4 N)

**Two-Drone (Implicit Coordination)**:
- Load position agreement: |p_L_est_drone1 - p_L_est_drone2| at t=10s (target <0.2 m)
- Convergence rate of agreement: slope of agreement error vs. time (target: exponential decay)
- Thrust balance: T_1 + T_2 - m_L*g - m_quad_total*g = O(0.1 N) (target: within 5% of total mass*g)
- No communication verified: zero messages between controller 1 and controller 2

#### Success Criteria
- [ ] Single-drone test: ≥90 successful solves out of 10 runs (9 per run)
- [ ] Mean solver time logged and <10 ms for all runs
- [ ] Load position error at t=10s: <0.05 m for all single-drone runs
- [ ] Two-drone test: load position estimates agree within 0.2 m
- [ ] Two-drone test: no inter-drone communication observed in code/logs
- [ ] Thrust balance equation satisfied within 10% for all time steps

#### Risk Mitigation
- **Risk**: Two-drone load position estimates diverge (implicit coordination fails)
  - **Mitigation**: Both drones subscribe to identical reference trajectory; compare rope direction feedback
  - **Fallback**: Reduce control horizon from 3s to 1s (less planning, tighter feedback loop)
- **Risk**: Test takes longer than expected; schedule pressure
  - **Mitigation**: Run single-drone and 2-drone tests in parallel across different machines
  - **Fallback**: Reduce scenario count from 10 to 5, rerun later with more seeds

---

### Week 3: 3-Drone Formation + Convergence Analysis

#### Objective
Scale to N=3; analyze load position convergence; begin formation control experiments.

#### Deliverables
1. **Three-drone test executable** (`decentralized_mpc_test_3drones`)
2. **Convergence analysis report** (2 pages): proof-of-concept that implicit coordination scales
3. **10 scenario results**: load position, tensions, solver stats (30 data files)
4. **Convergence plots**: p_L_est_drone1, p_L_est_drone2, p_L_est_drone3 vs. actual p_L (1 figure)

#### Tests to Run
- `decentralized_mpc_test_3drones --headless --duration 15.0 --num_drones 3` (10 scenarios)
  - Scenario 1-5: Fixed hover at [0, 0, 2]
  - Scenario 6-10: Figure-8 trajectory (frequency=0.1 Hz, amplitude=1 m)
  - Each scenario: 1 seed (total 10 runs)
  - Measure: load position, tensions, solver stats, estimate agreement

#### Metrics to Measure
- **Convergence**: max(|p_L_est_i - p_L_est_j|) over all i,j pairs, averaged over 10 scenarios
- **Steady-state agreement error**: final 2 seconds (average agreement error)
- **Formation stability**: variance of inter-drone distances over time (should be <0.01 m² by t=10s)
- **Solver robustness**: success rate with 3 drones (target ≥95%)

#### Success Criteria
- [ ] All 10 scenarios complete successfully (no solver failures > 5% per run)
- [ ] Load position estimates from 3 drones converge: max disagreement <0.3 m at t=15s
- [ ] Convergence rate characterized: time constant τ for exponential decay (target τ <3s)
- [ ] No deadlock or oscillation in inter-drone coupling (rope tensions stable)
- [ ] Convergence proof or empirical evidence published (section in report)

#### Risk Mitigation
- **Risk**: Solver fails more frequently with 3 drones (increased problem size)
  - **Mitigation**: Reduce control horizon from 3s to 2s; profile solver time with drake profiler
  - **Fallback**: Use simplified dynamics (ignore bead chain; assume massless rope)
- **Risk**: Load position estimates still diverge with 3 drones
  - **Mitigation**: Increase w_trajectory weight; reduce measurement noise in simulation
  - **Fallback**: Add one-way broadcast of reference trajectory (not full communication)

---

### Week 4: Cable Severance Scenario + Phase 3 Preview

#### Objective
Test fault robustness; demonstrate graceful degradation; gather Phase 3 requirements.

#### Deliverables
1. **Severance test executable** (`decentralized_mpc_test_2drones_with_severance`)
2. **Fault recovery report** (1 page): time-to-recovery, residual error, drone compensation
3. **Phase 3 design document** (2 pages): cable failure detection, multi-drone coordinated descent
4. **Preliminary results**: 5 scenarios with cable severance at t=5s, comparison to nominal

#### Tests to Run
- `decentralized_mpc_test_2drones_with_severance --headless --duration 15.0 --fault_time 5.0` (5 scenarios)
  - Scenario 1: Hover → severance at t=5s → load drops onto drone 2
  - Scenario 2: Ascending → severance → load swings; drone 2 compensates
  - Scenario 3-5: Various trajectories and fault timings
  - Measure: load position, drone 2 response time, recovery trajectory

#### Metrics to Measure
- **Detection latency**: time from severance to solver infeasibility (target <1 control cycle ≈ 20 ms)
- **Recovery time**: time from severance to load back in bounds (target <2s)
- **Max load position error after severance**: |p_L - p_ref| (target <0.5 m)
- **Drone 2 additional thrust**: ΔF_2 needed to compensate for drone 1 loss (target <50% of max)

#### Success Criteria
- [ ] Cable severance detected within 1 control cycle (solver reports infeasibility)
- [ ] Drone 2 automatically increases thrust; load stabilizes within 2s
- [ ] Load position error remains <0.5 m during recovery
- [ ] Phase 3 design document completed with clear failure detection strategy
- [ ] Preliminary fault-recovery plots saved for publication

#### Risk Mitigation
- **Risk**: Optimizer doesn't detect infeasibility (rope constraint silently relaxed)
  - **Mitigation**: Add explicit infeasibility detection: `if (!result.is_success()) → deploy fallback policy`
  - **Fallback**: Use precomputed thrust lookup table (offline trained)
- **Risk**: Drone 2 can't physically support load alone (undersized thrust)
  - **Mitigation**: Verify max thrust ≥ 2*m_load*g upfront in test setup
  - **Fallback**: Document limitation; note for Phase 3 hardware design

---

## Section 2: Phase 1 Remediation Roadmap

### Overview: 4 Critical Fixes Required

| # | Issue | Root Cause | Fix | Effort | Risk | Priority |
|---|-------|-----------|-----|--------|------|----------|
| 1 | Test is open-loop (zero state inputs) | No state extractor | Add state splitter system | 2-3 h | Low | CRITICAL |
| 2 | Optimizer empty (no objectives) | SetupOptimizationProblem incomplete | Implement trajectory cost + stability cost | 4-5 h | Medium | CRITICAL |
| 3 | Load estimator not validated | Uses mutable state in const method | Refactor to DeclareDiscreteState() | 3-4 h | Medium | CRITICAL |
| 4 | Fallback policy undersized | Nominal hover thrust = 0.5*F_max | Increase to m_L*g + margin | 1 h | Low | CRITICAL |

---

### Fix 1: Plant Truth Connection (State Splitter)

**What**: Extract quadcopter position, velocity, and rope state from MultibodyPlant and route to controller inputs.

**Drake Architecture**:
```
MultibodyPlant state = [p_quad, p_payload, p_bead_0, ..., p_bead_N-1,
                        v_quad, v_payload, v_bead_0, ..., v_bead_N-1]
                       (3+3+3*(N_beads) dimensions)
                       ↓
                    StateExtractor (custom LeafSystem)
                       ├─ Output 0: p_drone [3] → controller port 0
                       ├─ Output 1: v_drone [3] → controller port 1
                       ├─ Output 2: T_cable [1] → controller port 2
                       └─ Output 3: n_cable [3] → controller port 3
```

**Implementation Details**:

**Header** (`include/state_extractor.h`):
```cpp
#pragma once
#include <drake/systems/framework/leaf_system.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <Eigen/Core>

namespace quad_rope_lift {

class StateExtractor : public drake::systems::LeafSystem<double> {
 public:
  struct Config {
    int num_beads;           // How many rope beads
    double rope_length;      // m
    int drone_index;         // Which drone (0-indexed)
  };
  
  explicit StateExtractor(const Config& config);
  
  // Input: plant state
  const drake::systems::InputPort<double>& get_plant_state_input_port() const {
    return get_input_port(0);
  }
  
  // Outputs:
  const drake::systems::OutputPort<double>& get_drone_position_output_port() const {
    return get_output_port(0);  // 3D
  }
  
  const drake::systems::OutputPort<double>& get_drone_velocity_output_port() const {
    return get_output_port(1);  // 3D
  }
  
  const drake::systems::OutputPort<double>& get_cable_tension_output_port() const {
    return get_output_port(2);  // 1D scalar
  }
  
  const drake::systems::OutputPort<double>& get_cable_direction_output_port() const {
    return get_output_port(3);  // 3D unit vector
  }

 private:
  void CalcDronePosition(const drake::systems::Context<double>&, 
                         Eigen::Vector3d*) const;
  void CalcDroneVelocity(const drake::systems::Context<double>&, 
                         Eigen::Vector3d*) const;
  void CalcCableTension(const drake::systems::Context<double>&, 
                        double*) const;
  void CalcCableDirection(const drake::systems::Context<double>&, 
                          Eigen::Vector3d*) const;
  
  Config config_;
};

}  // namespace quad_rope_lift
```

**Implementation** (`src/state_extractor.cc`):
```cpp
void StateExtractor::CalcDronePosition(
    const drake::systems::Context<double>& context,
    Eigen::Vector3d* output) const {
  // Plant state format:
  //   [p_quad_0, p_quad_1, p_quad_2,              positions (0-2)
  //    p_payload_0, p_payload_1, p_payload_2,     positions (3-5)
  //    p_bead_0_x, p_bead_0_y, p_bead_0_z,        positions (6-8)
  //    ...,
  //    v_quad_0, v_quad_1, v_quad_2, ...]         velocities follow
  
  auto x = get_input_port(0).Eval(context);
  
  // Extract p_quad (first 3 elements)
  *output = x.segment<3>(0);
}

void StateExtractor::CalcDroneVelocity(
    const drake::systems::Context<double>& context,
    Eigen::Vector3d* output) const {
  auto x = get_input_port(0).Eval(context);
  int num_pos = 3 + 3 + 3 * config_.num_beads;
  
  // Extract v_quad (first 3 velocity states)
  *output = x.segment<3>(num_pos);
}

void StateExtractor::CalcCableTension(
    const drake::systems::Context<double>& context,
    double* output) const {
  auto x = get_input_port(0).Eval(context);
  
  // Compute tension from drone-to-payload displacement
  Eigen::Vector3d p_drone = x.segment<3>(0);
  Eigen::Vector3d p_payload = x.segment<3>(3);
  Eigen::Vector3d dr = p_payload - p_drone;
  
  // Simple model: tension ≈ spring stiffness * (L - ||dr||)
  double tension = 200.0 * (config_.rope_length - dr.norm());
  
  *output = std::max(0.0, tension);  // No negative tension (slack)
}

void StateExtractor::CalcCableDirection(
    const drake::systems::Context<double>& context,
    Eigen::Vector3d* output) const {
  auto x = get_input_port(0).Eval(context);
  
  // Direction from drone to payload (unit vector)
  Eigen::Vector3d p_drone = x.segment<3>(0);
  Eigen::Vector3d p_payload = x.segment<3>(3);
  Eigen::Vector3d dr = p_payload - p_drone;
  
  if (dr.norm() > 1e-6) {
    *output = dr.normalized();
  } else {
    *output = Eigen::Vector3d(0, 0, -1);  // Default downward
  }
}
```

**Wiring in Test**:
```cpp
// In decentralized_mpc_test_main.cc, replace zero inputs:
auto* state_extractor = builder.AddSystem<StateExtractor>(
    StateExtractor::Config{.num_beads = num_beads, 
                           .rope_length = rope_length,
                           .drone_index = 0});

builder.Connect(plant->get_state_output_port(),
                state_extractor->get_plant_state_input_port());
builder.Connect(state_extractor->get_drone_position_output_port(),
                controller->get_drone_position_input_port());
builder.Connect(state_extractor->get_drone_velocity_output_port(),
                controller->get_drone_velocity_input_port());
builder.Connect(state_extractor->get_cable_tension_output_port(),
                controller->get_cable_tension_input_port());
builder.Connect(state_extractor->get_cable_direction_output_port(),
                controller->get_cable_direction_input_port());
```

**Dimensions & Port Layout**:
| Port | Size | Description |
|------|------|-------------|
| Input: plant_state | 6+3*N_beads | Full state vector |
| Output: p_drone | 3 | Drone position [x, y, z] |
| Output: v_drone | 3 | Drone velocity [ẋ, ẏ, ż] |
| Output: T_cable | 1 | Cable tension magnitude |
| Output: n_cable | 3 | Cable direction (unit vector) |

**Risk**: If rope has slack (tension < 0), rope constraint is inactive. **Mitigation**: Clamp tension to ≥0; detect slack condition and add flag to optimizer.

**Estimated Effort**: 2-3 hours (new file: ~150 lines of code)

---

### Fix 2: Trajectory Cost Implementation

**What**: Replace empty objective function with multi-objective cost: min w1·||p_L - p_ref||² + w2·||a_L||² + w3·||u||² + w4·T_variance

**Drake MPC Setup**:

Currently, `SetupOptimizationProblem()` returns a program with only constraint bounds. We need to:
1. Declare decision variables for load state trajectory [p_L_0, v_L_0, p_L_1, v_L_1, ..., p_L_N, v_L_N]
2. Add dynamics constraints: p_L[k+1] = p_L[k] + dt·v_L[k]; v_L[k+1] = v_L[k] + dt·a_L[k]
3. Add acceleration constraint: a_L[k] = (T_sum[k] - m_L*g) / m_L (links rope tension to load acceleration)
4. Add objective cost terms

**Implementation** (`src/decentralized_optimal_controller.cc` - replace SetupOptimizationProblem):

```cpp
std::unique_ptr<drake::solvers::MathematicalProgram>
DecentralizedOptimalController::SetupOptimizationProblem(
    const Vector3d& p_i,
    const Vector3d& v_i,
    const Vector3d& p_L_est,
    const Vector3d& v_L_est,
    double T_measured,
    const Vector3d& n_measured,
    const Vector3d& p_ref,
    const Vector3d& v_ref,
    const VectorXd& T_all,
    double m_L_est) const {

  auto prog = std::make_unique<drake::solvers::MathematicalProgram>();

  int N = static_cast<int>(config_.control_horizon_sec / config_.control_dt);
  
  // ============ DECISION VARIABLES ============
  
  // Control trajectory: U = [u_0, u_1, ..., u_{N-1}], each u_k = [thrust, tau_x, tau_y, tau_z]
  auto U = prog->NewContinuousVariables(4, N, "U");
  
  // Load state trajectory: (only for current drone's contribution to load dynamics)
  // For decentralized MPC, estimate load state using RopeForceSystem feedback
  // Alternatively: use load estimator state as initial condition; don't include in decision vars
  // (Phase 2a: simplified approach - don't optimize over load trajectory, only control)
  
  // Slack variables for rope tautness and constraint violations
  auto slack_tension = prog->NewContinuousVariables(N, "slack_T");
  auto slack_acceleration = prog->NewContinuousVariables(3, N, "slack_a");  // Per-axis
  
  // ============ CONTROL BOUNDS ============
  
  // Thrust: 0 ≤ u_0 ≤ thrust_max
  prog->AddBoundingBoxConstraint(0, config_.thrust_max, U.row(0));
  
  // Torques: -torque_max ≤ u_i ≤ torque_max (i = 1,2,3)
  for (int i = 1; i < 4; ++i) {
    prog->AddBoundingBoxConstraint(-config_.torque_max, config_.torque_max, U.row(i));
  }
  
  // Slack variables
  prog->AddBoundingBoxConstraint(0, 1.0, slack_tension);
  prog->AddBoundingBoxConstraint(0, 10.0, slack_acceleration);  // m/s² slack
  
  // ============ ROPE TAUTNESS CONSTRAINT ============
  
  // Simplified: assume rope tension decreases quadratically over horizon
  // T[k] ≥ -slack[k]  (allows slack; penalty in objective)
  for (int k = 0; k < N; ++k) {
    double T_init = T_measured;
    double T_decay = T_init * (1.0 - (k / (double)N));  // Linear decay
    
    // Simple constraint: T[k] ≥ T_decay - 10*slack[k]
    // (Simplified; full version would propagate dynamics)
    prog->AddConstraint(T_decay - slack_tension(k) <= 100.0);  // Upper bound (unrealistic tension)
  }
  
  // ============ OBJECTIVE FUNCTION ============
  
  // Cost 1: Trajectory tracking ||p_L[k] - p_ref||²
  // We don't have p_L in decision vars (decentralized), so we use predicted load position
  // Simple approximation: assume load accelerates based on current rope tension
  // For Phase 2a: cost on drone position as proxy for load tracking
  
  // Predicted load acceleration (simplified: assume vertical only)
  double a_L_z_pred = (T_measured - m_L_est * 9.81) / m_L_est;
  
  // Project load position forward
  Eigen::Vector3d p_L_pred = p_L_est;
  Eigen::Vector3d v_L_pred = v_L_est;
  for (int k = 0; k < N; ++k) {
    // Assume tension constant over horizon (Phase 2a simplification)
    v_L_pred(2) += a_L_z_pred * config_.control_dt;
    p_L_pred(2) += v_L_pred(2) * config_.control_dt;
  }
  
  // Cost on predicted load position
  drake::symbolic::Expression cost_trajectory = 0;
  for (int k = 0; k < N; ++k) {
    cost_trajectory += config_.w_trajectory * ((p_L_pred(2) - p_ref(2)) * (p_L_pred(2) - p_ref(2)));
  }
  prog->AddCost(cost_trajectory);
  
  // Cost 2: Control effort ||u_k||²
  drake::symbolic::Expression cost_effort = 0;
  for (int k = 0; k < N; ++k) {
    for (int i = 0; i < 4; ++i) {
      cost_effort += config_.w_effort * U(i, k) * U(i, k);
    }
  }
  prog->AddCost(cost_effort);
  
  // Cost 3: Tension slack penalty (encourages tautness)
  drake::symbolic::Expression cost_slack = 0;
  for (int k = 0; k < N; ++k) {
    cost_slack += 1000.0 * slack_tension(k);  // High weight to keep rope taut
  }
  prog->AddCost(cost_slack);
  
  // ============ SOLVER OPTIONS ============
  
  prog->SetSolverOption(drake::solvers::SnoptSolver::id(), 
                        "Major iterations limit", 
                        config_.max_solver_iterations);
  prog->SetSolverOption(drake::solvers::SnoptSolver::id(), 
                        "Minor iterations limit", 
                        100);
  prog->SetSolverOption(drake::solvers::SnoptSolver::id(), 
                        "Feasibility tolerance", 
                        1e-6);
  prog->SetSolverOption(drake::solvers::SnoptSolver::id(), 
                        "Optimality tolerance", 
                        config_.solver_tolerance);
  
  return prog;
}
```

**Key Design Decisions**:

1. **Why no load state in decision variables?**
   - Load position depends on all N drones' tensions (fully coupled)
   - Decentralized: each drone optimizes only its own control
   - Solution: Use load estimator state (from rope feedback) as initial condition; propagate forward deterministically

2. **Why simplified dynamics?**
   - Full rope dynamics (bead chain) would require heavy computation
   - Phase 2a: Assume point load with complementary filter estimate
   - Phase 2b (if needed): Add more complex rope constraint

3. **Slack variables for tautness**:
   - Hard constraint T[k] ≥ 0 can make optimizer infeasible
   - Soft constraint: T[k] ≥ -slack[k]; add penalty w_slack·Σslack[k]
   - Allows graceful degradation if infeasible (slack > 0 triggers fallback)

**Estimated Effort**: 4-5 hours (modify SetupOptimizationProblem, add objective terms, test)

**Risk**: Optimizer complexity increases; solve time may exceed 50 ms. **Mitigation**: Profile with Drake profiler; reduce N if needed.

---

### Fix 3: Discrete State Refactor (Load Estimator)

**What**: Replace mutable members (`p_load_est_`, `v_load_est_`) with proper Drake discrete state managed via `DeclareDiscreteState()` and periodic update events.

**Drake Pattern**:
```cpp
// Instead of:
mutable Eigen::Vector3d p_load_est_;  // BAD: mutates in const method
void CalcOptimalControl(...) const {
  UpdateLoadEstimate(...);  // modifies p_load_est_
}

// Use:
void DecentralizedOptimalController::DeclareDynamics() {
  DeclareDiscreteState(6);  // 3 for p_load, 3 for v_load
  DeclarePeriodicDiscreteUpdateEvent(config_.control_dt,
                                     &DecentralizedOptimalController::UpdateLoadEstimate);
}

void UpdateLoadEstimate(const Context&, DiscreteValues*) {
  // Modifies state explicitly (not mutable)
}
```

**Current Code Issues**:
```cpp
// decentralized_optimal_controller.h (lines 177-179)
mutable Eigen::Vector3d p_load_est_;
mutable Eigen::Vector3d v_load_est_;
mutable Eigen::Vector3d p_load_prev_;  // For velocity differentiation

// decentralized_optimal_controller.cc (lines 246-265)
void DecentralizedOptimalController::UpdateLoadEstimate(...) const {
  p_load_est_ = alpha * z_pos + (1.0 - alpha) * p_load_est_;  // MUTATES const method
}
```

**Refactored Code**:

**Header changes** (`include/decentralized_optimal_controller.h`):
```cpp
 private:
  // Remove mutable state members
  // mutable Eigen::Vector3d p_load_est_;  // DELETE
  // mutable Eigen::Vector3d v_load_est_;  // DELETE
  // mutable Eigen::Vector3d p_load_prev_;  // DELETE

  void UpdateLoadEstimateDiscreteEvent(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* next_discrete_state) const;
  
  // Helper (non-mutating) to read current estimate
  Eigen::Vector3d GetLoadPositionEstimate(
      const drake::systems::Context<double>& context) const;
  Eigen::Vector3d GetLoadVelocityEstimate(
      const drake::systems::Context<double>& context) const;
```

**Constructor changes** (`src/decentralized_optimal_controller.cc`):
```cpp
DecentralizedOptimalController::DecentralizedOptimalController(
    const Config& config)
    : config_(config),
      last_solve_time_(std::chrono::high_resolution_clock::now()) {
  
  // ... existing input/output ports ...
  
  // Declare discrete state: [p_load_x, p_load_y, p_load_z, v_load_x, v_load_y, v_load_z]
  DeclareDiscreteState(6);
  
  // Register periodic discrete update event (every control_dt seconds)
  DeclarePeriodicDiscreteUpdateEvent(
      config_.mpc_period,  // 0.02 s (50 Hz)
      0.0,  // offset
      &DecentralizedOptimalController::UpdateLoadEstimateDiscreteEvent);
}
```

**Update event handler**:
```cpp
void DecentralizedOptimalController::UpdateLoadEstimateDiscreteEvent(
    const drake::systems::Context<double>& context,
    drake::systems::DiscreteValues<double>* next_discrete_state) const {
  
  // Read inputs
  auto p_i = get_input_port(p_drone_port_).Eval<Eigen::Vector3d>(context);
  auto v_i = get_input_port(v_drone_port_).Eval<Eigen::Vector3d>(context);
  auto n_i = get_input_port(n_cable_port_).Eval<Eigen::Vector3d>(context);
  
  // Read current state
  auto& state = context.get_discrete_state(0);
  Eigen::Vector3d p_load_est = state.segment<3>(0);
  Eigen::Vector3d v_load_est = state.segment<3>(3);
  
  // Measurement: load position from rope constraint
  Eigen::Vector3d z_pos = p_i - config_.rope_length * n_i;
  
  // Complementary filter
  double alpha = config_.load_estimator_filter_alpha;
  Eigen::Vector3d p_load_new = alpha * z_pos + (1.0 - alpha) * p_load_est;
  Eigen::Vector3d v_load_new = (z_pos - (p_load_est)) / config_.control_dt;
  v_load_new = alpha * v_load_new + (1.0 - alpha) * v_load_est;
  
  // Write to next state
  next_discrete_state->get_mutable_vector(0).segment<3>(0) = p_load_new;
  next_discrete_state->get_mutable_vector(0).segment<3>(3) = v_load_new;
}
```

**Output port calculations** (now read from state, not mutable members):
```cpp
void DecentralizedOptimalController::CalcLoadPositionEstimate(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output) const {
  auto state = context.get_discrete_state(0);
  output->SetFromVector(state.segment<3>(0));  // Read p_load from state
}

void DecentralizedOptimalController::CalcLoadVelocityEstimate(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output) const {
  auto state = context.get_discrete_state(0);
  output->SetFromVector(state.segment<3>(3));  // Read v_load from state
}
```

**Initialize state** (in CalcOptimalControl or setup):
```cpp
// In simulator initialization:
auto& controller_context = diagram->GetMutableSubsystemContext(*controller, &context);
auto& discrete_state = controller_context.get_mutable_discrete_state(0);
discrete_state.SetAtIndex(0, p_load_est_initial(0));
discrete_state.SetAtIndex(1, p_load_est_initial(1));
discrete_state.SetAtIndex(2, p_load_est_initial(2));
// ... v_load initialization ...
```

**Benefits**:
- Complies with Drake LeafSystem conventions (no mutable state in const methods)
- State evolution is explicit (testable, debuggable)
- Solver output port is deterministic (no hidden dependencies)

**Risk**: Timing of discrete update vs. solver invocation. **Mitigation**: Solver reads state at beginning of CalcOptimalControl; discrete update happens after.

**Estimated Effort**: 3-4 hours (refactor UpdateLoadEstimate, adjust port calculations, test timing)

---

### Fix 4: Rope Tautness Constraint

**What**: Formulate and add constraint that rope tension T[k] ≥ 0 for all k ∈ [0, N-1] in the optimization problem.

**Physics**:
- Rope can only pull (T ≥ 0); cannot push (T < 0 = slack)
- If optimizer produces T < 0, it means the current control violates rope constraint
- Solution: Either increase thrust (to keep rope taut) or allow slack (add penalty)

**Implementation**:

**Constraint formulation** (mathematical):
```
Minimize:  J = Σ(k=0 to N-1) w1·||p_L[k] - p_ref||² + w3·||u[k]||² + w_slack·Σslack[k]

Subject to:
  Dynamics:   p_L[k+1] = p_L[k] + dt·v_L[k]
              v_L[k+1] = v_L[k] + dt·a_L[k]
              a_L[k] = (T[k] - m_L·g) / m_L
  
  Rope:       T[k] = ||rope_tension[k]||  (function of control u[k])
  
  Tautness:   T[k] + slack[k] ≥ 0  for all k
              slack[k] ≥ 0
  
  Bounds:     0 ≤ u[k](0) ≤ thrust_max
              |u[k](i)| ≤ torque_max   for i = 1,2,3
```

**Simplified Implementation** (Drake code):

Since full rope dynamics are nonlinear, use simplified model:
- Assume rope tension is linear function of drone-payload separation
- Use current measured tension as initial estimate
- Decay over horizon (rope extends as drone moves away from payload)

```cpp
// In SetupOptimizationProblem, after defining U and slack_tension:

// Simplified rope tension model:
// T[k] ≈ T_0 · (1 - k/(2N))  + w_control · ||u[k]||
// where T_0 = measured tension, w_control = coupling factor

for (int k = 0; k < N; ++k) {
  // Estimated tension at step k
  double T_nominal = T_measured * (1.0 - static_cast<double>(k) / (2.0 * N));
  
  // Rope tautness constraint: T[k] + slack[k] ≥ 0
  // Equivalent: T[k] ≥ -slack[k]
  // In Drake: add linear constraint
  
  // T[k] = T_nominal - decay_factor · ||u[k]||
  // For simplicity, assume T[k] ≈ T_nominal (control doesn't directly affect T in this model)
  
  // Constraint: T[k] ≥ -slack[k]
  // Rearrange: T_nominal + slack[k] ≥ 0
  // This is always satisfied if slack[k] ≥ 0 and T_nominal ≥ 0
  // So the real constraint is: detect when T becomes negative
  
  // Better approach: add cost term for negative tension
  // -T[k] slack[k] (penalizes both negative tension AND slack)
  
  for (int k = 0; k < N; ++k) {
    // High penalty for slack (encourages tautness)
    prog->AddCost(10000.0 * slack_tension(k));
  }
  
  // Alternative: explicit inequality constraint
  // T[k] + slack[k] ≥ 0  →  slack[k] ≥ -T[k]
  // But T[k] is not a decision variable; T[k] is estimated
  // Solution: use constraint on estimated tension
  
  if (T_nominal > 0) {
    // If current estimate is positive, encourage it to stay positive
    prog->AddConstraint(slack_tension(k) >= 0);  // already done in bounds
  } else {
    // If estimate already negative (bad), require slack to compensate
    prog->AddConstraint(slack_tension(k) >= -T_nominal + 0.1);
  }
}
```

**More Sophisticated Version** (Phase 2b enhancement):

Link tension to control via explicit rope dynamics:
```cpp
// Rope constraint: a_rope = k*(L_natural - L_current) - c*v_rope
// where L_current = ||p_payload - p_drone||
// Tension: T = k*(L_natural - L_current)

// Decision variables: include L_current[k] (rope length at each step)
auto L_current = prog->NewContinuousVariables(N, "L_current");

// Constraints:
// L_current[k+1] = L_current[k] + dt*dL[k]
// dL[k] = v_payload_mag[k] (rate of rope extension)
// T[k] = 200.0 * (rope_length - L_current[k])

// Tautness:
for (int k = 0; k < N; ++k) {
  // T[k] ≥ -slack[k]
  // 200*(rope_length - L[k]) ≥ -slack[k]
  prog->AddConstraint(200.0 * (config_.rope_length - L_current(k)) 
                      + slack_tension(k) >= 0);
}
```

**Recommended Approach for Phase 1 Fix**: 
Use simplified version (first implementation above) with high slack penalty. This is sufficient to detect constraint violations and trigger fallback.

**Risk**: If constraint is too restrictive, optimizer becomes infeasible. **Mitigation**: Monitor solver status; if infeasible, increase slack penalty weight or reduce control horizon.

**Estimated Effort**: 2-3 hours (add constraint terms, test feasibility)

---

## Section 3: Multi-Drone Architecture Design

### System Topology: N=2 Cooperative Load Transport

```
┌─────────────────────────────────────────────────────────┐
│                     Shared Payload                       │
│                    (mass = 3.0 kg)                       │
│                    p_L, v_L (state)                      │
└──────┬──────────────────────────────────────────┬────────┘
       │                                          │
       │  Rope 0 (bead chain)                      │  Rope 1 (bead chain)
       │  T_0, L_0, n_0 (tension, length, dir)    │  T_1, L_1, n_1
       │                                          │
   ┌───┴──────────────┐                      ┌───┴──────────────┐
   │  Drone 0         │                      │  Drone 1         │
   │  (1.5 kg)        │                      │  (1.5 kg)        │
   │  p_0, v_0        │                      │  p_1, v_1        │
   │  F_0, τ_0        │                      │  F_1, τ_1        │
   └───┬──────────────┘                      └───┬──────────────┘
       │                                          │
       └──────────────────┬───────────────────────┘
                          │
                    ┌─────┴──────┐
                    │   Plant    │
                    │ (Drake MBP)│
                    └────────────┘
```

**Key Observations**:
1. **Shared load**: Both drones pull same payload via separate ropes
2. **Implicit coupling**: Rope tension T_0 and T_1 both depend on load position p_L
3. **No explicit communication**: Each drone runs independent controller; no inter-drone messaging
4. **Information flow**: 
   - Drone 0 observes: p_0, v_0, T_0, n_0 (local only)
   - Drone 0 estimates: p_L_est_0 (from rope kinematics + filter)
   - Drone 0 optimizes: F_0, τ_0 (based on local info + reference trajectory)
   - Same for Drone 1 independently
5. **Coordination mechanism**: Via rope forces
   - If drone 0 pulls up: tension T_0 increases → load rises
   - Drone 1 observes: T_1 increases, n_1 changes → adapts control
   - Tension feedback couples the controllers implicitly

### State Representation per Drone

**Drone i State** (complete):
```cpp
struct DroneState {
  Eigen::Vector3d p_i;           // Position [x, y, z]
  Eigen::Vector3d v_i;           // Velocity [ẋ, ẏ, ż]
  Eigen::Quaterniond q_i;        // Attitude (roll, pitch, yaw)
  Eigen::Vector3d ω_i;           // Angular velocity
  
  // Rope measurements (from RopeForceSystem feedback)
  double T_i;                    // Tension magnitude (scalar)
  Eigen::Vector3d n_i;           // Tension direction (unit vector: payload direction)
  
  // Estimated load state (complementary filter)
  Eigen::Vector3d p_L_est_i;     // Load position estimate
  Eigen::Vector3d v_L_est_i;     // Load velocity estimate
  
  // Reference trajectory (shared across all drones)
  Eigen::Vector3d p_ref;         // Reference load position
  Eigen::Vector3d v_ref;         // Reference load velocity
  
  // Other drones' measured tensions (optional, for CLSR / balance)
  Eigen::VectorXd T_others;      // Tensions from other N-1 drones
};
```

**Critical Point**: Each drone estimates p_L independently!
- Drone 0: p_L_est_0 = p_0 - L*n_0  (from rope constraint)
- Drone 1: p_L_est_1 = p_1 - L*n_1  (from rope constraint)
- At equilibrium: p_L_est_0 ≈ p_L_est_1 (if estimates accurate)
- Disagreement: |p_L_est_0 - p_L_est_1| indicates estimation error or load oscillation

### Controller Hierarchy (per Drone)

```
Controller i (runs at 50 Hz):
  │
  ├─ ReadInputs(p_i, v_i, T_i, n_i, p_ref, p_L_est, m_L)
  │
  ├─ UpdateLoadEstimate()
  │   └─ p_L_est_i = α*z_measure + (1-α)*p_L_est_i
  │   └─ z_measure = p_i - L*n_i
  │
  ├─ SetupOptimizationProblem()
  │   ├─ Decision vars: [F_i[0], F_i[1], ..., F_i[N-1]]
  │   ├─ Objective: min w1·||p_L_est - p_ref||² + w3·||F_i||²
  │   ├─ Constraints: F_i_bounds, rope_tautness
  │   └─ (Note: Load state NOT in decision vars; decentralized)
  │
  ├─ Solve()
  │   └─ Drake SNOPT solver: 5-25 ms typical
  │
  ├─ ExtractControl()
  │   └─ Take receding horizon: u_i = F_i[0]  (first step only)
  │
  └─ ApplyToPlant()
      └─ Plant integrates: dp_i/dt = v_i, dv_i/dt = (F_i + rope_forces - m_i*g) / m_i
```

### Load Position Convergence Mechanism

**Why do estimates agree without communication?**

1. **Physics constraint**: Both ropes attached to same load
   - p_L = p_0 + L*n_0 = p_1 + L*n_1  (geometric constraint at equilibrium)
   - Therefore: p_L_est_0 ≈ p_L_est_1 if rope directions measured accurately

2. **Feedback coupling**: Rope tension indicates load motion
   - Drone 0 pulls up (F_0 ↑) → p_L rises → rope extends (L ↑) → T_0 decreases
   - Drone 1 observes: T_1 decreases (load moved up) → adapts control
   - Implicit signal: tension magnitude encodes load state

3. **Convergence**: Both estimates drift toward true load position
   - Complementary filter (α=0.1) low-pass filters measurement noise
   - Measurement: z = p_i - L*n_i is noisy but unbiased
   - Over time: p_L_est → true p_L as noise averages out

**Validation Method** (for Phase 2 tests):

```python
# Pseudocode for convergence analysis
def validate_implicit_coordination(sim_data, N_drones):
    p_L_true = sim_data['plant_state']['p_payload']
    p_L_estimates = [sim_data[f'drone_{i}']['p_load_est'] for i in range(N_drones)]
    
    # Pairwise agreement error
    agreement_errors = []
    for i in range(N_drones):
        for j in range(i+1, N_drones):
            error = np.linalg.norm(p_L_estimates[i] - p_L_estimates[j], axis=1)
            agreement_errors.append(error)
    
    # Estimation accuracy
    accuracy_errors = []
    for i in range(N_drones):
        error = np.linalg.norm(p_L_estimates[i] - p_L_true, axis=1)
        accuracy_errors.append(error)
    
    # Summary statistics
    max_agreement_error = np.max([np.max(e) for e in agreement_errors])
    steady_state_agreement = np.mean([e[-100:] for e in agreement_errors])  # Last 2 sec
    
    print(f"Max agreement error: {max_agreement_error:.3f} m")
    print(f"Steady-state agreement: {steady_state_agreement:.3f} m")
    print(f"=> Implicit coordination: {'PASS' if steady_state_agreement < 0.2 else 'FAIL'}")
    
    return {
        'max_agreement': max_agreement_error,
        'steady_state': steady_state_agreement,
        'accuracy': accuracy_errors,
    }
```

### Signal Flow Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│ PLANT (Drake MultibodyPlant + RopeForceSystem)                  │
│   State: [p_0, v_0, q_0, p_1, v_1, q_1, p_L, v_L, beads...]   │
│   Inputs: [F_0, τ_0, F_1, τ_1] from controllers                │
└─────────────────────────────────────────────────────────────────┘
                          ↓      ↓      ↓      ↓
            ┌─────────────────────────────────────────┐
            │        Output Extraction Layer           │
            │  (Sensor models, measurement noise)      │
            ├─────────────────────────────────────────┤
            │ Drone 0:                 Drone 1:        │
            │ p_0 → meas              p_1 → meas       │
            │ v_0 → meas              v_1 → meas       │
            │ T_0 → meas              T_1 → meas       │
            │ n_0 → meas              n_1 → meas       │
            └─────────────────────────────────────────┘
                    ↓                      ↓
        ┌───────────────────┐  ┌───────────────────┐
        │ Controller 0      │  │ Controller 1      │
        │                   │  │                   │
        │ Input ports:      │  │ Input ports:      │
        │  p_i, v_i, T_i,   │  │  p_i, v_i, T_i,   │
        │  n_i, p_ref,      │  │  n_i, p_ref,      │
        │  T_others[0:1],   │  │  T_others[0:1],   │
        │  m_L              │  │  m_L              │
        │                   │  │                   │
        │ Internal:         │  │ Internal:         │
        │  p_L_est_0        │  │  p_L_est_1        │
        │  v_L_est_0        │  │  v_L_est_1        │
        │                   │  │                   │
        │ Solver:           │  │ Solver:           │
        │  SetupOpt()       │  │  SetupOpt()       │
        │  Solve()          │  │  Solve()          │
        │                   │  │                   │
        │ Output: F_0, τ_0  │  │ Output: F_1, τ_1  │
        └─────────┬─────────┘  └─────────┬─────────┘
                  │                      │
                  └──────────┬───────────┘
                             ↓
              ┌──────────────────────────┐
              │ PLANT (next step)        │
              └──────────────────────────┘
```

**Key Point**: Red lines represent **only** physical forces (ropes) and sensor measurements. **No blue dotted lines** (no inter-drone communication).

---

## Section 4: Validation Metrics & Framework

### What Does "Implicit Coordination Works" Mean?

Three levels of evidence:

**Level 1: No Communication (Necessary Condition)**
- [ ] Code review: zero send/receive calls between controllers
- [ ] Network monitor: zero packets between drone 0 and drone 1 during simulation
- [ ] Timing: each controller runs independently; no blocking on other drone

**Level 2: Load Position Convergence (Sufficient Condition)**
- [ ] Estimate agreement: |p_L_est_0(t) - p_L_est_1(t)| → 0 as t increases
- [ ] Convergence rate: exponential decay with time constant τ < 3s
- [ ] Steady-state error: <0.2 m (allows 20cm disagreement; acceptable for soft cargo)

**Level 3: Thrust Balance (Implicit Equilibrium)**
- [ ] Force conservation: F_0 + F_1 + m_L*g ≈ (m_quad_0 + m_quad_1 + m_L)*g
- [ ] No oscillation: dF/dt stable; no runaway thrust
- [ ] Both drones active: neither drone saturated at max/min thrust (that would break coordination)

### Metric Definitions & Thresholds

| Metric | Definition | Phase 2a Target | Phase 2b Target | How Measured |
|--------|-----------|-----------------|-----------------|--------------|
| **Agreement Error** | max{&#124;p_L_est_i - p_L_est_j&#124;} | < 0.3 m | < 0.15 m | CSV columns |
| **Convergence τ** | Time constant of agreement decay | < 5 s | < 2 s | Curve fit exponential |
| **Thrust Balance** | &#124;T_0 + T_1 - m_L*g&#124; / (m_L*g) | < 15% | < 5% | CSV columns |
| **Solver Success** | # successful solves / total solves | > 95% | > 99% | CSV status column |
| **Solver Time** | P95 solve latency | < 25 ms | < 15 ms | CSV timing column |
| **Rope Tautness** | # steps with T < 0 / total steps | = 0 | = 0 | CSV tension column |
| **Position Tracking** | &#124;p_L - p_ref&#124; final 2 sec | < 0.1 m | < 0.05 m | CSV columns |

### Test Scenarios (Phase 2 Campaign)

**Scenario 1-5: Hover (N=2)**
- Drone 0: p_ref = [0, 0, 2.0] m
- Drone 1: p_ref = [0, 0, 2.0] m
- Duration: 10 s
- Expected: Both estimates converge to p_L_true ≈ [0, 0, 0.7] m (below 2m hover point)
- Measure: Convergence rate, steady-state error

**Scenario 6-10: Circular Motion (N=2)**
- Drone 0: p_ref = [1.0*cos(2π*0.1*t), 0, 2.0] m (circular, 10s period)
- Drone 1: p_ref = [1.0*cos(2π*0.1*t), 0, 2.0] m (same reference)
- Duration: 20 s (2 full circles)
- Expected: Estimates track, agreement maintained during motion
- Measure: Position tracking error, robustness to dynamic maneuvers

**Scenario 11-15: 3-Drone Hover (N=3)**
- All drones: p_ref = [0, 0, 2.0] m
- Duration: 15 s
- Expected: Pairwise agreements: |p_L_est_0 - p_L_est_1|, |p_L_est_0 - p_L_est_2|, |p_L_est_1 - p_L_est_2|
- Measure: Does coordination scale to N=3?

**Scenario 16-20: Cable Severance (N=2, preview of Phase 3)**
- Drone 0: p_ref = [0, 0, 2.0] m
- Drone 1: p_ref = [0, 0, 2.0] m
- At t=5.0s: Cable 0 severed (rope connection drops to zero)
- Duration: 15 s (5s nominal + 10s recovery)
- Expected: Solver detects infeasibility at t=5.0s; drone 1 increases thrust; load settles on drone 1
- Measure: Detection latency, recovery time, residual error

### Logging & Analysis Framework

**CSV Columns per Run** (per 50 Hz control cycle):
```
time[s], 
p_drone_0[3], v_drone_0[3], T_0, n_0[3],
p_drone_1[3], v_drone_1[3], T_1, n_1[3],
p_L_est_0[3], v_L_est_0[3],
p_L_est_1[3], v_L_est_1[3],
p_L_true[3] (from plant),
F_0, τ_0[3], solver_status_0, solver_time_0,
F_1, τ_1[3], solver_status_1, solver_time_1
```

**Analysis Script** (pseudocode):
```python
import pandas as pd
import numpy as np
from scipy.optimize import curve_fit

def analyze_phase2_results(csv_file, scenario_name):
    df = pd.read_csv(csv_file)
    
    # Extract columns
    p_L_est_0 = df[['p_L_est_0_x', 'p_L_est_0_y', 'p_L_est_0_z']].values
    p_L_est_1 = df[['p_L_est_1_x', 'p_L_est_1_y', 'p_L_est_1_z']].values
    p_L_true = df[['p_L_true_x', 'p_L_true_y', 'p_L_true_z']].values
    
    # Agreement error
    agreement = np.linalg.norm(p_L_est_0 - p_L_est_1, axis=1)
    
    # Fit exponential decay: agreement(t) = A*exp(-t/τ) + B
    def exponential(t, A, tau, B):
        return A * np.exp(-t / tau) + B
    
    t = df['time'].values
    try:
        popt, _ = curve_fit(exponential, t, agreement, p0=[0.5, 2.0, 0.01], maxfev=5000)
        A, tau, B = popt
        print(f"Convergence fit: τ = {tau:.2f} s, final error = {B:.3f} m")
    except:
        tau = np.inf
        B = np.mean(agreement[-20:])  # Last 0.4 sec
        print(f"Could not fit; steady-state error = {B:.3f} m")
    
    # Tracking accuracy
    tracking_error = np.linalg.norm(p_L_true - p_L_est_0, axis=1)
    print(f"Tracking RMSE: {np.sqrt(np.mean(tracking_error**2)):.3f} m")
    
    # Solver stats
    success_0 = np.mean(df['solver_status_0'] == 0)
    success_1 = np.mean(df['solver_status_1'] == 0)
    print(f"Solver success: drone 0 = {success_0:.1%}, drone 1 = {success_1:.1%}")
    
    return {
        'scenario': scenario_name,
        'convergence_tau': tau,
        'steady_state_error': B,
        'tracking_rmse': np.sqrt(np.mean(tracking_error**2)),
        'solver_success_0': success_0,
        'solver_success_1': success_1,
    }

# Run analysis on all 10 scenarios
results = []
for i in range(1, 11):
    csv = f"phase2_scenario_{i:02d}.csv"
    results.append(analyze_phase2_results(csv, f"Scenario {i}"))

# Summary table
summary_df = pd.DataFrame(results)
print(summary_df.to_string(index=False))
print(f"\nPhase 2 Success: Avg convergence τ = {summary_df['convergence_tau'].mean():.2f} s")
print(f"                Avg agreement error = {summary_df['steady_state_error'].mean():.3f} m")
```

---

## Section 5: Risk Register & Mitigation

| # | Risk | Likelihood | Impact | Mitigation | Fallback | Owner |
|---|------|-----------|--------|-----------|----------|-------|
| **R1** | Rope slack (T < 0) makes optimizer infeasible | HIGH | Blocks Phase 2 | Use slack variables; add penalty term w_slack·Σslack[k]; reduce control horizon | Simplify rope model; use open-loop fallback policy | Algorithm |
| **R2** | Load position estimates diverge between drones | MEDIUM | Coordination fails; must add communication | Verify measurement accuracy; add filtering; increase w_trajectory | Reduce control horizon; implement 1-way broadcast of reference (acceptable fallback) | Validation |
| **R3** | Solver time > 50 ms causes control lag | MEDIUM | Unstable control; may miss deadlines | Profile with Drake; reduce horizon N; optimize SNOPT tolerances | Implement simplified dynamics (linear approximation) | Engineering |
| **R4** | Test execution takes >4 weeks | MEDIUM | Schedule slip; delays Phase 3 | Parallelize runs across machines; reduce scenario count to 5 | Accept fewer results; publish with caveats | Project Mgmt |
| **R5** | Multi-drone test harness buggy (state wiring error) | MEDIUM | Wasted debugging effort | Code review state splitter early (Week 1); unit test state extraction | Revert to single-drone; debug wiring on paper first | Testing |
| **R6** | Cable severance not detected (Phase 4 preview fails) | LOW | Phase 3 delayed | Explicit infeasibility detection in solver result handler | Document limitation; plan more thorough Phase 3 design | Algorithm |
| **R7** | SNOPT solver not available (Drake build issue) | LOW | Can't run MPC solver | Check Drake installation early; have fallback (scipy minimize) | Use quadratic programming solver (less flexible) | Infrastructure |
| **R8** | Load estimator filter unstable (divg to NaN) | LOW | Simulation crashes | Add bounds checking; clip p_L_est to reasonable range | Use Kalman filter instead of complementary filter | Algorithm |

### Escalation Protocol

**If Risk R1 (rope slack) occurs**:
1. Analyze root cause: Is rope too short? Drones pulling apart too fast?
2. Try mitigation: Increase slack penalty weight from 1000 to 10000
3. If still infeasible: Reduce control horizon from 3s to 1.5s
4. If still infeasible: Switch to simplified rope model (massless, inextensible)
5. If optimizer still fails: Pause Phase 2; escalate to design review

**If Risk R3 (solver time) occurs**:
1. Profile solver with Drake's built-in timing (% in setup, solve, extraction)
2. Bottleneck: if setup > 50%, reduce # decision variables (shorter horizon)
3. Bottleneck: if solve > 50%, reduce solver tolerance from 1e-4 to 1e-3
4. Bottleneck: if extraction > 50%, optimize result parsing
5. If mean time > 25 ms: reduce control frequency from 50 Hz to 33 Hz (30 ms period)

---

## Section 6: Phase 2 Success Criteria Checklist

### Functionality (Code & Execution)

#### Phase 1 Remediation
- [ ] Controller reads plant state correctly (state splitter wired)
  - Evidence: Unit test for StateExtractor; manual verification of p_drone output
- [ ] MPC objective function implemented (trajectory + effort costs)
  - Evidence: Solver cost value logged; decreases over iterations
- [ ] Rope tautness constraint added to optimizer
  - Evidence: No solver solutions with T[k] < 0; slack variables ≤ tolerance
- [ ] Load estimator uses discrete state (no mutable members)
  - Evidence: Code review; no mutable Eigen::Vector3d in class
- [ ] All Phase 1 tests pass (single-drone closed-loop)
  - Evidence: `decentralized_mpc_test --headless --duration 10.0` completes; logs ≥450 samples (50 Hz × 10s)

#### Two-Drone Test
- [ ] Two-drone executable compiles and runs
  - Evidence: `decentralized_mpc_test_2drones --help` works; test completes in <2 min
- [ ] Load position estimates agree within 0.2 m at t=10s
  - Evidence: CSV column analysis; |p_L_est_0 - p_L_est_1| < 0.2 m (last 10% of data)
- [ ] No inter-drone communication observed
  - Evidence: Static code analysis; zero send/receive calls between controller instances
- [ ] Rope remains taut throughout simulation
  - Evidence: CSV tension column T_0 ≥ 0, T_1 ≥ 0 for all time steps

#### Three-Drone Test
- [ ] Three-drone executable runs on 10 scenarios
  - Evidence: 10 CSV output files; each with ≥900 rows (50 Hz × 15s ≥ minimum)
- [ ] Pairwise agreement converges: all pairs |p_L_est_i - p_L_est_j| < 0.3 m
  - Evidence: Analysis script confirms max pairwise error in last 2 sec < 0.3 m
- [ ] Solver success rate ≥ 95% per run
  - Evidence: CSV status columns; count successes

### Documentation

- [ ] **Phase 2 Architecture Document** (2 pages)
  - Includes: System diagram, signal flow, state representation, convergence mechanism
  - Evidence: PDF/MD file in repository; cited in this plan
- [ ] **Multi-Drone Test Results** (10 scenarios, 2-3 pages)
  - Includes: Table of metrics (agreement, convergence τ, tracking error)
  - Evidence: Plots of p_L_est_0, p_L_est_1, p_L_true vs. time (1 figure per scenario)
- [ ] **Implicit Coordination Explanation** (1-2 pages)
  - Explains how rope forces implicitly couple controllers
  - Evidence: Mathematical argument + empirical validation
- [ ] **PHASE_2_RESULTS.md** with metrics summary
  - Includes: All metrics from Section 4; go/no-go decisions; Phase 3 readiness assessment

### Code Quality

- [ ] All new source files follow Drake conventions
  - No mutable state in const methods
  - Proper port declarations and lifecycle
  - Evidence: Code review by Drake expert
- [ ] Test harness produces valid CSV logs
  - Columns align with specification in Section 4
  - Evidence: CSV parsing script runs without errors
- [ ] All Drake tests pass (no regression)
  - Evidence: `ctest` output; zero failures

### Insights & Scientific Contribution

- [ ] **Implicit Coordination Proof** (conceptual or empirical)
  - Statement: "Two independent decentralized MPC controllers, operating without inter-drone communication, achieve load position estimate agreement within X% via implicit rope coupling."
  - Evidence: Convergence analysis (τ, steady-state error); comparison to expected behavior
- [ ] **Convergence Rate Characterization**
  - Statement: "Load position estimates converge with time constant τ ≈ X seconds, independent of drone count (N=2 vs. N=3)."
  - Evidence: Curve fitting; tables comparing scenarios
- [ ] **Robustness to Network Failure** (Phase 3 preview)
  - Statement: "Single-cable severance detected and compensated within Y seconds; residual load error ≤ Z meters."
  - Evidence: Severance scenario results; time-to-recovery plot

---

## Section 7: Phase 3 Foresight (Cable Failure Foundation)

### What Phase 2 Must Provide for Phase 3

Phase 3 objective: **Decentralized Multi-Drone Recovery from Progressive Cable Faults**

#### Prerequisite 1: Implicit Coordination Validation (Phase 2 output)
- **What**: Proof that N≥2 drones coordinate without communication
- **Why Phase 3 needs it**: Must know baseline coordination behavior BEFORE introducing faults
- **How Phase 2 proves it**: Convergence metrics, thrust balance, zero communication
- **Phase 3 use**: Comparison baseline; if estimates diverge during fault, it's due to fault, not fundamental coordination failure

#### Prerequisite 2: Robust Load Estimator (Phase 2 output)
- **What**: Load position estimate remains accurate during normal operation
- **Why Phase 3 needs it**: Estimate may be unreliable during cable slack; need to quantify uncertainty
- **How Phase 2 proves it**: Tracking error vs. true position; noise analysis
- **Phase 3 use**: Adaptive Kalman filter tuning; confidence bounds on p_L_est

#### Prerequisite 3: Solver Infeasibility Detection (Phase 2 output)
- **What**: Optimizer detects when constraints become infeasible
- **Why Phase 3 needs it**: Cable severance makes optimization problem infeasible; must trigger fallback
- **How Phase 2 proves it**: In Week 4 cable severance scenario, verify solver.is_success() == false at t=5s
- **Phase 3 use**: Trigger fault handler (CLSR, emergency descent, etc.)

#### Prerequisite 4: Fallback Control Policy Undersized Analysis (Phase 1 fix)
- **What**: Document why fallback (constant thrust = 0.5*F_max) is inadequate for multi-drone
- **Why Phase 3 needs it**: Fallback must be improved for fault scenarios
- **How Phase 2 proves it**: Show that fallback would fail if cable severed
- **Phase 3 use**: Design Phase 3 fallback (thrust lookup table, bang-bang control, etc.)

### Phase 3 Design: Cable Failure Scenarios

**Scenario A: Single Cable Gradual Degradation (Fatigue)**
```
Time:     0s ────────── 5s ────────── 10s ────────── 15s
Cable 0:  100% → 90% → 70% → 40% → 0% (severed)
Control: Normal → Normal → Early warning → Reallocate → Recovery
Drones:  1,2,3,4       1,2,3,4     Alert issued  1,2,3,4    1,2,3 (+ descent)
```
- **Phase 2 foundation**: Validate NORMAL (0-5s) behavior; use as baseline
- **Phase 3 extension**: Detect degradation (5-10s) via load estimate uncertainty growth
- **Phase 3 extension**: Reallocate thrust (10-15s) to remaining drones
- **Phase 3 extension**: Coordinated descent (15-20s) with reduced thrust

**Scenario B: Sudden Cable Severance (Shock)**
```
Time:     0s ────── 5s ────────────────── 15s
Cable 0:  100% → 0% (severed at t=5s)
Control: Normal → Infeasible → Fallback → Recovery
Drones:  1,2,3,4    Alert!      1,2,3,4    1,2,3,4
```
- **Phase 2 foundation**: Cable severance test (Week 4) is dry run
- **Phase 3 extension**: Improve fallback policy; add thrust redistribution
- **Phase 3 extension**: Cascading fault (2 drones severed); validate graceful degradation

**Scenario C: Redundancy & Reconfiguration (Multi-Point Failure)**
```
Time:     0-5s ──── 5s ────── 10s ────────── 20s
Cable 0:  OK ──→ Severed; drone 1 offline
Cable 1:  OK ──→ ? (can drone 2 handle?)
Control: Nominal → Alert → Reconfigure → Coordinated recovery
```
- **Phase 2 foundation**: Implicit coupling behavior (prevents cascading instability)
- **Phase 3 extension**: Determine max cable failures supportable by N drones

### Cable Failure Detection Strategy (Phase 3 Options)

**Option 1: Optimization-Based (Implicit Infeasibility)**
- Solver detects that no feasible solution exists
- Advantage: Leverages existing MPC infrastructure
- Disadvantage: Reaction time = 1 control cycle (20 ms); may be too slow
- Phase 2 validation: Week 4 cable severance test demonstrates detection

**Option 2: Residual-Based (Load Estimator Divergence)**
- Monitor |p_L_est_i - p_L_est_j| between drone pairs
- If divergence > threshold: Indicates one drone has lost rope contact
- Advantage: Can detect partial degradation
- Disadvantage: Requires tuning threshold; slow (10-50 ms detection lag)
- Phase 2 validation: Implicit agreement error metric (Section 4) provides baseline

**Option 3: Tension-Based (Sudden Tension Drop)**
- Monitor dT/dt in cable tension
- If dT/dt < -threshold for >2 cycles: Cable likely severed
- Advantage: Fast (1-2 ms detection)
- Disadvantage: Noisy; may false-trigger on control transients
- Phase 2 validation: Tension CSV columns provide historical analysis

**Option 4: Hybrid (Option 1 + Option 2)**
- Use both optimization infeasibility AND estimate divergence
- Trigger fallback on first detection
- Phase 2 validation: Implement all three in simulation; compare detection times

### Phase 3 Contingency Descent Strategy

**Goal**: If N drones remaining, safely lower load to ground in <T_max seconds without crashing.

**Strategy 1: Coordinated Thrust Reduction**
```
For each drone i:
  T_i[k] = 0.6 * (m_load/N_remaining + m_drone) * g   (hover thrust for shared load)
  Setpoint: p_i descends at constant v_descent = 0.5 m/s
  Duration: h_current / v_descent ≈ 4 seconds to reach ground
```
- Simple, decentralized
- All drones execute same command (no communication)
- Risk: If one drone fails during descent, load crashes

**Strategy 2: Progressive Reallocation (CLSR)**
- Phase 2's CLSR analog: minimize ||F_i - F_nom||² subject to load constraints
- Phase 3 extension: Include soft landing constraint (v_L ≤ 1.0 m/s at contact)
- Phase 3 extension: Detect ground contact; reduce thrust to zero

**Strategy 3: Emergency Parachute / Airbag**
- Backup for hardware failure
- Not simulable in current framework

### Phase 3 Success Criteria (Sketch)

- [ ] Single cable severance at t=5s detected within 30 ms
- [ ] N=3 drones successfully land N-1 capability; load stays within bounds throughout
- [ ] Progressive degradation (fatigue) detected before catastrophic failure
- [ ] Recovery time (t_severity → t_stable) < 5 seconds
- [ ] No oscillations or instability during fault transients

---

## Summary: 4-Week Timeline & Deliverables

| Week | Focus | Deliverables | Success Gate |
|------|-------|--------------|--------------|
| **1** | Phase 1 Remediation | Plant truth extractor, MPC cost function, discrete state refactor, rope constraint | Phase 1 test passes ≥99% solver success |
| **2** | Single-drone + 2-drone | Phase 1 final report, 2-drone test harness, 10 scenarios × 1 seed | 2 drones maintain <0.2m agreement error |
| **3** | 3-drone formation | 3-drone test harness, convergence analysis, 10 scenarios × 1 seed | 3-drone pairwise agreement <0.3 m |
| **4** | Cable severance + Phase 3 preview | Fault scenario test (5 runs), Phase 3 design doc, recovery analysis | Detection latency <30 ms; recovery <2 s |

---

## Appendix A: Code Templates & Snippets

### A.1: StateExtractor Skeleton
```cpp
// File: include/state_extractor.h
#pragma once
#include <drake/systems/framework/leaf_system.h>
#include <Eigen/Core>

namespace quad_rope_lift {

class StateExtractor : public drake::systems::LeafSystem<double> {
 public:
  struct Config {
    int num_beads;
    double rope_length;
    int drone_index;
  };

  explicit StateExtractor(const Config& config);

  const drake::systems::InputPort<double>& get_plant_state_input_port() const {
    return get_input_port(0);
  }

  const drake::systems::OutputPort<double>& get_drone_position_output_port() const {
    return get_output_port(0);
  }

  const drake::systems::OutputPort<double>& get_drone_velocity_output_port() const {
    return get_output_port(1);
  }

  const drake::systems::OutputPort<double>& get_cable_tension_output_port() const {
    return get_output_port(2);
  }

  const drake::systems::OutputPort<double>& get_cable_direction_output_port() const {
    return get_output_port(3);
  }

 private:
  void CalcDronePosition(const drake::systems::Context<double>&, 
                         drake::systems::BasicVector<double>*) const;
  void CalcDroneVelocity(const drake::systems::Context<double>&, 
                         drake::systems::BasicVector<double>*) const;
  void CalcCableTension(const drake::systems::Context<double>&, 
                        drake::systems::BasicVector<double>*) const;
  void CalcCableDirection(const drake::systems::Context<double>&, 
                          drake::systems::BasicVector<double>*) const;
  Config config_;
};

}  // namespace quad_rope_lift
```

### A.2: MPC Cost Function Template
```cpp
// In SetupOptimizationProblem():

// Cost 1: Trajectory tracking
auto cost_pos = (p_L_est - p_ref).dot(p_L_est - p_ref) * config_.w_trajectory;
prog->AddCost(cost_pos);

// Cost 2: Control effort
for (int k = 0; k < N; ++k) {
  auto cost_effort = U.col(k).dot(U.col(k)) * config_.w_effort;
  prog->AddCost(cost_effort);
}

// Cost 3: Slack (rope tautness)
for (int k = 0; k < N; ++k) {
  prog->AddCost(1000.0 * slack_tension(k));
}
```

### A.3: Analysis Python Script
```python
# File: analysis/phase2_convergence_analysis.py
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

def load_scenario(scenario_id):
    csv_file = f"phase2_scenario_{scenario_id:02d}.csv"
    return pd.read_csv(csv_file)

def analyze_agreement(df):
    p_est_0 = df[['p_L_est_0_x', 'p_L_est_0_y', 'p_L_est_0_z']].values
    p_est_1 = df[['p_L_est_1_x', 'p_L_est_1_y', 'p_L_est_1_z']].values
    
    agreement = np.linalg.norm(p_est_0 - p_est_1, axis=1)
    
    # Exponential curve fit
    def exp_decay(t, A, tau, B):
        return A * np.exp(-t / tau) + B
    
    t = df['time'].values
    try:
        popt, _ = curve_fit(exp_decay, t, agreement, p0=[0.5, 2.0, 0.01])
        A, tau, B = popt
    except:
        tau, B = np.inf, np.mean(agreement[-20:])
    
    return agreement, tau, B

# Run analysis
for scenario in range(1, 11):
    df = load_scenario(scenario)
    agreement, tau, steady_state = analyze_agreement(df)
    
    print(f"Scenario {scenario:2d}: τ={tau:5.2f}s, error={steady_state:6.3f}m")
```

---

## Final Notes

This Phase 2 plan is **executable** and **risk-aware**. Success requires:

1. **Technical execution** (Weeks 1-4): Remediate Phase 1, validate 2-drone coordination, scale to 3 drones, test fault resilience
2. **Rigorous validation** (throughout): Use metrics in Section 4; don't claim success without evidence
3. **Adaptive strategy** (risk management): If critical risks materialize, escalate early; fallback plans are pre-approved
4. **Documentation discipline** (Weeks 1-4): Produce architecture doc, results tables, code comments; scientific paper-ready

**Go/No-Go Gates**:
- **After Week 1**: Phase 1 test must achieve ≥99% solver success + <10 ms mean solve time
- **After Week 2**: Two drones must maintain <0.2 m agreement error; zero communication verified
- **After Week 3**: Three drones must show scalability; convergence time constant documented
- **After Week 4**: Cable severance must be detected and recovered; Phase 3 strategy locked

Phase 2 is **complete** when all sections of this plan show green checkmarks, and the Phase 2 Results document is approved for publication.

