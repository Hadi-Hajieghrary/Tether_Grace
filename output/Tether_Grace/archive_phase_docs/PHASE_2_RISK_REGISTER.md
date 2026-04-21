# Phase 2 Risk Register & Mitigation Framework

**Date**: 2026-04-17  
**Target Period**: Weeks of April 21 - May 16, 2026  
**Audience**: Technical team, project stakeholders

---

## Executive Summary

Phase 2 introduces **8 identified risks** across three categories:

| Category | Count | Likelihood | Typical Impact |
|----------|-------|-----------|-----------------|
| **Algorithm** | 3 | HIGH/MEDIUM | Blocks progression, schedule slip |
| **Testing** | 3 | MEDIUM/LOW | Wasted debugging, false negatives |
| **Infrastructure** | 2 | LOW | Build delays, tool chain issues |

All risks have **documented mitigations** and **pre-approved fallbacks**. Escalation protocol defined in Section 4.

---

## Risk Register (Detailed)

### R1: Rope Slack Makes Optimizer Infeasible

**Risk ID**: R1  
**Category**: Algorithm  
**Likelihood**: HIGH (70%)  
**Impact**: CRITICAL — Blocks all Phase 2 testing  
**Severity Score**: 70% × 10 = 7.0 / 10

#### Description
During MPC optimization, the rope tautness constraint T[k] ≥ 0 can become impossible to satisfy if:
- Drones pull apart faster than rope can extend
- Load acceleration exceeds bounds of optimizer model
- Rope is too short or friction model is inaccurate

When infeasible, solver returns `is_success() == false` and controller falls back to constant thrust (0.5*F_max), which is undersized for multi-drone scenarios.

#### Historical Precedent
- **Phase 1**: Optimizer is empty stub (no objectives) → trivially feasible
- **Phase 2**: When objectives added, rope constraint may become binding
- **Similar system**: Cooperative aerial load transport (prior art) documented this issue

#### Detection Signals
1. **Solver status**: `result.is_success() == false` on first few solves
2. **Log analysis**: Solver repeatedly hits iteration limit without convergence
3. **Physical plausibility**: Estimated load position becomes NaN or extreme (>10m)

#### Probability Calculation
- P(rope model inaccurate) = 15%
- P(optimizer formulation error) = 30%
- P(control horizon too long) = 25%
- **Combined** (at least one): ~1 − 0.85×0.70×0.75 = 55% → round to 70% with safety margin

---

### R1: Mitigation Plan (Detailed)

#### Primary Mitigation: Slack Variables + Penalty

**What to do**:
In `SetupOptimizationProblem()`, replace hard constraint with soft:

```cpp
// HARD (causes infeasibility):
//   prog->AddConstraint(T[k] >= 0 for all k);

// SOFT (allows graceful relaxation):
auto slack_T = prog->NewContinuousVariables(N, "slack_T");
prog->AddBoundingBoxConstraint(0, 10.0, slack_T);  // Max 10N slack

for (int k = 0; k < N; ++k) {
  // Constraint becomes: T[k] >= -slack_T[k]
  // Cost penalty: 10000 * slack_T[k]  (encourages slack → 0)
  prog->AddCost(10000.0 * slack_T(k));
}
```

**Why it works**:
- If T[k] stays ≥ 0, slack = 0 → no penalty, constraint satisfied
- If T[k] must go < 0, solver allows it but pays 10000× cost per Newton
- Optimizer naturally tries to minimize slack (keep rope taut)
- Even if slack > 0, solution is feasible → solver succeeds

**Trade-off**: Solution may be suboptimal (slack > 0 increases cost), but **feasibility guaranteed** → fallback policy not needed.

**Implementation timeline**: Week 1, Phase 1 remediation, Fix 4

---

#### Secondary Mitigation: Reduce Control Horizon

**What to do**:
If solver still infeasible with slack variables, reduce horizon:

```cpp
// Current: 3.0 seconds, dt=0.01 → N=300 decision variables
// Try: 2.0 seconds, dt=0.01 → N=200 decision variables
// Try: 1.5 seconds, dt=0.01 → N=150 decision variables
// Try: 1.0 seconds, dt=0.01 → N=100 decision variables

config_.control_horizon_sec = 1.5;  // Halve horizon
```

**Why it works**:
- Shorter horizon = fewer decision variables = smaller optimization problem
- Tautness constraint easier to satisfy over 1.5s than 3.0s
- Feedback control loop still runs at 50 Hz (no change to control frequency)
- Trade-off: Less foresight; controller reacts more than predicts

**Effectiveness**: Typically solves infeasibility if root cause is horizon length

**Implementation timeline**: Week 1-2 (if R1 manifests)

---

#### Tertiary Mitigation: Simplify Rope Model

**What to do**:
Replace bead-chain dynamics with point-mass load + simplified rope:

```cpp
// Current: Rope is 8-bead chain; coupled dynamics with load
// Simplified: Rope is massless, inextensible; tension = spring force

// Tension estimate:
double k_rope = 200.0;  // Spring constant [N/m]
double L_natural = config_.rope_length;
double L_current = (p_payload - p_drone).norm();
double T_estimate = k_rope * std::max(0.0, L_natural - L_current);

// Use T_estimate in tautness constraint instead of full model
```

**Why it works**:
- Reduces problem complexity
- Tautness becomes algebraic constraint (T = f(positions)) not differential
- Optimizer can reason about feasibility more easily

**Trade-off**: Less fidelity to actual rope physics; may miss some dynamics

**Implementation timeline**: Week 2-3 (only if R1 AND secondary mitigation both fail)

---

#### Fallback (Approved Contingency)

**Scenario**: All mitigations fail; optimizer remains infeasible even with slack.

**What to do**:
1. Disable MPC optimizer; fall back to **pre-computed thrust lookup table**
2. Table indexed by: [estimated_load_height, vertical_velocity]
3. Lookup returns: nominal thrust for single drone (phase out MPC logic)
4. Log event: increment `solver_infeasibility_count`
5. **Continue test**: Fallback policy allows simulation to proceed (won't fail)

**Implementation**: Implement ThrustLookupTable class (Phase 2, Week 1 contingency)

**Risk exposure if fallback triggered**: 
- Phase 2 cannot demonstrate MPC + rope tautness + coordination together
- Deliverable changes: results show "fallback policy test" instead of "MPC test"
- **Not a schedule slip** (fallback is pre-built), but **lower scientific value**

---

### R1: Escalation Trigger & Protocol

**When to escalate R1**: 
- If solver returns `is_success() == false` on >5% of solves after Week 1 remediation complete

**Escalation action**:
1. Call technical review (1 hour); involve Drake expert if available
2. Profile optimizer with Drake's profiler; identify bottleneck
3. Implement secondary mitigation (reduce horizon)
4. If still failing: tertiary mitigation (simplify rope)
5. If all mitigations insufficient: **loop back to design** (horizon may be unrealistic; may need to abandon MPC for Phase 2)

**Owner**: Algorithm/Drake lead  
**Decision authority**: Project PI  
**Timeline**: Escalate decision by EOD Week 1

---

## Risk R2: Load Position Estimates Diverge (Coordination Failure)

**Risk ID**: R2  
**Category**: Algorithm  
**Likelihood**: MEDIUM (40%)  
**Impact**: HIGH — Coordination unvalidated; defeats Phase 2 purpose  
**Severity Score**: 40% × 8 = 3.2 / 10

#### Description
The core claim of Phase 2 is: **Two independent decentralized controllers coordinate implicitly (via rope forces) without explicit communication.**

If estimates p_L_est_0 and p_L_est_1 remain far apart (|error| > 0.3m) throughout simulation, it suggests:
1. Estimates are not converging to true load position
2. Each drone has different belief about load state → different control choices
3. Implicit coordination is **not working**

#### Root Causes (Priority Order)
1. **Rope measurement noise too high** (15% probability)
   - Complementary filter (α=0.1) insufficient to filter noise
   - z_measured = p_i - L*n_i has high variance if p_i or n_i noisy

2. **Filter time constant too long** (10% probability)
   - α=0.1 → convergence time τ ≈ 10s; may not converge in 10s test duration
   - Should use higher α (faster) but risks instability

3. **Rope kinematics constraint violated** (10% probability)
   - Assumption: p_L = p_i - L*n_i assumes rope always taut and straight
   - Reality: rope can curve, stretch, slip → constraint inaccurate
   - Drone positions inconsistent with same load → estimates diverge

4. **Load acceleration model wrong** (5% probability)
   - Load acceleration a_L = (T_total - m_L*g) / m_L assumes point mass
   - Real rope has distributed mass; beads are not massless
   - Estimated a_L wrong → filter diverges from true state

#### Detection Signals
1. **Test output**: CSV column analysis shows |p_L_est_0 - p_L_est_1| > 0.3m at t=10s
2. **Convergence curve**: Agreement error does NOT decay exponentially; flat or growing
3. **Comparison to actual**: p_L_est_0 and p_L_est_1 track different trajectories vs. true p_L

#### Probability Calculation
- P(rope measurement noisy) = 15%
- P(filter tuning poor) = 10%
- P(kinematics assumption broken) = 10%
- P(model mismatch) = 5%
- **At least one manifests**: ~1 − 0.85×0.90×0.90×0.95 = 38% → round to 40%

---

### R2: Mitigation Plan

#### Primary: Validate Measurement Accuracy (Week 1-2)

**What to do**:
1. In Phase 1 single-drone test, log both:
   - True load position p_L_true (from plant)
   - Measured load position z_meas = p_i - L*n_i (from rope constraint)
   - Estimated load position p_L_est (after filtering)

2. Compute measurement error: e_meas = ||z_meas - p_L_true||

3. Analyze error distribution:
   ```python
   mean_error = np.mean(e_meas)
   std_error = np.std(e_meas)
   max_error = np.max(e_meas)
   print(f"Measurement accuracy: μ={mean_error:.3f}m, σ={std_error:.3f}m")
   ```

4. If σ_error > 0.1m: Noise too high; implement better measurement (see below)

**Timeline**: Week 1, during Phase 1 validation

---

#### Secondary: Increase Filter Gain (If Filter Too Slow)

**What to do**:
```cpp
// Original: α = 0.1 (slow filter, τ ≈ 10 seconds)
// Increased: α = 0.25 (faster filter, τ ≈ 4 seconds)
// Even faster: α = 0.5 (very fast, τ ≈ 2 seconds)

config_.load_estimator_filter_alpha = 0.25;

// Test all three and compare:
for (double alpha : {0.1, 0.25, 0.5}) {
  // Run scenario, measure convergence time, noise rejection
  // Pick α that gives best trade-off
}
```

**Trade-off analysis**:
- Higher α → faster response, more noise
- Lower α → better noise rejection, slower convergence
- Optimal: α where convergence time ~ 3s and noise < 0.1m

**Implementation timeline**: Week 2 (if R2 manifests)

---

#### Tertiary: Improve Rope Measurement (Better Sensor Model)

**What to do**:
Replace simple rope constraint z = p_i - L*n with weighted measurement:

```cpp
// Current (simple):
Eigen::Vector3d z_measure = p_i - config_.rope_length * n_i;

// Improved (weights by rope tautness):
Eigen::Vector3d z_measure;
if (T_i > 1.0) {  // Rope is taut
  z_measure = p_i - config_.rope_length * n_i;  // Use measurement
} else {  // Rope is slack; measurement unreliable
  z_measure = p_L_est_prev;  // Trust previous estimate, not measurement
}

// Even better (Kalman filter):
// Measurement model: z = p_i - L*n + noise ~ N(0, R)
// Process model: dp_L/dt = v_L + process_noise ~ N(0, Q)
// Use Kalman filter instead of complementary filter
```

**Why it works**:
- Acknowledges that rope constraint is approximate (not perfect)
- Handles rope slack gracefully
- Kalman filter is optimal (minimum error variance)

**Implementation timeline**: Week 2-3 (only if primary + secondary both insufficient)

---

#### Fallback (Approved Contingency)

**Scenario**: Estimates still diverge after tuning filter; R2 becomes unmitigatable.

**What to do**:
1. **Redefine success criterion**: Instead of "estimates agree within 0.2m", change to:
   - "Estimates agree within 0.5m" (2-drone practical tolerance)
   - OR "Thrust balance equation satisfied" (secondary proof of coordination)

2. **Document the issue**: Write technical note explaining why estimates diverge
   - Root cause: rope kinematics assumption breaks under dynamic loading
   - Recommendation: Add distributed-mass rope model (Phase 3+)

3. **Proceed with caution**: Phase 2 can still validate thrust balance and overall system behavior
   - May not prove implicit coordination via estimate agreement
   - But can prove via physical force balance

---

### R2: Escalation Trigger

**When to escalate R2**:
- After Week 2 two-drone test, if |p_L_est_0 - p_L_est_1|_steadystate > 0.4m (double target)

**Escalation action**:
1. Analyze convergence plots; identify root cause (noise? drift? coupling?)
2. Try secondary mitigation (adjust filter α)
3. If unsuccessful: Call technical review; consider design alternatives
4. If unresolvable: Accept fallback criterion; redefine Phase 2 success as "thrust balance + robustness", not "estimate agreement"

**Owner**: Validation / Control lead  
**Decision authority**: Project PI  
**Timeline**: Escalate by EOD Week 2

---

## Risk R3: Solver Time > 50 ms (Control Lag)

**Risk ID**: R3  
**Category**: Algorithm  
**Likelihood**: MEDIUM (35%)  
**Impact**: HIGH — Control becomes unstable or misses deadlines  
**Severity Score**: 35% × 8 = 2.8 / 10

#### Description
MPC must solve in ≤20ms to maintain 50 Hz control frequency with 0ms communication latency.

If mean solver time > 20ms, then:
- Receding horizon MPC can't update fast enough
- Control becomes open-loop between solves
- If solver time > 50ms: Misses deadline; previous control output held; possible instability

**Acceptable performance**:
- P50 solve time: <10 ms (typical)
- P95 solve time: <15 ms (occasional large problem)
- P99 solve time: <25 ms (rare worst-case)
- Mean: <8 ms

#### Root Causes (Priority Order)
1. **Control horizon too long** (20% probability)
   - Current: 3.0 seconds → N=300 decision variables
   - Solver complexity ~ O(N²) → 300 vars is expensive
   - Solution: Reduce to N=150 (1.5s horizon)

2. **SNOPT solver not well-tuned** (10% probability)
   - Feasibility tolerance too tight (1e-6 instead of 1e-4)
   - Major iteration limit too high (100 instead of 50)
   - Print frequency enabled (verbose output slows solver)

3. **Problem structure not exploited** (5% probability)
   - Optimizer doesn't leverage sparsity of constraint matrix
   - Should use sparse linear algebra, but Drake may use dense
   - Solution: Reformulate problem to be more sparse

---

### R3: Mitigation Plan

#### Primary: Measure Solver Time (Week 1)

**What to do**:
```cpp
// In CalcOptimalControl:
auto t_start = std::chrono::high_resolution_clock::now();
auto result = drake::solvers::Solve(*prog, ...);
auto t_end = std::chrono::high_resolution_clock::now();
double solver_time_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

// Log to CSV
solver_time_log << solver_time_ms << "\n";
```

2. After Phase 1 test (10 runs), analyze distribution:
   ```python
   times = np.loadtxt("solver_time.csv")
   print(f"P50: {np.percentile(times, 50):.2f} ms")
   print(f"P95: {np.percentile(times, 95):.2f} ms")
   print(f"Max:  {np.max(times):.2f} ms")
   ```

3. If P95 > 15 ms: Trigger secondary mitigation

**Timeline**: Week 1 (automated measurement)

---

#### Secondary: Tune SNOPT Options (Week 1-2)

**What to do**:
```cpp
drake::solvers::SolverOptions opts;

// Reduce iteration limits
opts.SetOption(drake::solvers::SnoptSolver::id(), 
               "Major iterations limit", 50);  // Down from 100
opts.SetOption(drake::solvers::SnoptSolver::id(), 
               "Minor iterations limit", 50);  // Down from 100

// Relax tolerances
opts.SetOption(drake::solvers::SnoptSolver::id(), 
               "Feasibility tolerance", 1e-4);  // Up from 1e-6
opts.SetOption(drake::solvers::SnoptSolver::id(), 
               "Optimality tolerance", 1e-3);   // Up from 1e-4

// Disable verbose output
opts.SetOption(drake::solvers::SnoptSolver::id(), 
               "Print frequency", 0);  // Disable printing

auto result = drake::solvers::Solve(*prog, std::nullopt, std::make_optional(opts));
```

**Expected improvement**: 20-30% reduction in solve time

**Trade-off**: Slightly less accurate solution, but still feasible

**Timeline**: Week 1-2

---

#### Tertiary: Reduce Control Horizon (Week 2)

**What to do**:
```cpp
// Current: 3.0 seconds
config_.control_horizon_sec = 1.5;  // Halve it

// Recompute N:
int N = static_cast<int>(config_.control_horizon_sec / config_.control_dt);
// N = 1.5 / 0.01 = 150 (down from 300)
```

**Expected improvement**: ~50% reduction in solver time (roughly O(N²) → O(N²/4))

**Trade-off**: Shorter planning horizon; controller more reactive, less predictive

**Testing**: Verify control stability doesn't degrade (load oscillations, etc.)

**Timeline**: Week 2 (if secondary mitigation insufficient)

---

#### Fallback: Lower Control Frequency (Approved Contingency)

**Scenario**: Even with horizon reduced to 1.0s, solver time > 25ms.

**What to do**:
1. **Reduce control frequency**: 50 Hz → 33 Hz (period 30ms)
   ```cpp
   config_.mpc_period = 0.030;  // 33 Hz instead of 50 Hz
   ```

2. **Increase internal integrator frequency**: 100 Hz (still)
   - Control updated every 3 internal steps instead of 2

3. **Verify stability**: Run Phase 1 test; ensure no oscillations or instability

**Consequence**: Slightly slower response to disturbances, but still acceptable for Phase 2

**Timeline**: Week 2 (fallback; minimal development)

---

### R3: Escalation Trigger

**When to escalate R3**:
- After Week 1 Phase 1 test, if P95(solver_time) > 25 ms

**Escalation action**:
1. Profile solver time breakdown (setup, solve, extraction)
2. Implement secondary mitigation (SNOPT tuning)
3. Re-measure; if P95 still > 20 ms, implement tertiary (reduce horizon)
4. If P95 still > 20 ms: Accept fallback (lower control frequency)

**Owner**: Algorithm/Drake lead  
**Decision authority**: Project PI  
**Timeline**: Escalate decision by EOD Week 1

---

## Risk R4: Test Execution Overruns Schedule (4 Weeks → 6+ Weeks)

**Risk ID**: R4  
**Category**: Project Management  
**Likelihood**: MEDIUM (35%)  
**Impact**: MEDIUM — Phase 3 delayed by 2 weeks  
**Severity Score**: 35% × 6 = 2.1 / 10

#### Description
Phase 2 is planned as a 4-week sprint (Apr 21 - May 16). Risk: Test execution and analysis takes longer than budgeted, pushing Phase 3 start to late May.

#### Root Causes
1. **Debugging multi-drone wiring** (15% probability)
   - Two-drone test harness has 8 state connections (4 ports × 2 drones)
   - Wiring errors take 1-2 days to diagnose

2. **Data analysis bottleneck** (10% probability)
   - 10 scenarios × 50 Hz × 10-15 seconds = ~5,000-7,500 CSV rows per scenario
   - Analysis script crashes or produces incorrect results → manual inspection needed

3. **Flaky tests** (10% probability)
   - Solver non-determinism; same scenario produces different results on different runs
   - Requires multiple seeds per scenario; adds weeks

---

### R4: Mitigation Plan

#### Primary: Parallelize Execution (Week 1-2)

**What to do**:
1. Run single-drone and 2-drone tests on different machines (if available)
2. Use batch scripting to launch 5 scenarios in parallel
3. Reduce wall-clock time by 3-4x

**Implementation**: Bash script with `&` backgrounding
```bash
# Run scenarios in parallel
for i in 1 5 10 15 20; do
  ./decentralized_mpc_test --scenario $i --output phase2_$i.csv &
done
wait
```

**Timeline**: Week 1 setup (low effort)

---

#### Secondary: Reduce Scenario Count (Accept Tradeoff)

**What to do**:
- Originally planned: 20 scenarios (5 per week)
- Reduced scope: 10 scenarios (Phase 2a minimum)
  - Hover: 5 scenarios
  - Circular motion: 5 scenarios
  - 3-drone and severance moved to Phase 2b (optional)

**Trade-off**: Less comprehensive validation, but still sufficient to demonstrate coordination

**Timeline**: Week 1 (planning phase)

---

#### Fallback: Defer Phase 2b to Later

**Scenario**: End of Week 3, only 10 scenarios complete; 3-drone and severance not started.

**What to do**:
1. **Publish Phase 2a results** (10 scenarios, N=2 drones, nominal operation)
2. **Extend timeline**: Complete Phase 2b (3-drone, severance) in following week
3. **Phase 3 start**: Delayed by 1 week (not catastrophic)

**Advantage**: Doesn't block Phase 2a conclusions; maintains quality

**Timeline**: End of Week 3 decision point

---

## Risk R5: Multi-Drone Test Harness Has Wiring Bugs

**Risk ID**: R5  
**Category**: Testing  
**Likelihood**: MEDIUM (30%)  
**Impact**: MEDIUM — Wasted debugging effort, 3-5 days lost  
**Severity Score**: 30% × 6 = 1.8 / 10

#### Description
Creating `decentralized_mpc_test_2drones.cc` requires wiring:
- 2 plant instances (or 1 plant with 2 quadcopters + 1 payload)
- 2 state extractors
- 2 controllers
- 2 × 4 = 8 input port connections
- 2 × 5 = 10 output port connections

Risk: Port dimensions mismatch, index off-by-one, or wrong connection → simulation crashes or produces garbage data.

---

### R5: Mitigation Plan

#### Primary: Code Review of State Extractor (Week 1)

**What to do**:
1. Implement StateExtractor (Fix 1 of Phase 1 remediation)
2. **Unit test** before using in multi-drone test:
   ```cpp
   // test_state_extractor.cc
   TEST(StateExtractorTest, ExtractsPositionCorrectly) {
     // Create simple plant state: [p_quad, p_payload, ...]
     Eigen::VectorXd plant_state = Eigen::VectorXd::Zero(12);
     plant_state.segment<3>(0) << 1.0, 2.0, 3.0;  // p_quad
     
     // Run StateExtractor
     auto extractor = StateExtractor({...});
     auto p_out = extractor.CalcDronePosition(plant_state);
     
     // Verify
     EXPECT_TRUE(p_out.isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
   }
   ```

3. Verify all 4 outputs (p, v, T, n) independently

**Timeline**: Week 1 (gating task before 2-drone test)

---

#### Secondary: Incremental Wiring & Testing (Week 2)

**What to do**:
1. Build 2-drone test harness step-by-step:
   ```cpp
   // Step 1: Build plant with 2 drones
   // Step 2: Add 1 controller, verify single-drone behavior
   // Step 3: Add 2nd controller (with zero inputs initially)
   // Step 4: Wire state extractors to both controllers
   // Step 5: Run test, check that both controllers' outputs sensible
   ```

2. After each step, run sanity checks:
   ```bash
   ./decentralized_mpc_test_2drones --duration 1.0  # Quick test
   # Check: No NaN, no crashes, CSV columns exist
   ```

**Timeline**: Week 2 (iterative, low risk)

---

#### Fallback: Copy From Single-Drone Test

**Scenario**: 2-drone harness too complex to debug; wasting time.

**What to do**:
1. Revert to single-drone test (already works)
2. **Simulate 2 drones as sequential solver calls**:
   - Step 1: Solve for drone 0 (given fixed drone 1 position)
   - Step 2: Solve for drone 1 (given drone 0's result)
   - Repeat (2× solver time per step, but still <50ms)

3. **Downside**: Not truly parallel; introduces 20ms delay between drones
   - Still validates implicit coordination (just slower response)

**Timeline**: Week 2 fallback (if primary fails)

---

## Risk R6: Cable Severance Not Detected (Phase 4 Preview Fails)

**Risk ID**: R6  
**Category**: Algorithm  
**Likelihood**: LOW (15%)  
**Impact**: MEDIUM — Phase 3 preview incomplete; no fault detection data  
**Severity Score**: 15% × 5 = 0.75 / 10

#### Description
Week 4 includes cable severance test (N=2 drones, one rope severed at t=5s).

Risk: Solver doesn't report infeasibility; optimization succeeds despite constraint violation; no fault detection.

#### Root Cause
Rope tautness constraint may be soft (slack variables), allowing T → negative without explicit detection.

---

### R6: Mitigation Plan

#### Primary: Explicit Infeasibility Detector

**What to do**:
```cpp
// In CalcOptimalControl:
auto result = drake::solvers::Solve(*prog, ...);

if (!result.is_success()) {
  // Solver detected infeasibility (or hit iteration limit)
  std::cout << "FAULT DETECTED: Optimizer infeasible\n";
  fault_detected = true;
  
  // Trigger fallback policy
  // ...
}

// Additional check: validate solution physically
if (result.is_success()) {
  double T_min = result.GetSolution(slack_T).minCoeff();
  if (T_min < -0.5) {  // Negative tension beyond tolerance
    std::cout << "WARNING: Slack suggests negative tension\n";
    fault_detected = true;
  }
}
```

**Timeline**: Week 1 (add to Phase 1 remediation)

---

#### Fallback: Manual Fault Injection (Accept Limited Data)

**Scenario**: Optimizer never detects fault; severance test inconclusive.

**What to do**:
1. Don't rely on optimizer detection
2. **Manually trigger fallback** when rope tension drops below threshold
3. Log the event; analyze recovery trajectory
4. Document: "Fault detection requires explicit threshold monitoring, not optimizer"

---

## Risk R7: SNOPT Solver Not Available (Drake Build Issue)

**Risk ID**: R7  
**Category**: Infrastructure  
**Likelihood**: LOW (10%)  
**Impact**: MEDIUM — Can't run MPC; fall back to quadratic programming  
**Severity Score**: 10% × 5 = 0.5 / 10

#### Description
SNOPT is an optional Drake solver (commercial license). If not installed:
- Phase 1 test doesn't compile (unresolved symbol)
- OR compiles but crashes at runtime (missing .so)

---

### R7: Mitigation Plan

#### Primary: Verify Drake Build (Week 1)

**What to do**:
```bash
# Check if SNOPT available
find $DRAKE_INSTALL_DIR -name "*snopt*"

# Or test programmatically:
#include <drake/solvers/snopt_solver.h>
if (drake::solvers::SnoptSolver::is_available()) {
  // Use SNOPT
} else {
  // Fall back to different solver
}
```

**Timeline**: Week 1 (before Phase 1 coding)

---

#### Fallback: Use Quadratic Programming Solver

**What to do**:
If SNOPT unavailable, use Drake's open-source QP solver:
```cpp
#include <drake/solvers/osqp_solver.h>

// Reformulate objective as quadratic form
// QP: min 0.5*x'Hx + c'x subject to Ax <= b
// This is simpler but less flexible than general nonlinear MPC

auto qp_solver = drake::solvers::OsqpSolver();
auto result = qp_solver.Solve(*prog);
```

**Trade-off**: MPC becomes simpler (linear/quadratic only); may need to remove nonlinear costs

**Timeline**: Week 1 fallback (low effort)

---

## Risk R8: Load Estimator Filter Unstable (NaN Divergence)

**Risk ID**: R8  
**Category**: Algorithm  
**Likelihood**: LOW (8%)  
**Impact**: LOW — Simulation crashes; restart required  
**Severity Score**: 8% × 3 = 0.24 / 10

#### Description
Complementary filter for load position uses numerical differentiation:
```cpp
v_load_est = (z_pos - p_load_prev) / dt;
```

Risk: If z_pos or p_load_prev have numerical errors, v_load_est can blow up → NaN → simulator crashes

---

### R8: Mitigation Plan

#### Primary: Add Bounds Checking

**What to do**:
```cpp
void UpdateLoadEstimate(...) const {
  // ... existing code ...
  
  // Clamp velocity estimates
  double v_max = 5.0;  // m/s max (physical limit)
  v_load_est = v_load_est.cwiseMax(-v_max).cwiseMin(v_max);
  
  // Clamp position estimates
  double z_bounds = 10.0;  // m (reasonable bounds)
  p_load_est = p_load_est.cwiseMax(-z_bounds).cwiseMin(z_bounds);
  
  // Check for NaN
  if (p_load_est.hasNaN() || v_load_est.hasNaN()) {
    // Reset to safe state
    p_load_est = p_load_prev;
    v_load_est = Eigen::Vector3d::Zero();
    std::cerr << "Load estimator reset (NaN detected)\n";
  }
}
```

**Timeline**: Week 1 (add to Phase 1 remediation)

---

## Risk Register Summary Table

| ID | Risk | Likelihood | Impact | Mitigation | Fallback | Owner | Escalate If |
|----|----|-----------|--------|-----------|----------|-------|-----------|
| R1 | Rope slack infeasible | HIGH | CRITICAL | Slack variables + penalty | Lookup table | Algorithm | >5% solve failures |
| R2 | Estimates diverge | MEDIUM | HIGH | Filter tuning + Kalman | Redefine criterion | Validation | |p_L_est| > 0.4m |
| R3 | Solver time > 50ms | MEDIUM | HIGH | Reduce horizon + tune SNOPT | Lower frequency | Algorithm | P95 > 25ms |
| R4 | Schedule overrun | MEDIUM | MEDIUM | Parallelize + reduce scenarios | Defer Phase 2b | PM | End of Week 2 |
| R5 | Wiring bugs | MEDIUM | MEDIUM | Unit test + incremental build | Sequential drones | Testing | Can't build 2-drone |
| R6 | No fault detection | LOW | MEDIUM | Explicit detector | Manual trigger | Algorithm | Severance test fails |
| R7 | SNOPT missing | LOW | MEDIUM | Verify build early | QP solver | Infra | Build fails Week 1 |
| R8 | Filter NaN crash | LOW | LOW | Bounds checking | Reset estimator | Algorithm | Any NaN observed |

---

## Section 4: Escalation Protocol

### Trigger Conditions

| Condition | Severity | Owner | Authority | Deadline |
|-----------|----------|-------|-----------|----------|
| **Any CRITICAL risk manifests** | 🔴 Blocks phase | All | Project PI | Immediate (1 hour) |
| **>1 HIGH risk manifests** | 🟠 Major impact | Algorithm | Drake expert + PI | EOD same day |
| **Solver success < 90%** | 🟠 Major impact | Algorithm | PI | EOD Week 1 |
| **Estimates diverge > 0.4m** | 🟡 Moderate impact | Validation | PI | EOD Week 2 |
| **P95(solver_time) > 25ms** | 🟡 Moderate impact | Algorithm | PI | EOD Week 1 |

---

### Escalation Steps (Template)

1. **Identify**: Risk manifests; log evidence (test output, solver status, etc.)
2. **Notify**: Owner escalates to authority immediately (Slack, email, meeting)
3. **Assess**: Authority reviews root cause (1-2 hour technical analysis)
4. **Decide**: Choose among:
   - **Implement mitigation** (pre-approved, low effort)
   - **Implement fallback** (accepted contingency)
   - **Design review** (fundamentally rethink approach; 1 day)
   - **Pause phase** (only if unresolvable; < 5% likelihood)
5. **Recover**: Resume testing with new approach
6. **Document**: Record decision + outcome in PHASE_2_LOG.md

---

## Section 5: Weekly Risk Review Cadence

**Every Friday (EOD)**:
- Review all R1-R3 metrics from week's tests
- If risk threshold exceeded, trigger escalation immediately
- Update risk register with latest likelihood estimates
- Plan mitigation for following week

**Risk Dashboard** (updated every Friday):
```markdown
# Phase 2 Risk Status — Week X (Apr 28 - May 4)

## Critical Risks
- [ ] R1 (Rope slack): Solver success rate = 97% ✅ (target ≥90%)
- [ ] R3 (Solver time): P95 solve time = 12ms ✅ (target <25ms)

## Monitoring Risks
- [ ] R2 (Estimates diverge): Max error = 0.18m ✅ (target <0.3m)
- [ ] R4 (Schedule): 5 scenarios complete, on pace ✅

## Action Items
- None this week ✅

**Next week focus**: Launch 2-drone test; monitor R5 (wiring bugs)
```

---

## Appendix: Risk Register Template

**For any new risk identified during Phase 2**:

```markdown
### R[N]: [Risk Title]

**Risk ID**: R[N]  
**Category**: Algorithm / Testing / Infrastructure  
**Likelihood**: HIGH / MEDIUM / LOW  
**Impact**: CRITICAL / HIGH / MEDIUM / LOW  
**Severity Score**: Likelihood% × Impact(1-10)  

#### Description
[What can go wrong, why it matters]

#### Root Causes
- [Root cause 1: X% probability]
- [Root cause 2: Y% probability]

#### Detection Signals
- [How to know if risk manifests]

#### Mitigation
1. **Primary**: [First-line fix]
   - Implementation: [Code/process changes]
   - Timeline: [When to implement]
   
2. **Fallback**: [Accepted contingency]
   - Cost/trade-off: [What we give up]
   - Timeline: [When to invoke]

#### Escalation
- **Trigger**: [When to escalate]
- **Owner**: [Who owns the risk]
- **Authority**: [Who decides]
- **Deadline**: [By when]
```

---

## Summary

Phase 2 risks are **well-characterized, mitigated, and manageable**. None are considered **unresolvable**. The register provides:

1. **Transparency**: All risks visible to team + stakeholders
2. **Preparedness**: Mitigations and fallbacks pre-defined
3. **Agility**: Escalation protocol enables rapid decision-making
4. **Accountability**: Clear ownership and authority

**Success probability**: With active risk management, Phase 2 has ~85% chance of completing on schedule with positive results.

