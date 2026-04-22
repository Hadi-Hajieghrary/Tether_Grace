# DRAKE API CORRECTNESS & SYSTEMS ARCHITECTURE AUDIT
## Phase 1 Decentralized Optimal Controller

**Date:** 2026-04-17  
**Scope:** Decentralized Optimal Controller, Phase 1 Test Harness  
**Auditor:** Drake Simulator Expert  
**Status:** CRITICAL ISSUES IDENTIFIED

---

## EXECUTIVE SUMMARY

Phase 1 contains **six critical Drake API correctness issues** and **one foundational systems architecture failure**. The optimizer is a stub with no objectives/constraints, the test harness is open-loop with the controller completely disconnected from plant dynamics, and the load estimator receives only constant zero inputs. These are not Phase 1 omissions—they are active architectural faults that prevent any signal flow verification.

**Critical Finding:** The test cannot validate controller correctness because the controller reads constant zeros (drone position, velocity, tension) while the plant evolves independently. The controller output (optimal control) is computed but never applied to the plant. This is not a feature-incomplete Phase 1; it's an open-loop stub.

---

## ISSUE 1: CONTEXT SEMANTICS VIOLATION

**Severity:** HIGH  
**File:** `/workspaces/Tether_Grace/Research/cpp/src/decentralized_optimal_controller.cc`  
**Lines:** 68–152 (CalcOptimalControl method)

### The Problem

```cpp
void DecentralizedOptimalController::CalcOptimalControl(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* control_output) const {
  // Line 87: Modifying MUTABLE member state during CalcOutput
  UpdateLoadEstimate(p_i.segment<3>(0), v_i.segment<3>(0), n_i, dt);
```

The method is declared `const` but modifies three mutable member variables:
- `p_load_est_` (line 258 in UpdateLoadEstimate)
- `v_load_est_` (line 262 in UpdateLoadEstimate)
- `p_load_prev_` (line 264 in UpdateLoadEstimate)
- `solver_failures_` (line 108, 135)
- `solver_time_ms_` (line 127)
- `solver_iterations_` (not shown, but declared mutable at line 184)

**Drake Framework Expectation:**
- `CalcOutput` callbacks must be **pure**: no side effects on system state
- Mutable members violate this contract and can cause:
  - Non-deterministic recomputation if Drake caches are invalidated
  - Incorrect behavior when context is cloned/branched
  - Race conditions in parallel sweeps or Monte Carlo simulations
  - Violation of dependency tracking (Drake's topology layer assumes pure outputs)

**Current Code (WRONG):**
```cpp
void DecentralizedOptimalController::CalcOptimalControl(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* control_output) const {
  // ... read inputs ...
  UpdateLoadEstimate(p_i.segment<3>(0), v_i.segment<3>(0), n_i, dt);  // MODIFIES MUTABLE STATE
  // ...
  solver_failures_++;  // MODIFIES MUTABLE STATE
  solver_time_ms_ = duration;  // MODIFIES MUTABLE STATE
}
```

**Why This Breaks Drake's Guarantees:**
1. If Drake internally caches output values, the mutable state changes will not be reflected
2. If CalcOutput is called multiple times for the same context (e.g., during sensitivity analysis), the second call sees corrupted state
3. Discrete-event simulation may skip calls to CalcOutput if deemed "unchanged," but mutable state will still be modified
4. The load estimator state (`p_load_est_`, `v_load_est_`) should be in **discrete state** (xd) or **continuous state** (xc), not mutable members

### Root Cause

The controller conflates three distinct concerns:
1. **Load estimator state** → Should be a **discrete state variable** declared via `DeclareDiscreteState()`
2. **Solver diagnostics** → Should be **Context parameters** or output-only (no side effects in CalcOutput)
3. **Optimization logic** → Must be **pure** (read inputs, produce outputs, no mutations)

---

## ISSUE 2: MATHEMATICAL PROGRAM IS A FEASIBILITY STUB

**Severity:** CRITICAL  
**File:** `/workspaces/Tether_Grace/Research/cpp/src/decentralized_optimal_controller.cc`  
**Lines:** 178–227 (SetupOptimizationProblem method)

### The Problem

```cpp
std::unique_ptr<drake::solvers::MathematicalProgram>
DecentralizedOptimalController::SetupOptimizationProblem(...) const {
  auto prog = std::make_unique<drake::solvers::MathematicalProgram>();
  int N = static_cast<int>(config_.control_horizon_sec / config_.control_dt);
  
  // Decision variables: U = [u_0, u_1, ..., u_{N-1}]
  auto U = prog->NewContinuousVariables(4, N, "U");
  auto slack_tension = prog->NewContinuousVariables(N, "slack_T");
  
  // ============ OBJECTIVE FUNCTION ============
  // Phase 1: Simple feasibility (no explicit cost function)
  // The optimizer will find feasible solutions respecting the bounds below.
  
  // ============ CONSTRAINTS ============
  prog->AddBoundingBoxConstraint(0, config_.thrust_max, U.row(0));
  prog->AddBoundingBoxConstraint(-config_.torque_max, config_.torque_max, U.row(1));
  // ... (torque bound constraints)
  
  return prog;  // RETURNS PROGRAM WITH NO OBJECTIVES, MISSING DYNAMICS
}
```

**Critical Omissions:**

1. **No Objective Function**
   - The program has **zero cost functions** (line 203 comment confirms this is intentional)
   - The solver has no reason to prefer one feasible solution over another
   - Result: SNOPT returns an arbitrary point in the feasible region (likely the initial point)

2. **Missing Dynamics Constraints**
   - No constraints on load position (p_load trajectory)
   - No constraints on load velocity (v_load trajectory)
   - No discretized dynamics: `p_L[k+1] = f(p_L[k], v_L[k], T_i[k], ...)`
   - No integration method (Euler, RK4, etc.)

3. **Missing Rope Tautness Constraint**
   - Comment at line 220: "Full rope tautness constraints added in Phase 2"
   - Rope constraint is critical: `||p_i - p_L|| = L` (for rigid rope) or `||p_i - p_L|| ≤ L` (slack rope)
   - Without this, the optimizer doesn't know the rope exists

4. **Unused Decision Variables**
   - `slack_tension` is declared but never used
   - `U` decision variables are never extracted or applied to plant

5. **No Warm-Start or Initial Guess**
   - SNOPT initialized with `std::nullopt` (line 124) → cold start
   - No initial guess for control trajectory
   - Longer solve times, suboptimal convergence

### Solver Invocation Issue

**Line 124:**
```cpp
auto result = drake::solvers::Solve(*prog, std::nullopt, std::make_optional(solver_options));
```

**Drake Solve() Signature (correct usage):**
```cpp
template <typename C>
MathematicalProgramResult Solve(
    const MathematicalProgram& prog,
    const std::optional<Eigen::VectorXd>& initial_guess = std::nullopt,
    const std::optional<SolverOptions>& solver_options = std::nullopt);
```

**Current Code:** Correct signature, but **semantically empty** program.

### Consequence

When the test runs (line 124–142 of test_main.cc):
```cpp
auto result = drake::solvers::Solve(*prog, std::nullopt, std::make_optional(solver_options));

if (!result.is_success()) {
  solver_failures_++;
  Vector4d u_fallback = Vector4d::Zero();
  u_fallback(0) = config_.thrust_max * 0.5;  // Fallback: 50% hover thrust
  control_output->SetFromVector(u_fallback);
  return;
}

// Extract optimal control (receding horizon: take first step)
Vector4d u_optimal = Vector4d::Zero();
u_optimal(0) = config_.thrust_max * 0.5;  // Nominal hover thrust
control_output->SetFromVector(u_optimal);
```

**Result:** Whether solver succeeds or fails, output is always `[thrust=0.5×F_max, 0, 0, 0]` (nominal hover). The optimization result is **never examined or used**.

---

## ISSUE 3: SOLVER OPTION CONFIGURATION INCOMPLETE

**Severity:** MEDIUM  
**File:** `/workspaces/Tether_Grace/Research/cpp/src/decentralized_optimal_controller.cc`  
**Lines:** 116–122

### The Problem

```cpp
drake::solvers::SolverOptions solver_options;
solver_options.SetOption(drake::solvers::SnoptSolver::id(),
                        "Major iterations limit",
                        config_.max_solver_iterations);
solver_options.SetOption(drake::solvers::SnoptSolver::id(),
                        "Print frequency",
                        config_.verbose_solver ? 1 : 0);
```

**Missing Critical Options:**

1. **Feasibility Tolerance** (default 1e-6)
   - Current: not set
   - Should be: ≥ 1e-5 (for this MPC problem with tight rope constraints)

2. **Optimality Tolerance** (default 1e-6)
   - Current: not set
   - Meaningless when no objective function exists

3. **Warm-Start Flag** (SNOPT-specific)
   - Current: not set
   - Needed when reusing solutions across horizon steps
   - Set via: `solver_options.SetOption(SnoptSolver::id(), "Warm start", 1);`

4. **Time Limit** (SNOPT "Time limit" option)
   - Current: no time limit
   - For 50 Hz MPC (20 ms deadline), should set ≤ 15 ms
   - Prevents solver from consuming an entire control cycle

5. **Constraint Violation (cvxpy/Ipopt style)**
   - SNOPT has no direct "solver_tolerance" equivalent
   - The `config_.solver_tolerance` (line 52) is declared but **never used**

### Consequence

- Solver may not converge in 20 ms (MPC period), violating real-time guarantees
- No constraint on solution accuracy
- Solver has no guidance on time budget

---

## ISSUE 4: PORT DECLARATION & WIRING CORRECTNESS

**Severity:** HIGH  
**File:** `/workspaces/Tether_Grace/Research/cpp/src/decentralized_optimal_controller.cc` (lines 29–53)  
**File:** `/workspaces/Tether_Grace/Research/cpp/src/decentralized_mpc_test_main.cc` (lines 254–283)

### Input Port Declarations (CORRECT)

```cpp
p_drone_port_ = DeclareVectorInputPort("p_drone", BasicVector<double>(3)).get_index();
v_drone_port_ = DeclareVectorInputPort("v_drone", BasicVector<double>(3)).get_index();
T_cable_port_ = DeclareVectorInputPort("T_cable", BasicVector<double>(1)).get_index();
n_cable_port_ = DeclareVectorInputPort("n_cable", BasicVector<double>(3)).get_index();
p_ref_port_ = DeclareVectorInputPort("p_ref", BasicVector<double>(3)).get_index();
v_ref_port_ = DeclareVectorInputPort("v_ref", BasicVector<double>(3)).get_index();
T_others_port_ = DeclareVectorInputPort("T_others", BasicVector<double>(4)).get_index();
m_load_port_ = DeclareVectorInputPort("m_load", BasicVector<double>(1)).get_index();
```

**Analysis:**
- ✓ Port 0: p_drone → size 3 ✓
- ✓ Port 1: v_drone → size 3 ✓
- ✓ Port 2: T_cable → size 1 ✓
- ✓ Port 3: n_cable → size 3 ✓
- ✓ Port 4: p_ref → size 3 ✓
- ✓ Port 5: v_ref → size 3 ✓
- ✓ Port 6: T_others → size 4 ✓
- ✓ Port 7: m_load → size 1 ✓

### Output Port Declarations (CORRECT)

```cpp
DeclareVectorOutputPort("u_optimal", 4, &DecentralizedOptimalController::CalcOptimalControl);
DeclareVectorOutputPort("p_load_est", 3, &DecentralizedOptimalController::CalcLoadPositionEstimate);
DeclareVectorOutputPort("v_load_est", 3, &DecentralizedOptimalController::CalcLoadVelocityEstimate);
DeclareVectorOutputPort("solver_time_ms", 1, &DecentralizedOptimalController::CalcSolverTime);
DeclareVectorOutputPort("solver_status", 1, &DecentralizedOptimalController::CalcSolverStatus);
```

**Analysis:**
- ✓ Port 0: u_optimal → size 4 ✓
- ✓ Port 1: p_load_est → size 3 ✓
- ✓ Port 2: v_load_est → size 3 ✓
- ✓ Port 3: solver_time_ms → size 1 ✓
- ✓ Port 4: solver_status → size 1 ✓

### Test Harness Wiring (BROKEN - CRITICAL)

**Actual connections in test_main.cc:**

```cpp
// Lines 276–283: Controller inputs wired to CONSTANT ZEROS
auto* zero_pos = builder.AddSystem<ConstantVectorSource<double>>(Eigen::VectorXd::Zero(3));
auto* zero_vel = builder.AddSystem<ConstantVectorSource<double>>(Eigen::VectorXd::Zero(3));
Eigen::Matrix<double, 1, 1> zero_t(0.0);
auto* zero_tension = builder.AddSystem<ConstantVectorSource<double>>(zero_t);
auto* zero_direction = builder.AddSystem<ConstantVectorSource<double>>(
    Eigen::Vector3d(0.0, 0.0, 1.0));

builder.Connect(zero_pos->get_output_port(),
                controller->get_drone_position_input_port());
builder.Connect(zero_vel->get_output_port(),
                controller->get_drone_velocity_input_port());
builder.Connect(zero_tension->get_output_port(),
                controller->get_cable_tension_input_port());
builder.Connect(zero_direction->get_output_port(),
                controller->get_cable_direction_input_port());
```

**Size matching verification:**
- ✓ zero_pos (size 3) → p_drone (size 3) ✓
- ✓ zero_vel (size 3) → v_drone (size 3) ✓
- ✓ zero_tension (size 1) → T_cable (size 1) ✓
- ✓ zero_direction (size 3) → n_cable (size 3) ✓
- ✓ ref_pos (size 3) → p_ref (size 3) ✓
- ✓ ref_vel (size 3) → v_ref (size 3) ✓
- ✓ T_others_source (size 4) → T_others (size 4) ✓
- ✓ m_load_source (size 1) → m_load (size 1) ✓

**Verdict:** Sizes match correctly. **However, data flow is fundamentally broken.**

### The Critical Missing Connection

**What should happen (closed-loop):**
```cpp
// Extract plant state (position of quad body + rope/load)
// Connect plant output to controller input
builder.Connect(plant->get_state_output_port(),
                state_extractor->get_input_port());
builder.Connect(state_extractor->get_drone_position_output_port(),
                controller->get_drone_position_input_port());
builder.Connect(state_extractor->get_drone_velocity_output_port(),
                controller->get_drone_velocity_input_port());
builder.Connect(rope_forces->get_top_segment_tension_output_port(),
                controller->get_cable_tension_input_port());
// ... etc
```

**What actually happens (open-loop):**
```cpp
// Lines 194–197: Rope forces connected to plant ✓
builder.Connect(plant->get_state_output_port(), rope_forces->get_plant_state_input_port());
builder.Connect(rope_forces->get_forces_output_port(), plant->get_applied_spatial_force_input_port());

// Lines 276–283: Controller receives constant zeros ✗
builder.Connect(zero_pos->get_output_port(), controller->get_drone_position_input_port());
builder.Connect(zero_vel->get_output_port(), controller->get_drone_velocity_input_port());
builder.Connect(zero_tension->get_output_port(), controller->get_cable_tension_input_port());

// Lines 293–304: Controller output LOGGED but NEVER APPLIED to plant ✗
builder.Connect(controller->get_optimal_control_output_port(), control_logger->get_input_port());
// NO CONNECTION FROM CONTROLLER OUTPUT TO PLANT INPUT
```

---

## ISSUE 5: MULTIBODYPLANT STATE INITIALIZATION

**Severity:** MEDIUM  
**File:** `/workspaces/Tether_Grace/Research/cpp/src/decentralized_mpc_test_main.cc`  
**Lines:** 320–333

### The Problem

```cpp
auto& plant_context = diagram->GetMutableSubsystemContext(*plant, &context);

// Quadcopter initial position: [0, 0, 1.5] m
Eigen::VectorXd x0 = plant->GetPositions(plant_context);
x0.segment<3>(0) = Eigen::Vector3d(0.0, 0.0, 1.5);  // p_quad
x0.segment<3>(3) = Eigen::Vector3d(0.0, 0.0, 0.0);  // v_quad (WRONG)
// ... (payload and rope beads at rest below quad)

plant->SetPositions(&plant_context, x0);
plant->SetVelocities(&plant_context, Eigen::VectorXd::Zero(plant->num_velocities()));
```

**Issue 1: Segment Indexing Error**

The code assumes:
- `x0.segment<3>(0)` = position of quad (indices 0–2)
- `x0.segment<3>(3)` = velocity of quad (indices 3–5)

**But `GetPositions()` returns ONLY positions**, not velocity!

**Drake State Ordering (for a plant with multiple bodies):**
```
GetPositions() → [p_quad (3), p_payload (3), p_bead[0] (3), ..., p_bead[7] (3)]
                  (26 total position DOFs)

GetVelocities() → [v_quad (3), v_payload (3), v_bead[0] (3), ..., v_bead[7] (3)]
                  (26 total velocity DOFs)
```

**Actual plant configuration (from test_main.cc lines 94–155):**
- 1 quadcopter (3 DOF position: x, y, z)
- 1 payload (3 DOF position: x, y, z)
- 8 rope beads (3 DOF position each: 24 total)
- Total positions: 30 DOF

**Correct initialization:**
```cpp
Eigen::VectorXd q = plant->GetPositions(plant_context);  // 30 DOF
Eigen::VectorXd v = plant->GetVelocities(plant_context);  // 30 DOF

// Set quadcopter position [0, 0, 1.5]
q.segment<3>(0) = Eigen::Vector3d(0.0, 0.0, 1.5);

// Set payload position [0, 0, 0.5] (1.0 m below quad)
q.segment<3>(3) = Eigen::Vector3d(0.0, 0.0, 0.5);

// Set rope beads in vertical line
for (int i = 0; i < 8; ++i) {
  double z_bead = 1.5 - (i + 1) * (1.0 / 8.0);  // Interpolate between quad and payload
  q.segment<3>(6 + 3*i) = Eigen::Vector3d(0.0, 0.0, z_bead);
}

// All velocities zero
v.setZero();

plant->SetPositions(&plant_context, q);
plant->SetVelocities(&plant_context, v);
```

**Issue 2: Line 327 Comment Contradicts Line 332**

Line 327 comment: "// v_quad (zero initially)"  
Line 332: `SetVelocities(&plant_context, Eigen::VectorXd::Zero(...))`  

The comment suggests manual velocity assignment, but line 332 sets all velocities to zero anyway. Redundant but not incorrect.

---

## ISSUE 6: ROPE FORCE SYSTEM INPUT/OUTPUT INTEGRATION

**Severity:** MEDIUM (design, not API misuse)  
**File:** `/workspaces/Tether_Grace/Research/cpp/src/decentralized_mpc_test_main.cc`  
**Lines:** 181–198

### The Problem

```cpp
auto* rope_forces = builder.AddSystem<RopeForceSystem>(
    *plant,
    quad_body, payload_body, bead_ptrs,
    Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
    rope_length,
    200.0, 15.0, 1e-9);

// Connect rope forces to plant
builder.Connect(plant->get_state_output_port(),
                rope_forces->get_plant_state_input_port());
builder.Connect(rope_forces->get_forces_output_port(),
                plant->get_applied_spatial_force_input_port());
```

**RopeForceSystem expects:**
- Input: plant state (positions + velocities)
- Output: vector of `ExternallyAppliedSpatialForce<double>` (abstract type)

**Verification (from rope_force_system.h, lines 49–56):**
```cpp
const InputPort<double>& get_plant_state_input_port() const {
  return get_input_port(plant_state_port_);  // size = num_pos + num_vel = 52
}

const OutputPort<double>& get_forces_output_port() const {
  return get_output_port(forces_port_);  // Abstract type: vector<ExternallyAppliedSpatialForce<double>>
}

const OutputPort<double>& get_tension_output_port() const {
  return get_output_port(tension_port_);  // size 4: [tension, fx, fy, fz]
}
```

**Issue: Tension Port Never Used**

The rope force system computes `get_tension_output_port()` (size 4: magnitude + 3D force vector), but it is **never connected to the controller**:

**Expected (for controller to read cable tension/direction):**
```cpp
// Extract rope tension from rope force system
builder.Connect(rope_forces->get_tension_output_port(),
                controller->get_cable_tension_input_port());
```

**Actual:** The controller's `T_cable_port_` and `n_cable_port_` are fed constant zeros.

**Consequence:** The controller has no awareness of rope tension or direction, so it cannot:
- Compute load acceleration from rope forces
- Detect rope slack/tautness
- Respond to disturbances transmitted through the rope

---

## ISSUE 7: SIGNAL FLOW ARCHITECTURE FAILURE

**Severity:** CRITICAL  
**Scope:** Entire test_main.cc

### Current Architecture (Open-Loop)

```
┌─────────────────────────────────────────────────────┐
│                  DIAGRAM                            │
├─────────────────────────────────────────────────────┤
│                                                      │
│  ConstantVectorSource (zeros)                       │
│  │ [0, 0, 0]                                        │
│  └──→ controller.get_drone_position_input_port()    │
│                                                      │
│  ConstantVectorSource (zeros)                       │
│  │ [0, 0, 0]                                        │
│  └──→ controller.get_drone_velocity_input_port()    │
│                                                      │
│  ConstantVectorSource (0)                           │
│  │ [0]                                              │
│  └──→ controller.get_cable_tension_input_port()     │
│                                                      │
│  ConstantVectorSource ([0, 0, 1])                   │
│  │                                                  │
│  └──→ controller.get_cable_direction_input_port()   │
│                                                      │
│  ConstantVectorSource ([0, 0, 2])                   │
│  │                                                  │
│  └──→ controller.get_reference_trajectory_input()   │
│                                                      │
│  ┌─────────────────────────────────────┐            │
│  │  DecentralizedOptimalController     │            │
│  │ (reads constant zeros, solves MPC)  │            │
│  │ Output: u = [0.5*F_max, 0, 0, 0]   │            │
│  └─────────────────────────────────────┘            │
│         │                                            │
│         │ → VectorLogSink (logs, then drops)        │
│         ✗ (DISCONNECTED FROM PLANT)                 │
│                                                      │
│  ┌─────────────────────────────────────┐            │
│  │  MultibodyPlant (EVOLVING FREELY)   │            │
│  │ state[0] = plant.continuous_time()  │            │
│  │  quad evolves under gravity + rope  │            │
│  │  payload hangs below rope            │            │
│  └─────────────────────────────────────┘            │
│         │                                            │
│         │ state → RopeForceSystem                    │
│         │         (computes rope tensions)           │
│         │                                            │
│         └──→ forces → plant.get_applied_input()      │
│                                                      │
│         plant state → VectorLogSink (logged)         │
│                                                      │
└─────────────────────────────────────────────────────┘
```

**Data Flow Analysis:**

| Component | Input | Output | Connected? |
|-----------|-------|--------|-----------|
| Plant | gravity, rope forces | state (pos, vel) | ✓ (forces) |
| RopeForceSystem | plant state | forces, tension | ✓ (forces) |
| Controller | const zeros | optimal control | ✗ (output logged, never used) |
| Loggers | plant state, control output | CSV files | ✓ (logs only) |

**Critical Gaps:**

1. **Plant state ↛ Controller inputs**
   - Plant evolves: quad + payload + rope
   - Controller reads: [0, 0, 0] always
   - Result: **Controller has no knowledge of actual system state**

2. **Controller output ↛ Plant inputs**
   - Controller computes: u_opt = [0.5×F_max, 0, 0, 0]
   - Plant receives: only rope forces, no thrust/torque from controller
   - Result: **Controller has no effect on dynamics**

3. **Rope tension ↛ Controller**
   - RopeForceSystem computes: T_rope, n_rope, F_rope
   - Controller reads: T_cable = [0], n_cable = [0, 0, 1]
   - Result: **Controller cannot sense rope force feedback**

### Why This Is Not a "Phase 1 Limitation"

The code comment (line 265):
```cpp
// For Phase 1: use zero inputs for drone state and cable measurements
// (Full state extraction will be added in Phase 2 with ESKF feedback)
```

This is misleading. Phase 1 should still have:
1. **A state extractor** (even if trivial): extract quad position/velocity from plant state
2. **Closed-loop wiring**: at minimum, read actual plant state
3. **A basic controller law**: even if just proportional, apply output to plant
4. **Loop closure verification**: demonstrate that plant state propagates through controller

Currently, the test cannot distinguish between:
- A broken controller
- A working controller with inverted gains
- A missing controller entirely

---

## ISSUE 8: LOAD ESTIMATOR RECEIVES CONSTANT ZERO INPUTS

**Severity:** MEDIUM  
**File:** `/workspaces/Tether_Grace/Research/cpp/src/decentralized_optimal_controller.cc`  
**Lines:** 246–265 (UpdateLoadEstimate method)

### The Problem

```cpp
void DecentralizedOptimalController::UpdateLoadEstimate(
    const Vector3d& p_i,          // ← receives [0, 0, 0] (constant)
    const Vector3d& v_i,          // ← receives [0, 0, 0] (constant)
    const Vector3d& n_measured,   // ← receives [0, 0, 1] (constant)
    double dt) const {

  // Measurement: z_pos = p_i - L*n
  Vector3d z_pos = p_i - config_.rope_length * n_measured;
  // z_pos = [0, 0, 0] - 1.0*[0, 0, 1] = [0, 0, -1.0]

  // Low-pass filter: p_load = α*z_pos + (1-α)*p_load_prev
  p_load_est_ = alpha * z_pos + (1.0 - alpha) * p_load_est_;
  // p_load_est_ → constant [0, 0, -1.0]

  // Velocity: numerical differentiation
  Vector3d v_measured = (z_pos - p_load_prev_) / dt;
  // v_measured = ([0, 0, -1.0] - [0, 0, -1.0]) / dt = [0, 0, 0]

  v_load_est_ = alpha * v_measured + (1.0 - alpha) * v_load_est_;
  // v_load_est_ → constant [0, 0, 0]

  p_load_prev_ = z_pos;
}
```

**Actual Behavior:**

After initialization (dt=0.01, alpha=0.1):

```
Iteration 1: p_load_est = 0.1*[0,0,-1] + 0.9*[0,0,0] = [0, 0, -0.1]
Iteration 2: p_load_est = 0.1*[0,0,-1] + 0.9*[0,0,-0.1] = [0, 0, -0.19]
Iteration 3: p_load_est = 0.1*[0,0,-1] + 0.9*[0,0,-0.19] = [0, 0, -0.271]
...
Steady-state: p_load_est → [0, 0, -1.0]

v_load_est → [0, 0, 0]  (always zero, since z_pos never changes)
```

**Why This Matters:**

The load estimator's purpose is to track load state from noisy measurements. With constant input, it converges to a constant estimate and never validates:
1. Filter stability (does it suppress noise without excessive lag?)
2. Load dynamics response (does velocity estimate track acceleration?)
3. Integration with controller (does control action change the estimate?)

**Test Coverage Gap:**

The test cannot verify that the load estimator responds to:
- Sudden rope tension changes
- Load swinging
- Controller thrust changes affecting load acceleration

---

## REMEDIATION ROADMAP

### Phase 1 Hotfixes (Low-Risk, Immediate Impact)

#### Fix 1A: Move Load Estimator State to Discrete State (CRITICAL)

**Current (WRONG):**
```cpp
mutable Eigen::Vector3d p_load_est_;
mutable Eigen::Vector3d v_load_est_;

void CalcOptimalControl(...) const {
  UpdateLoadEstimate(...);  // Modifies mutable state in CalcOutput
}
```

**Corrected:**
```cpp
// In DecentralizedOptimalController.h, replace lines 177–179:
// DELETE: mutable Eigen::Vector3d p_load_est_;
//         mutable Eigen::Vector3d v_load_est_;
//         mutable Eigen::Vector3d p_load_prev_;

// In DecentralizedOptimalController::DecentralizedOptimalController() constructor,
// after port declarations (line 65), ADD:

// Declare discrete state for load estimator
// xd = [p_load_x, p_load_y, p_load_z, v_load_x, v_load_y, v_load_z, p_prev_x, p_prev_y, p_prev_z]
auto& load_est_state = DeclareDiscreteState(9);  // 9 DOF

// Declare update callback (runs at config_.mpc_period)
DeclarePeriodicDiscreteUpdateEvent(
    config_.mpc_period,
    0.0,  // offset
    &DecentralizedOptimalController::UpdateLoadEstimateDiscrete);
```

**New discrete update method:**
```cpp
void DecentralizedOptimalController::UpdateLoadEstimateDiscrete(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  
  const auto& inputs = context.get_discrete_state_input_port().get_vector_data();
  
  // Read inputs (now must actually come from somewhere)
  Vector3d p_i = inputs.segment<3>(0);
  Vector3d n_measured = inputs.segment<3>(3);
  
  // Get current state
  auto& xd = discrete_state->get_mutable_vector(0);
  Vector3d p_load_est = xd.segment<3>(0);
  Vector3d v_load_est = xd.segment<3>(3);
  Vector3d p_load_prev = xd.segment<3>(6);
  
  // Update (complementary filter)
  double dt = config_.mpc_period;
  double alpha = config_.load_estimator_filter_alpha;
  
  Vector3d z_pos = p_i - config_.rope_length * n_measured;
  Vector3d p_load_new = alpha * z_pos + (1.0 - alpha) * p_load_est;
  Vector3d v_load_new = alpha * (z_pos - p_load_prev) / dt + (1.0 - alpha) * v_load_est;
  
  // Write updated state
  xd.segment<3>(0) = p_load_new;
  xd.segment<3>(3) = v_load_new;
  xd.segment<3>(6) = z_pos;
}
```

**Impact:** CalcOptimalControl becomes pure; state updates happen deterministically at discrete intervals.

---

#### Fix 1B: Connect Plant State to Controller Inputs (CRITICAL)

**Current (WRONG):**
```cpp
// Lines 266–283 in test_main.cc
auto* zero_pos = builder.AddSystem<ConstantVectorSource<double>>(
    Eigen::VectorXd::Zero(3));
builder.Connect(zero_pos->get_output_port(),
                controller->get_drone_position_input_port());
```

**Corrected:**

Create a state extractor system:

```cpp
// Create state extractor to pull quad/rope state from plant
class QuadStateExtractor : public LeafSystem<double> {
 public:
  QuadStateExtractor(const MultibodyPlant<double>& plant) : plant_(plant) {
    // Input: plant state
    DeclareVectorInputPort("plant_state",
        plant.num_positions() + plant.num_velocities());
    
    // Outputs: quad position, velocity; rope tension; rope direction
    DeclareVectorOutputPort("p_quad", 3, &QuadStateExtractor::CalcQuadPosition);
    DeclareVectorOutputPort("v_quad", 3, &QuadStateExtractor::CalcQuadVelocity);
    DeclareVectorOutputPort("T_rope", 1, &QuadStateExtractor::CalcRopeTension);
    DeclareVectorOutputPort("n_rope", 3, &QuadStateExtractor::CalcRopeDirection);
  }
  
 private:
  void CalcQuadPosition(const Context<double>& context, 
                       BasicVector<double>* output) const {
    const auto& state = get_input_port().Eval<VectorXd>(context);
    // Quad position = state.segment<3>(0)
    output->SetFromVector(state.segment<3>(0));
  }
  
  // ... similar for velocity
  // (T_rope and n_rope: read from rope_forces system)
  
  const MultibodyPlant<double>& plant_;
};

// In test_main.cc, replace lines 266–283 with:
auto* state_extractor = builder.AddSystem<QuadStateExtractor>(*plant);

builder.Connect(plant->get_state_output_port(),
                state_extractor->get_input_port(0));
builder.Connect(state_extractor->get_output_port("p_quad"),
                controller->get_drone_position_input_port());
builder.Connect(state_extractor->get_output_port("v_quad"),
                controller->get_drone_velocity_input_port());
builder.Connect(rope_forces->get_tension_output_port(),
                controller->get_cable_tension_input_port());
// ... etc
```

**Impact:** Controller now reads actual plant state; feedback loop is closed.

---

#### Fix 2: Add Objectives and Dynamics Constraints to MPC (HIGH)

**Current (STUB):**
```cpp
// Lines 202–226 in decentralized_optimal_controller.cc
// ============ OBJECTIVE FUNCTION ============
// Phase 1: Simple feasibility (no explicit cost function)

// ============ CONSTRAINTS ============
prog->AddBoundingBoxConstraint(0, config_.thrust_max, U.row(0));
// ... (only bound constraints)
```

**Minimal Phase 1 Addition (Feasibility Focus):**

```cpp
// Add simple tracking cost: minimize || u - u_hover ||^2
Vector4d u_hover = Vector4d::Zero();
u_hover(0) = config_.load_mass_nominal * 9.81;  // Hover thrust

for (int k = 0; k < N; ++k) {
  // Add quadratic cost: || U[k] - u_hover ||^2
  Eigen::Vector4d u_k = U.col(k);
  prog->AddQuadraticCost(
      config_.w_effort * Eigen::MatrixXd::Identity(4, 4),
      -2.0 * config_.w_effort * u_hover,
      u_k);
}

// Add simple load position constraint: keep load above ground
// p_load[k] = p_i[k] - L*n[k]
// Constraint: z >= 0.1 (avoid ground)
// (Simplified: assume vertical rope, n[k] = [0, 0, 1])
for (int k = 0; k < N; ++k) {
  prog->AddBoundingBoxConstraint(0.1, 10.0, U.row(0)(k));  // Thrust ∈ [0.1×g×m, 10×g×m]
}
```

**Impact:** Optimizer now has objective and simple constraint; can verify cost minimization.

---

#### Fix 3: Set Solver Time Limit (MEDIUM)

**Current (WRONG):**
```cpp
solver_options.SetOption(drake::solvers::SnoptSolver::id(),
                        "Major iterations limit",
                        config_.max_solver_iterations);
```

**Corrected:**
```cpp
// Add time limit for real-time constraint (20 ms cycle, allocate 15 ms)
solver_options.SetOption(drake::solvers::SnoptSolver::id(),
                        "Time limit",
                        15.0);  // seconds (actually milliseconds in SNOPT)

// Set feasibility tolerance
solver_options.SetOption(drake::solvers::SnoptSolver::id(),
                        "Feasibility tolerance",
                        1e-5);
```

**Impact:** Solver respects real-time deadline; prevents MPC from blocking.

---

### Phase 2 Refactoring (Medium-Risk, Proper Architecture)

#### Refactor 1: Rope Constraint in MPC

Add symbolic rope constraint to MPC:
```cpp
// Decision variables: load trajectory p_L[k], rope tension T[k]
auto p_L = prog->NewContinuousVariables(3, N+1, "p_load");
auto T = prog->NewContinuousVariables(N, "T");

// Rope constraint: ||p_i[k] - p_L[k]|| = L (hard constraint)
// or: ||p_i[k] - p_L[k]|| <= L (soft, with slack)
for (int k = 0; k < N; ++k) {
  // Simplified: assume rope is taut and vertical
  prog->AddConstraint(
      p_L(2, k) == p_i(2) - config_.rope_length);
}
```

#### Refactor 2: Load Dynamics Prediction

Add discretized load dynamics:
```cpp
// p_L[k+1] = p_L[k] + v_L[k] * dt
// v_L[k+1] = v_L[k] + a_L[k] * dt
// where a_L[k] = (T[k] - m_L * g) / m_L (vertical motion, simplified)

for (int k = 0; k < N-1; ++k) {
  prog->AddLinearEqualityConstraint(
      p_L.col(k+1) == p_L.col(k) + v_L.col(k) * config_.control_dt);
  
  prog->AddLinearEqualityConstraint(
      v_L.col(k+1) == v_L.col(k) + a_L.col(k) * config_.control_dt);
}
```

---

### Phase 3+ Long-Term (Major Refactoring)

#### Architectural Improvements

1. **Separate Estimation and Control:**
   - Move load estimator to its own system (`LoadEstimatorSystem`)
   - Use discrete-time Kalman filter (ESKF) for nonlinear dynamics
   - Controller reads estimates from estimator output, not mutable state

2. **Controller Loop Restructuring:**
   - Use `AffineSystem` or `TimeVaryingAffineSystem` for linear approximations
   - Implement `ZeroOrderHold` for 50 Hz MPC updates (currently implicit)
   - Add state feedback + feedforward terms

3. **Rope Constraint Formulation:**
   - Use complementary constraints: either rope is taut (T > 0) or slack (||p_i - p_L|| < L)
   - Add cable collision detection with ground
   - Model rope as series of spring-damper segments (already done in RopeForceSystem)

4. **Multi-Drone Coordination:**
   - Add distributed optimization: each drone solves locally, exchanges tensions with neighbors
   - Add consensus constraint on load position: all drones agree on estimated p_L
   - Implement decentralized receding horizon control (DRHC)

---

## SUMMARY TABLE: ISSUES & FIXES

| Issue | Severity | Component | Root Cause | Phase 1 Fix | Phase 2+ Solution |
|-------|----------|-----------|-----------|------------|------------------|
| Context Semantics (mutable in Calc) | HIGH | LeafSystem | Conflated concerns (state, diagnostics, logic) | Move state to DiscreteState | Estimator as separate system |
| MPC is feasibility stub | CRITICAL | MathematicalProgram | No objectives, no dynamics | Add quadratic cost + bound constraints | Full nonlinear MPC with rope constraint |
| Solver options incomplete | MEDIUM | Solver | Missing time limit, feasibility tolerance | Add time/feasibility limits | Warm-start, warm-restart across horizon |
| Port wiring broken (zeros) | CRITICAL | Signal flow | Test harness isolation | Add state extractor, connect plant → controller | Feedback with state observer |
| Controller output unused | CRITICAL | Signal flow | No wiring to plant | Add actuator system, wire control → plant | Thrust allocator + attitude controller |
| Plant IC initialization | MEDIUM | MultibodyPlant | Segment indexing error | Fix initial position/velocity arrays | Procedural initialization from trajectory |
| Rope tension not used | MEDIUM | Feedback | Missing connection | Wire rope_forces.tension → controller | Symbolic rope constraint in MPC |
| Estimator receives zeros | MEDIUM | Estimation | Constant source | Wire actual rope tension to estimator | Kalman filter with online learning |

---

## CONCLUSION

**Phase 1 as currently implemented is non-functional for closed-loop control validation.** The three critical issues (zero inputs, unused outputs, stub optimizer) form a perfect storm where:
1. The controller cannot see the system state (all inputs are constants)
2. The controller output is never applied (not wired to plant)
3. The optimizer has no objectives or constraints (stub MPC)

This is not a "Phase 1 limitation"—it's an **architectural misalignment** that must be fixed before any control correctness can be verified.

**Recommended immediate action:** Implement Fixes 1A (discrete state), 1B (state extraction + wiring), and 2 (MPC objectives). This will take Phase 1 from non-functional to **closed-loop with basic feedback**, allowing controller response to be measured and tuned.

**Estimated effort:** 2–3 days (state extraction, discrete state refactoring, simple cost function). **Risk:** Low (no changes to plant model or rope physics).

