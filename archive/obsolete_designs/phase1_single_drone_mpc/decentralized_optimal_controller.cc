#include "decentralized_optimal_controller.h"

#include <cmath>
#include <iostream>
#include <iomanip>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <drake/solvers/snopt_solver.h>
#include <drake/solvers/solve.h>

namespace quad_rope_lift {

using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

DecentralizedOptimalController::DecentralizedOptimalController(
    const Config& config)
    : config_(config),
      last_solve_time_(std::chrono::high_resolution_clock::now()) {

  // Declare input ports
  p_drone_port_ =
      DeclareVectorInputPort("p_drone", drake::systems::BasicVector<double>(3))
          .get_index();
  v_drone_port_ =
      DeclareVectorInputPort("v_drone", drake::systems::BasicVector<double>(3))
          .get_index();
  T_cable_port_ =
      DeclareVectorInputPort("T_cable", drake::systems::BasicVector<double>(1))
          .get_index();
  n_cable_port_ =
      DeclareVectorInputPort("n_cable", drake::systems::BasicVector<double>(3))
          .get_index();
  p_ref_port_ =
      DeclareVectorInputPort("p_ref", drake::systems::BasicVector<double>(3))
          .get_index();
  v_ref_port_ =
      DeclareVectorInputPort("v_ref", drake::systems::BasicVector<double>(3))
          .get_index();
  T_others_port_ =
      DeclareVectorInputPort("T_others", drake::systems::BasicVector<double>(4))
          .get_index();
  m_load_port_ =
      DeclareVectorInputPort("m_load", drake::systems::BasicVector<double>(1))
          .get_index();

  // Declare output ports
  DeclareVectorOutputPort("u_optimal", 4,
                          &DecentralizedOptimalController::CalcOptimalControl);
  DeclareVectorOutputPort("p_load_est", 3,
                          &DecentralizedOptimalController::CalcLoadPositionEstimate);
  DeclareVectorOutputPort("v_load_est", 3,
                          &DecentralizedOptimalController::CalcLoadVelocityEstimate);
  DeclareVectorOutputPort("solver_time_ms", 1,
                          &DecentralizedOptimalController::CalcSolverTime);
  DeclareVectorOutputPort("solver_status", 1,
                          &DecentralizedOptimalController::CalcSolverStatus);

  // Declare discrete state for load estimator
  // State vector: [p_load(3), v_load(3), p_load_prev(3)]
  Eigen::VectorXd load_state_init(9);
  load_state_init.setZero();
  load_state_init.segment<3>(0) = Eigen::Vector3d(0.0, 0.0, -1.0);  // Initial position estimate
  load_state_index_ = DeclareDiscreteState(load_state_init);

  // Declare periodic update event for load estimator at control_dt
  DeclarePeriodicDiscreteUpdateEvent(
      config_.control_dt,  // 0.01 s period
      0.0,                 // offset
      &DecentralizedOptimalController::UpdateLoadEstimateDiscreteEvent);
}

void DecentralizedOptimalController::CalcOptimalControl(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* control_output) const {

  // Read inputs from BasicVector ports
  auto p_i = get_input_port(p_drone_port_).Eval<drake::systems::BasicVector<double>>(context).value();
  auto v_i = get_input_port(v_drone_port_).Eval<drake::systems::BasicVector<double>>(context).value();
  double T_i = get_input_port(T_cable_port_).Eval<drake::systems::BasicVector<double>>(context).value()(0);
  auto n_i_vec = get_input_port(n_cable_port_).Eval<drake::systems::BasicVector<double>>(context).value();
  auto p_ref = get_input_port(p_ref_port_).Eval<drake::systems::BasicVector<double>>(context).value();
  auto v_ref = get_input_port(v_ref_port_).Eval<drake::systems::BasicVector<double>>(context).value();
  auto T_others = get_input_port(T_others_port_).Eval<drake::systems::BasicVector<double>>(context).value();
  double m_L_est = get_input_port(m_load_port_).Eval<drake::systems::BasicVector<double>>(context).value()(0);

  // Convert to Vector3d for load estimation
  Vector3d n_i = n_i_vec.segment<3>(0);

  // Read load estimate from discrete state
  const auto& state_vec = context.get_discrete_state(load_state_index_).value();
  Vector3d p_load_est = state_vec.segment<3>(0);
  Vector3d v_load_est = state_vec.segment<3>(3);

  // Construct T_all: [T_0, T_1, T_2, T_3]
  Vector4d T_all;
  T_all(config_.drone_index) = T_i;
  for (int j = 0; j < config_.num_drones; ++j) {
    if (j != config_.drone_index) {
      T_all(j) = T_others(j);
    }
  }

  // Setup and solve optimization problem
  auto prog = SetupOptimizationProblem(p_i.segment<3>(0), v_i.segment<3>(0),
                                        p_load_est, v_load_est,
                                        T_i, n_i,
                                        p_ref.segment<3>(0), v_ref.segment<3>(0),
                                        T_all, m_L_est);

  if (!prog) {
    std::cerr << "Failed to setup optimization problem for drone "
              << config_.drone_index << "\n";
    solver_failures_++;
    control_output->SetZero();
    return;
  }

  // Solve
  auto t_start = std::chrono::high_resolution_clock::now();

  drake::solvers::SolverOptions solver_options;
  solver_options.SetOption(drake::solvers::SnoptSolver::id(),
                          "Major iterations limit",
                          config_.max_solver_iterations);
  solver_options.SetOption(drake::solvers::SnoptSolver::id(),
                          "Print frequency",
                          config_.verbose_solver ? 1 : 0);

  auto result = drake::solvers::Solve(*prog, std::nullopt, std::make_optional(solver_options));

  auto t_end = std::chrono::high_resolution_clock::now();
  solver_time_ms_ =
      std::chrono::duration<double, std::milli>(t_end - t_start).count();

  if (!result.is_success()) {
    if (config_.verbose_solver) {
      std::cerr << "Solver failed for drone " << config_.drone_index
                << " (took " << solver_time_ms_ << " ms)\n";
    }
    solver_failures_++;

    // Fallback: nominal hover control
    Vector4d u_fallback = Vector4d::Zero();
    u_fallback(0) = config_.thrust_max * 0.5;
    control_output->SetFromVector(u_fallback);
    return;
  }

  // Extract optimal control from solution (receding horizon: take first step u_0)
  // Decision variable U has shape (4, N), flattened to 4*N in column-major order
  // First 4 elements are [U(0,0), U(1,0), U(2,0), U(3,0)] = u_0
  auto U_var = prog->decision_variables().segment(0, prog->num_vars());
  auto U_flat = result.GetSolution(U_var.head(4 * static_cast<int>(prog->num_vars() / 4)));

  // Extract first control input u_0 (first column of U)
  Vector4d u_optimal;
  if (U_flat.size() >= 4) {
    u_optimal = U_flat.segment<4>(0);
  } else {
    // Fallback if extraction fails
    u_optimal = Vector4d::Zero();
    u_optimal(0) = config_.thrust_max * 0.5;
  }

  // Saturate to bounds (safety check)
  u_optimal(0) = std::max(0.0, std::min(config_.thrust_max, u_optimal(0)));
  for (int i = 1; i < 4; ++i) {
    u_optimal(i) = std::max(-config_.torque_max, std::min(config_.torque_max, u_optimal(i)));
  }

  control_output->SetFromVector(u_optimal);
}

void DecentralizedOptimalController::CalcLoadPositionEstimate(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output) const {
  const auto& state_vec = context.get_discrete_state(load_state_index_).value();
  Eigen::Vector3d p_load_est = state_vec.segment<3>(0);
  output->SetFromVector(p_load_est);
}

void DecentralizedOptimalController::CalcLoadVelocityEstimate(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output) const {
  const auto& state_vec = context.get_discrete_state(load_state_index_).value();
  Eigen::Vector3d v_load_est = state_vec.segment<3>(3);
  output->SetFromVector(v_load_est);
}

void DecentralizedOptimalController::CalcSolverTime(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output) const {
  output->SetAtIndex(0, solver_time_ms_);
}

void DecentralizedOptimalController::CalcSolverStatus(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output) const {
  output->SetAtIndex(0, solver_failures_ > 0 ? 1.0 : 0.0);
}

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

  // Decision variables: control trajectory U = [u_0, u_1, ..., u_{N-1}]
  // where u_k = [thrust_k, tau_x_k, tau_y_k, tau_z_k]
  auto U = prog->NewContinuousVariables(4, N, "U");

  // Slack variables for rope tautness constraint
  auto slack_tension = prog->NewContinuousVariables(N, "slack_T");

  // ============ OBJECTIVE FUNCTION ============
  // Phase 1: Trajectory tracking cost (minimum viable formulation)
  // Penalize deviation of load position estimate from reference

  // Simplified trajectory tracking cost (based on current load estimate)
  double traj_error_norm = (p_L_est - p_ref).norm();
  prog->AddCost(config_.w_trajectory * traj_error_norm * traj_error_norm);

  // Add control effort penalty (encourages low input magnitudes)
  // Use quadratic cost on individual control inputs to avoid Expression issues
  for (int k = 0; k < N; ++k) {
    for (int i = 0; i < 4; ++i) {
      prog->AddCost(config_.w_effort * U(i, k) * U(i, k));
    }
  }

  // ============ CONSTRAINTS ============

  // Thrust bounds: 0 ≤ thrust ≤ thrust_max
  prog->AddBoundingBoxConstraint(0, config_.thrust_max, U.row(0));

  // Torque bounds: -torque_max ≤ tau_i ≤ torque_max
  prog->AddBoundingBoxConstraint(-config_.torque_max, config_.torque_max,
                                 U.row(1));
  prog->AddBoundingBoxConstraint(-config_.torque_max, config_.torque_max,
                                 U.row(2));
  prog->AddBoundingBoxConstraint(-config_.torque_max, config_.torque_max,
                                 U.row(3));

  // ============ ROPE TAUTNESS CONSTRAINT ============
  // Constraint: T_total ≥ m_L * g for all timesteps
  // For decentralized single-drone MPC: T_i + T_others_total ≥ m_L * g

  double T_others_total = T_all.sum() - T_measured;  // Total from other drones
  double min_tension_required = m_L_est * 9.81;

  for (int k = 0; k < N; ++k) {
    // Constraint: T_i_k ≥ max(0, min_tension_required - T_others_total)
    double T_i_min = std::max(0.0, min_tension_required - T_others_total);
    prog->AddLinearConstraint(U(0, k) >= T_i_min);
  }

  // Slack variable bounds
  prog->AddBoundingBoxConstraint(0, 1.0, slack_tension);

  return prog;
}

Eigen::Vector3d DecentralizedOptimalController::PropagateLoadAcceleration(
    const Vector4d& T_all,
    double m_L) const {

  // Compute total tension force from all drones
  // T_j = magnitude of tension * direction
  // For now, assume vertical tensions only (simplified)

  double T_total = T_all.sum();

  // Vertical acceleration: a_z = (T_total - m_L*g) / m_L
  Vector3d a_L = Vector3d::Zero();
  a_L(2) = (T_total - m_L * 9.81) / m_L;

  return a_L;
}

void DecentralizedOptimalController::UpdateLoadEstimateDiscreteEvent(
    const drake::systems::Context<double>& context,
    drake::systems::DiscreteValues<double>* discrete_state) const {

  // Read input ports
  auto p_i_vec = get_input_port(p_drone_port_)
      .Eval<drake::systems::BasicVector<double>>(context).value();
  auto v_i_vec = get_input_port(v_drone_port_)
      .Eval<drake::systems::BasicVector<double>>(context).value();
  auto n_i_vec = get_input_port(n_cable_port_)
      .Eval<drake::systems::BasicVector<double>>(context).value();

  Vector3d p_i = p_i_vec.segment<3>(0);
  Vector3d v_i = v_i_vec.segment<3>(0);
  Vector3d n_i = n_i_vec.segment<3>(0);

  // Get mutable reference to discrete state and update it
  Vector3d p_load_est = discrete_state->get_mutable_value(load_state_index_).segment<3>(0);
  Vector3d v_load_est = discrete_state->get_mutable_value(load_state_index_).segment<3>(3);
  Vector3d p_load_prev = discrete_state->get_mutable_value(load_state_index_).segment<3>(6);

  // Complementary filter for load position
  // Measurement: z_pos = p_i - L*n (rope constraint)
  Vector3d z_pos = p_i - config_.rope_length * n_i;

  // Low-pass filter: p_load = α*z_pos + (1-α)*p_load_prev
  double alpha = config_.load_estimator_filter_alpha;
  p_load_est = alpha * z_pos + (1.0 - alpha) * p_load_est;

  // Velocity: numerical differentiation + filtering
  Vector3d v_measured = (z_pos - p_load_prev) / config_.control_dt;
  v_load_est = alpha * v_measured + (1.0 - alpha) * v_load_est;

  // Update state for next cycle
  discrete_state->get_mutable_value(load_state_index_).segment<3>(0) = p_load_est;
  discrete_state->get_mutable_value(load_state_index_).segment<3>(3) = v_load_est;
  discrete_state->get_mutable_value(load_state_index_).segment<3>(6) = z_pos;
}

}  // namespace quad_rope_lift
