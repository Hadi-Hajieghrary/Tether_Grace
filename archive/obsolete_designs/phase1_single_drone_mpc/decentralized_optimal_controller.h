#pragma once

#include <chrono>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <drake/systems/framework/leaf_system.h>
#include <drake/solvers/mathematical_program.h>
#include <drake/solvers/solve.h>

namespace quad_rope_lift {

/// Decentralized Optimal Controller for cooperative load transport.
///
/// Solves a multi-objective MPC problem per drone:
///
///   min w1·||p_load - p_ref||² + w2·||a_load||² + w3·||u||² + w4·Var(T_i)
///   s.t. rope kinematics, thrust bounds, tautness constraint
///
/// Runs at 50 Hz (20 ms per solve). Uses SNOPT for nonlinear optimization.
class DecentralizedOptimalController final
    : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DecentralizedOptimalController);

  struct Config {
    int drone_index;                      // 0-based drone ID
    int num_drones;                       // Total N
    double rope_length;                   // m
    double control_horizon_sec;           // 3.0 - 5.0
    double control_dt;                    // 0.01 s (100 Hz internal)
    double mpc_period;                    // 0.02 s (50 Hz optimization)

    // Objective weights (normalized by magnitude, scaled by priority)
    double w_trajectory;                  // 50.0 - trajectory tracking
    double w_stability;                   // 5.0  - load acceleration minimization
    double w_effort;                      // 0.5  - control effort
    double w_tension_balance;             // 0.05 - tension balance

    // Bounds
    double thrust_max;                    // N, ~14.7 for 1.5 kg quad
    double torque_max;                    // N·m, ~10.0
    double tilt_angle_max;                // rad, ~0.785 (45°)

    // Estimator parameters
    double load_estimator_filter_alpha;   // 0.1, complementary filter coefficient
    double load_mass_nominal;             // kg, 3.0

    // Solver parameters
    int max_solver_iterations;            // 100
    double solver_tolerance;              // 1e-4
    bool verbose_solver;                  // true for debug output
  };

  explicit DecentralizedOptimalController(const Config& config);

  // Input ports (all at 100 Hz)
  const drake::systems::InputPort<double>& get_drone_position_input_port()
      const {
    return get_input_port(0);  // Eigen::Vector3d
  }

  const drake::systems::InputPort<double>& get_drone_velocity_input_port()
      const {
    return get_input_port(1);  // Eigen::Vector3d
  }

  const drake::systems::InputPort<double>& get_cable_tension_input_port()
      const {
    return get_input_port(2);  // double (scalar)
  }

  const drake::systems::InputPort<double>& get_cable_direction_input_port()
      const {
    return get_input_port(3);  // Eigen::Vector3d (unit vector)
  }

  const drake::systems::InputPort<double>&
  get_reference_trajectory_input_port() const {
    return get_input_port(4);  // Eigen::Vector3d (p_ref at current time)
  }

  const drake::systems::InputPort<double>&
  get_reference_velocity_input_port() const {
    return get_input_port(5);  // Eigen::Vector3d (v_ref at current time)
  }

  const drake::systems::InputPort<double>&
  get_other_tensions_input_port() const {
    return get_input_port(6);  // Eigen::VectorXd (tensions from other drones)
  }

  const drake::systems::InputPort<double>&
  get_payload_mass_input_port() const {
    return get_input_port(7);  // double (estimated payload mass)
  }

  // Output port: optimal control [thrust, tau_x, tau_y, tau_z]
  const drake::systems::OutputPort<double>&
  get_optimal_control_output_port() const {
    return get_output_port(0);  // Eigen::Matrix<double, 4, 1>
  }

  // Diagnostic output ports
  const drake::systems::OutputPort<double>& get_load_position_estimate_output_port()
      const {
    return get_output_port(1);  // Eigen::Vector3d (estimated p_load)
  }

  const drake::systems::OutputPort<double>& get_load_velocity_estimate_output_port()
      const {
    return get_output_port(2);  // Eigen::Vector3d (estimated v_load)
  }

  const drake::systems::OutputPort<double>&
  get_solver_time_output_port() const {
    return get_output_port(3);  // double (solver time in ms)
  }

  const drake::systems::OutputPort<double>&
  get_solver_status_output_port() const {
    return get_output_port(4);  // double (0=success, 1=failure)
  }

 private:
  void CalcOptimalControl(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* control_output) const;

  void CalcLoadPositionEstimate(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* output) const;

  void CalcLoadVelocityEstimate(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* output) const;

  void CalcSolverTime(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* output) const;

  void CalcSolverStatus(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* output) const;

  // Setup MPC optimization problem
  std::unique_ptr<drake::solvers::MathematicalProgram>
  SetupOptimizationProblem(
      const Eigen::Vector3d& p_i,
      const Eigen::Vector3d& v_i,
      const Eigen::Vector3d& p_L_est,
      const Eigen::Vector3d& v_L_est,
      double T_measured,
      const Eigen::Vector3d& n_measured,
      const Eigen::Vector3d& p_ref,
      const Eigen::Vector3d& v_ref,
      const Eigen::VectorXd& T_others,
      double m_L_est) const;

  // Load state propagation (nonlinear dynamics)
  Eigen::Vector3d PropagateLoadAcceleration(
      const Eigen::Vector4d& T_all,
      double m_L) const;

  // Discrete state update event for load estimator
  void UpdateLoadEstimateDiscreteEvent(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  // Configuration
  Config config_;

  // Discrete state index for load estimator
  // State vector: [p_load_x, p_load_y, p_load_z, v_load_x, v_load_y, v_load_z, p_load_prev_x, p_load_prev_y, p_load_prev_z]
  drake::systems::DiscreteStateIndex load_state_index_;

  // Solver statistics (mutable for output calculation)
  mutable int solver_failures_ = 0;
  mutable double solver_time_ms_ = 0.0;
  mutable int solver_iterations_ = 0;

  // Timing
  mutable std::chrono::time_point<std::chrono::high_resolution_clock> last_solve_time_;

  // Port indices
  int p_drone_port_;
  int v_drone_port_;
  int T_cable_port_;
  int n_cable_port_;
  int p_ref_port_;
  int v_ref_port_;
  int T_others_port_;
  int m_load_port_;
};

}  // namespace quad_rope_lift
