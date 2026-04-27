#pragma once

#include <memory>
#include <optional>
#include <vector>

#include <Eigen/Core>
#include <drake/multibody/plant/externally_applied_spatial_force.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/leaf_system.h>

#include "quadcopter_controller.h"  // TrajectoryWaypoint

namespace quad_rope_lift {

/// Drop-in replacement for `DecentralizedLocalController` that solves a
/// receding-horizon QP with a linearised tension-ceiling constraint on
/// every prediction step. Prediction is done in error-state coordinates
/// (e_p, e_v) with a pre-computed condensed model (Φ, Ω) and a DARE
/// terminal cost; the tension constraint is a first-order Taylor
/// expansion of the rope-spring law around the current chord geometry.
///
/// The port interface matches `DecentralizedLocalController` exactly,
/// so the sim harness switches between them via `--controller=`.
class MpcLocalController final
    : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MpcLocalController);

  struct Params {
    // Formation + shared reference (same semantics as the baseline).
    Eigen::Vector3d formation_offset = Eigen::Vector3d::Zero();
    std::vector<TrajectoryWaypoint> waypoints;
    double rope_length = 1.25;
    double rope_drop = 1.211;   // hover-equilibrium drone–payload Δz

    // Cascade gains (unchanged from baseline's outer/inner loop).
    double position_kp = 30.0;
    double position_kd = 15.0;
    double altitude_kp = 100.0;
    double altitude_kd = 24.0;
    double attitude_kp = 25.0;
    double attitude_kd = 4.0;

    // Anti-swing (identical to the baseline controller).
    double swing_kd = 0.8;
    double swing_offset_max = 0.3;
    /// Weight of the anti-swing term in the MPC reference acceleration
    /// a_target = a_track + w_swing · a_swing; matches the baseline's
    /// w_swing so the unconstrained limit of the MPC reduces to the
    /// baseline command.
    double w_swing = 0.3;

    // Actuation limits.
    double max_tilt = 0.6;
    double min_thrust = 0.0;
    double max_thrust = 150.0;
    double max_torque = 10.0;
    double gravity = 9.81;

    // Pickup ramp (identical to baseline; initial_pretensioned default).
    double pickup_ramp_duration = 1.0;
    double pickup_detection_threshold = 0.3;
    double pickup_target_tension_nominal = 7.36;
    double tension_feedback_kp = 1.5;
    bool initial_pretensioned = true;

    // ── MPC horizon + QP weights ────────────────────────────────────
    int horizon_steps = 5;           // N_p (default 5; eval 10)
    double dt_mpc = 2e-4;             // control step (matches sim)
    double q_pos = 100.0;             // state-cost position weight
    double q_vel = 10.0;              // state-cost velocity weight
    // Input-cost weight. Matches the baseline's w_effort so that the
    // unconstrained MPC optimum reduces to the baseline's PD+swing
    // target (U* = w_t/(w_t+w_e)·a_target ≈ a_target when w_e ≪ w_t).
    double r_ctrl = 0.02;

    // ── Tension ceiling ─────────────────────────────────────────────
    double T_max_safe = 100.0;           // N (hard ceiling per rope)
    double tension_slack_weight = 1.0e4; // penalty on slack variables

    // ── Tension linearisation ──────────────────────────────────────
    // Effective rope stiffness used by the linearised constraint.
    // Matches the series stiffness k_seg / N_seg = 2777.8 N/m.
    double k_eff = 25000.0 / 9.0;
    // Effective body-to-body rest length, chosen so the linearised
    // nominal hover tension evaluates to the true static load share
    // (≈ 8 N per rope for the 5-drone reference case) rather than the
    // naïve geometric d_body − L_rest value. Populated by the harness
    // from the hover fixed point.
    double L_eff_body = 1.448;
  };

  MpcLocalController(
      const drake::multibody::MultibodyPlant<double>& plant,
      const drake::multibody::RigidBody<double>& quadcopter_body,
      const drake::multibody::RigidBody<double>& payload_body,
      const Params& params);

  const drake::systems::InputPort<double>& get_plant_state_input_port() const {
    return get_input_port(plant_state_port_);
  }
  const drake::systems::InputPort<double>& get_tension_input_port() const {
    return get_input_port(tension_port_);
  }

  const drake::systems::OutputPort<double>& get_control_output_port() const {
    return get_output_port(control_port_);
  }
  const drake::systems::OutputPort<double>& get_control_vector_output_port() const {
    return get_output_port(control_vector_port_);
  }
  const drake::systems::OutputPort<double>& get_diagnostics_output_port() const {
    return get_output_port(diagnostics_port_);
  }

  void set_mass(double mass) { mass_ = mass; }

  // Telemetry helpers for logging (fill by the last MPC solve).
  double last_mpc_solve_time_us() const { return last_mpc_solve_time_us_; }
  double last_mpc_cost_value()    const { return last_mpc_cost_value_; }
  double last_mpc_slack_max()     const { return last_mpc_slack_max_; }
  double last_mpc_horizon_T_max() const { return last_mpc_horizon_T_max_; }

 private:
  void CalcControlForce(
      const drake::systems::Context<double>& context,
      std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>*
          output) const;
  void CalcControlVector(const drake::systems::Context<double>& context,
                         drake::systems::BasicVector<double>* output) const;
  void CalcDiagnostics(const drake::systems::Context<double>& context,
                       drake::systems::BasicVector<double>* output) const;

  // Plant handles.
  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::multibody::BodyIndex quad_body_index_;
  drake::multibody::BodyIndex payload_body_index_;
  double mass_;

  Params params_;

  int plant_state_port_{-1};
  int tension_port_{-1};
  int control_port_{-1};
  int control_vector_port_{-1};
  int diagnostics_port_{-1};

  // Pre-computed MPC matrices (constant at ctor).
  Eigen::MatrixXd A_;             // 6x6 state-transition (block-diag of 2x2 double integrators)
  Eigen::MatrixXd B_;             // 6x3 control-input matrix (carries negative signs)
  Eigen::MatrixXd Phi_;           // 6·N_p × 6 condensed free-response stack
  Eigen::MatrixXd Omega_;         // 6·N_p × 3·N_p block-lower-triangular prediction matrix
  Eigen::MatrixXd H_;             // Cost Hessian (= (w_t + w_e)·I_{3·N_p})

  // DARE diagnostics cached at ctor.
  double dare_spectral_radius_{0.0};

  // One-shot pickup latch (same as baseline).
  mutable std::optional<double> pickup_start_time_;

  // Tension-rate state: numerical derivative of measured tension,
  // used by the richer linearised tension row to react to bead-chain
  // wave propagation during fault transients.
  mutable double last_T_meas_{0.0};
  mutable double T_dot_filtered_{0.0};
  mutable double last_tick_time_{-1.0};
  mutable int    warmup_ticks_{0};
  // Low-pass filter time constant on T_dot (s).
  static constexpr double kTDotTau = 0.005;
  // Extrapolation horizon limit: clamp j·dt·T_dot so very long horizons
  // don't produce unphysical tension predictions.
  static constexpr double kMaxForwardT = 200.0;  // N

  // Warm-start cache — primal only; OSQP dual warm-start disabled for
  // simplicity (still benefits from cached Hessian + shifted primal).
  mutable Eigen::VectorXd U_warm_;

  // Telemetry for the diagnostics port.
  mutable double last_mpc_solve_time_us_{0.0};
  mutable double last_mpc_cost_value_{0.0};
  mutable double last_mpc_slack_max_{0.0};
  mutable double last_mpc_horizon_T_max_{0.0};
  mutable double last_thrust_cmd_{0.0};
  mutable double last_tilt_mag_{0.0};
  mutable double last_T_ff_{0.0};
  mutable double last_swing_offset_mag_{0.0};
  mutable int    last_mpc_relax_flag_{0};
};

}  // namespace quad_rope_lift
