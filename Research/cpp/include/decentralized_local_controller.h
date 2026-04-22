#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <drake/multibody/plant/externally_applied_spatial_force.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/leaf_system.h>

#include "quadcopter_controller.h"  // TrajectoryWaypoint only

namespace quad_rope_lift {

/// Decentralised, QP-based controller for a single drone in a cooperative
/// cable-suspended payload lift. Each instance consumes only local signals —
/// own pose, own rope-tension measurement, the observable payload pose, and
/// a shared feed-forward reference — so the formation scales to arbitrary N
/// without peer-to-peer communication.
///
/// Every tick solves a 3-variable QP that projects a PD+anti-swing
/// acceleration target onto the actuator envelope (tilt and thrust box
/// constraints), then synthesises thrust and desired tilt fed to a Lee-style
/// attitude inner loop.
class DecentralizedLocalController final
    : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DecentralizedLocalController);

  struct Params {
    // Formation + shared reference --------------------------------------
    Eigen::Vector3d formation_offset = Eigen::Vector3d::Zero();
    std::vector<TrajectoryWaypoint> waypoints;
    double rope_length = 1.25;
    double rope_drop = 1.25;  ///< effective drone-to-payload vertical distance

    // Tracking (outer loop) ---------------------------------------------
    double position_kp = 30.0;
    double position_kd = 15.0;
    double altitude_kp = 100.0;
    double altitude_kd = 24.0;

    // Attitude (inner loop) ---------------------------------------------
    double attitude_kp = 25.0;
    double attitude_kd = 4.0;

    // Anti-swing --------------------------------------------------------
    /// Proportional gain on −v_{L,xy}; larger ⇒ more aggressive damping
    /// of the under-slung pendulum.
    double swing_kd = 0.8;
    /// Saturation (m) on the anti-swing slot offset; keeps the drone
    /// inside the formation radius.
    double swing_offset_max = 0.3;

    // Per-drone QP weights ---------------------------------------------
    /// Normalised to 1; w_swing and w_effort are relative to w_track.
    double w_track  = 1.0;
    double w_swing  = 0.3;
    double w_effort = 0.02;

    // Actuation limits --------------------------------------------------
    double max_tilt = 0.6;        ///< rad (34°)
    double min_thrust = 0.0;       ///< N
    double max_thrust = 150.0;     ///< N
    double max_torque = 10.0;      ///< N·m
    double gravity = 9.81;         ///< m/s²

    // Pickup ramp (fully local, time-based — NO peer information) -------
    double pickup_ramp_duration = 1.0;                ///< s
    double pickup_detection_threshold = 0.3;           ///< N
    double pickup_target_tension_nominal = 7.36;       ///< N (m_L·g/N_nominal)
    double tension_feedback_kp = 1.5;                  ///< during pickup only

    /// When true, the rope is assumed taut at the hover equilibrium from
    /// t = 0, so T_ff = measured_tension on the very first tick and no
    /// drone free-falls while the rope engages.
    bool initial_pretensioned = true;

    // L1 adaptive outer loop -------------------------------------------
    /// Master enable; default off so baseline runs are bit-identical to
    /// pre-L1 behaviour.
    bool   l1_enabled           = false;
    /// Adaptive gain Γ. Euler-discretisation stability bound
    /// Γ < 2/(T_s · p_{22}) ≈ 4.75e4 (see Supplementary §A).
    double l1_gamma             = 2000.0;
    /// Low-pass bandwidth ω_c (rad/s); ω_c/ω_n^z ≥ 3 required.
    double l1_omega_c           = 25.0;
    /// Projection bounds on σ̂ (m/s²).
    double l1_sigma_min         = -5.0;
    double l1_sigma_max         =  5.0;
    /// Seed and bounds for the rope-stiffness estimate k̂_eff (N/m).
    double l1_k_eff_nominal     = 2777.8;
    double l1_k_eff_min         =  500.0;
    double l1_k_eff_max         = 5000.0;
    /// Gradient gain for the k̂_eff estimator.
    double l1_gamma_k           = 5000.0;
    /// Minimum stretch (m) below which the k̂_eff update is disabled.
    double l1_stretch_threshold = 5e-4;
    /// Control period (s) of the L1 discrete-update event.
    double l1_control_step      = 2e-4;
  };

  DecentralizedLocalController(
      const drake::multibody::MultibodyPlant<double>& plant,
      const drake::multibody::RigidBody<double>& quadcopter_body,
      const drake::multibody::RigidBody<double>& payload_body,
      const Params& params);

  // Input ports — all local ------------------------------------------------
  const drake::systems::InputPort<double>& get_plant_state_input_port() const {
    return get_input_port(plant_state_port_);
  }
  /// Own rope tension, 4D: [T, fx, fy, fz] (magnitude + force on drone).
  const drake::systems::InputPort<double>& get_tension_input_port() const {
    return get_input_port(tension_port_);
  }
  /// Optional dynamic formation-offset (3-vector). When unconnected,
  /// ComputeSlotReference falls back to `Params::formation_offset`;
  /// when connected, it is typically driven by `FormationCoordinator`.
  const drake::systems::InputPort<double>&
  get_formation_offset_override_input_port() const {
    return get_input_port(formation_offset_override_port_);
  }

  // Output ports -----------------------------------------------------------
  /// Spatial force applied to this drone's body (abstract, for plant input).
  const drake::systems::OutputPort<double>& get_control_output_port() const {
    return get_output_port(control_port_);
  }
  /// Control wrench as 6D [τ_x, τ_y, τ_z, f_x, f_y, f_z] for logging.
  const drake::systems::OutputPort<double>& get_control_vector_output_port() const {
    return get_output_port(control_vector_port_);
  }
  /// Diagnostics: [swing_speed, swing_offset_mag, qp_total_cost].
  const drake::systems::OutputPort<double>& get_diagnostics_output_port() const {
    return get_output_port(diagnostics_port_);
  }
  /// L1 adaptive state (5-vector [ê_z, ê̇_z, σ̂, u_ad, k̂_eff]).
  /// Only valid when `Params::l1_enabled = true`; otherwise returns zeros.
  const drake::systems::OutputPort<double>& get_l1_state_output_port() const {
    return get_output_port(l1_state_port_);
  }

  void set_mass(double mass) { mass_ = mass; }

 private:
  void CalcControlForce(
      const drake::systems::Context<double>& context,
      std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>* output) const;

  void CalcControlVector(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* output) const;

  void CalcDiagnostics(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* output) const;

  /// Periodic unrestricted-update event for the L1 adaptive loop.
  /// Reads the plant state + measured tension, runs the state predictor,
  /// adaptive law, and LP filter, and writes back to the L1 discrete
  /// state. Only invoked when `Params::l1_enabled = true`.
  drake::systems::EventStatus CalcL1Update(
      const drake::systems::Context<double>& context,
      drake::systems::State<double>* state) const;

  void CalcL1State(const drake::systems::Context<double>& context,
                   drake::systems::BasicVector<double>* output) const;

  /// Compute the drone's formation-slot reference at time t
  /// (payload reference + formation_offset, without the rope-drop z shift).
  void ComputeSlotReference(double t,
                            Eigen::Vector3d* p_slot_xy,
                            Eigen::Vector3d* v_slot_xy) const;

  /// Compute the payload's reference trajectory at time t.
  void ComputePayloadReference(double t,
                                Eigen::Vector3d* p_ref,
                                Eigen::Vector3d* v_ref) const;

  // Plant handles
  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::multibody::BodyIndex quad_body_index_;
  drake::multibody::BodyIndex payload_body_index_;
  double mass_;

  Params params_;

  // Port indices
  int plant_state_port_{-1};
  int tension_port_{-1};
  int formation_offset_override_port_{-1};
  int control_port_{-1};
  int control_vector_port_{-1};
  int diagnostics_port_{-1};
  int l1_state_port_{-1};

  // L1 discrete-state slot (invalid when l1_enabled = false).
  drake::systems::DiscreteStateIndex l1_state_idx_;

  // One-shot latch — idempotent.
  mutable std::optional<double> pickup_start_time_;
  mutable double last_qp_cost_ = 0.0;
  mutable double last_swing_offset_mag_ = 0.0;
  // QP-solver telemetry (last solve): wall-clock microseconds + active-set
  // flags in {0,1} for the six box constraints, ordered as
  //   [a_x lo, a_x hi, a_y lo, a_y hi, a_z lo, a_z hi].
  mutable double last_solve_time_us_ = 0.0;
  mutable double last_active_set_[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  // Last commanded thrust (N) and tilt magnitude (rad), for diagnostics.
  mutable double last_thrust_cmd_ = 0.0;
  mutable double last_tilt_mag_ = 0.0;
  // Last feed-forward tension value used this tick (for diagnostics).
  mutable double last_T_ff_ = 0.0;
};

}  // namespace quad_rope_lift
