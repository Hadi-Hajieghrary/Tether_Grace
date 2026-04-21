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

/// Fully local, peer-free, QP-based quadrotor controller for cooperative
/// payload lift with under-slung load. Each drone uses only:
///
///   * Its OWN state (pose + velocity from plant),
///   * Its OWN rope-tension measurement (scalar + 3D force vector),
///   * The PAYLOAD state (pose + velocity — physically observable on each
///     drone via an onboard IR range sensor or visual tracking of the
///     payload, simulated here as ground-truth payload-body pose from the
///     MultibodyPlant),
///   * A SHARED feed-forward payload-reference trajectory (no feedback, no
///     peer communication — the reference is the mission plan that every
///     drone is given before flight).
///
/// At each control step the drone solves a local quadratic program that
/// jointly minimises (i) payload-tracking error, (ii) payload swing, and
/// (iii) the drone's own control effort, subject to tilt-envelope and
/// thrust-envelope constraints. The QP output is a commanded acceleration
/// that is mapped to thrust + desired tilt + a fast attitude PD inner loop.
///
/// This is the IEEE-paper's target controller: truly decentralised (no peer
/// tensions input, no N_alive estimate, no peer-state estimator) and
/// explicitly tracking + anti-swing + effort-minimising at every step.
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
    /// Coefficient multiplying -v_payload_xy; larger → more aggressive
    /// damping of under-slung-payload pendulum motion.
    double swing_kd = 2.0;
    /// Saturation on the anti-swing slot offset (metres), to keep the
    /// drone inside the formation radius.
    double swing_offset_max = 0.5;

    // Per-drone QP weights ---------------------------------------------
    double w_track = 1.0;
    double w_swing = 0.8;
    double w_effort = 0.03;

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
  int control_port_{-1};
  int control_vector_port_{-1};
  int diagnostics_port_{-1};

  // One-shot latch — idempotent.
  mutable std::optional<double> pickup_start_time_;
  mutable double last_qp_cost_ = 0.0;
  mutable double last_swing_offset_mag_ = 0.0;
};

}  // namespace quad_rope_lift
