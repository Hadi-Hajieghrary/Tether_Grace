#pragma once

/// @file fault_detecting_gpac_controller.h
/// @brief Thin fault-detection + escape wrapper around the real GPAC
///        controller from Tether_Lift.
///
/// This replaces the flat-PD FaultDetectingQuadcopterController with the
/// full GPAC hierarchy (Layer 1: position + anti-swing, Layer 2: geometric
/// SO(3) attitude, Layer 4: ESO disturbance rejection) while adding
/// cable-snap detection and an escape maneuver for the freed agent.

#include <optional>
#include <vector>

#include <Eigen/Core>
#include <drake/multibody/plant/externally_applied_spatial_force.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/basic_vector.h>
#include <drake/systems/framework/leaf_system.h>

#include "extended_state_observer.h"
#include "gpac_quadcopter_controller.h"

namespace tether_grace {

/// Parameters that extend GPACParams with fault-detection fields.
struct FaultDetectingGPACParams {
  quad_rope_lift::GPACParams gpac;

  bool enable_fault_detection{false};
  double fault_detect_tension_threshold{1.0};
  double fault_detect_hold_time{0.12};
  double fault_detect_arming_time{0.75};

  // Escape geometry
  double first_escape_delay{0.55};
  double second_escape_delay{2.6};
  double first_escape_distance{2.2};
  double second_escape_distance{4.4};
  double first_escape_climb{0.7};
  double second_escape_climb{1.3};
  double escape_tangent_bias{0.18};
  double escape_velocity_bias{0.10};

  // Load-tracking mode: close the loop on actual payload position
  bool enable_load_tracking{false};

  // Oracle mode: perfect load-share feedforward (centralized upper bound)
  // When > 0, adds oracle_load_share * g to the vertical feedforward
  // at the exact fault time. Set to m_L / (N-k) for perfect knowledge.
  double oracle_load_share{0.0};
  double load_kp{6.0};
  double load_kd{8.0};
  double load_ki{0.2};
  double initial_theta{1.0};  // initial m_L/N guess (kg)

  // Reactive FTC baseline: boost thrust when tension spike detected
  bool enable_reactive_ftc{false};
  double reactive_boost_fraction{0.3};
  double reactive_tension_threshold{1.5};  // multiplied by pickup_target_tension
  double reactive_debounce_time{0.2};

  // Gain-scheduled PID baseline: boost PID gains when tension spike detected
  bool enable_gain_scheduling{false};
  double gs_tension_threshold{1.5};   // multiplied by pickup_target_tension
  double gs_debounce_time{0.2};       // seconds
  double gs_kp_scale{1.5};            // multiply position_kp by this post-fault
  double gs_kd_scale{1.25};           // multiply position_kd by this post-fault
};

/// Wraps GPACQuadcopterController with cable-snap fault detection and an
/// escape manoeuvre.  During normal flight, every control calculation is
/// delegated to the real GPAC hierarchy.  When a cable snap is detected the
/// wrapper overrides the waypoints to an escape trajectory.
class FaultDetectingGPACController final
    : public drake::systems::LeafSystem<double> {
public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FaultDetectingGPACController);

  FaultDetectingGPACController(
      const drake::multibody::MultibodyPlant<double> &plant,
      const drake::multibody::RigidBody<double> &quadcopter_body,
      const FaultDetectingGPACParams &params);

  // === Input ports ===

  const drake::systems::InputPort<double> &get_plant_state_input_port() const {
    return get_input_port(plant_state_port_);
  }
  const drake::systems::InputPort<double> &get_tension_input_port() const {
    return get_input_port(tension_port_);
  }
  const drake::systems::InputPort<double> &
  get_cable_direction_input_port() const {
    return get_input_port(cable_direction_port_);
  }

  /// Load position input (3D) — from load GPS or estimator.
  /// Only used when enable_load_tracking is true.
  const drake::systems::InputPort<double> &
  get_load_position_input_port() const {
    return get_input_port(load_position_port_);
  }

  /// Load velocity input (3D) — from load estimator.
  const drake::systems::InputPort<double> &
  get_load_velocity_input_port() const {
    return get_input_port(load_velocity_port_);
  }

  /// Load trajectory input (9D) — [p_des(3), v_des(3), a_des(3)].
  const drake::systems::InputPort<double> &
  get_load_trajectory_input_port() const {
    return get_input_port(load_trajectory_port_);
  }

  // === Output ports ===

  const drake::systems::OutputPort<double> &get_control_output_port() const {
    return get_output_port(control_port_);
  }
  const drake::systems::OutputPort<double> &
  get_control_vector_output_port() const {
    return get_output_port(control_vector_port_);
  }

  void set_mass(double mass) { gpac_params_.mass = mass; }

private:
  void DoCalcTimeDerivatives(
      const drake::systems::Context<double> &context,
      drake::systems::ContinuousState<double> *derivatives) const override;

  void SetDefaultState(const drake::systems::Context<double> &context,
                       drake::systems::State<double> *state) const override;

  void CalcControlForce(
      const drake::systems::Context<double> &context,
      std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>
          *output) const;

  void CalcControlVector(const drake::systems::Context<double> &context,
                         drake::systems::BasicVector<double> *output) const;

  void ComputeGPACTrajectory(double t, Eigen::Vector3d &pos_des,
                             Eigen::Vector3d &vel_des,
                             Eigen::Vector3d &acc_des) const;

  /// Evaluate escape waypoints (same layout as GPACWaypoint).
  void ComputeEscapeTrajectory(double t, Eigen::Vector3d &pos_des,
                               Eigen::Vector3d &vel_des) const;

  /// Build the escape waypoints from current state.
  std::vector<quad_rope_lift::GPACWaypoint>
  BuildEscapeWaypoints(const Eigen::Vector3d &start_position,
                       const Eigen::Vector3d &start_velocity,
                       const Eigen::Vector3d &nominal_center_position,
                       double start_time, double nominal_altitude) const;

  /// Check tension and possibly trigger escape.
  void MaybeActivateEscape(double time, double measured_tension,
                           const Eigen::Vector3d &translation,
                           const Eigen::Vector3d &translational_velocity,
                           const Eigen::Vector3d &nominal_position,
                           double nominal_altitude) const;

  // ---- GPAC Control layers (delegated) ----

  /// Layer 1: position + anti-swing + PID + ESO feedforward → desired thrust
  Eigen::Vector3d
  ComputeLayer1Control(const drake::systems::Context<double> &context,
                       const Eigen::Vector3d &position,
                       const Eigen::Vector3d &velocity,
                       double measured_tension) const;

  /// Layer 2: geometric SO(3) attitude control → body torque
  Eigen::Vector3d ComputeLayer2Control(const Eigen::Matrix3d &R,
                                       const Eigen::Matrix3d &R_d,
                                       const Eigen::Vector3d &Omega,
                                       const Eigen::Vector3d &Omega_d) const;

  // ---- References ----

  const drake::multibody::MultibodyPlant<double> &plant_;
  drake::multibody::BodyIndex quad_body_index_;

  // GPAC parameters (mutable so set_mass works and ESO state can evolve)
  mutable quad_rope_lift::GPACParams gpac_params_;

  // Escape parameters (immutable after construction)
  FaultDetectingGPACParams fault_params_;

  // Original waypoints
  std::vector<quad_rope_lift::GPACWaypoint> nominal_waypoints_;

  // Precomputed trajectory helpers
  double ascent_duration_;
  double ascent_direction_;

  // Mutable runtime state (fault detection + ESO)
  mutable std::optional<double> pickup_start_time_;
  mutable std::optional<double> low_tension_start_time_;
  mutable std::optional<double> escape_start_time_;
  mutable std::vector<quad_rope_lift::GPACWaypoint> active_escape_waypoints_;

  // Internal ESO
  mutable quad_rope_lift::gpac::DroneESO internal_eso_;
  mutable double last_update_time_{-1.0};
  mutable Eigen::Vector3d last_control_accel_{Eigen::Vector3d::Zero()};

  // Port indices
  int plant_state_port_{};
  int tension_port_{};
  int cable_direction_port_{};
  int load_position_port_{};
  int load_velocity_port_{};
  int load_trajectory_port_{};
  int control_port_{};
  int control_vector_port_{};

  // Adaptive mass estimate (mutable for online learning)
  mutable double theta_hat_{1.0};
  mutable double theta_hat_filter_{1.0};  // low-pass filtered

  // Reactive FTC state
  mutable std::optional<double> reactive_high_tension_start_;
  mutable bool reactive_boost_active_{false};

  // Gain-scheduled PID state
  mutable std::optional<double> gs_high_tension_start_;
  mutable bool gs_gains_boosted_{false};
};

} // namespace tether_grace
