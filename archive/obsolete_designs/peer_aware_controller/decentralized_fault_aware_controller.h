#pragma once

#include <optional>
#include <vector>

#include <Eigen/Core>
#include <drake/multibody/plant/externally_applied_spatial_force.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/leaf_system.h>

#include "quadcopter_controller.h"  // Reuse ControllerParams + Waypoint

namespace quad_rope_lift {

/// Decentralized, peer-aware lift controller.
///
/// Structurally identical to QuadcopterLiftController (cascade PD: position →
/// tilt → torque), but augments the tension feedforward with per-drone load
/// rebalancing: it monitors peer tensions, infers the number of "alive" drones
/// N_alive (those whose rope still carries load above pickup_threshold), and
/// updates the pickup_target_tension to m_L · g / N_alive.
///
/// When a peer's cable severs, its reported tension collapses to ~0; N_alive
/// decrements automatically; this drone's feedforward increases so it picks up
/// the slack with no centralized coordinator.
class DecentralizedFaultAwareController
    : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DecentralizedFaultAwareController);

  /// @param plant The MultibodyPlant (used to read quad body pose).
  /// @param quadcopter_body This drone's rigid body.
  /// @param params Standard QuadcopterLiftController params.
  /// @param payload_mass Total payload mass [kg] — used for N_alive feedforward.
  /// @param num_drones Total drones in the formation.
  DecentralizedFaultAwareController(
      const drake::multibody::MultibodyPlant<double>& plant,
      const drake::multibody::RigidBody<double>& quadcopter_body,
      const ControllerParams& params,
      double payload_mass,
      int num_drones);

  // Input ports mirror QuadcopterLiftController
  const drake::systems::InputPort<double>& get_plant_state_input_port() const {
    return get_input_port(plant_state_port_);
  }
  const drake::systems::InputPort<double>& get_tension_input_port() const {
    return get_input_port(tension_port_);
  }
  /// Vector of size num_drones containing each peer's measured scalar tension.
  /// This drone's own entry is included but ignored (we use our own measurement).
  const drake::systems::InputPort<double>& get_peer_tensions_input_port() const {
    return get_input_port(peer_tensions_port_);
  }

  /// Output: ExternallyAppliedSpatialForce applied to this drone's body
  /// (drop-in replacement for QuadcopterLiftController::get_control_output_port).
  const drake::systems::OutputPort<double>& get_control_output_port() const {
    return get_output_port(control_port_);
  }

  /// Diagnostic: [N_alive, my_target_tension, peer_tension_sum]
  const drake::systems::OutputPort<double>& get_diagnostics_output_port() const {
    return get_output_port(diagnostics_port_);
  }

  /// Vector form of the control output for logging: [tau_x, tau_y, tau_z, f_x, f_y, f_z]
  const drake::systems::OutputPort<double>& get_control_vector_output_port() const {
    return get_output_port(control_vector_port_);
  }

  void set_mass(double mass) { mass_ = mass; }

 private:
  void CalcControlForce(
      const drake::systems::Context<double>& context,
      std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>* output) const;

  void CalcDiagnostics(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* output) const;

  void CalcControlVector(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* output) const;

  void ComputeTrajectory(double t, Eigen::Vector3d& pos_des,
                         Eigen::Vector3d& vel_des) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::multibody::BodyIndex quad_body_index_;
  double mass_;
  double payload_mass_;
  int num_drones_;

  // Trajectory / formation
  Eigen::Vector3d formation_offset_;
  std::vector<TrajectoryWaypoint> waypoints_;
  bool use_waypoints_;
  double initial_altitude_;
  double final_altitude_;
  double ascent_start_time_;
  double climb_rate_;
  double ascent_duration_;
  double ascent_direction_;

  // PD gains
  double position_kp_;
  double position_kd_;
  double max_tilt_angle_;
  double altitude_kp_;
  double altitude_kd_;
  double attitude_kp_;
  double attitude_kd_;

  // Tension feedforward
  double tension_kp_;
  double tension_altitude_gain_;
  double tension_altitude_max_;

  // Pickup phase
  double pickup_duration_;
  double pickup_target_tension_nominal_;  // nominal share = m_L·g / N_total
  double pickup_threshold_;
  mutable std::optional<double> pickup_start_time_;

  // Limits
  double min_thrust_;
  double max_thrust_;
  double max_torque_;
  double gravity_;

  // Peer-fault detection threshold: a peer is considered failed if its
  // reported tension falls below this value.
  double peer_failure_threshold_;

  // Port indices
  int plant_state_port_{-1};
  int tension_port_{-1};
  int peer_tensions_port_{-1};
  int control_port_{-1};
  int control_vector_port_{-1};
  int diagnostics_port_{-1};

  // Drone index (for ignoring own entry in peer_tensions vector)
  int my_index_{-1};

 public:
  /// Must be called right after construction — sets which entry in the
  /// peer-tension vector is this drone's own (to exclude from N_alive count).
  void set_my_index(int idx) { my_index_ = idx; }
};

}  // namespace quad_rope_lift
