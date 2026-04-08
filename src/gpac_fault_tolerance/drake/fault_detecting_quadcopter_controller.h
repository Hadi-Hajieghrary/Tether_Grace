#pragma once

#include <optional>
#include <vector>

#include <Eigen/Core>
#include <drake/common/drake_copyable.h>
#include <drake/multibody/plant/externally_applied_spatial_force.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/basic_vector.h>
#include <drake/systems/framework/leaf_system.h>

#include "quadcopter_controller.h"

namespace tether_grace {

struct FaultDetectingControllerParams {
  quad_rope_lift::ControllerParams base;
  bool enable_fault_detection{false};
  double fault_detect_tension_threshold{0.2};
  double fault_detect_hold_time{0.12};
  double fault_detect_arming_time{0.75};
  double first_escape_delay{1.5};
  double second_escape_delay{3.0};
  double first_escape_distance{1.8};
  double second_escape_distance{3.8};
  double first_escape_climb{0.9};
  double second_escape_climb{1.8};
  double escape_tangent_bias{0.7};
  double escape_velocity_bias{0.4};
};

class FaultDetectingQuadcopterController final
    : public drake::systems::LeafSystem<double> {
public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FaultDetectingQuadcopterController);

  FaultDetectingQuadcopterController(
      const drake::multibody::MultibodyPlant<double> &plant,
      const drake::multibody::RigidBody<double> &quadcopter_body,
      const FaultDetectingControllerParams &params);

  const drake::systems::InputPort<double> &get_plant_state_input_port() const {
    return get_input_port(plant_state_port_);
  }

  const drake::systems::InputPort<double> &get_tension_input_port() const {
    return get_input_port(tension_port_);
  }

  const drake::systems::InputPort<double> &
  get_estimated_state_input_port() const {
    return get_input_port(estimated_state_port_);
  }

  const drake::systems::OutputPort<double> &get_control_output_port() const {
    return get_output_port(control_port_);
  }

  const drake::systems::OutputPort<double> &
  get_control_vector_output_port() const {
    return get_output_port(control_vector_port_);
  }

  void set_mass(double mass) { mass_ = mass; }

private:
  void CalcControlForce(
      const drake::systems::Context<double> &context,
      std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>
          *output) const;

  void CalcControlVector(const drake::systems::Context<double> &context,
                         drake::systems::BasicVector<double> *output) const;

  void ComputeTrajectory(double t, Eigen::Vector3d *pos_des,
                         Eigen::Vector3d *vel_des) const;
  void MaybeActivateEscape(double time, double measured_tension,
                           const Eigen::Vector3d &translation,
                           const Eigen::Vector3d &translational_velocity,
                           const Eigen::Vector3d &nominal_position,
                           double nominal_altitude) const;
  std::vector<quad_rope_lift::TrajectoryWaypoint>
  BuildEscapeWaypoints(const Eigen::Vector3d &start_position,
                       const Eigen::Vector3d &start_velocity,
                       const Eigen::Vector3d &nominal_center_position,
                       double start_time, double nominal_altitude) const;

  const drake::multibody::MultibodyPlant<double> &plant_;
  drake::multibody::BodyIndex quad_body_index_;
  mutable double mass_;

  Eigen::Vector3d formation_offset_;
  std::vector<quad_rope_lift::TrajectoryWaypoint> nominal_waypoints_;
  bool use_waypoints_;

  double initial_altitude_;
  double final_altitude_;
  double ascent_start_time_;
  double climb_rate_;
  double ascent_duration_;
  double ascent_direction_;

  double position_kp_;
  double position_kd_;
  double max_tilt_angle_;
  double altitude_kp_;
  double altitude_kd_;
  double attitude_kp_;
  double attitude_kd_;
  double tension_kp_;
  double tension_altitude_gain_;
  double tension_altitude_max_;
  double pickup_duration_;
  double pickup_target_tension_;
  double pickup_threshold_;
  double min_thrust_;
  double max_thrust_;
  double max_torque_;
  double gravity_;

  bool enable_fault_detection_;
  double fault_detect_tension_threshold_;
  double fault_detect_hold_time_;
  double fault_detect_arming_time_;
  double first_escape_delay_;
  double second_escape_delay_;
  double first_escape_distance_;
  double second_escape_distance_;
  double first_escape_climb_;
  double second_escape_climb_;
  double escape_tangent_bias_;
  double escape_velocity_bias_;

  mutable std::optional<double> pickup_start_time_;
  mutable std::optional<double> low_tension_start_time_;
  mutable std::optional<double> escape_start_time_;
  mutable std::vector<quad_rope_lift::TrajectoryWaypoint>
      active_escape_waypoints_;

  int plant_state_port_{-1};
  int tension_port_{-1};
  int estimated_state_port_{-1};
  int control_port_{-1};
  int control_vector_port_{-1};
};

} // namespace tether_grace