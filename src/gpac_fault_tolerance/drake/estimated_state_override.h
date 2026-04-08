#pragma once

/// @file estimated_state_override.h
/// @brief Drake LeafSystem that replaces a single free body's position and
///        velocity DOFs in the full plant state with ESKF estimates, leaving
///        all other bodies unchanged.

#include <Eigen/Core>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/tree/rigid_body.h>
#include <drake/systems/framework/basic_vector.h>
#include <drake/systems/framework/leaf_system.h>

namespace tether_grace {

/// Takes the full plant state as input, along with an ESKF estimated pose
/// [pos(3), quat(4)] and estimated velocity [vel(3)], and produces a
/// modified state vector where the specified body's DOFs are replaced by
/// the ESKF estimates.  Angular velocity passes through from ground truth
/// (ESKF does not directly estimate body-frame angular velocity).
class EstimatedStateOverride final : public drake::systems::LeafSystem<double> {
public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(EstimatedStateOverride);

  EstimatedStateOverride(const drake::multibody::MultibodyPlant<double> &plant,
                         const drake::multibody::RigidBody<double> &body);

  const drake::systems::InputPort<double> &get_plant_state_input_port() const {
    return get_input_port(plant_state_port_);
  }
  const drake::systems::InputPort<double> &
  get_estimated_pose_input_port() const {
    return get_input_port(estimated_pose_port_);
  }
  const drake::systems::InputPort<double> &
  get_estimated_velocity_input_port() const {
    return get_input_port(estimated_velocity_port_);
  }
  const drake::systems::OutputPort<double> &get_output_port_ref() const {
    return get_output_port(output_port_);
  }

private:
  void CalcOverriddenState(const drake::systems::Context<double> &context,
                           drake::systems::BasicVector<double> *output) const;

  int plant_state_port_{};
  int estimated_pose_port_{};
  int estimated_velocity_port_{};
  int output_port_{};

  const drake::multibody::MultibodyPlant<double> &plant_;
  drake::multibody::BodyIndex body_index_;
  int state_size_{};
};

} // namespace tether_grace
