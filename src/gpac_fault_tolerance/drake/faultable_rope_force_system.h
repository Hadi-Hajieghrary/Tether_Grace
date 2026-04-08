#pragma once

#include <optional>
#include <vector>

#include <Eigen/Core>
#include <drake/common/drake_copyable.h>
#include <drake/multibody/plant/externally_applied_spatial_force.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/basic_vector.h>
#include <drake/systems/framework/leaf_system.h>

namespace tether_grace {

class FaultableRopeForceSystem final
    : public drake::systems::LeafSystem<double> {
public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FaultableRopeForceSystem);

  FaultableRopeForceSystem(
      const drake::multibody::MultibodyPlant<double> &plant,
      const drake::multibody::RigidBody<double> &quadcopter_body,
      const drake::multibody::RigidBody<double> &payload_body,
      const std::vector<const drake::multibody::RigidBody<double> *>
          &bead_bodies,
      const Eigen::Vector3d &quadcopter_attachment_point,
      const Eigen::Vector3d &payload_attachment_point, double rope_rest_length,
      double segment_stiffness, double segment_damping,
      std::optional<double> sever_time, double min_distance_threshold = 1e-9);

  const drake::systems::InputPort<double> &get_plant_state_input_port() const {
    return get_input_port(plant_state_port_);
  }

  const drake::systems::OutputPort<double> &get_forces_output_port() const {
    return get_output_port(forces_port_);
  }

  const drake::systems::OutputPort<double> &get_tension_output_port() const {
    return get_output_port(tension_port_);
  }

  bool is_severed(double time) const;

private:
  void CalcRopeForces(
      const drake::systems::Context<double> &context,
      std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>
          *output) const;

  void CalcTopSegmentTension(const drake::systems::Context<double> &context,
                             drake::systems::BasicVector<double> *output) const;

  const drake::multibody::MultibodyPlant<double> &plant_;
  std::vector<drake::multibody::BodyIndex> body_indices_;
  std::vector<Eigen::Vector3d> attachment_points_;
  int num_segments_{};
  double segment_rest_length_{};
  double segment_stiffness_{};
  double segment_damping_{};
  double min_distance_{};
  std::optional<double> sever_time_;

  int plant_state_port_{-1};
  int forces_port_{-1};
  int tension_port_{-1};
};

} // namespace tether_grace