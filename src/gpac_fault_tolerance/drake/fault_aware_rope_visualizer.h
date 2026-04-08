#pragma once

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <drake/common/drake_copyable.h>
#include <drake/geometry/meshcat.h>
#include <drake/geometry/rgba.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/basic_vector.h>
#include <drake/systems/framework/leaf_system.h>

namespace tether_grace {

class FaultAwareRopeVisualizer final
    : public drake::systems::LeafSystem<double> {
public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FaultAwareRopeVisualizer);

  FaultAwareRopeVisualizer(
      const drake::multibody::MultibodyPlant<double> &plant,
      const std::vector<std::pair<const drake::multibody::RigidBody<double> *,
                                  Eigen::Vector3d>> &body_attachment_points,
      std::shared_ptr<drake::geometry::Meshcat> meshcat,
      std::string meshcat_path, double line_width,
      const drake::geometry::Rgba &line_color, std::optional<double> sever_time,
      double update_period = 1.0 / 30.0);

  const drake::systems::InputPort<double> &get_plant_state_input_port() const {
    return get_input_port(plant_state_port_);
  }

private:
  drake::systems::EventStatus
  UpdateVisualization(const drake::systems::Context<double> &context) const;

  const drake::multibody::MultibodyPlant<double> &plant_;
  std::shared_ptr<drake::geometry::Meshcat> meshcat_;
  std::string meshcat_path_;
  double line_width_{};
  drake::geometry::Rgba line_color_;
  std::optional<double> sever_time_;
  mutable bool deleted_after_sever_{false};

  std::vector<drake::multibody::BodyIndex> body_indices_;
  std::vector<Eigen::Vector3d> attachment_points_;
  int plant_state_port_{-1};
};

} // namespace tether_grace