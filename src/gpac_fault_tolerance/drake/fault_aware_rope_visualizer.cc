#include "gpac_fault_tolerance/drake/fault_aware_rope_visualizer.h"

namespace tether_grace {

using drake::geometry::Meshcat;
using drake::multibody::MultibodyPlant;
using drake::multibody::RigidBody;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::EventStatus;

FaultAwareRopeVisualizer::FaultAwareRopeVisualizer(
    const MultibodyPlant<double> &plant,
    const std::vector<std::pair<const RigidBody<double> *, Eigen::Vector3d>>
        &body_attachment_points,
    std::shared_ptr<Meshcat> meshcat, std::string meshcat_path,
    double line_width, const drake::geometry::Rgba &line_color,
    std::optional<double> sever_time, double update_period)
    : plant_(plant), meshcat_(std::move(meshcat)),
      meshcat_path_(std::move(meshcat_path)), line_width_(line_width),
      line_color_(line_color), sever_time_(sever_time) {
  body_indices_.reserve(body_attachment_points.size());
  attachment_points_.reserve(body_attachment_points.size());
  for (const auto &[body, point] : body_attachment_points) {
    body_indices_.push_back(body->index());
    attachment_points_.push_back(point);
  }

  plant_state_port_ =
      DeclareVectorInputPort(
          "plant_state",
          BasicVector<double>(plant.num_positions() + plant.num_velocities()))
          .get_index();
  DeclarePeriodicPublishEvent(update_period, 0.0,
                              &FaultAwareRopeVisualizer::UpdateVisualization);
}

EventStatus FaultAwareRopeVisualizer::UpdateVisualization(
    const Context<double> &context) const {
  if (sever_time_.has_value() && context.get_time() >= *sever_time_) {
    if (!deleted_after_sever_) {
      meshcat_->Delete(meshcat_path_);
      deleted_after_sever_ = true;
    }
    return EventStatus::Succeeded();
  }

  const auto &state_vector = get_input_port(plant_state_port_).Eval(context);
  auto plant_context = plant_.CreateDefaultContext();
  plant_.SetPositionsAndVelocities(plant_context.get(), state_vector);

  const int num_points = static_cast<int>(body_indices_.size());
  Eigen::Matrix3Xd points_world(3, num_points);
  for (int i = 0; i < num_points; ++i) {
    const auto &body = plant_.get_body(body_indices_[i]);
    const auto &X_WB = plant_.EvalBodyPoseInWorld(*plant_context, body);
    points_world.col(i) = X_WB * attachment_points_[i];
  }
  meshcat_->SetLine(meshcat_path_, points_world, line_width_, line_color_);
  return EventStatus::Succeeded();
}

} // namespace tether_grace