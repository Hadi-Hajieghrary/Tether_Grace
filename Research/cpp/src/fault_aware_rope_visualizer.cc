#include "fault_aware_rope_visualizer.h"

#include <drake/systems/framework/basic_vector.h>

namespace quad_rope_lift {

using drake::geometry::Rgba;
using drake::multibody::MultibodyPlant;
using drake::multibody::RigidBody;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::EventStatus;

FaultAwareRopeVisualizer::FaultAwareRopeVisualizer(
    const MultibodyPlant<double>& plant,
    const std::vector<std::pair<const RigidBody<double>*, Eigen::Vector3d>>&
        body_attachment_points,
    std::shared_ptr<drake::geometry::Meshcat> meshcat,
    std::string meshcat_path,
    double line_width,
    const Rgba& line_color,
    double fault_time,
    double update_period)
    : plant_(plant),
      meshcat_(std::move(meshcat)),
      meshcat_path_(std::move(meshcat_path)),
      line_width_(line_width),
      line_color_(line_color),
      fault_time_(fault_time) {
  body_indices_.reserve(body_attachment_points.size());
  attachment_points_.reserve(body_attachment_points.size());
  for (const auto& [body, point] : body_attachment_points) {
    body_indices_.push_back(body->index());
    attachment_points_.push_back(point);
  }
  plant_state_port_ = DeclareVectorInputPort(
      "plant_state",
      BasicVector<double>(plant.num_positions() + plant.num_velocities()))
      .get_index();
  DeclarePeriodicPublishEvent(update_period, 0.0,
                              &FaultAwareRopeVisualizer::Update);
}

EventStatus FaultAwareRopeVisualizer::Update(
    const Context<double>& context) const {
  const double t = context.get_time();

  if (fault_time_ >= 0.0 && t >= fault_time_) {
    if (!hidden_) {
      // One-shot: replace the polyline with a degenerate (two coincident
      // zero-length) line at the scene origin, and set its opacity to 0.
      // Both operations record into the Meshcat animation timeline, so the
      // replay correctly hides the rope from fault_time onward.
      Eigen::Matrix3Xd zero_line = Eigen::Matrix3Xd::Zero(3, 2);
      meshcat_->SetLine(meshcat_path_, zero_line, 0.1,
                        Rgba(0.0, 0.0, 0.0, 0.0));
      meshcat_->Delete(meshcat_path_);
      hidden_ = true;
    }
    return EventStatus::Succeeded();
  }

  // Normal visualisation.
  const auto& state_vec = get_input_port(plant_state_port_).Eval(context);
  auto plant_context = plant_.CreateDefaultContext();
  plant_.SetPositionsAndVelocities(plant_context.get(), state_vec);

  const int n = static_cast<int>(body_indices_.size());
  Eigen::Matrix3Xd points(3, n);
  for (int i = 0; i < n; ++i) {
    const auto& body = plant_.get_body(body_indices_[i]);
    const auto& X_WB = plant_.EvalBodyPoseInWorld(*plant_context, body);
    points.col(i) = X_WB * attachment_points_[i];
  }
  meshcat_->SetLine(meshcat_path_, points, line_width_, line_color_);

  return EventStatus::Succeeded();
}

}  // namespace quad_rope_lift
