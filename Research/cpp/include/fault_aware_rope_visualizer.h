#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <drake/geometry/meshcat.h>
#include <drake/geometry/rgba.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/event.h>
#include <drake/systems/framework/leaf_system.h>

namespace quad_rope_lift {

/// Fault-aware rope polyline visualizer for Meshcat.
///
/// Identical to `RopeVisualizer` until the simulation time reaches
/// `fault_time`; after that the visualizer replaces the rope polyline with a
/// fully-transparent line and stops issuing further updates. In a recorded
/// Meshcat animation, this causes the rope to disappear cleanly at the fault
/// event without leaving a ghost line at the last taut configuration.
class FaultAwareRopeVisualizer final : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FaultAwareRopeVisualizer);

  /// @param plant MultibodyPlant containing all tracked bodies.
  /// @param body_attachment_points Ordered (body, local-point) list from drone
  ///        through bead chain to payload.
  /// @param meshcat Shared Meshcat instance.
  /// @param meshcat_path Path in the Meshcat scene tree for the line.
  /// @param line_width Polyline width in screen pixels.
  /// @param line_color RGBA colour.
  /// @param fault_time Time [s] at which the rope is severed; set to a
  ///        negative value for "never severed" (behaves like RopeVisualizer).
  /// @param update_period Visualisation update period [s].
  FaultAwareRopeVisualizer(
      const drake::multibody::MultibodyPlant<double>& plant,
      const std::vector<std::pair<const drake::multibody::RigidBody<double>*,
                                  Eigen::Vector3d>>& body_attachment_points,
      std::shared_ptr<drake::geometry::Meshcat> meshcat,
      std::string meshcat_path,
      double line_width,
      const drake::geometry::Rgba& line_color,
      double fault_time = -1.0,
      double update_period = 1.0 / 60.0);

  const drake::systems::InputPort<double>& get_plant_state_input_port() const {
    return get_input_port(plant_state_port_);
  }

 private:
  drake::systems::EventStatus Update(
      const drake::systems::Context<double>& context) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  std::shared_ptr<drake::geometry::Meshcat> meshcat_;

  std::vector<drake::multibody::BodyIndex> body_indices_;
  std::vector<Eigen::Vector3d> attachment_points_;

  std::string meshcat_path_;
  double line_width_;
  drake::geometry::Rgba line_color_;
  double fault_time_;

  int plant_state_port_{-1};
  // One-shot "hidden" latch — protected by idempotency of the Meshcat call.
  mutable bool hidden_ = false;
};

}  // namespace quad_rope_lift
