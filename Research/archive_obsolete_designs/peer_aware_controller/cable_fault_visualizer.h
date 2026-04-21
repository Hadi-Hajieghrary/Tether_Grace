#pragma once
///
/// @file cable_fault_visualizer.h
///
/// Drake LeafSystem that hides a fault quadcopter's rope-bead chain in Meshcat
/// at the moment the cable is severed.  It fires a per-step unrestricted update
/// that watches the simulation clock; once t >= fault_time it calls
/// meshcat->SetProperty(path, "visible", false) on every bead body of the
/// affected rope and is a no-op for all later steps.
///
/// Drake's MeshcatVisualizer (default params) publishes bodies at paths:
///   /drake/visualizer/{model_instance_name}/{body_name}
/// e.g. "/drake/visualizer/multi_quad_payload_system/rope_0_bead_3"
/// Setting "visible"=false on that path hides it and all its children
/// (geometry).

#include <memory>
#include <string>

#include <drake/geometry/meshcat.h>
#include <drake/systems/framework/leaf_system.h>

namespace quad_rope_lift {

class CableFaultVisualizer final : public drake::systems::LeafSystem<double> {
public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CableFaultVisualizer);

  /// @param meshcat               Shared Meshcat instance.
  /// @param fault_time            Simulation time at which to hide the rope
  /// [s].
  /// @param fault_quad_index      0-based index of the quad whose rope to hide.
  /// @param num_rope_beads        Number of bead bodies per rope chain.
  /// @param model_instance_name   MultibodyPlant model instance name.
  /// @param visualizer_prefix     MeshcatVisualizerParams::prefix (default
  /// "visualizer").
  /// @param meshcat_root          Meshcat root prefix (default "drake").
  ///
  /// Full body path =
  /// /{meshcat_root}/{visualizer_prefix}/{model_instance_name}/{body_name} e.g.
  /// /drake/visualizer/multi_quad_payload_system/rope_0_bead_3
  CableFaultVisualizer(
      std::shared_ptr<drake::geometry::Meshcat> meshcat, double fault_time,
      int fault_quad_index, int num_rope_beads,
      const std::string &model_instance_name = "multi_quad_payload_system",
      const std::string &visualizer_prefix = "visualizer",
      const std::string &meshcat_root = "drake")
      : meshcat_(std::move(meshcat)), fault_time_(fault_time),
        fault_quad_index_(fault_quad_index), num_rope_beads_(num_rope_beads),
        model_instance_name_(model_instance_name),
        visualizer_prefix_(visualizer_prefix), meshcat_root_(meshcat_root) {
    DeclarePerStepUnrestrictedUpdateEvent(&CableFaultVisualizer::MaybeHideRope);
  }

private:
  drake::systems::EventStatus
  MaybeHideRope(const drake::systems::Context<double> &context,
                drake::systems::State<double> * /*state*/) const {
    if (!hidden_ && context.get_time() >= fault_time_) {
      hidden_ = true;
      for (int i = 0; i < num_rope_beads_; ++i) {
        const std::string body_name = "rope_" +
                                      std::to_string(fault_quad_index_) +
                                      "_bead_" + std::to_string(i);
        // Drake MeshcatVisualizer path: /{root}/{viz_prefix}/{model}/{body}
        // e.g. /drake/visualizer/multi_quad_payload_system/rope_0_bead_3
        const std::string path = "/" + meshcat_root_ + "/" +
                                 visualizer_prefix_ + "/" +
                                 model_instance_name_ + "/" + body_name;
        meshcat_->SetProperty(path, "visible", false);
      }
    }
    return drake::systems::EventStatus::Succeeded();
  }

  std::shared_ptr<drake::geometry::Meshcat> meshcat_;
  double fault_time_;
  int fault_quad_index_;
  int num_rope_beads_;
  std::string model_instance_name_;
  std::string visualizer_prefix_;
  std::string meshcat_root_;
  mutable bool hidden_ = false; // guarded by single-threaded Drake sim
};

} // namespace quad_rope_lift
