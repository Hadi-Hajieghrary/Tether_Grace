#pragma once

#include <memory>
#include <string>

#include <drake/geometry/meshcat.h>
#include <drake/systems/framework/leaf_system.h>

namespace quad_rope_lift {

/// Deletes a named Meshcat path once the simulation time reaches `fault_time`.
/// Used to hide a RopeVisualizer polyline (and any other scene objects) for a
/// drone whose cable has been severed, so the replay is not cluttered by a
/// ghost rope after the fault.
class MeshcatFaultHider final : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MeshcatFaultHider);

  MeshcatFaultHider(std::shared_ptr<drake::geometry::Meshcat> meshcat,
                    std::string path, double fault_time,
                    double poll_period = 0.02)
      : meshcat_(std::move(meshcat)),
        path_(std::move(path)),
        fault_time_(fault_time) {
    DeclarePeriodicPublishEvent(poll_period, 0.0,
                                &MeshcatFaultHider::MaybeHide);
  }

 private:
  void MaybeHide(const drake::systems::Context<double>& context) const {
    if (!hidden_ && context.get_time() >= fault_time_) {
      // Delete the object entirely; any recording up to fault_time keeps
      // the rope visible, but after t_fault the playback shows nothing.
      meshcat_->Delete(path_);
      hidden_ = true;
    }
  }

  std::shared_ptr<drake::geometry::Meshcat> meshcat_;
  std::string path_;
  double fault_time_;
  mutable bool hidden_ = false;  // Drake publishers are const; this is a
                                 // one-shot latch for a side-effect that
                                 // does not participate in dynamics.
};

}  // namespace quad_rope_lift
