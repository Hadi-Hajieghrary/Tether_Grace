#pragma once

#include <vector>
#include <Eigen/Core>
#include <drake/multibody/plant/externally_applied_spatial_force.h>
#include <drake/systems/framework/leaf_system.h>

namespace quad_rope_lift {

/// Combines ExternallyAppliedSpatialForce vectors from two rope systems
/// Merges per-body forces from both ropes and outputs combined forces
class ForceCombiner : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ForceCombiner);

  ForceCombiner(int num_bodies, int num_ropes = 2);

 private:
  void CalcCombinedForces(
      const drake::systems::Context<double>& context,
      drake::AbstractValue* output) const;

  int num_bodies_;
  int num_ropes_;
};

}  // namespace quad_rope_lift
