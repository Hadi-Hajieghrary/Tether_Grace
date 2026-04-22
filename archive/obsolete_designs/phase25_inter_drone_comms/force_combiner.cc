#include "force_combiner.h"

#include <map>

namespace quad_rope_lift {

ForceCombiner::ForceCombiner(int num_bodies, int num_ropes)
    : num_bodies_(num_bodies), num_ropes_(num_ropes) {
  // Input ports: abstract-valued force vectors from rope systems
  for (int i = 0; i < num_ropes; ++i) {
    DeclareAbstractInputPort(
        "F_rope" + std::to_string(i),
        drake::Value<std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>>());
  }

  // Output port: combined forces
  DeclareAbstractOutputPort(
      "F_combined",
      [this]() {
        return std::make_unique<drake::Value<
            std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>>>();
      },
      [this](const drake::systems::Context<double>& context,
             drake::AbstractValue* output) {
        this->CalcCombinedForces(context, output);
      });
}

void ForceCombiner::CalcCombinedForces(
    const drake::systems::Context<double>& context,
    drake::AbstractValue* output) const {
  auto& output_vec = output->get_mutable_value<
      std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>>();

  // Combine forces: iterate through each body and sum forces from all ropes
  output_vec.clear();
  output_vec.reserve(num_bodies_);

  // Build a map of forces by body index for efficient merging
  std::map<drake::multibody::BodyIndex, drake::multibody::SpatialForce<double>> combined;

  // Process each rope's forces
  for (int i = 0; i < num_ropes_; ++i) {
    const auto& F_rope = get_input_port(i).Eval<
        std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>>(context);

    for (const auto& force : F_rope) {
      if (combined.count(force.body_index)) {
        combined[force.body_index] += force.F_Bq_W;
      } else {
        combined[force.body_index] = force.F_Bq_W;
      }
    }
  }

  // Output combined forces
  for (const auto& [body_idx, spatial_force] : combined) {
    drake::multibody::ExternallyAppliedSpatialForce<double> force;
    force.body_index = body_idx;
    force.p_BoBq_B = Eigen::Vector3d::Zero();  // Applied at body origin
    force.F_Bq_W = spatial_force;
    output_vec.push_back(force);
  }
}

}  // namespace quad_rope_lift
