#include "tension_communicator.h"

#include <string>

namespace quad_rope_lift {

TensionCommunicator::TensionCommunicator(int drone_index,
                                         double communication_delay,
                                         int num_drones)
    : drone_index_(drone_index), communication_delay_(communication_delay),
      num_drones_(num_drones) {
  // Input: Tension estimates from all drones (num_drones scalar inputs)
  for (int i = 0; i < num_drones; ++i) {
    DeclareInputPort("T_drone" + std::to_string(i),
                     drake::systems::kVectorValued, 1);
  }

  // Output: Tensions from all drones (4D vector for compatibility)
  DeclareVectorOutputPort("T_others", drake::systems::BasicVector<double>(4),
                          &TensionCommunicator::CalcOtherTensions);

  // Declare discrete state for communication delay (if needed)
  if (communication_delay > 0.0) {
    // TODO: Implement delayed communication using DiscreteState
    // For Phase 2.5, use instantaneous communication
  }
}

void TensionCommunicator::CalcOtherTensions(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output) const {
  // Read tensions from all drones
  Eigen::Vector4d T_all = Eigen::Vector4d::Zero();

  for (int i = 0; i < num_drones_; ++i) {
    auto T_vec = get_input_port(i).Eval<drake::systems::BasicVector<double>>(context).value();
    if (i < 4) {
      T_all(i) = T_vec(0);
    }
  }

  // Output T_others vector: [T0, T1, T2, T3] (padded to 4D)
  output->SetFromVector(T_all);
}

}  // namespace quad_rope_lift
