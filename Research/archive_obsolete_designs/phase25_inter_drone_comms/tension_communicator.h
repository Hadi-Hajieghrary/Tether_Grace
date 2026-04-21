#pragma once

#include <Eigen/Core>
#include <drake/systems/framework/leaf_system.h>

namespace quad_rope_lift {

/// Inter-drone tension communication system
/// Each drone receives the other drone's tension estimates
/// Implements decentralized information sharing for implicit coordination
class TensionCommunicator : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TensionCommunicator);

  /// @param drone_index Index of this drone (0, 1, 2, or 3)
  /// @param communication_delay Delay in seconds (0 for instantaneous)
  /// @param num_drones Total number of drones (default 4)
  explicit TensionCommunicator(int drone_index, double communication_delay = 0.0,
                               int num_drones = 4);

 private:
  void CalcOtherTensions(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* output) const;

  int drone_index_;
  double communication_delay_;
  int num_drones_;
};

}  // namespace quad_rope_lift
