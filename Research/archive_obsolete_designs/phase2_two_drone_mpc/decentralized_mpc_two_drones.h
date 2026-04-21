#pragma once

#include <Eigen/Core>
#include <drake/systems/framework/leaf_system.h>

namespace quad_rope_lift {

/// Cable severance injector for Phase 2 multi-drone testing
/// Simulates cable tension loss by zeroing T_cable at a specified time
class CableSeveranceInjector : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CableSeveranceInjector);

  /// @param drone_index Index of drone whose cable severs (0, 1, etc.)
  /// @param severance_time Time (seconds) when cable is cut. Negative = never.
  CableSeveranceInjector(int drone_index, double severance_time);

 private:
  void CalcTensionWithFault(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* output) const;

  int drone_index_;
  double severance_time_;
};

}  // namespace quad_rope_lift
