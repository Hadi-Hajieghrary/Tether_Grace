#pragma once

#include <drake/systems/framework/leaf_system.h>

namespace tether_grace::gpac_fault_tolerance::drake {

class DisturbanceBoundProjector final
    : public ::drake::systems::LeafSystem<double> {
public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DisturbanceBoundProjector);

  DisturbanceBoundProjector();

  const ::drake::systems::InputPort<double> &get_lower_bound_input_port() const;
  const ::drake::systems::InputPort<double> &get_upper_bound_input_port() const;
  const ::drake::systems::OutputPort<double> &
  get_scalar_bound_output_port() const;

private:
  void CalcScalarBound(const ::drake::systems::Context<double> &context,
                       ::drake::systems::BasicVector<double> *output) const;

  int lower_bound_port_{};
  int upper_bound_port_{};
  int scalar_bound_port_{};
};

} // namespace tether_grace::gpac_fault_tolerance::drake
