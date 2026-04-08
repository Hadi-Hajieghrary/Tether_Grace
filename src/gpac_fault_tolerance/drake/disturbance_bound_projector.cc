#include "gpac_fault_tolerance/drake/disturbance_bound_projector.h"

#include <algorithm>
#include <cmath>

namespace tether_grace::gpac_fault_tolerance::drake {

DisturbanceBoundProjector::DisturbanceBoundProjector() {
  lower_bound_port_ =
      this->DeclareVectorInputPort("lower_bound", 3).get_index();
  upper_bound_port_ =
      this->DeclareVectorInputPort("upper_bound", 3).get_index();
  scalar_bound_port_ =
      this->DeclareVectorOutputPort("scalar_bound", 1,
                                    &DisturbanceBoundProjector::CalcScalarBound)
          .get_index();
}

const ::drake::systems::InputPort<double> &
DisturbanceBoundProjector::get_lower_bound_input_port() const {
  return this->get_input_port(lower_bound_port_);
}

const ::drake::systems::InputPort<double> &
DisturbanceBoundProjector::get_upper_bound_input_port() const {
  return this->get_input_port(upper_bound_port_);
}

const ::drake::systems::OutputPort<double> &
DisturbanceBoundProjector::get_scalar_bound_output_port() const {
  return this->get_output_port(scalar_bound_port_);
}

void DisturbanceBoundProjector::CalcScalarBound(
    const ::drake::systems::Context<double> &context,
    ::drake::systems::BasicVector<double> *output) const {
  const auto &lower = get_lower_bound_input_port().Eval(context);
  const auto &upper = get_upper_bound_input_port().Eval(context);

  double scalar_bound = 0.0;
  for (int axis = 0; axis < 3; ++axis) {
    scalar_bound = std::max(scalar_bound, std::abs(lower[axis]));
    scalar_bound = std::max(scalar_bound, std::abs(upper[axis]));
  }
  output->SetAtIndex(0, scalar_bound);
}

} // namespace tether_grace::gpac_fault_tolerance::drake
