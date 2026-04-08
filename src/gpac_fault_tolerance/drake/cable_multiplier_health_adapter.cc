#include "gpac_fault_tolerance/drake/cable_multiplier_health_adapter.h"

#include <drake/common/drake_throw.h>
#include <drake/systems/framework/basic_vector.h>

namespace tether_grace::gpac_fault_tolerance::drake {

CableMultiplierHealthAdapter::CableMultiplierHealthAdapter(
    int num_cables, double healthy_threshold)
    : num_cables_(num_cables), healthy_threshold_(healthy_threshold) {
  DRAKE_THROW_UNLESS(num_cables_ > 0);
  multiplier_port_ =
      this->DeclareVectorInputPort("cable_multiplier", num_cables_).get_index();
  cable_health_port_ = this->DeclareVectorOutputPort(
                               "cable_health", num_cables_,
                               &CableMultiplierHealthAdapter::CalcCableHealth)
                           .get_index();
}

const ::drake::systems::InputPort<double> &
CableMultiplierHealthAdapter::get_multiplier_input_port() const {
  return this->get_input_port(multiplier_port_);
}

const ::drake::systems::OutputPort<double> &
CableMultiplierHealthAdapter::get_cable_health_output_port() const {
  return this->get_output_port(cable_health_port_);
}

int CableMultiplierHealthAdapter::num_cables() const { return num_cables_; }

double CableMultiplierHealthAdapter::healthy_threshold() const {
  return healthy_threshold_;
}

void CableMultiplierHealthAdapter::CalcCableHealth(
    const ::drake::systems::Context<double> &context,
    ::drake::systems::BasicVector<double> *output) const {
  const auto &multiplier = get_multiplier_input_port().Eval(context);
  auto values = output->get_mutable_value();
  values.setZero();
  for (int index = 0; index < num_cables_; ++index) {
    values[index] = multiplier[index] >= healthy_threshold_ ? 1.0 : 0.0;
  }
}

} // namespace tether_grace::gpac_fault_tolerance::drake