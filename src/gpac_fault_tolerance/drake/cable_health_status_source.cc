#include "gpac_fault_tolerance/drake/cable_health_status_source.h"

#include <drake/common/drake_throw.h>
#include <drake/systems/framework/basic_vector.h>

namespace tether_grace::gpac_fault_tolerance::drake {

CableHealthStatusSource::CableHealthStatusSource(
    const CableSnapProfileParams &params)
    : profile_(params) {
  DRAKE_THROW_UNLESS(profile_.params().num_cables > 0);
  cable_health_port_ = this->DeclareVectorOutputPort(
                               "cable_health", profile_.params().num_cables,
                               &CableHealthStatusSource::CalcCableHealth)
                           .get_index();
}

const ::drake::systems::OutputPort<double> &
CableHealthStatusSource::get_cable_health_output_port() const {
  return this->get_output_port(cable_health_port_);
}

int CableHealthStatusSource::num_cables() const {
  return profile_.params().num_cables;
}

const CableSnapProfileParams &CableHealthStatusSource::params() const {
  return profile_.params();
}

void CableHealthStatusSource::CalcCableHealth(
    const ::drake::systems::Context<double> &context,
    ::drake::systems::BasicVector<double> *output) const {
  const auto multipliers = profile_.multipliers(context.get_time());
  auto values = output->get_mutable_value();
  values.setZero();
  for (int index = 0; index < num_cables(); ++index) {
    values[index] = multipliers[static_cast<std::size_t>(index)];
  }
}

} // namespace tether_grace::gpac_fault_tolerance::drake