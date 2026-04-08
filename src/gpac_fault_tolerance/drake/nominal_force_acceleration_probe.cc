#include "gpac_fault_tolerance/drake/nominal_force_acceleration_probe.h"

#include <drake/common/drake_throw.h>
#include <drake/systems/framework/basic_vector.h>

namespace tether_grace::gpac_fault_tolerance::drake {

NominalForceAccelerationProbe::NominalForceAccelerationProbe(
    double vehicle_mass_kg)
    : vehicle_mass_kg_(vehicle_mass_kg) {
  DRAKE_THROW_UNLESS(vehicle_mass_kg_ > 0.0);
  force_input_port_ =
      this->DeclareVectorInputPort("nominal_force", 3).get_index();
  acceleration_output_port_ =
      this->DeclareVectorOutputPort(
              "nominal_acceleration", 3,
              &NominalForceAccelerationProbe::CalcAcceleration)
          .get_index();
}

const ::drake::systems::InputPort<double> &
NominalForceAccelerationProbe::get_force_input_port() const {
  return this->get_input_port(force_input_port_);
}

const ::drake::systems::OutputPort<double> &
NominalForceAccelerationProbe::get_acceleration_output_port() const {
  return this->get_output_port(acceleration_output_port_);
}

void NominalForceAccelerationProbe::CalcAcceleration(
    const ::drake::systems::Context<double> &context,
    ::drake::systems::BasicVector<double> *output) const {
  const auto &force = get_force_input_port().Eval(context);
  output->SetAtIndex(0, force[0] / vehicle_mass_kg_);
  output->SetAtIndex(1, force[1] / vehicle_mass_kg_);
  output->SetAtIndex(2, force[2] / vehicle_mass_kg_);
}

} // namespace tether_grace::gpac_fault_tolerance::drake