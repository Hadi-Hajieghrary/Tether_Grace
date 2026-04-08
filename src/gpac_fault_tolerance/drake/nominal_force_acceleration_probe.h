#pragma once

#include <drake/systems/framework/leaf_system.h>

namespace tether_grace::gpac_fault_tolerance::drake {

class NominalForceAccelerationProbe final
    : public ::drake::systems::LeafSystem<double> {
public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(NominalForceAccelerationProbe);

  explicit NominalForceAccelerationProbe(double vehicle_mass_kg);

  const ::drake::systems::InputPort<double> &get_force_input_port() const;
  const ::drake::systems::OutputPort<double> &
  get_acceleration_output_port() const;

private:
  void CalcAcceleration(const ::drake::systems::Context<double> &context,
                        ::drake::systems::BasicVector<double> *output) const;

  double vehicle_mass_kg_{};
  int force_input_port_{};
  int acceleration_output_port_{};
};

} // namespace tether_grace::gpac_fault_tolerance::drake