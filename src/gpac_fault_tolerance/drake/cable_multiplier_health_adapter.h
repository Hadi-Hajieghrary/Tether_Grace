#pragma once

#include <drake/systems/framework/leaf_system.h>

namespace tether_grace::gpac_fault_tolerance::drake {

class CableMultiplierHealthAdapter final
    : public ::drake::systems::LeafSystem<double> {
public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CableMultiplierHealthAdapter);

  explicit CableMultiplierHealthAdapter(int num_cables,
                                        double healthy_threshold = 0.5);

  const ::drake::systems::InputPort<double> &get_multiplier_input_port() const;
  const ::drake::systems::OutputPort<double> &
  get_cable_health_output_port() const;

  int num_cables() const;
  double healthy_threshold() const;

private:
  void CalcCableHealth(const ::drake::systems::Context<double> &context,
                       ::drake::systems::BasicVector<double> *output) const;

  int num_cables_{};
  double healthy_threshold_{};
  int multiplier_port_{};
  int cable_health_port_{};
};

} // namespace tether_grace::gpac_fault_tolerance::drake