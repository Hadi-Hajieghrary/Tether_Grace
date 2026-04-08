#pragma once

#include <drake/systems/framework/leaf_system.h>

#include "gpac_fault_tolerance/core/cable_snap_profile.h"

namespace tether_grace::gpac_fault_tolerance::drake {

class CableHealthStatusSource final
    : public ::drake::systems::LeafSystem<double> {
public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CableHealthStatusSource);

  explicit CableHealthStatusSource(const CableSnapProfileParams &params);

  const ::drake::systems::OutputPort<double> &
  get_cable_health_output_port() const;

  int num_cables() const;
  const CableSnapProfileParams &params() const;

private:
  void CalcCableHealth(const ::drake::systems::Context<double> &context,
                       ::drake::systems::BasicVector<double> *output) const;

  CableSnapProfile profile_;
  int cable_health_port_{};
};

} // namespace tether_grace::gpac_fault_tolerance::drake