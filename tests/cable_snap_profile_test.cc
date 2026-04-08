#include "gpac_fault_tolerance/core/cable_snap_profile.h"

#include <cassert>
#include <cmath>

namespace {

bool Near(double lhs, double rhs, double tolerance = 1e-9) {
  return std::abs(lhs - rhs) <= tolerance;
}

} // namespace

int main() {
  using tether_grace::gpac_fault_tolerance::CableSnapProfile;
  using tether_grace::gpac_fault_tolerance::CableSnapProfileParams;

  CableSnapProfileParams params;
  params.num_cables = 3;
  params.faulted_cable = 1;
  params.fault_time_seconds = 10.0;
  params.instantaneous = false;
  params.ramp_duration_seconds = 2.0;

  CableSnapProfile profile(params);
  assert(!profile.fault_active(9.5));
  assert(profile.fault_active(10.5));
  assert(Near(profile.multiplier(9.0, 1), 1.0));
  assert(Near(profile.multiplier(10.0, 1), 1.0));
  assert(Near(profile.multiplier(11.0, 1), 0.5));
  assert(Near(profile.multiplier(12.0, 1), 0.0));
  assert(Near(profile.multiplier(12.0, 0), 1.0));

  return 0;
}