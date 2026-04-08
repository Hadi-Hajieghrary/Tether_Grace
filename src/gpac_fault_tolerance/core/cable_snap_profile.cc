#include "gpac_fault_tolerance/core/cable_snap_profile.h"

#include <algorithm>

namespace tether_grace::gpac_fault_tolerance {

CableSnapProfile::CableSnapProfile(const CableSnapProfileParams &params)
    : params_(params) {}

bool CableSnapProfile::fault_active(double time_seconds) const {
  return params_.faulted_cable >= 0 &&
         time_seconds >= params_.fault_time_seconds;
}

double CableSnapProfile::multiplier(double time_seconds,
                                    int cable_index) const {
  if (cable_index < 0 || cable_index >= params_.num_cables) {
    return 1.0;
  }
  if (cable_index != params_.faulted_cable ||
      time_seconds < params_.fault_time_seconds) {
    return 1.0;
  }
  if (params_.instantaneous || params_.ramp_duration_seconds <= 0.0) {
    return params_.post_fault_scale;
  }

  const double elapsed = time_seconds - params_.fault_time_seconds;
  const double alpha =
      std::clamp(elapsed / params_.ramp_duration_seconds, 0.0, 1.0);
  return (1.0 - alpha) + alpha * params_.post_fault_scale;
}

std::vector<double> CableSnapProfile::multipliers(double time_seconds) const {
  std::vector<double> values(params_.num_cables, 1.0);
  for (int cable_index = 0; cable_index < params_.num_cables; ++cable_index) {
    values[cable_index] = multiplier(time_seconds, cable_index);
  }
  return values;
}

const CableSnapProfileParams &CableSnapProfile::params() const {
  return params_;
}

} // namespace tether_grace::gpac_fault_tolerance
