#pragma once

#include <vector>

namespace tether_grace::gpac_fault_tolerance {

struct CableSnapProfileParams {
  int num_cables{0};
  int faulted_cable{-1};
  double fault_time_seconds{1e10};
  double post_fault_scale{0.0};
  double ramp_duration_seconds{0.001};
  bool instantaneous{true};
};

class CableSnapProfile {
public:
  explicit CableSnapProfile(const CableSnapProfileParams &params);

  bool fault_active(double time_seconds) const;
  double multiplier(double time_seconds, int cable_index) const;
  std::vector<double> multipliers(double time_seconds) const;
  const CableSnapProfileParams &params() const;

private:
  CableSnapProfileParams params_;
};

} // namespace tether_grace::gpac_fault_tolerance
