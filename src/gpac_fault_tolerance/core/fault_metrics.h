#pragma once

#include <vector>

namespace tether_grace::gpac_fault_tolerance {

struct FaultMetricsConfig {
  double fault_time_seconds{0.0};
  double window_seconds{5.0};
  double peak_window_seconds{2.0};
  double recovery_multiplier{1.5};
  double recovery_hold_seconds{0.5};
  double nominal_start_time_seconds{8.0};
};

struct FaultMetrics {
  double fault_time_seconds{0.0};
  double window_seconds{0.0};
  double nominal_rmse{0.0};
  double pre_fault_rmse{0.0};
  double post_fault_rmse{0.0};
  double rmse_ratio{0.0};
  double peak_deviation{0.0};
  double recovery_time_seconds{0.0};
};

FaultMetrics ComputeFaultMetrics(const std::vector<double> &times,
                                 const std::vector<double> &errors,
                                 const FaultMetricsConfig &config);

} // namespace tether_grace::gpac_fault_tolerance
