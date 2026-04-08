#include "gpac_fault_tolerance/core/fault_metrics.h"

#include <cassert>
#include <cmath>
#include <vector>

namespace {

bool Near(double lhs, double rhs, double tolerance = 1e-6) {
  return std::abs(lhs - rhs) <= tolerance;
}

} // namespace

int main() {
  using tether_grace::gpac_fault_tolerance::ComputeFaultMetrics;
  using tether_grace::gpac_fault_tolerance::FaultMetricsConfig;

  const std::vector<double> times{0.0, 1.0, 2.0, 3.0, 4.0, 5.0,
                                  5.5, 6.0, 6.5, 7.0, 8.0};
  const std::vector<double> errors{0.05, 0.05, 0.05, 0.05, 0.05, 0.20,
                                   0.18, 0.06, 0.05, 0.05, 0.05};

  FaultMetricsConfig config;
  config.fault_time_seconds = 5.0;
  config.window_seconds = 3.0;
  config.peak_window_seconds = 1.0;
  config.recovery_hold_seconds = 0.5;
  config.nominal_start_time_seconds = 0.0;

  const auto metrics = ComputeFaultMetrics(times, errors, config);
  assert(Near(metrics.pre_fault_rmse, 0.05));
  assert(metrics.post_fault_rmse > metrics.pre_fault_rmse);
  assert(metrics.rmse_ratio > 1.0);
  assert(Near(metrics.peak_deviation, 0.20));
  assert(Near(metrics.recovery_time_seconds, 1.0));

  return 0;
}
