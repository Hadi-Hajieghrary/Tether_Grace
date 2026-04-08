#include "gpac_fault_tolerance/core/fault_metrics.h"

#include <algorithm>
#include <cmath>

namespace tether_grace::gpac_fault_tolerance {

namespace {

double ComputeRmse(const std::vector<double> &samples) {
  if (samples.empty()) {
    return 0.0;
  }

  double sum = 0.0;
  for (double sample : samples) {
    sum += sample * sample;
  }
  return std::sqrt(sum / static_cast<double>(samples.size()));
}

} // namespace

FaultMetrics ComputeFaultMetrics(const std::vector<double> &times,
                                 const std::vector<double> &errors,
                                 const FaultMetricsConfig &config) {
  FaultMetrics metrics;
  metrics.fault_time_seconds = config.fault_time_seconds;
  metrics.window_seconds = config.window_seconds;

  if (times.empty() || errors.empty() || times.size() != errors.size()) {
    return metrics;
  }

  std::vector<double> nominal_errors;
  std::vector<double> pre_fault_errors;
  std::vector<double> post_fault_errors;

  for (std::size_t index = 0; index < times.size(); ++index) {
    const double time_seconds = times[index];
    const double error = errors[index];

    if (time_seconds >= config.nominal_start_time_seconds) {
      nominal_errors.push_back(error);
    }
    if (time_seconds >=
            std::max(config.nominal_start_time_seconds,
                     config.fault_time_seconds - config.window_seconds) &&
        time_seconds < config.fault_time_seconds) {
      pre_fault_errors.push_back(error);
    }
    if (time_seconds >= config.fault_time_seconds &&
        time_seconds < config.fault_time_seconds + config.window_seconds) {
      post_fault_errors.push_back(error);
    }
    if (time_seconds >= config.fault_time_seconds &&
        time_seconds < config.fault_time_seconds + config.peak_window_seconds) {
      metrics.peak_deviation = std::max(metrics.peak_deviation, error);
    }
  }

  metrics.nominal_rmse = ComputeRmse(nominal_errors);

  if (config.fault_time_seconds >= 1e9) {
    metrics.pre_fault_rmse = metrics.nominal_rmse;
    metrics.post_fault_rmse = metrics.nominal_rmse;
    metrics.rmse_ratio = metrics.nominal_rmse > 0.0 ? 1.0 : 0.0;
    metrics.recovery_time_seconds = 0.0;
    return metrics;
  }

  metrics.pre_fault_rmse = ComputeRmse(pre_fault_errors);
  metrics.post_fault_rmse = ComputeRmse(post_fault_errors);
  if (metrics.pre_fault_rmse > 1e-9) {
    metrics.rmse_ratio = metrics.post_fault_rmse / metrics.pre_fault_rmse;
  }

  const double threshold = config.recovery_multiplier * metrics.pre_fault_rmse;
  metrics.recovery_time_seconds = config.window_seconds;
  for (std::size_t start_index = 0; start_index < times.size(); ++start_index) {
    if (times[start_index] < config.fault_time_seconds ||
        errors[start_index] >= threshold) {
      continue;
    }

    bool sustained = false;
    for (std::size_t end_index = start_index; end_index < times.size();
         ++end_index) {
      if (times[end_index] >=
          times[start_index] + config.recovery_hold_seconds) {
        sustained = true;
        break;
      }
      if (errors[end_index] >= threshold) {
        sustained = false;
        break;
      }
    }

    if (sustained) {
      metrics.recovery_time_seconds =
          times[start_index] - config.fault_time_seconds;
      break;
    }
  }

  return metrics;
}

} // namespace tether_grace::gpac_fault_tolerance
