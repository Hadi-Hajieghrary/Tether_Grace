#pragma once

#include <vector>

#include "gpac_fault_tolerance/core/l1_adaptive_core.h"

namespace tether_grace::gpac_fault_tolerance {

struct DesiredGeometryRedistributorParams {
  bool enabled{false};
  double ramp_duration_seconds{0.2};
};

struct DesiredGeometryRedistributorOutput {
  std::vector<Vector3> offsets;
  std::vector<Vector3> offset_rates;
};

class DesiredGeometryRedistributor {
public:
  explicit DesiredGeometryRedistributor(
      const DesiredGeometryRedistributorParams &params = {});

  DesiredGeometryRedistributorOutput
  Evaluate(const std::vector<Vector3> &nominal_offsets,
           const std::vector<bool> &cable_healthy, double time_seconds,
           double fault_time_seconds) const;

  const DesiredGeometryRedistributorParams &params() const;

private:
  DesiredGeometryRedistributorParams params_{};
};

} // namespace tether_grace::gpac_fault_tolerance