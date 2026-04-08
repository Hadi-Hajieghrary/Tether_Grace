#include "gpac_fault_tolerance/adapters/gpac_seam_adapter.h"

#include <cassert>
#include <cmath>

namespace {

bool Near(double lhs, double rhs, double tolerance = 1e-9) {
  return std::abs(lhs - rhs) <= tolerance;
}

} // namespace

int main() {
  using tether_grace::gpac_fault_tolerance::adapters::GpacSeamAdapter;
  using tether_grace::gpac_fault_tolerance::adapters::GpacSeamAdapterParams;

  GpacSeamAdapterParams params;
  params.disturbance_margin_mps2 = 2.0;
  params.disturbance_coupling = 1.5;
  params.l1_params.sigma_limit = 5.0;

  GpacSeamAdapter adapter(params);
  adapter.Reset({0.0, 0.0, 0.0});
  const auto &initial = adapter.output();
  for (int axis = 0; axis < 3; ++axis) {
    assert(Near(initial.controller_disturbance_estimate.acceleration_mps2[axis],
                0.0));
    assert(Near(initial.safety_disturbance_bound.lower_mps2[axis], -2.0));
    assert(Near(initial.safety_disturbance_bound.upper_mps2[axis], 2.0));
  }

  const auto &updated =
      adapter.Update({1.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, 0.01);
  assert(std::abs(updated.raw_sigma_hat[0]) > 1e-6);
  assert(std::abs(updated.filtered_compensation.acceleration_mps2[0]) > 1e-6);
  assert(Near(updated.controller_disturbance_estimate.acceleration_mps2[0],
              updated.filtered_compensation.acceleration_mps2[0]));
  assert(updated.safety_disturbance_bound.upper_mps2[0] > 2.0);
  assert(updated.safety_disturbance_bound.lower_mps2[0] < -2.0);

  return 0;
}