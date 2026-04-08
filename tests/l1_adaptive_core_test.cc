#include "gpac_fault_tolerance/core/l1_adaptive_core.h"

#include <cassert>
#include <cmath>

namespace {

using tether_grace::gpac_fault_tolerance::L1AdaptiveCore;
using tether_grace::gpac_fault_tolerance::L1AdaptiveParams;
using tether_grace::gpac_fault_tolerance::Vector3;

bool Near(double lhs, double rhs, double tolerance = 1e-9) {
  return std::abs(lhs - rhs) <= tolerance;
}

} // namespace

int main() {
  L1AdaptiveParams params;
  params.sigma_limit = 5.0;
  L1AdaptiveCore core(params);
  core.Reset({0.0, 0.0, 0.0});

  const auto &initial =
      core.Update({0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, 0.01);
  for (int axis = 0; axis < 3; ++axis) {
    assert(Near(initial.compensation[axis], 0.0));
    assert(Near(initial.sigma_hat[axis], 0.0));
  }

  Vector3 measured_position{1.0, 0.0, 0.0};
  for (int step = 0; step < 20; ++step) {
    core.Update(measured_position, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, 0.01);
  }

  const auto &disturbed = core.state();
  assert(disturbed.sigma_hat[0] <= params.sigma_limit);
  assert(disturbed.sigma_hat[0] >= -params.sigma_limit);
  assert(std::abs(disturbed.compensation[0]) > 1e-6);
  assert(std::abs(disturbed.prediction_error[0]) > 1e-6);

  L1AdaptiveParams acceleration_params;
  acceleration_params.predictor_pole = 0.0;
  L1AdaptiveCore acceleration_core(acceleration_params);
  acceleration_core.Reset({0.0, 0.0, 0.0});

  const auto &acceleration_only = acceleration_core.Update(
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {3.0, 0.0, 0.0}, 0.1);
  assert(Near(acceleration_only.predicted_velocity[0], 0.3));
  assert(Near(acceleration_only.predicted_velocity[1], 0.0));
  assert(Near(acceleration_only.predicted_velocity[2], 0.0));

  return 0;
}
