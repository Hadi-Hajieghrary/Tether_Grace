#include "gpac_fault_tolerance/core/desired_geometry_redistributor.h"

#include <cassert>
#include <cmath>
#include <vector>

namespace {

bool Near(double lhs, double rhs, double tolerance = 1e-9) {
  return std::abs(lhs - rhs) <= tolerance;
}

} // namespace

int main() {
  using tether_grace::gpac_fault_tolerance::DesiredGeometryRedistributor;
  using tether_grace::gpac_fault_tolerance::DesiredGeometryRedistributorParams;
  using tether_grace::gpac_fault_tolerance::Vector3;

  const std::vector<Vector3> nominal_offsets{{1.0, 0.0, 0.0},
                                             {-0.5, 0.8660254037844386, 0.0},
                                             {-0.5, -0.8660254037844386, 0.0}};
  const std::vector<bool> cable_healthy{false, true, true};

  DesiredGeometryRedistributorParams disabled_params;
  DesiredGeometryRedistributor disabled(disabled_params);
  auto disabled_output =
      disabled.Evaluate(nominal_offsets, cable_healthy, 1.2, 1.0);
  assert(Near(disabled_output.offsets[1][0], nominal_offsets[1][0]));
  assert(Near(disabled_output.offsets[1][1], nominal_offsets[1][1]));
  assert(Near(disabled_output.offset_rates[1][0], 0.0));

  DesiredGeometryRedistributorParams params;
  params.enabled = true;
  params.ramp_duration_seconds = 0.2;
  DesiredGeometryRedistributor redistributor(params);

  const auto pre_fault =
      redistributor.Evaluate(nominal_offsets, cable_healthy, 0.9, 1.0);
  assert(Near(pre_fault.offsets[2][0], nominal_offsets[2][0]));
  assert(Near(pre_fault.offset_rates[2][1], 0.0));

  const auto mid_ramp =
      redistributor.Evaluate(nominal_offsets, cable_healthy, 1.1, 1.0);
  assert(Near(mid_ramp.offsets[1][0], -0.25));
  assert(mid_ramp.offsets[1][1] > nominal_offsets[1][1]);
  assert(mid_ramp.offset_rates[1][0] > 0.0);
  assert(mid_ramp.offset_rates[2][1] < 0.0);

  const auto post_ramp =
      redistributor.Evaluate(nominal_offsets, cable_healthy, 1.3, 1.0);
  assert(Near(post_ramp.offsets[0][0], nominal_offsets[0][0]));
  assert(Near(post_ramp.offsets[0][1], nominal_offsets[0][1]));
  assert(Near(post_ramp.offsets[1][0], 0.0, 1e-6));
  assert(Near(post_ramp.offsets[1][1], 1.0, 1e-6));
  assert(Near(post_ramp.offsets[2][0], 0.0, 1e-6));
  assert(Near(post_ramp.offsets[2][1], -1.0, 1e-6));
  assert(Near(post_ramp.offset_rates[1][0], 0.0));
  assert(Near(post_ramp.offset_rates[2][1], 0.0));

  return 0;
}