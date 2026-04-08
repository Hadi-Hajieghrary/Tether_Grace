#include "gpac_fault_tolerance/core/desired_geometry_redistributor.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace tether_grace::gpac_fault_tolerance {
namespace {

double SquaredNormXY(const Vector3 &value) {
  return value[0] * value[0] + value[1] * value[1];
}

} // namespace

DesiredGeometryRedistributor::DesiredGeometryRedistributor(
    const DesiredGeometryRedistributorParams &params)
    : params_(params) {}

DesiredGeometryRedistributorOutput DesiredGeometryRedistributor::Evaluate(
    const std::vector<Vector3> &nominal_offsets,
    const std::vector<bool> &cable_healthy, double time_seconds,
    double fault_time_seconds) const {
  DesiredGeometryRedistributorOutput output;
  output.offsets = nominal_offsets;
  output.offset_rates.assign(nominal_offsets.size(), Vector3{0.0, 0.0, 0.0});

  if (!params_.enabled || nominal_offsets.size() != cable_healthy.size() ||
      time_seconds < fault_time_seconds) {
    return output;
  }

  std::vector<int> healthy_indices;
  healthy_indices.reserve(cable_healthy.size());
  for (std::size_t index = 0; index < cable_healthy.size(); ++index) {
    if (cable_healthy[index]) {
      healthy_indices.push_back(static_cast<int>(index));
    }
  }
  if (healthy_indices.size() < 2) {
    return output;
  }

  const double ramp_duration = std::max(params_.ramp_duration_seconds, 1e-6);
  const double blend =
      std::clamp((time_seconds - fault_time_seconds) / ramp_duration, 0.0, 1.0);
  const double blend_rate =
      (blend > 0.0 && blend < 1.0) ? (1.0 / ramp_duration) : 0.0;

  Vector3 centroid{0.0, 0.0, 0.0};
  double original_squared_radius_sum = 0.0;
  for (int healthy_index : healthy_indices) {
    centroid[0] += nominal_offsets[healthy_index][0];
    centroid[1] += nominal_offsets[healthy_index][1];
    original_squared_radius_sum +=
        SquaredNormXY(nominal_offsets[healthy_index]);
  }
  centroid[0] /= static_cast<double>(healthy_indices.size());
  centroid[1] /= static_cast<double>(healthy_indices.size());

  std::vector<Vector3> centered_offsets(nominal_offsets.size(),
                                        Vector3{0.0, 0.0, 0.0});
  double centered_squared_radius_sum = 0.0;
  for (int healthy_index : healthy_indices) {
    centered_offsets[healthy_index] = nominal_offsets[healthy_index];
    centered_offsets[healthy_index][0] -= centroid[0];
    centered_offsets[healthy_index][1] -= centroid[1];
    centered_squared_radius_sum +=
        SquaredNormXY(centered_offsets[healthy_index]);
  }
  const double scale = std::sqrt(original_squared_radius_sum /
                                 (centered_squared_radius_sum + 1e-9));

  std::vector<Vector3> target_offsets = nominal_offsets;
  for (int healthy_index : healthy_indices) {
    target_offsets[healthy_index][0] =
        scale * centered_offsets[healthy_index][0];
    target_offsets[healthy_index][1] =
        scale * centered_offsets[healthy_index][1];
  }

  for (int healthy_index : healthy_indices) {
    for (int axis = 0; axis < 3; ++axis) {
      const double delta = target_offsets[healthy_index][axis] -
                           nominal_offsets[healthy_index][axis];
      output.offsets[healthy_index][axis] =
          nominal_offsets[healthy_index][axis] + blend * delta;
      output.offset_rates[healthy_index][axis] = blend_rate * delta;
    }
  }
  return output;
}

const DesiredGeometryRedistributorParams &
DesiredGeometryRedistributor::params() const {
  return params_;
}

} // namespace tether_grace::gpac_fault_tolerance