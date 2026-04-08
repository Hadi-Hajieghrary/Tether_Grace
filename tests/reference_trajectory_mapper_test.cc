#include "gpac_fault_tolerance/core/reference_trajectory_mapper.h"

#include <cassert>
#include <cmath>
#include <vector>

namespace {

bool Near(double lhs, double rhs, double tolerance = 1e-9) {
  return std::abs(lhs - rhs) <= tolerance;
}

} // namespace

int main() {
  using tether_grace::gpac_fault_tolerance::ComposeDroneReferenceSample;
  using tether_grace::gpac_fault_tolerance::ComposeDroneReferenceSamples;
  using tether_grace::gpac_fault_tolerance::DesiredGeometryRedistributorOutput;
  using tether_grace::gpac_fault_tolerance::DroneReferenceSample;
  using tether_grace::gpac_fault_tolerance::FlattenDroneReferenceSample;
  using tether_grace::gpac_fault_tolerance::FlattenDroneReferenceSamples;
  using tether_grace::gpac_fault_tolerance::LoadReferenceSample;
  using tether_grace::gpac_fault_tolerance::Vector3;

  LoadReferenceSample load_reference;
  load_reference.position = {1.0, 2.0, 3.0};
  load_reference.velocity = {0.1, 0.2, 0.3};
  load_reference.acceleration = {0.4, 0.5, 0.6};

  const DroneReferenceSample single = ComposeDroneReferenceSample(
      load_reference, Vector3{0.5, -0.25, 0.0}, Vector3{0.05, -0.10, 0.0}, 1.2);
  assert(Near(single.position[0], 1.5));
  assert(Near(single.position[1], 1.75));
  assert(Near(single.position[2], 4.2));
  assert(Near(single.velocity[0], 0.15));
  assert(Near(single.velocity[1], 0.10));
  assert(Near(single.velocity[2], 0.3));
  assert(Near(single.acceleration[2], 0.6));

  const auto flattened_single = FlattenDroneReferenceSample(single);
  assert(Near(flattened_single[0], 1.5));
  assert(Near(flattened_single[4], 0.10));
  assert(Near(flattened_single[8], 0.6));

  DesiredGeometryRedistributorOutput redistributed_geometry;
  redistributed_geometry.offsets = {
      Vector3{0.0, 1.0, 0.0},
      Vector3{0.0, -1.0, 0.0},
  };
  redistributed_geometry.offset_rates = {
      Vector3{0.2, 0.0, 0.0},
      Vector3{-0.2, 0.0, 0.0},
  };
  const std::vector<double> cable_lengths{1.0, 1.5};
  const auto samples = ComposeDroneReferenceSamples(
      load_reference, redistributed_geometry, cable_lengths);
  assert(samples.size() == 2);
  assert(Near(samples[0].position[1], 3.0));
  assert(Near(samples[1].position[1], 1.0));
  assert(Near(samples[0].position[2], 4.0));
  assert(Near(samples[1].position[2], 4.5));
  assert(Near(samples[0].velocity[0], 0.3));
  assert(Near(samples[1].velocity[0], -0.1));

  const auto flattened = FlattenDroneReferenceSamples(samples);
  assert(flattened.size() == 2);
  assert(Near(flattened[0][3], 0.3));
  assert(Near(flattened[1][2], 4.5));

  return 0;
}