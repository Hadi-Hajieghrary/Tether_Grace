#include "gpac_fault_tolerance/core/reference_trajectory_mapper.h"

#include <algorithm>

namespace tether_grace::gpac_fault_tolerance {

DroneReferenceSample ComposeDroneReferenceSample(
    const LoadReferenceSample &load_reference, const Vector3 &planar_offset,
    const Vector3 &planar_offset_rate, double cable_length_m) {
  DroneReferenceSample sample;
  sample.position = load_reference.position;
  sample.velocity = load_reference.velocity;
  sample.acceleration = load_reference.acceleration;

  for (int axis = 0; axis < 3; ++axis) {
    sample.position[axis] += planar_offset[axis];
    sample.velocity[axis] += planar_offset_rate[axis];
  }
  sample.position[2] += cable_length_m;
  return sample;
}

std::vector<DroneReferenceSample> ComposeDroneReferenceSamples(
    const LoadReferenceSample &load_reference,
    const DesiredGeometryRedistributorOutput &redistributed_geometry,
    const std::vector<double> &cable_lengths_m) {
  const std::size_t count =
      std::min(redistributed_geometry.offsets.size(), cable_lengths_m.size());
  std::vector<DroneReferenceSample> samples;
  samples.reserve(count);
  for (std::size_t index = 0; index < count; ++index) {
    const Vector3 zero_rate{0.0, 0.0, 0.0};
    const Vector3 &offset_rate =
        index < redistributed_geometry.offset_rates.size()
            ? redistributed_geometry.offset_rates[index]
            : zero_rate;
    samples.push_back(ComposeDroneReferenceSample(
        load_reference, redistributed_geometry.offsets[index], offset_rate,
        cable_lengths_m[index]));
  }
  return samples;
}

DroneTrajectoryVector
FlattenDroneReferenceSample(const DroneReferenceSample &drone_reference) {
  return {
      drone_reference.position[0],     drone_reference.position[1],
      drone_reference.position[2],     drone_reference.velocity[0],
      drone_reference.velocity[1],     drone_reference.velocity[2],
      drone_reference.acceleration[0], drone_reference.acceleration[1],
      drone_reference.acceleration[2],
  };
}

std::vector<DroneTrajectoryVector> FlattenDroneReferenceSamples(
    const std::vector<DroneReferenceSample> &drone_references) {
  std::vector<DroneTrajectoryVector> flattened;
  flattened.reserve(drone_references.size());
  for (const auto &drone_reference : drone_references) {
    flattened.push_back(FlattenDroneReferenceSample(drone_reference));
  }
  return flattened;
}

} // namespace tether_grace::gpac_fault_tolerance