#pragma once

#include <array>
#include <vector>

#include "gpac_fault_tolerance/core/desired_geometry_redistributor.h"

namespace tether_grace::gpac_fault_tolerance {

struct LoadReferenceSample {
  Vector3 position{0.0, 0.0, 0.0};
  Vector3 velocity{0.0, 0.0, 0.0};
  Vector3 acceleration{0.0, 0.0, 0.0};
};

struct DroneReferenceSample {
  Vector3 position{0.0, 0.0, 0.0};
  Vector3 velocity{0.0, 0.0, 0.0};
  Vector3 acceleration{0.0, 0.0, 0.0};
};

using DroneTrajectoryVector = std::array<double, 9>;

DroneReferenceSample ComposeDroneReferenceSample(
    const LoadReferenceSample &load_reference, const Vector3 &planar_offset,
    const Vector3 &planar_offset_rate, double cable_length_m);

std::vector<DroneReferenceSample> ComposeDroneReferenceSamples(
    const LoadReferenceSample &load_reference,
    const DesiredGeometryRedistributorOutput &redistributed_geometry,
    const std::vector<double> &cable_lengths_m);

DroneTrajectoryVector
FlattenDroneReferenceSample(const DroneReferenceSample &drone_reference);

std::vector<DroneTrajectoryVector> FlattenDroneReferenceSamples(
    const std::vector<DroneReferenceSample> &drone_references);

} // namespace tether_grace::gpac_fault_tolerance