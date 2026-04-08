#pragma once

#include <array>

namespace tether_grace::gpac_fault_tolerance::contracts {

using Vector3 = std::array<double, 3>;

struct DisturbanceEstimateSignal {
  Vector3 acceleration_mps2{};
};

struct DisturbanceCompensationSignal {
  Vector3 acceleration_mps2{};
};

struct DisturbanceBoundSignal {
  Vector3 lower_mps2{};
  Vector3 upper_mps2{};
};

struct CableTensionSample {
  double tension_newtons{0.0};
  Vector3 cable_direction_body{};
};

struct CableHealthMultipliers {
  std::array<double, 8> multipliers{1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
};

} // namespace tether_grace::gpac_fault_tolerance::contracts
