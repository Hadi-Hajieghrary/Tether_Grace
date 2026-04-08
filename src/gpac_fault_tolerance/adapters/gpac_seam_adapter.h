#pragma once

#include "gpac_fault_tolerance/contracts/signal_contracts.h"
#include "gpac_fault_tolerance/core/l1_adaptive_core.h"

namespace tether_grace::gpac_fault_tolerance::adapters {

using contracts::DisturbanceBoundSignal;
using contracts::DisturbanceCompensationSignal;
using contracts::DisturbanceEstimateSignal;
using contracts::Vector3;

struct GpacSeamAdapterParams {
  L1AdaptiveParams l1_params{};
  double disturbance_margin_mps2{2.0};
  double disturbance_coupling{1.5};
};

struct GpacSeamAdapterOutput {
  DisturbanceEstimateSignal controller_disturbance_estimate{};
  DisturbanceCompensationSignal filtered_compensation{};
  DisturbanceBoundSignal safety_disturbance_bound{};
  Vector3 raw_sigma_hat{};
  Vector3 prediction_error{};
};

class GpacSeamAdapter {
public:
  explicit GpacSeamAdapter(const GpacSeamAdapterParams &params = {});

  void Reset(const Vector3 &position,
             const Vector3 &velocity = {0.0, 0.0, 0.0});
  const GpacSeamAdapterOutput &Update(const Vector3 &measured_position,
                                      const Vector3 &measured_velocity,
                                      const Vector3 &commanded_acceleration,
                                      double dt);

  const GpacSeamAdapterOutput &output() const;
  const GpacSeamAdapterParams &params() const;

private:
  void RefreshOutputFromCore();

  GpacSeamAdapterParams params_;
  L1AdaptiveCore l1_core_;
  GpacSeamAdapterOutput output_{};
};

} // namespace tether_grace::gpac_fault_tolerance::adapters
