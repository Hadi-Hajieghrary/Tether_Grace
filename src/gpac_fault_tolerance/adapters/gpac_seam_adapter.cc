#include "gpac_fault_tolerance/adapters/gpac_seam_adapter.h"

#include <cmath>

namespace tether_grace::gpac_fault_tolerance::adapters {

namespace {

double BoundComponent(double sigma_hat, double margin, double coupling) {
  return margin + coupling * std::abs(sigma_hat);
}

} // namespace

GpacSeamAdapter::GpacSeamAdapter(const GpacSeamAdapterParams &params)
    : params_(params), l1_core_(params.l1_params) {}

void GpacSeamAdapter::Reset(const Vector3 &position, const Vector3 &velocity) {
  l1_core_.Reset(position, velocity);
  RefreshOutputFromCore();
}

const GpacSeamAdapterOutput &
GpacSeamAdapter::Update(const Vector3 &measured_position,
                        const Vector3 &measured_velocity,
                        const Vector3 &commanded_acceleration, double dt) {
  l1_core_.Update(measured_position, measured_velocity, commanded_acceleration,
                  dt);
  RefreshOutputFromCore();
  return output_;
}

const GpacSeamAdapterOutput &GpacSeamAdapter::output() const { return output_; }

const GpacSeamAdapterParams &GpacSeamAdapter::params() const { return params_; }

void GpacSeamAdapter::RefreshOutputFromCore() {
  const auto &state = l1_core_.state();
  output_.raw_sigma_hat = state.sigma_hat;
  output_.prediction_error = state.prediction_error;
  output_.filtered_compensation.acceleration_mps2 = state.compensation;

  // The active GPAC controller seam expects acceleration units and already
  // applies mass scaling. Feed the filtered L1 compensation here, not
  // cable-tension terms that are already handled elsewhere.
  output_.controller_disturbance_estimate.acceleration_mps2 =
      state.compensation;

  for (int axis = 0; axis < 3; ++axis) {
    const double bound =
        BoundComponent(state.sigma_hat[axis], params_.disturbance_margin_mps2,
                       params_.disturbance_coupling);
    output_.safety_disturbance_bound.lower_mps2[axis] = -bound;
    output_.safety_disturbance_bound.upper_mps2[axis] = bound;
  }
}

} // namespace tether_grace::gpac_fault_tolerance::adapters
