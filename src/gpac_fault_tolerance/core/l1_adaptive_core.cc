#include "gpac_fault_tolerance/core/l1_adaptive_core.h"

#include <algorithm>
#include <cmath>

namespace tether_grace::gpac_fault_tolerance {

namespace {

double Clip(double value, double limit) {
  return std::clamp(value, -limit, limit);
}

} // namespace

L1AdaptiveCore::L1AdaptiveCore(const L1AdaptiveParams &params)
    : params_(params) {}

void L1AdaptiveCore::Reset(const Vector3 &position, const Vector3 &velocity) {
  state_.predicted_position = position;
  state_.predicted_velocity = velocity;
  state_.sigma_hat = {0.0, 0.0, 0.0};
  state_.compensation = {0.0, 0.0, 0.0};
  state_.prediction_error = {0.0, 0.0, 0.0};
}

const L1AdaptiveState &
L1AdaptiveCore::Update(const Vector3 &measured_position,
                       const Vector3 &measured_velocity,
                       const Vector3 &commanded_acceleration, double dt) {
  const double alpha = std::exp(-params_.filter_bandwidth * dt);
  const double pole = params_.predictor_pole;

  for (int axis = 0; axis < 3; ++axis) {
    const double position_error =
        measured_position[axis] - state_.predicted_position[axis];
    state_.predicted_position[axis] +=
        (state_.predicted_velocity[axis] + 2.0 * pole * position_error) * dt;
    state_.predicted_velocity[axis] +=
        (state_.sigma_hat[axis] +
         params_.input_gain * commanded_acceleration[axis] +
         pole * pole * position_error) *
        dt;

    state_.prediction_error[axis] =
        state_.predicted_position[axis] - measured_position[axis];
    const double velocity_error =
        state_.predicted_velocity[axis] - measured_velocity[axis];
    const double sigma_raw =
        -pole * velocity_error - pole * pole * state_.prediction_error[axis];

    state_.sigma_hat[axis] = Clip(sigma_raw, params_.sigma_limit);
    state_.compensation[axis] = alpha * state_.compensation[axis] +
                                (1.0 - alpha) * (-state_.sigma_hat[axis]);
  }

  return state_;
}

const L1AdaptiveState &L1AdaptiveCore::state() const { return state_; }

void L1AdaptiveCore::set_state(const L1AdaptiveState &state) { state_ = state; }

const L1AdaptiveParams &L1AdaptiveCore::params() const { return params_; }

void L1AdaptiveCore::set_params(const L1AdaptiveParams &params) {
  params_ = params;
}

} // namespace tether_grace::gpac_fault_tolerance
