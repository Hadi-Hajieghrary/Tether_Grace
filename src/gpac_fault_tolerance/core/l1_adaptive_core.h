#pragma once

#include <array>

namespace tether_grace::gpac_fault_tolerance {

using Vector3 = std::array<double, 3>;

struct L1AdaptiveParams {
  double filter_bandwidth{20.0};
  double predictor_pole{50.0};
  double input_gain{1.0};
  double sigma_limit{20.0};
};

struct L1AdaptiveState {
  Vector3 predicted_position{};
  Vector3 predicted_velocity{};
  Vector3 sigma_hat{};
  Vector3 compensation{};
  Vector3 prediction_error{};
};

class L1AdaptiveCore {
public:
  explicit L1AdaptiveCore(const L1AdaptiveParams &params = {});

  void Reset(const Vector3 &position,
             const Vector3 &velocity = {0.0, 0.0, 0.0});
  const L1AdaptiveState &Update(const Vector3 &measured_position,
                                const Vector3 &measured_velocity,
                                const Vector3 &commanded_acceleration,
                                double dt);

  const L1AdaptiveState &state() const;
  void set_state(const L1AdaptiveState &state);
  const L1AdaptiveParams &params() const;
  void set_params(const L1AdaptiveParams &params);

private:
  L1AdaptiveParams params_;
  L1AdaptiveState state_;
};

} // namespace tether_grace::gpac_fault_tolerance
