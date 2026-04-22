#pragma once

#include <cmath>
#include <random>

#include <Eigen/Core>
#include <drake/systems/framework/basic_vector.h>
#include <drake/systems/framework/leaf_system.h>

namespace quad_rope_lift {

/// Dryden turbulence model: a stationary Gaussian process with the
/// Dryden spectral density, sampled by passing white noise through
/// the Dryden shaping filter. Outputs a 3-vector velocity gust that
/// the harness adds to each drone's world-frame velocity feed-forward.
///
/// Inputs:  none (the model is stochastic, driven by an internal seed).
/// Outputs: `gust_velocity` (3-vector, m/s).
///
/// Parameters follow MIL-HDBK-1797 Section 5.2.3 with low-altitude
/// constants.
class DrydenWindModel final : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DrydenWindModel);

  struct Params {
    double mean_wind_speed = 0.0;   // m/s
    double altitude        = 10.0;   // m (above ground level)
    double intensity_scale = 1.0;    // multiplies σ_u, σ_v, σ_w
    double update_period   = 2e-3;   // s
    int    seed            = 1;
  };

  explicit DrydenWindModel(const Params& params)
      : params_(params), rng_(params.seed), gauss_(0.0, 1.0) {
    gust_port_ = DeclareVectorOutputPort(
        "gust_velocity", drake::systems::BasicVector<double>(3),
        &DrydenWindModel::CalcGust).get_index();
    Eigen::VectorXd init = Eigen::VectorXd::Zero(3);
    state_index_ = DeclareDiscreteState(init);
    DeclarePeriodicUnrestrictedUpdateEvent(
        params_.update_period, 0.0, &DrydenWindModel::UpdateGust);
  }

  const drake::systems::OutputPort<double>& get_gust_output_port() const {
    return get_output_port(gust_port_);
  }

 private:
  double TurbulenceLengthScale(int axis) const {
    // MIL-HDBK-1797 low-altitude scale lengths.
    const double h = std::max(params_.altitude, 2.0);
    if (axis == 2) return h;
    return h / std::pow(0.177 + 0.000823 * h, 1.2);
  }

  double TurbulenceSigma(int axis) const {
    const double h = std::max(params_.altitude, 2.0);
    const double sigma_w = params_.intensity_scale * 0.1 * params_.mean_wind_speed;
    if (axis == 2) return sigma_w;
    return sigma_w / std::pow(0.177 + 0.000823 * h, 0.4);
  }

  drake::systems::EventStatus UpdateGust(
      const drake::systems::Context<double>&,
      drake::systems::State<double>* state) const {
    auto& mut_vec = state->get_mutable_discrete_state(state_index_);
    auto xd = mut_vec.get_mutable_value();
    const double Ts = params_.update_period;
    const double V  = std::max(params_.mean_wind_speed, 0.5);
    for (int a = 0; a < 3; ++a) {
      const double L = TurbulenceLengthScale(a);
      const double sigma = TurbulenceSigma(a);
      const double alpha = std::exp(-V * Ts / L);
      const double noise = gauss_(rng_);
      xd[a] = alpha * xd[a] + sigma * std::sqrt(1.0 - alpha * alpha) * noise;
    }
    return drake::systems::EventStatus::Succeeded();
  }

  void CalcGust(const drake::systems::Context<double>& ctx,
                drake::systems::BasicVector<double>* out) const {
    const auto& xd = ctx.get_discrete_state(state_index_).value();
    out->SetAtIndex(0, xd[0]);
    out->SetAtIndex(1, xd[1]);
    out->SetAtIndex(2, xd[2]);
  }

  Params params_;
  int gust_port_{-1};
  drake::systems::DiscreteStateIndex state_index_;
  mutable std::mt19937_64 rng_;
  mutable std::normal_distribution<double> gauss_;
};

}  // namespace quad_rope_lift
