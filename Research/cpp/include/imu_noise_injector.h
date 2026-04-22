#pragma once

#include <random>

#include <drake/systems/framework/basic_vector.h>
#include <drake/systems/framework/leaf_system.h>

namespace quad_rope_lift {

/// Adds band-limited Gaussian noise to a 6-vector IMU signal
/// (accel_xyz, gyro_xyz). The harness inserts this between the plant
/// state output and any controller that should experience sensor
/// imperfection. Noise standard deviations scale with
/// `params.intensity` so the ablation study can vary a single knob.
class ImuNoiseInjector final : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ImuNoiseInjector);

  struct Params {
    double accel_sigma  = 0.05;   // m/s² (1-σ)
    double gyro_sigma   = 0.01;   // rad/s (1-σ)
    double intensity    = 1.0;    // multiplier
    double update_period= 2e-4;
    int    seed         = 7;
  };

  ImuNoiseInjector(const Params& params)
      : params_(params), rng_(params.seed), gauss_(0.0, 1.0) {
    in_port_  = DeclareVectorInputPort("clean",
                    drake::systems::BasicVector<double>(6)).get_index();
    out_port_ = DeclareVectorOutputPort(
        "noisy", drake::systems::BasicVector<double>(6),
        &ImuNoiseInjector::CalcOutput).get_index();
    DeclarePeriodicUnrestrictedUpdateEvent(
        params_.update_period, 0.0, &ImuNoiseInjector::Refresh);
    Eigen::VectorXd init = Eigen::VectorXd::Zero(6);
    noise_state_ = DeclareDiscreteState(init);
  }

  const drake::systems::InputPort<double>& get_clean_input_port() const {
    return get_input_port(in_port_);
  }
  const drake::systems::OutputPort<double>& get_noisy_output_port() const {
    return get_output_port(out_port_);
  }

 private:
  drake::systems::EventStatus Refresh(
      const drake::systems::Context<double>&,
      drake::systems::State<double>* state) const {
    auto& mut_vec = state->get_mutable_discrete_state(noise_state_);
    auto xd = mut_vec.get_mutable_value();
    const double sa = params_.accel_sigma * params_.intensity;
    const double sg = params_.gyro_sigma  * params_.intensity;
    for (int i = 0; i < 3; ++i) xd[i]   = sa * gauss_(rng_);
    for (int i = 0; i < 3; ++i) xd[3+i] = sg * gauss_(rng_);
    return drake::systems::EventStatus::Succeeded();
  }

  void CalcOutput(const drake::systems::Context<double>& ctx,
                  drake::systems::BasicVector<double>* out) const {
    const auto& clean = get_input_port(in_port_).Eval(ctx);
    const auto& n     = ctx.get_discrete_state(noise_state_).value();
    for (int i = 0; i < 6; ++i) out->SetAtIndex(i, clean[i] + n[i]);
  }

  Params params_;
  int in_port_{-1};
  int out_port_{-1};
  drake::systems::DiscreteStateIndex noise_state_;
  mutable std::mt19937_64 rng_;
  mutable std::normal_distribution<double> gauss_;
};

}  // namespace quad_rope_lift
