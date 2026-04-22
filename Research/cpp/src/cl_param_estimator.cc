#include "cl_param_estimator.h"

#include <algorithm>
#include <cmath>

#include <drake/common/value.h>

namespace quad_rope_lift {

using drake::multibody::MultibodyPlant;
using drake::multibody::RigidBody;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::EventStatus;
using drake::systems::State;

namespace {
constexpr double kGravity = 9.81;
constexpr int kStateSize = 7;
constexpr int kThetaM = 0;
constexpr int kThetaK = 1;
constexpr int kPrevVLz = 2;
constexpr int kFilteredALz = 3;
constexpr int kLastInnovation = 4;
constexpr int kLastRankMargin = 5;
constexpr int kLastDelta = 6;
}  // namespace

CLParamEstimator::CLParamEstimator(
    const MultibodyPlant<double>& plant,
    const RigidBody<double>& quadcopter_body,
    const RigidBody<double>& payload_body,
    const Eigen::Vector3d& quadcopter_attachment_point,
    const Eigen::Vector3d& payload_attachment_point,
    const CLParamEstimatorParams& params)
    : plant_(plant),
      quad_body_index_(quadcopter_body.index()),
      payload_body_index_(payload_body.index()),
      quad_attach_(quadcopter_attachment_point),
      payload_attach_(payload_attachment_point),
      params_(params) {
  plant_state_port_ =
      DeclareVectorInputPort(
          "plant_state",
          BasicVector<double>(plant.num_positions() + plant.num_velocities()))
          .get_index();
  tension_port_ =
      DeclareVectorInputPort("rope_tension", BasicVector<double>(4))
          .get_index();

  // Discrete state — theta_hat initialised from params.
  Eigen::VectorXd s0 = Eigen::VectorXd::Zero(kStateSize);
  s0[kThetaM] = params_.initial_theta[0];
  s0[kThetaK] = params_.initial_theta[1];
  state_index_ = DeclareDiscreteState(s0);

  history_index_ = DeclareAbstractState(
      drake::Value<std::deque<CLHistoryPoint>>());

  DeclarePeriodicUnrestrictedUpdateEvent(params_.update_period, 0.0,
                                         &CLParamEstimator::Update);

  theta_hat_port_ =
      DeclareVectorOutputPort("theta_hat", BasicVector<double>(2),
                              &CLParamEstimator::CalcThetaHat).get_index();
  innovation_port_ =
      DeclareVectorOutputPort("innovation", BasicVector<double>(1),
                              &CLParamEstimator::CalcInnovation).get_index();
  rank_margin_port_ =
      DeclareVectorOutputPort("rank_margin", BasicVector<double>(1),
                              &CLParamEstimator::CalcRankMargin).get_index();
  history_size_port_ =
      DeclareVectorOutputPort("history_size", BasicVector<double>(1),
                              &CLParamEstimator::CalcHistorySize).get_index();
}

void CLParamEstimator::ComputeSample(
    const Context<double>& context, Eigen::Vector2d* /*unused_Y*/,
    double* T_out, double* delta_out, Eigen::Vector3d* v_L_out) const {
  const auto& state_vec = get_input_port(plant_state_port_).Eval(context);
  const auto& tension = get_input_port(tension_port_).Eval(context);

  std::lock_guard<std::mutex> lock(ctx_mtx_);
  if (!plant_context_) plant_context_ = plant_.CreateDefaultContext();
  plant_.SetPositionsAndVelocities(plant_context_.get(), state_vec);

  const auto& quad_body = plant_.get_body(quad_body_index_);
  const auto& payload_body = plant_.get_body(payload_body_index_);
  const auto& X_Q = plant_.EvalBodyPoseInWorld(*plant_context_, quad_body);
  const auto& X_L = plant_.EvalBodyPoseInWorld(*plant_context_, payload_body);
  const auto& V_L =
      plant_.EvalBodySpatialVelocityInWorld(*plant_context_, payload_body);

  // Rope end-points in world frame. Stretch δ ≥ 0 (tension-only rope).
  const Eigen::Vector3d p_qattach = X_Q * quad_attach_;
  const Eigen::Vector3d p_lattach = X_L * payload_attach_;
  const double chord = (p_qattach - p_lattach).norm();
  const double delta = std::max(0.0, chord - params_.rope_rest_length);

  *delta_out = delta;
  *T_out = tension[0];
  *v_L_out = V_L.translational();
}

EventStatus CLParamEstimator::Update(const Context<double>& context,
                                     State<double>* state) const {
  const auto& discrete = context.get_discrete_state(state_index_).value();
  double theta_m = discrete[kThetaM];   // m_L / N_alive
  double theta_k = discrete[kThetaK];   // k_rope
  double prev_v_L_z = discrete[kPrevVLz];
  double filtered_a_L_z = discrete[kFilteredALz];

  // Sample acquisition (δ, T, v_L).
  Eigen::Vector2d dummy;
  double T_meas = 0.0, delta = 0.0;
  Eigen::Vector3d v_L;
  ComputeSample(context, &dummy, &T_meas, &delta, &v_L);

  // Numerical differentiation + low-pass smoothing of a_L_z.
  const double dt = params_.update_period;
  const double raw_a_L_z = (v_L.z() - prev_v_L_z) / dt;
  const double alpha = dt / (params_.accel_filter_tau + dt);
  filtered_a_L_z = alpha * raw_a_L_z + (1.0 - alpha) * filtered_a_L_z;

  // cos β ≈ Δz_attach / L_chord (geometric projection of the rope
  // tension onto the vertical axis). Use the current geometry since
  // it is locally observable. Clamp to avoid 1/0 when the rope is
  // momentarily horizontal.
  const double L_chord = params_.rope_rest_length + delta;
  const double dz_attach = std::sqrt(
      std::max(1e-9, L_chord * L_chord
                         - params_.formation_radius * params_.formation_radius
                               * 0.49));  // r_eff = 0.7·r_f, r_eff² = 0.49·r_f²
  const double cos_beta = std::clamp(dz_attach / std::max(L_chord, 1e-6),
                                     0.1, 1.0);

  // ---- Channel M: mass share via payload-EOM projection ----
  //   m_meas = T · cos β,  Y_m = a_L_z + g,  θ_m = m_L / N_alive
  const double Y_m = filtered_a_L_z + kGravity;
  const double meas_m = T_meas * cos_beta;
  const double eps_m = meas_m - Y_m * theta_m;
  double theta_m_dot = params_.gamma[0] * Y_m * eps_m;

  // ---- Channel K: rope stiffness via axial spring law ----
  //   k_meas = T,  Y_k = δ,  θ_k = k_rope
  const double Y_k = delta;
  const double meas_k = T_meas;
  const double eps_k = meas_k - Y_k * theta_k;
  double theta_k_dot = params_.gamma[1] * Y_k * eps_k;

  // Concurrent-learning contribution: accumulate over history-buffer
  // samples to guarantee convergence without persistent instantaneous
  // excitation (Chowdhary-Johnson 2011).
  const auto& history =
      context.get_abstract_state<std::deque<CLHistoryPoint>>(history_index_);
  if (params_.rho > 0.0 && !history.empty()) {
    double cl_m = 0.0, cl_k = 0.0;
    for (const auto& h : history) {
      cl_m += h.Y_m * (h.meas_m - h.Y_m * theta_m);
      cl_k += h.Y_k * (h.meas_k - h.Y_k * theta_k);
    }
    theta_m_dot += params_.rho * params_.gamma[0] * cl_m;
    theta_k_dot += params_.rho * params_.gamma[1] * cl_k;
  }

  theta_m = std::clamp(theta_m + theta_m_dot * dt,
                       params_.m_min, params_.m_max);
  theta_k = std::clamp(theta_k + theta_k_dot * dt,
                       params_.k_min, params_.k_max);

  // Joint innovation for diagnostics: the two channels are weighted
  // equally in the RMS sense after scaling by the nominal values.
  const double innovation_combined = 0.5 * (eps_m / std::max(kGravity, 1e-3)
                                             + eps_k / params_.k_max);

  // History admission: keep the sample if it provides excitation in
  // EITHER channel. We use scalar variance (second moment) of each
  // channel's regressor across the buffer as the “rank margin”.
  auto& mutable_history =
      state->get_mutable_abstract_state<std::deque<CLHistoryPoint>>(
          history_index_);
  const bool excited_m = std::abs(Y_m) > params_.min_excitation;
  const bool excited_k = Y_k > 0.5e-3;  // ≥ 0.5 mm stretch
  if (excited_m || excited_k) {
    // Cheap informativeness check: require the new Y to differ from
    // the buffer mean by > σ_threshold in at least one channel.
    double mean_m = 0.0, mean_k = 0.0;
    if (!history.empty()) {
      for (const auto& h : history) {
        mean_m += h.Y_m;
        mean_k += h.Y_k;
      }
      mean_m /= history.size();
      mean_k /= history.size();
    }
    const double sigma_m = std::abs(Y_m - mean_m);
    const double sigma_k = std::abs(Y_k - mean_k);
    if (history.empty()
        || sigma_m > params_.rank_gain_threshold
        || sigma_k > 1.0e-4) {
      CLHistoryPoint h;
      h.time = context.get_time();
      h.Y_m = Y_m;
      h.meas_m = meas_m;
      h.Y_k = Y_k;
      h.meas_k = meas_k;
      mutable_history.push_back(h);
      while (static_cast<int>(mutable_history.size()) >
             params_.max_history_size) {
        mutable_history.pop_front();
      }
    }
  }

  // Rank margin diagnostic: minimum of the two second-moments.
  auto second_moment = [&](auto accessor) {
    double s2 = 0.0;
    for (const auto& h : mutable_history) {
      const double y = accessor(h);
      s2 += y * y;
    }
    return s2;
  };
  const double lam_m = second_moment([](const CLHistoryPoint& h) { return h.Y_m; });
  const double lam_k = second_moment([](const CLHistoryPoint& h) { return h.Y_k; });
  const double rank_margin_diag = std::min(lam_m, lam_k);

  // Persist state + diagnostics.
  auto& out = state->get_mutable_discrete_state(state_index_);
  out[kThetaM] = theta_m;
  out[kThetaK] = theta_k;
  out[kPrevVLz] = v_L.z();
  out[kFilteredALz] = filtered_a_L_z;
  out[kLastInnovation] = innovation_combined;
  out[kLastRankMargin] = rank_margin_diag;
  out[kLastDelta] = delta;
  return EventStatus::Succeeded();
}

void CLParamEstimator::CalcThetaHat(const Context<double>& context,
                                    BasicVector<double>* out) const {
  const auto& s = context.get_discrete_state(state_index_).value();
  out->SetAtIndex(0, s[kThetaM]);
  out->SetAtIndex(1, s[kThetaK]);
}
void CLParamEstimator::CalcInnovation(const Context<double>& context,
                                      BasicVector<double>* out) const {
  out->SetAtIndex(
      0, context.get_discrete_state(state_index_).value()[kLastInnovation]);
}
void CLParamEstimator::CalcRankMargin(const Context<double>& context,
                                      BasicVector<double>* out) const {
  out->SetAtIndex(
      0, context.get_discrete_state(state_index_).value()[kLastRankMargin]);
}
void CLParamEstimator::CalcHistorySize(const Context<double>& context,
                                       BasicVector<double>* out) const {
  const auto& history =
      context.get_abstract_state<std::deque<CLHistoryPoint>>(history_index_);
  out->SetAtIndex(0, static_cast<double>(history.size()));
}

}  // namespace quad_rope_lift
