#pragma once

#include <memory>
#include <mutex>

#include <Eigen/Core>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/basic_vector.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/framework/leaf_system.h>

namespace quad_rope_lift {

/// Parameters for the 2-D extended concurrent-learning parameter
/// estimator. Tracks per-drone payload mass share and rope stiffness
/// online from the local scalar rope tension.
struct CLParamEstimatorParams {
  /// Adaptation gain on the stochastic term (gradient scaling).
  /// 2-D diagonal, one per channel. Tuned to give a ~0.5 s settling
  /// time under typical regressor magnitudes (Y_m ≈ 10, Y_k ≈ 3e-3):
  ///   θ_m_dot ≈ γ_m · Y_m · ε_m = 0.2 · 10 · 1 = 2 kg/s ⇒ settles 1 kg
  ///     bias in 0.5 s.
  ///   θ_k_dot ≈ γ_k · Y_k · ε_k = 2e6 · 3e-3 · 1 = 6e3 N/m·s ⇒ settles
  ///     a 1 kN/m bias in 0.17 s.
  Eigen::Vector2d gamma = (Eigen::Vector2d() << 0.2, 2.0e6).finished();

  /// Concurrent-learning weight on the historical term (dimensionless).
  double rho = 0.1;

  /// Low-pass filter time-constant on the numerically-differentiated
  /// payload acceleration (seconds). Matches the baseline estimator.
  double accel_filter_tau = 0.05;

  /// Physical projection bounds for theta = [m_hat, k_rope_hat].
  double m_min = 0.1;   // kg  (per-drone share lower bound)
  double m_max = 20.0;  // kg
  double k_min = 500.0;   // N/m
  double k_max = 20000.0; // N/m  (bead-chain effective stiffness ≈ 2778)

  /// Initial parameter guess.
  Eigen::Vector2d initial_theta =
      (Eigen::Vector2d() << 0.75, 2778.0).finished();

  /// History-buffer configuration (concurrent learning).
  int    max_history_size = 50;

  /// Minimum increase in lambda_min(Sum Y_j^T Y_j) required to admit
  /// a new data point to the buffer. Guards against adding linearly
  /// dependent rows.
  double rank_gain_threshold = 1.0e-3;

  /// Minimum regressor norm for a sample to be considered "excited".
  double min_excitation = 0.5;

  /// Rate of the discrete update (seconds).
  double update_period = 2.0e-4;

  /// Nominal rope rest length — needed to compute the stretch delta.
  double rope_rest_length = 1.25;

  /// Per-drone formation radius used to compute the chord between
  /// drone and payload attachments (horizontal component is
  /// 0.7·r_formation because of the 0.3·r payload attachment offset).
  double formation_radius = 0.8;

  /// Vertical drone-attachment offset (drone attachment sits
  /// |quad_attachment_offset.z()| below the drone body centre).
  double quad_attachment_z_offset = 0.09;  // m
};

/// One history-buffer entry. Stores two 1-D regressor/measurement
/// pairs (one per estimated parameter) to drive two *independent*
/// concurrent-learning updates that share a single history buffer.
struct CLHistoryPoint {
  double time;
  // Channel M (payload-EOM-derived mass share, m_L / N):
  //   measurement_m = T · cos β  ,  regressor Y_m = (a_L_z + g)
  //   T · cos β = (m_L / N) · (a_L_z + g)
  double Y_m;      // a_L_z + g
  double meas_m;   // T · cos β
  // Channel K (rope-axial spring identification, k_rope):
  //   measurement_k = T          ,  regressor Y_k = δ   (δ ≥ 0 = stretch)
  //   T = k_rope · δ
  double Y_k;      // δ
  double meas_k;   // T
};

/// Observer-only decentralised CL estimator for (m_L/N, k_rope).
///
/// Runs **two independent 1-D CL loops** that share one history
/// buffer (admission decided by a joint excitation criterion):
///
///   (M) payload-vertical-EOM channel — identifies the per-drone mass
///       share. From the payload EOM and symmetric load-sharing,
///         T_i · cos β ≈ (m_L / N) · (a_L_z + g).
///       Measurement m_meas = T · cos β, regressor Y_m = (a_L_z + g).
///
///   (K) rope-axial spring channel — identifies the effective rope
///       stiffness. From the Kelvin–Voigt model neglecting the small
///       damping contribution during cruise,
///         T = k_rope · δ.
///       Measurement k_meas = T, regressor Y_k = δ (stretch ≥ 0).
///
/// Each channel uses the same gradient + concurrent-learning update
///     θ̇ = γ (Y·ε + ρ Σ_j Y_j · ε_j)
/// with a box projection and an independent rank-margin threshold
/// (λ²_buffer for each scalar regressor = scalar variance).
///
/// Input ports:
///   - plant_state:  full MultibodyPlant state (positions ⊕ velocities)
///   - rope_tension: 4-vector [T, fx, fy, fz] for this drone's rope
///
/// Output ports:
///   - theta_hat:    2-vector [m_hat, k_hat]
///   - innovation:   scalar T − Y·θ at the current sample
///   - rank_margin:  λ_min(Σ Y_j^T Y_j) across the current history buffer
///   - history_size: scalar count of stored points (for logging)
class CLParamEstimator final : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CLParamEstimator);

  CLParamEstimator(
      const drake::multibody::MultibodyPlant<double>& plant,
      const drake::multibody::RigidBody<double>& quadcopter_body,
      const drake::multibody::RigidBody<double>& payload_body,
      const Eigen::Vector3d& quadcopter_attachment_point,
      const Eigen::Vector3d& payload_attachment_point,
      const CLParamEstimatorParams& params);

  const drake::systems::InputPort<double>& get_plant_state_input_port() const {
    return get_input_port(plant_state_port_);
  }
  const drake::systems::InputPort<double>& get_tension_input_port() const {
    return get_input_port(tension_port_);
  }
  const drake::systems::OutputPort<double>& get_theta_hat_output_port() const {
    return get_output_port(theta_hat_port_);
  }
  const drake::systems::OutputPort<double>& get_innovation_output_port() const {
    return get_output_port(innovation_port_);
  }
  const drake::systems::OutputPort<double>& get_rank_margin_output_port() const {
    return get_output_port(rank_margin_port_);
  }
  const drake::systems::OutputPort<double>& get_history_size_output_port() const {
    return get_output_port(history_size_port_);
  }

  const CLParamEstimatorParams& params() const { return params_; }

 private:
  drake::systems::EventStatus Update(
      const drake::systems::Context<double>& context,
      drake::systems::State<double>* state) const;

  void CalcThetaHat(const drake::systems::Context<double>& context,
                    drake::systems::BasicVector<double>* output) const;
  void CalcInnovation(const drake::systems::Context<double>& context,
                      drake::systems::BasicVector<double>* output) const;
  void CalcRankMargin(const drake::systems::Context<double>& context,
                      drake::systems::BasicVector<double>* output) const;
  void CalcHistorySize(const drake::systems::Context<double>& context,
                       drake::systems::BasicVector<double>* output) const;

  /// Computes the current rope stretch δ, the payload vertical
  /// acceleration, and the (T, Y) sample from the plant state + rope
  /// tension input. Uses a cached plant context.
  void ComputeSample(const drake::systems::Context<double>& context,
                     Eigen::Vector2d* Y_out,
                     double* T_out,
                     double* delta_out,
                     Eigen::Vector3d* v_L_out) const;

  // Plant + body handles.
  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::multibody::BodyIndex quad_body_index_;
  drake::multibody::BodyIndex payload_body_index_;
  Eigen::Vector3d quad_attach_;
  Eigen::Vector3d payload_attach_;
  CLParamEstimatorParams params_;

  // Port indices.
  int plant_state_port_{-1};
  int tension_port_{-1};
  int theta_hat_port_{-1};
  int innovation_port_{-1};
  int rank_margin_port_{-1};
  int history_size_port_{-1};

  // Discrete-state layout (size 7):
  //   [0, 1] theta_hat         — 2 scalars
  //   [2]    prev_v_L_z        — previous payload vertical velocity
  //   [3]    filtered_a_L_z    — low-pass-filtered payload vertical accel
  //   [4]    last_innovation   — scalar residual from the latest tick
  //   [5]    last_rank_margin  — cached λ_min (diagnostic)
  //   [6]    last_delta        — cached rope stretch (diagnostic)
  drake::systems::DiscreteStateIndex state_index_;

  // Abstract state for the history buffer.
  drake::systems::AbstractStateIndex history_index_;

  // Cached plant context (single instance, reused across ticks).
  mutable std::unique_ptr<drake::systems::Context<double>> plant_context_;
  mutable std::mutex ctx_mtx_;
};

}  // namespace quad_rope_lift
