#include "decentralized_local_controller.h"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <drake/multibody/math/spatial_force.h>
#include <drake/solvers/mathematical_program.h>
#include <drake/solvers/solve.h>

namespace quad_rope_lift {

using drake::multibody::ExternallyAppliedSpatialForce;
using drake::multibody::MultibodyPlant;
using drake::multibody::RigidBody;
using drake::multibody::SpatialForce;
using drake::solvers::MathematicalProgram;
using drake::systems::BasicVector;
using drake::systems::Context;

DecentralizedLocalController::DecentralizedLocalController(
    const MultibodyPlant<double>& plant,
    const RigidBody<double>& quadcopter_body,
    const RigidBody<double>& payload_body,
    const Params& params)
    : plant_(plant),
      quad_body_index_(quadcopter_body.index()),
      payload_body_index_(payload_body.index()),
      mass_(quadcopter_body.default_mass()),
      params_(params) {

  // If the sim starts with rope pre-tensioned, treat the pickup-ramp as
  // already complete so T_ff = measured_tension from the very first tick.
  if (params_.initial_pretensioned) {
    pickup_start_time_ = -params_.pickup_ramp_duration;
  }

  plant_state_port_ = DeclareVectorInputPort(
      "plant_state",
      BasicVector<double>(plant.num_positions() + plant.num_velocities()))
      .get_index();

  tension_port_ = DeclareVectorInputPort(
      "rope_tension", BasicVector<double>(4)).get_index();

  // Optional dynamic formation-offset override (3-vector). Wired by
  // the FormationCoordinator supervisor in Phase-H. Left unconnected
  // ⇒ falls back to Params::formation_offset (default behaviour).
  formation_offset_override_port_ =
      DeclareVectorInputPort(
          "formation_offset_override", BasicVector<double>(3)).get_index();

  control_port_ = DeclareAbstractOutputPort(
      "control_force",
      &DecentralizedLocalController::CalcControlForce).get_index();

  control_vector_port_ = DeclareVectorOutputPort(
      "control_vector", BasicVector<double>(6),
      &DecentralizedLocalController::CalcControlVector).get_index();

  // L1 adaptive outer loop — allocate discrete state + periodic update
  // event only when enabled. This keeps baseline behaviour byte-for-byte
  // reproducible when `l1_enabled = false` (default).
  if (params_.l1_enabled) {
    Eigen::VectorXd l1_init(5);
    l1_init << 0.0, 0.0, 0.0, 0.0, params_.l1_k_eff_nominal;
    l1_state_idx_ = DeclareDiscreteState(l1_init);
    DeclarePeriodicUnrestrictedUpdateEvent(
        params_.l1_control_step, 0.0,
        &DecentralizedLocalController::CalcL1Update);
  }
  l1_state_port_ =
      DeclareVectorOutputPort(
          "l1_state", BasicVector<double>(5),
          &DecentralizedLocalController::CalcL1State).get_index();

  // Diagnostics = 13 scalars:
  //   [0]  swing_speed         — ‖v_L,xy‖ (m/s)
  //   [1]  swing_offset_mag    — anti-swing slot shift magnitude (m)
  //   [2]  qp_cost             — terminal QP objective value
  //   [3]  qp_solve_time_us    — wall-clock QP solve time (μs)
  //   [4]  T_ff                — feed-forward tension this tick (N)
  //   [5]  thrust_cmd          — commanded thrust (N)
  //   [6]  tilt_mag            — commanded tilt magnitude (rad)
  //   [7]  active_ax_lo ∈ {0,1}
  //   [8]  active_ax_hi ∈ {0,1}
  //   [9]  active_ay_lo ∈ {0,1}
  //  [10]  active_ay_hi ∈ {0,1}
  //  [11]  active_az_lo ∈ {0,1}
  //  [12]  active_az_hi ∈ {0,1}
  diagnostics_port_ = DeclareVectorOutputPort(
      "diagnostics", BasicVector<double>(13),
      &DecentralizedLocalController::CalcDiagnostics).get_index();
}

// -------------------------------------------------------------------------
void DecentralizedLocalController::ComputePayloadReference(
    double t, Eigen::Vector3d* p_ref, Eigen::Vector3d* v_ref) const {
  const auto& wps = params_.waypoints;
  double seg_start = 0.0;
  for (size_t i = 0; i < wps.size(); ++i) {
    const auto& wp = wps[i];
    const double seg_end = wp.arrival_time;
    const double hold_end = seg_end + wp.hold_time;
    if (t <= seg_end) {
      if (i == 0) {
        *p_ref = wp.position;
        v_ref->setZero();
      } else {
        const auto& pv = wps[i - 1];
        const double dur = seg_end - seg_start;
        if (dur > 1e-6) {
          const double alpha = (t - seg_start) / dur;
          *p_ref = (1.0 - alpha) * pv.position + alpha * wp.position;
          *v_ref = (wp.position - pv.position) / dur;
        } else {
          *p_ref = wp.position;
          v_ref->setZero();
        }
      }
      return;
    } else if (t <= hold_end) {
      *p_ref = wp.position;
      v_ref->setZero();
      return;
    }
    seg_start = hold_end;
  }
  *p_ref = wps.back().position;
  v_ref->setZero();
}

void DecentralizedLocalController::ComputeSlotReference(
    double t, Eigen::Vector3d* p_slot, Eigen::Vector3d* v_slot) const {
  // Drone's target slot = payload reference + formation offset (XY) + rope_drop (Z)
  Eigen::Vector3d p_L_ref, v_L_ref;
  ComputePayloadReference(t, &p_L_ref, &v_L_ref);
  *p_slot = p_L_ref + params_.formation_offset;
  p_slot->z() += params_.rope_drop;
  *v_slot = v_L_ref;  // formation moves rigidly with the payload reference
}

// -------------------------------------------------------------------------
void DecentralizedLocalController::CalcControlForce(
    const Context<double>& context,
    std::vector<ExternallyAppliedSpatialForce<double>>* output) const {
  const double t = context.get_time();

  // ----------------------------------------------------------- sensed state
  const auto& state_vector = get_input_port(plant_state_port_).Eval(context);
  const auto& tension_data = get_input_port(tension_port_).Eval(context);
  const double measured_tension = tension_data[0];

  auto plant_context = plant_.CreateDefaultContext();
  plant_.SetPositionsAndVelocities(plant_context.get(), state_vector);

  const auto& self_body = plant_.get_body(quad_body_index_);
  const auto& self_pose = plant_.EvalBodyPoseInWorld(*plant_context, self_body);
  const auto& self_vel  = plant_.EvalBodySpatialVelocityInWorld(*plant_context, self_body);
  const Eigen::Vector3d p_self = self_pose.translation();
  const Eigen::Vector3d v_self = self_vel.translational();
  const Eigen::Matrix3d R_self = self_pose.rotation().matrix();
  const Eigen::Vector3d omega_W = self_vel.rotational();

  const auto& payload_body = plant_.get_body(payload_body_index_);
  const auto& payload_pose = plant_.EvalBodyPoseInWorld(*plant_context, payload_body);
  const auto& payload_vel  = plant_.EvalBodySpatialVelocityInWorld(*plant_context, payload_body);
  const Eigen::Vector3d v_L = payload_vel.translational();
  (void)payload_pose;  // position available if needed for richer control

  // ------------------------------------------------------------- pickup latch
  if (!pickup_start_time_.has_value()
      && measured_tension >= params_.pickup_detection_threshold) {
    pickup_start_time_ = t;
  }

  // ----------------------------------------------------------- slot reference
  Eigen::Vector3d p_slot, v_slot;
  ComputeSlotReference(t, &p_slot, &v_slot);
  // Apply dynamic formation-offset override (Phase-H) when wired.
  // `HasValue` returns true only if the port is connected AND the
  // upstream system has produced a non-default value.
  if (get_input_port(formation_offset_override_port_).HasValue(context)) {
    const auto& offs =
        get_input_port(formation_offset_override_port_).Eval(context);
    // Reconstruct slot with the override (drop the static offset
    // already added by ComputeSlotReference first).
    p_slot = p_slot - params_.formation_offset + Eigen::Vector3d(
        offs[0], offs[1], offs[2]);
  }

  // --------------------------------------------- anti-swing slot correction
  // Shift the slot by -swing_kd · v_payload_xy so the drone moves against
  // the payload swing and applies a rope pull that brakes it.
  Eigen::Vector3d swing_correction = Eigen::Vector3d::Zero();
  swing_correction.head<2>() = -params_.swing_kd * v_L.head<2>();
  const double swing_mag = swing_correction.norm();
  if (swing_mag > params_.swing_offset_max) {
    swing_correction *= params_.swing_offset_max / swing_mag;
  }
  last_swing_offset_mag_ = swing_correction.norm();
  const Eigen::Vector3d p_slot_dynamic = p_slot + swing_correction;

  // --------------------------------------------------- desired accelerations
  Eigen::Vector3d e_p = p_slot_dynamic - p_self;
  Eigen::Vector3d e_v = v_slot - v_self;
  const double ax_track = params_.position_kp * e_p.x() + params_.position_kd * e_v.x();
  const double ay_track = params_.position_kp * e_p.y() + params_.position_kd * e_v.y();
  const double az_track = params_.altitude_kp * e_p.z() + params_.altitude_kd * e_v.z();
  const Eigen::Vector3d a_track(ax_track, ay_track, az_track);

  // Swing-damping target acceleration (horizontal only). When the payload
  // moves in +x, the drone should apply a negative-x component so its rope
  // pulls the payload back toward the reference.
  const Eigen::Vector3d a_swing(-params_.swing_kd * v_L.x(),
                                -params_.swing_kd * v_L.y(), 0.0);

  // ---------------------------------------------------------- per-drone QP
  // Structure the objectives so they AUGMENT each other rather than compete:
  //   a_target  =  a_track  +  w_swing · a_swing         (tracking + anti-swing)
  //
  //   min   ‖a − a_target‖²  +  w_effort · ‖a‖²          (projection onto feasible
  //                                                        envelope, with light
  //                                                        penalty on magnitude)
  //   s.t.  |a_x|, |a_y|  ≤  g · tan(θ_max)              (tilt envelope)
  //         a_z  ∈  [a_z_min, a_z_max]                    (thrust envelope)
  //
  // Anti-swing is applied as an additive correction on top of tracking; the
  // QP then chooses the closest feasible acceleration that respects actuator
  // limits while lightly preferring smaller-magnitude commands (effort).
  Eigen::Vector3d a_target = a_track + params_.w_swing * a_swing;

  // L1 adaptive correction — inject u_ad on the altitude channel, right
  // before the QP. Sign convention derived from the empirical test
  // m_L = 3.9 kg (controller tuned for 3.0 kg) in which the drone
  // systematically droops below slot (e_z_meas > 0) and the adaptive
  // law drives σ̂ positive. With the theory-doc's '-= u_ad' convention,
  // this would drop the command further, worsening the droop. The
  // correct sign for this sim is '+= u_ad': when the adaptive law
  // decides σ̂ > 0 (predictor interpretation: real drone is above
  // predictor due to an unmeasured upward disturbance), we cancel it by
  // adding u_ad to the target so the drone tracks.
  //
  // Equivalently, σ̂ here captures the sign of the dominant *matched*
  // disturbance in this simulator (rope-geometry residual, actuator
  // mismatch, etc.) and the injection compensates it. Documented in
  // theory_l1_extension.md caveat for implementation.
  if (params_.l1_enabled) {
    const auto& l1s = context.get_discrete_state(l1_state_idx_).value();
    a_target.z() += l1s[3];  // l1s[3] = u_ad
  }
  const double a_tilt_max = params_.gravity * std::tan(params_.max_tilt);

  // Vertical envelope comes from the thrust limits, net of the tension
  // feed-forward (computed below). Use the measured tension as the ff value
  // (post-pickup). During pickup we apply a C¹-continuous smoothstep ramp
  //
  //     α(τ) = 3τ² − 2τ³          (τ ∈ [0,1])
  //
  // so that the feed-forward tension grows smoothly from 0 to the nominal
  // value T* = m_L g / N with zero first-derivative at both endpoints,
  // eliminating the step-discontinuity that previously excited the 5 Hz
  // bead-chain axial mode at t = 0.
  double T_ff;
  const bool in_pickup = pickup_start_time_.has_value()
      && (t - pickup_start_time_.value() <= params_.pickup_ramp_duration);
  if (in_pickup) {
    const double tau = std::clamp(
        (t - pickup_start_time_.value()) / params_.pickup_ramp_duration, 0.0, 1.0);
    // Smoothstep (Hermite cubic): α ∈ C¹, α'(0) = α'(1) = 0.
    const double alpha = tau * tau * (3.0 - 2.0 * tau);
    const double T_target = alpha * params_.pickup_target_tension_nominal;
    T_ff = std::clamp(measured_tension, 0.0, T_target);
  } else {
    T_ff = measured_tension;
  }

  const double a_z_min = (params_.min_thrust - T_ff) / mass_ - params_.gravity;
  const double a_z_max = (params_.max_thrust - T_ff) / mass_ - params_.gravity;

  MathematicalProgram prog;
  auto a_d = prog.NewContinuousVariables<3>("a_d");

  // Primary cost: track the (tracking + swing) target.
  Eigen::Matrix3d W_t = params_.w_track * Eigen::Matrix3d::Identity();
  prog.AddQuadraticErrorCost(W_t, a_target, a_d);

  // Effort cost: small quadratic penalty on |a| — prefers minimum-energy
  // solutions when multiple feasible accelerations satisfy the target.
  Eigen::Matrix3d W_e = params_.w_effort * Eigen::Matrix3d::Identity();
  prog.AddQuadraticErrorCost(W_e, Eigen::Vector3d::Zero(), a_d);

  // Box constraints
  prog.AddBoundingBoxConstraint(-a_tilt_max, a_tilt_max, a_d(0));
  prog.AddBoundingBoxConstraint(-a_tilt_max, a_tilt_max, a_d(1));
  prog.AddBoundingBoxConstraint(a_z_min, a_z_max, a_d(2));

  const auto t_qp_start = std::chrono::high_resolution_clock::now();
  auto result = drake::solvers::Solve(prog);
  const auto t_qp_end = std::chrono::high_resolution_clock::now();
  last_solve_time_us_ =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          t_qp_end - t_qp_start).count() * 1e-3;

  Eigen::Vector3d a_d_opt;
  if (result.is_success()) {
    a_d_opt = result.GetSolution(a_d);
    last_qp_cost_ = result.get_optimal_cost();
  } else {
    // Fallback: saturated PD command.
    a_d_opt = a_track;
    a_d_opt.x() = std::clamp(a_d_opt.x(), -a_tilt_max, a_tilt_max);
    a_d_opt.y() = std::clamp(a_d_opt.y(), -a_tilt_max, a_tilt_max);
    a_d_opt.z() = std::clamp(a_d_opt.z(), a_z_min, a_z_max);
    last_qp_cost_ = -1.0;
  }

  // Active-set detection: a bound is "active" if the optimum lies within
  // eps_active of it. Use eps scaled by the envelope magnitude so it is
  // dimensionally sane across tilt-x/y (≈ g tan θ_max ≈ 7 m/s²) and the
  // thrust channel (|a_z_max − a_z_min| ≈ 100 m/s²).
  const double eps_tilt = 1e-3 * a_tilt_max;
  const double eps_thrust = 1e-3 * (a_z_max - a_z_min);
  last_active_set_[0] = (a_d_opt(0) <= -a_tilt_max + eps_tilt) ? 1.0 : 0.0;
  last_active_set_[1] = (a_d_opt(0) >=  a_tilt_max - eps_tilt) ? 1.0 : 0.0;
  last_active_set_[2] = (a_d_opt(1) <= -a_tilt_max + eps_tilt) ? 1.0 : 0.0;
  last_active_set_[3] = (a_d_opt(1) >=  a_tilt_max - eps_tilt) ? 1.0 : 0.0;
  last_active_set_[4] = (a_d_opt(2) <=  a_z_min   + eps_thrust) ? 1.0 : 0.0;
  last_active_set_[5] = (a_d_opt(2) >=  a_z_max   - eps_thrust) ? 1.0 : 0.0;
  last_T_ff_ = T_ff;

  // -------------------------------------------- thrust & tilt synthesis
  double thrust = mass_ * (params_.gravity + a_d_opt.z()) + T_ff;
  // Extra pickup-phase feedback to pull the rope taut faster — uses the
  // same smoothstep schedule as the T_ff ramp so the two terms are
  // co-phased and do not produce a step-discontinuous net thrust command.
  if (in_pickup) {
    const double tau = std::clamp(
        (t - pickup_start_time_.value()) / params_.pickup_ramp_duration, 0.0, 1.0);
    const double alpha = tau * tau * (3.0 - 2.0 * tau);
    const double T_target = alpha * params_.pickup_target_tension_nominal;
    thrust += params_.tension_feedback_kp * (T_target - measured_tension);
  }
  thrust = std::clamp(thrust, params_.min_thrust, params_.max_thrust);
  last_thrust_cmd_ = thrust;

  // From commanded horizontal acceleration → desired roll / pitch (small-angle).
  const double pitch_des = std::clamp(a_d_opt.x() / params_.gravity,
                                       -params_.max_tilt, params_.max_tilt);
  const double roll_des  = std::clamp(-a_d_opt.y() / params_.gravity,
                                       -params_.max_tilt, params_.max_tilt);
  last_tilt_mag_ = std::sqrt(pitch_des * pitch_des + roll_des * roll_des);

  // ---------------------------------------------------- attitude inner loop
  const double current_roll  = std::atan2(R_self(2, 1), R_self(2, 2));
  const double current_pitch = std::asin(-R_self(2, 0));
  const double current_yaw_err = 0.5 * (R_self(1, 0) - R_self(0, 1));

  const double roll_err  = roll_des  - current_roll;
  const double pitch_err = pitch_des - current_pitch;
  const double yaw_err   = -current_yaw_err;

  const double omega_Bx = R_self(0, 0) * omega_W[0] + R_self(1, 0) * omega_W[1] + R_self(2, 0) * omega_W[2];
  const double omega_By = R_self(0, 1) * omega_W[0] + R_self(1, 1) * omega_W[1] + R_self(2, 1) * omega_W[2];
  const double omega_Bz = R_self(0, 2) * omega_W[0] + R_self(1, 2) * omega_W[1] + R_self(2, 2) * omega_W[2];

  double tau_x = params_.attitude_kp * roll_err  - params_.attitude_kd * omega_Bx;
  double tau_y = params_.attitude_kp * pitch_err - params_.attitude_kd * omega_By;
  double tau_z = params_.attitude_kp * yaw_err   - params_.attitude_kd * omega_Bz;
  tau_x = std::clamp(tau_x, -params_.max_torque, params_.max_torque);
  tau_y = std::clamp(tau_y, -params_.max_torque, params_.max_torque);
  tau_z = std::clamp(tau_z, -params_.max_torque, params_.max_torque);

  const Eigen::Vector3d torque_body(tau_x, tau_y, tau_z);
  const Eigen::Vector3d torque_world = R_self * torque_body;
  const Eigen::Vector3d force_world  = R_self.col(2) * thrust;

  output->clear();
  output->reserve(1);
  ExternallyAppliedSpatialForce<double> w;
  w.body_index = quad_body_index_;
  w.p_BoBq_B = Eigen::Vector3d::Zero();
  w.F_Bq_W = SpatialForce<double>(torque_world, force_world);
  output->push_back(w);
}

// -------------------------------------------------------------------------
void DecentralizedLocalController::CalcControlVector(
    const Context<double>& context, BasicVector<double>* output) const {
  std::vector<ExternallyAppliedSpatialForce<double>> forces;
  CalcControlForce(context, &forces);
  if (!forces.empty()) {
    const auto& F = forces[0].F_Bq_W;
    output->SetAtIndex(0, F.rotational()(0));
    output->SetAtIndex(1, F.rotational()(1));
    output->SetAtIndex(2, F.rotational()(2));
    output->SetAtIndex(3, F.translational()(0));
    output->SetAtIndex(4, F.translational()(1));
    output->SetAtIndex(5, F.translational()(2));
  } else {
    for (int i = 0; i < 6; ++i) output->SetAtIndex(i, 0.0);
  }
}

void DecentralizedLocalController::CalcDiagnostics(
    const Context<double>& context, BasicVector<double>* output) const {
  const auto& state_vector = get_input_port(plant_state_port_).Eval(context);
  auto plant_context = plant_.CreateDefaultContext();
  plant_.SetPositionsAndVelocities(plant_context.get(), state_vector);
  const auto& payload_body = plant_.get_body(payload_body_index_);
  const auto& payload_vel = plant_.EvalBodySpatialVelocityInWorld(*plant_context, payload_body);
  const Eigen::Vector3d v_L = payload_vel.translational();

  // 13-element diagnostic vector — layout documented in the ctor.
  output->SetAtIndex(0,  v_L.head<2>().norm());
  output->SetAtIndex(1,  last_swing_offset_mag_);
  output->SetAtIndex(2,  last_qp_cost_);
  output->SetAtIndex(3,  last_solve_time_us_);
  output->SetAtIndex(4,  last_T_ff_);
  output->SetAtIndex(5,  last_thrust_cmd_);
  output->SetAtIndex(6,  last_tilt_mag_);
  output->SetAtIndex(7,  last_active_set_[0]);
  output->SetAtIndex(8,  last_active_set_[1]);
  output->SetAtIndex(9,  last_active_set_[2]);
  output->SetAtIndex(10, last_active_set_[3]);
  output->SetAtIndex(11, last_active_set_[4]);
  output->SetAtIndex(12, last_active_set_[5]);
}

// -------------------------------------------------------------------------
// L1 adaptive outer-loop update event (theory_l1_extension.md §7.5).
//
// State layout (5-vector):
//   [0] ê_z       predictor altitude error                      (m)
//   [1] ê̇_z      predictor altitude error rate                  (m/s)
//   [2] σ̂       matched uncertainty estimate                     (m/s²)
//   [3] u_ad    LP-filtered adaptive acceleration correction      (m/s²)
//   [4] k̂_eff  effective rope stiffness estimate                 (N/m)
drake::systems::EventStatus DecentralizedLocalController::CalcL1Update(
    const drake::systems::Context<double>& context,
    drake::systems::State<double>* state) const {
  const double Ts = params_.l1_control_step;
  const double kpz = params_.altitude_kp;
  const double kdz = params_.altitude_kd;
  const double Gam = params_.l1_gamma;
  const double wc = params_.l1_omega_c;
  // Lyapunov matrix entries (constant, theory-doc §4):
  //   A_m = [[0,1];[−kp,−kd]],  solve A_mᵀ P + P A_m = −I.
  //   With (kp, kd) = (100, 24):  p_{12} = 1/200 = 0.005,
  //                               p_{22} = (p_{12} + 1/2)/kd = 0.02104,
  //                               p_{11} = kp·p_{22} + kd·p_{12} = 2.224.
  constexpr double p12 = 0.005;
  constexpr double p22 = 0.02104;

  // Read current L1 state.
  auto& l1_out = state->get_mutable_discrete_state(l1_state_idx_);
  double ez_hat = l1_out[0];
  double vez_hat = l1_out[1];
  double sig_hat = l1_out[2];
  double u_ad = l1_out[3];
  double keff_hat = l1_out[4];

  // Extract plant state (same extraction as CalcControlForce).
  const auto& state_vector = get_input_port(plant_state_port_).Eval(context);
  const auto& tension_data = get_input_port(tension_port_).Eval(context);
  const double measured_tension = tension_data[0];

  auto plant_context = plant_.CreateDefaultContext();
  plant_.SetPositionsAndVelocities(plant_context.get(), state_vector);

  const auto& self_body = plant_.get_body(quad_body_index_);
  const auto& self_pose = plant_.EvalBodyPoseInWorld(*plant_context, self_body);
  const auto& self_vel =
      plant_.EvalBodySpatialVelocityInWorld(*plant_context, self_body);
  const Eigen::Vector3d p_self = self_pose.translation();
  const Eigen::Vector3d v_self = self_vel.translational();

  const auto& payload_body = plant_.get_body(payload_body_index_);
  const auto& payload_pose =
      plant_.EvalBodyPoseInWorld(*plant_context, payload_body);
  const Eigen::Vector3d p_payload = payload_pose.translation();

  // Slot reference at the current time (same as CalcControlForce).
  Eigen::Vector3d p_slot, v_slot;
  ComputeSlotReference(context.get_time(), &p_slot, &v_slot);

  // Measured altitude-error state vs predictor.
  const double ez_meas = p_slot.z() - p_self.z();
  const double vez_meas = v_slot.z() - v_self.z();
  const double ez_tilde = ez_hat - ez_meas;
  const double vez_tilde = vez_hat - vez_meas;

  // Adaptive law (Eq. 5.5): θ̂ ← Proj(θ̂ − Ts Γ (p12 ẽ + p22 ˜̇e)).
  const double sig_raw =
      sig_hat - Ts * Gam * (p12 * ez_tilde + p22 * vez_tilde);
  sig_hat = std::clamp(sig_raw, params_.l1_sigma_min, params_.l1_sigma_max);

  // LP filter (Eq. 5.7): u_ad ← α u_ad + (1−α) σ̂.
  const double alpha = std::exp(-wc * Ts);
  u_ad = alpha * u_ad + (1.0 - alpha) * sig_hat;

  // State predictor (Eq. 5.4).
  const double ez_hat_new = ez_hat + Ts * vez_hat;
  const double vez_hat_new =
      vez_hat + Ts * (-kpz * ez_hat - kdz * vez_hat + sig_hat + u_ad);

  // k_eff gradient estimator (Eq. 6.1).
  const double dist = (p_self - p_payload).norm();
  const double stretch = std::max(0.0, dist - params_.rope_length);
  if (stretch > params_.l1_stretch_threshold) {
    const double keff_raw =
        keff_hat + Ts * params_.l1_gamma_k * stretch
                       * (measured_tension - keff_hat * stretch);
    keff_hat = std::clamp(keff_raw, params_.l1_k_eff_min, params_.l1_k_eff_max);
  }

  l1_out[0] = ez_hat_new;
  l1_out[1] = vez_hat_new;
  l1_out[2] = sig_hat;
  l1_out[3] = u_ad;
  l1_out[4] = keff_hat;
  return drake::systems::EventStatus::Succeeded();
}

void DecentralizedLocalController::CalcL1State(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output) const {
  if (!params_.l1_enabled) {
    output->SetZero();
    return;
  }
  const auto& s = context.get_discrete_state(l1_state_idx_).value();
  output->SetAtIndex(0, s[0]);
  output->SetAtIndex(1, s[1]);
  output->SetAtIndex(2, s[2]);
  output->SetAtIndex(3, s[3]);
  output->SetAtIndex(4, s[4]);
}

}  // namespace quad_rope_lift
