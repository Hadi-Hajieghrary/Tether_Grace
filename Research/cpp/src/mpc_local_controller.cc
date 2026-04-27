#include "mpc_local_controller.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>

#include <drake/multibody/math/spatial_force.h>
#include <drake/solvers/mathematical_program.h>
#include <drake/solvers/solve.h>

#include "controller_utils.h"

namespace quad_rope_lift {

using drake::multibody::ExternallyAppliedSpatialForce;
using drake::multibody::MultibodyPlant;
using drake::multibody::RigidBody;
using drake::multibody::SpatialForce;
using drake::solvers::MathematicalProgram;
using drake::systems::BasicVector;
using drake::systems::Context;

MpcLocalController::MpcLocalController(
    const MultibodyPlant<double>& plant,
    const RigidBody<double>& quadcopter_body,
    const RigidBody<double>& payload_body,
    const Params& params)
    : plant_(plant),
      quad_body_index_(quadcopter_body.index()),
      payload_body_index_(payload_body.index()),
      mass_(quadcopter_body.default_mass()),
      params_(params) {

  plant_state_port_ = DeclareVectorInputPort(
      "plant_state",
      BasicVector<double>(plant.num_positions() + plant.num_velocities()))
      .get_index();
  tension_port_ = DeclareVectorInputPort(
      "rope_tension", BasicVector<double>(4)).get_index();
  // Optional dynamic formation-offset override; same semantics as the
  // baseline controller's port. Unconnected ⇒ use params_.formation_offset.
  DeclareVectorInputPort("formation_offset_override",
                         BasicVector<double>(3));

  control_port_ = DeclareAbstractOutputPort(
      "control_force",
      &MpcLocalController::CalcControlForce).get_index();
  control_vector_port_ = DeclareVectorOutputPort(
      "control_vector", BasicVector<double>(6),
      &MpcLocalController::CalcControlVector).get_index();
  // Diagnostics port: 13 scalars (same as baseline) so both controllers
  // share the sim harness's VectorLogSink<13> without changes. Slots
  // 7..10 (unused in MPC — it doesn't track tilt-active-set bits) carry
  // MPC-specific telemetry (mpc_slack_max, mpc_horizon_T_max,
  // mpc_relax_flag); 11..12 stay zero.
  diagnostics_port_ = DeclareVectorOutputPort(
      "diagnostics", BasicVector<double>(13),
      &MpcLocalController::CalcDiagnostics).get_index();

  // ── Pre-compute constant MPC matrices ────────────────────────────
  const int Np = params_.horizon_steps;
  const double Ts = params_.dt_mpc;

  // Error-state double integrator with x = [e_p; e_v] where e_p =
  // p_slot − p_drone (positive e_p_z ⇒ drone below slot). The drone's
  // *commanded* acceleration u acts on the DRONE body, so the error
  // evolves as ė_p = e_v, ė_v = slot_acc − u ≈ −u (for the short
  // horizon, slot_acc ≈ 0). Hence B carries a NEGATIVE Ts on the
  // velocity block (and −Ts²/2 on the position block).
  //   A = [[I3, Ts·I3]; [0, I3]]      (6×6)
  //   B = [[−Ts²/2·I3]; [−Ts·I3]]     (6×3)
  A_ = Eigen::MatrixXd::Identity(6, 6);
  A_.topRightCorner(3, 3) = Ts * Eigen::Matrix3d::Identity();
  B_ = Eigen::MatrixXd::Zero(6, 3);
  B_.topRows(3) = -0.5 * Ts * Ts * Eigen::Matrix3d::Identity();
  B_.bottomRows(3) = -Ts * Eigen::Matrix3d::Identity();

  // Phi  =  [A; A^2; ...; A^Np]                       (6·Np × 6)
  // Omega = block-lower-triangular where block(j, l) = A^(j-l) · B
  Phi_.resize(6 * Np, 6);
  Omega_.resize(6 * Np, 3 * Np);
  Omega_.setZero();
  Eigen::MatrixXd Ak = A_;
  for (int j = 0; j < Np; ++j) {
    Phi_.block(6 * j, 0, 6, 6) = Ak;
    // Fill column blocks.
    Eigen::MatrixXd Ajl = Eigen::MatrixXd::Identity(6, 6);
    for (int l = j; l >= 0; --l) {
      Omega_.block(6 * j, 3 * l, 6, 3) = Ajl * B_;
      Ajl = A_ * Ajl;
    }
    Ak = A_ * Ak;
  }

  // ── Terminal-cost derivation via DARE on the per-axis double
  //    integrator. The resulting closed-loop spectral radius is only
  //    reported for diagnostics — the active cost below is a
  //    reference-tracking cost, not a state-regulator cost.
  Eigen::Matrix2d As;
  As << 1.0, Ts, 0.0, 1.0;
  Eigen::Vector2d Bs;
  Bs << 0.5 * Ts * Ts, Ts;
  Eigen::Matrix2d Qs;
  Qs << params_.q_pos, 0.0, 0.0, params_.q_vel;
  bool dare_converged = false;
  double dare_residual = 0.0;
  const Eigen::Matrix2d Ps =
      SolveScalarDARE(As, Bs, Qs, params_.r_ctrl,
                       &dare_converged, &dare_residual);
  const Eigen::RowVector2d Ks =
      DARELqrGain(As, Bs, Ps, params_.r_ctrl);
  dare_spectral_radius_ = DARESpectralRadius(As, Bs, Ks);

  // Reference-tracking cost: the MPC minimises
  //   Σ_k  w_t · ‖U_k − a_target‖²  +  w_e · ‖U_k‖²
  // so the cascade PD still defines the target acceleration every
  // tick and the MPC's horizon is used solely to enforce the
  // linearised tension ceiling. The Hessian is a constant block-
  // diagonal (w_t + w_e)·I_{3Np}; the linear cost f is rebuilt per
  // tick as f_k = −2·w_t·a_target.
  const double w_t = 1.0;
  const double w_e = params_.r_ctrl;
  H_ = (w_t + w_e) * Eigen::MatrixXd::Identity(3 * Np, 3 * Np);

  U_warm_ = Eigen::VectorXd::Zero(3 * Np);

  std::cout << "MpcLocalController built: N_p=" << Np
            << ", DARE ρ(A-BK)=" << dare_spectral_radius_
            << (dare_converged ? "" : " (NON-CONVERGED)")
            << ", L_eff_body=" << params_.L_eff_body
            << " m, T_max_safe=" << params_.T_max_safe << " N\n";
  if (!dare_converged) {
    std::cout << "  WARNING: DARE Picard iteration residual="
              << dare_residual
              << " after 500 steps; terminal cost may be degraded.\n";
  }
  if (dare_spectral_radius_ >= 1.0) {
    std::cout << "  WARNING: DARE closed-loop pole outside unit disc — "
              << "MPC terminal cost will NOT guarantee stability!\n";
  }
}

void MpcLocalController::CalcControlForce(
    const Context<double>& context,
    std::vector<ExternallyAppliedSpatialForce<double>>* output) const {
  const double t = context.get_time();
  const int Np = params_.horizon_steps;

  // ── Sensed state (same extraction as the baseline controller) ───
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
  const Eigen::Matrix3d R_self = self_pose.rotation().matrix();
  const Eigen::Vector3d omega_W = self_vel.rotational();

  const auto& payload_body = plant_.get_body(payload_body_index_);
  const auto& payload_pose =
      plant_.EvalBodyPoseInWorld(*plant_context, payload_body);
  const auto& payload_vel =
      plant_.EvalBodySpatialVelocityInWorld(*plant_context, payload_body);
  const Eigen::Vector3d p_payload = payload_pose.translation();
  const Eigen::Vector3d v_L = payload_vel.translational();

  // Pickup latch (same semantics as baseline).
  if (!pickup_start_time_.has_value() &&
      measured_tension >= params_.pickup_detection_threshold) {
    pickup_start_time_ = t;
  }
  if (params_.initial_pretensioned && !pickup_start_time_.has_value()) {
    pickup_start_time_ = -params_.pickup_ramp_duration;
  }

  // Slot reference plus horizontal anti-swing, honouring the optional
  // formation-offset override port when connected.
  Eigen::Vector3d formation_offset = params_.formation_offset;
  const auto& offset_override_port =
      GetInputPort("formation_offset_override");
  if (offset_override_port.HasValue(context)) {
    const auto& offs = offset_override_port.Eval(context);
    formation_offset = Eigen::Vector3d(offs[0], offs[1], offs[2]);
  }
  Eigen::Vector3d p_slot, v_slot;
  ComputeSlotReferenceAt(t, params_.waypoints, formation_offset,
                         params_.rope_drop, &p_slot, &v_slot);
  Eigen::Vector3d swing_correction = Eigen::Vector3d::Zero();
  swing_correction.head<2>() = -params_.swing_kd * v_L.head<2>();
  const double swing_mag = swing_correction.norm();
  if (swing_mag > params_.swing_offset_max) {
    swing_correction *= params_.swing_offset_max / swing_mag;
  }
  last_swing_offset_mag_ = swing_correction.norm();
  const Eigen::Vector3d p_slot_dyn = p_slot + swing_correction;

  // Error state (6): [e_p; e_v], with e_p = p_slot − p_self (consistent
  // with the baseline, so positive z-error means drone below slot).
  Eigen::VectorXd x(6);
  x.head(3) = p_slot_dyn - p_self;
  x.tail(3) = v_slot - v_self;

  // T_ff (pickup-ramp gated measured tension) and thrust envelope.
  double T_ff;
  const bool in_pickup = pickup_start_time_.has_value()
      && (t - pickup_start_time_.value() <= params_.pickup_ramp_duration);
  if (in_pickup) {
    const double tau = std::clamp(
        (t - pickup_start_time_.value()) / params_.pickup_ramp_duration,
        0.0, 1.0);
    const double alpha = tau * tau * (3.0 - 2.0 * tau);
    const double T_target = alpha * params_.pickup_target_tension_nominal;
    T_ff = std::clamp(measured_tension, 0.0, T_target);
  } else {
    T_ff = measured_tension;
  }
  last_T_ff_ = T_ff;
  const double a_tilt_max = params_.gravity * std::tan(params_.max_tilt);
  const double a_z_min = (params_.min_thrust - T_ff) / mass_ - params_.gravity;
  const double a_z_max = (params_.max_thrust - T_ff) / mass_ - params_.gravity;

  // First-order low-pass filter on dT/dt, used to extrapolate the
  // tension forward along the horizon. The filter is disabled for the
  // first few ticks so the ZOH warm-up transient (initial output 0
  // followed by the true tension) does not produce a spurious spike.
  constexpr int kWarmupTicks = 50;
  if (last_tick_time_ < 0.0) {
    last_T_meas_ = measured_tension;
    last_tick_time_ = t;
    T_dot_filtered_ = 0.0;
    warmup_ticks_ = 0;
  } else {
    const double dt_obs = std::max(1e-6, t - last_tick_time_);
    const double T_dot_raw = (measured_tension - last_T_meas_) / dt_obs;
    if (warmup_ticks_ < kWarmupTicks) {
      // Seed the filter but don't extrapolate yet.
      T_dot_filtered_ = 0.0;
    } else {
      const double alpha = dt_obs / (kTDotTau + dt_obs);
      T_dot_filtered_ = alpha * T_dot_raw + (1.0 - alpha) * T_dot_filtered_;
    }
    last_T_meas_ = measured_tension;
    last_tick_time_ = t;
    ++warmup_ticks_;
  }

  // Assemble per-step tension-ceiling rows. The linearisation is
  //   T(k+j) ≈ T(k) + jΔt · (dT/dt)_filt + c_jᵀ (A^j − I) x(k)
  //                                    + c_jᵀ Ω_{row j} U,
  // with c_j = [−k_eff n̂(k); 0]. Using (A^j − I) rather than A^j
  // folds only the *change* in state into the free response; without
  // this the current non-zero slot-tracking error would tighten d_T
  // into infeasibility at steady state.
  const double k_eff = params_.k_eff;
  const Eigen::Vector3d chord_now = p_self - p_payload;
  const double d_now = chord_now.norm();
  const Eigen::Vector3d n_hat_now =
      (d_now > 1e-6) ? (chord_now / d_now) : Eigen::Vector3d(0, 0, 1);
  Eigen::VectorXd cj(6);
  cj.head(3) = -k_eff * n_hat_now;
  cj.tail(3).setZero();

  Eigen::MatrixXd CT = Eigen::MatrixXd::Zero(Np, 3 * Np);
  Eigen::VectorXd dT = Eigen::VectorXd::Zero(Np);
  Eigen::MatrixXd Aj = Eigen::MatrixXd::Identity(6, 6);
  double horizon_T_max_nom = measured_tension;
  // T_dot feed-forward is additive: extrapolates observed oscillatory
  // tension ringing forward (clipped to T_dot ≥ 0 to avoid unphysical
  // bias during benign cruise).
  const double T_dot_use =
      (warmup_ticks_ > 200) ? std::max(0.0, T_dot_filtered_) : 0.0;
  const Eigen::MatrixXd I6 = Eigen::MatrixXd::Identity(6, 6);
  for (int j = 0; j < Np; ++j) {
    Aj = A_ * Aj;  // Aj = A^(j+1)
    const double forward = std::clamp(
        (j + 1) * params_.dt_mpc * T_dot_use,
        -kMaxForwardT, kMaxForwardT);
    const double T_pred_free = measured_tension + forward
        + cj.dot((Aj - I6) * x);
    horizon_T_max_nom = std::max(horizon_T_max_nom, T_pred_free);
    CT.row(j) = cj.transpose() * Omega_.block(6 * j, 0, 6, 3 * Np);
    dT(j) = params_.T_max_safe - T_pred_free;
  }
  last_mpc_horizon_T_max_ = horizon_T_max_nom;

  // Reference acceleration (same as the baseline's slot-tracking + swing
  // damping). The MPC minimises deviation from this + effort penalty.
  const Eigen::Vector3d e_p = x.head<3>();
  const Eigen::Vector3d e_v = x.tail<3>();
  Eigen::Vector3d a_track;
  a_track.x() = params_.position_kp * e_p.x() + params_.position_kd * e_v.x();
  a_track.y() = params_.position_kp * e_p.y() + params_.position_kd * e_v.y();
  a_track.z() = params_.altitude_kp * e_p.z() + params_.altitude_kd * e_v.z();
  const Eigen::Vector3d a_swing(-params_.swing_kd * v_L.x(),
                                -params_.swing_kd * v_L.y(), 0.0);
  const Eigen::Vector3d a_target_ref = a_track + params_.w_swing * a_swing;

  // ── Build QP  (primal = [U; s] with slack s ≥ 0, length Np) ─────
  // Cost:   Σ_k (w_t·‖U_k − a_target_ref‖² + w_e·‖U_k‖²) + w_s·1ᵀ·s
  //     =   ½ Uᵀ H U + fᵀ U + const  +  w_s·1ᵀ·s
  // with   H = (w_t + w_e)·I_{3·Np}  (constant; pre-computed in ctor),
  //        f_k = −2·w_t·a_target_ref for each horizon step k.
  // s.t.   C_T·U − s ≤ d_T
  //        box on U,  s ≥ 0
  MathematicalProgram prog;
  auto U = prog.NewContinuousVariables(3 * Np, "U");
  auto S = prog.NewContinuousVariables(Np, "S");

  const double w_t = 1.0;
  Eigen::VectorXd f = Eigen::VectorXd::Zero(3 * Np);
  for (int k = 0; k < Np; ++k) {
    f.segment<3>(3 * k) = -2.0 * w_t * a_target_ref;
  }
  prog.AddQuadraticCost(H_, f, U);
  // Linear slack penalty w_s · 1' · s  (no quadratic term; keeps the
  // slack acting as a ≤-inequality relaxation).
  prog.AddLinearCost(params_.tension_slack_weight
                         * Eigen::VectorXd::Ones(Np),
                     S);

  // Box bounds on U — per-step tilt/thrust envelopes.
  for (int j = 0; j < Np; ++j) {
    prog.AddBoundingBoxConstraint(-a_tilt_max, a_tilt_max, U(3 * j));
    prog.AddBoundingBoxConstraint(-a_tilt_max, a_tilt_max, U(3 * j + 1));
    prog.AddBoundingBoxConstraint(a_z_min,     a_z_max,    U(3 * j + 2));
  }
  // Slack bounds s ≥ 0.
  for (int j = 0; j < Np; ++j) {
    prog.AddBoundingBoxConstraint(
        0.0, std::numeric_limits<double>::infinity(), S(j));
  }
  // Linear ineq: C_T · U − s ≤ d_T, i.e. [C_T  −I] · [U; s] ≤ d_T.
  Eigen::MatrixXd Ac(Np, 3 * Np + Np);
  Ac.leftCols(3 * Np) = CT;
  Ac.rightCols(Np) = -Eigen::MatrixXd::Identity(Np, Np);
  Eigen::VectorXd lb =
      -std::numeric_limits<double>::infinity() * Eigen::VectorXd::Ones(Np);
  Eigen::VectorXd ub = dT;
  drake::solvers::VectorXDecisionVariable z(3 * Np + Np);
  z.head(3 * Np) = U;
  z.tail(Np) = S;
  prog.AddLinearConstraint(Ac, lb, ub, z);

  // Warm-start primal from the shifted previous solve (slack always
  // starts at zero; the warm-start is primal-only).
  prog.SetInitialGuess(U, U_warm_);
  prog.SetInitialGuess(S, Eigen::VectorXd::Zero(Np));

  const auto t0 = std::chrono::high_resolution_clock::now();
  const auto result = drake::solvers::Solve(prog);
  const auto t1 = std::chrono::high_resolution_clock::now();
  last_mpc_solve_time_us_ =
      std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0)
          .count() * 1e-3;

  Eigen::Vector3d a_d_opt;
  if (result.is_success()) {
    const Eigen::VectorXd U_opt = result.GetSolution(U);
    const Eigen::VectorXd S_opt = result.GetSolution(S);
    a_d_opt = U_opt.head(3);
    last_mpc_cost_value_ = result.get_optimal_cost();
    last_mpc_slack_max_ = S_opt.maxCoeff();
    last_mpc_relax_flag_ = (last_mpc_slack_max_ > 1.0) ? 1 : 0;
    // Shift-warm for the next call.
    U_warm_.head(3 * (Np - 1)) = U_opt.tail(3 * (Np - 1));
    U_warm_.tail(3) = U_opt.tail(3);
  } else {
    // Fallback: baseline's single-step PD command.
    a_d_opt.x() = std::clamp(params_.position_kp * x(0)
                              + params_.position_kd * x(3),
                             -a_tilt_max, a_tilt_max);
    a_d_opt.y() = std::clamp(params_.position_kp * x(1)
                              + params_.position_kd * x(4),
                             -a_tilt_max, a_tilt_max);
    a_d_opt.z() = std::clamp(params_.altitude_kp * x(2)
                              + params_.altitude_kd * x(5),
                             a_z_min, a_z_max);
    last_mpc_cost_value_ = -1.0;
    last_mpc_slack_max_ = 0.0;
    last_mpc_relax_flag_ = 2;  // solver-failure sentinel
  }

  // ── Thrust / tilt synthesis (identical to baseline) ─────────────
  double thrust = mass_ * (params_.gravity + a_d_opt.z()) + T_ff;
  if (in_pickup) {
    const double tau = std::clamp(
        (t - pickup_start_time_.value()) / params_.pickup_ramp_duration,
        0.0, 1.0);
    const double alpha = tau * tau * (3.0 - 2.0 * tau);
    const double T_target = alpha * params_.pickup_target_tension_nominal;
    thrust += params_.tension_feedback_kp * (T_target - measured_tension);
  }
  thrust = std::clamp(thrust, params_.min_thrust, params_.max_thrust);
  last_thrust_cmd_ = thrust;

  const double pitch_des = std::clamp(a_d_opt.x() / params_.gravity,
                                       -params_.max_tilt, params_.max_tilt);
  const double roll_des  = std::clamp(-a_d_opt.y() / params_.gravity,
                                       -params_.max_tilt, params_.max_tilt);
  last_tilt_mag_ = std::sqrt(pitch_des * pitch_des + roll_des * roll_des);

  // Attitude inner loop (identical to baseline).
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
  const Eigen::Vector3d force_world = R_self.col(2) * thrust;

  output->clear();
  ExternallyAppliedSpatialForce<double> w;
  w.body_index = quad_body_index_;
  w.p_BoBq_B = Eigen::Vector3d::Zero();
  w.F_Bq_W = SpatialForce<double>(torque_world, force_world);
  output->push_back(w);
}

void MpcLocalController::CalcControlVector(
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

void MpcLocalController::CalcDiagnostics(
    const Context<double>& context, BasicVector<double>* output) const {
  // Refresh the last_*_ cache by running the control computation; see
  // the matching note in DecentralizedLocalController::CalcDiagnostics
  // for the port-ordering rationale.
  std::vector<ExternallyAppliedSpatialForce<double>> refresh_forces;
  CalcControlForce(context, &refresh_forces);

  const auto& state_vector = get_input_port(plant_state_port_).Eval(context);
  auto plant_context = plant_.CreateDefaultContext();
  plant_.SetPositionsAndVelocities(plant_context.get(), state_vector);
  const auto& payload_body = plant_.get_body(payload_body_index_);
  const auto& payload_vel =
      plant_.EvalBodySpatialVelocityInWorld(*plant_context, payload_body);
  const Eigen::Vector3d v_L = payload_vel.translational();

  // 13-slot layout (MPC-specific telemetry in slots 7–9 that the
  // baseline uses for tilt active-set bits — MPC doesn't track them):
  //   [0] swing_speed,  [1] swing_offset_mag,
  //   [2] mpc_cost,     [3] mpc_solve_time_us,
  //   [4] T_ff,         [5] thrust_cmd,    [6] tilt_mag,
  //   [7] mpc_slack_max,         [8] mpc_horizon_T_max,
  //   [9] mpc_relax_flag (0 nominal / 1 slack-active / 2 solver-fail),
  //   [10]–[12] reserved (always 0).
  output->SetAtIndex(0,  v_L.head<2>().norm());
  output->SetAtIndex(1,  last_swing_offset_mag_);
  output->SetAtIndex(2,  last_mpc_cost_value_);
  output->SetAtIndex(3,  last_mpc_solve_time_us_);
  output->SetAtIndex(4,  last_T_ff_);
  output->SetAtIndex(5,  last_thrust_cmd_);
  output->SetAtIndex(6,  last_tilt_mag_);
  output->SetAtIndex(7,  last_mpc_slack_max_);
  output->SetAtIndex(8,  last_mpc_horizon_T_max_);
  output->SetAtIndex(9,  static_cast<double>(last_mpc_relax_flag_));
  output->SetAtIndex(10, 0.0);
  output->SetAtIndex(11, 0.0);
  output->SetAtIndex(12, 0.0);
}

}  // namespace quad_rope_lift
