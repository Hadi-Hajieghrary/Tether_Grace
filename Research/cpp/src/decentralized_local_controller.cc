#include "decentralized_local_controller.h"

#include <algorithm>
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

  plant_state_port_ = DeclareVectorInputPort(
      "plant_state",
      BasicVector<double>(plant.num_positions() + plant.num_velocities()))
      .get_index();

  tension_port_ = DeclareVectorInputPort(
      "rope_tension", BasicVector<double>(4)).get_index();

  control_port_ = DeclareAbstractOutputPort(
      "control_force",
      &DecentralizedLocalController::CalcControlForce).get_index();

  control_vector_port_ = DeclareVectorOutputPort(
      "control_vector", BasicVector<double>(6),
      &DecentralizedLocalController::CalcControlVector).get_index();

  diagnostics_port_ = DeclareVectorOutputPort(
      "diagnostics", BasicVector<double>(3),
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
  const Eigen::Vector3d a_target = a_track + params_.w_swing * a_swing;
  const double a_tilt_max = params_.gravity * std::tan(params_.max_tilt);

  // Vertical envelope comes from the thrust limits, net of the tension
  // feed-forward (computed below). Use the measured tension as the ff value
  // (post-pickup). During pickup we instead clamp to the ramp-limit (so the
  // thrust stays bounded while the rope is being tensioned).
  double T_ff;
  const bool in_pickup = pickup_start_time_.has_value()
      && (t - pickup_start_time_.value() <= params_.pickup_ramp_duration);
  if (in_pickup) {
    const double tau = std::clamp(
        (t - pickup_start_time_.value()) / params_.pickup_ramp_duration, 0.0, 1.0);
    const double T_target = tau * params_.pickup_target_tension_nominal;
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

  auto result = drake::solvers::Solve(prog);
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

  // -------------------------------------------- thrust & tilt synthesis
  double thrust = mass_ * (params_.gravity + a_d_opt.z()) + T_ff;
  // Extra pickup-phase feedback to pull the rope taut faster.
  if (in_pickup) {
    const double tau = std::clamp(
        (t - pickup_start_time_.value()) / params_.pickup_ramp_duration, 0.0, 1.0);
    const double T_target = tau * params_.pickup_target_tension_nominal;
    thrust += params_.tension_feedback_kp * (T_target - measured_tension);
  }
  thrust = std::clamp(thrust, params_.min_thrust, params_.max_thrust);

  // From commanded horizontal acceleration → desired roll / pitch (small-angle).
  const double pitch_des = std::clamp(a_d_opt.x() / params_.gravity,
                                       -params_.max_tilt, params_.max_tilt);
  const double roll_des  = std::clamp(-a_d_opt.y() / params_.gravity,
                                       -params_.max_tilt, params_.max_tilt);

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

  // [swing_speed, swing_offset_mag, qp_total_cost]
  output->SetAtIndex(0, v_L.head<2>().norm());
  output->SetAtIndex(1, last_swing_offset_mag_);
  output->SetAtIndex(2, last_qp_cost_);
}

}  // namespace quad_rope_lift
