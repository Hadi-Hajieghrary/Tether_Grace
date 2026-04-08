/// @file fault_detecting_gpac_controller.cc
/// @brief Implementation of FaultDetectingGPACController — the full GPAC
///        hierarchy (position+anti-swing, geometric SO(3) attitude, ESO)
///        extended with cable-snap detection and escape manoeuvre.

#include "gpac_fault_tolerance/drake/fault_detecting_gpac_controller.h"

#include <algorithm>
#include <cmath>

#include <drake/common/drake_assert.h>
#include <drake/multibody/math/spatial_force.h>

#include "gpac_math.h"

namespace tether_grace {

namespace gpac = quad_rope_lift::gpac;

using drake::multibody::ExternallyAppliedSpatialForce;
using drake::multibody::SpatialForce;
using drake::systems::BasicVector;
using drake::systems::Context;
using quad_rope_lift::GPACParams;
using quad_rope_lift::GPACWaypoint;

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

FaultDetectingGPACController::FaultDetectingGPACController(
    const drake::multibody::MultibodyPlant<double> &plant,
    const drake::multibody::RigidBody<double> &quadcopter_body,
    const FaultDetectingGPACParams &params)
    : plant_(plant), quad_body_index_(quadcopter_body.index()),
      gpac_params_(params.gpac), fault_params_(params),
      nominal_waypoints_(params.gpac.waypoints),
      internal_eso_(params.gpac.eso_omega, params.gpac.eso_b0) {

  // Pre-compute ascent trajectory
  const double altitude_change =
      gpac_params_.final_altitude - gpac_params_.initial_altitude;
  ascent_duration_ = (gpac_params_.climb_rate > 0.0)
                         ? std::abs(altitude_change) / gpac_params_.climb_rate
                         : 0.0;
  ascent_direction_ = (altitude_change >= 0.0) ? 1.0 : -1.0;

  // === Input ports ===

  plant_state_port_ =
      DeclareVectorInputPort(
          "plant_state",
          BasicVector<double>(plant.num_positions() + plant.num_velocities()))
          .get_index();

  tension_port_ = DeclareVectorInputPort("rope_tension", BasicVector<double>(4))
                      .get_index();

  cable_direction_port_ =
      DeclareVectorInputPort("cable_direction", BasicVector<double>(3))
          .get_index();

  // Load-tracking ports (always declared; only read when enable_load_tracking)
  load_position_port_ =
      DeclareVectorInputPort("load_position", BasicVector<double>(3))
          .get_index();
  load_velocity_port_ =
      DeclareVectorInputPort("load_velocity", BasicVector<double>(3))
          .get_index();
  load_trajectory_port_ =
      DeclareVectorInputPort("load_trajectory", BasicVector<double>(9))
          .get_index();

  // Initialize adaptive mass estimate
  theta_hat_ = params.initial_theta;
  theta_hat_filter_ = params.initial_theta;

  // === Continuous state for PID integral ===
  DeclareContinuousState(3); // [integral_x, integral_y, integral_z]

  // === Output ports ===

  control_port_ =
      DeclareAbstractOutputPort("control_force",
                                &FaultDetectingGPACController::CalcControlForce)
          .get_index();

  control_vector_port_ =
      DeclareVectorOutputPort("control_vector", 6,
                              &FaultDetectingGPACController::CalcControlVector)
          .get_index();
}

// ---------------------------------------------------------------------------
// Continuous state (integral of position error)
// ---------------------------------------------------------------------------

void FaultDetectingGPACController::DoCalcTimeDerivatives(
    const Context<double> &context,
    drake::systems::ContinuousState<double> *derivatives) const {

  // In load-tracking mode, after pickup, and not escaped: integrate LOAD error.
  // Otherwise (during pickup, drone-tracking, or escape mode): integrate DRONE error.
  const bool pickup_done_for_integral = pickup_start_time_.has_value() &&
      (context.get_time() - *pickup_start_time_) > gpac_params_.pickup_ramp_duration;
  if (fault_params_.enable_load_tracking && !escape_start_time_.has_value() &&
      pickup_done_for_integral) {
    const Eigen::Vector3d load_pos =
        get_input_port(load_position_port_).Eval(context);
    const auto &load_traj =
        get_input_port(load_trajectory_port_).Eval(context);
    const Eigen::Vector3d load_pos_des = load_traj.head<3>();

    const Eigen::Vector3d load_error = load_pos_des - load_pos;
    auto &deriv = derivatives->get_mutable_vector();
    deriv.SetAtIndex(0, load_error.x());
    deriv.SetAtIndex(1, load_error.y());
    deriv.SetAtIndex(2, load_error.z());
    return;
  }

  const auto &state_vector = get_input_port(plant_state_port_).Eval(context);
  auto plant_context = plant_.CreateDefaultContext();
  plant_.SetPositionsAndVelocities(plant_context.get(), state_vector);

  const auto &quad_body = plant_.get_body(quad_body_index_);
  const Eigen::Vector3d position =
      plant_.EvalBodyPoseInWorld(*plant_context, quad_body).translation();

  const double t = context.get_time();
  Eigen::Vector3d pos_des, vel_des, acc_des;
  ComputeGPACTrajectory(t, pos_des, vel_des, acc_des);

  const Eigen::Vector3d pos_error = pos_des - position;
  auto &deriv = derivatives->get_mutable_vector();
  deriv.SetAtIndex(0, pos_error.x());
  deriv.SetAtIndex(1, pos_error.y());
  deriv.SetAtIndex(2, pos_error.z());
}

void FaultDetectingGPACController::SetDefaultState(
    const Context<double> & /*context*/,
    drake::systems::State<double> *state) const {
  state->get_mutable_continuous_state().get_mutable_vector().SetZero();
}

// ---------------------------------------------------------------------------
// Trajectory helpers
// ---------------------------------------------------------------------------

void FaultDetectingGPACController::ComputeGPACTrajectory(
    double t, Eigen::Vector3d &pos_des, Eigen::Vector3d &vel_des,
    Eigen::Vector3d &acc_des) const {

  acc_des.setZero();

  // If in escape mode, use escape waypoints (no formation offset)
  if (escape_start_time_.has_value() && !active_escape_waypoints_.empty()) {
    ComputeEscapeTrajectory(t, pos_des, vel_des);
    return;
  }

  // Normal GPAC waypoint tracking
  const auto &waypoints = nominal_waypoints_;
  if (!waypoints.empty()) {
    double segment_start_time = 0.0;
    for (size_t i = 0; i < waypoints.size(); ++i) {
      const auto &wp = waypoints[i];
      const double segment_end_time = wp.arrival_time;
      const double hold_end_time = segment_end_time + wp.hold_time;

      if (t <= segment_end_time) {
        if (i == 0) {
          pos_des = wp.position + gpac_params_.formation_offset;
          vel_des.setZero();
        } else {
          const auto &prev = waypoints[i - 1];
          const double dur = segment_end_time - segment_start_time;
          if (dur > 1e-6) {
            const double alpha = (t - segment_start_time) / dur;
            pos_des = (1.0 - alpha) * prev.position + alpha * wp.position +
                      gpac_params_.formation_offset;
            vel_des = (wp.position - prev.position) / dur;
          } else {
            pos_des = wp.position + gpac_params_.formation_offset;
            vel_des.setZero();
          }
        }
        return;
      }
      if (t <= hold_end_time) {
        pos_des = wp.position + gpac_params_.formation_offset;
        vel_des.setZero();
        return;
      }
      segment_start_time = hold_end_time;
    }
    pos_des = waypoints.back().position + gpac_params_.formation_offset;
    vel_des.setZero();
    return;
  }

  // Legacy altitude ramp (no waypoints)
  double desired_altitude;
  double desired_velocity_z;
  if (t <= gpac_params_.ascent_start_time || gpac_params_.climb_rate <= 0.0) {
    desired_altitude = gpac_params_.initial_altitude;
    desired_velocity_z = 0.0;
  } else if (t >= gpac_params_.ascent_start_time + ascent_duration_) {
    desired_altitude = gpac_params_.final_altitude;
    desired_velocity_z = 0.0;
  } else {
    const double elapsed = t - gpac_params_.ascent_start_time;
    desired_altitude = gpac_params_.initial_altitude +
                       ascent_direction_ * gpac_params_.climb_rate * elapsed;
    desired_velocity_z = ascent_direction_ * gpac_params_.climb_rate;
  }
  pos_des =
      Eigen::Vector3d(gpac_params_.formation_offset.x(),
                      gpac_params_.formation_offset.y(), desired_altitude);
  vel_des = Eigen::Vector3d(0.0, 0.0, desired_velocity_z);
}

void FaultDetectingGPACController::ComputeEscapeTrajectory(
    double t, Eigen::Vector3d &pos_des, Eigen::Vector3d &vel_des) const {

  const auto &wps = active_escape_waypoints_;
  DRAKE_DEMAND(!wps.empty());
  double segment_start_time = 0.0;
  for (size_t i = 0; i < wps.size(); ++i) {
    const double end_t = wps[i].arrival_time;
    const double hold_t = end_t + wps[i].hold_time;
    if (t <= end_t) {
      if (i == 0) {
        pos_des = wps[i].position;
        vel_des.setZero();
      } else {
        const double dur = end_t - segment_start_time;
        if (dur > 1e-6) {
          const double alpha = (t - segment_start_time) / dur;
          pos_des =
              (1.0 - alpha) * wps[i - 1].position + alpha * wps[i].position;
          vel_des = (wps[i].position - wps[i - 1].position) / dur;
        } else {
          pos_des = wps[i].position;
          vel_des.setZero();
        }
      }
      return;
    }
    if (t <= hold_t) {
      pos_des = wps[i].position;
      vel_des.setZero();
      return;
    }
    segment_start_time = hold_t;
  }
  pos_des = wps.back().position;
  vel_des.setZero();
}

// ---------------------------------------------------------------------------
// Escape waypoint construction (identical geometry to prior code)
// ---------------------------------------------------------------------------

std::vector<GPACWaypoint> FaultDetectingGPACController::BuildEscapeWaypoints(
    const Eigen::Vector3d &start_position,
    const Eigen::Vector3d &start_velocity,
    const Eigen::Vector3d &nominal_center_position, double start_time,
    double nominal_altitude) const {

  Eigen::Vector3d radial = start_position - nominal_center_position;
  radial.z() = 0.0;
  if (radial.head<2>().norm() < 1e-6) {
    radial = gpac_params_.formation_offset;
    radial.z() = 0.0;
  }
  if (radial.head<2>().norm() < 1e-6) {
    radial = start_position;
    radial.z() = 0.0;
  }
  if (radial.head<2>().norm() < 1e-6) {
    radial = Eigen::Vector3d::UnitX();
  } else {
    radial.normalize();
  }

  Eigen::Vector3d tangent(-radial.y(), radial.x(), 0.0);
  const Eigen::Vector3d h_vel(start_velocity.x(), start_velocity.y(), 0.0);
  if (h_vel.head<2>().norm() > 0.15 && tangent.dot(h_vel) < 0.0) {
    tangent *= -1.0;
  } else if (h_vel.head<2>().norm() <= 0.15 && radial.y() < 0.0) {
    tangent *= -1.0;
  }

  Eigen::Vector3d corridor =
      radial + fault_params_.escape_tangent_bias * tangent;
  if (h_vel.head<2>().norm() > 0.15) {
    corridor += fault_params_.escape_velocity_bias * h_vel.normalized();
  }
  corridor.z() = 0.0;
  if (corridor.head<2>().norm() < 1e-6) {
    corridor = radial;
  } else {
    corridor.normalize();
  }

  const double t1 = start_time + fault_params_.first_escape_delay;
  const double t2 = start_time + fault_params_.second_escape_delay;
  const double target_alt = std::max(start_position.z(), nominal_altitude);

  Eigen::Vector3d first_dir = radial;
  if (h_vel.head<2>().norm() > 0.35 &&
      first_dir.dot(h_vel.normalized()) < -0.25) {
    first_dir = (first_dir + 0.25 * h_vel.normalized()).normalized();
  }

  const Eigen::Vector3d p1 =
      start_position + fault_params_.first_escape_distance * first_dir +
      Eigen::Vector3d(0, 0, fault_params_.first_escape_climb);
  const Eigen::Vector3d p2 =
      start_position + fault_params_.second_escape_distance * corridor +
      Eigen::Vector3d(0, 0, fault_params_.second_escape_climb);

  return {
      {start_position, start_time, 0.05},
      {Eigen::Vector3d(p1.x(), p1.y(), std::max(p1.z(), target_alt)), t1, 0.10},
      {Eigen::Vector3d(p2.x(), p2.y(), std::max(p2.z(), target_alt + 0.5)), t2,
       1000.0},
  };
}

// ---------------------------------------------------------------------------
// Fault detection
// ---------------------------------------------------------------------------

void FaultDetectingGPACController::MaybeActivateEscape(
    double time, double measured_tension, const Eigen::Vector3d &translation,
    const Eigen::Vector3d &translational_velocity,
    const Eigen::Vector3d &nominal_position, double nominal_altitude) const {

  if (!fault_params_.enable_fault_detection || escape_start_time_.has_value())
    return;
  if (!pickup_start_time_.has_value())
    return;
  if (time - *pickup_start_time_ < fault_params_.fault_detect_arming_time)
    return;

  if (measured_tension <= fault_params_.fault_detect_tension_threshold) {
    if (!low_tension_start_time_.has_value()) {
      low_tension_start_time_ = time;
      return;
    }
    if (time - *low_tension_start_time_ >=
        fault_params_.fault_detect_hold_time) {
      escape_start_time_ = time;
      active_escape_waypoints_ =
          BuildEscapeWaypoints(translation, translational_velocity,
                               nominal_position - gpac_params_.formation_offset,
                               time, nominal_altitude);
    }
  } else {
    low_tension_start_time_.reset();
  }
}

// ---------------------------------------------------------------------------
// Layer 1: Position + Anti-swing + PID + ESO feedforward
// ---------------------------------------------------------------------------

Eigen::Vector3d FaultDetectingGPACController::ComputeLayer1Control(
    const Context<double> &context, const Eigen::Vector3d &position,
    const Eigen::Vector3d &velocity, double measured_tension) const {

  const double t = context.get_time();

  // Integral state (with anti-windup)
  const auto &state = context.get_continuous_state_vector();
  Eigen::Vector3d pos_integral(std::clamp(state[0], -gpac_params_.max_integral,
                                          gpac_params_.max_integral),
                               std::clamp(state[1], -gpac_params_.max_integral,
                                          gpac_params_.max_integral),
                               std::clamp(state[2], -gpac_params_.max_integral,
                                          gpac_params_.max_integral));

  // Trajectory
  Eigen::Vector3d pos_des, vel_des, acc_des;
  ComputeGPACTrajectory(t, pos_des, vel_des, acc_des);

  // Pickup phase
  bool in_pickup_phase = false;
  double target_tension = gpac_params_.pickup_target_tension;

  if (!pickup_start_time_.has_value() &&
      measured_tension >= gpac_params_.pickup_detection_threshold) {
    pickup_start_time_ = t;
  }

  if (pickup_start_time_.has_value()) {
    const double dt_pickup = t - *pickup_start_time_;
    if (dt_pickup >= 0.0 && dt_pickup <= gpac_params_.pickup_ramp_duration) {
      in_pickup_phase = true;
      double ramp =
          std::clamp(dt_pickup / gpac_params_.pickup_ramp_duration, 0.0, 1.0);
      target_tension = ramp * gpac_params_.pickup_target_tension;

      double alt_adj = gpac_params_.tension_altitude_gain *
                       (target_tension - measured_tension);
      alt_adj = std::clamp(alt_adj, -gpac_params_.tension_altitude_max,
                           gpac_params_.tension_altitude_max);
      pos_des.z() += alt_adj;
    }
  }

  // Fault detection (before computing errors)
  MaybeActivateEscape(t, measured_tension, position, velocity, pos_des,
                      pos_des.z());
  // Re-evaluate trajectory after possible escape activation
  ComputeGPACTrajectory(t, pos_des, vel_des, acc_des);

  // ===== LOAD-TRACKING MODE =====
  // When enabled and not in escape: close the loop on actual payload position.
  // Force = theta_hat * (a_des + g - Kp*e_L - Kd*de_L - Ki*int_e_L)
  //       + anti_swing + disturbance_ff
  // Load-tracking activates only after pickup is complete (cable taut + ramp done)
  const bool pickup_complete = pickup_start_time_.has_value() &&
      (t - *pickup_start_time_) > gpac_params_.pickup_ramp_duration;

  if (fault_params_.enable_load_tracking && !escape_start_time_.has_value() &&
      pickup_complete) {

    const Eigen::Vector3d load_pos =
        get_input_port(load_position_port_).Eval(context);
    const Eigen::Vector3d load_vel =
        get_input_port(load_velocity_port_).Eval(context);
    const auto &load_traj =
        get_input_port(load_trajectory_port_).Eval(context);

    const Eigen::Vector3d load_pos_des = load_traj.head<3>();
    const Eigen::Vector3d load_vel_des = load_traj.segment<3>(3);
    const Eigen::Vector3d load_acc_des = load_traj.tail<3>();

    // Load tracking errors
    const Eigen::Vector3d load_error = load_pos_des - load_pos;
    const Eigen::Vector3d load_vel_error = load_vel_des - load_vel;

    const Eigen::Vector3d e3(0.0, 0.0, 1.0);

    // Simple adaptive mass estimation: use cable tension to infer load share.
    // In steady state: T_i ≈ theta_hat * g (vertical equilibrium).
    // Update theta_hat with slow filter: theta += alpha * (T_z/g - theta)
    const double T_z = measured_tension; // tension magnitude ≈ vertical component
    const double g = gpac_params_.gravity;
    const double theta_from_tension = T_z / g;
    const double alpha = 0.005; // slow adaptation rate
    theta_hat_ += alpha * (theta_from_tension - theta_hat_);
    theta_hat_ = std::clamp(theta_hat_, 0.3, 15.0); // safety bounds
    // Low-pass filter for smoother control
    theta_hat_filter_ += 0.02 * (theta_hat_ - theta_hat_filter_);

    // Anti-swing on S² (cable direction control)
    Eigen::Vector3d anti_swing = Eigen::Vector3d::Zero();
    if (gpac_params_.enable_antiswing) {
      const auto &cable_port = get_input_port(cable_direction_port_);
      if (cable_port.HasValue(context)) {
        const Eigen::Vector3d q = cable_port.Eval(context).normalized();
        const Eigen::Vector3d q_d(0.0, 0.0, -1.0); // vertical desired
        anti_swing = gpac_params_.cable_kq * gpac::CableDirectionError(q, q_d);
      }
    }

    // Disturbance feedforward via internal ESO
    Eigen::Vector3d disturbance_ff = Eigen::Vector3d::Zero();
    if (gpac_params_.enable_eso_feedforward) {
      const double dt =
          (last_update_time_ > 0.0) ? (t - last_update_time_) : 0.001;
      if (dt > 0.0001 && last_update_time_ != t) {
        internal_eso_.Update(position, last_control_accel_, dt);
        last_update_time_ = t;
      }
      disturbance_ff = internal_eso_.disturbance();
    }

    // Hybrid approach: use the ORIGINAL drone-level PID control (gravity ff +
    // PID on drone position error + cable tension ff) but REPLACE the integral
    // term with integration on LOAD error instead of drone error. This gives:
    // - Drone PD for fast transient response and stability
    // - Load integral for eliminating the persistent Z offset after cable snap
    //
    // The pos_integral state now integrates load error (done in DoCalcTimeDerivatives).

    // Drone-level PD (same as original, but integral uses load error)
    Eigen::Vector3d pos_des_drone, vel_des_drone, acc_des_drone;
    ComputeGPACTrajectory(t, pos_des_drone, vel_des_drone, acc_des_drone);
    Eigen::Vector3d drone_pos_error = pos_des_drone - position;
    Eigen::Vector3d drone_vel_error = vel_des_drone - velocity;

    // Gravity feedforward
    Eigen::Vector3d F_ff =
        gpac_params_.mass * (g * e3 + acc_des_drone);

    // PD on drone position + integral on LOAD position
    Eigen::Vector3d F_fb = gpac_params_.mass *
        (gpac_params_.position_kp * drone_pos_error +
         gpac_params_.position_kd * drone_vel_error +
         fault_params_.load_ki * pos_integral);  // integral is on LOAD error

    // Cable tension feedforward OR oracle load-share feedforward
    Eigen::Vector3d F_tension;
    if (fault_params_.oracle_load_share > 0.0) {
      // Oracle: replace cable-tension FF with perfect load-share knowledge
      F_tension = fault_params_.oracle_load_share * g * e3;
    } else {
      double tension_ff_limit = gpac_params_.pickup_target_tension;
      F_tension = std::clamp(measured_tension, 0.0, tension_ff_limit) * e3;
    }

    // ESO compensation (drone-level): subtract model-based feedforward
    Eigen::Vector3d F_eso = Eigen::Vector3d::Zero();
    if (gpac_params_.enable_eso_feedforward) {
      const Eigen::Vector3d model_accel =
          g * e3 + F_tension / gpac_params_.mass;
      const Eigen::Vector3d residual = disturbance_ff - model_accel;
      F_eso = gpac_params_.mass * residual;
    }

    Eigen::Vector3d F_des = F_ff + F_fb + F_tension + F_eso + anti_swing;
    last_control_accel_ = F_des / gpac_params_.mass;
    return F_des;
  }

  // ===== DRONE-TRACKING MODE (original / escape) =====
  // Position / velocity errors (drone-level)
  const Eigen::Vector3d pos_error = pos_des - position;
  const Eigen::Vector3d vel_error = vel_des - velocity;

  // Anti-swing
  Eigen::Vector3d anti_swing = Eigen::Vector3d::Zero();
  if (gpac_params_.enable_antiswing && !escape_start_time_.has_value()) {
    const auto &cable_port = get_input_port(cable_direction_port_);
    if (cable_port.HasValue(context)) {
      const Eigen::Vector3d q = cable_port.Eval(context).normalized();
      const Eigen::Vector3d q_d(0.0, 0.0, -1.0);
      anti_swing = gpac_params_.cable_kq * gpac::CableDirectionError(q, q_d);
    }
  }

  // Disturbance feedforward via internal ESO
  Eigen::Vector3d disturbance_ff = Eigen::Vector3d::Zero();
  if (gpac_params_.enable_eso_feedforward) {
    const double dt =
        (last_update_time_ > 0.0) ? (t - last_update_time_) : 0.001;
    if (dt > 0.0001 && last_update_time_ != t) {
      internal_eso_.Update(position, last_control_accel_, dt);
      last_update_time_ = t;
    }
    disturbance_ff = internal_eso_.disturbance();
  }

  // Build total force
  const Eigen::Vector3d e3(0.0, 0.0, 1.0);

  // Feedforward: gravity + trajectory acceleration
  Eigen::Vector3d F_ff =
      gpac_params_.mass * (gpac_params_.gravity * e3 + acc_des);

  // Oracle load-share feedforward (centralized upper bound)
  if (fault_params_.oracle_load_share > 0.0 && escape_start_time_.has_value() == false) {
    F_ff += fault_params_.oracle_load_share * gpac_params_.gravity * e3;
  }

  // Feedback: PID
  Eigen::Vector3d F_fb =
      gpac_params_.mass * (gpac_params_.position_kp * pos_error +
                           gpac_params_.position_kd * vel_error +
                           gpac_params_.position_ki * pos_integral);

  // Cable tension feedforward
  double tension_ff_limit =
      in_pickup_phase ? target_tension : gpac_params_.pickup_target_tension;
  Eigen::Vector3d F_tension =
      std::clamp(measured_tension, 0.0, tension_ff_limit) * e3;
  if (in_pickup_phase) {
    F_tension +=
        gpac_params_.tension_kp * (target_tension - measured_tension) * e3;
  }

  // ESO compensation: subtract model-based feedforward to avoid double-counting.
  // The ESO estimates the TOTAL disturbance (including gravity and cable tension),
  // so we subtract what the model already compensates to get residual disturbance.
  Eigen::Vector3d F_eso = Eigen::Vector3d::Zero();
  if (gpac_params_.enable_eso_feedforward) {
    const Eigen::Vector3d model_accel =
        gpac_params_.gravity * e3 +
        F_tension / gpac_params_.mass;  // model-based acceleration
    const Eigen::Vector3d residual = disturbance_ff - model_accel;
    F_eso = gpac_params_.mass * residual;
  }

  Eigen::Vector3d F_des = F_ff + F_fb + F_tension + F_eso + anti_swing;

  // Reactive FTC baseline: boost thrust when tension exceeds threshold
  if (fault_params_.enable_reactive_ftc && !escape_start_time_.has_value() &&
      pickup_start_time_.has_value()) {
    const double tension_thr =
        fault_params_.reactive_tension_threshold *
        gpac_params_.pickup_target_tension;
    if (measured_tension > tension_thr) {
      if (!reactive_high_tension_start_.has_value()) {
        reactive_high_tension_start_ = t;
      } else if (t - *reactive_high_tension_start_ >=
                 fault_params_.reactive_debounce_time) {
        reactive_boost_active_ = true;
      }
    } else {
      reactive_high_tension_start_.reset();
      // Keep boost active once triggered (no reset)
    }
    if (reactive_boost_active_) {
      F_des *= (1.0 + fault_params_.reactive_boost_fraction);
    }
  }

  // Gain-scheduled PID baseline: increase PID gains when tension spike detected
  if (fault_params_.enable_gain_scheduling && !escape_start_time_.has_value() &&
      pickup_start_time_.has_value()) {
    const double tension_thr =
        fault_params_.gs_tension_threshold *
        gpac_params_.pickup_target_tension;
    if (measured_tension > tension_thr) {
      if (!gs_high_tension_start_.has_value()) {
        gs_high_tension_start_ = t;
      } else if (t - *gs_high_tension_start_ >=
                 fault_params_.gs_debounce_time) {
        gs_gains_boosted_ = true;
      }
    } else {
      gs_high_tension_start_.reset();
    }
    if (gs_gains_boosted_) {
      // Recompute PID feedback with scaled gains
      const double kp_eff =
          gpac_params_.position_kp * fault_params_.gs_kp_scale;
      const double kd_eff =
          gpac_params_.position_kd * fault_params_.gs_kd_scale;
      Eigen::Vector3d F_fb_boosted =
          gpac_params_.mass *
          (kp_eff * pos_error + kd_eff * vel_error +
           gpac_params_.position_ki * pos_integral);
      // Replace the feedback term: F_des = F_ff + F_tension + F_eso +
      // anti_swing + F_fb_boosted
      F_des = F_ff + F_fb_boosted + F_tension + F_eso + anti_swing;
    }
  }

  last_control_accel_ = F_des / gpac_params_.mass;
  return F_des;
}

// ---------------------------------------------------------------------------
// Layer 2: Geometric SO(3) attitude control
// ---------------------------------------------------------------------------

Eigen::Vector3d FaultDetectingGPACController::ComputeLayer2Control(
    const Eigen::Matrix3d &R, const Eigen::Matrix3d &R_d,
    const Eigen::Vector3d &Omega, const Eigen::Vector3d &Omega_d) const {

  const Eigen::Vector3d e_R = gpac::AttitudeError(R, R_d);
  const Eigen::Vector3d e_Omega =
      gpac::AngularVelocityError(Omega, R, R_d, Omega_d);

  Eigen::Vector3d tau =
      -gpac_params_.attitude_kR * e_R - gpac_params_.attitude_kOmega * e_Omega;
  for (int i = 0; i < 3; ++i) {
    tau[i] =
        std::clamp(tau[i], -gpac_params_.max_torque, gpac_params_.max_torque);
  }
  return tau;
}

// ---------------------------------------------------------------------------
// CalcControlForce — main output
// ---------------------------------------------------------------------------

void FaultDetectingGPACController::CalcControlForce(
    const Context<double> &context,
    std::vector<ExternallyAppliedSpatialForce<double>> *output) const {

  const auto &state_vector = get_input_port(plant_state_port_).Eval(context);
  const auto &tension_data = get_input_port(tension_port_).Eval(context);
  const double measured_tension = tension_data[0];

  auto plant_context = plant_.CreateDefaultContext();
  plant_.SetPositionsAndVelocities(plant_context.get(), state_vector);

  const auto &quad_body = plant_.get_body(quad_body_index_);
  const auto &pose_world =
      plant_.EvalBodyPoseInWorld(*plant_context, quad_body);
  const auto &velocity_world =
      plant_.EvalBodySpatialVelocityInWorld(*plant_context, quad_body);

  const Eigen::Vector3d position = pose_world.translation();
  const Eigen::Vector3d velocity = velocity_world.translational();
  const Eigen::Matrix3d R = pose_world.rotation().matrix();
  const Eigen::Vector3d omega_W = velocity_world.rotational();
  const Eigen::Vector3d Omega = R.transpose() * omega_W;

  // === Layer 1 ===
  const Eigen::Vector3d F_des =
      ComputeLayer1Control(context, position, velocity, measured_tension);

  double thrust = F_des.norm();
  thrust = std::clamp(thrust, gpac_params_.min_thrust, gpac_params_.max_thrust);

  // === Layer 2: Geometric attitude control ===
  const double psi_d = 0.0;
  const Eigen::Matrix3d R_d = gpac::DesiredRotation(F_des, psi_d);
  const Eigen::Vector3d Omega_d = Eigen::Vector3d::Zero();
  const Eigen::Vector3d tau_body = ComputeLayer2Control(R, R_d, Omega, Omega_d);

  const Eigen::Vector3d torque_world = R * tau_body;
  const Eigen::Vector3d force_world = R.col(2) * thrust;

  output->clear();
  output->reserve(1);
  ExternallyAppliedSpatialForce<double> wrench;
  wrench.body_index = quad_body_index_;
  wrench.p_BoBq_B = Eigen::Vector3d::Zero();
  wrench.F_Bq_W = SpatialForce<double>(torque_world, force_world);
  output->push_back(wrench);
}

void FaultDetectingGPACController::CalcControlVector(
    const Context<double> &context, BasicVector<double> *output) const {

  std::vector<ExternallyAppliedSpatialForce<double>> forces;
  CalcControlForce(context, &forces);
  if (!forces.empty()) {
    const auto &F = forces[0].F_Bq_W;
    output->SetAtIndex(0, F.rotational()(0));
    output->SetAtIndex(1, F.rotational()(1));
    output->SetAtIndex(2, F.rotational()(2));
    output->SetAtIndex(3, F.translational()(0));
    output->SetAtIndex(4, F.translational()(1));
    output->SetAtIndex(5, F.translational()(2));
  } else {
    for (int i = 0; i < 6; ++i)
      output->SetAtIndex(i, 0.0);
  }
}

} // namespace tether_grace
