#include "decentralized_fault_aware_controller.h"

#include <algorithm>
#include <cmath>

#include <drake/multibody/math/spatial_force.h>

namespace quad_rope_lift {

using drake::multibody::ExternallyAppliedSpatialForce;
using drake::multibody::MultibodyPlant;
using drake::multibody::RigidBody;
using drake::multibody::SpatialForce;
using drake::systems::BasicVector;
using drake::systems::Context;

DecentralizedFaultAwareController::DecentralizedFaultAwareController(
    const MultibodyPlant<double>& plant,
    const RigidBody<double>& quadcopter_body,
    const ControllerParams& params,
    double payload_mass,
    int num_drones)
    : plant_(plant),
      quad_body_index_(quadcopter_body.index()),
      mass_(quadcopter_body.default_mass()),
      payload_mass_(payload_mass),
      num_drones_(num_drones),
      formation_offset_(params.formation_offset),
      waypoints_(params.waypoints),
      use_waypoints_(!params.waypoints.empty()),
      initial_altitude_(params.initial_altitude),
      final_altitude_(params.final_altitude),
      ascent_start_time_(params.ascent_start_time),
      climb_rate_(params.climb_rate),
      position_kp_(params.position_kp),
      position_kd_(params.position_kd),
      max_tilt_angle_(params.max_tilt_angle),
      altitude_kp_(params.altitude_kp),
      altitude_kd_(params.altitude_kd),
      attitude_kp_(params.attitude_kp),
      attitude_kd_(params.attitude_kd),
      tension_kp_(params.tension_feedback_kp),
      tension_altitude_gain_(params.tension_altitude_gain),
      tension_altitude_max_(params.tension_altitude_max),
      pickup_duration_(params.pickup_ramp_duration),
      pickup_target_tension_nominal_(params.pickup_target_tension),
      pickup_threshold_(params.pickup_detection_threshold),
      min_thrust_(params.min_thrust),
      max_thrust_(params.max_thrust),
      max_torque_(params.max_torque),
      gravity_(params.gravity),
      // A peer is considered "failed" if its reported tension falls below
      // 25% of the nominal per-drone share. This avoids false failure flags
      // during the pickup ramp (where tensions transiently dip).
      peer_failure_threshold_(params.pickup_target_tension * 0.25) {

  ascent_duration_ = (climb_rate_ > 0.0)
      ? std::abs(final_altitude_ - initial_altitude_) / climb_rate_
      : 0.0;
  ascent_direction_ = (final_altitude_ - initial_altitude_ >= 0.0) ? 1.0 : -1.0;

  plant_state_port_ = DeclareVectorInputPort(
      "plant_state",
      BasicVector<double>(plant.num_positions() + plant.num_velocities()))
      .get_index();

  tension_port_ = DeclareVectorInputPort(
      "rope_tension", BasicVector<double>(4)).get_index();

  peer_tensions_port_ = DeclareVectorInputPort(
      "peer_tensions", BasicVector<double>(num_drones)).get_index();

  control_port_ = DeclareAbstractOutputPort(
      "control_force",
      &DecentralizedFaultAwareController::CalcControlForce).get_index();

  control_vector_port_ = DeclareVectorOutputPort(
      "control_vector", BasicVector<double>(6),
      &DecentralizedFaultAwareController::CalcControlVector).get_index();

  diagnostics_port_ = DeclareVectorOutputPort(
      "diagnostics", BasicVector<double>(3),
      &DecentralizedFaultAwareController::CalcDiagnostics).get_index();
}

void DecentralizedFaultAwareController::CalcControlVector(
    const Context<double>& context,
    BasicVector<double>* output) const {
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

void DecentralizedFaultAwareController::ComputeTrajectory(
    double t, Eigen::Vector3d& pos_des, Eigen::Vector3d& vel_des) const {
  if (use_waypoints_ && !waypoints_.empty()) {
    double segment_start_time = 0.0;
    for (size_t i = 0; i < waypoints_.size(); ++i) {
      const auto& wp = waypoints_[i];
      const double segment_end_time = wp.arrival_time;
      const double hold_end_time = segment_end_time + wp.hold_time;

      if (t <= segment_end_time) {
        if (i == 0) {
          pos_des = wp.position + formation_offset_;
          vel_des.setZero();
        } else {
          const auto& prev_wp = waypoints_[i - 1];
          const double segment_duration = segment_end_time - segment_start_time;
          if (segment_duration > 1e-6) {
            const double alpha = (t - segment_start_time) / segment_duration;
            pos_des = (1.0 - alpha) * prev_wp.position + alpha * wp.position + formation_offset_;
            vel_des = (wp.position - prev_wp.position) / segment_duration;
          } else {
            pos_des = wp.position + formation_offset_;
            vel_des.setZero();
          }
        }
        return;
      } else if (t <= hold_end_time) {
        pos_des = wp.position + formation_offset_;
        vel_des.setZero();
        return;
      }
      segment_start_time = hold_end_time;
    }
    pos_des = waypoints_.back().position + formation_offset_;
    vel_des.setZero();
  } else {
    double desired_altitude;
    double desired_velocity_z;
    if (t <= ascent_start_time_ || climb_rate_ <= 0.0) {
      desired_altitude = initial_altitude_;
      desired_velocity_z = 0.0;
    } else if (t >= ascent_start_time_ + ascent_duration_) {
      desired_altitude = final_altitude_;
      desired_velocity_z = 0.0;
    } else {
      const double elapsed = t - ascent_start_time_;
      desired_altitude = initial_altitude_ + ascent_direction_ * climb_rate_ * elapsed;
      desired_velocity_z = ascent_direction_ * climb_rate_;
    }
    pos_des = Eigen::Vector3d(formation_offset_.x(), formation_offset_.y(), desired_altitude);
    vel_des = Eigen::Vector3d(0.0, 0.0, desired_velocity_z);
  }
}

void DecentralizedFaultAwareController::CalcControlForce(
    const Context<double>& context,
    std::vector<ExternallyAppliedSpatialForce<double>>* output) const {

  const auto& state_vector = get_input_port(plant_state_port_).Eval(context);
  const auto& tension_data = get_input_port(tension_port_).Eval(context);
  const auto& peer_tensions = get_input_port(peer_tensions_port_).Eval(context);
  const double measured_tension = tension_data[0];

  auto plant_context = plant_.CreateDefaultContext();
  plant_.SetPositionsAndVelocities(plant_context.get(), state_vector);

  const double t = context.get_time();

  if (!pickup_start_time_.has_value() && measured_tension >= pickup_threshold_) {
    pickup_start_time_ = t;
  }

  const auto& quad_body = plant_.get_body(quad_body_index_);
  const auto& pose_world = plant_.EvalBodyPoseInWorld(*plant_context, quad_body);
  const auto& velocity_world = plant_.EvalBodySpatialVelocityInWorld(*plant_context, quad_body);

  const Eigen::Vector3d translation = pose_world.translation();
  const Eigen::Vector3d translational_vel = velocity_world.translational();

  Eigen::Vector3d pos_des, vel_des;
  ComputeTrajectory(t, pos_des, vel_des);

  // ================= PEER-AWARE LOAD REBALANCING =================
  // Each peer contributes smoothly to N_alive as min(1, T/T_nominal). Healthy
  // peer → 1, severed peer (T→0) → 0. This smoothing avoids discontinuous
  // jumps in the feedforward target that would destabilize the discrete solver.
  //
  // Rebalancing is only active once the pickup ramp has fully completed,
  // because during pickup all tensions are transiently below nominal and
  // would otherwise spoof N_alive downward.
  const double T_nominal = pickup_target_tension_nominal_;
  double n_alive = static_cast<double>(num_drones_);
  const bool rebalancing_active =
      pickup_start_time_.has_value() &&
      (t - pickup_start_time_.value() > 1.5 * pickup_duration_);

  if (rebalancing_active) {
    n_alive = 0.0;
    for (int i = 0; i < num_drones_; ++i) {
      const double T_i = (i == my_index_) ? measured_tension : peer_tensions[i];
      n_alive += std::clamp(T_i / T_nominal, 0.0, 1.0);
    }
    if (n_alive < 0.5) n_alive = 0.5;  // floor to prevent divergence
  }

  const double target_tension_scaled = (payload_mass_ * gravity_) / n_alive;

  // ================= TENSION-AWARE PICKUP =================
  bool in_pickup_phase = false;
  double target_tension = target_tension_scaled;

  if (pickup_start_time_.has_value()) {
    const double time_since_pickup = t - pickup_start_time_.value();
    if (time_since_pickup >= 0.0 && time_since_pickup <= pickup_duration_) {
      in_pickup_phase = true;
      double ramp_fraction = time_since_pickup / pickup_duration_;
      ramp_fraction = std::clamp(ramp_fraction, 0.0, 1.0);
      target_tension = ramp_fraction * target_tension_scaled;
      const double tension_error = target_tension - measured_tension;
      double altitude_adjustment = tension_altitude_gain_ * tension_error;
      altitude_adjustment = std::clamp(altitude_adjustment,
                                       -tension_altitude_max_,
                                       tension_altitude_max_);
      pos_des.z() += altitude_adjustment;
    }
  }

  // ================= POSITION CONTROL (cascade) =================
  const Eigen::Vector3d pos_error = pos_des - translation;
  const Eigen::Vector3d vel_error = vel_des - translational_vel;

  const double ax_des = position_kp_ * pos_error.x() + position_kd_ * vel_error.x();
  const double ay_des = position_kp_ * pos_error.y() + position_kd_ * vel_error.y();

  double pitch_des = std::clamp(ax_des / gravity_, -max_tilt_angle_, max_tilt_angle_);
  double roll_des = std::clamp(-ay_des / gravity_, -max_tilt_angle_, max_tilt_angle_);

  const double commanded_accel_z = altitude_kp_ * pos_error.z() + altitude_kd_ * vel_error.z();
  double thrust = mass_ * (gravity_ + commanded_accel_z);

  // Tension feed-forward: during pickup we clamp to the ramped target to
  // avoid the drone jumping upward when the rope first goes taut. After
  // pickup completes, feed forward the full measured tension — this gives
  // exact gravity compensation regardless of the true payload mass or peer
  // state, which is the key to the decentralized fault-aware rebalancing.
  const double tension_ff = in_pickup_phase
      ? std::clamp(measured_tension, 0.0, target_tension)
      : measured_tension;
  thrust += tension_ff;

  if (in_pickup_phase) {
    thrust += tension_kp_ * (target_tension - measured_tension);
  }

  thrust = std::clamp(thrust, min_thrust_, max_thrust_);

  // ================= ATTITUDE CONTROL =================
  const Eigen::Matrix3d R = pose_world.rotation().matrix();
  const double current_roll = std::atan2(R(2, 1), R(2, 2));
  const double current_pitch = std::asin(-R(2, 0));
  const double current_yaw_error = 0.5 * (R(1, 0) - R(0, 1));

  const double roll_error = roll_des - current_roll;
  const double pitch_error = pitch_des - current_pitch;
  const double yaw_error = -current_yaw_error;

  const Eigen::Vector3d omega_W = velocity_world.rotational();
  const double omega_Bx = R(0, 0) * omega_W[0] + R(1, 0) * omega_W[1] + R(2, 0) * omega_W[2];
  const double omega_By = R(0, 1) * omega_W[0] + R(1, 1) * omega_W[1] + R(2, 1) * omega_W[2];
  const double omega_Bz = R(0, 2) * omega_W[0] + R(1, 2) * omega_W[1] + R(2, 2) * omega_W[2];

  double tau_x = attitude_kp_ * roll_error - attitude_kd_ * omega_Bx;
  double tau_y = attitude_kp_ * pitch_error - attitude_kd_ * omega_By;
  double tau_z = attitude_kp_ * yaw_error - attitude_kd_ * omega_Bz;

  tau_x = std::clamp(tau_x, -max_torque_, max_torque_);
  tau_y = std::clamp(tau_y, -max_torque_, max_torque_);
  tau_z = std::clamp(tau_z, -max_torque_, max_torque_);

  const Eigen::Vector3d torque_body(tau_x, tau_y, tau_z);
  const Eigen::Vector3d torque_world = R * torque_body;
  const Eigen::Vector3d force_world = R.col(2) * thrust;

  output->clear();
  output->reserve(1);

  ExternallyAppliedSpatialForce<double> control_wrench;
  control_wrench.body_index = quad_body_index_;
  control_wrench.p_BoBq_B = Eigen::Vector3d::Zero();
  control_wrench.F_Bq_W = SpatialForce<double>(torque_world, force_world);
  output->push_back(control_wrench);
}

void DecentralizedFaultAwareController::CalcDiagnostics(
    const Context<double>& context,
    BasicVector<double>* output) const {
  const auto& tension_data = get_input_port(tension_port_).Eval(context);
  const auto& peer_tensions = get_input_port(peer_tensions_port_).Eval(context);
  const double measured_tension = tension_data[0];

  double peer_sum = 0.0;
  const double T_nominal = pickup_target_tension_nominal_;
  const double t = context.get_time();
  double n_alive = static_cast<double>(num_drones_);
  const bool rebalancing_active =
      pickup_start_time_.has_value() &&
      (t - pickup_start_time_.value() > 1.5 * pickup_duration_);
  if (rebalancing_active) {
    n_alive = 0.0;
    for (int i = 0; i < num_drones_; ++i) {
      const double T_i = (i == my_index_) ? measured_tension : peer_tensions[i];
      n_alive += std::clamp(T_i / T_nominal, 0.0, 1.0);
      peer_sum += T_i;
    }
    if (n_alive < 0.5) n_alive = 0.5;
  } else {
    for (int i = 0; i < num_drones_; ++i) {
      const double T_i = (i == my_index_) ? measured_tension : peer_tensions[i];
      peer_sum += T_i;
    }
  }
  const double my_target = (payload_mass_ * 9.81) / n_alive;

  output->SetAtIndex(0, n_alive);
  output->SetAtIndex(1, my_target);
  output->SetAtIndex(2, peer_sum);
}

}  // namespace quad_rope_lift
