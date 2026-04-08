#include "gpac_fault_tolerance/drake/fault_detecting_quadcopter_controller.h"

#include <algorithm>
#include <cmath>

#include <drake/common/drake_assert.h>
#include <drake/multibody/math/spatial_force.h>

namespace tether_grace {

using drake::multibody::ExternallyAppliedSpatialForce;
using drake::multibody::MultibodyPlant;
using drake::multibody::RigidBody;
using drake::multibody::SpatialForce;
using drake::systems::BasicVector;
using drake::systems::Context;
using quad_rope_lift::ControllerParams;
using quad_rope_lift::TrajectoryWaypoint;

FaultDetectingQuadcopterController::FaultDetectingQuadcopterController(
    const MultibodyPlant<double> &plant,
    const RigidBody<double> &quadcopter_body,
    const FaultDetectingControllerParams &params)
    : plant_(plant), quad_body_index_(quadcopter_body.index()),
      mass_(quadcopter_body.default_mass()),
      formation_offset_(params.base.formation_offset),
      nominal_waypoints_(params.base.waypoints),
      use_waypoints_(!params.base.waypoints.empty()),
      initial_altitude_(params.base.initial_altitude),
      final_altitude_(params.base.final_altitude),
      ascent_start_time_(params.base.ascent_start_time),
      climb_rate_(params.base.climb_rate),
      position_kp_(params.base.position_kp),
      position_kd_(params.base.position_kd),
      max_tilt_angle_(params.base.max_tilt_angle),
      altitude_kp_(params.base.altitude_kp),
      altitude_kd_(params.base.altitude_kd),
      attitude_kp_(params.base.attitude_kp),
      attitude_kd_(params.base.attitude_kd),
      tension_kp_(params.base.tension_feedback_kp),
      tension_altitude_gain_(params.base.tension_altitude_gain),
      tension_altitude_max_(params.base.tension_altitude_max),
      pickup_duration_(params.base.pickup_ramp_duration),
      pickup_target_tension_(params.base.pickup_target_tension),
      pickup_threshold_(params.base.pickup_detection_threshold),
      min_thrust_(params.base.min_thrust), max_thrust_(params.base.max_thrust),
      max_torque_(params.base.max_torque), gravity_(params.base.gravity),
      enable_fault_detection_(params.enable_fault_detection),
      fault_detect_tension_threshold_(params.fault_detect_tension_threshold),
      fault_detect_hold_time_(params.fault_detect_hold_time),
      fault_detect_arming_time_(params.fault_detect_arming_time),
      first_escape_delay_(params.first_escape_delay),
      second_escape_delay_(params.second_escape_delay),
      first_escape_distance_(params.first_escape_distance),
      second_escape_distance_(params.second_escape_distance),
      first_escape_climb_(params.first_escape_climb),
      second_escape_climb_(params.second_escape_climb),
      escape_tangent_bias_(params.escape_tangent_bias),
      escape_velocity_bias_(params.escape_velocity_bias) {
  const double altitude_change = final_altitude_ - initial_altitude_;
  ascent_duration_ =
      (climb_rate_ > 0.0) ? std::abs(altitude_change) / climb_rate_ : 0.0;
  ascent_direction_ = (altitude_change >= 0.0) ? 1.0 : -1.0;

  plant_state_port_ =
      DeclareVectorInputPort(
          "plant_state",
          BasicVector<double>(plant.num_positions() + plant.num_velocities()))
          .get_index();
  tension_port_ = DeclareVectorInputPort("rope_tension", BasicVector<double>(4))
                      .get_index();
  estimated_state_port_ =
      DeclareVectorInputPort("estimated_state", BasicVector<double>(6))
          .get_index();
  control_port_ = DeclareAbstractOutputPort(
                      "control_force",
                      &FaultDetectingQuadcopterController::CalcControlForce)
                      .get_index();
  control_vector_port_ =
      DeclareVectorOutputPort(
          "control_vector", 6,
          &FaultDetectingQuadcopterController::CalcControlVector)
          .get_index();
}

std::vector<TrajectoryWaypoint>
FaultDetectingQuadcopterController::BuildEscapeWaypoints(
    const Eigen::Vector3d &start_position,
    const Eigen::Vector3d &start_velocity,
    const Eigen::Vector3d &nominal_center_position, double start_time,
    double nominal_altitude) const {
  Eigen::Vector3d radial_direction = start_position - nominal_center_position;
  radial_direction.z() = 0.0;
  if (radial_direction.head<2>().norm() < 1e-6) {
    radial_direction = formation_offset_;
    radial_direction.z() = 0.0;
  }
  if (radial_direction.head<2>().norm() < 1e-6) {
    radial_direction = start_position;
    radial_direction.z() = 0.0;
  }
  if (radial_direction.head<2>().norm() < 1e-6) {
    radial_direction = Eigen::Vector3d::UnitX();
  } else {
    radial_direction.normalize();
  }

  Eigen::Vector3d tangent_direction(-radial_direction.y(), radial_direction.x(),
                                    0.0);
  const Eigen::Vector3d horizontal_velocity(start_velocity.x(),
                                            start_velocity.y(), 0.0);
  if (horizontal_velocity.head<2>().norm() > 0.15 &&
      tangent_direction.dot(horizontal_velocity) < 0.0) {
    tangent_direction *= -1.0;
  } else if (horizontal_velocity.head<2>().norm() <= 0.15 &&
             radial_direction.y() < 0.0) {
    tangent_direction *= -1.0;
  }

  Eigen::Vector3d corridor_direction =
      radial_direction + escape_tangent_bias_ * tangent_direction;
  if (horizontal_velocity.head<2>().norm() > 0.15) {
    corridor_direction +=
        escape_velocity_bias_ * horizontal_velocity.normalized();
  }
  corridor_direction.z() = 0.0;
  if (corridor_direction.head<2>().norm() < 1e-6) {
    corridor_direction = radial_direction;
  } else {
    corridor_direction.normalize();
  }

  const double first_time = start_time + first_escape_delay_;
  const double second_time = start_time + second_escape_delay_;
  const double target_altitude = std::max(start_position.z(), nominal_altitude);
  Eigen::Vector3d first_direction = radial_direction;
  if (horizontal_velocity.head<2>().norm() > 0.35 &&
      first_direction.dot(horizontal_velocity.normalized()) < -0.25) {
    first_direction =
        (first_direction + 0.25 * horizontal_velocity.normalized())
            .normalized();
  }
  const Eigen::Vector3d first_position =
      start_position + first_escape_distance_ * first_direction +
      Eigen::Vector3d(0.0, 0.0, first_escape_climb_);
  const Eigen::Vector3d second_position =
      start_position + second_escape_distance_ * corridor_direction +
      Eigen::Vector3d(0.0, 0.0, second_escape_climb_);

  return {
      {start_position, start_time, 0.05},
      {Eigen::Vector3d(first_position.x(), first_position.y(),
                       std::max(first_position.z(), target_altitude)),
       first_time, 0.10},
      {Eigen::Vector3d(second_position.x(), second_position.y(),
                       std::max(second_position.z(), target_altitude + 0.5)),
       second_time, 1000.0},
  };
}

void FaultDetectingQuadcopterController::MaybeActivateEscape(
    double time, double measured_tension, const Eigen::Vector3d &translation,
    const Eigen::Vector3d &translational_velocity,
    const Eigen::Vector3d &nominal_position, double nominal_altitude) const {
  if (!enable_fault_detection_ || escape_start_time_.has_value()) {
    return;
  }
  if (!pickup_start_time_.has_value()) {
    return;
  }
  if (time - *pickup_start_time_ < fault_detect_arming_time_) {
    return;
  }

  if (measured_tension <= fault_detect_tension_threshold_) {
    if (!low_tension_start_time_.has_value()) {
      low_tension_start_time_ = time;
      return;
    }
    if (time - *low_tension_start_time_ >= fault_detect_hold_time_) {
      escape_start_time_ = time;
      active_escape_waypoints_ = BuildEscapeWaypoints(
          translation, translational_velocity,
          nominal_position - formation_offset_, time, nominal_altitude);
    }
  } else {
    low_tension_start_time_.reset();
  }
}

void FaultDetectingQuadcopterController::ComputeTrajectory(
    double t, Eigen::Vector3d *pos_des, Eigen::Vector3d *vel_des) const {
  DRAKE_DEMAND(pos_des != nullptr);
  DRAKE_DEMAND(vel_des != nullptr);

  const std::vector<TrajectoryWaypoint> *waypoints = &nominal_waypoints_;
  const bool using_escape_waypoints = !active_escape_waypoints_.empty();
  if (using_escape_waypoints) {
    waypoints = &active_escape_waypoints_;
  }
  const Eigen::Vector3d waypoint_offset =
      using_escape_waypoints ? Eigen::Vector3d::Zero() : formation_offset_;

  if (!waypoints->empty()) {
    double segment_start_time = 0.0;
    for (size_t i = 0; i < waypoints->size(); ++i) {
      const auto &wp = (*waypoints)[i];
      const double segment_end_time = wp.arrival_time;
      const double hold_end_time = segment_end_time + wp.hold_time;

      if (t <= segment_end_time) {
        if (i == 0) {
          *pos_des = wp.position + waypoint_offset;
          vel_des->setZero();
        } else {
          const auto &prev_wp = (*waypoints)[i - 1];
          const double segment_duration = segment_end_time - segment_start_time;
          if (segment_duration > 1e-6) {
            const double alpha = (t - segment_start_time) / segment_duration;
            *pos_des = (1.0 - alpha) * prev_wp.position + alpha * wp.position +
                       waypoint_offset;
            *vel_des = (wp.position - prev_wp.position) / segment_duration;
          } else {
            *pos_des = wp.position + waypoint_offset;
            vel_des->setZero();
          }
        }
        return;
      }
      if (t <= hold_end_time) {
        *pos_des = wp.position + waypoint_offset;
        vel_des->setZero();
        return;
      }
      segment_start_time = hold_end_time;
    }
    *pos_des = waypoints->back().position + waypoint_offset;
    vel_des->setZero();
    return;
  }

  double desired_altitude{};
  double desired_velocity_z{};
  if (t <= ascent_start_time_ || climb_rate_ <= 0.0) {
    desired_altitude = initial_altitude_;
    desired_velocity_z = 0.0;
  } else if (t >= ascent_start_time_ + ascent_duration_) {
    desired_altitude = final_altitude_;
    desired_velocity_z = 0.0;
  } else {
    const double elapsed = t - ascent_start_time_;
    desired_altitude =
        initial_altitude_ + ascent_direction_ * climb_rate_ * elapsed;
    desired_velocity_z = ascent_direction_ * climb_rate_;
  }
  *pos_des = Eigen::Vector3d(formation_offset_.x(), formation_offset_.y(),
                             desired_altitude);
  *vel_des = Eigen::Vector3d(0.0, 0.0, desired_velocity_z);
}

void FaultDetectingQuadcopterController::CalcControlForce(
    const Context<double> &context,
    std::vector<ExternallyAppliedSpatialForce<double>> *output) const {
  const auto &state_vector = get_input_port(plant_state_port_).Eval(context);
  const auto &tension_data = get_input_port(tension_port_).Eval(context);
  const double measured_tension = tension_data[0];

  auto plant_context = plant_.CreateDefaultContext();
  plant_.SetPositionsAndVelocities(plant_context.get(), state_vector);

  const double t = context.get_time();
  if (!pickup_start_time_.has_value() &&
      measured_tension >= pickup_threshold_) {
    pickup_start_time_ = t;
  }

  const auto &quad_body = plant_.get_body(quad_body_index_);
  const auto &pose_world =
      plant_.EvalBodyPoseInWorld(*plant_context, quad_body);
  const auto &velocity_world =
      plant_.EvalBodySpatialVelocityInWorld(*plant_context, quad_body);

  Eigen::Vector3d translation;
  Eigen::Vector3d translational_vel;
  const auto &est_port = get_input_port(estimated_state_port_);
  if (est_port.HasValue(context)) {
    const auto &estimated = est_port.Eval(context);
    translation = Eigen::Vector3d(estimated[0], estimated[1], estimated[2]);
    translational_vel =
        Eigen::Vector3d(estimated[3], estimated[4], estimated[5]);
  } else {
    translation = pose_world.translation();
    translational_vel = velocity_world.translational();
  }

  Eigen::Vector3d pos_des;
  Eigen::Vector3d vel_des;
  ComputeTrajectory(t, &pos_des, &vel_des);
  MaybeActivateEscape(t, measured_tension, translation, translational_vel,
                      pos_des, pos_des.z());
  ComputeTrajectory(t, &pos_des, &vel_des);

  bool in_pickup_phase = false;
  double target_tension = pickup_target_tension_;
  if (!escape_start_time_.has_value() && pickup_start_time_.has_value()) {
    const double time_since_pickup = t - *pickup_start_time_;
    if (time_since_pickup >= 0.0 && time_since_pickup <= pickup_duration_) {
      in_pickup_phase = true;
      double ramp_fraction = time_since_pickup / pickup_duration_;
      ramp_fraction = std::clamp(ramp_fraction, 0.0, 1.0);
      target_tension = ramp_fraction * pickup_target_tension_;
      double altitude_adjustment =
          tension_altitude_gain_ * (target_tension - measured_tension);
      altitude_adjustment = std::clamp(
          altitude_adjustment, -tension_altitude_max_, tension_altitude_max_);
      pos_des.z() += altitude_adjustment;
    }
  }

  const Eigen::Vector3d pos_error = pos_des - translation;
  const Eigen::Vector3d vel_error = vel_des - translational_vel;
  const double ax_des =
      position_kp_ * pos_error.x() + position_kd_ * vel_error.x();
  const double ay_des =
      position_kp_ * pos_error.y() + position_kd_ * vel_error.y();
  const double pitch_des =
      std::clamp(ax_des / gravity_, -max_tilt_angle_, max_tilt_angle_);
  const double roll_des =
      std::clamp(-ay_des / gravity_, -max_tilt_angle_, max_tilt_angle_);

  const double commanded_accel_z =
      altitude_kp_ * pos_error.z() + altitude_kd_ * vel_error.z();
  double thrust = mass_ * (gravity_ + commanded_accel_z);
  const double tension_feedforward_limit =
      in_pickup_phase ? target_tension : pickup_target_tension_;
  thrust += std::clamp(measured_tension, 0.0, tension_feedforward_limit);
  if (in_pickup_phase) {
    thrust += tension_kp_ * (target_tension - measured_tension);
  }
  thrust = std::clamp(thrust, min_thrust_, max_thrust_);

  const Eigen::Matrix3d R = pose_world.rotation().matrix();
  const double current_roll = std::atan2(R(2, 1), R(2, 2));
  const double current_pitch = std::asin(-R(2, 0));
  const double current_yaw_error = 0.5 * (R(1, 0) - R(0, 1));
  const double roll_error = roll_des - current_roll;
  const double pitch_error = pitch_des - current_pitch;
  const double yaw_error = -current_yaw_error;

  const Eigen::Vector3d omega_W = velocity_world.rotational();
  const double omega_Bx =
      R(0, 0) * omega_W[0] + R(1, 0) * omega_W[1] + R(2, 0) * omega_W[2];
  const double omega_By =
      R(0, 1) * omega_W[0] + R(1, 1) * omega_W[1] + R(2, 1) * omega_W[2];
  const double omega_Bz =
      R(0, 2) * omega_W[0] + R(1, 2) * omega_W[1] + R(2, 2) * omega_W[2];

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

void FaultDetectingQuadcopterController::CalcControlVector(
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
    for (int i = 0; i < 6; ++i) {
      output->SetAtIndex(i, 0.0);
    }
  }
}

} // namespace tether_grace