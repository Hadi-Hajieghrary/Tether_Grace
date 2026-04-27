#pragma once
///
/// @file safe_hover_controller.h
///
/// Drake LeafSystem that commands a quadcopter to fly to and hold a fixed
/// world-frame hover position using a PD cascade (same gains / structure as
/// QuadcopterLiftController but with a stationary setpoint and no tension
/// feedback).  Used for the faulty quadcopter after its tether is cut.

#include <algorithm>
#include <cmath>
#include <vector>

#include <Eigen/Core>

#include <drake/multibody/math/spatial_force.h>
#include <drake/multibody/plant/externally_applied_spatial_force.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/basic_vector.h>
#include <drake/systems/framework/leaf_system.h>

namespace quad_rope_lift {

/// A simple "fly-to-and-hold" controller for a quadcopter whose tether has
/// been severed.  It ignores the formation trajectory and all tension inputs;
/// it only uses plant state to PD-track @p hover_position.
class SafeHoverController final : public drake::systems::LeafSystem<double> {
public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SafeHoverController);

  /// @param plant            The MultibodyPlant containing all bodies.
  /// @param body             The quadcopter RigidBody to control.
  /// @param hover_position   World-frame target position [m].
  /// @param mass             Quadcopter mass [kg] (may be updated with
  /// set_mass).
  /// @param position_kp/kd  Horizontal PD gains (default: match
  /// QuadcopterLiftController).
  /// @param altitude_kp/kd  Vertical PD gains.
  /// @param attitude_kp/kd  Attitude PD gains.
  SafeHoverController(const drake::multibody::MultibodyPlant<double> &plant,
                      const drake::multibody::RigidBody<double> &body,
                      Eigen::Vector3d hover_position, double mass,
                      double position_kp = 15.0, double position_kd = 7.0,
                      double altitude_kp = 25.0, double altitude_kd = 12.0,
                      double attitude_kp = 15.0, double attitude_kd = 2.5,
                      double max_tilt_angle = 0.4, double max_thrust = 150.0,
                      double max_torque = 10.0, double gravity = 9.81)
      : plant_(plant), body_index_(body.index()),
        hover_position_(std::move(hover_position)), mass_(mass),
        position_kp_(position_kp), position_kd_(position_kd),
        altitude_kp_(altitude_kp), altitude_kd_(altitude_kd),
        attitude_kp_(attitude_kp), attitude_kd_(attitude_kd),
        max_tilt_angle_(max_tilt_angle), max_thrust_(max_thrust),
        max_torque_(max_torque), gravity_(gravity) {
    plant_state_port_ =
        DeclareVectorInputPort(
            "plant_state", drake::systems::BasicVector<double>(
                               plant.num_positions() + plant.num_velocities()))
            .get_index();

    control_port_ = DeclareAbstractOutputPort(
                        "control_force", &SafeHoverController::CalcControlForce)
                        .get_index();

    control_vector_port_ =
        DeclareVectorOutputPort("control_vector", 6,
                                &SafeHoverController::CalcControlVector)
            .get_index();
  }

  /// Update mass (call after welding visual model, mirroring
  /// QuadcopterLiftController).
  void set_mass(double m) { mass_ = m; }

  const drake::systems::InputPort<double> &get_plant_state_input_port() const {
    return get_input_port(plant_state_port_);
  }
  const drake::systems::OutputPort<double> &get_control_output_port() const {
    return get_output_port(control_port_);
  }
  const drake::systems::OutputPort<double> &
  get_control_vector_output_port() const {
    return get_output_port(control_vector_port_);
  }

private:
  void CalcControlForce(
      const drake::systems::Context<double> &context,
      std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>
          *output) const {
    using drake::multibody::ExternallyAppliedSpatialForce;
    using drake::multibody::SpatialForce;

    // Read plant state.
    const auto &state_vec = get_input_port(plant_state_port_).Eval(context);
    auto plant_ctx = plant_.CreateDefaultContext();
    plant_.SetPositionsAndVelocities(plant_ctx.get(), state_vec);

    const auto &body = plant_.get_body(body_index_);
    const auto &pose = plant_.EvalBodyPoseInWorld(*plant_ctx, body);
    const auto &vel_spatial =
        plant_.EvalBodySpatialVelocityInWorld(*plant_ctx, body);

    const Eigen::Vector3d pos = pose.translation();
    const Eigen::Vector3d vel_trans = vel_spatial.translational();
    const Eigen::Matrix3d R = pose.rotation().matrix();
    const Eigen::Vector3d omega_W = vel_spatial.rotational();

    // --- Position / altitude PD ---
    const Eigen::Vector3d pos_err = hover_position_ - pos;
    const Eigen::Vector3d vel_err = -vel_trans; // desired velocity = 0

    const double ax_des =
        position_kp_ * pos_err.x() + position_kd_ * vel_err.x();
    const double ay_des =
        position_kp_ * pos_err.y() + position_kd_ * vel_err.y();
    const double pitch_des =
        std::clamp(ax_des / gravity_, -max_tilt_angle_, max_tilt_angle_);
    const double roll_des =
        std::clamp(-ay_des / gravity_, -max_tilt_angle_, max_tilt_angle_);

    const double az_des =
        altitude_kp_ * pos_err.z() + altitude_kd_ * vel_err.z();
    const double thrust =
        std::clamp(mass_ * (gravity_ + az_des), 0.0, max_thrust_);

    // --- Attitude PD ---
    const double current_roll = std::atan2(R(2, 1), R(2, 2));
    const double current_pitch = std::asin(-R(2, 0));
    const double yaw_skew = 0.5 * (R(1, 0) - R(0, 1));

    const double omega_Bx =
        R(0, 0) * omega_W[0] + R(1, 0) * omega_W[1] + R(2, 0) * omega_W[2];
    const double omega_By =
        R(0, 1) * omega_W[0] + R(1, 1) * omega_W[1] + R(2, 1) * omega_W[2];
    const double omega_Bz =
        R(0, 2) * omega_W[0] + R(1, 2) * omega_W[1] + R(2, 2) * omega_W[2];

    const double tau_x = std::clamp(attitude_kp_ * (roll_des - current_roll) -
                                        attitude_kd_ * omega_Bx,
                                    -max_torque_, max_torque_);
    const double tau_y = std::clamp(attitude_kp_ * (pitch_des - current_pitch) -
                                        attitude_kd_ * omega_By,
                                    -max_torque_, max_torque_);
    const double tau_z =
        std::clamp(attitude_kp_ * (-yaw_skew) - attitude_kd_ * omega_Bz,
                   -max_torque_, max_torque_);

    const Eigen::Vector3d torque_W = R * Eigen::Vector3d(tau_x, tau_y, tau_z);
    const Eigen::Vector3d force_W = R.col(2) * thrust;

    output->clear();
    ExternallyAppliedSpatialForce<double> w;
    w.body_index = body_index_;
    w.p_BoBq_B = Eigen::Vector3d::Zero();
    w.F_Bq_W = SpatialForce<double>(torque_W, force_W);
    output->push_back(w);
  }

  void CalcControlVector(const drake::systems::Context<double> &context,
                         drake::systems::BasicVector<double> *output) const {
    std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>> forces;
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

  const drake::multibody::MultibodyPlant<double> &plant_;
  drake::multibody::BodyIndex body_index_;
  Eigen::Vector3d hover_position_;
  double mass_;

  double position_kp_, position_kd_;
  double altitude_kp_, altitude_kd_;
  double attitude_kp_, attitude_kd_;
  double max_tilt_angle_, max_thrust_, max_torque_, gravity_;

  int plant_state_port_{-1};
  int control_port_{-1};
  int control_vector_port_{-1};
};

} // namespace quad_rope_lift
