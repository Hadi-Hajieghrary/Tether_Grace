/// @file estimated_state_override.cc
/// @brief Implementation of EstimatedStateOverride — replaces a single
///        free body's position and velocity in the plant state with ESKF
///        estimates.

#include "gpac_fault_tolerance/drake/estimated_state_override.h"

#include <drake/common/drake_assert.h>
#include <drake/math/rigid_transform.h>
#include <drake/math/rotation_matrix.h>
#include <drake/multibody/math/spatial_velocity.h>

namespace tether_grace {

using drake::math::RigidTransformd;
using drake::math::RotationMatrixd;
using drake::multibody::SpatialVelocity;
using drake::systems::BasicVector;
using drake::systems::Context;

EstimatedStateOverride::EstimatedStateOverride(
    const drake::multibody::MultibodyPlant<double> &plant,
    const drake::multibody::RigidBody<double> &body)
    : plant_(plant), body_index_(body.index()),
      state_size_(plant.num_positions() + plant.num_velocities()) {

  DRAKE_DEMAND(body.is_floating());

  plant_state_port_ =
      DeclareVectorInputPort("plant_state", BasicVector<double>(state_size_))
          .get_index();

  // ESKF estimated_pose: [pos_x, pos_y, pos_z, qw, qx, qy, qz] (7)
  estimated_pose_port_ =
      DeclareVectorInputPort("estimated_pose", BasicVector<double>(7))
          .get_index();

  // ESKF estimated_velocity: [vx, vy, vz] (3)
  estimated_velocity_port_ =
      DeclareVectorInputPort("estimated_velocity", BasicVector<double>(3))
          .get_index();

  output_port_ =
      DeclareVectorOutputPort("overridden_state", state_size_,
                              &EstimatedStateOverride::CalcOverriddenState)
          .get_index();
}

void EstimatedStateOverride::CalcOverriddenState(
    const Context<double> &context, BasicVector<double> *output) const {
  // Start with ground truth
  const auto &plant_state = get_input_port(plant_state_port_).Eval(context);
  auto out = output->get_mutable_value();
  out = plant_state;

  // Use a temporary MBP context to resolve DOF to state-vector mapping:
  // set the modified body's pose and velocity, then read back the state.
  auto plant_context = plant_.CreateDefaultContext();
  plant_.SetPositionsAndVelocities(plant_context.get(), plant_state);

  // Read ESKF outputs
  const auto &pose = get_input_port(estimated_pose_port_).Eval(context);
  const Eigen::Vector3d position = pose.head<3>();
  const Eigen::Vector4d quat = pose.tail<4>(); // [w, x, y, z]

  const auto &vel = get_input_port(estimated_velocity_port_).Eval(context);

  // Override pose
  const Eigen::Quaterniond q(quat[0], quat[1], quat[2], quat[3]);
  plant_.SetFreeBodyPose(plant_context.get(), plant_.get_body(body_index_),
                         RigidTransformd(RotationMatrixd(q), position));

  // Override translational velocity; keep angular velocity from ground truth
  const auto &gt_spatial_vel = plant_.EvalBodySpatialVelocityInWorld(
      *plant_context, plant_.get_body(body_index_));
  plant_.SetFreeBodySpatialVelocity(
      plant_context.get(), plant_.get_body(body_index_),
      SpatialVelocity<double>(gt_spatial_vel.rotational(), vel));

  // Read back the modified state vector
  out = plant_.GetPositionsAndVelocities(*plant_context);
}

} // namespace tether_grace
