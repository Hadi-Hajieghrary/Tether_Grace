#include "gpac_fault_tolerance/drake/faultable_rope_force_system.h"

#include <drake/multibody/math/spatial_force.h>

namespace tether_grace {

using drake::multibody::ExternallyAppliedSpatialForce;
using drake::multibody::MultibodyPlant;
using drake::multibody::RigidBody;
using drake::multibody::SpatialForce;
using drake::systems::BasicVector;
using drake::systems::Context;

FaultableRopeForceSystem::FaultableRopeForceSystem(
    const MultibodyPlant<double> &plant,
    const RigidBody<double> &quadcopter_body,
    const RigidBody<double> &payload_body,
    const std::vector<const RigidBody<double> *> &bead_bodies,
    const Eigen::Vector3d &quadcopter_attachment_point,
    const Eigen::Vector3d &payload_attachment_point, double rope_rest_length,
    double segment_stiffness, double segment_damping,
    std::optional<double> sever_time, double min_distance_threshold)
    : plant_(plant), segment_stiffness_(segment_stiffness),
      segment_damping_(segment_damping), min_distance_(min_distance_threshold),
      sever_time_(sever_time) {
  const int num_beads = static_cast<int>(bead_bodies.size());
  const int num_bodies = num_beads + 2;
  num_segments_ = num_beads + 1;
  segment_rest_length_ = rope_rest_length / num_segments_;

  body_indices_.reserve(num_bodies);
  attachment_points_.reserve(num_bodies);

  body_indices_.push_back(quadcopter_body.index());
  attachment_points_.push_back(quadcopter_attachment_point);

  for (const auto *bead : bead_bodies) {
    body_indices_.push_back(bead->index());
    attachment_points_.push_back(Eigen::Vector3d::Zero());
  }

  body_indices_.push_back(payload_body.index());
  attachment_points_.push_back(payload_attachment_point);

  plant_state_port_ =
      DeclareVectorInputPort(
          "plant_state",
          BasicVector<double>(plant.num_positions() + plant.num_velocities()))
          .get_index();

  forces_port_ =
      DeclareAbstractOutputPort("external_forces",
                                &FaultableRopeForceSystem::CalcRopeForces)
          .get_index();

  tension_port_ =
      DeclareVectorOutputPort("top_segment_tension", BasicVector<double>(4),
                              &FaultableRopeForceSystem::CalcTopSegmentTension)
          .get_index();
}

bool FaultableRopeForceSystem::is_severed(double time) const {
  return sever_time_.has_value() && time >= *sever_time_;
}

void FaultableRopeForceSystem::CalcRopeForces(
    const Context<double> &context,
    std::vector<ExternallyAppliedSpatialForce<double>> *output) const {
  const int num_bodies = static_cast<int>(body_indices_.size());
  output->clear();
  output->reserve(num_bodies);

  if (is_severed(context.get_time())) {
    for (int i = 0; i < num_bodies; ++i) {
      ExternallyAppliedSpatialForce<double> spatial_force;
      spatial_force.body_index = body_indices_[i];
      spatial_force.p_BoBq_B = attachment_points_[i];
      spatial_force.F_Bq_W = SpatialForce<double>(Eigen::Vector3d::Zero(),
                                                  Eigen::Vector3d::Zero());
      output->push_back(spatial_force);
    }
    return;
  }

  const auto &state_vector = get_input_port(plant_state_port_).Eval(context);
  auto plant_context = plant_.CreateDefaultContext();
  plant_.SetPositionsAndVelocities(plant_context.get(), state_vector);

  std::vector<Eigen::Vector3d> positions(num_bodies);
  std::vector<Eigen::Vector3d> velocities(num_bodies);

  for (int i = 0; i < num_bodies; ++i) {
    const auto &body = plant_.get_body(body_indices_[i]);
    const auto &X_WB = plant_.EvalBodyPoseInWorld(*plant_context, body);
    const auto &V_WB =
        plant_.EvalBodySpatialVelocityInWorld(*plant_context, body);
    const Eigen::Vector3d &p_BoP_B = attachment_points_[i];
    positions[i] = X_WB * p_BoP_B;
    const Eigen::Vector3d r_W = X_WB.rotation() * p_BoP_B;
    velocities[i] = V_WB.translational() + V_WB.rotational().cross(r_W);
  }

  std::vector<Eigen::Vector3d> net_forces(num_bodies, Eigen::Vector3d::Zero());

  for (int i = 0; i < num_segments_; ++i) {
    const Eigen::Vector3d d = positions[i] - positions[i + 1];
    const double dist = d.norm();
    if (dist < min_distance_) {
      continue;
    }
    const double stretch = dist - segment_rest_length_;
    if (stretch <= 0.0) {
      continue;
    }

    const Eigen::Vector3d e = d / dist;
    const Eigen::Vector3d dv = velocities[i] - velocities[i + 1];
    const double stretch_rate = e.dot(dv);

    double tension = segment_stiffness_ * stretch;
    if (stretch_rate > 0.0) {
      tension += segment_damping_ * stretch_rate;
    }
    if (tension <= 0.0) {
      continue;
    }

    const Eigen::Vector3d force = -tension * e;
    net_forces[i] += force;
    net_forces[i + 1] -= force;
  }

  for (int i = 0; i < num_bodies; ++i) {
    ExternallyAppliedSpatialForce<double> spatial_force;
    spatial_force.body_index = body_indices_[i];
    spatial_force.p_BoBq_B = attachment_points_[i];
    spatial_force.F_Bq_W =
        SpatialForce<double>(Eigen::Vector3d::Zero(), net_forces[i]);
    output->push_back(spatial_force);
  }
}

void FaultableRopeForceSystem::CalcTopSegmentTension(
    const Context<double> &context, BasicVector<double> *output) const {
  if (is_severed(context.get_time())) {
    output->set_value(Eigen::Vector4d::Zero());
    return;
  }

  const auto &state_vector = get_input_port(plant_state_port_).Eval(context);
  auto plant_context = plant_.CreateDefaultContext();
  plant_.SetPositionsAndVelocities(plant_context.get(), state_vector);

  const auto &body0 = plant_.get_body(body_indices_[0]);
  const auto &body1 = plant_.get_body(body_indices_[1]);
  const auto &X_WB0 = plant_.EvalBodyPoseInWorld(*plant_context, body0);
  const auto &X_WB1 = plant_.EvalBodyPoseInWorld(*plant_context, body1);
  const auto &V_WB0 =
      plant_.EvalBodySpatialVelocityInWorld(*plant_context, body0);
  const auto &V_WB1 =
      plant_.EvalBodySpatialVelocityInWorld(*plant_context, body1);

  const Eigen::Vector3d p0 = X_WB0 * attachment_points_[0];
  const Eigen::Vector3d p1 = X_WB1 * attachment_points_[1];
  const Eigen::Vector3d r0_W = X_WB0.rotation() * attachment_points_[0];
  const Eigen::Vector3d r1_W = X_WB1.rotation() * attachment_points_[1];
  const Eigen::Vector3d v0 =
      V_WB0.translational() + V_WB0.rotational().cross(r0_W);
  const Eigen::Vector3d v1 =
      V_WB1.translational() + V_WB1.rotational().cross(r1_W);

  const Eigen::Vector3d d = p0 - p1;
  const double dist = d.norm();
  Eigen::Vector4d result = Eigen::Vector4d::Zero();

  if (dist >= min_distance_) {
    const double stretch = dist - segment_rest_length_;
    if (stretch > 0.0) {
      const Eigen::Vector3d e = d / dist;
      const double stretch_rate = e.dot(v0 - v1);
      double tension = segment_stiffness_ * stretch;
      if (stretch_rate > 0.0) {
        tension += segment_damping_ * stretch_rate;
      }
      if (tension > 0.0) {
        const Eigen::Vector3d force_world = -tension * e;
        result << tension, force_world;
      }
    }
  }

  output->set_value(result);
}

} // namespace tether_grace