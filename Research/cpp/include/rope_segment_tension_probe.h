#pragma once

#include <memory>
#include <mutex>
#include <vector>

#include <Eigen/Core>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/basic_vector.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/framework/leaf_system.h>

namespace quad_rope_lift {

/// Diagnostic probe that computes the per-segment tension of a single rope
/// (the full bead-chain from drone attachment down to payload attachment).
///
/// Mirrors the tension-only Kelvin–Voigt model used by `RopeForceSystem`
/// internally so the two are numerically identical, but exposes ALL
/// segments (not just the top one) for visualisation / paper figures —
/// specifically the rope-tension waterfall plot:
///
///     T_{j}(t), j = 1 .. N_seg = N_beads + 1
///
/// where T_1 is the segment between drone and bead 1 (top), T_{N_seg} is
/// the segment between bead N_beads and payload (bottom). The output is
/// purely a scalar magnitude ≥ 0 (one-sided rope).
///
/// Does NOT apply forces to the plant. Purely an observer.
class RopeSegmentTensionProbe final
    : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RopeSegmentTensionProbe);

  RopeSegmentTensionProbe(
      const drake::multibody::MultibodyPlant<double>& plant,
      const drake::multibody::RigidBody<double>& quadcopter_body,
      const drake::multibody::RigidBody<double>& payload_body,
      const std::vector<const drake::multibody::RigidBody<double>*>&
          bead_bodies,
      const Eigen::Vector3d& quadcopter_attachment_point,
      const Eigen::Vector3d& payload_attachment_point,
      double rope_rest_length,
      double segment_stiffness,
      double segment_damping,
      double min_distance_threshold = 1e-9)
      : plant_(plant),
        segment_stiffness_(segment_stiffness),
        segment_damping_(segment_damping),
        min_distance_(min_distance_threshold) {
    const int num_beads = static_cast<int>(bead_bodies.size());
    num_segments_ = num_beads + 1;
    segment_rest_length_ = rope_rest_length / num_segments_;

    body_indices_.reserve(num_beads + 2);
    attachment_points_.reserve(num_beads + 2);
    body_indices_.push_back(quadcopter_body.index());
    attachment_points_.push_back(quadcopter_attachment_point);
    for (const auto* bead : bead_bodies) {
      body_indices_.push_back(bead->index());
      attachment_points_.push_back(Eigen::Vector3d::Zero());
    }
    body_indices_.push_back(payload_body.index());
    attachment_points_.push_back(payload_attachment_point);

    plant_state_port_ =
        DeclareVectorInputPort(
            "plant_state",
            drake::systems::BasicVector<double>(plant.num_positions()
                                                + plant.num_velocities()))
            .get_index();
    tensions_port_ =
        DeclareVectorOutputPort(
            "segment_tensions",
            drake::systems::BasicVector<double>(num_segments_),
            &RopeSegmentTensionProbe::CalcSegmentTensions)
            .get_index();
  }

  const drake::systems::InputPort<double>& get_plant_state_input_port() const {
    return get_input_port(plant_state_port_);
  }
  const drake::systems::OutputPort<double>& get_tensions_output_port() const {
    return get_output_port(tensions_port_);
  }

  int num_segments() const { return num_segments_; }

 private:
  void CalcSegmentTensions(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* output) const {
    const auto& state_vector = get_input_port(plant_state_port_).Eval(context);
    // Cache a single plant context per probe and reuse it across ticks.
    // plant_.CreateDefaultContext() is expensive — doing it every tick
    // dominates wall time for large bead-chain plants.
    std::lock_guard<std::mutex> lock(ctx_mtx_);
    if (!plant_context_) {
      plant_context_ = plant_.CreateDefaultContext();
    }
    plant_.SetPositionsAndVelocities(plant_context_.get(), state_vector);

    const int nb = static_cast<int>(body_indices_.size());
    std::vector<Eigen::Vector3d> p(nb), v(nb);
    for (int i = 0; i < nb; ++i) {
      const auto& body = plant_.get_body(body_indices_[i]);
      const auto& X = plant_.EvalBodyPoseInWorld(*plant_context_, body);
      const auto& V = plant_.EvalBodySpatialVelocityInWorld(*plant_context_, body);
      p[i] = X * attachment_points_[i];
      const Eigen::Vector3d rW = X.rotation() * attachment_points_[i];
      v[i] = V.translational() + V.rotational().cross(rW);
    }

    Eigen::VectorXd T = Eigen::VectorXd::Zero(num_segments_);
    for (int j = 0; j < num_segments_; ++j) {
      const Eigen::Vector3d d = p[j] - p[j + 1];
      const double dist = d.norm();
      if (dist < min_distance_) continue;
      const double stretch = dist - segment_rest_length_;
      if (stretch <= 0.0) continue;
      const Eigen::Vector3d e = d / dist;
      const double sdot = e.dot(v[j] - v[j + 1]);
      double tension = segment_stiffness_ * stretch;
      if (sdot > 0.0) tension += segment_damping_ * sdot;
      if (tension < 0.0) tension = 0.0;
      T(j) = tension;
    }
    output->set_value(T);
  }

  const drake::multibody::MultibodyPlant<double>& plant_;
  std::vector<drake::multibody::BodyIndex> body_indices_;
  std::vector<Eigen::Vector3d> attachment_points_;
  int num_segments_{0};
  double segment_rest_length_{0.0};
  double segment_stiffness_;
  double segment_damping_;
  double min_distance_;
  int plant_state_port_{-1};
  int tensions_port_{-1};
  mutable std::unique_ptr<drake::systems::Context<double>> plant_context_;
  mutable std::mutex ctx_mtx_;
};

}  // namespace quad_rope_lift
