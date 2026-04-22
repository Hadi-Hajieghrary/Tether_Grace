#pragma once

#include <Eigen/Core>
#include <drake/systems/framework/leaf_system.h>

namespace quad_rope_lift {

/// Extracts state for TWO drones from a shared plant state
/// Input: Full plant state
/// Outputs (x 2 drones):
///   0,1: Drone 0 position, velocity
///   2,3: Drone 0 cable tension, direction
///   4,5: Drone 1 position, velocity
///   6,7: Drone 1 cable tension, direction
class DualPlantStateSplitter : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DualPlantStateSplitter);

  DualPlantStateSplitter(int num_beads, double rope_length);

 private:
  void CalcDrone0Position(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* output) const;
  void CalcDrone0Velocity(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* output) const;
  void CalcDrone0Tension(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* output) const;
  void CalcDrone0Direction(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* output) const;

  void CalcDrone1Position(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* output) const;
  void CalcDrone1Velocity(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* output) const;
  void CalcDrone1Tension(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* output) const;
  void CalcDrone1Direction(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* output) const;

  int num_beads_;
  double rope_length_;
};

}  // namespace quad_rope_lift
