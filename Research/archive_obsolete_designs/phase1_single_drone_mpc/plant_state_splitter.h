#pragma once

#include <Eigen/Core>
#include <drake/systems/framework/leaf_system.h>

namespace quad_rope_lift {

/// Extracts drone state and cable measurements from plant state vector.
///
/// Input: Full plant state (positions and velocities of quad, payload, rope beads)
/// Outputs:
///   0: Drone position p_quad (3D)
///   1: Drone velocity v_quad (3D)
///   2: Cable tension T_cable (scalar)
///   3: Cable direction n_cable (3D unit vector)
class PlantStateSplitter : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PlantStateSplitter);

  /// @param num_beads Number of beads in rope model (for state indexing)
  /// @param rope_length Rope length in meters (for constraint validation)
  PlantStateSplitter(int num_beads, double rope_length);

 private:
  // Output calculation methods
  void CalcDronePosition(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* output) const;

  void CalcDroneVelocity(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* output) const;

  void CalcCableTension(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* output) const;

  void CalcCableDirection(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* output) const;

  // Configuration
  int num_beads_;
  double rope_length_;
};

}  // namespace quad_rope_lift
