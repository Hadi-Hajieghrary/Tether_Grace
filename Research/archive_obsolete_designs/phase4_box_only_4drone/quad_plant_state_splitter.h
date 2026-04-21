#pragma once

#include <vector>
#include <Eigen/Core>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/leaf_system.h>

namespace quad_rope_lift {

/// Extracts state measurements for 4 drones from a single multibody plant.
/// Outputs 16 ports: (position, velocity, cable_tension, cable_direction) × 4 drones
class QuadPlantStateSplitter : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(QuadPlantStateSplitter);

  /// @param num_beads Number of beads per rope (e.g., 8)
  /// @param rope_length Total rope length in meters
  QuadPlantStateSplitter(int num_beads, double rope_length);

  /// Get output ports for drone i (i = 0, 1, 2, 3)
  /// Returns indices: base_idx = i*4
  ///   base_idx + 0: drone position (3D)
  ///   base_idx + 1: drone velocity (3D)
  ///   base_idx + 2: cable tension (1D scalar)
  ///   base_idx + 3: cable direction (3D unit vector)
  const drake::systems::OutputPort<double>& get_drone_position_output_port(int drone_idx) const {
    return get_output_port(drone_idx * 4 + 0);
  }

  const drake::systems::OutputPort<double>& get_drone_velocity_output_port(int drone_idx) const {
    return get_output_port(drone_idx * 4 + 1);
  }

  const drake::systems::OutputPort<double>& get_cable_tension_output_port(int drone_idx) const {
    return get_output_port(drone_idx * 4 + 2);
  }

  const drake::systems::OutputPort<double>& get_cable_direction_output_port(int drone_idx) const {
    return get_output_port(drone_idx * 4 + 3);
  }

 private:
  void CalcDronePosition(int drone_idx,
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* output) const;

  void CalcDroneVelocity(int drone_idx,
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* output) const;

  void CalcCableTension(int drone_idx,
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* output) const;

  void CalcCableDirection(int drone_idx,
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* output) const;

  int num_beads_;
  double rope_length_;
  int num_bodies_per_drone_;  // 1 quad + 1 payload_contribution + num_beads
};

}  // namespace quad_rope_lift
