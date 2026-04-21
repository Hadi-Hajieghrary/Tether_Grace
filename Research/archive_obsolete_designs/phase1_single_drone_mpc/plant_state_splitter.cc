#include "plant_state_splitter.h"

#include <cmath>
#include <iostream>

namespace quad_rope_lift {

PlantStateSplitter::PlantStateSplitter(int num_beads, double rope_length)
    : num_beads_(num_beads), rope_length_(rope_length) {

  // Input port: plant state from MultibodyPlant
  // Drake free-floating bodies: position (7D) + velocity (6D) per body
  // System: 1 quad + 1 payload + num_beads beads = (2 + num_beads) bodies
  // Total state: (2 + num_beads) * (7 + 6) = (2 + num_beads) * 13
  int num_bodies = 2 + num_beads;  // quad + payload + beads
  int state_size = num_bodies * 13;  // 7D positions + 6D velocities per body

  DeclareInputPort("plant_state", drake::systems::kVectorValued, state_size);

  // Output ports using BasicVector
  DeclareVectorOutputPort("p_quad", drake::systems::BasicVector<double>(3),
                          &PlantStateSplitter::CalcDronePosition);
  DeclareVectorOutputPort("v_quad", drake::systems::BasicVector<double>(3),
                          &PlantStateSplitter::CalcDroneVelocity);
  DeclareVectorOutputPort("T_cable", drake::systems::BasicVector<double>(1),
                          &PlantStateSplitter::CalcCableTension);
  DeclareVectorOutputPort("n_cable", drake::systems::BasicVector<double>(3),
                          &PlantStateSplitter::CalcCableDirection);
}

void PlantStateSplitter::CalcDronePosition(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output) const {

  auto plant_state = get_input_port(0).Eval<drake::systems::BasicVector<double>>(context).value();

  // State layout:
  // [p_quad(3), p_payload(3), p_beads(3*N), v_quad(3), v_payload(3), v_beads(3*N)]
  // p_quad is at indices 0-2

  if (plant_state.size() < 3) {
    std::cerr << "PlantStateSplitter: plant_state too small (" << plant_state.size()
              << " < 3)\n";
    output->SetZero();
    return;
  }

  output->SetFromVector(plant_state.segment<3>(0));
}

void PlantStateSplitter::CalcDroneVelocity(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output) const {

  auto plant_state = get_input_port(0).Eval<drake::systems::BasicVector<double>>(context).value();

  // v_quad starts at index 6 + 3*num_beads
  int v_quad_idx = 6 + 3 * num_beads_;

  if (plant_state.size() < v_quad_idx + 3) {
    std::cerr << "PlantStateSplitter: plant_state too small for v_quad access\n";
    output->SetZero();
    return;
  }

  output->SetFromVector(plant_state.segment<3>(v_quad_idx));
}

void PlantStateSplitter::CalcCableDirection(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output) const {

  auto plant_state = get_input_port(0).Eval<drake::systems::BasicVector<double>>(context).value();

  // Extract positions
  Eigen::Vector3d p_quad = plant_state.segment<3>(0);      // indices 0-2
  Eigen::Vector3d p_payload = plant_state.segment<3>(3);   // indices 3-5

  // Cable direction: from quad to payload (direction load is being pulled)
  Eigen::Vector3d cable_vec = p_payload - p_quad;
  double cable_len = cable_vec.norm();

  // Normalize with safety check for degenerate case
  const double MIN_LENGTH = 1e-6;
  if (cable_len < MIN_LENGTH) {
    // Quad and payload coincident or very close
    output->SetFromVector(Eigen::Vector3d(0.0, 0.0, -1.0));
  } else {
    output->SetFromVector(cable_vec / cable_len);
  }
}

void PlantStateSplitter::CalcCableTension(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output) const {

  auto plant_state = get_input_port(0).Eval<drake::systems::BasicVector<double>>(context).value();

  // Simplified quasi-static estimate: T ≈ m_payload * g
  // In Phase 2, this would be integrated with full dynamics estimation
  const double m_payload = 3.0;
  const double g = 9.81;

  Eigen::VectorXd tension(1);
  tension(0) = m_payload * g;
  output->SetFromVector(tension);
}

}  // namespace quad_rope_lift
