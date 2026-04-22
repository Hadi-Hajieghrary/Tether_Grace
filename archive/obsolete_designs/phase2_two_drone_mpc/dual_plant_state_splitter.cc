#include "dual_plant_state_splitter.h"

#include <cmath>
#include <iostream>

namespace quad_rope_lift {

DualPlantStateSplitter::DualPlantStateSplitter(int num_beads, double rope_length)
    : num_beads_(num_beads), rope_length_(rope_length) {
  // Input port: plant state from MultibodyPlant
  // 2 quads + 1 payload + 2*num_beads beads (2 ropes with num_beads each)
  int num_bodies = 2 + 1 + 2 * num_beads;  // 2 quads + 1 payload + 2 sets of beads
  int state_size = num_bodies * 13;  // 7D pos + 6D vel per body
  DeclareInputPort("plant_state", drake::systems::kVectorValued, state_size);

  // Output ports for drone 0
  DeclareVectorOutputPort("p_drone0", drake::systems::BasicVector<double>(3),
                          &DualPlantStateSplitter::CalcDrone0Position);
  DeclareVectorOutputPort("v_drone0", drake::systems::BasicVector<double>(3),
                          &DualPlantStateSplitter::CalcDrone0Velocity);
  DeclareVectorOutputPort("T_cable0", drake::systems::BasicVector<double>(1),
                          &DualPlantStateSplitter::CalcDrone0Tension);
  DeclareVectorOutputPort("n_cable0", drake::systems::BasicVector<double>(3),
                          &DualPlantStateSplitter::CalcDrone0Direction);

  // Output ports for drone 1
  DeclareVectorOutputPort("p_drone1", drake::systems::BasicVector<double>(3),
                          &DualPlantStateSplitter::CalcDrone1Position);
  DeclareVectorOutputPort("v_drone1", drake::systems::BasicVector<double>(3),
                          &DualPlantStateSplitter::CalcDrone1Velocity);
  DeclareVectorOutputPort("T_cable1", drake::systems::BasicVector<double>(1),
                          &DualPlantStateSplitter::CalcDrone1Tension);
  DeclareVectorOutputPort("n_cable1", drake::systems::BasicVector<double>(3),
                          &DualPlantStateSplitter::CalcDrone1Direction);
}

// ============ DRONE 0 ============

void DualPlantStateSplitter::CalcDrone0Position(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output) const {
  auto plant_state =
      get_input_port(0).Eval<drake::systems::BasicVector<double>>(context).value();
  output->SetFromVector(plant_state.segment<3>(0));
}

void DualPlantStateSplitter::CalcDrone0Velocity(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output) const {
  auto plant_state =
      get_input_port(0).Eval<drake::systems::BasicVector<double>>(context).value();
  // Velocity at index 6 + 3*num_beads_base + 3 (for both quad and payload positions)
  // Actually: quad0 pos (7D) + payload pos (7D) + beads pos + quad0 vel = 7 + 7 + 7*num_beads + base
  // Simplified: state is [q0_pos(7), q1_pos(7), payload_pos(7), rope0_beads_pos(7*n),
  // rope1_beads_pos(7*n), q0_vel(6), ...]
  // For now: positions are 7*(num_bodies_pos), then velocities
  int num_bodies_pos = 2 + num_beads_;  // quad0, quad1, payload, rope0_beads
  int v_quad0_idx = 7 * num_bodies_pos;

  if (plant_state.size() < v_quad0_idx + 3) {
    output->SetZero();
    return;
  }
  output->SetFromVector(plant_state.segment<3>(v_quad0_idx));
}

void DualPlantStateSplitter::CalcDrone0Tension(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output) const {
  const double m_payload = 3.0;
  const double g = 9.81;
  Eigen::VectorXd tension(1);
  tension(0) = m_payload * g;
  output->SetFromVector(tension);
}

void DualPlantStateSplitter::CalcDrone0Direction(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output) const {
  auto plant_state =
      get_input_port(0).Eval<drake::systems::BasicVector<double>>(context).value();

  // Quad 0 position: indices 0-2
  // Payload position: indices 14-16 (after quad0 at 0-6, quad1 at 7-13)
  Eigen::Vector3d p_quad0 = plant_state.segment<3>(0);
  Eigen::Vector3d p_payload = plant_state.segment<3>(14);

  Eigen::Vector3d cable_vec = p_payload - p_quad0;
  double cable_len = cable_vec.norm();

  if (cable_len < 1e-6) {
    output->SetFromVector(Eigen::Vector3d(0.0, 0.0, -1.0));
  } else {
    output->SetFromVector(cable_vec / cable_len);
  }
}

// ============ DRONE 1 ============

void DualPlantStateSplitter::CalcDrone1Position(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output) const {
  auto plant_state =
      get_input_port(0).Eval<drake::systems::BasicVector<double>>(context).value();
  // Quad 1 position at indices 7-9
  output->SetFromVector(plant_state.segment<3>(7));
}

void DualPlantStateSplitter::CalcDrone1Velocity(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output) const {
  auto plant_state =
      get_input_port(0).Eval<drake::systems::BasicVector<double>>(context).value();

  int num_bodies_pos = 2 + num_beads_;
  int v_quad1_idx = 7 * num_bodies_pos + 3;  // After quad0 velocity

  if (plant_state.size() < v_quad1_idx + 3) {
    output->SetZero();
    return;
  }
  output->SetFromVector(plant_state.segment<3>(v_quad1_idx));
}

void DualPlantStateSplitter::CalcDrone1Tension(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output) const {
  const double m_payload = 3.0;
  const double g = 9.81;
  Eigen::VectorXd tension(1);
  tension(0) = m_payload * g;
  output->SetFromVector(tension);
}

void DualPlantStateSplitter::CalcDrone1Direction(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output) const {
  auto plant_state =
      get_input_port(0).Eval<drake::systems::BasicVector<double>>(context).value();

  // Quad 1 position: indices 7-9
  // Payload position: indices 14-16
  Eigen::Vector3d p_quad1 = plant_state.segment<3>(7);
  Eigen::Vector3d p_payload = plant_state.segment<3>(14);

  Eigen::Vector3d cable_vec = p_payload - p_quad1;
  double cable_len = cable_vec.norm();

  if (cable_len < 1e-6) {
    output->SetFromVector(Eigen::Vector3d(0.0, 0.0, -1.0));
  } else {
    output->SetFromVector(cable_vec / cable_len);
  }
}

}  // namespace quad_rope_lift
