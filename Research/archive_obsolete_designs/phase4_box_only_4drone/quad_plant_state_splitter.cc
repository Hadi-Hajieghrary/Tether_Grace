#include "quad_plant_state_splitter.h"

#include <cmath>
#include <iostream>

namespace quad_rope_lift {

QuadPlantStateSplitter::QuadPlantStateSplitter(int num_beads, double rope_length)
    : num_beads_(num_beads), rope_length_(rope_length) {
  // Plant structure:
  // 4 quadcopters + 1 payload + 4*num_beads rope beads = (5 + 4*num_beads) bodies
  int num_bodies = 5 + 4 * num_beads;
  int state_size = num_bodies * 13;  // 13D per body (pos+quat+vel+ang_vel reduced)

  DeclareVectorInputPort("plant_state", state_size);

  // Output 16 ports (4 drones × 4 measurements each)
  for (int i = 0; i < 4; ++i) {
    DeclareVectorOutputPort(
        "drone_" + std::to_string(i) + "_position", 3,
        [this, i](const drake::systems::Context<double>& context,
                  drake::systems::BasicVector<double>* output) {
          this->CalcDronePosition(i, context, output);
        });

    DeclareVectorOutputPort(
        "drone_" + std::to_string(i) + "_velocity", 3,
        [this, i](const drake::systems::Context<double>& context,
                  drake::systems::BasicVector<double>* output) {
          this->CalcDroneVelocity(i, context, output);
        });

    DeclareVectorOutputPort(
        "drone_" + std::to_string(i) + "_tension", 1,
        [this, i](const drake::systems::Context<double>& context,
                  drake::systems::BasicVector<double>* output) {
          this->CalcCableTension(i, context, output);
        });

    DeclareVectorOutputPort(
        "drone_" + std::to_string(i) + "_direction", 3,
        [this, i](const drake::systems::Context<double>& context,
                  drake::systems::BasicVector<double>* output) {
          this->CalcCableDirection(i, context, output);
        });
  }
}

void QuadPlantStateSplitter::CalcDronePosition(
    int drone_idx,
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output) const {
  auto state = get_input_port(0).Eval<drake::systems::BasicVector<double>>(context).value();

  // Drake layout per body: q = [quat(4), pos(3)] => position starts at offset drone_idx*7 + 4
  int offset = drone_idx * 7 + 4;
  Eigen::Vector3d position = state.segment<3>(offset);

  output->SetFromVector(position);
}

void QuadPlantStateSplitter::CalcDroneVelocity(
    int drone_idx,
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output) const {
  auto state = get_input_port(0).Eval<drake::systems::BasicVector<double>>(context).value();

  // Total bodies: 5 + 4*num_beads
  int num_bodies = 5 + 4 * num_beads_;
  // Velocity starts at offset: num_bodies * 7 (7 generalized coordinates per body)
  // Per-body velocity layout: [angular(3), linear(3)] => linear at +3
  int velocity_offset = num_bodies * 7;
  int offset = velocity_offset + drone_idx * 6 + 3;

  Eigen::Vector3d velocity = state.segment<3>(offset);
  output->SetFromVector(velocity);
}

void QuadPlantStateSplitter::CalcCableTension(
    int drone_idx,
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output) const {
  auto state = get_input_port(0).Eval<drake::systems::BasicVector<double>>(context).value();

  // Quad position at offset drone_idx*7+4, payload at offset 4*7+4=32 (5th body position)
  Eigen::Vector3d quad_pos = state.segment<3>(drone_idx * 7 + 4);
  Eigen::Vector3d payload_pos = state.segment<3>(4 * 7 + 4);

  // Cable vector
  Eigen::Vector3d cable_vec = payload_pos - quad_pos;
  double distance = cable_vec.norm();

  // Rope tension: if stretched, T = k * (d - L)
  double stiffness = 200.0;  // N/m
  double tension = std::max(0.0, stiffness * (distance - rope_length_));

  output->SetFromVector(Eigen::Matrix<double, 1, 1>(tension));
}

void QuadPlantStateSplitter::CalcCableDirection(
    int drone_idx,
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output) const {
  auto state = get_input_port(0).Eval<drake::systems::BasicVector<double>>(context).value();

  // Quad position at offset drone_idx*7+4, payload at offset 4*7+4=32 (5th body position)
  Eigen::Vector3d quad_pos = state.segment<3>(drone_idx * 7 + 4);
  Eigen::Vector3d payload_pos = state.segment<3>(4 * 7 + 4);

  // Cable direction (unit vector from quad to payload)
  Eigen::Vector3d cable_vec = payload_pos - quad_pos;
  double distance = cable_vec.norm();

  Eigen::Vector3d direction = Eigen::Vector3d::Zero();
  if (distance > 1e-6) {
    direction = cable_vec / distance;
  }

  output->SetFromVector(direction);
}

}  // namespace quad_rope_lift
