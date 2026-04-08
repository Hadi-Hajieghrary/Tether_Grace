#include "gpac_fault_tolerance/drake/stacked_drone_trajectory_demux.h"

#include <drake/common/drake_throw.h>
#include <drake/systems/framework/basic_vector.h>

#include <string>

namespace tether_grace::gpac_fault_tolerance::drake {

StackedDroneTrajectoryDemux::StackedDroneTrajectoryDemux(int num_vehicles)
    : num_vehicles_(num_vehicles) {
  DRAKE_THROW_UNLESS(num_vehicles_ > 0);

  stacked_trajectories_port_ =
      this->DeclareVectorInputPort("stacked_drone_trajectories",
                                   9 * num_vehicles_)
          .get_index();

  for (int vehicle_index = 0; vehicle_index < num_vehicles_; ++vehicle_index) {
    this->DeclareVectorOutputPort(
        "vehicle_" + std::to_string(vehicle_index) + "_trajectory", 9,
        [this, vehicle_index](const ::drake::systems::Context<double> &context,
                              ::drake::systems::BasicVector<double> *output) {
          this->CalcVehicleTrajectory(context, output, vehicle_index);
        });
  }
}

const ::drake::systems::InputPort<double> &
StackedDroneTrajectoryDemux::get_stacked_trajectories_input_port() const {
  return this->get_input_port(stacked_trajectories_port_);
}

const ::drake::systems::OutputPort<double> &
StackedDroneTrajectoryDemux::get_vehicle_trajectory_output_port(
    int vehicle_index) const {
  DRAKE_THROW_UNLESS(vehicle_index >= 0);
  DRAKE_THROW_UNLESS(vehicle_index < num_vehicles_);
  return this->get_output_port(vehicle_index);
}

int StackedDroneTrajectoryDemux::num_vehicles() const { return num_vehicles_; }

void StackedDroneTrajectoryDemux::CalcVehicleTrajectory(
    const ::drake::systems::Context<double> &context,
    ::drake::systems::BasicVector<double> *output, int vehicle_index) const {
  const auto &stacked = get_stacked_trajectories_input_port().Eval(context);
  for (int element = 0; element < 9; ++element) {
    output->SetAtIndex(element, stacked[9 * vehicle_index + element]);
  }
}

} // namespace tether_grace::gpac_fault_tolerance::drake