#pragma once

#include <drake/systems/framework/leaf_system.h>

namespace tether_grace::gpac_fault_tolerance::drake {

class StackedDroneTrajectoryDemux final
    : public ::drake::systems::LeafSystem<double> {
public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(StackedDroneTrajectoryDemux);

  explicit StackedDroneTrajectoryDemux(int num_vehicles);

  const ::drake::systems::InputPort<double> &
  get_stacked_trajectories_input_port() const;
  const ::drake::systems::OutputPort<double> &
  get_vehicle_trajectory_output_port(int vehicle_index) const;

  int num_vehicles() const;

private:
  void CalcVehicleTrajectory(const ::drake::systems::Context<double> &context,
                             ::drake::systems::BasicVector<double> *output,
                             int vehicle_index) const;

  int num_vehicles_{};
  int stacked_trajectories_port_{};
};

} // namespace tether_grace::gpac_fault_tolerance::drake