#pragma once

#include <vector>

#include <drake/systems/framework/leaf_system.h>

#include "gpac_fault_tolerance/core/desired_geometry_redistributor.h"

namespace tether_grace::gpac_fault_tolerance::drake {

struct FaultTolerantReferenceMapperParams {
  std::vector<Vector3> nominal_offsets;
  std::vector<double> cable_lengths_m;
  /// When < 1e10, used as a manual override for all cables.
  /// When left at default (1e10), per-cable fault times are auto-detected
  /// from runtime cable-health transitions and stored in discrete state.
  double fault_time_seconds{1e10};
  DesiredGeometryRedistributorParams redistribution{};
};

class FaultTolerantReferenceMapper final
    : public ::drake::systems::LeafSystem<double> {
public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FaultTolerantReferenceMapper);

  explicit FaultTolerantReferenceMapper(
      const FaultTolerantReferenceMapperParams &params);

  const ::drake::systems::InputPort<double> &
  get_load_position_input_port() const;
  const ::drake::systems::InputPort<double> &
  get_load_velocity_input_port() const;
  const ::drake::systems::InputPort<double> &
  get_load_acceleration_input_port() const;
  const ::drake::systems::InputPort<double> &
  get_cable_health_input_port() const;

  const ::drake::systems::OutputPort<double> &
  get_stacked_drone_trajectories_output_port() const;

  int num_vehicles() const;
  const FaultTolerantReferenceMapperParams &params() const;

private:
  void CalcStackedDroneTrajectories(
      const ::drake::systems::Context<double> &context,
      ::drake::systems::BasicVector<double> *output) const;

  ::drake::systems::EventStatus
  UpdateFaultTimes(const ::drake::systems::Context<double> &context,
                   ::drake::systems::DiscreteValues<double> *state) const;

  FaultTolerantReferenceMapperParams params_{};
  DesiredGeometryRedistributor redistributor_;

  int load_position_port_{};
  int load_velocity_port_{};
  int load_acceleration_port_{};
  int cable_health_port_{};
  int stacked_drone_trajectories_port_{};
  int fault_times_state_index_{};
};

} // namespace tether_grace::gpac_fault_tolerance::drake