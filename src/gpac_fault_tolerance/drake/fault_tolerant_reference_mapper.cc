#include "gpac_fault_tolerance/drake/fault_tolerant_reference_mapper.h"

#include <drake/common/drake_throw.h>
#include <drake/systems/framework/basic_vector.h>

#include "gpac_fault_tolerance/core/reference_trajectory_mapper.h"

namespace tether_grace::gpac_fault_tolerance::drake {
namespace {

constexpr double kNoFaultSentinel = 1e10;

Vector3 ReadVector3(const ::drake::systems::Context<double> &context,
                    const ::drake::systems::InputPort<double> &port) {
  const auto &vector = port.Eval(context);
  return {vector[0], vector[1], vector[2]};
}

std::vector<bool>
ReadCableHealth(const ::drake::systems::Context<double> &context,
                const ::drake::systems::InputPort<double> &port, int count) {
  const auto &values = port.Eval(context);
  std::vector<bool> healthy(static_cast<std::size_t>(count), true);
  for (int index = 0; index < count; ++index) {
    healthy[static_cast<std::size_t>(index)] = values[index] >= 0.5;
  }
  return healthy;
}

} // namespace

FaultTolerantReferenceMapper::FaultTolerantReferenceMapper(
    const FaultTolerantReferenceMapperParams &params)
    : params_(params), redistributor_(params.redistribution) {
  DRAKE_THROW_UNLESS(!params_.nominal_offsets.empty());
  DRAKE_THROW_UNLESS(params_.nominal_offsets.size() ==
                     params_.cable_lengths_m.size());

  const int count = static_cast<int>(params_.nominal_offsets.size());
  load_position_port_ =
      this->DeclareVectorInputPort("load_position", 3).get_index();
  load_velocity_port_ =
      this->DeclareVectorInputPort("load_velocity", 3).get_index();
  load_acceleration_port_ =
      this->DeclareVectorInputPort("load_acceleration", 3).get_index();
  cable_health_port_ =
      this->DeclareVectorInputPort("cable_health", count).get_index();
  stacked_drone_trajectories_port_ =
      this->DeclareVectorOutputPort(
              "stacked_drone_trajectories", 9 * count,
              &FaultTolerantReferenceMapper::CalcStackedDroneTrajectories)
          .get_index();

  // Per-cable fault-time tracking: N doubles initialised to sentinel (no
  // fault).  Only used when params_.fault_time_seconds == kNoFaultSentinel.
  fault_times_state_index_ = this->DeclareDiscreteState(
      Eigen::VectorXd::Constant(count, kNoFaultSentinel));

  this->DeclarePerStepDiscreteUpdateEvent(
      &FaultTolerantReferenceMapper::UpdateFaultTimes);
}

const ::drake::systems::InputPort<double> &
FaultTolerantReferenceMapper::get_load_position_input_port() const {
  return this->get_input_port(load_position_port_);
}

const ::drake::systems::InputPort<double> &
FaultTolerantReferenceMapper::get_load_velocity_input_port() const {
  return this->get_input_port(load_velocity_port_);
}

const ::drake::systems::InputPort<double> &
FaultTolerantReferenceMapper::get_load_acceleration_input_port() const {
  return this->get_input_port(load_acceleration_port_);
}

const ::drake::systems::InputPort<double> &
FaultTolerantReferenceMapper::get_cable_health_input_port() const {
  return this->get_input_port(cable_health_port_);
}

const ::drake::systems::OutputPort<double> &
FaultTolerantReferenceMapper::get_stacked_drone_trajectories_output_port()
    const {
  return this->get_output_port(stacked_drone_trajectories_port_);
}

int FaultTolerantReferenceMapper::num_vehicles() const {
  return static_cast<int>(params_.nominal_offsets.size());
}

const FaultTolerantReferenceMapperParams &
FaultTolerantReferenceMapper::params() const {
  return params_;
}

::drake::systems::EventStatus FaultTolerantReferenceMapper::UpdateFaultTimes(
    const ::drake::systems::Context<double> &context,
    ::drake::systems::DiscreteValues<double> *state) const {
  const std::vector<bool> cable_healthy =
      ReadCableHealth(context, get_cable_health_input_port(), num_vehicles());
  const double time = context.get_time();
  auto &fault_times = state->get_mutable_vector(fault_times_state_index_);
  for (int i = 0; i < num_vehicles(); ++i) {
    if (!cable_healthy[static_cast<std::size_t>(i)] &&
        fault_times.GetAtIndex(i) >= kNoFaultSentinel) {
      // First detection of this cable's fault.
      fault_times.SetAtIndex(i, time);
    } else if (cable_healthy[static_cast<std::size_t>(i)]) {
      // Cable recovered — reset so a future fault is re-detected.
      fault_times.SetAtIndex(i, kNoFaultSentinel);
    }
  }
  return ::drake::systems::EventStatus::Succeeded();
}

void FaultTolerantReferenceMapper::CalcStackedDroneTrajectories(
    const ::drake::systems::Context<double> &context,
    ::drake::systems::BasicVector<double> *output) const {
  LoadReferenceSample load_reference;
  load_reference.position =
      ReadVector3(context, get_load_position_input_port());
  load_reference.velocity =
      ReadVector3(context, get_load_velocity_input_port());
  load_reference.acceleration =
      ReadVector3(context, get_load_acceleration_input_port());

  const std::vector<bool> cable_healthy =
      ReadCableHealth(context, get_cable_health_input_port(), num_vehicles());

  // Determine effective fault time: manual override or auto-detected.
  double effective_fault_time = params_.fault_time_seconds;
  if (params_.fault_time_seconds >= kNoFaultSentinel) {
    const auto &fault_times =
        context.get_discrete_state(fault_times_state_index_).value();
    effective_fault_time = kNoFaultSentinel;
    for (int i = 0; i < num_vehicles(); ++i) {
      if (!cable_healthy[static_cast<std::size_t>(i)] &&
          fault_times[i] < effective_fault_time) {
        effective_fault_time = fault_times[i];
      }
    }
  }

  const auto redistributed =
      redistributor_.Evaluate(params_.nominal_offsets, cable_healthy,
                              context.get_time(), effective_fault_time);
  const auto drone_references = ComposeDroneReferenceSamples(
      load_reference, redistributed, params_.cable_lengths_m);
  const auto flattened = FlattenDroneReferenceSamples(drone_references);

  auto values = output->get_mutable_value();
  values.setZero();
  for (int vehicle_index = 0; vehicle_index < num_vehicles(); ++vehicle_index) {
    for (int element = 0; element < 9; ++element) {
      values[9 * vehicle_index + element] =
          flattened[static_cast<std::size_t>(vehicle_index)]
                   [static_cast<std::size_t>(element)];
    }
  }
}

} // namespace tether_grace::gpac_fault_tolerance::drake