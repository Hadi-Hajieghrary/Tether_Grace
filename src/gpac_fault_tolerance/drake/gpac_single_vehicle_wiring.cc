#include "gpac_fault_tolerance/drake/gpac_single_vehicle_wiring.h"

namespace tether_grace::gpac_fault_tolerance::drake {

SingleVehicleFaultToleranceBundle AddSingleVehicleFaultToleranceSystems(
    ::drake::systems::DiagramBuilder<double> *builder,
    const adapters::GpacSeamAdapterParams &adapter_params,
    double update_period_seconds) {
  SingleVehicleFaultToleranceBundle bundle;
  bundle.l1_wrapper = builder->AddSystem<L1GpacDrakeWrapper>(
      adapter_params, update_period_seconds);
  bundle.disturbance_bound_projector =
      builder->AddSystem<DisturbanceBoundProjector>();
  return bundle;
}

void ConnectSingleVehicleFaultToleranceInputs(
    ::drake::systems::DiagramBuilder<double> *builder,
    const SingleVehicleFaultToleranceBundle &bundle,
    const ::drake::systems::OutputPort<double> &position_source,
    const ::drake::systems::OutputPort<double> &velocity_source,
    const ::drake::systems::OutputPort<double> &commanded_acceleration_source) {
  builder->Connect(position_source,
                   bundle.l1_wrapper->get_position_input_port());
  builder->Connect(velocity_source,
                   bundle.l1_wrapper->get_velocity_input_port());
  builder->Connect(commanded_acceleration_source,
                   bundle.l1_wrapper->get_commanded_acceleration_input_port());
}

void ConnectSingleVehicleFaultToleranceInputs(
    ::drake::systems::DiagramBuilder<double> *builder,
    const SingleVehicleFaultToleranceBundle &bundle,
    const SingleVehicleFaultToleranceInputSources &input_sources) {
  DRAKE_DEMAND(input_sources.position_source != nullptr);
  DRAKE_DEMAND(input_sources.velocity_source != nullptr);
  DRAKE_DEMAND(input_sources.commanded_acceleration_source != nullptr);
  ConnectSingleVehicleFaultToleranceInputs(
      builder, bundle, *input_sources.position_source,
      *input_sources.velocity_source,
      *input_sources.commanded_acceleration_source);
}

SingleVehicleFaultToleranceInputSources MakeInputSourcesFromNominalForceProbe(
    ::drake::systems::DiagramBuilder<double> *builder,
    const ::drake::systems::OutputPort<double> &position_source,
    const ::drake::systems::OutputPort<double> &velocity_source,
    const ::drake::systems::OutputPort<double> &nominal_force_source,
    double vehicle_mass_kg) {
  auto *probe =
      builder->AddSystem<NominalForceAccelerationProbe>(vehicle_mass_kg);
  builder->Connect(nominal_force_source, probe->get_force_input_port());

  SingleVehicleFaultToleranceInputSources input_sources;
  input_sources.position_source = &position_source;
  input_sources.velocity_source = &velocity_source;
  input_sources.commanded_acceleration_source =
      &probe->get_acceleration_output_port();
  return input_sources;
}

void ConnectSingleVehicleFaultTolerance(
    ::drake::systems::DiagramBuilder<double> *builder,
    const SingleVehicleFaultToleranceBundle &bundle,
    tether_lift::GPACLoadTrackingController *controller,
    quad_rope_lift::gpac::GPACCbfSafetyFilter *safety_filter) {
  builder->Connect(bundle.l1_wrapper->get_controller_disturbance_output_port(),
                   controller->get_disturbance_input());

  builder->Connect(
      bundle.l1_wrapper->get_safety_lower_bound_output_port(),
      bundle.disturbance_bound_projector->get_lower_bound_input_port());
  builder->Connect(
      bundle.l1_wrapper->get_safety_upper_bound_output_port(),
      bundle.disturbance_bound_projector->get_upper_bound_input_port());
  builder->Connect(
      bundle.disturbance_bound_projector->get_scalar_bound_output_port(),
      safety_filter->get_disturbance_bound_input());
}

MultiVehicleReferenceBundle AddMultiVehicleReferenceSystems(
    ::drake::systems::DiagramBuilder<double> *builder,
    const FaultTolerantReferenceMapperParams &params) {
  MultiVehicleReferenceBundle bundle;
  bundle.reference_mapper =
      builder->AddSystem<FaultTolerantReferenceMapper>(params);
  bundle.trajectory_demux = builder->AddSystem<StackedDroneTrajectoryDemux>(
      bundle.reference_mapper->num_vehicles());
  builder->Connect(
      bundle.reference_mapper->get_stacked_drone_trajectories_output_port(),
      bundle.trajectory_demux->get_stacked_trajectories_input_port());
  return bundle;
}

void ConnectMultiVehicleReferenceInputs(
    ::drake::systems::DiagramBuilder<double> *builder,
    const MultiVehicleReferenceBundle &bundle,
    const ::drake::systems::OutputPort<double> &load_position_source,
    const ::drake::systems::OutputPort<double> &load_velocity_source,
    const ::drake::systems::OutputPort<double> &load_acceleration_source,
    const ::drake::systems::OutputPort<double> &cable_health_source) {
  builder->Connect(load_position_source,
                   bundle.reference_mapper->get_load_position_input_port());
  builder->Connect(load_velocity_source,
                   bundle.reference_mapper->get_load_velocity_input_port());
  builder->Connect(load_acceleration_source,
                   bundle.reference_mapper->get_load_acceleration_input_port());
  builder->Connect(cable_health_source,
                   bundle.reference_mapper->get_cable_health_input_port());
}

void ConnectMultiVehicleReferenceInputs(
    ::drake::systems::DiagramBuilder<double> *builder,
    const MultiVehicleReferenceBundle &bundle,
    const MultiVehicleReferenceInputSources &input_sources) {
  DRAKE_DEMAND(input_sources.load_position_source != nullptr);
  DRAKE_DEMAND(input_sources.load_velocity_source != nullptr);
  DRAKE_DEMAND(input_sources.load_acceleration_source != nullptr);
  DRAKE_DEMAND(input_sources.cable_health_source != nullptr);
  ConnectMultiVehicleReferenceInputs(builder, bundle,
                                     *input_sources.load_position_source,
                                     *input_sources.load_velocity_source,
                                     *input_sources.load_acceleration_source,
                                     *input_sources.cable_health_source);
}

MultiVehicleReferenceInputSources
MakeMultiVehicleReferenceInputsFromCableSnapProfile(
    ::drake::systems::DiagramBuilder<double> *builder,
    const ::drake::systems::OutputPort<double> &load_position_source,
    const ::drake::systems::OutputPort<double> &load_velocity_source,
    const ::drake::systems::OutputPort<double> &load_acceleration_source,
    const CableSnapProfileParams &cable_health_params) {
  auto *cable_health_source =
      builder->AddSystem<CableHealthStatusSource>(cable_health_params);

  MultiVehicleReferenceInputSources input_sources;
  input_sources.load_position_source = &load_position_source;
  input_sources.load_velocity_source = &load_velocity_source;
  input_sources.load_acceleration_source = &load_acceleration_source;
  input_sources.cable_health_source =
      &cable_health_source->get_cable_health_output_port();
  return input_sources;
}

MultiVehicleReferenceInputSources
MakeMultiVehicleReferenceInputsFromCableMultiplierSource(
    ::drake::systems::DiagramBuilder<double> *builder,
    const ::drake::systems::OutputPort<double> &load_position_source,
    const ::drake::systems::OutputPort<double> &load_velocity_source,
    const ::drake::systems::OutputPort<double> &load_acceleration_source,
    const ::drake::systems::OutputPort<double> &cable_multiplier_source,
    int num_cables, double healthy_threshold) {
  auto *cable_health_adapter = builder->AddSystem<CableMultiplierHealthAdapter>(
      num_cables, healthy_threshold);
  builder->Connect(cable_multiplier_source,
                   cable_health_adapter->get_multiplier_input_port());

  MultiVehicleReferenceInputSources input_sources;
  input_sources.load_position_source = &load_position_source;
  input_sources.load_velocity_source = &load_velocity_source;
  input_sources.load_acceleration_source = &load_acceleration_source;
  input_sources.cable_health_source =
      &cable_health_adapter->get_cable_health_output_port();
  return input_sources;
}

void ConnectMultiVehicleControllerTrajectories(
    ::drake::systems::DiagramBuilder<double> *builder,
    const MultiVehicleReferenceBundle &bundle,
    const std::vector<tether_lift::GPACLoadTrackingController *> &controllers) {
  DRAKE_DEMAND(bundle.reference_mapper != nullptr);
  DRAKE_DEMAND(bundle.trajectory_demux != nullptr);
  DRAKE_DEMAND(static_cast<int>(controllers.size()) ==
               bundle.trajectory_demux->num_vehicles());
  for (int index = 0; index < bundle.trajectory_demux->num_vehicles();
       ++index) {
    DRAKE_DEMAND(controllers[static_cast<std::size_t>(index)] != nullptr);
    builder->Connect(
        bundle.trajectory_demux->get_vehicle_trajectory_output_port(index),
        controllers[static_cast<std::size_t>(index)]
            ->get_drone_trajectory_input());
  }
}

} // namespace tether_grace::gpac_fault_tolerance::drake
