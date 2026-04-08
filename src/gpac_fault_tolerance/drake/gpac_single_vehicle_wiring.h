#pragma once

#include <drake/systems/framework/diagram_builder.h>

#include "gpac_cbf_safety_filter.h"
#include "gpac_fault_tolerance/adapters/gpac_seam_adapter.h"
#include "gpac_fault_tolerance/drake/cable_health_status_source.h"
#include "gpac_fault_tolerance/drake/cable_multiplier_health_adapter.h"
#include "gpac_fault_tolerance/drake/disturbance_bound_projector.h"
#include "gpac_fault_tolerance/drake/fault_tolerant_reference_mapper.h"
#include "gpac_fault_tolerance/drake/l1_gpac_drake_wrapper.h"
#include "gpac_fault_tolerance/drake/nominal_force_acceleration_probe.h"
#include "gpac_fault_tolerance/drake/stacked_drone_trajectory_demux.h"
#include "gpac_load_tracking_controller.h"

namespace tether_grace::gpac_fault_tolerance::drake {

struct SingleVehicleFaultToleranceInputSources {
  const ::drake::systems::OutputPort<double> *position_source{};
  const ::drake::systems::OutputPort<double> *velocity_source{};
  const ::drake::systems::OutputPort<double> *commanded_acceleration_source{};
};

struct SingleVehicleFaultToleranceBundle {
  L1GpacDrakeWrapper *l1_wrapper{};
  DisturbanceBoundProjector *disturbance_bound_projector{};
};

struct MultiVehicleReferenceBundle {
  FaultTolerantReferenceMapper *reference_mapper{};
  StackedDroneTrajectoryDemux *trajectory_demux{};
};

struct MultiVehicleReferenceInputSources {
  const ::drake::systems::OutputPort<double> *load_position_source{};
  const ::drake::systems::OutputPort<double> *load_velocity_source{};
  const ::drake::systems::OutputPort<double> *load_acceleration_source{};
  const ::drake::systems::OutputPort<double> *cable_health_source{};
};

SingleVehicleFaultToleranceBundle AddSingleVehicleFaultToleranceSystems(
    ::drake::systems::DiagramBuilder<double> *builder,
    const adapters::GpacSeamAdapterParams &adapter_params =
        adapters::GpacSeamAdapterParams(),
    double update_period_seconds = 0.005);

void ConnectSingleVehicleFaultToleranceInputs(
    ::drake::systems::DiagramBuilder<double> *builder,
    const SingleVehicleFaultToleranceBundle &bundle,
    const ::drake::systems::OutputPort<double> &position_source,
    const ::drake::systems::OutputPort<double> &velocity_source,
    const ::drake::systems::OutputPort<double> &commanded_acceleration_source);

void ConnectSingleVehicleFaultToleranceInputs(
    ::drake::systems::DiagramBuilder<double> *builder,
    const SingleVehicleFaultToleranceBundle &bundle,
    const SingleVehicleFaultToleranceInputSources &input_sources);

SingleVehicleFaultToleranceInputSources MakeInputSourcesFromNominalForceProbe(
    ::drake::systems::DiagramBuilder<double> *builder,
    const ::drake::systems::OutputPort<double> &position_source,
    const ::drake::systems::OutputPort<double> &velocity_source,
    const ::drake::systems::OutputPort<double> &nominal_force_source,
    double vehicle_mass_kg);

void ConnectSingleVehicleFaultTolerance(
    ::drake::systems::DiagramBuilder<double> *builder,
    const SingleVehicleFaultToleranceBundle &bundle,
    tether_lift::GPACLoadTrackingController *controller,
    quad_rope_lift::gpac::GPACCbfSafetyFilter *safety_filter);

MultiVehicleReferenceBundle AddMultiVehicleReferenceSystems(
    ::drake::systems::DiagramBuilder<double> *builder,
    const FaultTolerantReferenceMapperParams &params);

void ConnectMultiVehicleReferenceInputs(
    ::drake::systems::DiagramBuilder<double> *builder,
    const MultiVehicleReferenceBundle &bundle,
    const ::drake::systems::OutputPort<double> &load_position_source,
    const ::drake::systems::OutputPort<double> &load_velocity_source,
    const ::drake::systems::OutputPort<double> &load_acceleration_source,
    const ::drake::systems::OutputPort<double> &cable_health_source);

void ConnectMultiVehicleReferenceInputs(
    ::drake::systems::DiagramBuilder<double> *builder,
    const MultiVehicleReferenceBundle &bundle,
    const MultiVehicleReferenceInputSources &input_sources);

MultiVehicleReferenceInputSources
MakeMultiVehicleReferenceInputsFromCableSnapProfile(
    ::drake::systems::DiagramBuilder<double> *builder,
    const ::drake::systems::OutputPort<double> &load_position_source,
    const ::drake::systems::OutputPort<double> &load_velocity_source,
    const ::drake::systems::OutputPort<double> &load_acceleration_source,
    const CableSnapProfileParams &cable_health_params);

MultiVehicleReferenceInputSources
MakeMultiVehicleReferenceInputsFromCableMultiplierSource(
    ::drake::systems::DiagramBuilder<double> *builder,
    const ::drake::systems::OutputPort<double> &load_position_source,
    const ::drake::systems::OutputPort<double> &load_velocity_source,
    const ::drake::systems::OutputPort<double> &load_acceleration_source,
    const ::drake::systems::OutputPort<double> &cable_multiplier_source,
    int num_cables, double healthy_threshold = 0.5);

void ConnectMultiVehicleControllerTrajectories(
    ::drake::systems::DiagramBuilder<double> *builder,
    const MultiVehicleReferenceBundle &bundle,
    const std::vector<tether_lift::GPACLoadTrackingController *> &controllers);

} // namespace tether_grace::gpac_fault_tolerance::drake
