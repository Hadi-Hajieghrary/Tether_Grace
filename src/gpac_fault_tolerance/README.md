# GPAC Fault Tolerance Module

This module contains the root-owned fault-tolerance building blocks developed in Tether_Grace for GPAC-style load tracking, adaptive disturbance rejection, cable-snap fault modeling, post-fault reference redistribution, and Drake-side wiring helpers.

The code in this directory is intentionally separated from the Tether_Lift submodule. The goal is to let Tether_Grace add fault-tolerance logic, experiments, and integration seams without modifying submodule-owned controller or plant code.

## Purpose

The module provides four layers:

1. `contracts/`
   Defines small signal types used at the controller and safety-filter seams.
2. `core/`
   Implements Drake-free C++ logic for adaptive compensation, cable-fault schedules, metric computation, and fault-tolerant reference generation.
3. `adapters/`
   Converts core outputs into the specific signal semantics expected by GPAC controller and safety-filter integration points.
4. `drake/`
   Provides optional Drake `LeafSystem` wrappers and wiring helpers that expose the root-owned logic inside diagrams.

At a high level, the module supports two distinct but complementary paths:

- Residual disturbance adaptation path:
  measured state -> L1 adaptive core -> controller disturbance estimate + filtered compensation + safety bounds
- Nominal reference reshaping path:
  load reference + cable health -> desired-geometry redistribution -> per-drone trajectory references

That separation is deliberate. The adaptive residual path remains focused on disturbance rejection, while the post-fault geometry logic lives upstream on the nominal reference path.

## Design Principles

- Root-owned only: new logic belongs in Tether_Grace, not in submodules.
- Drake-free core first: the reusable algorithms in `core/` do not depend on Drake.
- Explicit seams: controller-facing and safety-facing signals are represented with small contract structs.
- Residual-only adaptive output: adaptive compensation is treated as a disturbance residual, not as a replacement for nominal cable or load compensation.
- Optional Drake wrappers: the Drake layer is built only when explicitly enabled.

## Directory Overview

### `contracts/`

`signal_contracts.h` defines the shared signal vocabulary used across the module.

Important contract types:

- `DisturbanceEstimateSignal`
  3D disturbance estimate in acceleration units, meters per second squared.
- `DisturbanceCompensationSignal`
  3D filtered compensation in acceleration units.
- `DisturbanceBoundSignal`
  Lower and upper disturbance bounds in acceleration units.
- `CableTensionSample`
  A cable-tension sample with scalar tension and body-frame cable direction.
- `CableHealthMultipliers`
  Fixed-size per-cable multiplier container for health-like scaling values.

These contracts keep the adaptive and fault-tolerance logic decoupled from any single controller implementation.

### `core/`

The `core/` directory contains the reusable Drake-free algorithms.

#### `l1_adaptive_core.*`

Implements a lightweight L1-style adaptive estimator and compensation generator.

Main responsibilities:

- Maintain predicted position and velocity states.
- Estimate a disturbance term `sigma_hat`.
- Produce filtered adaptive compensation.
- Track prediction error for diagnostics.

Key public types:

- `L1AdaptiveParams`
  Tuning for filter bandwidth, predictor pole, input gain, and sigma saturation.
- `L1AdaptiveState`
  Runtime state including predictions, estimate, compensation, and error.
- `L1AdaptiveCore`
  Update-driven adaptive core with `Reset(...)` and `Update(...)` entry points.

This class is the computational heart of the residual disturbance path.

#### `fault_metrics.*`

Computes post-fault performance summaries from time histories of error signals.

Metrics include:

- nominal RMSE
- pre-fault RMSE
- post-fault RMSE
- RMSE ratio
- peak deviation after fault
- recovery time

This logic is used to compare baseline and fault-tolerant behavior in experiments and regression-style studies.

#### `cable_snap_profile.*`

Represents a simple parameterized cable fault schedule.

Main features:

- configurable number of cables
- selected faulted cable index
- fault activation time
- post-fault scale
- optional ramp duration or instantaneous transition

The profile exposes `multiplier(...)` and `multipliers(...)`, which makes it useful both in experiments and in optional Drake signal sources.

#### `desired_geometry_redistributor.*`

Implements post-fault redistribution of nominal planar formation offsets.

Inputs:

- nominal per-vehicle offsets
- cable-health boolean flags
- current time
- fault time

Outputs:

- redistributed offsets
- redistributed offset rates

The redistributor is intended to recenter and renormalize the healthy vehicles after a cable fault so the nominal multi-vehicle geometry remains well-behaved even when one cable path is effectively lost.

#### `reference_trajectory_mapper.*`

Maps a load-level reference plus redistributed geometry into per-drone trajectory references.

Key concepts:

- `LoadReferenceSample`
  Position, velocity, and acceleration for the load.
- `DroneReferenceSample`
  Position, velocity, and acceleration for one vehicle.
- `DroneTrajectoryVector`
  Flattened 9-element `[p, v, a]` representation.

Responsibilities:

- compose per-drone reference samples from a load reference and geometry offsets
- apply cable lengths when lifting planar offsets into full 3D references
- flatten reference samples into controller-friendly 9D vectors

This is the main bridge from fault-tolerant nominal geometry logic to controller trajectory inputs.

### `adapters/`

`gpac_seam_adapter.*` packages the adaptive core into the signals expected by GPAC-related controller and safety seams.

`GpacSeamAdapter`:

- owns an `L1AdaptiveCore`
- updates it from measured position, measured velocity, and commanded acceleration
- exposes three integration products:
  - controller disturbance estimate
  - filtered compensation
  - safety disturbance bound
- also surfaces raw `sigma_hat` and prediction error for diagnostics

`GpacSeamAdapterParams` adds seam-specific policy on top of raw L1 tuning:

- `l1_params`
- `disturbance_margin_mps2`
- `disturbance_coupling`

This adapter is the recommended entry point when the consumer needs GPAC-oriented signals rather than direct access to the lower-level adaptive core.

### `drake/`

The Drake layer wraps the core and adapter logic as `LeafSystem` components and provides convenience wiring for larger diagrams.

This layer is optional and disabled by default in the root build.

#### Residual disturbance systems

##### `l1_gpac_drake_wrapper.*`

Wraps `GpacSeamAdapter` as a periodic Drake system.

Inputs:

- position
- velocity
- commanded acceleration

Outputs:

- controller disturbance estimate
- filtered compensation
- safety lower bound
- safety upper bound
- raw sigma estimate
- prediction error

This is the Drake entry point for the residual adaptive path.

##### `disturbance_bound_projector.*`

Reduces vector lower and upper disturbance bounds to a scalar bound suitable for safety-filter interfaces that expect a single disturbance magnitude.

##### `nominal_force_acceleration_probe.*`

Converts a nominal force signal into an acceleration signal using vehicle mass.

This is useful when the surrounding diagram produces force commands but the adaptive seam is defined in acceleration units.

#### Nominal reference systems

##### `fault_tolerant_reference_mapper.*`

Wraps the reference redistribution and mapping logic as a Drake system.

Inputs:

- load position
- load velocity
- load acceleration
- cable health vector

Output:

- stacked per-drone trajectory vector

Parameters:

- nominal offsets
- cable lengths
- fault time
- redistribution settings

This system is the Drake entry point for the nominal post-fault reference path.

##### `stacked_drone_trajectory_demux.*`

Splits a stacked trajectory vector into one 9D trajectory output per vehicle.

##### `cable_health_status_source.*`

Converts a `CableSnapProfile` into a Drake output port representing cable health over time.

##### `cable_multiplier_health_adapter.*`

Converts an existing upstream per-cable multiplier signal into a binary or thresholded cable-health vector.

This lets the nominal reference mapper accept either:

- a root-owned snap profile source, or
- a multiplier-like signal already present in a larger diagram

without requiring any submodule changes.

#### Wiring helpers

`gpac_single_vehicle_wiring.*` contains builder-level helpers that assemble the above systems consistently.

Single-vehicle helpers:

- `AddSingleVehicleFaultToleranceSystems(...)`
- `ConnectSingleVehicleFaultToleranceInputs(...)`
- `MakeInputSourcesFromNominalForceProbe(...)`
- `ConnectSingleVehicleFaultTolerance(...)`

These helpers simplify connecting the residual adaptive path into a controller and safety-filter pair.

Multi-vehicle helpers:

- `AddMultiVehicleReferenceSystems(...)`
- `ConnectMultiVehicleReferenceInputs(...)`
- `MakeMultiVehicleReferenceInputsFromCableSnapProfile(...)`
- `MakeMultiVehicleReferenceInputsFromCableMultiplierSource(...)`
- `ConnectMultiVehicleControllerTrajectories(...)`

These helpers simplify building the nominal fault-tolerant reference path from load-level references to per-controller trajectory ports.

## Typical Signal Flow

### 1. Residual adaptive compensation path

This path estimates unmodeled disturbance and publishes controller- and safety-facing outputs.

Flow:

1. Measured position and velocity arrive from the plant or estimator.
2. Commanded nominal acceleration arrives from the surrounding controller path.
3. `L1AdaptiveCore` updates its prediction and disturbance estimate.
4. `GpacSeamAdapter` converts the result into disturbance estimate, filtered compensation, and bounds.
5. Optional Drake wrappers expose those values to the controller and safety-filter subsystems.

Use this path when the nominal controller already exists and you want to add fault-tolerant residual compensation around it.

### 2. Post-fault nominal reference path

This path reshapes the target references after a cable fault.

Flow:

1. A load-level reference is provided as position, velocity, and acceleration.
2. Cable health is provided either by `CableSnapProfile` or by an upstream multiplier signal through `CableMultiplierHealthAdapter`.
3. `DesiredGeometryRedistributor` computes healthy-vehicle offsets and offset rates.
4. `ReferenceTrajectoryMapper` composes per-drone references.
5. Optional Drake systems emit stacked trajectories and demultiplex them to one controller input per vehicle.

Use this path when fault tolerance requires changing the nominal multi-vehicle geometry rather than only adding disturbance rejection.

## Build and Dependency Model

The module is split into two CMake targets:

- `gpac_fault_tolerance_core`
  Always built in the root project. Contains the Drake-free core and adapters.
- `gpac_fault_tolerance_drake`
  Built only when `ENABLE_GPAC_DRAKE_WRAPPERS=ON`.

Default behavior:

- C++17 is required.
- core tests are enabled in the root build.
- Drake wrappers are off by default so the root project remains buildable without Drake headers and related diagram dependencies.

The optional Drake target includes headers from:

- the local `src/` tree
- `Tether_Lift/Research/cpp/include`

That dependency is an integration boundary only. The root-owned fault-tolerance logic still lives in this module.

## Tests

The root project currently validates the Drake-free layer with unit tests for:

- `L1AdaptiveCore`
- fault metrics
- cable snap profile
- desired geometry redistribution
- reference trajectory mapping
- GPAC seam adapter

Those tests protect the algorithmic and seam-conversion behavior independently of any Drake diagram assembly.

## When To Use Which Class

- Use `L1AdaptiveCore` when you only need the adaptive estimator and compensation logic.
- Use `GpacSeamAdapter` when you need controller- and safety-filter-ready disturbance signals.
- Use `CableSnapProfile` when you need a deterministic cable-fault schedule.
- Use `DesiredGeometryRedistributor` when healthy vehicles should absorb formation changes after a fault.
- Use `ReferenceTrajectoryMapper` when you need per-drone `[p, v, a]` references from a load-level reference.
- Use `L1GpacDrakeWrapper` when integrating the adaptive seam into a Drake diagram.
- Use `FaultTolerantReferenceMapper` plus `StackedDroneTrajectoryDemux` when integrating post-fault nominal references into a Drake diagram.
- Use `CableHealthStatusSource` or `CableMultiplierHealthAdapter` depending on whether cable state originates from a root-owned profile or an existing multiplier signal.

## Limitations and Assumptions

- The adaptive and disturbance-bound contracts are acceleration-valued.
- The cable-health and multiplier utilities model per-cable status but do not perform fault detection on their own.
- The nominal reference reshaping logic assumes that redistributed geometry should be handled upstream of the controller residual path.
- The Drake wrappers are integration helpers, not the source of truth for the underlying algorithms.

## Summary

`src/gpac_fault_tolerance` is the root-owned implementation seam for adding fault tolerance to GPAC-related control stacks in Tether_Grace. It combines:

- reusable adaptive estimation and compensation
- cable-fault scheduling
- post-fault reference redistribution
- per-drone trajectory mapping
- GPAC-facing seam adapters
- optional Drake wrappers and wiring helpers

The result is a modular path for studying and integrating cable-snap resilience while preserving the repository boundary that keeps Tether_Lift read-only.