# Fault-Tolerance Experiments Module

This directory contains the root-owned experiment harness used to study cable-snap fault tolerance, compare GPAC-style concurrent learning against L1 adaptive compensation, and evaluate topology-invariance tradeoffs before integrating changes into the C++ and Drake layers.

The experiments here are intentionally lightweight and Python-based. They are not meant to replace the full simulation stack in the submodules. Instead, they provide a fast reduced-order environment for:

- validating control ideas
- comparing controller structures under matched disturbances and fault cases
- sweeping adaptive-control parameters
- ranking candidate tunings under explicit RMSE, spread, and peak-deviation objectives
- generating repeatable artifacts for reports and design decisions

## What This Module Does

At a high level, `experiments/` provides a reduced-order payload transport model with three main uses:

1. Single-scenario comparison
   Run one CL-vs-L1 experiment with a specified cable snap, seed, and tuning to inspect time traces, sigma estimates, and aggregate fault metrics.
2. Multi-scenario validation
   Run the same controller configuration across many random seeds and many faulted cables to estimate average post-fault behavior and topology sensitivity.
3. Parameter search and reporting
   Sweep adaptive parameters, score candidates against CL, filter by constraints, and generate markdown summaries from the resulting JSON artifacts.

4. Unknowns-status regeneration
  Rebuild the current U1-U6 evidence pack from one root-owned command, including reduced-order reference runs, full-Drake comparison analysis, topology-condition checks, CL history-corruption analysis, degradation pilots, and the consolidated markdown report.

The experiments are entirely root-owned and live in Tether_Grace. They do not modify any submodule code.

## Directory Layout

This module currently consists of one Python package-like folder:

- `python/`
  Contains the reduced-order model, metric helpers, sweep tools, validation runners, and report-generation scripts.

## Core Experiment Model

### `python/topology_invariance_root.py`

This is the central experiment model and the most important file in the directory.

It implements a reduced-order multi-agent suspended-load simulation with:

- a load-level reference trajectory generator
- multiple quadrotor agents connected to the load by spring-damper cables
- stochastic wind disturbances
- cable-snap fault injection
- GPAC-CL-style baseline logic
- L1 adaptive compensation logic
- optional shared payload-level L1 channel
- local post-fault adaptive-force shaping
- desired-geometry redistribution after fault

Conceptually, this file answers the question:

"Given a specific fault time, faulted cable, seed, wind level, and controller configuration, how does CL compare to L1 on payload tracking, peak deviation, and cross-cable robustness?"

#### Main components inside the model

- `Params`
  Collects model constants such as masses, cable lengths, gains, damping, and time steps.
- `trajectory(...)`
  Generates the desired load path, velocity, and acceleration.
- `cable_force(...)`
  Computes the spring-damper cable force and current tension.
- `Wind`
  Generates colored-noise-like disturbance forces.
- `ConcurrentLearning`
  Implements the reduced-order CL-style baseline update logic.
- `ExtendedStateObserver`
  Provides the disturbance-estimation support used by the CL path.
- `L1AdaptiveController`
  Implements the reduced-order L1 adaptive logic used in the L1 path.
- `allocate_shared_adaptive_force(...)`
  Distributes a shared payload-level adaptive force across healthy cables.
- `project_local_adaptive_force(...)`
  Shapes post-fault local adaptive action, including local lateral throttling or cable-radial projection.
- `desired_geometry_offset(...)`
  Computes post-fault nominal offset redistribution for healthy vehicles.
- `simulate(...)`
  Runs the full reduced-order simulation for one mode and one scenario.
- `save_figures(...)`
  Generates summary plots for one CL-vs-L1 scenario.

#### Supported control structures in the reduced-order model

The simulator can evaluate two broad modes:

- `cl`
  Concurrent-learning baseline with ESO-style disturbance support.
- `l1`
  L1 adaptive compensation with optional structure refinements.

The L1 mode supports several design features that were added during the fault-tolerance study:

- local L1 compensation
- shared payload-level L1 compensation
- allocator choice for shared compensation
- post-fault local lateral throttling
- projection of local adaptive action into specific planar directions
- post-fault desired-geometry redistribution

This makes `topology_invariance_root.py` the reduced-order research sandbox for exploring not only tuning changes but also architectural changes.

#### Outputs from the main experiment script

When run directly, the script writes artifacts such as:

- scenario summary JSON
- CL-vs-L1 comparison plots
- payload RMSE comparison figure

The default output location is under `outputs/gpac_fault_tolerance/phase1_python` unless overridden.

### `python/fault_metrics_root.py`

This file contains the shared metric computation logic used throughout the experiment scripts.

It defines:

- `FaultMetrics`
  A compact dataclass describing fault-response behavior.
- `compute_fault_metrics(...)`
  A reusable function for turning time/error histories into evaluation metrics.

The metrics include:

- nominal RMSE
- pre-fault RMSE
- post-fault RMSE
- post/pre RMSE ratio
- peak deviation after fault
- recovery time

This helper is the common scoring backend for both single-scenario and matrix-style evaluation.

## Experiment Driver Scripts

### `python/run_l1_tuning_sweep.py`

This script performs a basic single-scenario parameter sweep over:

- `omega`
- `predictor_pole`
- `sigma_max`

It uses one seed and one faulted cable, compares each L1 candidate against a CL baseline, and writes a JSON grid containing:

- the CL baseline metrics
- the best L1 candidate
- all tested parameter combinations sorted by performance

Use this script when the goal is quick early-stage tuning rather than broad statistical validation.

### `python/run_l1_matrix_sweep.py`

This is the main tuning-search tool for the reduced-order fault-tolerance study.

It evaluates L1 candidates across:

- multiple random seeds
- multiple faulted cables
- one or more parameter grids for `omega`, `predictor_pole`, and `sigma_max`

It also supports the structural L1 options used in later iterations of the study, including:

- shared payload L1 settings
- local lateral weighting
- local projection mode
- desired-geometry redistribution mode and ramp time

For each candidate it computes aggregate measures such as:

- mean post-fault RMSE
- mean peak deviation
- mean recovery time
- cable-to-cable spread
- delta versus CL
- weighted tradeoff score

It writes a JSON artifact containing:

- run configuration
- CL baseline summary
- all candidate summaries
- best candidate by raw RMSE
- best candidate by weighted tradeoff
- Pareto front

Use this script when the goal is broad search and tradeoff analysis, not just one-off tuning.

### `python/run_fault_matrix.py`

This script validates one chosen L1 configuration against CL across a matrix of seeds and faulted cables.

It produces several artifact types:

- `fault_matrix_summary.json`
- `fault_matrix_summary.csv`
- `fault_matrix_report.md`
- optional `fault_matrix_summary.png`

This is the script to use once a candidate controller structure or tuning has already been selected and you want a matched CL-vs-L1 comparison over a broader set of scenarios.

In practice, this script is the main validation step between:

- candidate generation from the sweep tools
- and the markdown reporting tools downstream

### `python/run_unknowns_status_pack.py`

This script is the top-level orchestration entrypoint for the README unknowns U1-U6.

It drives the existing root-owned scripts end to end:

- optional regeneration of the full-Drake batch or render-only artifact refresh
- full-Drake batch analysis
- reduced-order reference summaries for the canonical 3/5/7 manifest
- reduced-order versus full-Drake comparison
- topology-invariance condition validation
- CL history-corruption analysis
- extended gradual-degradation runs across the replayed 3/5/7-drone scenarios
- replayable non-CL adaptive-law search for U6
- consolidated status report assembly

The output is a self-contained artifact folder under `outputs/gpac_fault_tolerance/unknowns_status_pack/` containing:

- `unknowns_status.json`
- `unknowns_status_report.md`
- the intermediate reduced-order and U2/U3/U5/U6 artifacts used to build the report
- `run_unknowns_status_pack_manifest.json` describing the exact inputs used for the run

For a fast smoke pass while iterating on the orchestration itself, run the script with `--profile smoke`.

For long report-grade runs, use `--profile report` together with `--resume` to skip completed stage outputs, or `--stages` to execute only selected stages such as `u6,status_pack,run_manifest`. The runner also writes `run_unknowns_status_pack_stage_status.json` in the output directory so you can see which stage is running, completed, reused, skipped, or failed, along with per-stage timestamps and durations.

### `python/run_multi_agent_recording.py`

This script runs a single reduced-order suspended-load scenario with a configurable number of quadrotors and defaults to a five-agent fault case.

It is intended for signal-level inspection rather than parameter search.

By default it:

- instantiates a five-quadrotor formation
- injects one cable snap at a chosen time
- runs both CL and L1 fault-response scenarios
- records payload, quad, cable, wind, and controller signals into compressed `.npz` artifacts
- writes a JSON summary of the scenario and metrics
- generates multi-panel plots for payload tracking, per-quad signals, cable/controller behavior, and XY geometry

Use this script when the goal is to inspect full time histories for one multi-vehicle scenario and keep a root-owned artifact bundle that is easy to review.

### `python/run_multi_agent_meshcat.py`

This script re-runs one reduced-order multi-agent scenario and visualizes it in Meshcat using the shared Drake-enabled Python environment.

It creates a browser-viewable 3D scene with:

- payload actual and desired positions
- per-quad actual and desired positions
- actual and desired cable segments
- full actual and desired trajectory traces

It supports both live stepping and Meshcat recording publication.

Use this script when the goal is to inspect the scenario spatially in a browser rather than through static matplotlib figures.

## Reporting and Candidate Selection

### `python/build_fault_tolerance_report.py`

This script merges two existing experiment artifacts:

- a matrix sweep JSON
- a fault matrix JSON

It then generates a markdown report that summarizes:

- the best L1 candidate
- the best balanced candidate
- top candidate tables
- optional constrained feasible sets
- aggregate CL-vs-L1 comparison
- scenario rankings by post-fault RMSE and by peak deviation
- a short recommendation section

Use this script when you already have experiment outputs and want a human-readable report for review, documentation, or decision making.

### `python/select_l1_candidates.py`

This script filters a matrix sweep result using explicit design constraints such as:

- minimum RMSE improvement versus CL
- maximum spread penalty versus CL
- maximum peak penalty versus CL

It then ranks the feasible candidates by one selected criterion:

- RMSE
- spread
- peak
- overall tradeoff score

The output is a markdown selection report that makes it easier to decide which L1 candidates deserve deeper validation.

Use this script when the search space is already computed and the next task is narrowing the candidate set to something reviewable and defensible.

## Typical Workflow

The intended experiment workflow is usually:

1. Start with `topology_invariance_root.py`
   Verify that one CL-vs-L1 scenario behaves as expected and inspect the plots.
2. Run `run_l1_tuning_sweep.py`
   Perform a quick single-scenario scan for rough L1 tuning ranges.
3. Run `run_l1_matrix_sweep.py`
   Evaluate candidates across multiple seeds and faulted cables.
4. Run `select_l1_candidates.py`
   Apply explicit performance constraints to trim the candidate set.
5. Run `run_fault_matrix.py`
   Validate one chosen candidate against CL over the target scenario matrix.
6. Run `build_fault_tolerance_report.py`
   Produce a consolidated markdown report from the sweep and validation artifacts.

This separation is deliberate:

- the model file owns simulation logic
- the sweep scripts own search logic
- the fault-matrix script owns matched validation
- the report scripts own artifact summarization

## Common Inputs

Across the scripts, the most common inputs are:

- duration
- fault time
- seed or seed list
- fault cable or fault-cable list
- wind sigma
- L1 adaptive parameters
- shared payload L1 parameters
- local projection and lateral-weight settings
- desired-geometry redistribution settings
- output directory

Most scripts expose these as CLI arguments so experiments can be rerun reproducibly from the terminal.

## Common Outputs

The experiment stack primarily emits artifacts under `outputs/gpac_fault_tolerance/`.

Depending on the script, artifacts can include:

- JSON summaries
- CSV tables
- Markdown reports
- PNG summary plots

These outputs serve different roles:

- JSON for machine-readable intermediate results
- CSV for quick tabular inspection
- Markdown for human-readable review
- PNG for visual sanity checks and presentations

## Metric Philosophy

The experiment module does not optimize a single scalar objective by default. Instead, it keeps several fault-tolerance measures visible at once:

- payload tracking accuracy after fault
- peak excursion immediately after fault
- recovery behavior
- variation across faulted cables, used as a topology-invariance proxy

This matters because some tunings improve RMSE while worsening cable-to-cable spread. The scripts in this directory are structured to expose that tradeoff rather than hide it.

## Why This Module Exists

This experiment layer exists to answer research and integration questions quickly before committing to heavier C++ or Drake-side implementation. It provides:

- a root-owned validation environment
- repeatable tuning and comparison tools
- a bridge between qualitative controller ideas and measurable fault-response evidence

In the broader Tether_Grace workflow, this directory is the rapid-evaluation layer that sits between:

- design ideas from the references and planning documents
- and root-owned implementation in `src/gpac_fault_tolerance/`

## Notes and Boundaries

- This is a reduced-order model, not the full production simulation stack.
- The scripts are intended for controller comparison and design iteration, not for proving full-system flight readiness.
- The generated artifacts are inputs to engineering judgment; they are not a substitute for higher-fidelity simulation or hardware validation.
- The experiments remain root-owned and do not require modifying Tether_Lift.

## Summary

`experiments/` is the root-owned fault-tolerance experimentation module for Tether_Grace. It provides:

- a reduced-order CL-vs-L1 cable-snap simulator
- shared fault metric computation
- parameter sweep tooling
- multi-seed, multi-cable validation runners
- constrained candidate selection
- markdown report generation

Together, these scripts make it possible to move from controller ideas to quantified evidence quickly, while preserving the repository boundary between Tether_Grace and its read-only submodules.