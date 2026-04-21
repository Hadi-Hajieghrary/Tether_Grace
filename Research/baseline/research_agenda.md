# Research Agenda Beyond Tether_Lift

This note turns the Tether_Lift baseline into a practical launchpad for new work in [../cpp](../cpp), [../scripts](../scripts), and [../outputs](../outputs).

## First Principle

Do not start by adding more theory on top of an ambiguous baseline.

Start by making the inherited baseline honest, reproducible, and extensible. New territories are more valuable once the baseline is explicit about what is live, what is dormant, and what is only claimed in paper form.

## Phase 0: Baseline Hardening

These tasks should happen before any major claim extension.

1. Create a successor executable in `Research/cpp` with workspace-safe defaults rooted in `/workspaces/Tether_Grace/Research/outputs`.
2. Add a machine-readable run manifest that records commit SHA, executable, seed, `N`, flags, output folder, and enabled subsystems.
3. Make `--headless` actually skip Meshcat and replay export when requested.
4. Split empirical figure generation from synthetic or illustrative figure generation.
5. Add a provenance table linking every figure/table to the exact run outputs used.

## Phase 1: Honest Successor Baseline

The most conservative next research step is not a brand-new controller. It is a clean successor baseline that reproduces the current live behavior with better observability.

That successor baseline should preserve:

- bead-chain rope physics
- payload and wind environment
- the current logging backbone
- the current nominal controller as a comparison baseline

It should improve:

- path portability
- module enable/disable clarity
- experiment manifests
- estimator and controller provenance
- figure traceability

## Phase 2: Incremental Activation Of Dormant Research Assets

Once the successor baseline is stable, the dormant Tether_Lift branches can be activated one layer at a time.

### Direction A: Estimator-closed control

- Activate the ESKF path in a controlled branch.
- Compare plant-truth closure versus estimator-closed closure under the same scenarios.
- Add explicit logging for estimator residuals, covariances, and control-side state source selection.

### Direction B: Load-centric control path

- Promote the load-centric planning and allocation modules into an executable path.
- Compare drone-centric waypoint tracking versus load-centric reference tracking under the same rope asymmetry and wind conditions.

### Direction C: GPAC layer integration

- Wire GPAC Layer 1 and Layer 2 first.
- Add per-layer logging before performance claims are made.
- Introduce CL, ESO, and CBF only after their activation state is visible in the run manifest and CSV outputs.

## New Research Territories Worth Pursuing

### 1. Geometry-aware decentralized load transport

The current baseline already contains the seed of decentralized estimation through local cable geometry. A strong next territory is to make the controller itself geometry-aware and load-centric rather than only drone-centric.

### 2. Honest safety-certified cooperative transport

The repository contains safety-filter code and paper ambition, but not a live baseline safety stack. A valuable research direction is to make safety supervision observable, reproducible, and empirically tied to specific fault and disturbance scenarios.

### 3. Estimation under slack, low tension, and transient pickup

The pickup phase is structurally important in Tether_Lift. New work can focus on state-estimation quality specifically during slack-to-taut transitions, where observability, outliers, and disturbance rejection are all weakest.

### 4. Scaling beyond the paper case

The simulator already supports variable `N`. New work can investigate how the bead-chain physics, force allocation, logging load, and estimator quality behave for larger formations once the evidence pipeline is cleaned up.

### 5. Fault-tolerant or degraded-mode transport

Tether_Grace already contains fault-tolerance work. A clean future bridge is to ground that fault-tolerant research against an honest Tether_Lift successor baseline rather than against the ambiguous mixed state of paper narrative and dormant code.

### 6. Reproducible scientific figure pipelines

Tether_Lift exposes a common research failure mode: figures can drift away from executable reality. A worthwhile meta-research contribution is a figure pipeline where every result panel is manifest-backed, regenerable, and clearly labeled as empirical or illustrative.

## Reuse, Isolate, Refactor

### Reuse first

- `RopeForceSystem`
- `rope_utils`
- `WindDisturbance`
- `WindForceApplicator`
- `SimulationDataLogger`
- committed Tether_Lift outputs as baseline comparison data

### Isolate early

- all path assumptions tied to `/workspaces/Tether_Lift`
- figure-generation functions with hard-coded paper numbers
- dormant GPAC and load-centric modules from the live baseline executable

### Refactor first

- runtime manifest generation
- subsystem enable/disable configuration
- estimator/control source selection logging
- figure provenance registry

## What Success Looks Like

The Research folder is ready for real expansion when it can do all of the following:

1. reproduce the inherited baseline with explicit evidence and portable commands
2. run one successor executable whose enabled subsystems are visible in a manifest
3. compare legacy and new controllers on the same scenarios without changing the evidence standard
4. generate paper-ready figures that are traceable to specific runs
5. state clearly which claims are baseline facts and which are new contributions