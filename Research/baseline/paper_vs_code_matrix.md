# Tether_Lift Paper Versus Code Matrix

This matrix separates claims that are safe to inherit from claims that need re-validation before new research builds on them.

| Topic | Paper or README narrative | Live code reality | Status | What future work should do |
|---|---|---|---|---|
| Overall controller architecture | Tether_Lift is presented as a four-layer GPAC stack with CL, ESO, and CBF | The default executable in `Research/cpp/src/main.cc` uses `QuadcopterLiftController` as the live controller | Mismatch | Treat GPAC as a dormant branch until a new executable wires it in explicitly |
| State-estimator closure | Results section says all loops are closed through the ESKF | Default runtime sets `use_estimated_in_controller = false` | Mismatch | Reproduce baseline with explicit estimator-closed variants before inheriting this claim |
| Concurrent learning evidence | Manuscript presents CL convergence and no-CL ablations | Default runtime does not activate or log a CL-driven closed loop | Mismatch | Re-run with explicit CL wiring and manifest-backed evidence |
| ESO disturbance rejection | Manuscript presents ESO feedforward/tracking benefit | Default runtime contains wind and sensors, but not the full ESO-based control path | Mismatch | Treat ESO as future integration work, not baseline fact |
| CBF safety activity | Manuscript reports CBF activity and angle/tilt safety margins | Default runtime does not instantiate the GPAC CBF layer | Mismatch | Recompute safety claims only after activating and logging the safety path |
| Headless execution | Docs imply `--headless` disables visualization | Current code still initializes Meshcat, records, and exports HTML | Mismatch | Fix headless semantics in future research tools |
| Output path portability | Docs assume `/workspaces/Tether_Lift` layout | Current repository lives under `/workspaces/Tether_Grace/Tether_Lift` | Mismatch | Use explicit `--output-dir` and workspace-safe defaults in new executables |
| Figure provenance | Figures are presented as baseline results | The figure script mixes log-derived plots with synthetic or hard-coded support figures | Mismatch | Add a figure provenance registry before using the existing plots as evidence |
| Baseline physics model | Cooperative lift with bead-chain ropes, rope asymmetry, wind, and payload contact | This is exactly what the live executable builds and simulates | Observed | Safe to inherit as baseline plant and disturbance context |
| Local load-estimation idea | Decentralized load estimation from local cable geometry | Per-quad load estimators are instantiated and logged | Observed | Good candidate for extension, but note current downstream use is limited |
| GPAC code exists in repository | GPAC controllers, estimators, and safety layers are described in docs and paper | Those files are present in the C++ tree and compile, but are not active in the default runtime | Observed | Reuse them as research assets, not as already-validated baseline facts |
| Vendored Drake tree | Large Drake source tree exists under Tether_Lift | This is a dependency/vendor subtree, not the first-party research subject | Observed | Keep it outside the research claim boundary |

## Operational Rule

When a new experiment or paper section is added in `Research/`, classify every inherited Tether_Lift claim into one of three buckets:

- `Observed baseline fact`
- `Dormant code asset`
- `Paper aspiration requiring fresh evidence`

That simple discipline prevents the new workspace from inheriting ambiguity as if it were validated science.