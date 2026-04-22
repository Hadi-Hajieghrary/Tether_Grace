# Tether_Grace

Decentralised, fault-tolerant cooperative payload lift with cable-suspended
quadrotors. The repository contains a Drake-based C++ simulator, the
controller stack (baseline QP, L1 adaptive outer loop, receding-horizon
MPC with tension-ceiling, and formation-reshape supervisor), an analysis
pipeline in Python, a tracked LaTeX manuscript and supplementary PDF, and
a campaign harness that produces the published numbers end-to-end.

## Top-level layout

| Path | Role |
|------|------|
| [`Research/`](Research) | All active source, documentation, analysis scripts, and manuscripts. |
| [`Tether_Lift/`](Tether_Lift) | Upstream baseline, vendored as a read-only git submodule. Not modified here. |
| [`output/`](output) | Simulation artefacts produced by the campaign runner. READMEs are tracked; raw CSVs, Meshcat replays, and per-tick logs are gitignored. |
| [`archive/`](archive) | Provenance of superseded code, documentation, and campaigns. See [`archive/README.md`](archive/README.md). |

Entry points for a new reader:

1. [`Research/docs/IMPLEMENTATION_STATUS.md`](Research/docs/IMPLEMENTATION_STATUS.md) — single-source code map. Every file under `Research/cpp/src` and `Research/analysis` is listed with its public interface and where its output flows.
2. [`Research/docs/theory/`](Research/docs/theory) — derivations for each controller layer.
3. [`Research/docs/latex/tether_grace_reference.pdf`](Research/docs/latex/tether_grace_reference.pdf) — the built manuscript.

## Reproducing the published numbers

```bash
# Build
cmake -S Research/cpp -B Research/cpp/build -DCMAKE_BUILD_TYPE=Release
cmake --build Research/cpp/build --target decentralized_fault_aware_sim -j8

# Run a single scenario (5-drone lemniscate, 40 s, dual 10-s fault)
./Research/cpp/build/decentralized_fault_aware_sim \
    --num-quads 5 --trajectory lemniscate3d --duration 40 \
    --scenario D_dual_10sec \
    --fault-0-quad 0 --fault-0-time 15 \
    --fault-1-quad 2 --fault-1-time 25 \
    --output-dir output/5drone_baseline_campaign/08_source_data

# Or run the full A/B/C/D sweep and publication-figure build
./Research/scripts/run_5drone_campaign.sh
```

The full IEEE-Transactions Monte-Carlo campaign (494 runs) is executed
by [`Research/scripts/run_transactions_campaign.sh`](Research/scripts/run_transactions_campaign.sh); expect ~16 wall-hours on an 8-core
host. See [`Research/docs/preregistration.md`](Research/docs/preregistration.md) for the frozen experiment specification.

## Dependencies

- **Drake** ≥ 1.32 (built against upstream master via the devcontainer).
- **Eigen** 3.4.
- **OSQP / osqp-eigen** (supplied by the Drake vendor bundle).
- **Python 3.10** with `numpy`, `pandas`, `matplotlib`, `scipy`, `pyyaml`.

A reproducible devcontainer lives in [`.devcontainer/`](.devcontainer); it
is a symlink into the Tether_Lift submodule.

## Citing

If you use this code, please cite the associated IEEE T-CST manuscript
(see [`Research/docs/latex/tether_grace_reference.pdf`](Research/docs/latex/tether_grace_reference.pdf) for the current draft title and abstract).

## License

Apache 2.0. See [`LICENSE`](LICENSE).
