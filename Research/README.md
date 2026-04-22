# Research

Active source tree for the Tether_Grace controller stack and its
companion analysis pipeline. The authoritative "what does this file do"
map is [`docs/IMPLEMENTATION_STATUS.md`](docs/IMPLEMENTATION_STATUS.md);
this README is a roadmap of the directory as a whole.

## Layout

```
Research/
├── cpp/
│   ├── CMakeLists.txt
│   ├── include/                   ── public headers
│   └── src/                       ── implementations + simulator main
├── analysis/
│   └── ieee/                      ── publication figures and metrics
├── docs/
│   ├── IMPLEMENTATION_STATUS.md   ── code map (read first)
│   ├── INDEX.md                   ── navigation hub
│   ├── EXTENSION_PLAN.md          ── roadmap + status
│   ├── preregistration.md         ── frozen experiment plan
│   ├── theory/                    ── derivations
│   └── latex/                     ── LaTeX sources + built PDF
├── scripts/                       ── campaign runners
├── tests/                         ── Python verification tests
└── baseline/                      ── orientation notes on Tether_Lift
```

## The controller stack

| Layer | Source | CLI flag | Theory |
|-------|--------|----------|--------|
| Baseline QP | [`cpp/src/decentralized_local_controller.cc`](cpp/src/decentralized_local_controller.cc) | (default) | [`docs/theory/theory_decentralized_local_controller.md`](docs/theory/theory_decentralized_local_controller.md) |
| L1 adaptive outer loop | same file | `--l1-enabled` | [`docs/theory/theory_l1_extension.md`](docs/theory/theory_l1_extension.md) |
| CL observer | [`cpp/src/cl_param_estimator.cc`](cpp/src/cl_param_estimator.cc) | `--adaptive` | [`docs/theory/theory_l1_extension.md`](docs/theory/theory_l1_extension.md) §6 |
| Receding-horizon MPC | [`cpp/src/mpc_local_controller.cc`](cpp/src/mpc_local_controller.cc) | `--controller=mpc` | [`docs/theory/theory_mpc_extension.md`](docs/theory/theory_mpc_extension.md) |
| Formation reshape | [`cpp/include/fault_detector.h`](cpp/include/fault_detector.h) + [`cpp/include/formation_coordinator.h`](cpp/include/formation_coordinator.h) | `--reshaping-enabled` | [`docs/theory/theory_reshaping_extension.md`](docs/theory/theory_reshaping_extension.md) |

The layers compose independently; every combination is a valid
controller configuration and is exercised by the campaign matrix.

## Building

```bash
cmake -S cpp -B cpp/build -DCMAKE_BUILD_TYPE=Release
cmake --build cpp/build --target decentralized_fault_aware_sim -j8
```

The build assumes Drake is on the `CMAKE_PREFIX_PATH` (the devcontainer
sets this automatically). A single executable target is produced:
`decentralized_fault_aware_sim`.

## Running a scenario

```bash
./cpp/build/decentralized_fault_aware_sim \
    --num-quads 5 \
    --trajectory lemniscate3d \
    --duration 40 \
    --scenario A_nominal \
    --output-dir /tmp/run
```

Refer to [`scripts/run_5drone_campaign.sh`](scripts/run_5drone_campaign.sh) for the canonical flag combinations used in the
published campaigns, and to [`docs/IMPLEMENTATION_STATUS.md`](docs/IMPLEMENTATION_STATUS.md) for the full CLI reference.

## Where the numbers in the paper come from

Every numerical claim in the manuscript maps to exactly one row of
[`docs/preregistration.md`](docs/preregistration.md). That file is
frozen prior to running the campaign and must not be edited while
simulations are in flight.
