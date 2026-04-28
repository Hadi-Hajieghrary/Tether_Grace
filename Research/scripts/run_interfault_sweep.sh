#!/usr/bin/env bash
# Deterministic inter-fault-gap sweep for the five-drone lemniscate
# reference. The first fault is fixed at t = 15 s on drone 0; the
# second fault lands on drone 2 at t = 15 + Δt for
# Δt ∈ {2, 4, 7, 10, 14, 20} s. Each run produces a CSV plus a
# sidecar JSON recording the inter-fault gap so the analysis pipeline
# can index the sweep.
#
# Purpose: characterise the compound-failure peak-tension envelope as
# a function of fault spacing. The simulator is fully deterministic —
# no Monte-Carlo sampling is involved — so one run per Δt is sufficient.
#
# Duration is trimmed to 28 s to keep total wall-time bounded; all
# faults still land well within the quasi-steady cruise window.
#
# Pre-requisite: Research/cpp/build/decentralized_fault_aware_sim is built.

set -u  # don't use -e because cp may race

EXE=/workspaces/Tether_Grace/Research/cpp/build/decentralized_fault_aware_sim
ROOT=/workspaces/Tether_Grace/output/5drone_baseline_campaign/inter_fault_sweep
mkdir -p "${ROOT}"

# Clean previous runs in this sub-campaign.
rm -f "${ROOT}"/scenario_dt_*.{csv,html,json}

# First fault always at t = 15 s on drone 0.
# Second fault on drone 2 at t = 15 + Δt.
# Six points span the range [2, 20] s with higher resolution at short
# gaps where the transient-overlap effect is strongest.
T_FIRST=15

DURATION=28
for DT in 2 4 7 10 14 20; do
    T_SECOND=$(python3 -c "print(${T_FIRST} + ${DT})")
    ID="dt_${DT}"
    DIR="${ROOT}"
    echo "============================================================"
    echo "=== Δt = ${DT} s (faults @ ${T_FIRST} & ${T_SECOND} s) ==="
    echo "============================================================"
    "$EXE" --output-dir "${DIR}" --scenario "${ID}" --num-quads 5 \
           --duration "${DURATION}" --trajectory lemniscate3d \
           --fault-0-quad 0 --fault-0-time "${T_FIRST}" \
           --fault-1-quad 2 --fault-1-time "${T_SECOND}"
    # Sidecar metadata so downstream plotting tools can index Δt.
    python3 -c "
import json
with open('${DIR}/scenario_${ID}.json', 'w') as f:
    json.dump({'inter_fault_gap': ${DT},
               't_fault_0': ${T_FIRST}, 't_fault_1': ${T_SECOND},
               'duration': ${DURATION}, 'num_quads': 5,
               'trajectory': 'lemniscate3d'}, f)
"
done

echo "============================================================"
echo "=== sweep complete. artefacts in: ${ROOT}                ==="
echo "============================================================"
ls -lh "${ROOT}"/scenario_dt_*.csv 2>/dev/null
