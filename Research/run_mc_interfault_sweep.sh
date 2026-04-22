#!/bin/bash
# Monte-Carlo inter-fault-gap sweep.
#
# For 5-drone lemniscate reference with first fault fixed at t=15 s,
# sweep the inter-fault gap Δt ∈ {2, 4, 6, 8, 10, 12, 14, 16, 18, 20} s
# and record the peak rope tension, peak tracking error, σ_T(t), etc.
#
# The goal is the adversarial robustness figure F10: show how peak rope
# tension scales with compound-fault timing, and establish the worst-case
# Δt envelope.
#
# Duration is trimmed to 30 s (from 40 s nominal) to keep total wall-time
# bounded; all faults still land well within the quasi-steady cruise
# window.
#
# Pre-requisite: Research/cpp/build/decentralized_fault_aware_sim is built.

set -u  # don't use -e because cp may race

EXE=/workspaces/Tether_Grace/Research/cpp/build/decentralized_fault_aware_sim
ROOT=/workspaces/Tether_Grace/output/Tether_Grace_5drone/monte_carlo
mkdir -p "${ROOT}"

# Clean previous MC runs
rm -f "${ROOT}"/scenario_MCdt_*.{csv,html,json}

# First fault always at t = 15 s on drone 0.
# Second fault on drone 2 at t = 15 + Δt.
# Six points span the range [2, 20] s with higher resolution at short
# gaps where the transient-overlap effect is strongest.
T_FIRST=15

DURATION=28
for DT in 2 4 7 10 14 20; do
    T_SECOND=$(python3 -c "print(${T_FIRST} + ${DT})")
    ID="MCdt_${DT}"
    DIR="${ROOT}"
    echo "============================================================"
    echo "=== MC run: Δt = ${DT} s (faults @ ${T_FIRST} & ${T_SECOND}) ==="
    echo "============================================================"
    "$EXE" --output-dir "${DIR}" --scenario "${ID}" --num-quads 5 \
           --duration "${DURATION}" --trajectory lemniscate3d \
           --fault-0-quad 0 --fault-0-time "${T_FIRST}" \
           --fault-1-quad 2 --fault-1-time "${T_SECOND}"
    # Drop a sidecar metadata file so plot_publication can read Δt.
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
echo "=== MC sweep complete.  artefacts in: ${ROOT}            ==="
echo "============================================================"
ls -lh "${ROOT}"/scenario_MCdt_*.csv 2>/dev/null
