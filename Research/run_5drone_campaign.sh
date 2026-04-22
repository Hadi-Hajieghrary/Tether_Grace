#!/bin/bash
# 5-Drone Dynamic 3D-Lemniscate Campaign — four scenarios:
#   A  nominal baseline
#   B  single fault at t=15 s (mid-trajectory)
#   C  two faults 5 s apart (t=15, t=20) — rapid compound failure
#   D  two faults 10 s apart (t=15, t=25) — system settles between faults
#
# All four use a 40-second dynamic 3-D lemniscate reference trajectory
# and N = 5 drones in a symmetric formation. Output tree is separate
# from the original 4-drone campaign to avoid clobbering.
#
# Pre-requisite: the Drake sim target is built
#     Research/cpp/build/decentralized_fault_aware_sim

EXE=/workspaces/Tether_Grace/Research/cpp/build/decentralized_fault_aware_sim
ROOT=/workspaces/Tether_Grace/output/Tether_Grace_5drone
DATA=${ROOT}/08_source_data
mkdir -p "${DATA}"

# Clean only this campaign's output (leave 4-drone campaign untouched)
rm -f "${DATA}"/scenario_*.{csv,html}
rm -rf "${ROOT}"/0{1,2,3,4}_scenario_*
rm -rf "${ROOT}"/0{0,7}_*

run_sim() {
    local id=$1
    local dir=$2
    shift 2
    echo "======================================================"
    echo "=== ${id} ==="
    echo "======================================================"
    mkdir -p "${ROOT}/${dir}"
    "$EXE" --output-dir "${DATA}" --scenario "${id}" --num-quads 5 \
           --duration 40 --trajectory lemniscate3d "$@"
    sync
    if [ -f "${DATA}/scenario_${id}.html" ]; then
        cp "${DATA}/scenario_${id}.html" "${ROOT}/${dir}/replay.html"
    else
        echo "WARNING: no HTML for ${id}"
    fi
}

# Extra CLI flags to the outer script pass through to every run_sim
# call (e.g. `./run_5drone_campaign.sh --l1-enabled --controller=mpc`).
# This keeps the A/B/C/D scenario matrix identical but lets you enable
# any of the Phase-F / G / H extensions on top.
EXTRA="$@"

# -------- A: nominal 5-drone dynamic baseline --------
run_sim A_5drone_nominal            01_scenario_A_nominal ${EXTRA}

# -------- B: single fault mid-trajectory -------------
run_sim B_5drone_single_fault       02_scenario_B_single_fault \
        --fault-0-quad 0 --fault-0-time 15 ${EXTRA}

# -------- C: two faults 5 s apart --------------------
#   quad 0 at t=15, quad 2 at t=20 — 5 seconds apart (compound failure)
run_sim C_5drone_dual_5sec          03_scenario_C_dual_5sec \
        --fault-0-quad 0 --fault-0-time 15 \
        --fault-1-quad 2 --fault-1-time 20 ${EXTRA}

# -------- D: two faults 10 s apart -------------------
#   quad 0 at t=15, quad 2 at t=25 — 10 seconds apart (system settles between)
run_sim D_5drone_dual_10sec         04_scenario_D_dual_10sec \
        --fault-0-quad 0 --fault-0-time 15 \
        --fault-1-quad 2 --fault-1-time 25 ${EXTRA}

echo "======================================================"
echo "=== simulations complete — generating IEEE figures ==="
echo "======================================================"

PLOT_SCEN=/workspaces/Tether_Grace/Research/analysis/ieee/plot_scenario.py
PLOT_CMP=/workspaces/Tether_Grace/Research/analysis/ieee/plot_comparison.py
PLOT_PUB=/workspaces/Tether_Grace/Research/analysis/ieee/plot_publication.py

for s in A_5drone_nominal:01_scenario_A_nominal \
         B_5drone_single_fault:02_scenario_B_single_fault \
         C_5drone_dual_5sec:03_scenario_C_dual_5sec \
         D_5drone_dual_10sec:04_scenario_D_dual_10sec; do
    id=${s%:*}
    dir=${s#*:}
    python3 "$PLOT_SCEN" "${DATA}/scenario_${id}.csv" "${ROOT}/${dir}"
done

mkdir -p "${ROOT}/07_cross_scenario_comparison"
python3 "$PLOT_CMP" "${ROOT}" "${ROOT}/07_cross_scenario_comparison"

# Publication-grade multi-agent figure suite (12 main + 2 supplementary).
mkdir -p "${ROOT}/09_publication_figures"
python3 "$PLOT_PUB" "${ROOT}" "${ROOT}/09_publication_figures"

echo "All artefacts in: ${ROOT}"
du -sh "${ROOT}"/*/
