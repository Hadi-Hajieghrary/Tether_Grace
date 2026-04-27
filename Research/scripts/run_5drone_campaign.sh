#!/usr/bin/env bash
# Five-drone A/B/C/D baseline campaign on the 3-D lemniscate reference.
#
#   A  nominal baseline (no faults)
#   B  single fault at t = 15 s on drone 0
#   C  two faults 5 s apart  (drone 0 @ 15 s,  drone 2 @ 20 s)
#   D  two faults 10 s apart (drone 0 @ 15 s,  drone 2 @ 25 s)
#
# Each run is 40 s long and uses the default controller unless extra
# flags are passed on the command line (they are forwarded to every
# run_sim call; see the EXTRA line below). After the four simulations
# the script calls the per-scenario and cross-scenario plotting
# pipelines plus the full publication-figure suite.
#
# Output tree: output/5drone_baseline_campaign/
#   01_scenario_A_nominal/       per-scenario plots + replay
#   02_scenario_B_single_fault/
#   03_scenario_C_dual_5sec/
#   04_scenario_D_dual_10sec/
#   07_cross_scenario_comparison/
#   08_source_data/              raw CSV + Meshcat HTML files
#   09_publication_figures/      12-figure publication suite
#
# Pre-requisite: Research/cpp/build/decentralized_fault_aware_sim exists
# (build it with cmake --build Research/cpp/build before running).

EXE=/workspaces/Tether_Grace/Research/cpp/build/decentralized_fault_aware_sim
ROOT=/workspaces/Tether_Grace/output/5drone_baseline_campaign
DATA=${ROOT}/08_source_data
mkdir -p "${DATA}"

# Clean only this campaign's output; archived campaigns under archive/
# are never touched.
rm -f "${DATA}"/scenario_*.{csv,html}
rm -rf "${ROOT}"/0{1,2,3,4}_scenario_*
rm -rf "${ROOT}"/0{0,7,9}_*

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

# Extra CLI flags pass through to every run_sim call, e.g.
#     ./run_5drone_campaign.sh --l1-enabled --controller=mpc
# keeps the A/B/C/D scenario matrix identical while layering on any of
# the optional controller extensions.
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
