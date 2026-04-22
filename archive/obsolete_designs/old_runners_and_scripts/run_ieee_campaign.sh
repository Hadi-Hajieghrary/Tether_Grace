#!/bin/bash
# IEEE-paper campaign: 6 scenarios, each scenario gets its own sub-folder of
# 10 PNGs + HTML replay. A shared data folder stores the raw CSV.
# NB: not using set -e — one failing scenario must not abort the campaign.

EXE=/workspaces/Tether_Grace/Research/cpp/build/decentralized_fault_aware_sim
ROOT=/workspaces/Tether_Grace/output/Tether_Grace
DATA=${ROOT}/08_source_data
mkdir -p "${DATA}"

# Clean prior
rm -f "${DATA}"/scenario_*.{csv,html}
rm -rf "${ROOT}"/0{1,2,3,4,5,6}_scenario_*

run_sim() {
    local id=$1
    local dir=$2
    shift 2
    echo "======================================================"
    echo "=== ${id} ==="
    echo "======================================================"
    mkdir -p "${ROOT}/${dir}"
    "$EXE" --output-dir "${DATA}" --scenario "${id}" "$@"
    # Give the filesystem a moment to flush (defensive against the
    # intermittent "HTML still materialising" race we hit in v6).
    sync
    # Copy HTML into the scenario folder. Keep the source too so a missing
    # file does not take down the whole campaign.
    if [ -f "${DATA}/scenario_${id}.html" ]; then
        cp "${DATA}/scenario_${id}.html" "${ROOT}/${dir}/replay.html"
    else
        echo "WARNING: no HTML for ${id} at ${DATA}/scenario_${id}.html"
    fi
}

# --------- 6 scenarios for the paper --------------------------------
run_sim S1_nominal_traverse 01_scenario_S1_nominal_traverse \
    --num-quads 4 --duration 25 --trajectory traverse

run_sim S2_cruise_fault 02_scenario_S2_cruise_fault \
    --num-quads 4 --duration 25 --trajectory traverse \
    --fault-0-quad 0 --fault-0-time 12

run_sim S3_dual_sequential 03_scenario_S3_dual_sequential \
    --num-quads 4 --duration 25 --trajectory traverse \
    --fault-0-quad 0 --fault-0-time 8 \
    --fault-1-quad 2 --fault-1-time 16

run_sim S4_figure8_nominal 04_scenario_S4_figure8_nominal \
    --num-quads 4 --duration 40 --trajectory figure8

run_sim S5_figure8_fault 05_scenario_S5_figure8_fault \
    --num-quads 4 --duration 40 --trajectory figure8 \
    --fault-0-quad 0 --fault-0-time 20

run_sim S6_triple_stress 06_scenario_S6_triple_stress \
    --num-quads 4 --duration 25 --trajectory traverse \
    --fault-0-quad 0 --fault-0-time 7 \
    --fault-1-quad 2 --fault-1-time 13 \
    --fault-2-quad 3 --fault-2-time 18

echo "======================================================"
echo "=== simulations complete — generating IEEE figures ==="
echo "======================================================"

PLOT_SCEN=/workspaces/Tether_Grace/Research/analysis/ieee/plot_scenario.py
PLOT_SYS=/workspaces/Tether_Grace/Research/analysis/ieee/plot_system.py
PLOT_CMP=/workspaces/Tether_Grace/Research/analysis/ieee/plot_comparison.py
PLOT_PUB=/workspaces/Tether_Grace/Research/analysis/ieee/plot_publication.py

# Per-scenario plots
for s in S1_nominal_traverse:01_scenario_S1_nominal_traverse \
         S2_cruise_fault:02_scenario_S2_cruise_fault \
         S3_dual_sequential:03_scenario_S3_dual_sequential \
         S4_figure8_nominal:04_scenario_S4_figure8_nominal \
         S5_figure8_fault:05_scenario_S5_figure8_fault \
         S6_triple_stress:06_scenario_S6_triple_stress; do
    id=${s%:*}
    dir=${s#*:}
    python3 "$PLOT_SCEN" "${DATA}/scenario_${id}.csv" "${ROOT}/${dir}"
done

# System-level figures
python3 "$PLOT_SYS" "${ROOT}/00_system_architecture"

# Cross-scenario comparison
python3 "$PLOT_CMP" "${ROOT}" "${ROOT}/07_cross_scenario_comparison"

# Publication-grade multi-agent figure suite (12 main + 2 supplementary).
mkdir -p "${ROOT}/09_publication_figures"
python3 "$PLOT_PUB" "${ROOT}" "${ROOT}/09_publication_figures"

echo "All artifacts in: ${ROOT}"
du -sh "${ROOT}"/*/ 2>/dev/null
