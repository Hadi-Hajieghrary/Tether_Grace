#!/usr/bin/env bash
# Phase-2 watcher: runs analysis + report-update for each P2 campaign
# as soon as it completes (by watching /tmp/phase2_chain_progress.log).
# Intended to run in the background alongside the chain.

set -u

PROGRESS=/tmp/phase2_chain_progress.log
LOG=/tmp/phase2_watcher.log
touch "${PROGRESS}" "${LOG}"

wait_for() {
    local pattern="$1"
    while ! grep -q "${pattern}" "${PROGRESS}" 2>/dev/null; do
        sleep 30
    done
}

process_campaign() {
    local name="$1"
    local plot_script="$2"
    local update_script="$3"
    echo "[$(date -Iseconds)] ${name}: running analysis" >> "${LOG}"
    cd /workspaces/Tether_Grace
    python3 "${plot_script}" >> "${LOG}" 2>&1
    python3 "${update_script}" >> "${LOG}" 2>&1
    echo "[$(date -Iseconds)] ${name}: analysis + report update done" >> "${LOG}"
}

# Wait for P2-C.
wait_for "p2c finished"
process_campaign "P2-C" \
    "Research/analysis/plot_p2c_mpc_ceiling.py" \
    "Research/analysis/update_p2c_report.py"

# Wait for P2-D.
wait_for "p2d finished"
process_campaign "P2-D" \
    "Research/analysis/plot_p2d_period_sweep.py" \
    "Research/analysis/update_p2d_report.py"

# Wait for P2-B.
wait_for "p2b finished"
process_campaign "P2-B" \
    "Research/analysis/plot_p2b_mass_mismatch.py" \
    "Research/analysis/update_p2b_report.py"

# Wait for P2-E.
wait_for "p2e finished"
process_campaign "P2-E" \
    "Research/analysis/plot_p2e_wind_seed.py" \
    "Research/analysis/update_p2e_report.py"

echo "[$(date -Iseconds)] WATCHER: all P2 campaigns analysed" >> "${LOG}"
