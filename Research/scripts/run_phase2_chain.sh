#!/usr/bin/env bash
# Phase 2 master chain: sequentially run P2-C, P2-D, P2-B, P2-E.
#
# P2-A is launched separately and must finish before this chain starts
# (each campaign invokes the same Drake-backed simulator; parallel
# runs would fight for CPU).  This script is intended to be launched
# once P2-A has completed.
#
# Wall time estimate (based on observed ~45 min per 30-s sim):
#   P2-C: 6 runs @ 30 s  = ~4.5 h
#   P2-D: 6 runs @ 30 s  = ~4.5 h
#   P2-B: 12 runs @ 12 s = ~3.5 h
#   P2-E: 15 runs @ 30 s = ~11 h
#   Total:              ~23 h
#
# Logs each sub-campaign's summary to /tmp/phase2_chain_progress.log.

set -u

ROOT_DIR=/workspaces/Tether_Grace
PROGRESS=/tmp/phase2_chain_progress.log
echo "[$(date -Iseconds)] Phase 2 chain starting" > "${PROGRESS}"

run_campaign() {
    local name="$1"
    local script="$2"
    echo "[$(date -Iseconds)] === ${name} starting" >> "${PROGRESS}"
    local t0; t0=$(date +%s)
    bash "${script}" > "/tmp/phase2_${name}.log" 2>&1
    local rc=$?
    local t1; t1=$(date +%s)
    echo "[$(date -Iseconds)] === ${name} finished rc=${rc} elapsed=$((t1-t0))s" \
        >> "${PROGRESS}"
}

run_campaign "p2c" "${ROOT_DIR}/Research/scripts/run_p2c_mpc_ceiling_sweep.sh"
run_campaign "p2d" "${ROOT_DIR}/Research/scripts/run_p2d_period_sweep.sh"
run_campaign "p2b" "${ROOT_DIR}/Research/scripts/run_p2b_mass_mismatch.sh"
run_campaign "p2e" "${ROOT_DIR}/Research/scripts/run_p2e_wind_seed_sweep.sh"

echo "[$(date -Iseconds)] Phase 2 chain complete" >> "${PROGRESS}"
