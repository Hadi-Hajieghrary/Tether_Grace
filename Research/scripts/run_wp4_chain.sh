#!/usr/bin/env bash
# Sequential chain for WP4 + WP2.3 sweeps after fault-time finishes.
#
# This script polls the fault-time sweep state and only kicks off
# WP4.1 (fault-index) once it completes, followed by WP2.3
# (N-cross-validation). Total compute: ~3.5 h (5 + 6 + 2 runs).
set -u

ROOT=/workspaces/Tether_Grace/Research/scripts

# Wait for fault-time sweep to finish (last run is t1_16s).
echo "[chain] waiting for fault-time sweep (t1_16s)..."
until [ -f /workspaces/Tether_Grace/output/fault_time_sweep/t1_16s/run.log ] \
   && grep -q 'Signal log saved' /workspaces/Tether_Grace/output/fault_time_sweep/t1_16s/run.log; do
    sleep 60
done
echo "[chain] fault-time sweep done"

# WP4.1 fault-index sweep (6 runs, ~2.5 h).
echo "[chain] launching WP4.1 fault-index sweep"
bash "${ROOT}/run_fault_index_sweep.sh"

# WP2.3 N-cross-validation (2 runs, ~45 min).
echo "[chain] launching WP2.3 N-cross-validation"
bash "${ROOT}/run_n_cross_validation.sh"

echo "[chain] all chained sweeps complete"
