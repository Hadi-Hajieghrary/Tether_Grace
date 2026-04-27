#!/usr/bin/env bash
# Post-WP4-chain launcher: waits for the WP4 chain (fault-time +
# fault-index + N-cross-validation) to complete, then runs
# WP4.4-extended robustness and WP4.5 main-mode-L1.

set -u
ROOT=/workspaces/Tether_Grace/Research/scripts
N_LAST=/workspaces/Tether_Grace/output/n_cross_validation/N7_m14kg_F2/run.log

echo "[post-chain] waiting for N-cross-validation to finish..."
until [ -f "${N_LAST}" ] && grep -q 'Signal log saved' "${N_LAST}"; do
    sleep 60
done
echo "[post-chain] WP4 chain done"

echo "[post-chain] launching WP4.4-ext robustness sweep"
bash "${ROOT}/run_wp4_robustness.sh"

echo "[post-chain] launching WP4.5 main-mode L1 sweep"
bash "${ROOT}/run_wp4_l1_mainmode.sh"

echo "[post-chain] all post-chain sweeps complete"
