#!/usr/bin/env bash
# Small ablation campaign: four controller configurations × four fault
# scenarios on the five-drone lemniscate reference.
#
#   baseline   default single-step QP, no extensions
#   l1         +L1 adaptive outer loop
#   mpc        +MPC controller with T_max = 100 N
#   fullstack  MPC + L1 + formation-reshape (all three composed)
#
# Scenarios (same as run_5drone_campaign.sh): A_nominal, B_single_fault,
# C_dual_5sec, D_dual_10sec. Each run is 40 s long.
#
# Output tree:
#   output/ablation/<config>/
#     08_source_data/                raw CSV + Meshcat HTML per scenario
#     01_<scenario>/                 per-scenario replay
#     09_publication_figures/        publication plots + metrics CSV
#   output/ablation/summary_metrics.csv
#                                    cross-configuration summary
#
# Expected wall-time: ~4–8 hours (16 runs × 15–30 min, depending on
# whether the MPC is active). For the larger pre-registered campaign
# see run_transactions_campaign.sh; this script is kept for quick-turn
# sanity checks that do not need the full transactions matrix.
#
# Usage:
#   ./run_ablation_campaign.sh              # all four configs
#   ./run_ablation_campaign.sh baseline     # a single config
#   ./run_ablation_campaign.sh mpc fullstack
#
# Pre-requisite: Research/cpp/build/decentralized_fault_aware_sim is built.

# NOTE: no -e; a single-scenario failure should not kill the campaign.
set -u

EXE=/workspaces/Tether_Grace/Research/cpp/build/decentralized_fault_aware_sim
ROOT=/workspaces/Tether_Grace/output/ablation
mkdir -p "${ROOT}"

# Scenarios (shared by every config).
declare -A FAULTS
FAULTS[A_nominal]=""
FAULTS[B_single_fault]="--fault-0-quad 0 --fault-0-time 15"
FAULTS[C_dual_5sec]="--fault-0-quad 0 --fault-0-time 15 --fault-1-quad 2 --fault-1-time 20"
FAULTS[D_dual_10sec]="--fault-0-quad 0 --fault-0-time 15 --fault-1-quad 2 --fault-1-time 25"

# Config flag sets.
declare -A CFG
CFG[baseline]=""
CFG[l1]="--l1-enabled"
CFG[mpc]="--controller=mpc --mpc-horizon 5 --mpc-tension-max 100"
CFG[fullstack]="--controller=mpc --mpc-horizon 5 --mpc-tension-max 100 --l1-enabled --reshaping-enabled"

# If positional args given, run only those configs; else run all.
if [ $# -gt 0 ]; then
    CONFIGS=("$@")
else
    CONFIGS=(baseline l1 mpc fullstack)
fi

run_one() {
    local cfg=$1 scen=$2
    local cfg_flags="${CFG[$cfg]}"
    local fault_flags="${FAULTS[$scen]}"
    local out_dir="${ROOT}/${cfg}/08_source_data"
    mkdir -p "${out_dir}" "${ROOT}/${cfg}/01_${scen}"
    echo "==================================================="
    echo "=== ${cfg} / ${scen}   cfg=${cfg_flags}          ==="
    echo "==================================================="
    local t0=$(date +%s)
    "$EXE" --output-dir "${out_dir}" --scenario "${scen}" --num-quads 5 \
           --duration 40 --trajectory lemniscate3d \
           ${cfg_flags} ${fault_flags}
    local t1=$(date +%s)
    echo "--- ${cfg}/${scen} elapsed $((t1-t0)) s"
    sync
    if [ -f "${out_dir}/scenario_${scen}.html" ]; then
        cp "${out_dir}/scenario_${scen}.html" "${ROOT}/${cfg}/01_${scen}/replay.html"
    fi
}

for cfg in "${CONFIGS[@]}"; do
    for scen in A_nominal B_single_fault C_dual_5sec D_dual_10sec; do
        run_one "${cfg}" "${scen}"
    done
done

echo "==================================================="
echo "=== Ablation campaign complete — running figures ==="
echo "==================================================="

PLOT_PUB=/workspaces/Tether_Grace/Research/analysis/ieee/plot_publication.py
for cfg in "${CONFIGS[@]}"; do
    mkdir -p "${ROOT}/${cfg}/09_publication_figures"
    python3 "${PLOT_PUB}" "${ROOT}/${cfg}" "${ROOT}/${cfg}/09_publication_figures" \
        > "${ROOT}/${cfg}/plot_publication.log" 2>&1 || true
done

# Build the cross-config summary CSV from each config's
# publication_metrics.csv so plot_ablation.py (next step) can pick it up.
python3 <<'PY'
import pandas as pd, os
from pathlib import Path
root = Path("/workspaces/Tether_Grace/output/ablation")
frames = []
for cfg_dir in sorted(root.iterdir()):
    m = cfg_dir / "09_publication_figures" / "publication_metrics.csv"
    if not m.exists():
        continue
    df = pd.read_csv(m)
    df.insert(0, "config", cfg_dir.name)
    frames.append(df)
if frames:
    out = root / "summary_metrics.csv"
    pd.concat(frames, ignore_index=True).to_csv(out, index=False)
    print(f"Wrote {out}")
else:
    print("No per-config metrics found; check plot_publication.log per config.")
PY

echo "All artefacts in ${ROOT}"
du -sh "${ROOT}"/*/ 2>/dev/null
