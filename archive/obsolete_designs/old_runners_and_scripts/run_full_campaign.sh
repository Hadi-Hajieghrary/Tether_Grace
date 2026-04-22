#!/bin/bash
# Run all 6 decentralized fault-aware scenarios + generate plots.
set -e
EXE=/workspaces/Tether_Grace/Research/cpp/build/decentralized_fault_aware_sim
OUT=/workspaces/Tether_Grace/Research/decentralized_replays
PLOT=/workspaces/Tether_Grace/Research/analysis/scenario_plots.py
mkdir -p "$OUT"

run_sim() {
    local name=$1; shift
    echo "======================================================"
    echo "=== $name ==="
    echo "======================================================"
    "$EXE" --num-quads 4 --output-dir "$OUT" --scenario "$name" "$@"
    python3 "$PLOT" "$OUT/scenario_${name}.csv" "$OUT/scenario_${name}.png"
}

# --- Straight-traverse scenarios ---
run_sim A_nominal           --duration 20 --trajectory traverse
run_sim B_single_fault      --duration 20 --trajectory traverse \
        --fault-0-quad 0 --fault-0-time 10
run_sim C_dual_fault        --duration 20 --trajectory traverse \
        --fault-0-quad 0 --fault-0-time 8 \
        --fault-1-quad 2 --fault-1-time 14

# --- Figure-8 (lemniscate) scenarios ---
run_sim D_figure8_nominal   --duration 30 --trajectory figure8
run_sim E_figure8_fault     --duration 30 --trajectory figure8 \
        --fault-0-quad 0 --fault-0-time 12

# --- Stress test: triple sequential failure ---
run_sim F_triple_fault      --duration 25 --trajectory traverse \
        --fault-0-quad 0 --fault-0-time 7 \
        --fault-1-quad 2 --fault-1-time 13 \
        --fault-2-quad 3 --fault-2-time 18

echo ""
echo "All scenarios complete. Outputs in: $OUT"
ls -lh "$OUT"/*.html "$OUT"/*.png 2>&1 | tail -20
