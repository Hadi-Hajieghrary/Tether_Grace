#!/bin/bash
# Run all 3 four-drone scenarios and save replay data

set -e

EXECUTABLE="/workspaces/Tether_Grace/Research/cpp/build/decentralized_mpc_four_drones_test"
OUTPUT_DIR="/workspaces/Tether_Grace/Research/four_drones_replays"

mkdir -p "$OUTPUT_DIR"

# Duration in seconds (start small for fast testing)
DURATION=${1:-5.0}

echo "=========================================="
echo "4-Drone Scenario Campaign"
echo "Duration: ${DURATION}s per scenario"
echo "Output: $OUTPUT_DIR"
echo "=========================================="

# Scenario A: Nominal (no faults)
echo ""
echo "### Scenario A: Nominal Multi-Drone Coordination ###"
"$EXECUTABLE" \
    --duration "$DURATION" \
    --scenario "A_nominal" \
    --output "$OUTPUT_DIR/scenario_A_nominal.csv" \
    --html "$OUTPUT_DIR/scenario_A_nominal.html" \
    2>&1 | tail -5

# Scenario B: Single cable severance at t=DURATION/2
echo ""
echo "### Scenario B: Single-Drone Failure (graceful degradation) ###"
SEVER_B=$(python3 -c "print($DURATION / 2.0)")
"$EXECUTABLE" \
    --duration "$DURATION" \
    --sever-0 "$SEVER_B" \
    --scenario "B_single_fault" \
    --output "$OUTPUT_DIR/scenario_B_single_fault.csv" \
    --html "$OUTPUT_DIR/scenario_B_single_fault.html" \
    2>&1 | tail -5

# Scenario C: Dual cable severance
echo ""
echo "### Scenario C: Dual-Drone Failure (system limits) ###"
SEVER_C0=$(python3 -c "print($DURATION * 0.3)")
SEVER_C1=$(python3 -c "print($DURATION * 0.6)")
"$EXECUTABLE" \
    --duration "$DURATION" \
    --sever-0 "$SEVER_C0" \
    --sever-1 "$SEVER_C1" \
    --scenario "C_dual_fault" \
    --output "$OUTPUT_DIR/scenario_C_dual_fault.csv" \
    --html "$OUTPUT_DIR/scenario_C_dual_fault.html" \
    2>&1 | tail -5

echo ""
echo "=========================================="
echo "All 3 scenarios complete"
echo "Replay files in: $OUTPUT_DIR"
echo "=========================================="
ls -lh "$OUTPUT_DIR"
