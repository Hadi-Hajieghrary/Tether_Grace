#!/usr/bin/env bash
# P2-A: tension feed-forward ablation campaign.
#
# Isolates the contribution of the local T_ff = measured_tension
# identity to the emergent-fault-tolerance mechanism.  Runs the three
# fault-bearing scenarios (V3, V4, V5) with --disable-tension-ff and
# compares against the archived capability-demo versions (which were
# produced with T_ff on).
#
# Expected result (per theory):
#   - Baseline without T_ff cannot cancel the rope-pull force directly
#     at the thrust level; the outer PD loop has to regulate through
#     altitude error.  Peak tracking error during faults should
#     increase; peak tension during the transient should also rise
#     because the surviving drones react more slowly.
#
# Output: output/p2a_tension_ff_ablation/<variant>_nff/
#
# Wall time: ~3 * 18 min = ~1 hour.

set -u

EXE=/workspaces/Tether_Grace/Research/cpp/build/decentralized_fault_aware_sim
ROOT=/workspaces/Tether_Grace/output/p2a_tension_ff_ablation
mkdir -p "${ROOT}"

PAYLOAD_KG=10
WIND_MPS=4.0
WIND_SEED=42
DURATION=30
TRAJ=lemniscate3d
NUM_QUADS=5

run_variant() {
    local tag="$1"
    shift
    local outdir="${ROOT}/${tag}"
    mkdir -p "${outdir}"
    echo "================================================================"
    echo "=== ${tag}"
    echo "================================================================"
    local t0; t0=$(date +%s)
    "$EXE" --output-dir "${outdir}" --scenario "${tag}" \
           --num-quads ${NUM_QUADS} --duration ${DURATION} \
           --trajectory ${TRAJ} --payload-mass ${PAYLOAD_KG} \
           --disable-tension-ff \
           "$@" \
           > "${outdir}/run.log" 2>&1
    local rc=$?
    local t1; t1=$(date +%s)
    echo "--- ${tag}  rc=${rc}  elapsed=$((t1-t0)) s"
    if [[ ${rc} -ne 0 ]]; then
        echo "    FAILED — inspect ${outdir}/run.log"
        tail -30 "${outdir}/run.log"
    fi
}

# V3_nff: single fault
run_variant V3_single_wind_nff \
    --wind-speed ${WIND_MPS} --wind-seed ${WIND_SEED} \
    --fault-0-quad 0 --fault-0-time 12

# V4_nff: dual fault 5 s
run_variant V4_dual_5s_wind_nff \
    --wind-speed ${WIND_MPS} --wind-seed ${WIND_SEED} \
    --fault-0-quad 0 --fault-0-time 12 \
    --fault-1-quad 2 --fault-1-time 17

# V5_nff: dual fault 10 s
run_variant V5_dual_10s_wind_nff \
    --wind-speed ${WIND_MPS} --wind-seed ${WIND_SEED} \
    --fault-0-quad 0 --fault-0-time 12 \
    --fault-1-quad 2 --fault-1-time 22

echo "================================================================"
echo "=== Summary"
echo "================================================================"
for v in V3_single_wind_nff V4_dual_5s_wind_nff V5_dual_10s_wind_nff; do
    csv="${ROOT}/${v}/scenario_${v}.csv"
    if [[ -f "$csv" ]]; then
        printf "  %-28s rows=%6d\n" "$v" "$(wc -l < "$csv")"
    else
        printf "  %-28s  MISSING\n" "$v"
    fi
done
