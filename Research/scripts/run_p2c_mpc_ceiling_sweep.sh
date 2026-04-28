#!/usr/bin/env bash
# P2-C: reduced MPC tension-ceiling sweep.
#
# Runs V4's dual-fault schedule at successively lower tension ceilings
# with both baseline (ceiling inactive, ceiling is observed only) and
# MPC (ceiling enforced via the hard linearised tension constraint).
# The hypothesis is that as the ceiling tightens, the baseline
# continues to exceed it (no mechanism prevents it) while the MPC
# pulls the peak tension down to the ceiling at the cost of tracking
# performance.
#
# Output: output/p2c_mpc_ceiling_sweep/<variant>/
# Wall time: ~10 runs * 18 min = ~3 wall-hours.

set -u

EXE=/workspaces/Tether_Grace/Research/cpp/build/decentralized_fault_aware_sim
ROOT=/workspaces/Tether_Grace/output/p2c_mpc_ceiling_sweep
mkdir -p "${ROOT}"

PAYLOAD_KG=10
WIND_MPS=4.0
WIND_SEED=42
DURATION=30
TRAJ=lemniscate3d
NUM_QUADS=5

CEILINGS="60 70 80 90 100"

run_one() {
    local tag="$1"; shift
    local outdir="${ROOT}/${tag}"
    mkdir -p "${outdir}"
    echo "================================================================"
    echo "=== ${tag}"
    echo "================================================================"
    local t0; t0=$(date +%s)
    "$EXE" --output-dir "${outdir}" --scenario "${tag}" \
           --num-quads ${NUM_QUADS} --duration ${DURATION} \
           --trajectory ${TRAJ} --payload-mass ${PAYLOAD_KG} \
           --wind-speed ${WIND_MPS} --wind-seed ${WIND_SEED} \
           --fault-0-quad 0 --fault-0-time 12 \
           --fault-1-quad 2 --fault-1-time 17 \
           "$@" > "${outdir}/run.log" 2>&1
    local rc=$?
    local t1; t1=$(date +%s)
    echo "--- ${tag}  rc=${rc}  elapsed=$((t1-t0)) s"
    if [[ ${rc} -ne 0 ]]; then
        tail -30 "${outdir}/run.log"
    fi
}

# Baseline reference (one run — ceiling is MPC-only, so baseline
# behaviour is independent of the sweep axis).
run_one "p2c_baseline" --controller=baseline

for T in ${CEILINGS}; do
    run_one "p2c_mpc_T${T}" --controller=mpc --mpc-horizon 5 --mpc-tension-max ${T}
done

echo "================================================================"
echo "=== Summary"
echo "================================================================"
for v in "${ROOT}"/*/; do
    csv="${v}scenario_$(basename "$v").csv"
    if [[ -f "$csv" ]]; then
        printf "  %-28s rows=%6d\n" "$(basename "$v")" \
            "$(wc -l < "$csv")"
    else
        printf "  %-28s  MISSING\n" "$(basename "$v")"
    fi
done
