#!/usr/bin/env bash
# WP4.5: Main-mode L1 test at the canonical operating point.
#
# V4 dual-fault schedule, payload 10 kg, FF enabled (canonical
# baseline), with vs without L1 layer added on top. n=4 paired
# wind-seed pairs (compute-budget compromise from the planned 8).
# Tests whether L1 delivers incremental benefit in main-mode at
# the canonical 10 kg point (rescue-mode in P2-B was at 3.9 kg).
#
# Output: output/wp4_l1_mainmode/<seed>_<config>/

set -u

EXE=/workspaces/Tether_Grace/Research/cpp/build/decentralized_fault_aware_sim
ROOT=/workspaces/Tether_Grace/output/wp4_l1_mainmode
mkdir -p "${ROOT}"

PAYLOAD_KG=10
WIND_MPS=4.0
DURATION=30
TRAJ=lemniscate3d
NUM_QUADS=5
T1=12.0; T2=17.0

SEEDS="42 100 200 300"

run_one() {
    local seed="$1"; local mode="$2"   # off | on
    local tag="seed${seed}_l1${mode}"
    local outdir="${ROOT}/${tag}"
    mkdir -p "${outdir}"
    local l1flag=""
    [ "${mode}" = "on" ] && l1flag="--l1-enabled"
    echo "=== ${tag}"
    local w0; w0=$(date +%s)
    "$EXE" --output-dir "${outdir}" --scenario "${tag}" \
           --num-quads ${NUM_QUADS} --duration ${DURATION} \
           --trajectory ${TRAJ} --payload-mass ${PAYLOAD_KG} \
           --wind-speed ${WIND_MPS} --wind-seed "${seed}" \
           --fault-0-quad 0 --fault-0-time "${T1}" \
           --fault-1-quad 2 --fault-1-time "${T2}" \
           ${l1flag} \
           > "${outdir}/run.log" 2>&1
    local w1; w1=$(date +%s)
    echo "   rc=$? elapsed=$((w1-w0))s"
}

for s in ${SEEDS}; do
    run_one "${s}" off
    run_one "${s}" on
done

echo "WP4 main-mode L1 sweep complete"
