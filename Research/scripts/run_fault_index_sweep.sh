#!/usr/bin/env bash
# WP4.1: Fault-index permutation sweep on V4 schedule.
#
# Six fault-pair geometries on the 5-drone formation:
#   (0,1) (0,2) (0,3) (1,2) (1,3) (2,3)
# (the (0,4), (1,4), (2,4), (3,4) cases are equivalent by symmetry
# of the lemniscate trajectory). Tests geometric-invariance of
# recovery envelope.
#
# Output: output/fault_index_sweep/pair_<i1>_<i2>/
set -u

EXE=/workspaces/Tether_Grace/Research/cpp/build/decentralized_fault_aware_sim
ROOT=/workspaces/Tether_Grace/output/fault_index_sweep
mkdir -p "${ROOT}"

PAYLOAD_KG=10
WIND_MPS=4.0
WIND_SEED=42
DURATION=30
TRAJ=lemniscate3d
NUM_QUADS=5
T1=12.0
T2=17.0

# pairs as space-separated "i1,i2"
PAIRS="0,1 0,2 0,3 1,2 1,3 2,3"

run_one() {
    local i1="$1"
    local i2="$2"
    local tag="pair_${i1}_${i2}"
    local outdir="${ROOT}/${tag}"
    mkdir -p "${outdir}"
    echo "=== ${tag}"
    local w0; w0=$(date +%s)
    "$EXE" --output-dir "${outdir}" --scenario "${tag}" \
           --num-quads ${NUM_QUADS} --duration ${DURATION} \
           --trajectory ${TRAJ} --payload-mass ${PAYLOAD_KG} \
           --wind-speed ${WIND_MPS} --wind-seed ${WIND_SEED} \
           --fault-0-quad "${i1}" --fault-0-time "${T1}" \
           --fault-1-quad "${i2}" --fault-1-time "${T2}" \
           > "${outdir}/run.log" 2>&1
    local w1; w1=$(date +%s)
    echo "   rc=$? elapsed=$((w1-w0))s"
}

for pair in ${PAIRS}; do
    IFS=',' read -r I1 I2 <<< "${pair}"
    run_one "${I1}" "${I2}"
done

echo "fault-index sweep complete"
