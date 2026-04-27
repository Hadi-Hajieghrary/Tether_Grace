#!/usr/bin/env bash
# WP4.2: Fault-time permutation sweep on V4 schedule.
#
# Sweep first-fault time t_1 across {8, 10, 12, 14, 16}s,
# preserving inter-fault gap (5 s) and fault drone indices
# (drone 0 then drone 2). Tests whether recovery is
# trajectory-phase-invariant within cruise window.
#
# Output: output/fault_time_sweep/t1_<value>/
set -u

EXE=/workspaces/Tether_Grace/Research/cpp/build/decentralized_fault_aware_sim
ROOT=/workspaces/Tether_Grace/output/fault_time_sweep
mkdir -p "${ROOT}"

PAYLOAD_KG=10
WIND_MPS=4.0
WIND_SEED=42
DURATION=30
TRAJ=lemniscate3d
NUM_QUADS=5
GAP=5.0

T1_LIST="8 10 12 14 16"

run_one() {
    local t1="$1"
    local t2; t2=$(python3 -c "print(${t1} + ${GAP})")
    local tag="t1_${t1}s"
    local outdir="${ROOT}/${tag}"
    mkdir -p "${outdir}"
    echo "=== ${tag}  (t1=${t1}s, t2=${t2}s)"
    local w0; w0=$(date +%s)
    "$EXE" --output-dir "${outdir}" --scenario "${tag}" \
           --num-quads ${NUM_QUADS} --duration ${DURATION} \
           --trajectory ${TRAJ} --payload-mass ${PAYLOAD_KG} \
           --wind-speed ${WIND_MPS} --wind-seed ${WIND_SEED} \
           --fault-0-quad 0 --fault-0-time "${t1}" \
           --fault-1-quad 2 --fault-1-time "${t2}" \
           > "${outdir}/run.log" 2>&1
    local w1; w1=$(date +%s)
    echo "   rc=$? elapsed=$((w1-w0))s"
}

for t in ${T1_LIST}; do
    run_one "${t}"
done
echo "fault-time sweep complete"
