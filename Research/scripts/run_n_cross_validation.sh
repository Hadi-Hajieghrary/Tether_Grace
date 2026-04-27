#!/usr/bin/env bash
# WP2.3: Team-size cross-validation runs.
#
# Two team sizes complementing the canonical N=5: N=3 (with payload
# scaled to keep the actuator-margin ratio comparable, m_L=6 kg) and
# N=7 (m_L=14 kg). V4-style dual-fault schedule. Single seed.
# Existence-of-recovery test, not statistical sampling.
#
# Output: output/n_cross_validation/N{3,7}/
set -u

EXE=/workspaces/Tether_Grace/Research/cpp/build/decentralized_fault_aware_sim
ROOT=/workspaces/Tether_Grace/output/n_cross_validation
mkdir -p "${ROOT}"

WIND_MPS=4.0
WIND_SEED=42
DURATION=30
TRAJ=lemniscate3d
T1=12.0
T2=17.0

run_single() {
    local N="$1"; local M="$2"; local i1="$3"
    local tag="N${N}_m${M}kg_F1"
    local outdir="${ROOT}/${tag}"
    mkdir -p "${outdir}"
    echo "=== ${tag}"
    local w0; w0=$(date +%s)
    "$EXE" --output-dir "${outdir}" --scenario "${tag}" \
           --num-quads "${N}" --duration ${DURATION} \
           --trajectory ${TRAJ} --payload-mass "${M}" \
           --wind-speed ${WIND_MPS} --wind-seed ${WIND_SEED} \
           --fault-0-quad "${i1}" --fault-0-time "${T1}" \
           > "${outdir}/run.log" 2>&1
    local w1; w1=$(date +%s)
    echo "   rc=$? elapsed=$((w1-w0))s"
}

run_dual() {
    local N="$1"; local M="$2"; local i1="$3"; local i2="$4"
    local tag="N${N}_m${M}kg_F2"
    local outdir="${ROOT}/${tag}"
    mkdir -p "${outdir}"
    echo "=== ${tag}"
    local w0; w0=$(date +%s)
    "$EXE" --output-dir "${outdir}" --scenario "${tag}" \
           --num-quads "${N}" --duration ${DURATION} \
           --trajectory ${TRAJ} --payload-mass "${M}" \
           --wind-speed ${WIND_MPS} --wind-seed ${WIND_SEED} \
           --fault-0-quad "${i1}" --fault-0-time "${T1}" \
           --fault-1-quad "${i2}" --fault-1-time "${T2}" \
           > "${outdir}/run.log" 2>&1
    local w1; w1=$(date +%s)
    echo "   rc=$? elapsed=$((w1-w0))s"
}

# N=3, m_L=6 kg, F=1 (one fault; F=2 would leave only 1 surviving
# drone — geometrically infeasible at N=3).
run_single 3 6 0
# N=7, m_L=14 kg, F=2 (canonical dual-fault schedule).
run_dual 7 14 0 2

echo "N cross-validation complete"
