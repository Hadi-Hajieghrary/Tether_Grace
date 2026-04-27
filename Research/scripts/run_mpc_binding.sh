#!/usr/bin/env bash
# Tier-3H: MPC-Binding Scenario.
#
# Drive the MPC tension-ceiling layer into actually-binding regime
# with heavier payload (25 kg), faster reference (T=6s), tighter
# ceiling (40-60 N), dual faults at canonical V4 schedule.
#
# Pairs: (mpc-on/off) at three ceiling values.
#
# Output: output/mpc_binding/<tag>/

set -u

EXE=/workspaces/Tether_Grace/Research/cpp/build/decentralized_fault_aware_sim
ROOT=/workspaces/Tether_Grace/output/mpc_binding
mkdir -p "${ROOT}"

PAYLOAD_KG=25
WIND_MPS=4.0
WIND_SEED=42
DURATION=30
TRAJ=lemniscate3d
NUM_QUADS=5
PERIOD=6.0
T1=12.0
T2=17.0

run_one() {
    local tag="$1"
    local controller="$2"   # baseline | mpc
    local ceiling="$3"
    local outdir="${ROOT}/${tag}"
    mkdir -p "${outdir}"
    echo "=== ${tag} (controller=${controller}, ceiling=${ceiling}N, payload=${PAYLOAD_KG}kg, T_ref=${PERIOD}s)"
    local w0; w0=$(date +%s)
    if [ "${controller}" = "mpc" ]; then
        "$EXE" --output-dir "${outdir}" --scenario "${tag}" \
               --num-quads ${NUM_QUADS} --duration ${DURATION} \
               --trajectory ${TRAJ} --lemniscate-period ${PERIOD} \
               --payload-mass ${PAYLOAD_KG} \
               --wind-speed ${WIND_MPS} --wind-seed ${WIND_SEED} \
               --fault-0-quad 0 --fault-0-time "${T1}" \
               --fault-1-quad 2 --fault-1-time "${T2}" \
               --controller mpc --mpc-tension-max "${ceiling}" \
               > "${outdir}/run.log" 2>&1
    else
        "$EXE" --output-dir "${outdir}" --scenario "${tag}" \
               --num-quads ${NUM_QUADS} --duration ${DURATION} \
               --trajectory ${TRAJ} --lemniscate-period ${PERIOD} \
               --payload-mass ${PAYLOAD_KG} \
               --wind-speed ${WIND_MPS} --wind-seed ${WIND_SEED} \
               --fault-0-quad 0 --fault-0-time "${T1}" \
               --fault-1-quad 2 --fault-1-time "${T2}" \
               > "${outdir}/run.log" 2>&1
    fi
    local w1; w1=$(date +%s)
    echo "   rc=$? elapsed=$((w1-w0))s"
}

# Baseline (no MPC) reference
run_one mpc_binding_baseline baseline 0
# MPC at three ceiling levels
run_one mpc_binding_T040 mpc 40
run_one mpc_binding_T060 mpc 60
run_one mpc_binding_T080 mpc 80

echo "mpc-binding probe complete"
