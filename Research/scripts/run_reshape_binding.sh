#!/usr/bin/env bash
# Aggressive-period reshape binding probe (§V.D of IEEE paper).
#
# At canonical T=10s period the reshape layer is provably preservation-
# only (Thm 4, Prop 2) because trajectory tension is not the
# binding constraint. At aggressive T=6s the trajectory-induced
# peak pushes the binding regime and the reshape supervisor is
# predicted to bite (25.8% tension reduction prediction).
#
# Two paired runs at seed 42:
#   - T=6s, reshape OFF (baseline)
#   - T=6s, reshape ON (treatment)
#
# Output: output/reshape_binding/<tag>/

set -u

EXE=/workspaces/Tether_Grace/Research/cpp/build/decentralized_fault_aware_sim
ROOT=/workspaces/Tether_Grace/output/reshape_binding
mkdir -p "${ROOT}"

PAYLOAD_KG=10
WIND_MPS=4.0
WIND_SEED=42
DURATION=30
TRAJ=lemniscate3d
NUM_QUADS=5
PERIOD=6.0
T1=12.0
T2=16.48   # t1 + 2·τ_pend (dwell=2) to trigger reshape window

run_one() {
    local tag="$1"
    local reshape_flag="$2"
    local outdir="${ROOT}/${tag}"
    mkdir -p "${outdir}"
    echo "=== ${tag} (period=${PERIOD}s, reshape=${reshape_flag})"
    local w0; w0=$(date +%s)
    "$EXE" --output-dir "${outdir}" --scenario "${tag}" \
           --num-quads ${NUM_QUADS} --duration ${DURATION} \
           --trajectory ${TRAJ} --lemniscate-period ${PERIOD} \
           --payload-mass ${PAYLOAD_KG} \
           --wind-speed ${WIND_MPS} --wind-seed ${WIND_SEED} \
           --fault-0-quad 0 --fault-0-time "${T1}" \
           --fault-1-quad 2 --fault-1-time "${T2}" \
           ${reshape_flag} \
           > "${outdir}/run.log" 2>&1
    local w1; w1=$(date +%s)
    echo "   rc=$? elapsed=$((w1-w0))s"
}

run_one reshape_T6_off ""
run_one reshape_T6_on  "--reshaping-enabled"

echo "reshape-binding sweep complete"
