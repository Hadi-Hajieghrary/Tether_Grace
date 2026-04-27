#!/usr/bin/env bash
# Tier-2E: Slack-Domain Boundary Phase Diagram.
#
# Sweep (T_ref, m_L) to map where the theorem's admissibility
# hypotheses (H1 slack budget, H3 QP-transition) break down.
# Paper recommends 6x5=30 grid points; to fit the session compute
# budget we use a 3x3 grid (9 points, ~3.5h) covering the
# boundary of interest.
#
# Output: output/slack_domain/T{6,8,12}s_m{10,14,18}kg/

set -u

EXE=/workspaces/Tether_Grace/Research/cpp/build/decentralized_fault_aware_sim
ROOT=/workspaces/Tether_Grace/output/slack_domain
mkdir -p "${ROOT}"

WIND_MPS=4.0
WIND_SEED=42
DURATION=30
TRAJ=lemniscate3d
NUM_QUADS=5
T1=12.0
T2=17.0  # canonical V4 schedule

# Reduced 3x3 grid for session feasibility.
PERIODS="6 8 12"
MASSES="10 14 18"

run_one() {
    local T="$1"
    local M="$2"
    local tag="T${T}s_m${M}kg"
    local outdir="${ROOT}/${tag}"
    mkdir -p "${outdir}"
    echo "=== ${tag} (T_ref=${T}s, m_L=${M}kg)"
    local w0; w0=$(date +%s)
    "$EXE" --output-dir "${outdir}" --scenario "${tag}" \
           --num-quads ${NUM_QUADS} --duration ${DURATION} \
           --trajectory ${TRAJ} --lemniscate-period "${T}" \
           --payload-mass "${M}" \
           --wind-speed ${WIND_MPS} --wind-seed ${WIND_SEED} \
           --fault-0-quad 0 --fault-0-time "${T1}" \
           --fault-1-quad 2 --fault-1-time "${T2}" \
           > "${outdir}/run.log" 2>&1
    local w1; w1=$(date +%s)
    echo "   rc=$? elapsed=$((w1-w0))s"
}

for T in ${PERIODS}; do
    for M in ${MASSES}; do
        run_one "${T}" "${M}"
    done
done
echo "slack-domain phase diagram complete"
