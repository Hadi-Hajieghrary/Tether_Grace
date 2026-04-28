#!/usr/bin/env bash
# Dwell-gap sweep (§VI.E of IEEE paper).
#
# V4 dual-fault schedule with inter-fault gap Δt/τ_pend in
# {0.5, 0.75, 1.0, 1.25, 1.5, 2.0, 3.0} at the canonical
# τ_pend = 2.24 s. Seven deterministic runs at seed 42.
#
# Output: output/dwell_sweep/<tag>/

set -u

EXE=/workspaces/Tether_Grace/Research/cpp/build/decentralized_fault_aware_sim
ROOT=/workspaces/Tether_Grace/output/dwell_sweep
mkdir -p "${ROOT}"

PAYLOAD_KG=10
WIND_MPS=4.0
WIND_SEED=42
DURATION=30
TRAJ=lemniscate3d
NUM_QUADS=5

# τ_pend = 2π √(L/g) = 2.24 s → gaps in seconds (first fault at 12s)
GAPS="1.12 1.68 2.24 2.80 3.36 4.48 6.72"

run_one() {
    local tag="$1"
    local gap="$2"
    local t1=12.0
    local t2=$(python3 -c "print(${t1} + ${gap})")
    local outdir="${ROOT}/${tag}"
    mkdir -p "${outdir}"
    echo "=== ${tag} (gap=${gap}s, t1=${t1}, t2=${t2})"
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

# τ_d / τ_pend : 0.5, 0.75, 1.0, 1.25, 1.5, 2.0, 3.0
# τ_pend = 2.24, so τ_d values = 1.12, 1.68, 2.24, 2.80, 3.36, 4.48, 6.72
run_one dwell_0p50 1.12
run_one dwell_0p75 1.68
run_one dwell_1p00 2.24
run_one dwell_1p25 2.80
run_one dwell_1p50 3.36
run_one dwell_2p00 4.48
run_one dwell_3p00 6.72

echo "dwell sweep complete"
