#!/usr/bin/env bash
# P2-B: payload-mass mismatch sweep for the L1 adaptive layer.
#
# Design: run V1_nominal_nowind (no faults, no wind) at 4 actual
# payload masses while the controller's nominal is fixed at 3.0 kg.
# Three modes per mass:
#   (A) baseline with T_ff  — T_ff = measured_tension reacts to
#       actual mass, so tracking should be insensitive to mass mismatch.
#   (B) baseline with T_ff  DISABLED — mass mismatch enters as an
#       altitude-channel matched disturbance.
#   (C) mode-(B) + L1 augmentation — expected to recover tracking.
#
# Runs 4 * 3 = 12 sims.  Short duration (12 s) because we only need
# post-pickup steady-state tracking to measure the mismatch effect.
# Wall time: ~12 * 7 min = ~90 minutes.
#
# Output: output/p2b_mass_mismatch/<variant>/

set -u

EXE=/workspaces/Tether_Grace/Research/cpp/build/decentralized_fault_aware_sim
ROOT=/workspaces/Tether_Grace/output/p2b_mass_mismatch
mkdir -p "${ROOT}"

WIND_MPS=0.0
DURATION=12
TRAJ=lemniscate3d
NUM_QUADS=5

# Controller's nominal belief is held fixed at 3.0 kg throughout.
NOMINAL=3.0

MASSES="2.5 3.0 3.5 3.9"

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
           --trajectory ${TRAJ} \
           --controller-nominal-mass ${NOMINAL} \
           --wind-speed ${WIND_MPS} \
           "$@" > "${outdir}/run.log" 2>&1
    local rc=$?
    local t1; t1=$(date +%s)
    echo "--- ${tag}  rc=${rc}  elapsed=$((t1-t0)) s"
    if [[ ${rc} -ne 0 ]]; then tail -30 "${outdir}/run.log"; fi
}

for M in ${MASSES}; do
    M_ul=$(echo "${M}" | tr '.' 'p')   # 2p5, 3p0, ...
    run_one "p2b_m${M_ul}_ff_on" \
        --payload-mass ${M}
    run_one "p2b_m${M_ul}_ff_off" \
        --payload-mass ${M} --disable-tension-ff
    run_one "p2b_m${M_ul}_ff_off_l1" \
        --payload-mass ${M} --disable-tension-ff --l1-enabled
done

echo "================================================================"
echo "=== Summary"
echo "================================================================"
for v in "${ROOT}"/*/; do
    csv="${v}scenario_$(basename "$v").csv"
    if [[ -f "$csv" ]]; then
        printf "  %-32s rows=%6d\n" "$(basename "$v")" "$(wc -l < "$csv")"
    fi
done
