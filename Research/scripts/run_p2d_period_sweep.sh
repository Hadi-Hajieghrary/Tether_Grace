#!/usr/bin/env bash
# P2-D: lemniscate-period sweep for the reshape supervisor.
#
# Centripetal load on the rope scales as 1/T^2, so shorter periods
# give higher hover-equilibrium tensions.  The reshape supervisor's
# theoretical benefit (theory_reshaping_extension.md §C.3) grows in
# this regime.  Campaign:
#
#    T in {8, 10, 12} s, each with {baseline, baseline+reshape}
#
# Fault schedule is V4's (12 s, 17 s) so that each run exercises a
# post-fault regime.  Output: output/p2d_period_sweep/<variant>/
# Wall time: 6 runs * 18 min = ~2 wall-hours.

set -u

EXE=/workspaces/Tether_Grace/Research/cpp/build/decentralized_fault_aware_sim
ROOT=/workspaces/Tether_Grace/output/p2d_period_sweep
mkdir -p "${ROOT}"

PAYLOAD_KG=10
WIND_MPS=4.0
WIND_SEED=42
DURATION=30
TRAJ=lemniscate3d
NUM_QUADS=5

PERIODS="8 10 12"

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
    if [[ ${rc} -ne 0 ]]; then tail -30 "${outdir}/run.log"; fi
}

for T in ${PERIODS}; do
    run_one "p2d_T${T}_noreshape" \
        --lemniscate-period ${T}
    run_one "p2d_T${T}_reshape" \
        --lemniscate-period ${T} --reshaping-enabled
done

echo "================================================================"
echo "=== Summary"
echo "================================================================"
for v in "${ROOT}"/*/; do
    csv="${v}scenario_$(basename "$v").csv"
    if [[ -f "$csv" ]]; then
        printf "  %-28s rows=%6d\n" "$(basename "$v")" "$(wc -l < "$csv")"
    fi
done
