#!/usr/bin/env bash
# P2-E: wind-magnitude x seed sweep.
#
# Converts "works on this one seed" to a statistical statement.
# Sweep axes:
#   mean wind  in {0, 4, 6, 8, 10} m/s
#   wind seed  in {42, 43, 44}
#
# Scenario: V4's dual-fault schedule (12, 17 s) at 10 kg payload on
# lemniscate3D.  Baseline controller only — the extension stack is
# evaluated elsewhere.
#
# Runs 5 * 3 = 15 sims.  Duration 30 s.  Wall time: ~15 * 18 = ~4.5 h.
#
# Output: output/p2e_wind_seed_sweep/<variant>/

set -u

EXE=/workspaces/Tether_Grace/Research/cpp/build/decentralized_fault_aware_sim
ROOT=/workspaces/Tether_Grace/output/p2e_wind_seed_sweep
mkdir -p "${ROOT}"

PAYLOAD_KG=10
DURATION=30
TRAJ=lemniscate3d
NUM_QUADS=5

WINDS="0 4 6 8 10"
SEEDS="42 43 44"

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
           --fault-0-quad 0 --fault-0-time 12 \
           --fault-1-quad 2 --fault-1-time 17 \
           "$@" > "${outdir}/run.log" 2>&1
    local rc=$?
    local t1; t1=$(date +%s)
    echo "--- ${tag}  rc=${rc}  elapsed=$((t1-t0)) s"
    if [[ ${rc} -ne 0 ]]; then tail -30 "${outdir}/run.log"; fi
}

for W in ${WINDS}; do
    for S in ${SEEDS}; do
        run_one "p2e_w${W}_s${S}" \
            --wind-speed ${W} --wind-seed ${S}
    done
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
