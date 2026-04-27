#!/usr/bin/env bash
# Capability-demonstration campaign.
#
# Six-variant matrix on the five-drone lemniscate-3D reference with a
# 10 kg payload — chosen so the baseline PD/QP controller (whose
# gains were tuned around the default 3 kg reference load) tracks the
# trajectory cleanly without saturating its thrust envelope, while
# the load still demands genuine cooperation from the formation.
# After two cable severances the three surviving ropes each carry
# ≈ 33 N — well under the 100 N MPC ceiling. A hypothetical third
# severance would push the two remaining ropes past 49 N each and
# approach the 150 N actuator ceiling once dynamics are included,
# so three surviving drones is the practical safety floor.
#
#   V1 nominal_nowind        baseline QP, no faults, no wind
#   V2 nominal_wind          baseline QP, no faults, 4 m/s Dryden wind
#   V3 single_wind           baseline QP, single fault @ 12 s, wind
#   V4 dual_5s_wind          baseline QP, faults @ 12 s and 17 s, wind
#   V5 dual_10s_wind         baseline QP, faults @ 12 s and 22 s, wind
#   V6 dual_5s_fullstack     MPC + L1 + reshape, faults @ 12 s and 17 s, wind
#
# The sim is CPU-bound by the bead-chain integration (not throttled).
# On this host a single 30 s run takes ≈ 20 wall-minutes; the full
# six-variant campaign runs in ≈ 2 wall-hours.
#
# Output: output/capability_demo/<variant>/ with CSV + Meshcat replay.

set -u

EXE=/workspaces/Tether_Grace/Research/cpp/build/decentralized_fault_aware_sim
ROOT=/workspaces/Tether_Grace/output/capability_demo
mkdir -p "${ROOT}"

PAYLOAD_KG=10
WIND_MPS=4.0
WIND_SEED=42
DURATION=30
TRAJ=lemniscate3d
NUM_QUADS=5

run_variant() {
    local tag="$1"
    shift
    local outdir="${ROOT}/${tag}"
    mkdir -p "${outdir}"
    echo "================================================================"
    echo "=== ${tag}"
    echo "================================================================"
    local t0; t0=$(date +%s)
    "$EXE" --output-dir "${outdir}" --scenario "${tag}" \
           --num-quads ${NUM_QUADS} --duration ${DURATION} \
           --trajectory ${TRAJ} --payload-mass ${PAYLOAD_KG} \
           "$@" \
           > "${outdir}/run.log" 2>&1
    local rc=$?
    local t1; t1=$(date +%s)
    echo "--- ${tag}  rc=${rc}  elapsed=$((t1-t0)) s"
    if [[ ${rc} -ne 0 ]]; then
        echo "    FAILED — inspect ${outdir}/run.log"
        tail -30 "${outdir}/run.log"
    fi
}

# -------------------- Six variants --------------------

# V1: nominal, no wind
run_variant V1_nominal_nowind

# V2: nominal, wind
run_variant V2_nominal_wind \
    --wind-speed ${WIND_MPS} --wind-seed ${WIND_SEED}

# V3: single fault under wind
run_variant V3_single_wind \
    --wind-speed ${WIND_MPS} --wind-seed ${WIND_SEED} \
    --fault-0-quad 0 --fault-0-time 12

# V4: dual fault 5 s apart under wind
run_variant V4_dual_5s_wind \
    --wind-speed ${WIND_MPS} --wind-seed ${WIND_SEED} \
    --fault-0-quad 0 --fault-0-time 12 \
    --fault-1-quad 2 --fault-1-time 17

# V5: dual fault 10 s apart under wind
run_variant V5_dual_10s_wind \
    --wind-speed ${WIND_MPS} --wind-seed ${WIND_SEED} \
    --fault-0-quad 0 --fault-0-time 12 \
    --fault-1-quad 2 --fault-1-time 22

# V6: hardest scenario with the full stack
run_variant V6_dual_5s_fullstack \
    --wind-speed ${WIND_MPS} --wind-seed ${WIND_SEED} \
    --fault-0-quad 0 --fault-0-time 12 \
    --fault-1-quad 2 --fault-1-time 17 \
    --controller=mpc --mpc-horizon 5 --mpc-tension-max 100 \
    --l1-enabled --reshaping-enabled

echo "================================================================"
echo "=== Summary"
echo "================================================================"
for v in V1_nominal_nowind V2_nominal_wind V3_single_wind \
         V4_dual_5s_wind V5_dual_10s_wind V6_dual_5s_fullstack; do
    csv="${ROOT}/${v}/scenario_${v}.csv"
    if [[ -f "$csv" ]]; then
        printf "  %-26s rows=%6d  csv=%s\n" "$v" "$(wc -l < "$csv")" "$csv"
    else
        printf "  %-26s  MISSING\n" "$v"
    fi
done

echo
echo "Next: run Research/analysis/ieee/plot_capability_demo.py"
echo "      to produce the comparative figures from the six CSVs."
