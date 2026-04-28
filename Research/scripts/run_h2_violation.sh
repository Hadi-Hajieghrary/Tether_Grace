#!/usr/bin/env bash
# H2-boundary violation run (§IV & §VI of IEEE paper).
#
# Forces sub-threshold fault spacing τ_d = 0.5·τ_pend to expose
# the Lyapunov V growing monotonically across faults — the
# negative evidence complement to the H2 dwell hypothesis. The
# baseline theorem predicts ρ<1 contraction is lost when
# τ_d < τ_pend; this run provides an explicit empirical
# counterexample illustrating why H2 is load-bearing.
#
# Output: output/h2_violation/sub_threshold/

set -u

EXE=/workspaces/Tether_Grace/Research/cpp/build/decentralized_fault_aware_sim
ROOT=/workspaces/Tether_Grace/output/h2_violation
mkdir -p "${ROOT}"

PAYLOAD_KG=10
WIND_MPS=4.0
WIND_SEED=42
DURATION=30
TRAJ=lemniscate3d
NUM_QUADS=5

# τ_pend = 2.24 s → τ_d = 0.5·τ_pend = 1.12 s (below H2 threshold)
T1=12.0
GAP=1.12
T2=$(python3 -c "print(${T1} + ${GAP})")

tag="h2_sub_threshold"
outdir="${ROOT}/${tag}"
mkdir -p "${outdir}"
echo "=== ${tag} (gap=${GAP}s, t1=${T1}, t2=${T2})"
w0=$(date +%s)
"$EXE" --output-dir "${outdir}" --scenario "${tag}" \
       --num-quads ${NUM_QUADS} --duration ${DURATION} \
       --trajectory ${TRAJ} --payload-mass ${PAYLOAD_KG} \
       --wind-speed ${WIND_MPS} --wind-seed ${WIND_SEED} \
       --fault-0-quad 0 --fault-0-time "${T1}" \
       --fault-1-quad 2 --fault-1-time "${T2}" \
       > "${outdir}/run.log" 2>&1
w1=$(date +%s)
echo "   rc=$? elapsed=$((w1-w0))s"
echo "h2-violation run complete"
