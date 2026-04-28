#!/usr/bin/env bash
# Tier-3J: Stratified Random Fault-Schedule Sampling.
#
# Sample admissible fault schedules to demonstrate that V1--V6
# are not hand-picked. Each schedule satisfies:
#   - t_1 in [10, 18]s
#   - delta_t = t_2 - t_1 in [tau_pend=2.24, 8] s
#   - i_1 != i_2 with i_1, i_2 in {0..4}
#
# Paper recommendation: 100 runs. Session budget: 20 runs (~7.5h).
#
# Output: output/stratified_faults/seed_NNN/

set -u

EXE=/workspaces/Tether_Grace/Research/cpp/build/decentralized_fault_aware_sim
ROOT=/workspaces/Tether_Grace/output/stratified_faults
mkdir -p "${ROOT}"

PAYLOAD_KG=10
WIND_MPS=4.0
DURATION=30
TRAJ=lemniscate3d
NUM_QUADS=5
N_SAMPLES=20

# Generate the schedule manifest deterministically with python.
MANIFEST="${ROOT}/manifest.csv"
python3 << EOF > "${MANIFEST}"
import random
random.seed(2026)
print("seed,t1,t2,i1,i2")
for k in range(${N_SAMPLES}):
    rng = random.Random(1000 + k)
    t1 = rng.uniform(10.0, 18.0)
    delta_t = rng.uniform(2.24, 8.0)
    t2 = min(t1 + delta_t, 28.0)  # leave 2s post-fault tail
    i1 = rng.randint(0, 4)
    while True:
        i2 = rng.randint(0, 4)
        if i2 != i1:
            break
    print(f"{1000+k},{t1:.3f},{t2:.3f},{i1},{i2}")
EOF
echo "manifest written to ${MANIFEST}"

run_one() {
    local seed="$1"
    local t1="$2"
    local t2="$3"
    local i1="$4"
    local i2="$5"
    local tag="seed_${seed}"
    local outdir="${ROOT}/${tag}"
    mkdir -p "${outdir}"
    echo "=== ${tag} (i1=${i1}@${t1}, i2=${i2}@${t2})"
    local w0; w0=$(date +%s)
    "$EXE" --output-dir "${outdir}" --scenario "${tag}" \
           --num-quads ${NUM_QUADS} --duration ${DURATION} \
           --trajectory ${TRAJ} --payload-mass ${PAYLOAD_KG} \
           --wind-speed ${WIND_MPS} --wind-seed "${seed}" \
           --fault-0-quad "${i1}" --fault-0-time "${t1}" \
           --fault-1-quad "${i2}" --fault-1-time "${t2}" \
           > "${outdir}/run.log" 2>&1
    local w1; w1=$(date +%s)
    echo "   rc=$? elapsed=$((w1-w0))s"
}

# Skip header, iterate over manifest.
tail -n +2 "${MANIFEST}" | while IFS=, read -r seed t1 t2 i1 i2; do
    run_one "${seed}" "${t1}" "${t2}" "${i1}" "${i2}"
done

echo "stratified fault sweep complete"
