#!/usr/bin/env bash
# Tier-2F: L1 Adaptive Gain Stability Map.
#
# Sweep the L1 adaptive gain Gamma across a span that crosses the
# closed-form Euler-discretisation bound Gamma^* approx 4.75e4
# (Proposition 1, Section V). Test the L1 layer in its rescue-mode
# configuration: FF disabled (so L1 is the primary altitude
# compensator), small payload (3.9 kg actual vs 3.0 kg nominal,
# the P2-B mass-mismatch operating point), no faults, no wind.
#
# Decentralized: each drone runs its local L1 update with the
# given Gamma; no orchestrator, no shared state.
#
# Output: output/l1_gain_map/gamma_<value>/

set -u

EXE=/workspaces/Tether_Grace/Research/cpp/build/decentralized_fault_aware_sim
ROOT=/workspaces/Tether_Grace/output/l1_gain_map
mkdir -p "${ROOT}"

PAYLOAD_KG=3.9      # mass-mismatch operating point
NOM_KG=3.0          # controller nominal
WIND_MPS=0.0
WIND_SEED=42
DURATION=20
TRAJ=lemniscate3d
NUM_QUADS=5

# Gamma values in absolute units. Closed-form stability bound
# Gamma^* = 2 / (T_s p_{22}) ~ 4.75e4 (T_s = 5e-3 s, p_{22} from
# the discrete Lyapunov solution; see Proposition 1).
# Cover four orders of magnitude crossing the bound.
GAMMAS="500 2000 10000 30000 50000 80000"

run_one() {
    local gamma="$1"
    local tag="gamma_${gamma}"
    local outdir="${ROOT}/${tag}"
    mkdir -p "${outdir}"
    echo "=== ${tag}  (Gamma=${gamma}, payload=${PAYLOAD_KG} kg, nominal=${NOM_KG} kg)"
    local w0; w0=$(date +%s)
    "$EXE" --output-dir "${outdir}" --scenario "${tag}" \
           --num-quads ${NUM_QUADS} --duration ${DURATION} \
           --trajectory ${TRAJ} \
           --payload-mass ${PAYLOAD_KG} \
           --controller-nominal-mass ${NOM_KG} \
           --wind-speed ${WIND_MPS} --wind-seed ${WIND_SEED} \
           --l1-enabled --l1-gamma "${gamma}" \
           --disable-tension-ff \
           > "${outdir}/run.log" 2>&1
    local w1; w1=$(date +%s)
    echo "   rc=$? elapsed=$((w1-w0))s"
}

for g in ${GAMMAS}; do
    run_one "${g}"
done

echo "L1 gain-map sweep complete"
