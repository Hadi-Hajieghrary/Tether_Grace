#!/usr/bin/env bash
# WP4.4-extended: Four-axis robustness check at F=2.
#
# Single-seed, single-perturbation per axis around the canonical
# V4 operating point. Closes Reviewer 3's four-axis robustness
# claim with at least one off-nominal point on each axis.
#
# Axes:
#   m_L:           5, 10, 15 kg                       (mass)
#   k_s:           12500, 25000, 50000 N/m            (stiffness)
#   c_drag (proxy via wind speed):  0, 4, 8 m/s        (drag)
#   wind seed:      42, 100, 200                       (wind realisation)
#
# Output: output/wp4_robustness/<axis>_<level>/

set -u

EXE=/workspaces/Tether_Grace/Research/cpp/build/decentralized_fault_aware_sim
ROOT=/workspaces/Tether_Grace/output/wp4_robustness
mkdir -p "${ROOT}"

DURATION=30
TRAJ=lemniscate3d
NUM_QUADS=5
T1=12.0; T2=17.0
DEFAULT_M=10
DEFAULT_K=25000
DEFAULT_W=4
DEFAULT_SEED=42

run_one() {
    local axis="$1"; local level="$2"
    local m="$3"; local k="$4"; local w="$5"; local s="$6"
    local tag="${axis}_${level}"
    local outdir="${ROOT}/${tag}"
    mkdir -p "${outdir}"
    echo "=== ${tag}  (m=${m}kg, k=${k}N/m, w=${w}m/s, seed=${s})"
    local w0; w0=$(date +%s)
    "$EXE" --output-dir "${outdir}" --scenario "${tag}" \
           --num-quads ${NUM_QUADS} --duration ${DURATION} \
           --trajectory ${TRAJ} \
           --payload-mass "${m}" \
           --rope-stiffness "${k}" \
           --wind-speed "${w}" --wind-seed "${s}" \
           --fault-0-quad 0 --fault-0-time "${T1}" \
           --fault-1-quad 2 --fault-1-time "${T2}" \
           > "${outdir}/run.log" 2>&1
    local w1; w1=$(date +%s)
    echo "   rc=$? elapsed=$((w1-w0))s"
}

# Mass axis (single off-nominal each side; canonical is V4)
run_one mass low  5  ${DEFAULT_K} ${DEFAULT_W} ${DEFAULT_SEED}
run_one mass high 15 ${DEFAULT_K} ${DEFAULT_W} ${DEFAULT_SEED}

# Stiffness axis (half and double)
run_one stiff low  ${DEFAULT_M} 12500 ${DEFAULT_W} ${DEFAULT_SEED}
run_one stiff high ${DEFAULT_M} 50000 ${DEFAULT_W} ${DEFAULT_SEED}

# Wind / drag-proxy axis (no-wind reference and double-wind stress)
run_one wind low  ${DEFAULT_M} ${DEFAULT_K} 0 ${DEFAULT_SEED}
run_one wind high ${DEFAULT_M} ${DEFAULT_K} 8 ${DEFAULT_SEED}

# Wind realisation axis (different Dryden seeds at canonical 4 m/s)
run_one seed alt1 ${DEFAULT_M} ${DEFAULT_K} ${DEFAULT_W} 100
run_one seed alt2 ${DEFAULT_M} ${DEFAULT_K} ${DEFAULT_W} 200

echo "WP4 four-axis robustness sweep complete"
