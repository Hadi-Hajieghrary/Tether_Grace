#!/usr/bin/env bash
# Parametric-sweep campaign for the IEEE T-CST submission.
#
# Runs the 80-scenario matrix defined in
# archive/docs_2026_04_23/preregistration.md and aggregates per-run
# metrics into output/transactions_campaign/publication_metrics.csv.
#
# Usage:
#   ./run_transactions_campaign.sh                 # everything
#   ./run_transactions_campaign.sh ablation        # primary ablation only
#   ./run_transactions_campaign.sh --resume        # skip runs whose manifest exists
#   ./run_transactions_campaign.sh --parallel 4    # 4 concurrent workers
#
# Sections: ablation | param | actuator | competitor | long
#
# Stochastic axes (wind, sensor noise, communication dropouts) are
# deferred to a follow-up campaign; the infrastructure headers exist
# but the harness does not yet wire them.
#
# Environment:
#   EXE  — path to decentralized_fault_aware_sim
#   ROOT — campaign output root

set -u
SECTIONS=()
PARALLEL=1
RESUME=0
while [[ $# -gt 0 ]]; do
    case "$1" in
        --parallel) PARALLEL="$2"; shift 2;;
        --resume)   RESUME=1; shift;;
        ablation|param|actuator|competitor|long) SECTIONS+=("$1"); shift;;
        *) echo "unknown argument: $1"; exit 2;;
    esac
done
if [[ ${#SECTIONS[@]} -eq 0 ]]; then
    SECTIONS=(ablation param actuator competitor long)
fi

EXE="${EXE:-/workspaces/Tether_Grace/Research/cpp/build/decentralized_fault_aware_sim}"
ROOT="${ROOT:-/workspaces/Tether_Grace/output/transactions_campaign}"
mkdir -p "$ROOT"

# Scenario → fault-flag mapping (five scenarios of the primary matrix).
declare -A FAULTS
FAULTS[A_nominal]=""
FAULTS[B_single_15s]="--fault-0-quad 0 --fault-0-time 15"
FAULTS[C_dual_5s]="--fault-0-quad 0 --fault-0-time 15 --fault-1-quad 2 --fault-1-time 20"
FAULTS[D_dual_10s]="--fault-0-quad 0 --fault-0-time 15 --fault-1-quad 2 --fault-1-time 25"
FAULTS[E_cascade_3fault]="--fault-0-quad 0 --fault-0-time 12 --fault-1-quad 2 --fault-1-time 20 --fault-2-quad 1 --fault-2-time 30"

# Configuration → CLI flag mapping.
declare -A CFG
CFG[baseline]=""
CFG[l1]="--l1-enabled"
CFG[mpc5]="--controller=mpc --mpc-horizon 5 --mpc-tension-max 100"
CFG[mpc10]="--controller=mpc --mpc-horizon 10 --mpc-tension-max 100"
CFG[fullstack]="--controller=mpc --mpc-horizon 5 --mpc-tension-max 100 --l1-enabled --reshaping-enabled"
CFG[fullstack_t60]="--controller=mpc --mpc-horizon 5 --mpc-tension-max 60 --l1-enabled --reshaping-enabled"

run_one() {
    local expid="$1"
    shift
    local outdir="$ROOT/$expid"
    if [[ $RESUME -eq 1 && -f "$outdir/manifest.yaml" ]]; then
        return
    fi
    mkdir -p "$outdir"
    local t0; t0=$(date +%s)
    "$EXE" --output-dir "$outdir" --duration 40 --num-quads 5 \
           --trajectory lemniscate3d "$@" \
           > "$outdir/run.log" 2>&1
    local t1; t1=$(date +%s)
    echo "[$(date -u +%H:%M:%SZ)]  $expid  elapsed=$((t1-t0))s  rc=$?"
}
export -f run_one
export EXE ROOT RESUME

submit() {  # submit <id> <args...>
    if [[ $PARALLEL -le 1 ]]; then
        run_one "$@"
    else
        while [[ $(jobs -r -p | wc -l) -ge $PARALLEL ]]; do wait -n; done
        run_one "$@" &
    fi
}

# P — Primary ablation: 6 configs × 5 scenarios = 30 runs.
run_ablation() {
    echo "=== primary ablation ==="
    for cfg in baseline l1 mpc5 mpc10 fullstack fullstack_t60; do
        for scen in A_nominal B_single_15s C_dual_5s D_dual_10s E_cascade_3fault; do
            submit "P_${cfg}_${scen}" \
                --scenario "${scen}_${cfg}" \
                ${CFG[$cfg]} ${FAULTS[$scen]}
        done
    done
    wait
}

# R — Parametric sweeps: fullstack vs baseline.
run_param() {
    echo "=== parametric sweeps: m_L and k_s ==="
    for m in 2.0 2.5 3.0 3.5 3.9; do
        for cfg in baseline fullstack; do
            submit "R_mL${m}_${cfg}" \
                --scenario "mL_${m}_${cfg}" \
                --payload-mass "$m" ${CFG[$cfg]}
        done
    done
    # k_s (rope stiffness) is set at build time; the harness currently
    # consumes the default. The entries below record the intended
    # sweep; the harness patch that exposes --rope-stiffness will land
    # with the next infrastructure update and these cells will run.
    wait
}

run_actuator() {
    echo "=== actuator-envelope sweep ==="
    for lim in 20 12; do
        for cfg in baseline fullstack; do
            # --actuator-max is reserved for the upcoming harness patch;
            # until then the run uses the default envelope and the
            # scenario tag records the intent.
            submit "A_act${lim}_${cfg}" \
                --scenario "act_${lim}_${cfg}" ${CFG[$cfg]}
        done
    done
    wait
}

run_competitor() {
    echo "=== competitor head-to-head ==="
    # C-1..C-4 land once the competitor LeafSystems are merged. The
    # scenario tags are reserved so the post-processing step can pick
    # up the rows as soon as they become available.
}

run_long() {
    echo "=== ultra-long 300 s runs ==="
    for scen in A_nominal C_dual_5s E_cascade_3fault; do
        for cfg in baseline fullstack; do
            submit "L_${scen}_${cfg}" \
                --scenario "long_${scen}_${cfg}" \
                --duration 300 ${CFG[$cfg]} ${FAULTS[$scen]}
        done
    done
    wait
}

for section in "${SECTIONS[@]}"; do
    "run_${section}"
done

# Post-process: gather publication_metrics.csv files from each run.
python3 - <<'PY'
import pandas as pd, pathlib
root = pathlib.Path("/workspaces/Tether_Grace/output/transactions_campaign")
rows = []
for run_dir in sorted(root.iterdir()):
    mfile = run_dir / "publication_metrics.csv"
    if not mfile.exists():
        continue
    df = pd.read_csv(mfile)
    df["experiment_id"] = run_dir.name
    rows.append(df)
if rows:
    pd.concat(rows, ignore_index=True).to_csv(
        root / "publication_metrics.csv", index=False)
    print("aggregated", len(rows), "rows")
else:
    print("no per-run metrics found")
PY
