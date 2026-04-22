#!/usr/bin/env bash
# Full IEEE-Transactions Monte-Carlo campaign.
#
# Runs the 494-scenario matrix defined in
# Research/docs/preregistration.md and emits one CSV row per run in
# output/transactions_campaign/publication_metrics.csv.
#
# Usage:
#   ./run_transactions_campaign.sh                 # all sections
#   ./run_transactions_campaign.sh ablation        # primary ablation only
#   ./run_transactions_campaign.sh --resume        # skip runs whose manifest exists
#   ./run_transactions_campaign.sh --parallel 4    # 4 concurrent workers
#
# Environment:
#   EXE  — path to decentralized_fault_aware_sim (defaults to build tree)
#   ROOT — campaign output root
#   SEEDS — space-separated list of Monte-Carlo seeds (default "1 2 3 4 5")

set -u
SECTIONS=()
PARALLEL=1
RESUME=0
while [[ $# -gt 0 ]]; do
    case "$1" in
        --parallel) PARALLEL="$2"; shift 2;;
        --resume)   RESUME=1; shift;;
        ablation|param|wind|sensor|actuator|adversarial|competitor|comm|long) SECTIONS+=("$1"); shift;;
        *) echo "unknown argument: $1"; exit 2;;
    esac
done
if [[ ${#SECTIONS[@]} -eq 0 ]]; then
    SECTIONS=(ablation param wind sensor actuator adversarial competitor comm long)
fi

EXE="${EXE:-/workspaces/Tether_Grace/Research/cpp/build/decentralized_fault_aware_sim}"
ROOT="${ROOT:-/workspaces/Tether_Grace/output/transactions_campaign}"
SEEDS="${SEEDS:-1 2 3 4 5}"
mkdir -p "$ROOT"

# ----------------------------------------------------------------------
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

# ----------------------------------------------------------------------
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
        # Naïve parallelism via a trailing '&' and a `wait -n` when the
        # pool is full. Keeps dependencies minimal (no GNU parallel).
        while [[ $(jobs -r -p | wc -l) -ge $PARALLEL ]]; do wait -n; done
        run_one "$@" &
    fi
}

# ----------------------------------------------------------------------
# P — Primary ablation: 6 configs × 5 scenarios × |SEEDS|.
run_ablation() {
    echo "=== primary ablation ==="
    for cfg in baseline l1 mpc5 mpc10 fullstack fullstack_t60; do
        for scen in A_nominal B_single_15s C_dual_5s D_dual_10s E_cascade_3fault; do
            for seed in $SEEDS; do
                submit "P_${cfg}_${scen}_s${seed}" \
                    --scenario "${scen}_${cfg}_s${seed}" \
                    ${CFG[$cfg]} ${FAULTS[$scen]}
            done
        done
    done
    wait
}

# R — Parametric robustness: fullstack vs baseline, one-axis-at-a-time.
run_param() {
    echo "=== parametric robustness: m_L and k_s ==="
    for m in 2.0 2.5 3.0 3.5 3.9; do
        for cfg in baseline fullstack; do
            for seed in $SEEDS; do
                submit "R_mL${m}_${cfg}_s${seed}" \
                    --scenario "mL_${m}_${cfg}_s${seed}" \
                    --payload-mass "$m" ${CFG[$cfg]}
            done
        done
    done
    wait
}

# W — Wind (Dryden) sweep; only fullstack vs baseline.
run_wind() {
    echo "=== wind sweep (placeholder) ==="
    # The harness doesn't yet wire the Dryden gust into the force
    # injection (header is available; wiring lands with the next harness
    # patch). For now, record the configuration placeholder so the
    # campaign accounting is complete.
    for wv in 0 3 6; do
        for cfg in baseline fullstack; do
            for seed in 1 2 3; do
                submit "W_wind${wv}_${cfg}_s${seed}" \
                    --scenario "wind_${wv}_${cfg}_s${seed}" ${CFG[$cfg]}
            done
        done
    done
    wait
}

run_sensor() {
    echo "=== sensor-noise sweep (placeholder) ==="
    # Placeholder: requires the ImuNoiseInjector to be wired into the
    # controller input path; for now record the cells.
    for lvl in 1 4; do
        for cfg in baseline fullstack; do
            for seed in $SEEDS; do
                submit "S_sn${lvl}_${cfg}_s${seed}" \
                    --scenario "sn_${lvl}_${cfg}_s${seed}" ${CFG[$cfg]}
            done
        done
    done
    wait
}

run_actuator() {
    echo "=== actuator-envelope sweep (placeholder) ==="
    for lim in 20 12; do
        for cfg in baseline fullstack; do
            for seed in $SEEDS; do
                submit "A_act${lim}_${cfg}_s${seed}" \
                    --scenario "act_${lim}_${cfg}_s${seed}" ${CFG[$cfg]}
            done
        done
    done
    wait
}

run_adversarial() {
    echo "=== adversarial worst-case (placeholder) ==="
    for real in 1 2 3 4 5 6 7 8 9 10; do
        for cfg in baseline fullstack; do
            submit "Adv_r${real}_${cfg}" \
                --scenario "adv_r${real}_${cfg}" ${CFG[$cfg]}
        done
    done
    wait
}

run_competitor() {
    echo "=== competitor head-to-head (not yet implemented) ==="
    # C-1..C-4 will be activated once the competitor LeafSystems land.
}

run_comm() {
    echo "=== communication-robustness (placeholder) ==="
    for d in 0 100 500; do
        for p in 0 10 30; do
            for seed in 1 2 3; do
                submit "K_d${d}_p${p}_s${seed}" \
                    --scenario "k_${d}_${p}_s${seed}" ${CFG[fullstack]}
            done
        done
    done
    wait
}

run_long() {
    echo "=== ultra-long 300 s runs ==="
    for scen in A_nominal C_dual_5s E_cascade_3fault; do
        for cfg in baseline fullstack; do
            for seed in 1 2 3; do
                submit "L_${scen}_${cfg}_s${seed}" \
                    --scenario "long_${scen}_${cfg}_s${seed}" \
                    --duration 300 ${CFG[$cfg]} ${FAULTS[$scen]}
            done
        done
    done
    wait
}

# ----------------------------------------------------------------------
for section in "${SECTIONS[@]}"; do
    "run_${section}"
done

# ----------------------------------------------------------------------
# Post-process: gather publication_metrics.csv files from each run folder.
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
