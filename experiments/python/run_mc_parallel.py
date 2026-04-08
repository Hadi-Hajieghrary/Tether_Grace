#!/usr/bin/env python3
"""Parallel Monte Carlo: launch N_PARALLEL Drake sims simultaneously."""
import subprocess, json, sys, os, numpy as np
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor, as_completed

ROOT = Path("/workspaces/Tether_Grace")
EXE = ROOT / "build" / "full_drake_fault_runner"
MC_DIR = ROOT / "outputs" / "monte_carlo" / "mc_n7_3fault"
N_MC = 30
N_PARALLEL = 6  # 6 cores for N=7 (each needs more CPU time)

sys.path.insert(0, str(ROOT / "experiments" / "python"))
from run_fault_schedule_batch import build_payload_reference, load_csv_matrix

def run_one(i):
    rng = np.random.default_rng(seed=42 + i * 1000)
    # Randomize cable lengths (±5%)
    nom = [0.98, 1.01, 1.04, 1.07, 1.10, 1.13, 1.16]
    cables = [float(np.clip(L + rng.normal(0, 0.05*L), 0.5, 2.0)) for L in nom]
    # Fixed fault times — match the deterministic canonical scenario exactly
    fts = [7.0, 12.0, 14.0]
    seed = int(rng.integers(1, 100000))

    out = MC_DIR / f"run_{i:03d}"
    out.mkdir(parents=True, exist_ok=True)
    cmd = [str(EXE), "--num-quads", "7", "--duration", "30.0",
           "--output-dir", str(out),
           "--cable-lengths", ",".join(f"{v:.3f}" for v in cables),
           "--fault-cables", "1,3,5",
           "--fault-times", "7.000,12.000,14.000",
           "--headless", "--seed", str(seed)]
    try:
        subprocess.run(cmd, check=True, cwd=ROOT, capture_output=True, timeout=3600)
        # Extract metrics
        mf = json.loads((out / "run_manifest.json").read_text())
        h, traj = load_csv_matrix(Path(mf["log_dir"]) / "trajectories.csv")
        tm = traj[:, 0]
        xyz = np.column_stack([traj[:,h.index("load_x")], traj[:,h.index("load_y")], traj[:,h.index("load_z")]])
        ff = min(fts)
        ref, _, _ = build_payload_reference(tm, 30.0, xyz[:,2], ff)
        err = np.linalg.norm(xyz - ref, axis=1)
        post = tm >= ff
        return {"run": i, "rmse": float(np.sqrt(np.mean(err**2)))*100,
                "peak": float(np.max(err[post]))*100 if np.any(post) else 0,
                "ok": True}
    except Exception as e:
        return {"run": i, "rmse": -1, "peak": -1, "ok": False, "error": str(e)}

if __name__ == "__main__":
    MC_DIR.mkdir(parents=True, exist_ok=True)
    print(f"Running {N_MC} MC sims with {N_PARALLEL} parallel workers...")
    results = []
    with ProcessPoolExecutor(max_workers=N_PARALLEL) as pool:
        futures = {pool.submit(run_one, i): i for i in range(N_MC)}
        for future in as_completed(futures):
            r = future.result()
            results.append(r)
            done = sum(1 for x in results if x["ok"])
            if r["ok"]:
                print(f"  [{done}/{N_MC}] run_{r['run']:03d}: RMSE={r['rmse']:.1f} cm")
            else:
                print(f"  [{done}/{N_MC}] run_{r['run']:03d}: FAILED - {r.get('error','')[:80]}")

    ok = [r for r in results if r["ok"]]
    rmse = [r["rmse"] for r in ok]
    peak = [r["peak"] for r in ok]
    print(f"\n=== RESULTS ({len(ok)}/{N_MC} successful) ===")
    print(f"RMSE: {np.mean(rmse):.2f} ± {np.std(rmse):.2f} cm")
    print(f"Peak: {np.mean(peak):.2f} ± {np.std(peak):.2f} cm (95th: {np.percentile(peak,95):.2f})")

    (MC_DIR / "mc_results.json").write_text(json.dumps(results, indent=2))
