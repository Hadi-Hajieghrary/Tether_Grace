#!/usr/bin/env python3
"""Run all 3 MC scenarios in parallel with fixed fault times and 10 cores total."""
import subprocess, json, sys, numpy as np
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor, as_completed

ROOT = Path("/workspaces/Tether_Grace")
EXE = ROOT / "build" / "full_drake_fault_runner"
MC_ROOT = ROOT / "outputs" / "monte_carlo"
N_MC = 30
N_PARALLEL = 10

sys.path.insert(0, str(ROOT / "experiments" / "python"))
from run_fault_schedule_batch import build_payload_reference, load_csv_matrix

SCENARIOS = {
    "mc_n3_1fault": {
        "num_quads": 3, "nom_cables": [1.00, 1.08, 1.16],
        "fault_cables": "0", "fault_times": "7.000",
        "fts_list": [7.0],
    },
    "mc_n7_3fault": {
        "num_quads": 7, "nom_cables": [0.98, 1.01, 1.04, 1.07, 1.10, 1.13, 1.16],
        "fault_cables": "1,3,5", "fault_times": "7.000,12.000,14.000",
        "fts_list": [7.0, 12.0, 14.0],
    },
    "mc_n3_eskf_wind": {
        "num_quads": 3, "nom_cables": [1.00, 1.08, 1.16],
        "fault_cables": "0", "fault_times": "7.000",
        "fts_list": [7.0],
        "extra_args": ["--enable-wind", "--wind-mean", "2.0", "0.0", "0.0",
                        "--wind-sigma", "1.0", "--enable-eskf",
                        "--gps-noise", "0.02", "--baro-noise", "0.3"],
    },
}

def run_one(args):
    scen_name, i, scen = args
    rng = np.random.default_rng(seed=42 + i * 1000)
    cables = [float(np.clip(L + rng.normal(0, 0.05*L), 0.5, 2.0)) for L in scen["nom_cables"]]
    seed = int(rng.integers(1, 100000))

    out = MC_ROOT / scen_name / f"run_{i:03d}"
    out.mkdir(parents=True, exist_ok=True)
    cmd = [str(EXE), "--num-quads", str(scen["num_quads"]), "--duration", "30.0",
           "--output-dir", str(out),
           "--cable-lengths", ",".join(f"{v:.3f}" for v in cables),
           "--fault-cables", scen["fault_cables"],
           "--fault-times", scen["fault_times"],
           "--headless", "--seed", str(seed)]
    cmd += scen.get("extra_args", [])
    try:
        subprocess.run(cmd, check=True, cwd=ROOT, capture_output=True, timeout=1800)
        mf = json.loads((out / "run_manifest.json").read_text())
        h, traj = load_csv_matrix(Path(mf["log_dir"]) / "trajectories.csv")
        tm = traj[:, 0]
        xyz = np.column_stack([traj[:,h.index("load_x")], traj[:,h.index("load_y")], traj[:,h.index("load_z")]])
        ff = min(scen["fts_list"])
        ref, _, _ = build_payload_reference(tm, 30.0, xyz[:,2], ff)
        err = np.linalg.norm(xyz - ref, axis=1)
        post = tm >= ff
        return {"scen": scen_name, "run": i, "ok": True,
                "rmse": float(np.sqrt(np.mean(err**2)))*100,
                "peak": float(np.max(err[post]))*100 if np.any(post) else 0}
    except Exception as e:
        return {"scen": scen_name, "run": i, "ok": False, "error": str(e)[:100]}

if __name__ == "__main__":
    MC_ROOT.mkdir(parents=True, exist_ok=True)
    tasks = []
    for scen_name, scen in SCENARIOS.items():
        (MC_ROOT / scen_name).mkdir(parents=True, exist_ok=True)
        for i in range(N_MC):
            tasks.append((scen_name, i, scen))

    print(f"Running {len(tasks)} total MC sims ({N_MC} × {len(SCENARIOS)} scenarios) with {N_PARALLEL} workers...")
    results = []
    with ProcessPoolExecutor(max_workers=N_PARALLEL) as pool:
        futures = {pool.submit(run_one, t): t for t in tasks}
        for future in as_completed(futures):
            r = future.result()
            results.append(r)
            done = sum(1 for x in results if x["ok"])
            total = len(tasks)
            if r["ok"]:
                print(f"  [{done}/{total}] {r['scen']} run_{r['run']:03d}: RMSE={r['rmse']:.1f}", flush=True)
            else:
                print(f"  [FAIL] {r['scen']} run_{r['run']:03d}: {r.get('error','')[:60]}", flush=True)

    # Summarize per scenario
    for scen_name in SCENARIOS:
        ok = [r for r in results if r["scen"] == scen_name and r["ok"]]
        if not ok: continue
        rmse = [r["rmse"] for r in ok]
        peak = [r["peak"] for r in ok]
        print(f"\n=== {scen_name} ({len(ok)}/{N_MC}) ===")
        print(f"RMSE: {np.mean(rmse):.2f} ± {np.std(rmse):.2f}")
        print(f"Peak: {np.mean(peak):.2f} ± {np.std(peak):.2f} (95th: {np.percentile(peak,95):.2f})")

    (MC_ROOT / "mc_all_results.json").write_text(json.dumps(results, indent=2))
