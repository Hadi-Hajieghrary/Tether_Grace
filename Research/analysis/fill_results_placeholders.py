#!/usr/bin/env python3
"""
Fill numerical placeholders in the `results_section_draft.tex` template with
the actual metrics computed from the 5-drone campaign CSVs.

Metric convention
-----------------
All metrics are computed for t ≥ 0.5 s (startup transient excluded).
This matches the burn-in window applied by ``ieee_style.trim_start``
and used in all published tables in DECENTRALIZED_FAULT_AWARE_README.md
§6 and theory_decentralized_local_controller.md §9.

Usage:
    python3 fill_results_placeholders.py <campaign_root> <draft.tex> <out.tex>
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pandas as pd

# Startup-transient burn-in is the single source of truth defined in
# ieee_style; importing here prevents drift between figure-plotting
# pipeline and metric filler.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from ieee_style import BURN_IN_SECONDS as T_BURN_IN  # noqa: E402


def tracking_error(df):
    err = df[["ref_x", "ref_y", "ref_z"]].values - df[
        ["payload_x", "payload_y", "payload_z"]
    ].values
    return np.linalg.norm(err, axis=1)


def tension_mat(df, N):
    return np.column_stack([df[f"tension_{i}"].values for i in range(N)])


def thrust_mag(df, N):
    return np.stack([
        np.sqrt(df[f"fx_{i}"] ** 2 + df[f"fy_{i}"] ** 2 + df[f"fz_{i}"] ** 2).values
        for i in range(N)
    ], axis=1)


def num_drones(df):
    n = 0
    while f"quad{n}_x" in df.columns:
        n += 1
    return n


def scenario_metrics(df):
    N = num_drones(df)
    err = tracking_error(df)
    T = tension_mat(df, N)
    F = thrust_mag(df, N)
    sT = np.std(T, axis=1)
    qp_us = df.get("qp_solve_us_0")
    act_occ = np.nan
    if "act_ax_lo_0" in df.columns:
        cols = [f"act_{a}_{i}" for a in ["ax_lo", "ax_hi", "ay_lo", "ay_hi",
                                         "az_lo", "az_hi"]
                 for i in range(N)]
        act_occ = float(df[cols].mean().mean())
    return dict(
        RMS=float(np.sqrt(np.mean(err ** 2))),
        Peak=float(err.max()),
        PeakT=float(T.max()),
        PeakF=float(F.max()),
        Sig=float(np.sqrt(np.mean(sT ** 2))),
        qp_median_us=float(qp_us.median()) if qp_us is not None else float("nan"),
        act_occ=act_occ,
    )


def main():
    if len(sys.argv) < 4:
        print(__doc__); sys.exit(1)
    root = Path(sys.argv[1])
    draft = Path(sys.argv[2]).read_text()
    out = Path(sys.argv[3])

    data_root = root / "08_source_data"
    scenarios = {
        "A": "scenario_A_5drone_nominal.csv",
        "B": "scenario_B_5drone_single_fault.csv",
        "C": "scenario_C_5drone_dual_5sec.csv",
        "D": "scenario_D_5drone_dual_10sec.csv",
    }
    metrics = {}
    for key, fname in scenarios.items():
        path = data_root / fname
        if not path.exists():
            print(f"WARNING: {path} missing; leaving its macros as placeholders.")
            continue
        df = pd.read_csv(path)
        # Apply burn-in window: exclude startup transient (first T_BURN_IN s).
        # This must match ieee_style.trim_start(t_start=T_BURN_IN) used in all
        # publication plots and the README §6 table.
        if "time" in df.columns:
            df = df[df["time"] >= T_BURN_IN].reset_index(drop=True)
        else:
            import warnings
            warnings.warn(f"{fname}: no 'time' column found; burn-in not applied.")
        metrics[key] = scenario_metrics(df)
        print(f"{key}: {metrics[key]}")

    def fmt(x, digits=2):
        if isinstance(x, float) and np.isfinite(x):
            return f"{x:.{digits}f}"
        return "--"

    # Fill each \NEW* macro with the computed value.
    for key, m in metrics.items():
        draft = draft.replace(
            "\\NEWRms" + key + "}", "\\NEW" + "Rms" + key + "}"  # dummy noop
        )
        # Now replace the macro *definition* so that the body shows the value.
        for (name, val) in [
            ("Rms",   m["RMS"]),
            ("Peak",  m["Peak"]),
            ("PeakT", m["PeakT"]),
            ("PeakF", m["PeakF"]),
            ("Sig",   m["Sig"]),
        ]:
            macro = f"\\newcommand{{\\NEW{name}{key}}}{{\\ensuremath{{\\langle\\!\\langle"
            if macro in draft:
                idx = draft.find(macro)
                end = draft.find("}", idx + len(macro)) + 2  # close \ensuremath + \newcommand
                newline = (f"\\newcommand{{\\NEW{name}{key}}}{{{fmt(val)}}}\n")
                # Replace the entire definition line
                line_start = draft.rfind("\n", 0, idx) + 1
                line_end = draft.find("\n", idx) + 1
                draft = draft[:line_start] + newline + draft[line_end:]

    # Compute peak-tension ratio C/D for \NEWGapRatio.
    # Use a lambda for the regex replacement so the replacement string is
    # not reinterpreted (regex re.sub treats backslash escapes in the
    # replacement specially; using a lambda bypasses that).
    import re
    if "C" in metrics and "D" in metrics:
        ratio = metrics["C"]["PeakT"] / max(metrics["D"]["PeakT"], 1e-9)
        replacement = f"\\newcommand{{\\NEWGapRatio}}{{{ratio:.2f}}}\n"
        draft = re.sub(
            r"\\newcommand\{\\NEWGapRatio\}\{[^\n]+\}\n",
            lambda m: replacement,
            draft)
    # QP solve time + active occupancy from scenario A.
    if "A" in metrics:
        qp_us = metrics["A"]["qp_median_us"]
        if np.isfinite(qp_us):
            replacement_qp = f"\\newcommand{{\\NEWQPsolve}}{{{qp_us:.0f}}}\n"
            draft = re.sub(r"\\newcommand\{\\NEWQPsolve\}\{[^\n]+\}\n",
                           lambda m: replacement_qp, draft)
        act_occ = metrics["A"]["act_occ"]
        if np.isfinite(act_occ):
            replacement_act = (
                f"\\newcommand{{\\NEWQPactive}}{{{act_occ:.3f}}}\n")
            draft = re.sub(r"\\newcommand\{\\NEWQPactive\}\{[^\n]+\}\n",
                           lambda m: replacement_act, draft)

    out.write_text(draft)
    print(f"Filled draft written to {out}")


if __name__ == "__main__":
    main()
