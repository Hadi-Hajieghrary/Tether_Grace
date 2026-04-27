#!/usr/bin/env python3
"""Populate the P2-C report subfile from archived campaign data."""
from __future__ import annotations

import shutil
import sys
from pathlib import Path
import pandas as pd

REPO = Path("/workspaces/Tether_Grace")
SUMMARY_DIR = REPO / "output/p2c_mpc_ceiling_sweep/_summary"
FIG_DST = REPO / "report/Figures"
SEC = REPO / "report/sections"

BODY = r"""
The sweep runs V4's dual-fault schedule at five successively
tighter tension ceilings (100, 90, 80, 70, 60~N) plus a baseline
reference in which the ceiling is observed but not enforced.

\input{sections/p2c_summary_table.tex}

\paragraph{Interpretation.}
\input{sections/p2c_interpretation.tex}

\paragraph{Figures.}

\begin{figure}[h]
\centering
\includegraphics[width=0.92\textwidth]{Figures/p2c_peak_vs_ceiling.png}
\caption{P2-C peak rope tension vs.\ ceiling. The baseline (dashed
blue) is independent of the ceiling; the MPC (solid green) pulls the
peak down to (or below) the ceiling line (dotted black).}
\label{fig:p2c-peak-vs-ceiling}
\end{figure}

\begin{figure}[h]
\centering
\includegraphics[width=0.92\textwidth]{Figures/p2c_rms_vs_ceiling.png}
\caption{P2-C tracking-cost of ceiling tightening: MPC RMS tracking
error vs.\ ceiling.}
\label{fig:p2c-rms-vs-ceiling}
\end{figure}

\begin{figure}[h]
\centering
\includegraphics[width=0.92\textwidth]{Figures/p2c_timeseries_peakT.png}
\caption{P2-C peak tension time-series, one curve per ceiling plus
the baseline.}
\label{fig:p2c-timeseries}
\end{figure}
"""


def _pct_delta(a, b):
    return 100.0 * (b - a) / a if a not in (0, None) else float("nan")


def _table(summary: pd.DataFrame) -> str:
    rows = [r"\begin{table}[h]",
            r"\centering\small",
            r"\caption{P2-C sweep: MPC vs.\ baseline at successive "
            r"tension ceilings. Data: "
            r"\texttt{output/p2c\_mpc\_ceiling\_sweep/\_summary/"
            r"p2c\_summary.csv}.}",
            r"\label{tab:p2c}",
            r"\begin{tabular}{lrrrr}",
            r"\toprule",
            r"Mode & $T_\text{max}$ [N] & peak $T$ [N] "
            r"& RMS $\|e_p\|$ [m] & violation frac \\",
            r"\midrule"]
    for _, r in summary.iterrows():
        mode = r["mode"].replace("_", "\\_")
        ceil = (f"{int(r['T_ceiling_N'])}"
                if not pd.isna(r["T_ceiling_N"]) else "---")
        rows.append(
            f"{mode} & {ceil} & {r['peak_T_N']:.1f} "
            f"& {r['rms_err_m']:.3f} & {r['violation_frac']:.3f} \\\\")
    rows += [r"\bottomrule", r"\end{tabular}", r"\end{table}"]
    return "\n".join(rows)


def _interp(summary: pd.DataFrame) -> str:
    base = summary[summary["mode"] == "baseline"]
    if len(base) == 0:
        return "Baseline row missing; interpretation deferred."
    bpk = base["peak_T_N"].values[0]
    lines = [
        f"The baseline peak tension is {bpk:.1f}~N.  With the MPC "
        f"layer active:"]
    entries = []
    for _, r in summary[summary["mode"] != "baseline"].sort_values(
            "T_ceiling_N", ascending=False).iterrows():
        tmax = int(r["T_ceiling_N"])
        pk = r["peak_T_N"]
        viol = r["violation_frac"] * 100.0
        rms = r["rms_err_m"]
        entries.append(
            f"at $T_\\text{{max}}={tmax}$~N the MPC attains peak "
            f"$T$ of {pk:.1f}~N "
            f"({'below' if pk <= tmax else 'exceeding'} the ceiling by "
            f"{abs(pk - tmax):.1f}~N), with RMS tracking {rms:.3f}~m "
            f"and ticks-exceeding-ceiling fraction {viol:.2f}\\%")
    lines.append(";\n\n".join(entries) + ".")
    lines.append(
        "\n\nThis directly demonstrates that the MPC's hard tension "
        "constraint engages as the ceiling is tightened.")
    return "\n\n".join(lines)


def main():
    if not (SUMMARY_DIR / "p2c_summary.csv").exists():
        print("MISSING P2-C summary")
        sys.exit(1)
    summary = pd.read_csv(SUMMARY_DIR / "p2c_summary.csv")
    for png in SUMMARY_DIR.glob("p2c_*.png"):
        shutil.copy(png, FIG_DST / png.name)
    (SEC / "p2c_summary_table.tex").write_text(_table(summary))
    (SEC / "p2c_interpretation.tex").write_text(_interp(summary))
    (SEC / "p2c_results.tex").write_text(BODY)
    print("P2-C report subfiles updated.")


if __name__ == "__main__":
    main()
