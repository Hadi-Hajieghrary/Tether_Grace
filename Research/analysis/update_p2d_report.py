#!/usr/bin/env python3
"""Populate the P2-D report subfile from archived campaign data."""
from __future__ import annotations

import shutil
import sys
from pathlib import Path
import pandas as pd

REPO = Path("/workspaces/Tether_Grace")
SUMMARY_DIR = REPO / "output/p2d_period_sweep/_summary"
FIG_DST = REPO / "report/Figures"
SEC = REPO / "report/sections"

BODY = r"""
The sweep runs V4's dual-fault schedule at three lemniscate
periods (8, 10, 12~s) with and without the reshape supervisor.

\input{sections/p2d_summary_table.tex}

\paragraph{Interpretation.}
\input{sections/p2d_interpretation.tex}

\paragraph{Figures.}

\begin{figure}[h]
\centering
\includegraphics[width=0.92\textwidth]{Figures/p2d_peak_vs_period.png}
\caption{P2-D peak post-fault tension vs.\ lemniscate period, with
and without the reshape supervisor.}
\label{fig:p2d-peak-vs-period}
\end{figure}

\begin{figure}[h]
\centering
\includegraphics[width=0.92\textwidth]{Figures/p2d_reduction_vs_period.png}
\caption{P2-D percentage peak-tension reduction from reshape,
by period. Theory predicts $\approx 25.8\%$ at $T=12$~s,
increasing at shorter periods.}
\label{fig:p2d-reduction}
\end{figure}
"""


def _table(summary: pd.DataFrame) -> str:
    rows = [r"\begin{table}[h]",
            r"\centering\small",
            r"\caption{P2-D sweep: reshape benefit vs.\ lemniscate "
            r"period. Data: "
            r"\texttt{output/p2d\_period\_sweep/\_summary/p2d\_summary.csv}.}",
            r"\label{tab:p2d}",
            r"\begin{tabular}{rlrrr}",
            r"\toprule",
            r"$T$ [s] & mode & peak all [N] & peak post [N] "
            r"& RMS $\|e_p\|$ [m] \\",
            r"\midrule"]
    for _, r in summary.iterrows():
        mode = r["mode"].replace("_", " ")
        rows.append(
            f"{int(r['T_period_s'])} & {mode} "
            f"& {r['peak_T_all_N']:.1f} "
            f"& {r['peak_T_post_fault_N']:.1f} "
            f"& {r['rms_err_m']:.3f} \\\\")
    rows += [r"\bottomrule", r"\end{tabular}", r"\end{table}"]
    return "\n".join(rows)


def _interp(summary: pd.DataFrame) -> str:
    lines = []
    for T in sorted(summary["T_period_s"].unique()):
        nr = summary[(summary["T_period_s"] == T)
                     & (summary["mode"] == "noreshape")]
        re = summary[(summary["T_period_s"] == T)
                     & (summary["mode"] == "reshape")]
        if len(nr) == 0 or len(re) == 0:
            continue
        nrv = nr["peak_T_post_fault_N"].values[0]
        rev = re["peak_T_post_fault_N"].values[0]
        pct = 100.0 * (nrv - rev) / nrv if nrv > 0 else 0.0
        lines.append(
            f"At $T={int(T)}$~s, peak post-fault tension is "
            f"{nrv:.1f}~N without reshape and {rev:.1f}~N with "
            f"reshape, a {pct:+.1f}\\% change.")
    return ("\n\n".join(lines) +
            "\n\nThe trend with period should be interpreted against "
            "the theoretical prediction in "
            "\\cref{sec:reshape-optimal}: reshape benefit grows as "
            "centripetal load grows (shorter period).")


def main():
    if not (SUMMARY_DIR / "p2d_summary.csv").exists():
        print("MISSING P2-D summary")
        sys.exit(1)
    summary = pd.read_csv(SUMMARY_DIR / "p2d_summary.csv")
    for png in SUMMARY_DIR.glob("p2d_*.png"):
        shutil.copy(png, FIG_DST / png.name)
    (SEC / "p2d_summary_table.tex").write_text(_table(summary))
    (SEC / "p2d_interpretation.tex").write_text(_interp(summary))
    (SEC / "p2d_results.tex").write_text(BODY)
    print("P2-D report subfiles updated.")


if __name__ == "__main__":
    main()
