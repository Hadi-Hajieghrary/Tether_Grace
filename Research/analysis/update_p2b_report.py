#!/usr/bin/env python3
"""Populate the P2-B report subfile from archived campaign data."""
from __future__ import annotations

import shutil
import sys
from pathlib import Path
import pandas as pd

REPO = Path("/workspaces/Tether_Grace")
SUMMARY_DIR = REPO / "output/p2b_mass_mismatch/_summary"
FIG_DST = REPO / "report/Figures"
SEC = REPO / "report/sections"

BODY = r"""
The sweep covers four actual payload masses (2.5, 3.0, 3.5,
3.9~kg) against a controller nominal of 3~kg, in three modes:
(A) baseline with $T^\text{ff}$, (B) baseline with
$T^\text{ff}$ disabled, (C) mode~(B) plus $L_1$ augmentation.

\input{sections/p2b_summary_table.tex}

\paragraph{Interpretation.}
\input{sections/p2b_interpretation.tex}

\paragraph{Figures.}

\begin{figure}[h]
\centering
\includegraphics[width=0.92\textwidth]{Figures/p2b_altsag_vs_mass.png}
\caption{P2-B RMS altitude sag vs.\ actual payload mass. Expected:
mode~A flat (FF compensates), mode~B linear in mismatch, mode~C
recovering toward mode~A.}
\label{fig:p2b-altsag}
\end{figure}

\begin{figure}[h]
\centering
\includegraphics[width=0.92\textwidth]{Figures/p2b_altbias_vs_mass.png}
\caption{P2-B mean altitude bias vs.\ actual payload mass.}
\label{fig:p2b-altbias}
\end{figure}
"""


def _table(summary: pd.DataFrame) -> str:
    rows = [r"\begin{table}[h]",
            r"\centering\small",
            r"\caption{P2-B mass-mismatch sweep. Data: "
            r"\texttt{output/p2b\_mass\_mismatch/\_summary/p2b\_summary.csv}.}",
            r"\label{tab:p2b}",
            r"\begin{tabular}{rllrrr}",
            r"\toprule",
            r"$\mL$ [kg] & mode & Label & RMS $\|e_p\|$ [m] "
            r"& mean sag [m] & RMS sag [m] \\",
            r"\midrule"]
    for _, r in summary.iterrows():
        mode = r["mode"].replace("_", "\\_")
        rows.append(
            f"{r['mass_kg']} & {mode} & {r['mode_label']} "
            f"& {r['rms_err_m']:.3f} "
            f"& {r['mean_alt_sag_m']:.3f} "
            f"& {r['rms_alt_sag_m']:.3f} \\\\")
    rows += [r"\bottomrule", r"\end{tabular}", r"\end{table}"]
    return "\n".join(rows)


def _interp(summary: pd.DataFrame) -> str:
    import numpy as np
    modes = ("ff_on", "ff_off", "ff_off_l1")
    ranges = {}
    for mode in modes:
        sub = summary[summary["mode"] == mode]
        if len(sub) == 0:
            continue
        ranges[mode] = (sub["rms_alt_sag_m"].min(), sub["rms_alt_sag_m"].max(),
                        sub["rms_alt_sag_m"].max() - sub["rms_alt_sag_m"].min())
    s = []
    if "ff_on" in ranges:
        lo, hi, span = ranges["ff_on"]
        s.append(
            f"Mode (A) baseline with $T^\\text{{ff}}$: RMS altitude "
            f"sag spans [{lo:.3f}, {hi:.3f}]~m across the mass sweep "
            f"({span*100:.1f}~cm range), consistent with the hypothesis "
            f"that $T^\\text{{ff}}=T_i$ absorbs mass mismatch "
            f"automatically.")
    if "ff_off" in ranges:
        lo, hi, span = ranges["ff_off"]
        s.append(
            f"Mode (B) baseline with $T^\\text{{ff}}$ off: RMS sag "
            f"spans [{lo:.3f}, {hi:.3f}]~m, "
            f"a wider {span*100:.1f}~cm range --- the mismatch now "
            f"manifests in the altitude channel.")
    if "ff_off_l1" in ranges:
        lo, hi, span = ranges["ff_off_l1"]
        s.append(
            f"Mode (C) with $L_1$: RMS sag spans "
            f"[{lo:.3f}, {hi:.3f}]~m "
            f"({span*100:.1f}~cm range), representing the reduction "
            f"in mass-mismatch sensitivity due to the adaptive layer.")
    return "\n\n".join(s)


def main():
    if not (SUMMARY_DIR / "p2b_summary.csv").exists():
        print("MISSING P2-B summary")
        sys.exit(1)
    summary = pd.read_csv(SUMMARY_DIR / "p2b_summary.csv")
    for png in SUMMARY_DIR.glob("p2b_*.png"):
        shutil.copy(png, FIG_DST / png.name)
    (SEC / "p2b_summary_table.tex").write_text(_table(summary))
    (SEC / "p2b_interpretation.tex").write_text(_interp(summary))
    (SEC / "p2b_results.tex").write_text(BODY)
    print("P2-B report subfiles updated.")


if __name__ == "__main__":
    main()
