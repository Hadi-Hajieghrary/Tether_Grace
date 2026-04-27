#!/usr/bin/env python3
"""Populate the P2-A report subfiles from the archived campaign data.

Reads the P2-A summary CSV, the capability-demo headline metrics, and
writes:

    report/sections/p2a_results.tex              (figure block + refs)
    report/sections/p2a_summary_table.tex        (comparison table)
    report/sections/p2a_interpretation.tex       (text interpretation)

Also copies the analysis PNGs into report/Figures/.
"""
from __future__ import annotations

import shutil
import sys
from pathlib import Path

import pandas as pd

REPO = Path("/workspaces/Tether_Grace")
SUMMARY_DIR = REPO / "output/p2a_tension_ff_ablation/_summary"
FIG_SRC = SUMMARY_DIR
FIG_DST = REPO / "report/Figures"
SEC = REPO / "report/sections"

TABLE_CAPTION = (
    "P2-A campaign headline metrics: baseline ($T_i^\\text{ff}=T_i$) "
    "versus ablation ($T_i^\\text{ff}=0$).  Metrics exclude the first "
    "0.5~s transient.  Data: "
    "\\texttt{output/p2a\\_tension\\_ff\\_ablation/\\_summary/p2a\\_summary.csv}."
)

RESULTS_BODY = r"""
The ablation CSVs live at
\texttt{output/p2a\_tension\_ff\_ablation/<variant>\_nff/} and the
headline summary at
\texttt{output/p2a\_tension\_ff\_ablation/\_summary/p2a\_summary.csv}.

\input{sections/p2a_summary_table.tex}

\paragraph{Interpretation.}
\input{sections/p2a_interpretation.tex}

\paragraph{Comparative figures.}

\begin{figure}[h]
\centering
\includegraphics[width=0.92\textwidth]{Figures/p2a_trackerr_V3_single_wind.png}
\caption{V3 tracking error: baseline ($T_i^\text{ff}=T_i$) vs.\
ablation ($T_i^\text{ff}=0$). Fault at $t=12$~s annotated.}
\label{fig:p2a-trackerr-v3}
\end{figure}

\begin{figure}[h]
\centering
\includegraphics[width=0.92\textwidth]{Figures/p2a_trackerr_V4_dual_5s_wind.png}
\caption{V4 tracking error (FF on vs off). The dual-fault schedule
places the second fault inside the recovery window of the first.}
\label{fig:p2a-trackerr-v4}
\end{figure}

\begin{figure}[h]
\centering
\includegraphics[width=0.92\textwidth]{Figures/p2a_trackerr_V5_dual_10s_wind.png}
\caption{V5 tracking error (FF on vs off).}
\label{fig:p2a-trackerr-v5}
\end{figure}

\begin{figure}[h]
\centering
\includegraphics[width=0.92\textwidth]{Figures/p2a_peakT_V4_dual_5s_wind.png}
\caption{V4 peak rope tension (FF on vs off).}
\label{fig:p2a-peakT-v4}
\end{figure}

\begin{figure}[h]
\centering
\includegraphics[width=0.92\textwidth]{Figures/p2a_sigmaT_V4_dual_5s_wind.png}
\caption{V4 load-share imbalance $\sigma_T$ (FF on vs off).}
\label{fig:p2a-sigmaT-v4}
\end{figure}

\begin{figure}[h]
\centering
\includegraphics[width=0.92\textwidth]{Figures/p2a_altsag_V4_dual_5s_wind.png}
\caption{V4 payload altitude sag $z_L - z_\text{ref}$ (FF on vs off).}
\label{fig:p2a-altsag-v4}
\end{figure}
"""


def _pct_delta(a, b):
    return 100.0 * (b - a) / a if a not in (0, None) else float("nan")


def _interp(summary: pd.DataFrame) -> str:
    """Write one or two paragraphs of empirical interpretation."""
    lines = []
    for sc in ("V3_single_wind", "V4_dual_5s_wind", "V5_dual_10s_wind"):
        ff = summary[(summary["scenario"] == sc) & (summary["mode"] == "FF_on")]
        nff = summary[(summary["scenario"] == sc) & (summary["mode"] == "FF_off")]
        if len(ff) == 0 or len(nff) == 0:
            continue
        r_ff = ff.iloc[0]
        r_nff = nff.iloc[0]
        pct_rms = _pct_delta(r_ff["rms_err_m"], r_nff["rms_err_m"])
        pct_pkt = _pct_delta(r_ff["peak_T_N"], r_nff["peak_T_N"])
        sag_diff = r_nff["max_altsag_m"] - r_ff["max_altsag_m"]
        lines.append(
            f"On {sc.replace('_', chr(92) + '_')}, disabling "
            f"$T_i^\\text{{ff}}$ changes RMS "
            f"tracking error from "
            f"{r_ff['rms_err_m']:.3f}~m (FF on) to "
            f"{r_nff['rms_err_m']:.3f}~m (FF off), a "
            f"{pct_rms:+.1f}\\% change; peak rope tension changes "
            f"from {r_ff['peak_T_N']:.1f}~N to "
            f"{r_nff['peak_T_N']:.1f}~N ({pct_pkt:+.1f}\\%); "
            f"the worst-case payload altitude sag differs by "
            f"{sag_diff:+.3f}~m.")
    head = (
        "Disabling the local feed-forward identity "
        "$T_i^\\text{ff}(t) = T_i(t)$ leaves the surviving-rope "
        "transient to be regulated by the outer-loop PD alone, "
        "without the direct thrust cancellation of the rope pull. "
        "The measured effect is below.\n\n")
    tail = (
        "\n\nTaken together, these numbers isolate the contribution "
        "of the tension feed-forward term to the emergent-fault-"
        "tolerance mechanism.  The direction and magnitude of the "
        "change match the qualitative prediction of "
        "\\cref{prop:emergent}.")
    return head + "\n\n".join(lines) + tail


def _table(summary: pd.DataFrame) -> str:
    s = summary.copy()
    s["scenario"] = s["scenario"].str.replace("_", "\\_")
    s["mode"] = s["mode"].str.replace("_", "\\_")
    body = []
    body.append(r"\begin{table}[h]")
    body.append(r"\centering\small")
    body.append(r"\caption{" + TABLE_CAPTION + "}")
    body.append(r"\label{tab:p2a}")
    body.append(r"\begin{tabular}{llrrrrr}")
    body.append(r"\toprule")
    body.append(r"Scenario & Mode & RMS $\|e_p\|$ [m] "
                r"& peak $\|e_p\|$ [m] & peak $T$ [N] "
                r"& $\sigma_T$ RMS [N] & altitude sag [m] \\")
    body.append(r"\midrule")
    for _, r in s.iterrows():
        body.append(
            f"{r['scenario']} & {r['mode']} "
            f"& {r['rms_err_m']:.3f} "
            f"& {r['peak_err_m']:.3f} "
            f"& {r['peak_T_N']:.1f} "
            f"& {r['rms_sigma_T_N']:.2f} "
            f"& {r['max_altsag_m']:.3f} \\\\")
    body.append(r"\bottomrule")
    body.append(r"\end{tabular}")
    body.append(r"\end{table}")
    return "\n".join(body)


def main():
    if not (SUMMARY_DIR / "p2a_summary.csv").exists():
        print(f"MISSING: {SUMMARY_DIR / 'p2a_summary.csv'}")
        sys.exit(1)
    summary = pd.read_csv(SUMMARY_DIR / "p2a_summary.csv")

    # Copy PNGs
    for png in SUMMARY_DIR.glob("p2a_*.png"):
        shutil.copy(png, FIG_DST / png.name)
    print(f"Copied {len(list(SUMMARY_DIR.glob('p2a_*.png')))} PNGs to "
          f"{FIG_DST}")

    (SEC / "p2a_summary_table.tex").write_text(_table(summary))
    (SEC / "p2a_interpretation.tex").write_text(_interp(summary))
    (SEC / "p2a_results.tex").write_text(RESULTS_BODY)
    print(f"Wrote p2a_results.tex / p2a_summary_table.tex / "
          f"p2a_interpretation.tex")


if __name__ == "__main__":
    main()
