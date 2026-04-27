#!/usr/bin/env python3
"""WP2.2: Closed-form feasibility envelope for (N, m_L, F).

The actuator-margin condition is
    m_L g / (N - F)  <=  kappa * f_max.

Plot, in (N, m_L) space, the boundary curves at F = 0, 1, 2, 3 and
shade the infeasible region (above the F=0 line is unsafe even
without faults; the contracting curves at F>0 mark the admissible
region under increasing fault counts). Mark the demonstrated point
(N=5, m_L=10 kg, F=2).

Output: IEEE_T-CST/Figures/fig_feasibility_envelope.png
"""
from __future__ import annotations
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ieee_style import COLORS, DOUBLE_COL, setup_style  # noqa: E402

setup_style()

G = 9.81
KAPPA = 0.82
F_MAX = 150.0  # N — per-drone thrust ceiling

FIG = Path("/workspaces/Tether_Grace/IEEE_T-CST/Figures")
FIG.mkdir(parents=True, exist_ok=True)


def m_l_max(N: np.ndarray, F: int) -> np.ndarray:
    """Max admissible payload mass at integer (N, F): kappa f_max
    (N-F) / g."""
    surv = np.maximum(N - F, 1)
    return KAPPA * F_MAX * surv / G


def main():
    N_vals = np.arange(3, 11)
    fig, ax = plt.subplots(figsize=DOUBLE_COL)
    F_levels = [0, 1, 2, 3]
    style_map = {0: "-", 1: "--", 2: "-.", 3: ":"}
    for F, c in zip(F_levels, [COLORS[0], COLORS[1], COLORS[2], COLORS[3]]):
        m = m_l_max(N_vals, F)
        ax.plot(N_vals, m, color=c, marker="o", linewidth=1.4,
                linestyle=style_map[F],
                label=rf"$F={F}$ ($m_L^{{\max}} = "
                      rf"\kappa f_{{\max}} (N-F)/g$)")
    # Demonstrated point
    ax.plot(5, 10, marker="*", color="red", markersize=14,
            linestyle="None", label=r"demonstrated: $N{=}5$, "
                                    r"$m_L{=}10$ kg, $F{=}2$")
    # Shade infeasible region above F=0 (per-drone hover capacity)
    ax.fill_between(N_vals, m_l_max(N_vals, 0), 1e3,
                    color="gray", alpha=0.18,
                    label="infeasible (no-fault hover excess)")
    ax.set_xlabel(r"team size $N$")
    ax.set_ylabel(r"payload mass $m_L$ $[\mathrm{kg}]$")
    ax.set_xticks(N_vals)
    ax.set_ylim(0, 1.05 * m_l_max(N_vals, 0).max())
    ax.legend(loc="upper left", fontsize=7.5)
    ax.set_title(r"Actuator-margin feasibility envelope "
                 r"($\kappa f_{\max}=123$ N, $g=9.81\,\mathrm{m/s^2}$)",
                 fontsize=9)
    fig.tight_layout()
    fig.savefig(FIG / "fig_feasibility_envelope.png", dpi=180)
    plt.close(fig)
    print(f"  wrote {FIG / 'fig_feasibility_envelope.png'}")


if __name__ == "__main__":
    main()
