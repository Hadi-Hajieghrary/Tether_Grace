"""
IEEE Transactions figure style for the decentralized fault-aware lift paper.

Usage
-----
    from ieee_style import setup_style, COLORS, FAULT_STYLE, SINGLE_COL, DOUBLE_COL
    setup_style()                       # once per script

Single-column: 3.5 in × 2.3 in (default)
Double-column: 7.16 in × 2.8 in
Colour palette: Wong (colour-blind safe) for 4 drones + fault-event orange.
"""

from __future__ import annotations
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ------------------------------------------------------------------- constants
SINGLE_COL = (3.5, 2.3)
DOUBLE_COL = (7.16, 2.8)
DOUBLE_COL_TALL = (7.16, 4.2)
SQUARE = (3.5, 3.3)

# Wong colour-blind safe palette for 4 drones.
COLORS = [
    "#D55E00",   # drone 0 — vermilion/orange
    "#009E73",   # drone 1 — bluish-green
    "#0072B2",   # drone 2 — blue
    "#CC79A7",   # drone 3 — reddish-purple
]

# Other signals
REF_STYLE = dict(color="#000000", linestyle="--", linewidth=1.1, alpha=0.8,
                 label="Reference")
PAYLOAD_STYLE = dict(color="#E69F00", linewidth=1.6, label="Payload")
FAULT_STYLE = dict(color="#CC0000", linestyle=":", linewidth=1.0, alpha=0.8)


def setup_style():
    """Apply publication-quality rcParams to matplotlib."""
    # Prefer serif fonts close to IEEE's Times; fall back gracefully.
    plt.rcParams.update({
        "font.family": "serif",
        "font.serif": ["Times New Roman", "Times", "DejaVu Serif", "serif"],
        "mathtext.fontset": "stix",
        "font.size": 9,
        "axes.labelsize": 9,
        "axes.titlesize": 9,
        "xtick.labelsize": 8,
        "ytick.labelsize": 8,
        "legend.fontsize": 7.5,
        "figure.titlesize": 10,
        "figure.dpi": 120,
        "savefig.dpi": 300,
        "savefig.bbox": "tight",
        "savefig.pad_inches": 0.04,
        "lines.linewidth": 1.2,
        "axes.linewidth": 0.7,
        "xtick.major.width": 0.7,
        "ytick.major.width": 0.7,
        "xtick.major.size": 3.0,
        "ytick.major.size": 3.0,
        "axes.grid": True,
        "grid.linewidth": 0.35,
        "grid.alpha": 0.4,
        "grid.linestyle": "-",
        "axes.spines.top": False,
        "axes.spines.right": False,
        "legend.frameon": False,
        "legend.handlelength": 1.6,
        "legend.columnspacing": 1.0,
        "legend.borderaxespad": 0.4,
    })


def annotate_faults(ax, faults, label_each=False):
    """Draw vertical dashed lines at each fault time."""
    for i, (drone_idx, t_fault) in enumerate(faults):
        ax.axvline(t_fault, **FAULT_STYLE,
                   label=(f"Fault @ t={t_fault:.1f}s" if label_each and i == 0 else None))


def trim_start(df, t_start=0.5):
    """Drop initial simulation samples (startup transient)."""
    return df[df["time"] >= t_start].reset_index(drop=True)


def detect_faults(df, num_drones=4):
    """Infer fault times from tension time-series."""
    import numpy as np
    faults = []
    for i in range(num_drones):
        T = df[f"tension_{i}"].values
        time = df["time"].values
        near_zero = np.abs(T) < 0.2
        if near_zero.any():
            for j in range(len(T) - 1, 0, -1):
                if not near_zero[j]:
                    if j < len(T) - 1:
                        faults.append((i, float(time[j + 1])))
                    break
    return faults
