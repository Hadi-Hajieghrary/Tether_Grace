#!/usr/bin/env python3
"""
Draw the four system-level figures (architecture, controller, rope model,
formation geometry) as self-contained PNG files in IEEE style.

Usage:
    python3 plot_system.py <output_dir>
"""
from __future__ import annotations
import sys
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Circle
from matplotlib.lines import Line2D

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ieee_style import setup_style, COLORS, DOUBLE_COL, SINGLE_COL

setup_style()


# FIG 1  System Architecture (block diagram)
def fig_system_architecture(out: Path):
    fig, ax = plt.subplots(figsize=(7.16, 3.6))
    ax.set_xlim(0, 100); ax.set_ylim(0, 60); ax.axis("off")

    def box(x, y, w, h, label, fc="#E6F0FA", ec="#0072B2", fontsize=8, lw=1.0):
        p = FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.3",
                           fc=fc, ec=ec, lw=lw, zorder=2)
        ax.add_patch(p)
        ax.text(x + w / 2, y + h / 2, label, ha="center", va="center",
                fontsize=fontsize, zorder=3)

    def arrow(x0, y0, x1, y1, label="", label_pos=0.5, offset=(0, 1.2),
              color="#333333"):
        ax.add_patch(FancyArrowPatch((x0, y0), (x1, y1), arrowstyle="->",
                                     mutation_scale=8, lw=0.9, color=color,
                                     zorder=1))
        if label:
            lx = x0 + (x1 - x0) * label_pos + offset[0]
            ly = y0 + (y1 - y0) * label_pos + offset[1]
            ax.text(lx, ly, label, fontsize=6.5, color="#333333",
                    ha="center", va="center")

    # -- Plant at centre -----------------------------------------------
    box(35, 22, 30, 14, "Drake MultibodyPlant\n(4 quadrotors + payload +\n4×8 rope beads)",
        fc="#FFF4D9", ec="#E69F00", lw=1.2)

    # -- 4 controllers (stacked left) ----------------------------------
    ctrl_y = [43, 31, 19, 7]
    for i in range(4):
        box(2, ctrl_y[i], 22, 8,
            f"Drone {i} Controller\n(cascade PD + peer-aware)",
            fc="#E8F5EC", ec=COLORS[i], fontsize=7, lw=1.0)
        # Arrow from controller → plant
        arrow(24, ctrl_y[i] + 4, 35, 30, label="")
        # Arrow from plant → controller
        arrow(35, 28, 24, ctrl_y[i] + 4, label="", color="#666666")

    # -- Rope subsystems (top right) -----------------------------------
    box(72, 43, 24, 8, "RopeForceSystem × 4\n(spring-damper chain)",
        fc="#F0E8F7", ec="#CC79A7", fontsize=7)
    arrow(72, 47, 65, 36, label="spatial\nforces", label_pos=0.3, offset=(-2, 0))
    arrow(65, 34, 72, 47, label="state", label_pos=0.5, offset=(3.5, 0),
          color="#666666")

    # -- Cable fault gate + tension fault gate (right middle) ---------
    box(72, 28, 24, 8, "CableFaultGate +\nTensionFaultGate",
        fc="#FDEBEE", ec="#CC0000", fontsize=7)
    arrow(72, 32, 65, 32, label="gated\nforce & T", label_pos=0.4, offset=(-1, 1.2))

    # -- Tension aggregator (multiplexer) ------------------------------
    box(72, 10, 24, 14,
        "Peer-Tension Multiplexer\n(aggregates N scalar T\ninto the [T₀..T₃] vector)",
        fc="#E1F0E6", ec="#009E73", fontsize=7)
    arrow(72, 17, 24, 17,
          label=r"broadcast $\mathbf{T}_{peers}$ to every drone",
          label_pos=0.5, offset=(0, 1.5))

    # -- Legends -------------------------------------------------------
    ax.text(50, 57, "Decentralized Fault-Aware Lift — System Diagram",
            ha="center", fontsize=10, fontweight="bold")
    # Legend for arrow flavors
    ax.text(1, 54, "→  signal (control / sensor)", fontsize=7)
    fig.savefig(out)
    plt.close(fig)


# FIG 2  Controller Architecture (cascade PD + peer-aware)
def fig_controller_architecture(out: Path):
    fig, ax = plt.subplots(figsize=(7.16, 3.2))
    ax.set_xlim(0, 100); ax.set_ylim(0, 50); ax.axis("off")

    def blk(x, y, w, h, txt, ec="#0072B2", fc="#E6F0FA", lw=1.0, fs=7.5):
        p = FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.25",
                           fc=fc, ec=ec, lw=lw)
        ax.add_patch(p)
        ax.text(x + w / 2, y + h / 2, txt, ha="center", va="center", fontsize=fs)

    def arr(x0, y0, x1, y1, label="", lp=0.5, off=(0, 1.2)):
        ax.add_patch(FancyArrowPatch((x0, y0), (x1, y1), arrowstyle="->",
                                     mutation_scale=8, lw=0.9, color="#333333"))
        if label:
            lx = x0 + (x1 - x0) * lp + off[0]
            ly = y0 + (y1 - y0) * lp + off[1]
            ax.text(lx, ly, label, fontsize=6.5, ha="center", va="center")

    # Inputs on the left
    ax.text(5, 40, r"$\mathbf{p}_{\mathrm{ref}}, \mathbf{v}_{\mathrm{ref}}$",
            fontsize=9)
    ax.text(5, 32, r"$\mathbf{p}_i, \mathbf{v}_i$  (self)",
            fontsize=8)
    ax.text(5, 24, r"$T_i$  (own rope)", fontsize=8)
    ax.text(5, 14, r"$\mathbf{T}_{\mathrm{peers}}$ (N-vector)", fontsize=8,
            color="#CC0000")

    # Block 1: Trajectory generator
    blk(22, 36, 16, 8, "Trajectory\ngenerator\n(ComputeTrajectory)", ec="#0072B2")
    arr(17, 40, 22, 40)

    # Block 2: Peer-aware N_alive estimator  (NEW)
    blk(22, 10, 16, 10,
        r"$N_{\mathrm{alive}} = \sum_i \mathrm{clamp}(T_i / T_{\mathrm{nom}})$",
        ec="#CC0000", fc="#FDEBEE", fs=7)
    arr(21, 16, 22, 15)
    # peer-awareness output:
    blk(40, 24, 18, 8,
        r"Target tension $T^\star = \frac{m_L g}{N_{\mathrm{alive}}}$",
        ec="#CC0000", fc="#FDEBEE", fs=7)
    arr(38, 15, 40, 26)

    # Block 3: Position PD (outer loop)
    blk(40, 36, 18, 8, "Position PD\n(outer loop)", ec="#0072B2")
    arr(38, 40, 40, 40)
    # Block 4: Altitude PD + tension FF
    blk(62, 36, 18, 8, r"Altitude PD + $T^\star$ feed-forward", ec="#009E73", fs=7)
    arr(58, 40, 62, 40, label="pos err")
    arr(58, 28, 62, 36, label=r"$T^\star$", lp=0.6, off=(-1, 0.8))
    arr(26, 32, 62, 40, label=r"$\mathbf{v}_i$", lp=0.7, off=(-2, 1))

    # Block 5: Attitude PD (inner loop)
    blk(62, 20, 18, 8, "Attitude PD\n(inner loop)", ec="#0072B2")
    arr(49, 36, 62, 24, label="tilt cmd", lp=0.6, off=(1, -0.8))

    # Output: spatial force
    blk(84, 28, 14, 10,
        r"$\mathbf{F}_i = R_i \mathbf{f}_B$" + "\n"
        r"$\boldsymbol{\tau}_i$",
        ec="#E69F00", fc="#FFF4D9", fs=7)
    arr(80, 40, 84, 34)
    arr(80, 24, 84, 32)

    ax.text(50, 47,
            "Controller Architecture — Cascade PD with Peer-Aware Load-Share Adaptation",
            ha="center", fontsize=9, fontweight="bold")
    fig.savefig(out)
    plt.close(fig)


# FIG 3  Rope / Cable Model (schematic)
def fig_rope_model(out: Path):
    fig, ax = plt.subplots(figsize=DOUBLE_COL)
    ax.set_xlim(-0.5, 10); ax.set_ylim(-3.5, 1.2); ax.axis("off")

    # Drone at origin
    ax.add_patch(FancyBboxPatch((-0.35, 0.1), 0.7, 0.3,
                                boxstyle="round,pad=0.05",
                                fc="#D55E00", ec="black"))
    ax.text(0, 0.7, "Drone  $i$", ha="center", fontsize=8)

    # 8 beads at equal spacing, with spring-damper between each pair
    n = 8
    xs = np.linspace(1.0, 8.0, n)
    y = -1.6 + 0.4 * np.sin(np.linspace(0, np.pi, n))     # slight catenary visual
    for j in range(n):
        ax.add_patch(Circle((xs[j], y[j]), 0.15, fc="#444444", ec="black", zorder=3))
        if j < n - 1:
            # Spring-damper connector (wavy + rectangle)
            x0, y0 = xs[j], y[j]; x1, y1 = xs[j + 1], y[j + 1]
            ax.plot([x0 + 0.15, x1 - 0.15], [y0, y1], color="#888", lw=1.1, zorder=1)
    # Payload at the end
    ax.add_patch(Circle((9.0, -1.3), 0.35, fc="#E69F00", ec="black"))
    ax.text(9.0, -2.1, "Payload $L$", ha="center", fontsize=8)
    ax.plot([xs[-1] + 0.15, 9.0 - 0.35], [y[-1], -1.3], color="#888", lw=1.1)

    # Annotations
    ax.annotate(r"bead mass $m_b$", xy=(xs[3], y[3]), xytext=(xs[3] - 0.3, -2.8),
                fontsize=7, arrowprops=dict(arrowstyle="-", lw=0.6))
    ax.annotate(r"spring-damper: $T = \max(0, k_s \Delta L + c_s \dot\Delta L)$",
                xy=((xs[1] + xs[2]) / 2, (y[1] + y[2]) / 2),
                xytext=(3.3, -3.0), fontsize=7,
                arrowprops=dict(arrowstyle="-", lw=0.6))

    # Parameter box
    txt = (r"$N_{seg} = 8,\quad L = 1.25\ \mathrm{m},\quad m_b = 25\ \mathrm{g}$" + "\n"
           r"$k_s = 8000\ \mathrm{N/m},\quad c_s = 35\ \mathrm{N\!\cdot\!s/m}$" + "\n"
           r"$\zeta \approx 0.4,\quad \omega_n \approx 566\ \mathrm{rad/s}$" + "\n"
           r"EA/L $\approx 1000\ \mathrm{N/m}$ (6 mm polyester)")
    ax.text(0.2, -3.4, txt, fontsize=7,
            bbox=dict(boxstyle="round,pad=0.3", fc="#FFF4D9", ec="#E69F00", lw=0.8))

    ax.set_title("Rope Model — 8-bead Tension-only Spring-Damper Chain", fontsize=9)
    fig.savefig(out)
    plt.close(fig)


# FIG 4  Formation geometry (top-view)
def fig_formation(out: Path):
    fig, ax = plt.subplots(figsize=SINGLE_COL)
    r = 0.8
    theta = np.linspace(0, 2 * np.pi, 100)
    ax.plot(r * np.cos(theta), r * np.sin(theta), color="#cccccc", lw=0.6,
            linestyle=":")

    # Drones at 0,90,180,270
    for i, a in enumerate(np.linspace(0, 2 * np.pi, 5)[:-1]):
        x, y = r * np.cos(a), r * np.sin(a)
        ax.add_patch(Circle((x, y), 0.12, fc=COLORS[i], ec="black", zorder=3))
        ax.text(x + 0.18 * np.cos(a), y + 0.18 * np.sin(a),
                f"Drone {i}", fontsize=7, ha="center")
        # Rope line to payload
        ax.plot([x, 0], [y, 0], color="#888888", lw=0.7, linestyle="-", zorder=1)

    # Payload at centre
    ax.add_patch(Circle((0, 0), 0.14, fc="#E69F00", ec="black", zorder=3))
    ax.text(0.18, -0.18, "Payload", fontsize=7)

    ax.set_aspect("equal")
    ax.set_xlim(-1.3, 1.3); ax.set_ylim(-1.3, 1.3)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_title("Formation — 4 drones, radius 0.8 m", fontsize=9)
    # Turn grid lighter
    ax.grid(True, linewidth=0.3, alpha=0.3)
    fig.savefig(out)
    plt.close(fig)


def main():
    if len(sys.argv) < 2:
        print(__doc__); sys.exit(1)
    out_dir = Path(sys.argv[1])
    out_dir.mkdir(parents=True, exist_ok=True)
    fig_system_architecture(out_dir / "fig_system_architecture.png")
    fig_controller_architecture(out_dir / "fig_controller_architecture.png")
    fig_rope_model(out_dir / "fig_rope_model.png")
    fig_formation(out_dir / "fig_formation.png")
    print(f"4 system figures written to {out_dir}")


if __name__ == "__main__":
    main()
