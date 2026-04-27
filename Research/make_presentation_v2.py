#!/usr/bin/env python3
"""
Tether_Grace — Conference-Quality Video Presentation Generator (v2)
====================================================================

Builds a comprehensive ≥7-minute MP4 presentation that explicitly maps every
major paper claim to the corresponding simulation video, signal trace, and
manuscript figure. Designed for a top-tier control / robotics conference
(IEEE T-CST, ICRA, CDC, ACC).

Architecture
------------
  This script DOES NOT re-render the heavy synchronized sim+signal segments
  (s03_fault_t8, s04_fault_t12, s05_fault_t16, s06–s08_l1_*) — those already
  exist in output/presentation/ from make_presentation.py. We reuse them.

  We add new claim-mapping still-image segments (theorem cards, the
  4 architectural properties, IEEE-figure tours, dwell-time, actuator
  margin, reproducibility, claim→evidence master map) and stitch
  everything into output/presentation_v2.mp4 with polished transitions.

Output
------
  output/presentation_v2.mp4   (1920×1080 30fps h264 yuv420p, ~8 min)

Run from the repo root:
    python3 Research/make_presentation_v2.py

Dependencies: matplotlib, numpy, pandas, PIL (Pillow), ffmpeg.
"""

import os
import sys
import subprocess
import textwrap
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.gridspec import GridSpec
from PIL import Image

# ─── paths ──────────────────────────────────────────────────────────────
ROOT = Path("/workspaces/Tether_Grace")
OUT  = ROOT / "output"
PRES = OUT / "presentation"
PRES_V2 = OUT / "presentation_v2_segments"
FIGS = ROOT / "IEEE_T-CST_camera_ready" / "Figures"
PRES_V2.mkdir(exist_ok=True, parents=True)

# ─── output spec (must match existing segments for clean concat) ───────
W, H = 1920, 1080
FPS  = 30

# ─── colour palette ────────────────────────────────────────────────────
BG     = "#0d1117"
BG2    = "#161b22"
BG3    = "#1c2128"
BLUE   = "#58a6ff"
ORANGE = "#f78166"
GREEN  = "#56d364"
RED    = "#ff7b72"
YELLOW = "#e3b341"
PURPLE = "#bc8cff"
CYAN   = "#76e3ea"
GREY   = "#8b949e"
LIGHT  = "#c9d1d9"
WHITE  = "#e6edf3"

plt.rcParams.update({
    "figure.facecolor": BG,
    "axes.facecolor":   BG2,
    "axes.edgecolor":   GREY,
    "axes.labelcolor":  WHITE,
    "xtick.color":      GREY,
    "ytick.color":      GREY,
    "text.color":       WHITE,
    "grid.color":       "#21262d",
    "grid.linestyle":   "-",
    "grid.linewidth":   0.5,
    "font.size":        12,
    "axes.titlesize":   14,
    "legend.framealpha": 0.3,
    "legend.facecolor": BG2,
    "legend.edgecolor": GREY,
})

# ─── ffmpeg helpers ────────────────────────────────────────────────────
def sh(cmd):
    r = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    if r.returncode != 0:
        print("STDERR:", r.stderr[-1500:], file=sys.stderr)
        raise RuntimeError(f"Command failed: {cmd}")
    return r.stdout

def save_fig(fig, path, dpi=120):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    # No bbox_inches="tight" — we want exact 1920x1080 frames
    fig.savefig(path, dpi=dpi, facecolor=BG, edgecolor="none")
    plt.close(fig)

def still_to_video(img_path, out_path, duration):
    """Encode a still image into a uniform 1920x1080 30fps h264 video."""
    sh(f'ffmpeg -y -loglevel error -loop 1 -i "{img_path}" -t {duration} '
       f'-vf "scale={W}:{H}:force_original_aspect_ratio=decrease,'
       f'pad={W}:{H}:(ow-iw)/2:(oh-ih)/2:color={BG[1:]}" '
       f'-c:v libx264 -preset medium -crf 18 -pix_fmt yuv420p -r {FPS} '
       f'-movflags +faststart "{out_path}"')

def fade_video(in_path, out_path, fade_in=0.4, fade_out=0.5):
    """Add fade-in/fade-out to a video (optional, for polish)."""
    dur = float(sh(f'ffprobe -v quiet -show_entries format=duration '
                   f'-of csv=p=0 "{in_path}"').strip())
    fo_start = max(0.0, dur - fade_out)
    sh(f'ffmpeg -y -loglevel error -i "{in_path}" '
       f'-vf "fade=t=in:st=0:d={fade_in},fade=t=out:st={fo_start}:d={fade_out}" '
       f'-c:v libx264 -preset medium -crf 18 -pix_fmt yuv420p -r {FPS} '
       f'"{out_path}"')

def concat(video_list, out_path):
    """Concatenate uniform-encoded mp4s with the concat demuxer."""
    list_file = PRES_V2 / "_list.txt"
    with open(list_file, "w") as f:
        for v in video_list:
            f.write(f"file '{Path(v).resolve()}'\n")
    sh(f'ffmpeg -y -loglevel error -f concat -safe 0 -i "{list_file}" '
       f'-c copy -movflags +faststart "{out_path}"')

# ═══════════════════════════════════════════════════════════════════════
# NEW SEGMENT BUILDERS (still-image cards)
# ═══════════════════════════════════════════════════════════════════════

def card_title():
    """A0 — Title card with paper details."""
    fig = plt.figure(figsize=(W/100, H/100), facecolor=BG)
    ax  = fig.add_subplot(111); ax.set_facecolor(BG); ax.axis("off")
    # Top accent bar
    ax.add_patch(mpatches.Rectangle((0.0, 0.92), 1.0, 0.012,
                                    transform=ax.transAxes,
                                    facecolor=BLUE, edgecolor="none"))
    ax.add_patch(mpatches.Rectangle((0.0, 0.08), 1.0, 0.012,
                                    transform=ax.transAxes,
                                    facecolor=BLUE, edgecolor="none"))
    ax.text(0.5, 0.78, "Passive Fault Tolerance through",
            ha="center", va="center", color=WHITE, fontsize=42,
            fontweight="bold", transform=ax.transAxes)
    ax.text(0.5, 0.70, "Tension-to-Thrust Feed-Forward",
            ha="center", va="center", color=BLUE, fontsize=42,
            fontweight="bold", transform=ax.transAxes)
    ax.text(0.5, 0.59,
            "Hybrid Input-to-State Stability for Decentralized Multi-UAV\n"
            "Slung-Load Transport under Abrupt Cable Severance",
            ha="center", va="center", color=LIGHT, fontsize=22,
            transform=ax.transAxes, linespacing=1.4)
    ax.text(0.5, 0.45,
            "Hadi Hajieghrary    ·    Paul Schmitt",
            ha="center", va="center", color=WHITE, fontsize=22,
            transform=ax.transAxes)
    ax.text(0.5, 0.41, "Torc Robotics    ·    MassRobotics",
            ha="center", va="center", color=GREY, fontsize=16,
            style="italic", transform=ax.transAxes)
    # Separator
    ax.plot([0.25, 0.75], [0.34, 0.34], color=GREY, lw=0.6,
            transform=ax.transAxes)
    ax.text(0.5, 0.28, "IEEE Transactions on Control Systems Technology",
            ha="center", va="center", color=YELLOW, fontsize=18,
            style="italic", transform=ax.transAxes)
    ax.text(0.5, 0.20,
            "5 drones    ·    10 kg payload    ·    4 m/s Dryden wind    ·    Drake multibody simulation",
            ha="center", va="center", color=GREY, fontsize=14,
            transform=ax.transAxes)
    ax.text(0.5, 0.13,
            "Video presentation — claim-to-evidence map",
            ha="center", va="center", color=GREEN, fontsize=15,
            fontweight="bold", transform=ax.transAxes)
    p = PRES_V2 / "a00_title.png"
    save_fig(fig, p, dpi=110)
    out = PRES_V2 / "a00_title.mp4"
    still_to_video(p, out, duration=10)
    return str(out)

def card_outline():
    """A1 — Roadmap of the talk."""
    fig = plt.figure(figsize=(W/100, H/100), facecolor=BG)
    ax  = fig.add_subplot(111); ax.set_facecolor(BG); ax.axis("off")
    ax.text(0.5, 0.92, "Outline",
            ha="center", va="top", color=WHITE, fontsize=36,
            fontweight="bold", transform=ax.transAxes)
    chapters = [
        ("1", BLUE, "Problem & architecture",
         "What goes wrong when a cable cuts; why classical pipelines cannot react in time."),
        ("2", GREEN, "Three formal guarantees",
         "C1 reduction theorem  ·  C2 hybrid practical-ISS  ·  C3 closed-form L_1 bound."),
        ("3", ORANGE, "Fault-recovery campaign",
         "Three sim recordings (t_1 = 8, 12, 16 s) with synchronized signals + Lyapunov proxy."),
        ("4", RED, "Causal isolation by ablation",
         "Disable the identity T_ff = T_i  →  RMSE +34-39%  ·  sag x 3.6-4.0."),
        ("5", PURPLE, "L_1 adaptive augmentation",
         "Three gains gamma in {2 000, 30 000, 50 000} — plateau predicted by C3."),
        ("6", YELLOW, "Robustness probes & summary",
         "Dwell-time boundary  ·  actuator margin  ·  claim-to-evidence map."),
    ]
    y = 0.78
    for num, color, head, body in chapters:
        # Coloured number block
        ax.add_patch(mpatches.FancyBboxPatch(
            (0.06, y - 0.07), 0.06, 0.085,
            boxstyle="round,pad=0.005",
            transform=ax.transAxes,
            facecolor=color, edgecolor="none", alpha=0.95, zorder=2))
        ax.text(0.09, y - 0.03, num, ha="center", va="center",
                color=BG, fontsize=28, fontweight="bold",
                transform=ax.transAxes, zorder=3)
        ax.text(0.16, y - 0.005, head, ha="left", va="top",
                color=WHITE, fontsize=22, fontweight="bold",
                transform=ax.transAxes)
        ax.text(0.16, y - 0.055, body, ha="left", va="top",
                color=GREY, fontsize=14,
                transform=ax.transAxes)
        y -= 0.115
    ax.text(0.5, 0.04,
            "Each evidence segment shows the simulation video + live signals + the manuscript figure it backs",
            ha="center", va="bottom", color=YELLOW, fontsize=14,
            style="italic", transform=ax.transAxes)
    p = PRES_V2 / "a01_outline.png"
    save_fig(fig, p, dpi=110)
    out = PRES_V2 / "a01_outline.mp4"
    still_to_video(p, out, duration=12)
    return str(out)

def card_claim_evidence_map():
    """A2 — Claim → evidence master map."""
    fig = plt.figure(figsize=(W/100, H/100), facecolor=BG)
    ax  = fig.add_subplot(111); ax.set_facecolor(BG); ax.axis("off")
    ax.text(0.5, 0.94, "Claim — to — Evidence Map",
            ha="center", va="top", color=WHITE, fontsize=32,
            fontweight="bold", transform=ax.transAxes)
    ax.text(0.5, 0.89, "Every paper claim is backed by an artefact in this video",
            ha="center", va="top", color=GREY, fontsize=15,
            style="italic", transform=ax.transAxes)
    rows = [
        ("C1", BLUE,
         "Taut-cable reduction theorem",
         "delta = tau_rope / tau_pend = 0.003;  H1a 35.8 ms < 40 ms;  H3 0.10% < 1%",
         "Domain-audit table  ·  Section IV.A"),
        ("C2", GREEN,
         "Hybrid practical-ISS via T_ff = T_i",
         "rho_V4 = 0.20;  rho_V5 = 0.045;  recovery within tau_pend = 2.24 s",
         "V1-V6 fault-zoom;  Lyapunov proxy  ·  Section IV"),
        ("C3", ORANGE,
         "Closed-form L_1 gain bound",
         "Gamma* = 2/(T_s p_22) ~= 4.76e5;  sweep stays in plateau",
         "L_1 stability dashboard  ·  Section V.A"),
        ("P1", RED,
         "Identity is the causal mechanism (P2-A ablation)",
         "FF off  ->  RMSE +34-39%   sag x 3.6-4.0  across V3 V4 V5",
         "FF-ablation chart  ·  self-announcement figure"),
        ("P2", PURPLE,
         "Architecture is robust to fault timing",
         "RMSE 0.313-0.328 m;  sag 35-55 mm across t_1 in {8 12 16} s",
         "Three sim recordings + summary chart"),
        ("P3", YELLOW,
         "Actuator-feasible (no saturation)",
         "Max thrust ratio 0.79;  zero saturated-time fraction",
         "Actuator-margin chart  ·  Section VI.G"),
        ("P4", CYAN,
         "Recovery envelope contracts (rho < 1)",
         "V_pre 0.46-0.48 m  ->  V_post 0.32 m  steady state",
         "Lyapunov-decay illustration"),
    ]
    # Two columns: tag + title (left), payload (right)
    y = 0.82
    for tag, c, head, num, where in rows:
        ax.add_patch(mpatches.FancyBboxPatch(
            (0.04, y - 0.05), 0.045, 0.062,
            boxstyle="round,pad=0.005",
            transform=ax.transAxes,
            facecolor=c, edgecolor="none", zorder=2))
        ax.text(0.0625, y - 0.018, tag, ha="center", va="center",
                color=BG, fontsize=14, fontweight="bold",
                transform=ax.transAxes, zorder=3)
        ax.text(0.10, y - 0.005, head, ha="left", va="top",
                color=WHITE, fontsize=15, fontweight="bold",
                transform=ax.transAxes)
        ax.text(0.10, y - 0.030, num, ha="left", va="top",
                color=LIGHT, fontsize=12, family="monospace",
                transform=ax.transAxes)
        ax.text(0.62, y - 0.020, where, ha="left", va="top",
                color=GREY, fontsize=12, style="italic",
                transform=ax.transAxes)
        y -= 0.09
    p = PRES_V2 / "a02_claimmap.png"
    save_fig(fig, p, dpi=110)
    out = PRES_V2 / "a02_claimmap.mp4"
    still_to_video(p, out, duration=14)
    return str(out)

def card_problem_redux():
    """A3 — Tightened problem statement (replaces s01)."""
    fig = plt.figure(figsize=(W/100, H/100), facecolor=BG)
    ax  = fig.add_subplot(111); ax.set_facecolor(BG); ax.axis("off")
    ax.text(0.5, 0.93, "Problem — and Why Classical Pipelines Fail",
            ha="center", va="top", color=WHITE, fontsize=30,
            fontweight="bold", transform=ax.transAxes)
    # Left: problem
    ax.text(0.05, 0.83, "▶  Plant", color=BLUE, fontsize=20,
            fontweight="bold", transform=ax.transAxes, va="top")
    body1 = (
        "•  N multirotor UAVs lift a payload through compliant cables.\n"
        "•  A cable severs unannounced (abrasion, fatigue, obstacle).\n"
        "•  Tension redistributes across survivors within tau_pend ~= 2.24 s.\n"
        "•  Plant is hybrid: discrete cardinality decrement, not perturbation."
    )
    for i, line in enumerate(body1.split("\n")):
        ax.text(0.07, 0.78 - i*0.038, line, color=WHITE, fontsize=14,
                family="monospace", transform=ax.transAxes, va="top")

    ax.text(0.05, 0.55, "▶  Why classical methods cannot react in time",
            color=RED, fontsize=20, fontweight="bold",
            transform=ax.transAxes, va="top")
    body2 = (
        "•  Centralized force allocation: per-tick global state — broken at fault.\n"
        "•  Active fault tolerance: detect -> classify -> reconfigure ~ tau_pend.\n"
        "•  Slung-load + geometric: assume fixed cable cardinality.\n"
        "•  Distributed MPC: message complexity scales with horizon and rank."
    )
    for i, line in enumerate(body2.split("\n")):
        ax.text(0.07, 0.50 - i*0.038, line, color=LIGHT, fontsize=14,
                family="monospace", transform=ax.transAxes, va="top")

    # Right: the architectural commitment as the bottom punchline
    ax.add_patch(mpatches.FancyBboxPatch(
        (0.04, 0.08), 0.92, 0.22,
        boxstyle="round,pad=0.01",
        transform=ax.transAxes,
        facecolor=BG2, edgecolor=GREEN, linewidth=2.5, zorder=2))
    ax.text(0.5, 0.27, "★  Architectural commitment",
            ha="center", va="top", color=GREEN, fontsize=22,
            fontweight="bold", transform=ax.transAxes)
    ax.text(0.5, 0.22,
            r"$T_i^{\mathrm{ff}}(t)\;=\;T_i(t)$",
            ha="center", va="top", color=WHITE, fontsize=42,
            transform=ax.transAxes)
    ax.text(0.5, 0.13,
            "Each drone's altitude thrust feed-forward equals its OWN measured rope tension.\n"
            "No detection. No communication. No reconfiguration. No peer state.",
            ha="center", va="top", color=LIGHT, fontsize=15,
            transform=ax.transAxes, linespacing=1.4)
    p = PRES_V2 / "a03_problem.png"
    save_fig(fig, p, dpi=110)
    out = PRES_V2 / "a03_problem.mp4"
    still_to_video(p, out, duration=18)
    return str(out)

def card_four_properties():
    """A4 — The four architectural properties of the controller."""
    fig = plt.figure(figsize=(W/100, H/100), facecolor=BG)
    ax  = fig.add_subplot(111); ax.set_facecolor(BG); ax.axis("off")
    ax.text(0.5, 0.93, "Four Architectural Properties — by Construction",
            ha="center", va="top", color=WHITE, fontsize=30,
            fontweight="bold", transform=ax.transAxes)
    props = [
        (BLUE, "1.  Locality",
         "The information set I_i (eq. 12) contains no peer state, no fault flag,\n"
         "no shared scheduler.  Strictly co-located sensors + a pre-flight plan."),
        (GREEN, "2.  Passivity",
         "Same identity operates pre- and post-fault.  No detector, classifier, or\n"
         "reassignment rule in the closed loop.  Tension rises -> thrust steps."),
        (ORANGE, "3.  Robustness to multi-agent failure",
         "By induction on the fault counter:  recovery envelope carries for any\n"
         "F <= N - 2 unannounced severances under  m_L g / (N - F) <= kappa_act f_max."),
        (PURPLE, "4.  Team-size independence",
         "Per-drone law and lemma stack carry no N structurally.  Dependence\n"
         "concentrates in two scalars  m_L g / N  and  m_L g / (N - F)."),
    ]
    y = 0.83
    for color, head, body in props:
        ax.add_patch(mpatches.FancyBboxPatch(
            (0.05, y - 0.155), 0.90, 0.155,
            boxstyle="round,pad=0.008",
            transform=ax.transAxes,
            facecolor=BG2, edgecolor=color, linewidth=2.0, zorder=2))
        ax.text(0.07, y - 0.025, head, ha="left", va="top",
                color=color, fontsize=20, fontweight="bold",
                transform=ax.transAxes)
        for i, line in enumerate(body.split("\n")):
            ax.text(0.10, y - 0.07 - i*0.038, line, ha="left", va="top",
                    color=LIGHT, fontsize=13, family="monospace",
                    transform=ax.transAxes)
        y -= 0.19
    ax.text(0.5, 0.035,
            "Demonstrated at (N, m_L, F) = (5, 10 kg, 2)  =  27%  of the actuator envelope",
            ha="center", va="bottom", color=YELLOW, fontsize=14,
            style="italic", transform=ax.transAxes)
    p = PRES_V2 / "a04_props.png"
    save_fig(fig, p, dpi=110)
    out = PRES_V2 / "a04_props.mp4"
    still_to_video(p, out, duration=16)
    return str(out)

def card_theorems():
    """A5 — Three theorem statements, side-by-side."""
    fig = plt.figure(figsize=(W/100, H/100), facecolor=BG)
    ax  = fig.add_subplot(111); ax.set_facecolor(BG); ax.axis("off")
    ax.text(0.5, 0.94, "Three Closed-Form Guarantees",
            ha="center", va="top", color=WHITE, fontsize=32,
            fontweight="bold", transform=ax.transAxes)
    ax.text(0.5, 0.89, "Stated on the slack-excursion-bounded admissibility domain  Omega^dwell_tau",
            ha="center", va="top", color=GREY, fontsize=15,
            style="italic", transform=ax.transAxes)
    cards = [
        (0.04, BLUE, "C1",
         "Taut-cable reduction",
         "On Omega^dwell_tau the bead-chain plant is\n"
         "O(delta + eta_max) close to the lumped\n"
         "k_eff = k_s / N_seg model.\n\n"
         "delta = tau_rope / tau_pend ~= 0.003\n"
         "k_eff = 2778 N/m  (k_s = 25 000)\n\n"
         "Justifies the reduced-order plant\n"
         "used in the rest of the analysis.",
         "Section II.C  /  Lemmas 1-2"),
        (0.355, GREEN, "C2",
         "Hybrid practical-ISS",
         "Closed-loop tracking error admits an\n"
         "explicit recovery envelope:\n\n"
         "rho = exp(-alpha_min tau_pend)  in (0, 1)\n"
         "lambda = min(alpha_z, alpha_xy)  s^-1\n\n"
         "Carries by induction over  F <= N-2\n"
         "unannounced severances satisfying\n"
         "the dwell condition  tau_d >= tau_pend.",
         "Section IV  /  Lemmas 1-5  +  Thm 7"),
        (0.67, ORANGE, "C3",
         "L_1 closed-form bound",
         "Discrete-time sufficient stability bound\n"
         "on the L_1 adaptation gain:\n\n"
         "Gamma  <  Gamma*  =  2 / (T_s p_22)\n"
         "       ~=  4.76 x 10^5\n\n"
         "Removes the dominant L_1 hyper-parameter\n"
         "from empirical tuning.",
         "Section V.A  /  Proposition 1"),
    ]
    for x, color, tag, title, body, where in cards:
        # Coloured top stripe
        ax.add_patch(mpatches.Rectangle(
            (x, 0.79), 0.29, 0.04,
            transform=ax.transAxes, facecolor=color, edgecolor="none", zorder=2))
        ax.text(x + 0.145, 0.81, tag, ha="center", va="center",
                color=BG, fontsize=22, fontweight="bold",
                transform=ax.transAxes, zorder=3)
        # Body box
        ax.add_patch(mpatches.FancyBboxPatch(
            (x, 0.13), 0.29, 0.66,
            boxstyle="round,pad=0.005",
            transform=ax.transAxes,
            facecolor=BG2, edgecolor=color, linewidth=1.5, zorder=2))
        ax.text(x + 0.145, 0.74, title, ha="center", va="top",
                color=color, fontsize=18, fontweight="bold",
                transform=ax.transAxes)
        for i, line in enumerate(body.split("\n")):
            ax.text(x + 0.015, 0.69 - i*0.034, line, ha="left", va="top",
                    color=LIGHT, fontsize=13, family="monospace",
                    transform=ax.transAxes)
        ax.text(x + 0.145, 0.16, where, ha="center", va="top",
                color=GREY, fontsize=11, style="italic",
                transform=ax.transAxes)
    ax.text(0.5, 0.06,
            "Each theorem is supported by 5 lemmas (Section IV) and validated by a verification test  (Research/tests/)",
            ha="center", va="bottom", color=YELLOW, fontsize=14,
            style="italic", transform=ax.transAxes)
    p = PRES_V2 / "a05_theorems.png"
    save_fig(fig, p, dpi=110)
    out = PRES_V2 / "a05_theorems.mp4"
    still_to_video(p, out, duration=18)
    return str(out)

def _figure_card(fig_filename, title, claim_lines, output_name, duration=12,
                 max_w_frac=0.62, fig_caption=""):
    """Insert a manuscript figure on the right + claim text on the left."""
    fig_path = FIGS / fig_filename
    if not fig_path.exists():
        print(f"  WARN: missing figure {fig_path}; skipping {output_name}")
        return None
    fig = plt.figure(figsize=(W/100, H/100), facecolor=BG)
    ax_text = fig.add_axes([0.04, 0.10, 0.38, 0.78]); ax_text.axis("off"); ax_text.set_facecolor(BG)
    ax_img  = fig.add_axes([0.46, 0.08, 0.50, 0.85]); ax_img.axis("off"); ax_img.set_facecolor(BG)
    # Title at top
    fig.text(0.5, 0.95, title, ha="center", va="top", color=WHITE,
             fontsize=24, fontweight="bold")
    # Claim text on left
    ax_text.text(0, 0.95, "Claim", color=YELLOW, fontsize=18,
                 fontweight="bold", transform=ax_text.transAxes, va="top")
    for i, line in enumerate(claim_lines):
        col = WHITE if i == 0 else LIGHT
        weight = "bold" if i == 0 else "normal"
        ax_text.text(0, 0.88 - i*0.06, line, color=col, fontsize=15,
                     fontweight=weight,
                     transform=ax_text.transAxes, va="top")
    # Figure
    img = Image.open(fig_path)
    ax_img.imshow(img)
    if fig_caption:
        ax_img.text(0.5, -0.02, fig_caption, ha="center", va="top",
                    color=GREY, fontsize=11, style="italic",
                    transform=ax_img.transAxes)
    p = PRES_V2 / f"{output_name}.png"
    save_fig(fig, p, dpi=110)
    out = PRES_V2 / f"{output_name}.mp4"
    still_to_video(p, out, duration=duration)
    return str(out)

def card_self_announcement():
    return _figure_card(
        "fig_self_announcement_V4.png",
        "Self-Announcement Causal Chain  (V4 dual-fault, drone 0)",
        [
            "Identity T_ff = T_i is the only causal pathway",
            "",
            "•  Top: surviving rope tension T_1(t) jumps at fault.",
            "•  Then: T_ff command tracks T_1 within one control tick.",
            "•  Then: thrust f_1 steps proportionally.",
            "•  Result: altitude error stays bounded.",
            "",
            "Without the identity (P2-A, dashed): thrust",
            "step is absent;  altitude error grows ~300 mm.",
        ],
        "a06_self_announcement",
        duration=14,
        fig_caption="Manuscript Figure 11  (Section VI.D)")

def card_fault_zoom_V4():
    return _figure_card(
        "fig_faultzoom_lyapunov_V4.png",
        "Lyapunov Proxy V(t) on V4 Dual-Fault Schedule",
        [
            "C2 hybrid practical-ISS visualised",
            "",
            "•  Each fault: bounded jump  V(t_f^+) - V(t_f^-) <= chi(Delta_f)",
            "•  Between events: exponential contraction",
            "        V(t_{k+1}^-) <= rho V(t_k^-) + c",
            "•  Empirical fits:",
            "    rho_V4 = 0.20",
            "    rho_V5 = 0.045",
            "•  Both well below the rho < 1 sufficiency bound.",
        ],
        "a07_lyapunov_V4",
        duration=14,
        fig_caption="Manuscript Figure 10c  (Section VI.D)")

def card_dwell_sweep():
    return _figure_card(
        "fig_dwell_sweep.png",
        "Dwell-Time Boundary Probe",
        [
            "Empirical contraction beyond H2's sufficiency",
            "",
            "•  Sweep tau_d / tau_pend in {0.5 ... 2.0}",
            "•  rho_hat below unity at every point",
            "    including the sub-threshold tau_d = 0.5 tau_pend",
            "    where rho_hat ~= 0.63.",
            "",
            "H2 (tau_d >= tau_pend) is sufficient",
            "but not necessary on this trajectory family.",
        ],
        "a08_dwell",
        duration=12,
        fig_caption="Manuscript Figure 13  (Section VI.E)")

def card_actuator_margin():
    """Actuator-margin chart from the summary CSV."""
    df = pd.read_csv(OUT / "actuator_margin" / "summary.csv")
    fig = plt.figure(figsize=(W/100, H/100), facecolor=BG)
    fig.text(0.5, 0.94, "Actuator Margin  —  No Saturation",
             ha="center", va="top", color=WHITE, fontsize=28,
             fontweight="bold")
    fig.text(0.5, 0.88,
             "Property P3 of Definition 1: per-drone thrust stays inside the actuator envelope with margin.",
             ha="center", va="top", color=GREY, fontsize=14, style="italic")
    # Bar chart
    ax1 = fig.add_axes([0.07, 0.18, 0.46, 0.62])
    ax1.set_facecolor(BG2)
    bars = ax1.bar(df["variant"], df["max_ratio_per_variant"],
                   color=[GREEN, GREEN, GREEN, ORANGE], alpha=0.92,
                   edgecolor=WHITE, linewidth=1.0)
    ax1.axhline(0.9, color=YELLOW, lw=1.5, ls="--", label="Saturation hazard line  0.9 f_max")
    ax1.axhline(1.0, color=RED, lw=2.0, ls="-", label="Actuator ceiling  f_max")
    ax1.set_ylim(0, 1.1)
    ax1.set_ylabel(r"Max thrust ratio  $\max_i\, f_i / f_{max}$", fontsize=14)
    ax1.set_title("V3 V4 V5 V6 capability variants", color=WHITE, fontsize=14)
    ax1.grid(True, axis="y", alpha=0.3)
    ax1.legend(fontsize=11, loc="lower right")
    for bar, val in zip(bars, df["max_ratio_per_variant"]):
        ax1.text(bar.get_x() + bar.get_width()/2, val + 0.025,
                 f"{val:.3f}", ha="center", va="bottom",
                 color=WHITE, fontsize=13, fontweight="bold")

    # Right text
    ax2 = fig.add_axes([0.58, 0.18, 0.36, 0.62]); ax2.axis("off")
    ax2.set_facecolor(BG2)
    text = (
        "Across V3 V4 V5 V6:\n\n"
        "•  Max ratio  =  0.73   (V3-V5, drone 4)\n"
        "•  Max ratio  =  0.79   (V6, full stack)\n"
        "•  Time over 0.9 f_max:  0%\n\n"
        "Recovery is NOT actuator-limited.\n"
        "The cascade does not silently rely on\n"
        "saturation to absorb post-fault loads.\n\n"
        "Demonstrated point on the\n"
        "actuator-margin feasibility surface:\n"
        "  m_L g / (N - F) = 32.7 N\n"
        "  vs  kappa_act f_max = 123 N\n"
        "  =>  27% of the envelope\n"
    )
    ax2.text(0.0, 0.95, text, ha="left", va="top", color=LIGHT,
             fontsize=14, family="monospace",
             transform=ax2.transAxes,
             bbox=dict(boxstyle="round,pad=0.5", facecolor=BG2,
                       edgecolor=ORANGE, alpha=0.95))
    fig.text(0.5, 0.07,
             "Manuscript Section VI.G  ·  Table VIII  ·  Figure 18",
             ha="center", va="bottom", color=GREY, fontsize=12, style="italic")
    p = PRES_V2 / "a09_actuator.png"
    save_fig(fig, p, dpi=110)
    out = PRES_V2 / "a09_actuator.mp4"
    still_to_video(p, out, duration=12)
    return str(out)

def card_recovery_iae():
    """Per-fault recovery metrics from recovery_iae/summary.csv."""
    df = pd.read_csv(OUT / "recovery_iae" / "summary.csv")
    fig = plt.figure(figsize=(W/100, H/100), facecolor=BG)
    fig.text(0.5, 0.94, "Per-Fault Recovery Metrics",
             ha="center", va="top", color=WHITE, fontsize=28,
             fontweight="bold")
    fig.text(0.5, 0.88,
             "Peak error, peak sag, recovery time, integrated absolute error per fault.",
             ha="center", va="top", color=GREY, fontsize=14, style="italic")
    # 3 small panels
    gs = GridSpec(1, 3, left=0.06, right=0.97, top=0.78, bottom=0.16, wspace=0.32)
    labels = [f"{r['variant']} f{int(r['fault_idx'])}" for _, r in df.iterrows()]
    x = np.arange(len(labels))

    ax1 = fig.add_subplot(gs[0]); ax1.set_facecolor(BG2)
    ax1.bar(x, df["peak_e_m"]*1000, color=BLUE, alpha=0.92, edgecolor=WHITE)
    ax1.set_xticks(x); ax1.set_xticklabels(labels, rotation=35, fontsize=10, ha="right")
    ax1.set_ylabel("Peak error norm  [mm]", fontsize=12)
    ax1.set_title("Peak tracking error", fontsize=13, color=WHITE)
    ax1.grid(True, axis="y", alpha=0.3)

    ax2 = fig.add_subplot(gs[1]); ax2.set_facecolor(BG2)
    ax2.bar(x, df["peak_sag_mm"], color=ORANGE, alpha=0.92, edgecolor=WHITE)
    ax2.axhline(100, color=RED, lw=1.8, ls="--", label="Acceptance 100 mm")
    ax2.set_xticks(x); ax2.set_xticklabels(labels, rotation=35, fontsize=10, ha="right")
    ax2.set_ylabel("Peak altitude sag  [mm]", fontsize=12)
    ax2.set_title("Peak sag — all below ceiling", fontsize=13, color=WHITE)
    ax2.legend(fontsize=10); ax2.grid(True, axis="y", alpha=0.3)

    ax3 = fig.add_subplot(gs[2]); ax3.set_facecolor(BG2)
    ax3.bar(x, df["iae_post_ms"]*1000, color=GREEN, alpha=0.92, edgecolor=WHITE)
    ax3.set_xticks(x); ax3.set_xticklabels(labels, rotation=35, fontsize=10, ha="right")
    ax3.set_ylabel("IAE post-fault  [mm·s]", fontsize=12)
    ax3.set_title("Integrated absolute error", fontsize=13, color=WHITE)
    ax3.grid(True, axis="y", alpha=0.3)

    fig.text(0.5, 0.05,
             "First faults sub-tick (settle in <1 ms);  V4 V5 V6 second faults: 0.32 s, 1.58 s, 0.33 s",
             ha="center", va="bottom", color=YELLOW, fontsize=13)
    p = PRES_V2 / "a10_recovery.png"
    save_fig(fig, p, dpi=110)
    out = PRES_V2 / "a10_recovery.mp4"
    still_to_video(p, out, duration=12)
    return str(out)

def card_l1_dashboard():
    return _figure_card(
        "fig_l1_stability_dashboard.png",
        "L_1 Adaptive-Gain Stability Dashboard  (C3)",
        [
            "Closed-form bound  Gamma < Gamma*  validated empirically",
            "",
            "•  Sweep  Gamma in [500, 80 000]",
            "•  Spans  2.4 decades, reaches 0.17 Gamma*",
            "•  RMSE unimodal:  101 mm -> 88.5 mm -> 91.4 mm",
            "•  Variance ratio stays in [0.36, 0.37]",
            "•  Smooth degradation past Gamma*, not a sharp",
            "    instability — signature of a sufficient",
            "    Lyapunov bound.",
        ],
        "a11_l1_dashboard",
        duration=14,
        fig_caption="Manuscript Figure 17  (Section VI.F)")

def card_feasibility_envelope():
    return _figure_card(
        "fig_feasibility_envelope.png",
        "Actuator-Margin Feasibility Envelope",
        [
            "The case study is one point on a designable surface",
            "",
            "•  Demonstrated  (N, m_L, F)  =  (5, 10 kg, 2)",
            "    (red star)",
            "•  Sits at 27% of the F = 2 admissible band",
            "•  Designable extension along all three axes:",
            "    -  larger fleets  N",
            "    -  heavier payloads  m_L",
            "    -  more sequential faults  F",
            "•  Per-drone law has no  N  structurally.",
        ],
        "a12_feasibility",
        duration=12,
        fig_caption="Manuscript Figure 1  (Section II.C)")

def card_reproducibility():
    """Reproducibility / how-to-rerun card."""
    fig = plt.figure(figsize=(W/100, H/100), facecolor=BG)
    ax  = fig.add_subplot(111); ax.set_facecolor(BG); ax.axis("off")
    ax.text(0.5, 0.94, "Reproducibility",
            ha="center", va="top", color=WHITE, fontsize=30,
            fontweight="bold", transform=ax.transAxes)
    ax.text(0.5, 0.88,
            "Every claim is reproducible end-to-end from this repository.",
            ha="center", va="top", color=GREY, fontsize=15,
            style="italic", transform=ax.transAxes)

    # Code block — build & run
    code = (
        "# Build\n"
        "cmake -S Research/cpp -B Research/cpp/build -DCMAKE_BUILD_TYPE=Release\n"
        "cmake --build Research/cpp/build --target decentralized_fault_aware_sim\n"
        "\n"
        "# Capability demonstration   (V1 - V6, ~2 h on 8 cores)\n"
        "./Research/scripts/run_capability_demo.sh\n"
        "python3  Research/analysis/plot_capability_demo.py\n"
        "\n"
        "# Phase-2 stress campaigns   (causal isolation + L1 sweep)\n"
        "./Research/scripts/run_p2a_tension_ff_ablation.sh    # P2-A  causal isolation\n"
        "./Research/scripts/run_p2b_mass_mismatch.sh          # P2-B  L1 mass sweep\n"
        "./Research/scripts/run_l1_gain_map.sh                # C3    Gamma sweep\n"
        "\n"
        "# Theorem regression tests\n"
        "for t in Research/tests/test_*.py; do  python3 $t  ;  done\n"
    )
    ax.add_patch(mpatches.FancyBboxPatch(
        (0.05, 0.18), 0.90, 0.62,
        boxstyle="round,pad=0.01",
        transform=ax.transAxes,
        facecolor=BG2, edgecolor=BLUE, linewidth=1.5))
    ax.text(0.07, 0.77, code, ha="left", va="top", color=LIGHT,
            fontsize=13, family="monospace",
            transform=ax.transAxes)
    ax.text(0.5, 0.10,
            "Bit-exact deterministic at fixed (N, m_L, k_s, dt, Drake version, wind seed).\n"
            "Seed manifest:  seed_wind seed_init seed_sensor seed_fault seed_solver  ·  archived per run.",
            ha="center", va="bottom", color=GREEN, fontsize=14,
            transform=ax.transAxes, linespacing=1.5)
    p = PRES_V2 / "a13_repro.png"
    save_fig(fig, p, dpi=110)
    out = PRES_V2 / "a13_repro.mp4"
    still_to_video(p, out, duration=14)
    return str(out)

def card_final():
    """Final close-out card — claims and contact."""
    fig = plt.figure(figsize=(W/100, H/100), facecolor=BG)
    ax  = fig.add_subplot(111); ax.set_facecolor(BG); ax.axis("off")
    ax.text(0.5, 0.92,
            "Take-away",
            ha="center", va="top", color=WHITE, fontsize=34,
            fontweight="bold", transform=ax.transAxes)
    ax.text(0.5, 0.84,
            "For self-announcing structural transitions, a single decentralized identity\n"
            r"$T_i^{\mathrm{ff}}(t)\;=\;T_i(t)$" + "\n"
            "delivers hybrid practical-ISS without detector, communication, or reconfiguration.",
            ha="center", va="top", color=LIGHT, fontsize=18,
            transform=ax.transAxes, linespacing=1.7)

    # Three pillar boxes
    pillars = [
        (0.06, BLUE,
         "Theory",
         "C1  Reduction theorem\n"
         "C2  Hybrid practical-ISS\n"
         "C3  Closed-form L_1 bound\n"
         "5 lemmas, 3 propositions"),
        (0.37, GREEN,
         "Evidence",
         "Capability demo V1-V6\n"
         "Causal ablation P2-A\n"
         "L_1 gain sweep  C3\n"
         "Dwell + actuator probes"),
        (0.69, ORANGE,
         "Reproducibility",
         "Drake C++ simulator\n"
         "Python analysis pipeline\n"
         "5 verification tests\n"
         "Bit-exact campaigns"),
    ]
    for x, c, head, body in pillars:
        ax.add_patch(mpatches.FancyBboxPatch(
            (x, 0.32), 0.25, 0.32,
            boxstyle="round,pad=0.005",
            transform=ax.transAxes,
            facecolor=BG2, edgecolor=c, linewidth=2.0))
        ax.text(x + 0.125, 0.60, head, ha="center", va="top",
                color=c, fontsize=22, fontweight="bold",
                transform=ax.transAxes)
        ax.text(x + 0.125, 0.55, body, ha="center", va="top",
                color=LIGHT, fontsize=14, family="monospace",
                transform=ax.transAxes, linespacing=1.6)

    ax.text(0.5, 0.22,
            "Manuscript:   IEEE_T-CST_camera_ready/Main.pdf",
            ha="center", va="top", color=YELLOW, fontsize=16,
            transform=ax.transAxes)
    ax.text(0.5, 0.17,
            "Code, simulator, and campaign harness:   github.com  (this repository)",
            ha="center", va="top", color=GREY, fontsize=14,
            transform=ax.transAxes)
    ax.text(0.5, 0.09,
            "Hadi Hajieghrary  ·  Paul Schmitt    ·    Torc Robotics  ·  MassRobotics",
            ha="center", va="top", color=GREY, fontsize=14,
            style="italic", transform=ax.transAxes)
    ax.text(0.5, 0.04,
            "Thank you  —  questions welcome.",
            ha="center", va="top", color=GREEN, fontsize=18,
            fontweight="bold", transform=ax.transAxes)
    p = PRES_V2 / "a14_final.png"
    save_fig(fig, p, dpi=110)
    out = PRES_V2 / "a14_final.mp4"
    still_to_video(p, out, duration=14)
    return str(out)

# Section headers ------------------------------------------------------
def header(name, num, color, title, subtitle=""):
    fig = plt.figure(figsize=(W/100, H/100), facecolor=BG)
    ax  = fig.add_subplot(111); ax.set_facecolor(BG); ax.axis("off")
    # Big chapter mark
    ax.add_patch(mpatches.Circle((0.18, 0.55), 0.13,
                                 transform=ax.transAxes,
                                 facecolor=color, alpha=0.18, zorder=2))
    ax.text(0.18, 0.55, str(num), ha="center", va="center",
            color=color, fontsize=120, fontweight="bold",
            transform=ax.transAxes, zorder=3)
    ax.text(0.42, 0.62, title, ha="left", va="center",
            color=WHITE, fontsize=44, fontweight="bold",
            transform=ax.transAxes)
    if subtitle:
        ax.text(0.42, 0.50, subtitle, ha="left", va="center",
                color=color, fontsize=22, transform=ax.transAxes)
    out = PRES_V2 / f"hdr_{name}.mp4"
    p   = PRES_V2 / f"hdr_{name}.png"
    save_fig(fig, p, dpi=110)
    still_to_video(p, out, duration=4)
    return str(out)

# ═══════════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════════

def extract_body(src_mp4, out_path, start_s=18.0, end_s=316.335):
    """Extract a normalised body from the existing presentation.mp4
    (drops the original 18 s title and ~22 s conclusion), re-encoded to
    match our card stream so concat is byte-clean."""
    dur = end_s - start_s
    sh(f'ffmpeg -y -loglevel error '
       f'-ss {start_s} -i "{src_mp4}" -t {dur} '
       f'-vf "scale={W}:{H}:force_original_aspect_ratio=decrease,'
       f'pad={W}:{H}:(ow-iw)/2:(oh-ih)/2:color={BG[1:]},setsar=1" '
       f'-c:v libx264 -preset medium -crf 18 -pix_fmt yuv420p -r {FPS} '
       f'-an -movflags +faststart "{out_path}"')

# Path to the existing all-in-one rendered presentation
EXISTING_MP4 = ROOT / "IEEE_T-CST_camera_ready" / "presentation.mp4"

def main():
    print("=" * 72)
    print("  Tether_Grace  —  Conference Presentation v2  (claim-mapped)")
    print("=" * 72)
    sh("ffmpeg -version 2>&1 | head -1")

    if not EXISTING_MP4.exists():
        print(f"ERROR: existing presentation not found at {EXISTING_MP4}")
        print("Run Research/make_presentation.py first to render the heavy")
        print("synchronized sim+signal segments.")
        sys.exit(1)

    # --- Step A: render all the new claim-mapping cards (idempotent) -----
    print("\n[Step A] Rendering claim-mapping cards...")
    a00 = card_title()
    a01 = card_outline()
    a03 = card_problem_redux()
    a04 = card_four_properties()
    h_theory  = header("theory",   "I",  GREEN,
                       "Three closed-form guarantees",
                       "C1  reduction  ·  C2  hybrid practical-ISS  ·  C3  L_1 bound")
    a05 = card_theorems()
    a12 = card_feasibility_envelope()

    h_evidence = header("evidence", "II", ORANGE,
                        "Evidence — campaigns from the simulator",
                        "fault recovery  ·  causal ablation  ·  L_1 adaptive plateau")

    h_robust   = header("robust",   "III", YELLOW,
                        "Robustness probes & figure tour",
                        "self-announce  ·  Lyapunov proxy  ·  dwell  ·  actuator  ·  recovery")
    a06 = card_self_announcement()
    a07 = card_fault_zoom_V4()
    a08 = card_dwell_sweep()
    a09 = card_actuator_margin()
    a10 = card_recovery_iae()
    a11 = card_l1_dashboard()

    h_close    = header("close",    "IV", GREEN,
                        "Take-away & reproducibility",
                        "claim-to-evidence map  ·  build  ·  run  ·  test")
    a02 = card_claim_evidence_map()
    a13 = card_reproducibility()
    a14 = card_final()

    # --- Step B: extract body of existing presentation -------------------
    print("\n[Step B] Extracting body of existing presentation.mp4...")
    body = PRES_V2 / "_body_extract.mp4"
    # Drop original 0-18 s title and 316.335-338.335 s conclusion;
    # keep the 298 s of evidence content
    # Skip original [0-64 s] (title + tr_problem + s01_problem + s02_arch),
    # which we have already replaced with our claim-mapped intro cards.
    # Skip original [316.335-342.335 s] (tr_conclusion + s_conclusion),
    # replaced by our own outro.
    extract_body(EXISTING_MP4, body, start_s=64.0, end_s=316.335)
    body_dur = float(sh(f'ffprobe -v quiet -show_entries format=duration '
                        f'-of csv=p=0 "{body}"').strip())
    print(f"  body extract: {body_dur:.1f} s")

    # --- Step C: assemble final order ------------------------------------
    order = [
        # Setup
        a00, a01, a03, a04,
        # Theory
        h_theory, a05, a12,
        # Body (existing): three faults, summary, ablation, L1 sweep, summary
        h_evidence,
        str(body),
        # Robustness probes & figure tour  (uses claim-mapped cards)
        h_robust, a06, a07, a08, a09, a10, a11,
        # Take-away
        h_close, a02, a13, a14,
    ]

    # Estimate
    total = sum(float(sh(f'ffprobe -v quiet -show_entries format=duration '
                         f'-of csv=p=0 "{v}"').strip()) for v in order)
    print(f"\nEstimated total duration: {total:.1f} s  ({total/60:.2f} min)")

    # Stitch
    out = OUT / "presentation_v2.mp4"
    print(f"\nConcatenating {len(order)} segments  ->  {out}")
    concat(order, out)

    # Verify
    final_dur = float(sh(f'ffprobe -v quiet -show_entries format=duration '
                         f'-of csv=p=0 "{out}"').strip())
    size_mb = out.stat().st_size / 1e6
    print("\n" + "=" * 72)
    print(f"  presentation_v2.mp4  ready")
    print(f"  Duration:  {final_dur:.1f} s  ({final_dur/60:.2f} min)")
    print(f"  Size:      {size_mb:.1f} MB")
    print(f"  Path:      {out}")
    print("=" * 72)

if __name__ == "__main__":
    main()
