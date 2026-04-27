#!/usr/bin/env python3
"""
Add informational text overlays to the simulation panel black-bar areas
of the existing composite segment files, then rebuild presentation_v2.mp4.

Input  : output/presentation/*.mp4   (already-built composite segments)
Output : output/presentation_v2.mp4

Runtime: ~2–4 minutes (drawtext filter on 6 segments + concat — no re-render).

Layout of the left sim panel (960 × 1080 px):
  - Top black bar  : y =   0 – 315   (316 px)   ← title + context text
  - Simulation video: y = 316 – 763   (448 px)
  - Bottom black bar: y = 764 – 1079  (316 px)   ← key metrics text
"""

import os
import subprocess
import sys

PRES = "/workspaces/Tether_Grace/output/presentation"
OUT  = "/workspaces/Tether_Grace/output"
FPS  = 30

# ── palette ──────────────────────────────────────────────────────────────────
YELLOW = "#e3b341"
WHITE  = "#e6edf3"
GREY   = "#8b949e"
BLUE   = "#58a6ff"
GREEN  = "#56d364"
ORANGE = "#f78166"
RED    = "#ff7b72"
BG     = "#0d1117"

# ── per-segment y-positions ───────────────────────────────────────────────────
# Top block (316 px available: y=0..315)
TOP_YS     = [18,  68,  96, 124]       # line start y
TOP_FS     = [24,  16,  14,  14]       # fontsize
TOP_COLORS = [YELLOW, WHITE, GREY, GREY]

# Bottom block (316 px available: y=764..1079)
BOT_Y0  = 772
BOT_YS  = [BOT_Y0, BOT_Y0 + 30, BOT_Y0 + 58, BOT_Y0 + 86]
BOT_FS  = [15, 14, 14, 14]
BOT_COLORS = [YELLOW, WHITE, WHITE, GREEN]


# ── per-segment overlay text ──────────────────────────────────────────────────
OVERLAYS = {
    "s03_fault_t8": {
        "top": [
            "Fault-Time Sweep — Cable Severs at  t = 8 s  [1 of 3]",
            "Early fault: drone 0 cable cuts during the high-speed lemniscate turn",
            "vs t=12s / t=16s:  most aggressive phase — highest tensions (130.6 N)",
            "All 4 surviving drones respond passively with no coordination or detection",
        ],
        "bottom": [
            "KEY METRICS",
            "3D RMSE = 0.324 m     |     Peak tension = 130.6 N  (actuator limit 150 N)",
            "Peak altitude sag = 45.4 mm     |     Recovery within tau_pend = 2.24 s",
            "T_ff = T_i self-adjusts instantly — no detection delay, no inter-drone comms",
        ],
    },
    "s04_fault_t12": {
        "top": [
            "Fault-Time Sweep — Cable Severs at  t = 12 s  [2 of 3]",
            "Nominal scenario: fault at cruise phase — the paper baseline case",
            "vs t=8s / t=16s:  intermediate tensions and sag — mid-range severity",
            "Watch: T_ff traces (dashed) jump and then redistribute across 4 drones",
        ],
        "bottom": [
            "KEY METRICS",
            "3D RMSE = 0.328 m     |     Peak tension = 87.3 N",
            "Peak altitude sag = 55.4 mm     |     Recovery within one lemniscate period",
            "T_ff rises immediately after severance — passive mechanism at work",
        ],
    },
    "s05_fault_t16": {
        "top": [
            "Fault-Time Sweep — Cable Severs at  t = 16 s  [3 of 3]",
            "Late fault: lemniscate in gentle deceleration — lowest-severity case",
            "vs t=8s / t=12s:  lowest tensions (68.3 N) and sag (35.3 mm)",
            "Same controller logic throughout — only the trajectory phase differs",
        ],
        "bottom": [
            "KEY METRICS",
            "3D RMSE = 0.313 m     |     Peak tension = 68.3 N",
            "Peak altitude sag = 35.3 mm     |     Smoothest recovery of the three cases",
            "Demonstrates: robustness holds uniformly across the full lemniscate period",
        ],
    },
    "s06_l1_g2000": {
        "top": [
            "L1 Adaptive Augmentation — gamma = 2000  [LOW GAIN]",
            "Slow adaptation: k_eff_hat converges gradually toward 2778 N/m nominal",
            "vs gamma=30000 / gamma=50000:  slower stiffness estimate — same RMSE",
            "C3 theorem: gamma = 2000 satisfies closed-form gain bound gamma*dt*p_v <= 1",
        ],
        "bottom": [
            "KEY METRICS",
            "3D RMSE = 0.297 m     |     L1 gain condition satisfied",
            "k_eff_hat (purple, right axis) tracks nominal stiffness 2778 N/m",
            "Watch: slow but correct convergence — theorem certifies this gain",
        ],
    },
    "s07_l1_g30000": {
        "top": [
            "L1 Adaptive Augmentation — gamma = 30000  [MID GAIN]",
            "Faster stiffness adaptation vs gamma=2000: tighter k_eff_hat tracking",
            "vs gamma=2000 / gamma=50000:  same RMSE = 0.297 m — plateau confirmed",
            "C3 theorem: RMSE is invariant to gamma anywhere on the certified plateau",
        ],
        "bottom": [
            "KEY METRICS",
            "3D RMSE = 0.297 m     |     Plateau: RMSE invariant for gamma in [2000 50000]",
            "k_eff_hat converges faster — compare with gamma=2000 trace",
            "C3: gamma is certified by closed-form bound not by empirical tuning",
        ],
    },
    "s08_l1_g50000": {
        "top": [
            "L1 Adaptive Augmentation — gamma = 50000  [HIGH GAIN]",
            "Near the certified ceiling: gamma * dt * p_v approaching 1",
            "vs gamma=2000 / gamma=30000:  higher variance growth same plateau RMSE",
            "Demonstrates: beyond this bound adaptation would begin to destabilize",
        ],
        "bottom": [
            "KEY METRICS",
            "3D RMSE = 0.301 m     |     Near ceiling: gamma * dt * p_v approx 1",
            "Variance growth ratio highest — yet 0% actuator saturation throughout",
            "All 3 gamma cases: RMSE 0.297–0.301 m — flat plateau confirmed (C3)",
        ],
    },
}


# ── helpers ───────────────────────────────────────────────────────────────────

def run(cmd):
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    if result.returncode != 0:
        print("STDERR:", result.stderr[-3000:], file=sys.stderr)
        raise RuntimeError(f"Command failed:\n{cmd[:200]}")
    return result.stdout


def _esc(s):
    """Escape text for ffmpeg drawtext option value.

    ffmpeg's filter-option parser uses ':' as an option separator and '='
    as a key=value delimiter, so both must be backslash-escaped in the text
    body (the backslash itself must be doubled at the Python string level).
    """
    s = s.replace("\\", "\\\\")
    s = s.replace("'", "")       # strip single quotes — avoids quoting hell
    s = s.replace("%", "%%")    # drawtext format specifier
    s = s.replace(":", "\\:")   # ffmpeg filter option separator
    s = s.replace("=", "\\=")   # ffmpeg filter option assignment
    return s


def build_overlay_filter(top_lines, bottom_lines):
    """Return an ffmpeg -vf filter string that overlays text in both black bars.

    Text is centered in the left panel (0–960 px) of the 1920-wide composite.
    y positions target the black padding above and below the 960×448 sim video.
    """
    parts = []

    for i, line in enumerate(top_lines[:4]):
        y, fs, fc = TOP_YS[i], TOP_FS[i], TOP_COLORS[i]
        safe = _esc(line)
        parts.append(
            f"drawtext=text='{safe}'"
            f":fontcolor={fc}:fontsize={fs}"
            f":x=(960-text_w)/2:y={y}"
            f":box=1:boxcolor={BG}@0.85:boxborderw=8"
        )

    for i, line in enumerate(bottom_lines[:4]):
        y, fs, fc = BOT_YS[i], BOT_FS[i], BOT_COLORS[i]
        safe = _esc(line)
        parts.append(
            f"drawtext=text='{safe}'"
            f":fontcolor={fc}:fontsize={fs}"
            f":x=(960-text_w)/2:y={y}"
            f":box=1:boxcolor={BG}@0.85:boxborderw=8"
        )

    return ",".join(parts)


def apply_overlay(seg_name, in_path, out_path):
    cfg = OVERLAYS[seg_name]
    vf  = build_overlay_filter(cfg["top"], cfg["bottom"])
    print(f"  [{seg_name}] overlaying text...")
    run(
        f'ffmpeg -y -i "{in_path}" '
        f'-vf "{vf}" '
        f'-c:v libx264 -pix_fmt yuv420p -r {FPS} "{out_path}"'
    )


# ── ordered segment list ──────────────────────────────────────────────────────

def build_segment_list():
    """Return ordered list of segment paths for the final concat.

    Sync segments are replaced by their _ov (overlay) versions.
    """
    def p(name):
        return os.path.join(PRES, name)

    return [
        p("s00_title.mp4"),
        p("tr_problem.mp4"),
        p("s01_problem.mp4"),
        p("s02_arch.mp4"),
        p("tr_fault_sweep.mp4"),
        p("s03_fault_t8_ov.mp4"),
        p("s04_fault_t12_ov.mp4"),
        p("s05_fault_t16_ov.mp4"),
        p("s_ft_summary.mp4"),
        p("tr_ablation.mp4"),
        p("s_ablation.mp4"),
        p("tr_l1_sweep.mp4"),
        p("s06_l1_g2000_ov.mp4"),
        p("s07_l1_g30000_ov.mp4"),
        p("s08_l1_g50000_ov.mp4"),
        p("s_l1_summary.mp4"),
        p("tr_conclusion.mp4"),
        p("s_conclusion.mp4"),
    ]


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    print("=" * 65)
    print("Tether_Grace — Adding sim-panel context overlays")
    print("=" * 65)

    # Step 1: apply overlays to all 6 sync segments
    sync_segs = [
        "s03_fault_t8",
        "s04_fault_t12",
        "s05_fault_t16",
        "s06_l1_g2000",
        "s07_l1_g30000",
        "s08_l1_g50000",
    ]
    for seg in sync_segs:
        in_path  = os.path.join(PRES, f"{seg}.mp4")
        out_path = os.path.join(PRES, f"{seg}_ov.mp4")
        if not os.path.isfile(in_path):
            print(f"  WARNING: {in_path} not found — skipping", file=sys.stderr)
            continue
        apply_overlay(seg, in_path, out_path)

    # Step 2: concatenate
    segments = build_segment_list()
    missing  = [s for s in segments if not os.path.isfile(s)]
    if missing:
        print("ERROR: missing segments:", missing, file=sys.stderr)
        sys.exit(1)

    list_file    = os.path.join(PRES, "_concat_overlay.txt")
    final_output = os.path.join(OUT, "presentation_v2.mp4")

    with open(list_file, "w") as f:
        for v in segments:
            f.write(f"file '{v}'\n")

    print(f"\nConcatenating {len(segments)} segments → {final_output}")
    run(
        f'ffmpeg -y -f concat -safe 0 -i "{list_file}" '
        f'-c:v libx264 -pix_fmt yuv420p "{final_output}"'
    )

    dur = float(run(
        f'ffprobe -v quiet -show_entries format=duration '
        f'-of csv=p=0 "{final_output}"').strip())
    print(f"\n{'='*65}")
    print(f"  presentation_v2.mp4 ready: {dur:.1f} s  ({dur/60:.1f} min)")
    print(f"  Path: {final_output}")
    print(f"{'='*65}")


if __name__ == "__main__":
    main()
