#!/usr/bin/env python3
"""3D trajectory figure for V4 (canonical dual-fault scenario).

Replaces the bland top-down + side-view 2D projections (figs 6,7)
with one richer 3D figure that:
  - shows the V4 payload trajectory as a time-coded line
    (color = time → readers see the trajectory unwind);
  - overlays the lemniscate reference as a dashed black curve;
  - marks the two fault events with red crosses;
  - drops shadow projections on the (x-y) and (x-z) walls so
    the eye can read horizontal and vertical excursion at a
    glance without resorting to separate panels;
  - optionally shows the drone formation positions at three
    key instants (pre-fault cruise, between faults, post-fault
    recovery) as thin coloured stems linking the payload to
    each drone.

Output: IEEE_T-CST/Figures/fig_trajectory_3d_V4.png
"""
from __future__ import annotations
import sys
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (registers the projection)
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ieee_style import COLORS, setup_style  # noqa: E402
from plot_capability_demo import num_drones  # noqa: E402

setup_style()

CAP = Path("/workspaces/Tether_Grace/output/capability_demo")
FIG = Path("/workspaces/Tether_Grace/IEEE_T-CST/Figures")
FIG.mkdir(parents=True, exist_ok=True)

TAG = "V4_dual_5s_wind"
T1, T2 = 12.0, 17.0   # canonical V4 fault times
# Snapshots chosen to land at well-separated points on the
# 12-s-period lemniscate so the formation icons do not overlap:
#   t=10  s — pre-fault, formation on the right loop
#   t=15  s — between faults, after fault 1, formation on left loop
#   t=20  s — recovery, after fault 2, returning toward right loop
KEY_INSTANTS = [10.0, 15.0, 20.0]
T_START = 8.0
T_END = 30.0
QUAD_ARM = 0.18           # half-arm length (m) for drawn quadrotor
ROTOR_SIZE = 28           # marker size for rotor disks
PAYLOAD_SIZE = 80         # marker size for payload sphere


def _load() -> pd.DataFrame:
    return pd.read_csv(CAP / TAG / f"scenario_{TAG}.csv")


def _draw_quadcopter(ax, cx, cy, cz, color, body_alpha=1.0,
                     arm=QUAD_ARM):
    """Draw a simple quadrotor at (cx,cy,cz): four rotor disks at
    +x, -x, +y, -y arm endpoints joined by two cross-arms, and a
    small body marker at the center. The arms lie in a plane
    parallel to the world horizontal plane."""
    # Arm endpoints (4 rotors).
    px = [cx + arm, cx - arm, cx,        cx]
    py = [cy,        cy,        cy + arm, cy - arm]
    pz = [cz, cz, cz, cz]
    # Cross-arms.
    ax.plot([cx + arm, cx - arm], [cy, cy], [cz, cz],
            color=color, linewidth=1.6, alpha=body_alpha)
    ax.plot([cx, cx], [cy + arm, cy - arm], [cz, cz],
            color=color, linewidth=1.6, alpha=body_alpha)
    # Rotor disks.
    ax.scatter(px, py, pz, color=color, s=ROTOR_SIZE,
               edgecolor="black", linewidth=0.5,
               alpha=body_alpha, depthshade=False, zorder=10)
    # Body marker.
    ax.scatter([cx], [cy], [cz], color=color, s=ROTOR_SIZE * 0.7,
               edgecolor="black", linewidth=0.4, marker="s",
               alpha=body_alpha, depthshade=False, zorder=10)


def _draw_payload(ax, x, y, z, color):
    """Filled sphere proxy for the payload (a single 3D scatter
    with large size + no depth shading reads as a solid disk)."""
    ax.scatter([x], [y], [z], color=color, s=PAYLOAD_SIZE,
               edgecolor="black", linewidth=0.7, marker="o",
               alpha=0.95, depthshade=False, zorder=11)


def _color_line(ax, x, y, z, t, cmap_name="viridis", lw=1.4, label=None):
    """Plot a single 3D line whose colour codes the time index."""
    points = np.array([x, y, z]).T.reshape(-1, 1, 3)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    norm = plt.Normalize(t.min(), t.max())
    lc = Line3DCollection(segments, cmap=cmap_name, norm=norm,
                          linewidth=lw)
    lc.set_array(t)
    ax.add_collection3d(lc)
    if label is not None:
        ax.plot([], [], [], color=plt.colormaps[cmap_name](0.6), lw=lw,
                label=label)
    return lc


def main():
    df = _load()
    t = df["time"].values
    mask = (t >= T_START) & (t <= T_END)
    t_c = t[mask]
    px = df["payload_x"].values[mask]
    py = df["payload_y"].values[mask]
    pz = df["payload_z"].values[mask]
    rx = df["ref_x"].values[mask]
    ry = df["ref_y"].values[mask]
    rz = df["ref_z"].values[mask]

    fig = plt.figure(figsize=(10.0, 7.5))
    ax = fig.add_subplot(111, projection="3d")

    # Reference lemniscate (dashed black, slightly translucent).
    ax.plot(rx, ry, rz, color="black", linestyle="--", linewidth=0.8,
            alpha=0.75, label="reference (lemniscate)")

    # Payload trajectory, colour-coded by time.
    lc = _color_line(ax, px, py, pz, t_c, cmap_name="viridis",
                     lw=1.6, label="payload (colour = time)")

    # Wall projections — top-down (x-y) and side (x-z) shadows
    # so the reader picks up horizontal and vertical excursion
    # without reading separate panels.
    z_floor = pz.min() - 0.15
    y_back = py.max() + 0.6
    ax.plot(px, py, np.full_like(pz, z_floor),
            color="grey", linewidth=0.6, alpha=0.55)
    ax.plot(rx, ry, np.full_like(rz, z_floor),
            color="black", linewidth=0.5, alpha=0.4, linestyle=":")
    ax.plot(px, np.full_like(py, y_back), pz,
            color="grey", linewidth=0.6, alpha=0.55)
    ax.plot(rx, np.full_like(ry, y_back), rz,
            color="black", linewidth=0.5, alpha=0.4, linestyle=":")

    # Fault markers (red crosses).
    for tf, lab in [(T1, r"fault 1 ($t_1{=}12$ s)"),
                    (T2, r"fault 2 ($t_2{=}17$ s)")]:
        idx = int(np.argmin(np.abs(t_c - tf)))
        ax.scatter(px[idx], py[idx], pz[idx], marker="x", color="red",
                   s=70, linewidths=2.0, zorder=10, label=lab)

    # Drone-formation snapshots at three well-separated instants.
    N = num_drones(df)
    snap_colors = [COLORS[1], COLORS[2], COLORS[3]]
    snap_labels = [r"$t{=}10$ s (pre-fault, all 5 drones)",
                   r"$t{=}15$ s (after fault 1; drone 0 lost)",
                   r"$t{=}20$ s (after fault 2; drones 0, 2 lost)"]
    for snap_t, sc, sl in zip(KEY_INSTANTS, snap_colors, snap_labels):
        idx = int(np.argmin(np.abs(t - snap_t)))
        px_p = float(df["payload_x"].values[idx])
        py_p = float(df["payload_y"].values[idx])
        pz_p = float(df["payload_z"].values[idx])
        for i in range(N):
            qx = float(df[f"quad{i}_x"].values[idx])
            qy = float(df[f"quad{i}_y"].values[idx])
            qz = float(df[f"quad{i}_z"].values[idx])
            tens = float(df[f"tension_{i}"].values[idx])
            taut = tens > 0.5
            # Rope: solid if taut, dimmed if slack/severed.
            ax.plot([px_p, qx], [py_p, qy], [pz_p, qz],
                    color=sc,
                    alpha=0.85 if taut else 0.18,
                    linewidth=1.1 if taut else 0.7,
                    linestyle="-" if taut else (0, (2, 2)))
            # Quadcopter icon (faded if rope severed).
            _draw_quadcopter(ax, qx, qy, qz, color=sc,
                             body_alpha=0.95 if taut else 0.30)
        # Payload sphere.
        _draw_payload(ax, px_p, py_p, pz_p, color=sc)
        # One legend entry per snapshot (proxy artist).
        ax.plot([], [], [], color=sc, marker="s", linestyle="-",
                markersize=7, label=sl)

    # Gather drone coordinate ranges across all snapshots so the
    # camera box includes every quadrotor body without clipping.
    qxs, qys, qzs = [], [], []
    for snap_t in KEY_INSTANTS:
        idx = int(np.argmin(np.abs(t - snap_t)))
        for i in range(N):
            qxs.append(float(df[f"quad{i}_x"].values[idx]))
            qys.append(float(df[f"quad{i}_y"].values[idx]))
            qzs.append(float(df[f"quad{i}_z"].values[idx]))
    drone_pad = QUAD_ARM + 0.15  # extra room for drawn arms/disks

    ax.set_xlabel(r"$x$ $[\mathrm{m}]$", fontsize=11, labelpad=8)
    ax.set_ylabel(r"$y$ $[\mathrm{m}]$", fontsize=11, labelpad=8)
    ax.set_zlabel(r"$z$ $[\mathrm{m}]$", fontsize=11, labelpad=8)
    ax.tick_params(axis="both", which="major", labelsize=9)

    x_lo = min(px.min(), rx.min(), min(qxs)) - drone_pad
    x_hi = max(px.max(), rx.max(), max(qxs)) + drone_pad
    y_lo = min(py.min(), ry.min(), min(qys)) - drone_pad
    y_hi = max(py.max(), ry.max(), max(qys)) + drone_pad
    z_lo = z_floor
    z_hi = max(pz.max(), rz.max(), max(qzs)) + drone_pad
    ax.set_xlim(x_lo, x_hi)
    ax.set_ylim(y_lo, y_hi)
    ax.set_zlim(z_lo, z_hi)
    ax.view_init(elev=22, azim=-58)

    # Place the time colour-bar INSIDE the axes (top-right inset)
    # so the 3D box gets the full canvas width.
    cax = ax.inset_axes([0.82, 0.78, 0.17, 0.025])
    cbar = fig.colorbar(lc, cax=cax, orientation="horizontal")
    cbar.set_label(r"time $[\mathrm{s}]$", fontsize=8)
    cbar.ax.tick_params(labelsize=7)
    cbar.outline.set_linewidth(0.5)

    # Legend below the axes (two rows, 3-4 columns) so it never
    # overlaps the trajectory.
    handles, labels = ax.get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=3,
               fontsize=9, framealpha=0.92,
               bbox_to_anchor=(0.5, 0.005))
    ax.set_title("V4 dual-fault payload trajectory in 3D",
                 fontsize=11, pad=2)
    fig.subplots_adjust(left=0.0, right=1.0, bottom=0.13, top=0.97)
    fig.savefig(FIG / "fig_trajectory_3d_V4.png", dpi=220,
                bbox_inches="tight", pad_inches=0.05)
    plt.close(fig)
    print(f"  wrote {FIG / 'fig_trajectory_3d_V4.png'}")


if __name__ == "__main__":
    main()
