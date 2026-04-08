#!/usr/bin/env python3
"""Render 3D scenario snapshots from full-Drake trajectory CSV data.

Produces publication-quality 3D plots showing drones, payload, cables,
and formation geometry at three key time points per scenario.
"""

import csv
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Line3DCollection

# ── Configuration ──────────────────────────────────────────────────────

SCENARIOS = {
    'three_drones': {
        'csv': '/workspaces/Tether_Grace/outputs/full_drake_fault_batch/three_drones/logs/20260403_154505/trajectories.csv',
        'n_agents': 3,
        'fault_cables': [0],
        'fault_times': [7.0],
        'cable_lengths': [1.00, 1.08, 1.16],
        'snapshots': [
            (6.0,  'Pre-fault: all cables intact'),
            (7.5,  '0.5 s after cable 0 snaps'),
            (12.0, 'Settled: 2 agents carry payload'),
        ],
    },
    'five_drones': {
        'csv': '/workspaces/Tether_Grace/outputs/full_drake_fault_batch/five_drones/logs/20260403_155418/trajectories.csv',
        'n_agents': 5,
        'fault_cables': [1, 3],
        'fault_times': [7.0, 12.0],
        'cable_lengths': [1.00, 1.05, 1.10, 1.14, 1.18],
        'snapshots': [
            (6.0,  'Pre-fault: pentagon formation'),
            (8.0,  '1 s after cable 1 snaps'),
            (15.0, 'Settled: 3 agents carry payload'),
        ],
    },
    'seven_drones': {
        'csv': '/workspaces/Tether_Grace/outputs/full_drake_fault_batch/seven_drones/logs/20260403_161759/trajectories.csv',
        'n_agents': 7,
        'fault_cables': [1, 3, 5],
        'fault_times': [7.0, 12.0, 14.0],
        'cable_lengths': [0.98, 1.01, 1.04, 1.07, 1.10, 1.13, 1.16],
        'snapshots': [
            (6.0,  'Pre-fault: heptagon formation'),
            (8.0,  '1 s after cable 1 snaps'),
            (18.0, 'Settled: 4 agents carry payload'),
        ],
    },
}

OUTPUT_DIR = '/workspaces/Tether_Grace/IEEE_T-CST/Figures/scenario_snapshots'
os.makedirs(OUTPUT_DIR, exist_ok=True)


# ── Data loading ───────────────────────────────────────────────────────

def load_trajectories(csv_path, n_agents):
    """Load trajectory CSV and return dict of time -> positions."""
    times = []
    load_pos = []
    drone_pos = [[] for _ in range(n_agents)]

    with open(csv_path, 'r') as f:
        reader = csv.reader(f)
        header = next(reader)
        for row in reader:
            vals = [float(v) for v in row]
            t = vals[0]
            # Load: columns 1,2,3 = x,y,z
            lx, ly, lz = vals[1], vals[2], vals[3]
            times.append(t)
            load_pos.append([lx, ly, lz])
            # Each drone block: 13 columns (x,y,z,vx,vy,vz,qw,qx,qy,qz,wx,wy,wz)
            # First drone starts at column 14 (0-indexed: 14,15,16 = x,y,z)
            for i in range(n_agents):
                base = 14 + i * 13
                dx, dy, dz = vals[base], vals[base + 1], vals[base + 2]
                drone_pos[i].append([dx, dy, dz])

    times = np.array(times)
    load_pos = np.array(load_pos)
    drone_pos = [np.array(dp) for dp in drone_pos]
    return times, load_pos, drone_pos


def get_snapshot(times, load_pos, drone_pos, t_target):
    """Get positions at the nearest time to t_target."""
    idx = np.argmin(np.abs(times - t_target))
    return load_pos[idx], [dp[idx] for dp in drone_pos], times[idx]


def get_faulted_cables_at(fault_cables, fault_times, t):
    """Return set of cable indices that have faulted by time t."""
    faulted = set()
    for cable, ft in zip(fault_cables, fault_times):
        if t >= ft:
            faulted.add(cable)
    return faulted


# ── 3D Rendering ───────────────────────────────────────────────────────

def draw_drone(ax, pos, color='royalblue', size=80, marker='s', label=None,
               alpha=1.0, edgecolor='black', zorder=10):
    """Draw a drone marker in 3D."""
    ax.scatter(*pos, c=color, s=size, marker=marker, edgecolors=edgecolor,
              linewidths=0.8, alpha=alpha, zorder=zorder, label=label,
              depthshade=False)


def draw_payload(ax, pos, size=120):
    """Draw payload sphere marker."""
    ax.scatter(*pos, c='darkorange', s=size, marker='o', edgecolors='black',
              linewidths=1.0, zorder=12, depthshade=False)


def draw_cable(ax, drone_pos, load_pos, healthy=True, alpha=0.8):
    """Draw cable line from payload to drone."""
    xs = [load_pos[0], drone_pos[0]]
    ys = [load_pos[1], drone_pos[1]]
    zs = [load_pos[2], drone_pos[2]]
    if healthy:
        ax.plot(xs, ys, zs, '-', color='gray', linewidth=1.2, alpha=alpha, zorder=5)
    else:
        ax.plot(xs, ys, zs, '--', color='red', linewidth=1.0, alpha=0.4, zorder=5)


def draw_ground_shadow(ax, pos, color='gray', alpha=0.15, size=30):
    """Draw a shadow on the ground plane (z=0)."""
    ax.scatter(pos[0], pos[1], 0, c=color, s=size, marker='o', alpha=alpha,
              zorder=1, depthshade=False)


def draw_vertical_line(ax, pos, color='gray', alpha=0.1):
    """Draw faint vertical line from ground to position."""
    ax.plot([pos[0], pos[0]], [pos[1], pos[1]], [0, pos[2]],
            ':', color=color, linewidth=0.5, alpha=alpha, zorder=1)


def render_snapshot(ax, load_p, drone_ps, n_agents, faulted_set, fault_cables,
                    title, scenario_name):
    """Render a single 3D snapshot on the given axes."""

    # Draw ground plane grid
    grid_range = 3.0
    for x in np.arange(-grid_range, grid_range + 0.5, 1.0):
        ax.plot([x, x], [-grid_range, grid_range], [0, 0],
                '-', color='lightgray', linewidth=0.3, alpha=0.3)
        ax.plot([-grid_range, grid_range], [x, x], [0, 0],
                '-', color='lightgray', linewidth=0.3, alpha=0.3)

    # Draw payload
    draw_payload(ax, load_p)
    draw_ground_shadow(ax, load_p, color='darkorange', alpha=0.1, size=40)

    # Draw each drone
    for i in range(n_agents):
        dp = drone_ps[i]
        is_faulted = i in faulted_set

        if is_faulted:
            color = 'red'
            marker = 'X'
            edge = 'darkred'
            alpha = 0.7
        else:
            color = 'royalblue'
            marker = 's'
            edge = 'navy'
            alpha = 1.0

        draw_drone(ax, dp, color=color, marker=marker, edgecolor=edge, alpha=alpha)
        draw_ground_shadow(ax, dp, color=color, alpha=0.08)
        draw_vertical_line(ax, dp, color=color, alpha=0.15)

        # Cable
        if not is_faulted:
            draw_cable(ax, dp, load_p, healthy=True)
        else:
            # Draw broken cable stub (short dashed line toward payload)
            direction = np.array(load_p) - np.array(dp)
            dist = np.linalg.norm(direction)
            if dist > 0.3:
                stub_end = np.array(dp) + direction * 0.25 / dist
                ax.plot([dp[0], stub_end[0]], [dp[1], stub_end[1]],
                        [dp[2], stub_end[2]], '--', color='red',
                        linewidth=0.8, alpha=0.4)

        # Label
        offset = 0.12
        ax.text(dp[0] + offset, dp[1] + offset, dp[2] + 0.08,
                f'$Q_{i}$', fontsize=7, color=color if is_faulted else 'navy',
                fontweight='bold' if is_faulted else 'normal',
                ha='left', va='bottom', zorder=15)

    # Label payload
    ax.text(load_p[0] - 0.15, load_p[1] - 0.15, load_p[2] - 0.2,
            '$L$', fontsize=8, color='darkorange', fontweight='bold',
            ha='center', va='top', zorder=15)

    ax.set_title(title, fontsize=9, pad=2)


def set_axes_properties(ax, load_p, drone_ps, n_agents):
    """Set consistent axes limits and viewing angle."""
    # Compute bounding box centered on load
    all_pts = [load_p] + list(drone_ps)
    all_pts = np.array(all_pts)

    cx, cy = load_p[0], load_p[1]
    half_range = 2.5  # meters from center

    ax.set_xlim(cx - half_range, cx + half_range)
    ax.set_ylim(cy - half_range, cy + half_range)
    ax.set_zlim(0, 5.0)

    ax.set_xlabel('$x$ [m]', fontsize=7, labelpad=-2)
    ax.set_ylabel('$y$ [m]', fontsize=7, labelpad=-2)
    ax.set_zlabel('$z$ [m]', fontsize=7, labelpad=-2)

    ax.tick_params(axis='both', labelsize=6, pad=-2)
    ax.view_init(elev=25, azim=-55)

    # Lighter background
    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False
    ax.xaxis.pane.set_edgecolor('lightgray')
    ax.yaxis.pane.set_edgecolor('lightgray')
    ax.zaxis.pane.set_edgecolor('lightgray')
    ax.grid(True, alpha=0.2)


# ── Main rendering loop ───────────────────────────────────────────────

def render_scenario(name, config):
    """Render all three snapshots for one scenario as a single figure."""
    print(f"Rendering {name}...")

    times, load_pos, drone_pos = load_trajectories(config['csv'], config['n_agents'])

    fig = plt.figure(figsize=(15, 4.5), dpi=200)

    for panel_idx, (t_snap, subtitle) in enumerate(config['snapshots']):
        ax = fig.add_subplot(1, 3, panel_idx + 1, projection='3d')

        load_p, drone_ps, actual_t = get_snapshot(times, load_pos, drone_pos, t_snap)
        faulted = get_faulted_cables_at(
            config['fault_cables'], config['fault_times'], actual_t)

        panel_label = chr(ord('a') + panel_idx)
        title = f"({panel_label}) $t = {actual_t:.1f}$ s\n{subtitle}"

        render_snapshot(ax, load_p, drone_ps, config['n_agents'],
                       faulted, config['fault_cables'], title, name)
        set_axes_properties(ax, load_p, drone_ps, config['n_agents'])

    fig.suptitle(f"$N = {config['n_agents']}$ scenario: "
                 f"cables {config['fault_cables']} faulted at "
                 f"$t = {config['fault_times']}$ s",
                 fontsize=11, fontweight='bold', y=0.98)

    plt.tight_layout(rect=[0, 0, 1, 0.93])

    out_path = os.path.join(OUTPUT_DIR, f'scenario_{name}.png')
    fig.savefig(out_path, dpi=200, bbox_inches='tight',
                facecolor='white', edgecolor='none')
    plt.close(fig)
    print(f"  Saved: {out_path}")

    # Also save individual panels at higher DPI for flexibility
    for panel_idx, (t_snap, subtitle) in enumerate(config['snapshots']):
        fig_single = plt.figure(figsize=(5.5, 4.5), dpi=250)
        ax = fig_single.add_subplot(111, projection='3d')

        load_p, drone_ps, actual_t = get_snapshot(times, load_pos, drone_pos, t_snap)
        faulted = get_faulted_cables_at(
            config['fault_cables'], config['fault_times'], actual_t)

        panel_label = chr(ord('a') + panel_idx)
        title = f"({panel_label}) $t = {actual_t:.1f}$ s — {subtitle}"

        render_snapshot(ax, load_p, drone_ps, config['n_agents'],
                       faulted, config['fault_cables'], title, name)
        set_axes_properties(ax, load_p, drone_ps, config['n_agents'])

        panel_path = os.path.join(OUTPUT_DIR,
                                   f'scenario_{name}_panel_{panel_label}.png')
        fig_single.savefig(panel_path, dpi=250, bbox_inches='tight',
                           facecolor='white', edgecolor='none')
        plt.close(fig_single)

    print(f"  Individual panels saved")


if __name__ == '__main__':
    for name, config in SCENARIOS.items():
        render_scenario(name, config)
    print("\nAll scenario snapshots rendered.")
