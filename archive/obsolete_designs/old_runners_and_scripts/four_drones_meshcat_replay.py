#!/usr/bin/env python3
"""
4-Drone Scenario Meshcat Replay

Loads CSV trajectory data and animates the simulation using Drake's Meshcat.
Usage:
    python3 four_drones_meshcat_replay.py <csv_file>
    python3 four_drones_meshcat_replay.py four_drones_scenario_A.csv
"""

import sys
import time
import csv
from pathlib import Path

import numpy as np

# Ensure Drake Python bindings are discoverable (Drake binary install location)
_DRAKE_PY = "/opt/drake/lib/python3.10/site-packages"
if Path(_DRAKE_PY).exists() and _DRAKE_PY not in sys.path:
    sys.path.insert(0, _DRAKE_PY)

try:
    from pydrake.geometry import Meshcat, Box, Sphere, Rgba
    from pydrake.math import RigidTransform
except ImportError as e:
    print(f"ERROR: pydrake not available: {e}")
    print(f"Check that Drake is installed at /opt/drake")
    sys.exit(1)


def load_csv(csv_path):
    """Load CSV trajectory data into numpy array with headers."""
    with open(csv_path, 'r') as f:
        reader = csv.reader(f)
        headers = next(reader)
        rows = list(reader)

    # Parse data
    data = {}
    for i, name in enumerate(headers):
        name = name.strip()
        if not name:
            continue
        values = []
        for row in rows:
            if i < len(row):
                val = row[i].strip()
                try:
                    values.append(float(val))
                except ValueError:
                    values.append(val)
        data[name] = values

    return data


def setup_scene(meshcat, num_beads=8):
    """Setup visual geometry in Meshcat."""
    # Ground plane
    meshcat.SetObject(
        "/ground",
        Box(20, 20, 0.02),
        Rgba(0.7, 0.7, 0.7, 0.5)
    )
    meshcat.SetTransform("/ground", RigidTransform([0, 0, -0.01]))

    # 4 Quadcopters (colored spheres with markers)
    colors = [
        Rgba(1.0, 0.2, 0.2, 1.0),  # Red - drone 0
        Rgba(0.2, 1.0, 0.2, 1.0),  # Green - drone 1
        Rgba(0.2, 0.2, 1.0, 1.0),  # Blue - drone 2
        Rgba(1.0, 1.0, 0.2, 1.0),  # Yellow - drone 3
    ]
    for i, color in enumerate(colors):
        meshcat.SetObject(
            f"/quad_{i}",
            Box(0.3, 0.3, 0.08),
            color
        )

    # Payload (large orange sphere)
    meshcat.SetObject(
        "/payload",
        Sphere(0.15),
        Rgba(1.0, 0.5, 0.0, 1.0)
    )

    # Rope beads (small dark spheres)
    for rope in range(4):
        for b in range(num_beads):
            meshcat.SetObject(
                f"/rope_{rope}/bead_{b}",
                Sphere(0.03),
                Rgba(0.2, 0.2, 0.2, 0.8)
            )

    # Add text indicator for thrust levels (conceptual)
    meshcat.SetProperty("/Background", "top_color", [0.4, 0.5, 0.7])
    meshcat.SetProperty("/Background", "bottom_color", [0.7, 0.8, 0.9])


def animate(meshcat, data, num_beads=8, realtime_rate=1.0, target_fps=30):
    """Animate the trajectory in Meshcat with downsampling."""
    times = data['time']
    n_samples = len(times)

    # Downsample to target frame rate
    duration = times[-1] - times[0]
    target_frames = max(10, int(duration * target_fps))
    stride = max(1, n_samples // target_frames)
    frame_indices = list(range(0, n_samples, stride))
    if frame_indices[-1] != n_samples - 1:
        frame_indices.append(n_samples - 1)

    # Start recording for replay
    meshcat.StartRecording(set_visualizations_while_recording=False)

    print(f"Animating {len(frame_indices)} frames (downsampled from {n_samples} samples)...")

    for idx, i in enumerate(frame_indices):
        t = times[i]

        # Update quadcopter positions
        for d in range(4):
            x = data[f'quad{d}_x'][i]
            y = data[f'quad{d}_y'][i]
            z = data[f'quad{d}_z'][i]
            meshcat.SetTransform(
                f"/quad_{d}",
                RigidTransform([x, y, z]),
                time_in_recording=t
            )

        # Update payload position
        px = data['payload_x'][i]
        py = data['payload_y'][i]
        pz = data['payload_z'][i]
        meshcat.SetTransform(
            "/payload",
            RigidTransform([px, py, pz]),
            time_in_recording=t
        )

        # Update rope beads
        for rope in range(4):
            for b in range(num_beads):
                bx = data[f'rope{rope}_bead{b}_x'][i]
                by = data[f'rope{rope}_bead{b}_y'][i]
                bz = data[f'rope{rope}_bead{b}_z'][i]
                meshcat.SetTransform(
                    f"/rope_{rope}/bead_{b}",
                    RigidTransform([bx, by, bz]),
                    time_in_recording=t
                )

    meshcat.StopRecording()
    meshcat.PublishRecording()

    print("✓ Recording published to Meshcat")
    print(f"Open the browser at the Meshcat URL shown above to view.")


def main():
    if len(sys.argv) < 2:
        csv_path = "four_drones_replay.csv"
    else:
        csv_path = sys.argv[1]

    if not Path(csv_path).exists():
        print(f"ERROR: File not found: {csv_path}")
        sys.exit(1)

    print(f"Loading: {csv_path}")
    data = load_csv(csv_path)

    # Extract scenario name
    scenario = data.get('scenario', ['unknown'])[0] if data.get('scenario') else 'unknown'
    print(f"Scenario: {scenario}")
    print(f"Duration: {data['time'][-1]:.2f}s")
    print(f"Samples: {len(data['time'])}")

    # Start Meshcat server
    print("\nStarting Meshcat server...")
    meshcat = Meshcat()
    print(f"Meshcat web URL: {meshcat.web_url()}")
    print("Open this URL in your browser to see the animation.")

    # Setup scene
    setup_scene(meshcat)

    # Animate immediately (no user input prompt to support automation)
    animate(meshcat, data)

    print("\nAnimation complete. The replay can be played/paused in the Meshcat browser.")
    print("Press Ctrl+C to exit (server will stop).")

    # Keep server alive
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nExiting.")


if __name__ == "__main__":
    main()
