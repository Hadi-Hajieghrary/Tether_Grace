#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import shutil
import subprocess
from pathlib import Path
from typing import Sequence

import matplotlib
import numpy as np
from experiment_manifest import DEFAULT_FULL_DRAKE_BATCH_MANIFEST, ExperimentScenario, load_scenarios

matplotlib.use("Agg")
import matplotlib.pyplot as plt


ROOT = Path("/workspaces/Tether_Grace")
OUTPUT_ROOT = ROOT / "outputs" / "full_drake_fault_batch"
BUILD_EXE = ROOT / "build" / "full_drake_fault_runner"
FIGURE_DPI = 220
LIFTOFF_HEIGHT_DELTA = 0.05
LIFTOFF_HOLD_TIME = 0.1

plt.rcParams.update(
    {
        "figure.facecolor": "white",
        "axes.facecolor": "#fbfbfb",
        "axes.edgecolor": "#333333",
        "axes.grid": True,
        "grid.alpha": 0.22,
        "grid.linestyle": "--",
        "axes.titlesize": 14,
        "axes.labelsize": 11,
        "legend.fontsize": 9,
        "xtick.labelsize": 10,
        "ytick.labelsize": 10,
    }
)


def build_waypoints(initial_altitude: float, duration: float) -> list[tuple[np.ndarray, float, float]]:
    t1 = 0.08 * duration
    t2 = 0.18 * duration
    t3 = 0.28 * duration
    t4 = 0.38 * duration
    t5 = 0.48 * duration
    t6 = 0.58 * duration
    t7 = 0.68 * duration
    t8 = 0.78 * duration
    t9 = 0.85 * duration
    t10 = 0.90 * duration
    return [
        (np.array([0.0, 0.0, initial_altitude]), 0.0, 1.0),
        (np.array([0.0, 0.0, 2.8]), t1, 0.4),
        (np.array([1.6, 0.5, 3.0]), t2, 0.2),
        (np.array([2.4, 1.4, 3.2]), t3, 0.2),
        (np.array([3.0, 0.0, 3.1]), t4, 0.2),
        (np.array([2.1, -1.4, 2.9]), t5, 0.2),
        (np.array([0.0, 0.0, 3.0]), t6, 0.2),
        (np.array([-1.6, 0.5, 3.1]), t7, 0.2),
        (np.array([-2.6, 1.4, 3.3]), t8, 0.2),
        (np.array([-3.0, 0.0, 3.1]), t9, 0.2),
        (np.array([-2.0, -1.2, 2.9]), t10, 0.2),
        (np.array([0.0, 0.0, 2.6]), duration, 0.0),
    ]


def evaluate_waypoint_trajectory(
    waypoints: list[tuple[np.ndarray, float, float]], time_value: float
) -> tuple[np.ndarray, np.ndarray]:
    segment_start_time = 0.0
    for index, (position, arrival_time, hold_time) in enumerate(waypoints):
        hold_end_time = arrival_time + hold_time
        if time_value <= arrival_time:
            if index == 0:
                return position.copy(), np.zeros(3, dtype=float)
            previous_position, _, _ = waypoints[index - 1]
            segment_duration = arrival_time - segment_start_time
            if segment_duration > 1e-6:
                alpha = (time_value - segment_start_time) / segment_duration
                return (1.0 - alpha) * previous_position + alpha * position, (position - previous_position) / segment_duration
            return position.copy(), np.zeros(3, dtype=float)
        if time_value <= hold_end_time:
            return position.copy(), np.zeros(3, dtype=float)
        segment_start_time = hold_end_time
    return waypoints[-1][0].copy(), np.zeros(3, dtype=float)


def build_load_reference(time_values: np.ndarray, duration: float) -> np.ndarray:
    waypoints = build_waypoints(initial_altitude=1.2, duration=duration)
    reference = np.zeros((time_values.shape[0], 3), dtype=float)
    for index, time_value in enumerate(time_values):
        reference[index], _ = evaluate_waypoint_trajectory(waypoints, float(time_value))
    return reference


def infer_payload_z_offset(
    time_values: np.ndarray,
    base_reference_z: np.ndarray,
    actual_load_z: np.ndarray,
    first_fault_time: float,
) -> float:
    window_start = min(2.0, max(float(time_values[0]), float(first_fault_time) * 0.25))
    window_end = max(window_start + 0.5, float(first_fault_time) - 0.5)
    mask = (time_values >= window_start) & (time_values <= window_end)
    if not np.any(mask):
        mask = time_values <= max(float(first_fault_time) - 0.5, float(time_values[min(len(time_values) - 1, 200)]))
    if not np.any(mask):
        return float(np.median(base_reference_z - actual_load_z))
    return float(np.median(base_reference_z[mask] - actual_load_z[mask]))


def infer_payload_liftoff_time(time_values: np.ndarray, actual_load_z: np.ndarray) -> float:
    initial_height = float(actual_load_z[0])
    threshold = initial_height + LIFTOFF_HEIGHT_DELTA
    for index, sample_time in enumerate(time_values):
        if actual_load_z[index] < threshold:
            continue
        end_time = float(sample_time) + LIFTOFF_HOLD_TIME
        end_index = min(int(np.searchsorted(time_values, end_time, side="left")), len(time_values) - 1)
        if np.all(actual_load_z[index : end_index + 1] >= threshold):
            return float(sample_time)
    return float(time_values[0])


def build_payload_reference(
    time_values: np.ndarray,
    duration: float,
    actual_load_z: np.ndarray,
    first_fault_time: float,
) -> tuple[np.ndarray, float, float]:
    reference = build_load_reference(time_values, duration)
    z_offset = infer_payload_z_offset(time_values, reference[:, 2], actual_load_z, first_fault_time)
    reference[:, 2] -= z_offset
    liftoff_time = infer_payload_liftoff_time(time_values, actual_load_z)
    reference[time_values < liftoff_time, 2] = float(actual_load_z[0])
    return reference, z_offset, liftoff_time


def scenario_title(name: str) -> str:
    return name.replace("_", " ")


def save_figure(figure: plt.Figure, path: Path) -> None:
    figure.savefig(path, dpi=FIGURE_DPI, bbox_inches="tight", facecolor="white")
    plt.close(figure)


def new_axis_figure(width: float = 11.5, height: float = 4.8) -> tuple[plt.Figure, plt.Axes]:
    return plt.subplots(1, 1, figsize=(width, height), constrained_layout=True)


def add_fault_markers(axis: plt.Axes, fault_times: Sequence[float]) -> None:
    for fault_time in fault_times:
        axis.axvline(float(fault_time), color="#d62728", linestyle="--", linewidth=1.15, alpha=0.85)


def infer_nominal_quad_offsets(traj_headers: list[str], traj: np.ndarray, num_agents: int) -> np.ndarray:
    load_initial = traj[0, [traj_headers.index("load_x"), traj_headers.index("load_y"), traj_headers.index("load_z")]]
    offsets = np.zeros((num_agents, 3), dtype=float)
    for agent in range(num_agents):
        offsets[agent, :] = traj[0, [
            traj_headers.index(f"drone{agent}_x"),
            traj_headers.index(f"drone{agent}_y"),
            traj_headers.index(f"drone{agent}_z"),
        ]] - load_initial
    return offsets


def build_nominal_quad_reference(load_reference: np.ndarray, nominal_offsets: np.ndarray) -> np.ndarray:
    return load_reference[:, None, :] + nominal_offsets[None, :, :]


def build_health_schedule(scenario: Scenario, time: np.ndarray) -> np.ndarray:
    health = np.ones((time.shape[0], scenario.num_agents), dtype=float)
    for faulted_cable, fault_time in zip(scenario.fault_cables, scenario.fault_times):
        health[time >= float(fault_time), int(faulted_cable)] = 0.0
    return health


def clear_pngs(directory: Path) -> None:
    if not directory.exists():
        return
    for path in directory.glob("*.png"):
        path.unlink()


def ensure_output_root(clean: bool) -> None:
    if clean and OUTPUT_ROOT.exists():
        shutil.rmtree(OUTPUT_ROOT)
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)


def load_csv_matrix(path: Path) -> tuple[list[str], np.ndarray]:
    data = np.genfromtxt(path, delimiter=",", names=True)
    headers = list(data.dtype.names or [])
    if not headers:
        raise RuntimeError(f"No headers found in {path}")
    if data.ndim == 0:
        data = np.array([tuple(data)], dtype=data.dtype)
    matrix = np.column_stack([data[name] for name in headers])
    return headers, matrix


def run_drake_executable(scenario: ExperimentScenario) -> Path:
    if not BUILD_EXE.exists():
        raise FileNotFoundError(f"Expected built executable at {BUILD_EXE}")

    for event in scenario.fault_events:
        if event.profile not in {"snap", "step"} or abs(event.post_fault_scale) > 1e-9 or event.ramp_duration_seconds > 1e-9:
            raise NotImplementedError(
                "The current full-Drake runner supports only zero-scale instantaneous snap events. "
                f"Scenario '{scenario.name}' contains unsupported event {event}."
            )

    scenario_dir = OUTPUT_ROOT / scenario.name
    scenario_dir.mkdir(parents=True, exist_ok=True)
    command = [
        str(BUILD_EXE),
        "--num-quads",
        str(scenario.num_agents),
        "--duration",
        str(scenario.duration),
        "--output-dir",
        str(scenario_dir),
        "--cable-lengths",
        ",".join(f"{value:.3f}" for value in scenario.cable_lengths),
        "--headless",
        "--record-meshcat-html",
    ]
    if scenario.fault_cables:
        command += [
            "--fault-cables",
            ",".join(str(value) for value in scenario.fault_cables),
            "--fault-times",
            ",".join(f"{value:.3f}" for value in scenario.fault_times),
        ]
    # Append optional physics / controller overrides
    if scenario.max_thrust is not None:
        command += ["--max-thrust", str(scenario.max_thrust)]
    if scenario.payload_mass is not None:
        command += ["--payload-mass", str(scenario.payload_mass)]
    if scenario.tension_kp is not None:
        command += ["--tension-kp", str(scenario.tension_kp)]
    if scenario.disable_tension_ff:
        command += ["--disable-tension-ff"]
    if scenario.position_kp is not None:
        command += ["--position-kp", str(scenario.position_kp)]
    if scenario.position_kd is not None:
        command += ["--position-kd", str(scenario.position_kd)]
    if scenario.position_ki is not None:
        command += ["--position-ki", str(scenario.position_ki)]
    if scenario.enable_eso:
        command += ["--enable-eso"]
    if scenario.seed is not None:
        command += ["--seed", str(scenario.seed)]
    if scenario.enable_wind:
        command += ["--enable-wind"]
    if scenario.wind_mean is not None:
        command += ["--wind-mean"] + [str(v) for v in scenario.wind_mean]
    if scenario.wind_sigma is not None:
        command += ["--wind-sigma", str(scenario.wind_sigma)]
    if scenario.enable_eskf:
        command += ["--enable-eskf"]
    if scenario.imu_rate is not None:
        command += ["--imu-rate", str(scenario.imu_rate)]
    if scenario.gps_noise is not None:
        command += ["--gps-noise", str(scenario.gps_noise)]
    if scenario.baro_noise is not None:
        command += ["--baro-noise", str(scenario.baro_noise)]
    if scenario.gps_rate is not None:
        command += ["--gps-rate", str(scenario.gps_rate)]
    if scenario.baro_rate is not None:
        command += ["--baro-rate", str(scenario.baro_rate)]
    if scenario.trajectory_mode is not None:
        command += ["--trajectory", scenario.trajectory_mode]
    if scenario.enable_reactive_ftc:
        command += ["--enable-reactive-ftc"]
    if scenario.reactive_boost is not None:
        command += ["--reactive-boost", str(scenario.reactive_boost)]
    if scenario.oracle_load_share is not None:
        command += ["--oracle-load-share", str(scenario.oracle_load_share)]
    if scenario.num_rope_beads is not None:
        command += ["--num-rope-beads", str(scenario.num_rope_beads)]
    subprocess.run(command, check=True, cwd=ROOT)
    return scenario_dir


def normalize_meshcat_artifact(scenario_dir: Path) -> str:
    source = next(scenario_dir.glob("sim_recording_n*.html"), None)
    target = scenario_dir / "full_drake_meshcat_replay.html"
    if source is not None and source != target:
        if target.exists():
            target.unlink()
        source.rename(target)
    return str(target if target.exists() else source if source is not None else "")


def summarize_scenario(
    scenario: ExperimentScenario,
    manifest: dict[str, object],
    traj_headers: list[str],
    traj: np.ndarray,
    tension_headers: list[str],
    tension: np.ndarray,
    artifacts: dict[str, object],
) -> dict[str, object]:
    time = traj[:, 0]
    load_xyz = traj[:, [traj_headers.index("load_x"), traj_headers.index("load_y"), traj_headers.index("load_z")]]
    load_speed = np.linalg.norm(
        traj[:, [traj_headers.index("load_vx"), traj_headers.index("load_vy"), traj_headers.index("load_vz")]],
        axis=1,
    )
    peak_tensions = {
        header: float(np.max(tension[:, idx]))
        for idx, header in enumerate(tension_headers)
        if header.endswith("_mag")
    }
    return {
        "scenario": scenario.name,
        "num_agents": scenario.num_agents,
        "duration": scenario.duration,
        "fault_cables": scenario.fault_cables,
        "fault_times": scenario.fault_times,
        "cable_lengths": scenario.cable_lengths,
        "log_dir": manifest["log_dir"],
        "meshcat_html": manifest["meshcat_html"],
        "sample_count": int(time.shape[0]),
        "load_final_position": load_xyz[-1].tolist(),
        "load_peak_speed": float(np.max(load_speed)),
        "peak_tensions": peak_tensions,
        "artifacts": artifacts,
    }


def plot_scenario(
    scenario: ExperimentScenario,
    scenario_dir: Path,
    traj_headers: list[str],
    traj: np.ndarray,
    tension_headers: list[str],
    tension: np.ndarray,
    control_headers: list[str],
    control: np.ndarray,
    attitude_headers: list[str],
    attitude: np.ndarray,
) -> dict[str, str]:
    figure_dir = scenario_dir / "figures"
    figure_dir.mkdir(parents=True, exist_ok=True)
    clear_pngs(figure_dir)
    time = traj[:, 0]
    load_x = traj[:, traj_headers.index("load_x")]
    load_y = traj[:, traj_headers.index("load_y")]
    load_z = traj[:, traj_headers.index("load_z")]
    load_xyz = np.column_stack([load_x, load_y, load_z])
    first_fault_time = min(float(value) for value in scenario.fault_times) if scenario.fault_times else scenario.duration
    load_reference, payload_z_offset, payload_liftoff_time = build_payload_reference(
        time,
        scenario.duration,
        load_z,
        first_fault_time,
    )
    nominal_offsets = infer_nominal_quad_offsets(traj_headers, traj, scenario.num_agents)
    nominal_quad_reference = build_nominal_quad_reference(load_reference, nominal_offsets)
    cable_health = build_health_schedule(scenario, time)
    load_error_cm = 100.0 * np.linalg.norm(load_xyz - load_reference, axis=1)
    load_speed = np.linalg.norm(
        traj[:, [traj_headers.index("load_vx"), traj_headers.index("load_vy"), traj_headers.index("load_vz")]],
        axis=1,
    )

    quad_position_error = np.zeros((time.shape[0], scenario.num_agents), dtype=float)
    quad_speed = np.zeros((time.shape[0], scenario.num_agents), dtype=float)
    quad_force_norm = np.zeros((time.shape[0], scenario.num_agents), dtype=float)
    quad_lateral_force = np.zeros((time.shape[0], scenario.num_agents), dtype=float)
    quad_vertical_force = np.zeros((time.shape[0], scenario.num_agents), dtype=float)
    quad_torque_norm = np.zeros((time.shape[0], scenario.num_agents), dtype=float)
    quad_altitude = np.zeros((time.shape[0], scenario.num_agents), dtype=float)
    quad_tilt_deg = np.zeros((time.shape[0], scenario.num_agents), dtype=float)

    for agent in range(scenario.num_agents):
        quad_xyz = traj[:, [
            traj_headers.index(f"drone{agent}_x"),
            traj_headers.index(f"drone{agent}_y"),
            traj_headers.index(f"drone{agent}_z"),
        ]]
        quad_velocity = traj[:, [
            traj_headers.index(f"drone{agent}_vx"),
            traj_headers.index(f"drone{agent}_vy"),
            traj_headers.index(f"drone{agent}_vz"),
        ]]
        quad_force = control[:, [
            control_headers.index(f"drone{agent}_f_x"),
            control_headers.index(f"drone{agent}_f_y"),
            control_headers.index(f"drone{agent}_f_z"),
        ]]
        quad_torque = control[:, [
            control_headers.index(f"drone{agent}_tau_x"),
            control_headers.index(f"drone{agent}_tau_y"),
            control_headers.index(f"drone{agent}_tau_z"),
        ]]
        roll = attitude[:, attitude_headers.index(f"drone{agent}_roll")]
        pitch = attitude[:, attitude_headers.index(f"drone{agent}_pitch")]

        quad_position_error[:, agent] = np.linalg.norm(quad_xyz - nominal_quad_reference[:, agent, :], axis=1)
        quad_speed[:, agent] = np.linalg.norm(quad_velocity, axis=1)
        quad_force_norm[:, agent] = np.linalg.norm(quad_force, axis=1)
        quad_lateral_force[:, agent] = np.linalg.norm(quad_force[:, :2], axis=1)
        quad_vertical_force[:, agent] = quad_force[:, 2]
        quad_torque_norm[:, agent] = np.linalg.norm(quad_torque, axis=1)
        quad_altitude[:, agent] = quad_xyz[:, 2]
        quad_tilt_deg[:, agent] = np.degrees(np.sqrt(roll**2 + pitch**2))

    colors = [plt.get_cmap("tab10")(agent % 10) for agent in range(scenario.num_agents)]
    artifact_paths: dict[str, str] = {}

    payload_plots = [
        ("payload_x_tracking", load_reference[:, 0], load_x, "Load X tracking", "Position [m]", "desired x", "actual x", "#1f77b4"),
        ("payload_y_tracking", load_reference[:, 1], load_y, "Load Y tracking", "Position [m]", "desired y", "actual y", "#2ca02c"),
        ("payload_z_tracking", load_reference[:, 2], load_z, "Load Z tracking", "Position [m]", "desired z", "actual z", "#9467bd"),
    ]
    for key, desired, actual, title, ylabel, desired_label, actual_label, color in payload_plots:
        figure, axis = new_axis_figure()
        axis.plot(time, desired, "--", color="#4d4d4d", linewidth=1.8, label=desired_label)
        axis.plot(time, actual, color=color, linewidth=1.8, label=actual_label)
        add_fault_markers(axis, scenario.fault_times)
        axis.set_title(f"{scenario_title(scenario.name)} {title.lower()}")
        axis.set_xlabel("Time [s]")
        axis.set_ylabel(ylabel)
        axis.legend(loc="upper right")
        output_path = figure_dir / f"{key}.png"
        save_figure(figure, output_path)
        artifact_paths[key] = str(output_path)

    figure, axis = new_axis_figure()
    axis.plot(time, load_error_cm, color="#d62728", linewidth=2.0, label="tracking error")
    add_fault_markers(axis, scenario.fault_times)
    axis.set_title(f"{scenario_title(scenario.name)} payload tracking error")
    axis.set_xlabel("Time [s]")
    axis.set_ylabel("Error [cm]")
    axis.legend(loc="upper right")
    output_path = figure_dir / "payload_tracking_error.png"
    save_figure(figure, output_path)
    artifact_paths["payload_tracking_error"] = str(output_path)

    figure, axis = new_axis_figure(height=4.2)
    axis.plot(time, np.full_like(time, payload_z_offset), color="#8c564b", linewidth=1.8, label="inferred z offset")
    axis.axvline(payload_liftoff_time, color="#1f77b4", linestyle=":", linewidth=1.3, alpha=0.9, label="liftoff")
    add_fault_markers(axis, scenario.fault_times)
    axis.set_title(f"{scenario_title(scenario.name)} inferred payload z offset")
    axis.set_xlabel("Time [s]")
    axis.set_ylabel("Offset [m]")
    axis.legend(loc="upper right")
    output_path = figure_dir / "payload_z_offset.png"
    save_figure(figure, output_path)
    artifact_paths["payload_z_offset"] = str(output_path)

    figure, axis = new_axis_figure(height=5.2)
    for agent in range(scenario.num_agents):
        axis.plot(time, tension[:, tension_headers.index(f"rope{agent}_mag")], color=colors[agent], linewidth=1.5, label=f"Cable {agent}")
    add_fault_markers(axis, scenario.fault_times)
    axis.set_title(f"{scenario_title(scenario.name)} cable tensions")
    axis.set_xlabel("Time [s]")
    axis.set_ylabel("Tension [N]")
    axis.legend(loc="upper right", ncol=min(4, scenario.num_agents))
    output_path = figure_dir / "cable_tensions.png"
    save_figure(figure, output_path)
    artifact_paths["cable_tensions"] = str(output_path)

    figure, axis = new_axis_figure(height=4.6)
    for agent in range(scenario.num_agents):
        axis.plot(time, cable_health[:, agent], color=colors[agent], linewidth=1.5, label=f"Cable {agent}")
    add_fault_markers(axis, scenario.fault_times)
    axis.set_title(f"{scenario_title(scenario.name)} cable health schedule")
    axis.set_xlabel("Time [s]")
    axis.set_ylabel("Healthy = 1")
    axis.set_ylim(-0.05, 1.05)
    axis.legend(loc="upper right", ncol=min(4, scenario.num_agents))
    output_path = figure_dir / "cable_health_schedule.png"
    save_figure(figure, output_path)
    artifact_paths["cable_health_schedule"] = str(output_path)

    quad_state_plots = [
        ("quad_formation_deviation", quad_position_error, "Nominal formation deviation", "Deviation [m]"),
        ("quad_speed_norm", quad_speed, "Per-quad speed norm", "Speed [m/s]"),
        ("quad_tilt_magnitude", quad_tilt_deg, "Per-quad tilt magnitude", "Tilt [deg]"),
        ("quad_altitude", quad_altitude, "Per-quad altitude", "Altitude [m]"),
    ]
    for key, values, title, ylabel in quad_state_plots:
        figure, axis = new_axis_figure()
        for agent in range(scenario.num_agents):
            axis.plot(time, values[:, agent], color=colors[agent], linewidth=1.3, label=f"Quad {agent}")
        add_fault_markers(axis, scenario.fault_times)
        axis.set_title(f"{scenario_title(scenario.name)} {title.lower()}")
        axis.set_xlabel("Time [s]")
        axis.set_ylabel(ylabel)
        axis.legend(loc="upper right", ncol=min(4, scenario.num_agents))
        output_path = figure_dir / f"{key}.png"
        save_figure(figure, output_path)
        artifact_paths[key] = str(output_path)

    actuation_plots = [
        ("quad_force_norm", quad_force_norm, "Per-quad force norm", "Force [N]"),
        ("quad_lateral_force_norm", quad_lateral_force, "Per-quad lateral force norm", "Force [N]"),
        ("quad_torque_norm", quad_torque_norm, "Per-quad torque norm", "Torque [N m]"),
    ]
    for key, values, title, ylabel in actuation_plots:
        figure, axis = new_axis_figure()
        for agent in range(scenario.num_agents):
            axis.plot(time, values[:, agent], color=colors[agent], linewidth=1.3, label=f"Quad {agent}")
        add_fault_markers(axis, scenario.fault_times)
        axis.set_title(f"{scenario_title(scenario.name)} {title.lower()}")
        axis.set_xlabel("Time [s]")
        axis.set_ylabel(ylabel)
        axis.legend(loc="upper right", ncol=min(4, scenario.num_agents))
        output_path = figure_dir / f"{key}.png"
        save_figure(figure, output_path)
        artifact_paths[key] = str(output_path)

    total_vertical_force = np.sum(quad_vertical_force, axis=1)
    figure, axis = new_axis_figure()
    axis.plot(time, total_vertical_force, color="#111111", linewidth=2.0, label="Sum vertical force")
    axis.plot(time, load_speed, color="#17a589", linewidth=1.8, label="Load speed")
    add_fault_markers(axis, scenario.fault_times)
    axis.set_title(f"{scenario_title(scenario.name)} load speed and total vertical force")
    axis.set_xlabel("Time [s]")
    axis.set_ylabel("Magnitude")
    axis.legend(loc="upper right")
    output_path = figure_dir / "load_speed_and_total_vertical_force.png"
    save_figure(figure, output_path)
    artifact_paths["load_speed_and_total_vertical_force"] = str(output_path)

    figure, axis = new_axis_figure(width=10.2, height=6.0)
    axis.plot(load_reference[:, 0], load_reference[:, 1], "--", color="#4d4d4d", linewidth=2.0, label="Desired load")
    axis.plot(load_x, load_y, color="#2e8b57", linewidth=2.0, label="Actual load")
    for fault_time in scenario.fault_times:
        fault_index = min(int(np.searchsorted(time, float(fault_time), side="left")), len(time) - 1)
        axis.scatter(load_x[fault_index], load_y[fault_index], s=80, color="#d62728", alpha=0.85)
    axis.set_title(f"{scenario_title(scenario.name)} payload XY trajectory")
    axis.set_xlabel("x [m]")
    axis.set_ylabel("y [m]")
    axis.axis("equal")
    axis.legend(loc="upper right")
    output_path = figure_dir / "payload_xy_trajectory.png"
    save_figure(figure, output_path)
    artifact_paths["payload_xy_trajectory"] = str(output_path)

    figure, axis = new_axis_figure(width=10.2, height=6.0)
    final_load_reference = load_reference[-1, :2]
    for agent in range(scenario.num_agents):
        final_quad_actual = traj[-1, [traj_headers.index(f"drone{agent}_x"), traj_headers.index(f"drone{agent}_y")]]
        final_quad_nominal = final_load_reference + nominal_offsets[agent, :2]
        axis.scatter(final_quad_actual[0], final_quad_actual[1], color=colors[agent], s=95)
        axis.scatter(final_quad_nominal[0], final_quad_nominal[1], marker="x", color="#444444", s=160, linewidths=1.9)
        axis.plot(
            [final_quad_nominal[0], final_quad_actual[0]],
            [final_quad_nominal[1], final_quad_actual[1]],
            color=colors[agent],
            linewidth=1.2,
            alpha=0.9,
        )
        axis.text(final_quad_actual[0], final_quad_actual[1], str(agent), fontsize=11)
    axis.set_title(f"{scenario_title(scenario.name)} final quad positions")
    axis.set_xlabel("x [m]")
    axis.set_ylabel("y [m]")
    axis.axis("equal")
    output_path = figure_dir / "final_quad_positions.png"
    save_figure(figure, output_path)
    artifact_paths["final_quad_positions"] = str(output_path)

    figure, axis = new_axis_figure()
    axis.plot(time, load_z, color="#1f77b4", linewidth=2.0, label="load z actual")
    axis.plot(time, load_reference[:, 2], "--", color="#1f77b4", linewidth=1.8, label="load z ref")
    for agent in range(scenario.num_agents):
        axis.plot(time, quad_altitude[:, agent], color=colors[agent], linewidth=1.25, alpha=0.9, label=f"drone {agent} z")
    add_fault_markers(axis, scenario.fault_times)
    axis.set_title(f"{scenario_title(scenario.name)} vertical motion")
    axis.set_xlabel("Time [s]")
    axis.set_ylabel("z [m]")
    axis.legend(loc="upper right", ncol=min(4, scenario.num_agents + 2))
    output_path = figure_dir / "vertical_motion.png"
    save_figure(figure, output_path)
    artifact_paths["vertical_motion"] = str(output_path)

    figure, axis = new_axis_figure(width=10.2, height=6.0)
    axis.plot(load_x, load_y, color="#ff7f0e", linewidth=2.2, label="load actual")
    axis.plot(load_reference[:, 0], load_reference[:, 1], "--", color="#1f77b4", linewidth=2.0, label="load ref")
    for agent in range(scenario.num_agents):
        drone_x = traj[:, traj_headers.index(f"drone{agent}_x")]
        drone_y = traj[:, traj_headers.index(f"drone{agent}_y")]
        axis.plot(drone_x, drone_y, color=colors[agent], linewidth=1.25, alpha=0.85, label=f"drone {agent} path")
        axis.scatter(drone_x[0], drone_y[0], s=18, color=colors[agent], alpha=0.6)
        axis.scatter(drone_x[-1], drone_y[-1], s=38, marker="x", color=colors[agent])
    axis.set_title(f"{scenario_title(scenario.name)} XY trajectories")
    axis.set_xlabel("x [m]")
    axis.set_ylabel("y [m]")
    axis.axis("equal")
    axis.legend(loc="upper right", ncol=min(4, scenario.num_agents + 2))
    output_path = figure_dir / "xy_trajectories.png"
    save_figure(figure, output_path)
    artifact_paths["xy_trajectories"] = str(output_path)

    return artifact_paths


def run_scenario(scenario: ExperimentScenario, rerun_simulation: bool) -> dict[str, object]:
    scenario_dir = OUTPUT_ROOT / scenario.name
    if rerun_simulation:
        scenario_dir = run_drake_executable(scenario)
    manifest = json.loads((scenario_dir / "run_manifest.json").read_text(encoding="utf-8"))
    manifest["meshcat_html"] = normalize_meshcat_artifact(scenario_dir)
    (scenario_dir / "run_manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    log_dir = Path(manifest["log_dir"])

    traj_headers, traj = load_csv_matrix(log_dir / "trajectories.csv")
    tension_headers, tension = load_csv_matrix(log_dir / "tensions.csv")
    control_headers, control = load_csv_matrix(log_dir / "control_efforts.csv")
    attitude_headers, attitude = load_csv_matrix(log_dir / "attitude_data.csv")

    recording_path = scenario_dir / "full_drake_recording.npz"
    np.savez_compressed(
        recording_path,
        trajectories=traj,
        trajectory_headers=np.array(traj_headers, dtype=object),
        tensions=tension,
        tension_headers=np.array(tension_headers, dtype=object),
        controls=control,
        control_headers=np.array(control_headers, dtype=object),
        attitudes=attitude,
        attitude_headers=np.array(attitude_headers, dtype=object),
    )

    artifact_paths = plot_scenario(
        scenario,
        scenario_dir,
        traj_headers,
        traj,
        tension_headers,
        tension,
        control_headers,
        control,
        attitude_headers,
        attitude,
    )
    artifacts = {
        "meshcat_replay_html": manifest["meshcat_html"],
        "full_drake_recording_npz": str(recording_path),
        "figure_paths": artifact_paths,
    }
    summary = summarize_scenario(scenario, manifest, traj_headers, traj, tension_headers, tension, artifacts)
    (scenario_dir / "scenario_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return summary


def write_batch_manifest(summaries: Sequence[dict[str, object]], manifest_path: Path) -> None:
    manifest = {
        "generator": "experiments/python/run_fault_schedule_batch.py",
        "simulation_type": "full_drake_multibody",
        "shared_manifest_path": str(manifest_path),
        "scenarios": list(summaries),
    }
    (OUTPUT_ROOT / "batch_manifest.json").write_text(json.dumps(manifest, indent=2))


def write_readme(summaries: Sequence[dict[str, object]]) -> None:
    lines = [
        "# Full Drake Fault Batch",
        "",
        "This folder contains only root-owned full-Drake multibody scenario artifacts.",
        "",
        "## Top-level files",
        "",
        "- batch_manifest.json: scenario manifest for the regenerated batch.",
        "- comparison_metrics.json: structured metrics emitted by the analysis pass.",
        "- comparison_report.md: paper-oriented markdown summary of the regenerated batch.",
        "- comparison_scenarios.csv: one row per scenario.",
        "- comparison_fault_events.csv: one row per fault event.",
        "- comparison_load_tracking_summary.png: batch tracking summary.",
        "- comparison_fault_response_timing.png: mean detection and separation summary.",
        "- comparison_healthy_neighbor_clearance.png: healthy-neighbor clearance summary.",
        "- comparison_load_speed_envelope.png: batch peak-speed summary.",
        "- comparison_load_error_overlay.png: load-tracking overlay across scenarios.",
        "- comparison_fault_event_separation.png: healthy-neighbor clearance across all faults.",
        "",
        "## Scenarios",
    ]
    for summary in summaries:
        artifacts = summary.get("artifacts", {})
        figure_paths = artifacts.get("figure_paths", {}) if isinstance(artifacts, dict) else {}
        lines.extend(
            [
                f"- {summary['scenario']}: {summary['num_agents']} drones, faults at {summary['fault_times']}, cables {summary['fault_cables']}",
                f"  - Samples: {summary['sample_count']}",
                f"  - Log directory: {summary['log_dir']}",
                f"  - Meshcat replay: {artifacts.get('meshcat_replay_html', summary['meshcat_html'])}",
                f"  - Timeseries archive: {artifacts.get('full_drake_recording_npz', '')}",
                f"  - Scenario figures: {', '.join(sorted(Path(path).name for path in figure_paths.values()))}",
            ]
        )
    (OUTPUT_ROOT / "README.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--clean", action="store_true", help="Remove previous batch outputs first")
    parser.add_argument("--render-only", action="store_true", help="Regenerate artifacts from existing logs without rerunning simulations")
    parser.add_argument(
        "--manifest",
        type=Path,
        default=DEFAULT_FULL_DRAKE_BATCH_MANIFEST,
        help="Shared scenario manifest to drive the full-Drake batch",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.clean and args.render_only:
        raise ValueError("--clean and --render-only cannot be used together")
    ensure_output_root(clean=args.clean)
    summaries = []
    for scenario in load_scenarios(args.manifest):
        summaries.append(run_scenario(scenario, rerun_simulation=not args.render_only))
    write_batch_manifest(summaries, args.manifest)
    write_readme(summaries)


if __name__ == "__main__":
    main()