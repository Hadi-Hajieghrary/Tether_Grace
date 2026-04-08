#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from fault_metrics_root import FaultMetrics, compute_fault_metrics


np.set_printoptions(precision=3, suppress=True)


@dataclass
class Params:
    num_agents: int = 3
    quad_mass: float = 1.5
    load_mass: float = 3.0
    gravity: float = 9.81
    trajectory_amplitude: float = 1.5
    trajectory_period: float = 20.0
    trajectory_height: float = 1.5
    cable_lengths: np.ndarray = field(
        default_factory=lambda: np.array([0.914, 1.105, 0.995], dtype=float)
    )
    cable_stiffness: float = 400.0
    cable_damping: float = 15.0
    formation_radius: float = 0.6
    position_gain: np.ndarray = field(
        default_factory=lambda: np.array([6.0, 6.0, 8.0], dtype=float)
    )
    velocity_gain: np.ndarray = field(
        default_factory=lambda: np.array([8.0, 8.0, 10.0], dtype=float)
    )
    cable_gain: float = 4.0
    dt_sim: float = 0.0005
    dt_ctrl: float = 0.005

    def __post_init__(self) -> None:
        cable_lengths = np.asarray(self.cable_lengths, dtype=float).reshape(-1)
        if cable_lengths.size == 0:
            cable_lengths = np.array([1.0], dtype=float)
        if cable_lengths.size != self.num_agents:
            repeats = int(np.ceil(self.num_agents / cable_lengths.size))
            cable_lengths = np.tile(cable_lengths, repeats)[: self.num_agents]
        self.cable_lengths = cable_lengths.copy()

    def offsets(self) -> np.ndarray:
        angles = np.linspace(0.0, 2.0 * np.pi, self.num_agents, endpoint=False)
        offsets = np.zeros((self.num_agents, 3), dtype=float)
        offsets[:, 0] = self.formation_radius * np.cos(angles)
        offsets[:, 1] = self.formation_radius * np.sin(angles)
        return offsets


def trajectory(time_s: float, amplitude: float = 1.5, period: float = 20.0, z: float = 1.5) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    omega = 2.0 * np.pi / period
    ramp = np.clip(time_s / 5.0, 0.0, 1.0)
    position = np.array(
        [amplitude * np.sin(omega * time_s) * ramp, amplitude * np.sin(2.0 * omega * time_s) * 0.5 * ramp, z],
        dtype=float,
    )
    velocity = np.array(
        [amplitude * omega * np.cos(omega * time_s) * ramp, amplitude * omega * np.cos(2.0 * omega * time_s) * ramp, 0.0],
        dtype=float,
    )
    acceleration = np.array(
        [
            -amplitude * omega**2 * np.sin(omega * time_s) * ramp,
            -2.0 * amplitude * omega**2 * np.sin(2.0 * omega * time_s) * ramp,
            0.0,
        ],
        dtype=float,
    )
    return position, velocity, acceleration


def nominal_cable_distance(horizontal_radius: float, rest_length: float, vertical_load_share: float, stiffness: float) -> float:
    minimum_distance = max(rest_length, horizontal_radius + 1e-6)
    if vertical_load_share <= 0.0 or stiffness <= 0.0:
        return minimum_distance

    def supported_vertical_force(distance: float) -> float:
        vertical = np.sqrt(max(distance**2 - horizontal_radius**2, 0.0))
        if distance <= 1e-9:
            return 0.0
        tension = max(stiffness * (distance - rest_length), 0.0)
        return tension * vertical / distance

    upper_distance = minimum_distance
    while supported_vertical_force(upper_distance) < vertical_load_share:
        upper_distance += 0.05
        if upper_distance > rest_length + 5.0:
            break

    lower_distance = minimum_distance
    for _ in range(60):
        midpoint = 0.5 * (lower_distance + upper_distance)
        if supported_vertical_force(midpoint) < vertical_load_share:
            lower_distance = midpoint
        else:
            upper_distance = midpoint
    return upper_distance


def nominal_quad_relative_position(
    horizontal_offset: np.ndarray,
    rest_length: float,
    vertical_load_share: float,
    stiffness: float,
) -> np.ndarray:
    relative = horizontal_offset.copy()
    horizontal_radius = float(np.linalg.norm(relative[:2]))
    distance = nominal_cable_distance(horizontal_radius, rest_length, vertical_load_share, stiffness)
    vertical = np.sqrt(max(distance**2 - horizontal_radius**2, 0.0))
    relative[2] = vertical
    return relative


def cable_force(
    quad_position: np.ndarray,
    load_position: np.ndarray,
    quad_velocity: np.ndarray,
    load_velocity: np.ndarray,
    rest_length: float,
    stiffness: float,
    damping: float,
    active: bool = True,
    health_scale: float = 1.0,
) -> tuple[np.ndarray, np.ndarray, float, np.ndarray]:
    if not active or health_scale <= 1e-9:
        return np.zeros(3), np.zeros(3), 0.0, np.array([0.0, 0.0, -1.0], dtype=float)

    delta = quad_position - load_position
    distance = np.linalg.norm(delta)
    if distance < 1e-8:
        return np.zeros(3), np.zeros(3), 0.0, np.array([0.0, 0.0, -1.0], dtype=float)

    cable_direction = delta / distance
    stretch = distance - rest_length
    if stretch <= 0.0:
        return np.zeros(3), np.zeros(3), 0.0, cable_direction

    relative_velocity = quad_velocity - load_velocity
    rate = float(np.dot(relative_velocity, cable_direction))
    tension = health_scale * max(0.0, stiffness * stretch + damping * max(0.0, rate))
    quad_force = -tension * cable_direction
    load_force = tension * cable_direction
    return quad_force, load_force, tension, cable_direction


class Wind:
    def __init__(self, sigma: float = 0.5, tau: float = 0.5, seed: int = 42) -> None:
        self.sigma = sigma
        self.tau = tau
        self.rng = np.random.RandomState(seed)
        self.velocity = np.zeros(3, dtype=float)

    def step(self, dt: float) -> np.ndarray:
        decay = np.exp(-dt / self.tau)
        self.velocity = decay * self.velocity + self.sigma * np.sqrt(1.0 - decay**2) * self.rng.randn(3)
        return self.velocity.copy()

    def force(self, dt: float, mass: float = 1.5, drag_coeff: float = 0.1) -> np.ndarray:
        return drag_coeff * mass * self.step(dt)


class ConcurrentLearning:
    def __init__(
        self,
        gamma: float = 0.5,
        rho: float = 0.5,
        max_history_size: int = 50,
        data_spacing_time: float = 0.1,
        forgetting_factor: float = 1.0,
    ) -> None:
        self.theta = 0.5
        self.gamma = gamma
        self.rho = rho
        self.max_history_size = max_history_size
        self.data_spacing_time = data_spacing_time
        self.forgetting_factor = forgetting_factor
        self.history_y: list[float] = []
        self.history_phi: list[float] = []
        self.history_time: list[float] = []
        self.last_store_time = -1.0
        self.last_replay_term = 0.0

    def flush_history(self) -> None:
        self.history_y.clear()
        self.history_phi.clear()
        self.history_time.clear()

    @property
    def history_size(self) -> int:
        return len(self.history_y)

    def update(self, tension: float, cable_direction: np.ndarray, position_error: np.ndarray, dt: float, time_s: float) -> float:
        gravity = 9.81
        regressor = gravity
        cos_angle = max(0.01, float(cable_direction[2]))
        phi = tension * cos_angle

        prediction_error = regressor * self.theta - phi
        surface = float(np.dot(position_error, cable_direction))
        theta_dot = -self.gamma * regressor * surface

        if self.history_y:
            replay_terms = np.array([y_i * (y_i * self.theta - phi_i) for y_i, phi_i in zip(self.history_y, self.history_phi)], dtype=float)
            if self.forgetting_factor < 1.0:
                ages = np.arange(len(replay_terms) - 1, -1, -1, dtype=float)
                weights = np.power(self.forgetting_factor, ages)
                replay = float(np.sum(weights * replay_terms) / max(np.sum(weights), 1e-9))
            else:
                replay = float(np.mean(replay_terms))
            self.last_replay_term = replay
            theta_dot -= self.gamma * self.rho * replay
        else:
            self.last_replay_term = 0.0

        self.theta = float(np.clip(self.theta + theta_dot * dt, 0.1, 50.0))

        if time_s - self.last_store_time > self.data_spacing_time and regressor > 0.5:
            self.history_y.append(regressor)
            self.history_phi.append(phi)
            self.history_time.append(time_s)
            self.last_store_time = time_s
            if len(self.history_y) > self.max_history_size:
                self.history_y.pop(0)
                self.history_phi.pop(0)
                self.history_time.pop(0)

        return self.theta


class ExtendedStateObserver:
    def __init__(self, omega: float = 50.0, input_gain: float = 1.0 / 1.5) -> None:
        self.omega = omega
        self.input_gain = input_gain
        self.state = np.zeros((3, 3), dtype=float)

    def reset(self, position: np.ndarray, velocity: np.ndarray | None = None) -> None:
        velocity = np.zeros(3, dtype=float) if velocity is None else velocity
        for axis in range(3):
            self.state[axis] = [position[axis], velocity[axis], 0.0]

    def update(self, position: np.ndarray, control_command: np.ndarray, dt: float) -> np.ndarray:
        omega = self.omega
        for axis in range(3):
            error = position[axis] - self.state[axis, 0]
            self.state[axis, 0] += (self.state[axis, 1] + 3.0 * omega * error) * dt
            self.state[axis, 1] += (self.state[axis, 2] + 3.0 * omega**2 * error + self.input_gain * control_command[axis]) * dt
            self.state[axis, 2] += omega**3 * error * dt
            self.state[axis, 2] = np.clip(self.state[axis, 2], -20.0, 20.0)
        return self.state[:, 2].copy()


class L1AdaptiveController:
    def __init__(self, omega: float = 20.0, predictor_pole: float = 50.0, input_gain: float = 1.0 / 1.5, sigma_max: float = 20.0) -> None:
        self.omega = omega
        self.predictor_pole = predictor_pole
        self.input_gain = input_gain
        self.sigma_max = sigma_max
        self.predicted_position = np.zeros(3, dtype=float)
        self.predicted_velocity = np.zeros(3, dtype=float)
        self.sigma = np.zeros(3, dtype=float)
        self.compensation = np.zeros(3, dtype=float)
        self.prediction_error = np.zeros(3, dtype=float)

    def reset(self, position: np.ndarray, velocity: np.ndarray | None = None) -> None:
        self.predicted_position = position.copy()
        self.predicted_velocity = np.zeros(3, dtype=float) if velocity is None else velocity.copy()
        self.sigma = np.zeros(3, dtype=float)
        self.compensation = np.zeros(3, dtype=float)
        self.prediction_error = np.zeros(3, dtype=float)

    def update(self, position: np.ndarray, velocity: np.ndarray, control_command: np.ndarray, dt: float) -> np.ndarray:
        alpha = np.exp(-self.omega * dt)
        pole = self.predictor_pole

        for axis in range(3):
            position_error = position[axis] - self.predicted_position[axis]
            self.predicted_position[axis] += (self.predicted_velocity[axis] + 2.0 * pole * position_error) * dt
            self.predicted_velocity[axis] += (self.sigma[axis] + self.input_gain * control_command[axis] + pole**2 * position_error) * dt

            self.prediction_error[axis] = self.predicted_position[axis] - position[axis]
            velocity_error = self.predicted_velocity[axis] - velocity[axis]

            sigma_raw = -pole * velocity_error - pole**2 * self.prediction_error[axis]
            self.sigma[axis] = np.clip(sigma_raw, -self.sigma_max, self.sigma_max)
            self.compensation[axis] = alpha * self.compensation[axis] + (1.0 - alpha) * (-self.sigma[axis])

        return self.compensation.copy()


def allocate_shared_adaptive_force(
    shared_force: np.ndarray,
    cable_directions: list[np.ndarray],
    cable_ok: list[bool],
    allocator: str,
) -> list[np.ndarray]:
    allocations = [np.zeros(3, dtype=float) for _ in cable_directions]
    healthy_indices = [index for index, healthy in enumerate(cable_ok) if healthy]
    if not healthy_indices:
        return allocations

    if allocator == "z-weighted":
        weights = np.array(
            [max(float(cable_directions[index][2]), 0.05) for index in healthy_indices],
            dtype=float,
        )
    else:
        weights = np.ones(len(healthy_indices), dtype=float)
    weights /= np.sum(weights)

    for idx, weight in zip(healthy_indices, weights):
        allocations[idx] = weight * shared_force
    return allocations


def throttle_local_adaptive_force(
    local_force: np.ndarray,
    lateral_weight: float,
) -> np.ndarray:
    throttled = local_force.copy()
    throttled[0] *= lateral_weight
    throttled[1] *= lateral_weight
    return throttled

def project_local_adaptive_force(
    local_force: np.ndarray,
    cable_direction: np.ndarray,
    nominal_offset: np.ndarray,
    lateral_weight: float,
    projection_mode: str,
) -> np.ndarray:
    if projection_mode == "uniform_xy":
        return throttle_local_adaptive_force(local_force, lateral_weight)

    if projection_mode != "cable_radial":
        return local_force

    vertical_axis = np.array([0.0, 0.0, 1.0], dtype=float)
    horizontal_direction = cable_direction.copy()
    horizontal_direction[2] = 0.0
    horizontal_norm = np.linalg.norm(horizontal_direction)

    if horizontal_norm < 1e-6:
        horizontal_direction = nominal_offset.copy()
        horizontal_direction[2] = 0.0
        horizontal_norm = np.linalg.norm(horizontal_direction)

    if horizontal_norm < 1e-6:
        return throttle_local_adaptive_force(local_force, lateral_weight)

    radial_axis = horizontal_direction / horizontal_norm
    tangential_axis = np.cross(vertical_axis, radial_axis)

    radial_component = np.dot(local_force, radial_axis) * radial_axis
    tangential_component = np.dot(local_force, tangential_axis) * tangential_axis
    vertical_component = np.dot(local_force, vertical_axis) * vertical_axis
    return lateral_weight * radial_component + tangential_component + vertical_component


def desired_geometry_offset(
    nominal_offsets: np.ndarray,
    cable_ok: list[bool],
    index: int,
    time_s: float,
    fault_time: float,
    ramp_time: float,
    mode: str,
) -> tuple[np.ndarray, np.ndarray]:
    nominal_offset = nominal_offsets[index].copy()
    if mode != "healthy_centered_renormalized" or time_s < fault_time:
        return nominal_offset, np.zeros(3, dtype=float)

    healthy_indices = [healthy_index for healthy_index, healthy in enumerate(cable_ok) if healthy]
    if len(healthy_indices) < 2 or index not in healthy_indices:
        return nominal_offset, np.zeros(3, dtype=float)

    ramp_time = max(ramp_time, 1e-6)
    blend = min(max((time_s - fault_time) / ramp_time, 0.0), 1.0)
    blend_rate = 1.0 / ramp_time if 0.0 < blend < 1.0 else 0.0

    healthy_offsets = nominal_offsets[healthy_indices].copy()
    healthy_offsets[:, 2] = 0.0
    centroid = np.mean(healthy_offsets, axis=0)
    centered_offsets = healthy_offsets - centroid
    numerator = float(np.sum(np.linalg.norm(healthy_offsets[:, :2], axis=1) ** 2))
    denominator = float(np.sum(np.linalg.norm(centered_offsets[:, :2], axis=1) ** 2)) + 1e-9
    scale = np.sqrt(numerator / denominator)

    target_offsets = nominal_offsets.copy()
    for local_position, healthy_index in enumerate(healthy_indices):
        target_offsets[healthy_index, :2] = scale * centered_offsets[local_position, :2]

    target_offset = target_offsets[index]
    offset = (1.0 - blend) * nominal_offset + blend * target_offset
    offset_rate = blend_rate * (target_offset - nominal_offset)
    return offset, offset_rate


def normalize_fault_schedule(
    fault_cable: int | list[int] | tuple[int, ...] | np.ndarray,
    fault_time: float | list[float] | tuple[float, ...] | np.ndarray,
) -> list[tuple[float, int]]:
    if isinstance(fault_cable, np.ndarray):
        cables = [int(value) for value in fault_cable.reshape(-1).tolist()]
    elif isinstance(fault_cable, (list, tuple)):
        cables = [int(value) for value in fault_cable]
    else:
        cable_value = int(fault_cable)
        cables = [] if cable_value < 0 else [cable_value]

    if not cables:
        return []

    if isinstance(fault_time, np.ndarray):
        times = [float(value) for value in fault_time.reshape(-1).tolist()]
    elif isinstance(fault_time, (list, tuple)):
        times = [float(value) for value in fault_time]
    else:
        times = [float(fault_time)]

    if len(times) == 1 and len(cables) > 1:
        times = times * len(cables)
    if len(times) != len(cables):
        raise ValueError("fault_time must be scalar or have the same length as fault_cable")

    schedule = sorted((time_value, cable_value) for time_value, cable_value in zip(times, cables) if cable_value >= 0)
    return schedule


def normalize_fault_events(
    fault_cable: int | list[int] | tuple[int, ...] | np.ndarray,
    fault_time: float | list[float] | tuple[float, ...] | np.ndarray,
    fault_events: list[dict[str, Any]] | None = None,
) -> list[dict[str, Any]]:
    if fault_events is not None:
        events = []
        for event in fault_events:
            events.append(
                {
                    "cable_index": int(event["cable_index"]),
                    "time_seconds": float(event["time_seconds"]),
                    "post_fault_scale": float(event.get("post_fault_scale", 0.0)),
                    "profile": str(event.get("profile", "snap")),
                    "ramp_duration_seconds": float(event.get("ramp_duration_seconds", 0.0)),
                }
            )
        return sorted(events, key=lambda item: (item["time_seconds"], item["cable_index"]))

    return [
        {
            "cable_index": cable_index,
            "time_seconds": time_seconds,
            "post_fault_scale": 0.0,
            "profile": "snap",
            "ramp_duration_seconds": 0.0,
        }
        for time_seconds, cable_index in normalize_fault_schedule(fault_cable, fault_time)
    ]


def fault_event_multiplier(event: dict[str, Any], time_s: float) -> float:
    event_time = float(event["time_seconds"])
    if time_s < event_time:
        return 1.0

    post_fault_scale = float(event.get("post_fault_scale", 0.0))
    profile = str(event.get("profile", "snap"))
    ramp_duration = max(float(event.get("ramp_duration_seconds", 0.0)), 1e-9)
    elapsed = max(time_s - event_time, 0.0)

    if profile in {"snap", "step"}:
        return post_fault_scale
    if profile in {"ramp", "linear", "fray"}:
        alpha = np.clip(elapsed / ramp_duration, 0.0, 1.0)
        return (1.0 - alpha) + alpha * post_fault_scale
    if profile == "exponential":
        return post_fault_scale + (1.0 - post_fault_scale) * np.exp(-elapsed / ramp_duration)
    return post_fault_scale


def evaluate_cable_health(num_agents: int, fault_events: list[dict[str, Any]], time_s: float) -> np.ndarray:
    health = np.ones(num_agents, dtype=float)
    for event in fault_events:
        cable_index = int(event["cable_index"])
        if 0 <= cable_index < num_agents:
            health[cable_index] = min(health[cable_index], fault_event_multiplier(event, time_s))
    return health


def simulate(
    mode: str,
    fault_cable: int | list[int] | tuple[int, ...] | np.ndarray = -1,
    fault_time: float | list[float] | tuple[float, ...] | np.ndarray = 1e10,
    duration: float = 30.0,
    seed: int = 42,
    wind_sigma: float = 0.5,
    l1_config: dict[str, float] | None = None,
    params: Params | None = None,
    fault_events: list[dict[str, Any]] | None = None,
    cl_config: dict[str, float | int | bool] | None = None,
) -> dict[str, Any]:
    params = Params() if params is None else params
    normalized_fault_events = normalize_fault_events(fault_cable, fault_time, fault_events=fault_events)
    first_fault_time = normalized_fault_events[0]["time_seconds"] if normalized_fault_events else 1e10
    primary_fault_cable = normalized_fault_events[0]["cable_index"] if len(normalized_fault_events) == 1 else -1
    offsets = params.offsets()
    winds = [Wind(wind_sigma, 0.5, seed + index) for index in range(params.num_agents + 1)]

    if mode == "cl":
        config = {
            "gamma": 0.5,
            "rho": 0.5,
            "max_history_size": 50,
            "data_spacing_time": 0.1,
            "forgetting_factor": 1.0,
            "flush_on_fault": False,
        }
        if cl_config is not None:
            config.update(cl_config)
        cls = [
            ConcurrentLearning(
                gamma=float(config["gamma"]),
                rho=float(config["rho"]),
                max_history_size=int(config["max_history_size"]),
                data_spacing_time=float(config["data_spacing_time"]),
                forgetting_factor=float(config["forgetting_factor"]),
            )
            for _ in range(params.num_agents)
        ]
        esos = [ExtendedStateObserver(50.0, 1.0 / params.quad_mass) for _ in range(params.num_agents)]
        flush_on_fault = bool(config.get("flush_on_fault", False))
    else:
        config = {
            "omega": 20.0,
            "predictor_pole": 50.0,
            "input_gain": 1.0 / params.quad_mass,
            "sigma_max": 20.0,
            "shared_blend": 0.0,
            "shared_omega": 10.0,
            "shared_predictor_pole": 20.0,
            "shared_sigma_max": 10.0,
            "shared_ramp_time": 0.2,
            "shared_allocator": "equal",
            "shared_load_kp_scale": 0.5,
            "shared_load_kv_scale": 0.5,
            "local_lateral_weight": 1.0,
            "local_projection": "uniform_xy",
            "desired_geometry_mode": "off",
            "desired_geometry_ramp_time": 0.5,
        }
        if l1_config is not None:
            config.update(l1_config)
        local_l1_config = {
            "omega": float(config["omega"]),
            "predictor_pole": float(config["predictor_pole"]),
            "input_gain": float(config["input_gain"]),
            "sigma_max": float(config["sigma_max"]),
        }
        l1s = [L1AdaptiveController(**local_l1_config) for _ in range(params.num_agents)]
        shared_blend = float(config.get("shared_blend", 0.0))
        shared_allocator = str(config.get("shared_allocator", "equal"))
        shared_ramp_time = max(float(config.get("shared_ramp_time", 0.2)), 1e-6)
        shared_load_kp_scale = float(config.get("shared_load_kp_scale", 0.5))
        shared_load_kv_scale = float(config.get("shared_load_kv_scale", 0.5))
        local_lateral_weight = float(config.get("local_lateral_weight", 1.0))
        local_projection = str(config.get("local_projection", "uniform_xy"))
        desired_geometry_mode = str(config.get("desired_geometry_mode", "off"))
        desired_geometry_ramp_time = float(config.get("desired_geometry_ramp_time", 0.5))
        load_l1 = None
        if shared_blend > 0.0:
            load_l1 = L1AdaptiveController(
                omega=float(config.get("shared_omega", config["omega"])),
                predictor_pole=float(config.get("shared_predictor_pole", config["predictor_pole"])),
                input_gain=1.0 / params.load_mass,
                sigma_max=float(config.get("shared_sigma_max", config["sigma_max"])),
            )

    state = np.zeros(6 + 6 * params.num_agents, dtype=float)

    initial_load_position, _, _ = trajectory(
        0.0,
        amplitude=params.trajectory_amplitude,
        period=params.trajectory_period,
        z=params.trajectory_height,
    )
    state[0:3] = initial_load_position
    initial_vertical_share = params.load_mass * params.gravity / max(params.num_agents, 1)
    for index in range(params.num_agents):
        relative_position = nominal_quad_relative_position(
            offsets[index],
            params.cable_lengths[index],
            initial_vertical_share,
            params.cable_stiffness,
        )
        state[6 + 6 * index : 6 + 6 * index + 3] = initial_load_position + relative_position

    for index in range(params.num_agents):
        quad_position = state[6 + 6 * index : 6 + 6 * index + 3]
        quad_velocity = state[6 + 6 * index + 3 : 6 + 6 * index + 6]
        if mode == "cl":
            esos[index].reset(quad_position, quad_velocity)
        else:
            l1s[index].reset(quad_position, quad_velocity)
    if mode != "cl" and load_l1 is not None:
        load_l1.reset(initial_load_position, np.zeros(3, dtype=float))

    log: dict[str, Any] = {
        "t": [],
        "err": [],
        "load_err_vec": [],
        "tensions": [],
        "cable_directions": [],
        "cable_health": [],
        "sigma": [],
        "local_sigma": [],
        "local_compensation": [],
        "shared_sigma": [],
        "shared_compensation": [],
        "shared_force": [],
        "shared_allocations": [],
        "theta": [],
        "theta_agents": [],
        "cl_history_sizes": [],
        "cl_replay_terms": [],
        "load_p": [],
        "load_v": [],
        "load_d": [],
        "load_vd": [],
        "load_a_cmd": [],
        "load_wind": [],
        "quad_p": [],
        "quad_v": [],
        "quad_d": [],
        "quad_vd": [],
        "quad_force": [],
        "quad_accel_cmd": [],
        "feedback_force": [],
        "adaptive_force": [],
        "cable_force_comp": [],
        "anti_swing_force": [],
        "quad_wind": [],
        "mode": mode,
        "fault_cable": primary_fault_cable,
        "fault_time": first_fault_time,
        "fault_cables": np.array([int(event["cable_index"]) for event in normalized_fault_events], dtype=int),
        "fault_times": np.array([float(event["time_seconds"]) for event in normalized_fault_events], dtype=float),
        "fault_event_profiles": np.array([str(event.get("profile", "snap")) for event in normalized_fault_events], dtype=object),
        "fault_event_scales": np.array([float(event.get("post_fault_scale", 0.0)) for event in normalized_fault_events], dtype=float),
        "num_agents": params.num_agents,
        "cable_lengths": params.cable_lengths.copy(),
        "dt_ctrl": params.dt_ctrl,
        "dt_sim": params.dt_sim,
    }

    num_steps = int(duration / params.dt_sim)
    control_interval = int(params.dt_ctrl / params.dt_sim)
    commanded_forces = [np.zeros(3, dtype=float) for _ in range(params.num_agents)]
    commanded_accel = [np.zeros(3, dtype=float) for _ in range(params.num_agents)]
    quad_desired_positions = np.zeros((params.num_agents, 3), dtype=float)
    quad_desired_velocities = np.zeros((params.num_agents, 3), dtype=float)
    feedback_forces = np.zeros((params.num_agents, 3), dtype=float)
    adaptive_forces = np.zeros((params.num_agents, 3), dtype=float)
    cable_force_compensations = np.zeros((params.num_agents, 3), dtype=float)
    anti_swing_forces = np.zeros((params.num_agents, 3), dtype=float)
    controller_cable_directions = np.zeros((params.num_agents, 3), dtype=float)
    local_sigmas = np.zeros((params.num_agents, 3), dtype=float)
    local_compensations = np.zeros((params.num_agents, 3), dtype=float)
    theta_agents = np.zeros(params.num_agents, dtype=float)
    shared_sigma = np.zeros(3, dtype=float)
    shared_compensation = np.zeros(3, dtype=float)
    shared_force = np.zeros(3, dtype=float)
    shared_allocations = np.zeros((params.num_agents, 3), dtype=float)
    quad_wind_forces = np.zeros((params.num_agents, 3), dtype=float)
    load_wind_force = np.zeros(3, dtype=float)
    cl_history_sizes = np.zeros(params.num_agents, dtype=float)
    cl_replay_terms = np.zeros(params.num_agents, dtype=float)
    cl_flush_done = False
    for step in range(num_steps):
        time_s = step * params.dt_sim
        cable_health = evaluate_cable_health(params.num_agents, normalized_fault_events, time_s)
        cable_ok = [float(value) > 1e-6 for value in cable_health]
        fault_active = bool(np.any(cable_health < 0.999999))
        if mode == "cl" and fault_active and flush_on_fault and not cl_flush_done:
            for estimator in cls:
                estimator.flush_history()
            cl_flush_done = True

        load_desired, load_velocity_desired, load_accel_desired = trajectory(
            time_s,
            amplitude=params.trajectory_amplitude,
            period=params.trajectory_period,
            z=params.trajectory_height,
        )

        if step % control_interval == 0:
            load_position = state[0:3]
            load_velocity = state[3:6]
            cable_directions: list[np.ndarray] = []
            tensions: list[float] = []

            for index in range(params.num_agents):
                state_index = 6 + 6 * index
                quad_position = state[state_index : state_index + 3]
                quad_velocity = state[state_index + 3 : state_index + 6]
                cable_length = params.cable_lengths[index]
                delta = quad_position - load_position
                distance = np.linalg.norm(delta)
                cable_direction = delta / max(distance, 1e-6)
                _, _, tension, _ = cable_force(
                    quad_position,
                    load_position,
                    quad_velocity,
                    load_velocity,
                    cable_length,
                    params.cable_stiffness,
                    params.cable_damping,
                    cable_ok[index],
                    health_scale=float(cable_health[index]),
                )
                cable_directions.append(cable_direction)
                tensions.append(tension)

            shared_force_allocations = [np.zeros(3, dtype=float) for _ in range(params.num_agents)]
            local_l1_scale = 1.0
            local_sigmas.fill(0.0)
            local_compensations.fill(0.0)
            theta_agents.fill(0.0)
            shared_sigma.fill(0.0)
            shared_compensation.fill(0.0)
            shared_force.fill(0.0)
            shared_allocations.fill(0.0)
            if mode != "cl" and load_l1 is not None and fault_active:
                ramp = min(max((time_s - first_fault_time) / shared_ramp_time, 0.0), 1.0)
                shared_scale = min(max(shared_blend * ramp, 0.0), 1.0)
                local_l1_scale = 1.0 - shared_scale
                load_position_error = load_position - load_desired
                load_velocity_error = load_velocity - load_velocity_desired
                load_nominal_accel = (
                    load_accel_desired
                    - shared_load_kp_scale * params.position_gain * load_position_error
                    - shared_load_kv_scale * params.velocity_gain * load_velocity_error
                )
                shared_compensation = load_l1.update(load_position, load_velocity, load_nominal_accel, params.dt_ctrl)
                shared_force = shared_scale * params.load_mass * shared_compensation
                shared_force_allocations = allocate_shared_adaptive_force(
                    shared_force,
                    cable_directions,
                    cable_ok,
                    shared_allocator,
                )
                shared_sigma = load_l1.sigma.copy()
                shared_allocations = np.array(shared_force_allocations, dtype=float)

            for index in range(params.num_agents):
                state_index = 6 + 6 * index
                quad_position = state[state_index : state_index + 3]
                quad_velocity = state[state_index + 3 : state_index + 6]
                cable_length = params.cable_lengths[index]
                healthy_count = max(sum(1 for healthy in cable_ok if healthy), 1)
                vertical_load_share = params.load_mass * params.gravity / healthy_count if cable_ok[index] else 0.0

                quad_offset, quad_offset_rate = desired_geometry_offset(
                    offsets,
                    cable_ok,
                    index,
                    time_s,
                    first_fault_time,
                    desired_geometry_ramp_time if mode != "cl" else 0.5,
                    desired_geometry_mode if mode != "cl" else "off",
                )

                quad_relative = nominal_quad_relative_position(
                    quad_offset,
                    cable_length,
                    vertical_load_share,
                    params.cable_stiffness,
                )
                quad_desired = load_desired + quad_relative
                quad_velocity_desired = load_velocity_desired + quad_offset_rate
                quad_desired_positions[index] = quad_desired
                quad_desired_velocities[index] = quad_velocity_desired

                position_error = quad_position - quad_desired
                velocity_error = quad_velocity - quad_velocity_desired

                feedback_force = -(params.position_gain * position_error + params.velocity_gain * velocity_error)
                gravity_force = params.quad_mass * params.gravity * np.array([0.0, 0.0, 1.0])

                cable_direction = cable_directions[index]
                tension = tensions[index]
                cable_force_comp = tension * cable_direction

                desired_direction = np.array([0.0, 0.0, 1.0])
                swing_error = desired_direction - np.dot(desired_direction, cable_direction) * cable_direction
                anti_swing_force = params.cable_gain * swing_error
                controller_cable_directions[index] = cable_direction
                feedback_forces[index] = feedback_force
                cable_force_compensations[index] = cable_force_comp
                anti_swing_forces[index] = anti_swing_force

                if mode == "cl":
                    disturbance_estimate = esos[index].update(quad_position, commanded_accel[index], params.dt_ctrl)
                    eso_force = params.quad_mass * disturbance_estimate
                    theta = cls[index].update(tension, cable_direction, position_error, params.dt_ctrl, time_s)
                    cl_force = theta * (params.gravity * np.array([0.0, 0.0, 1.0]) + load_accel_desired)
                    adaptive_forces[index] = eso_force + cl_force
                    theta_agents[index] = theta
                    cl_history_sizes[index] = float(cls[index].history_size)
                    cl_replay_terms[index] = float(cls[index].last_replay_term)
                    total_force = gravity_force + feedback_force + cable_force_comp + anti_swing_force + eso_force + cl_force
                else:
                    l1_compensation = l1s[index].update(quad_position, quad_velocity, commanded_accel[index], params.dt_ctrl)
                    local_l1_force = params.quad_mass * l1_compensation
                    if fault_active:
                        local_l1_force = project_local_adaptive_force(
                            local_l1_force,
                            cable_direction,
                            offsets[index],
                            local_lateral_weight,
                            local_projection,
                        )
                    l1_force = local_l1_scale * local_l1_force + shared_force_allocations[index]
                    adaptive_forces[index] = l1_force
                    local_sigmas[index] = l1s[index].sigma.copy()
                    local_compensations[index] = l1_compensation.copy()
                    total_force = gravity_force + feedback_force + cable_force_comp + anti_swing_force + l1_force

                force_norm = np.linalg.norm(total_force)
                max_force = 3.5 * params.quad_mass * params.gravity
                if force_norm > max_force:
                    total_force *= max_force / force_norm

                commanded_forces[index] = total_force
                commanded_accel[index] = total_force / params.quad_mass - params.gravity * np.array([0.0, 0.0, 1.0])

        load_position = state[0:3]
        load_velocity = state[3:6]
        load_force = -params.load_mass * params.gravity * np.array([0.0, 0.0, 1.0])
        load_wind_force = winds[params.num_agents].force(params.dt_sim, params.load_mass, 0.03)
        load_force += load_wind_force

        for index in range(params.num_agents):
            state_index = 6 + 6 * index
            quad_position = state[state_index : state_index + 3]
            quad_velocity = state[state_index + 3 : state_index + 6]
            quad_cable_force, load_cable_force, _, _ = cable_force(
                quad_position,
                load_position,
                quad_velocity,
                load_velocity,
                params.cable_lengths[index],
                params.cable_stiffness,
                params.cable_damping,
                cable_ok[index],
                health_scale=float(cable_health[index]),
            )

            quad_wind_forces[index] = winds[index].force(params.dt_sim, params.quad_mass, 0.08)
            quad_force = (
                commanded_forces[index]
                + quad_cable_force
                - params.quad_mass * params.gravity * np.array([0.0, 0.0, 1.0])
                + quad_wind_forces[index]
            )

            quad_accel = quad_force / params.quad_mass
            state[state_index + 3 : state_index + 6] += quad_accel * params.dt_sim
            state[state_index : state_index + 3] += state[state_index + 3 : state_index + 6] * params.dt_sim
            load_force += load_cable_force

        load_accel = load_force / params.load_mass
        state[3:6] += load_accel * params.dt_sim
        state[0:3] += state[3:6] * params.dt_sim

        if step % control_interval == 0:
            quad_positions = np.zeros((params.num_agents, 3), dtype=float)
            quad_velocities = np.zeros((params.num_agents, 3), dtype=float)
            cable_directions = np.zeros((params.num_agents, 3), dtype=float)
            log["t"].append(time_s)
            log["load_p"].append(state[0:3].copy())
            log["load_v"].append(state[3:6].copy())
            log["load_d"].append(load_desired.copy())
            log["load_vd"].append(load_velocity_desired.copy())
            log["load_a_cmd"].append(load_accel_desired.copy())
            load_error = state[0:3] - load_desired
            log["load_err_vec"].append(load_error)
            log["err"].append(float(np.linalg.norm(load_error)))
            log["load_wind"].append(load_wind_force.copy())

            tensions = []
            for index in range(params.num_agents):
                state_index = 6 + 6 * index
                quad_position = state[state_index : state_index + 3]
                quad_velocity = state[state_index + 3 : state_index + 6]
                quad_positions[index] = quad_position
                quad_velocities[index] = quad_velocity
                _, _, tension, cable_direction = cable_force(
                    quad_position,
                    state[0:3],
                    quad_velocity,
                    state[3:6],
                    params.cable_lengths[index],
                    params.cable_stiffness,
                    params.cable_damping,
                    cable_ok[index],
                    health_scale=float(cable_health[index]),
                )
                tensions.append(tension)
                cable_directions[index] = cable_direction
            log["tensions"].append(tensions)
            log["cable_directions"].append(cable_directions)
            log["cable_health"].append(cable_health.copy())
            log["quad_p"].append(quad_positions)
            log["quad_v"].append(quad_velocities)
            log["quad_d"].append(quad_desired_positions.copy())
            log["quad_vd"].append(quad_desired_velocities.copy())
            log["quad_force"].append(np.array(commanded_forces, dtype=float))
            log["quad_accel_cmd"].append(np.array(commanded_accel, dtype=float))
            log["feedback_force"].append(feedback_forces.copy())
            log["adaptive_force"].append(adaptive_forces.copy())
            log["cable_force_comp"].append(cable_force_compensations.copy())
            log["anti_swing_force"].append(anti_swing_forces.copy())
            log["quad_wind"].append(quad_wind_forces.copy())

            if mode == "l1":
                log["sigma"].append(local_sigmas[0].copy())
                log["local_sigma"].append(local_sigmas.copy())
                log["local_compensation"].append(local_compensations.copy())
                log["shared_sigma"].append(shared_sigma.copy())
                log["shared_compensation"].append(shared_compensation.copy())
                log["shared_force"].append(shared_force.copy())
                log["shared_allocations"].append(shared_allocations.copy())
            else:
                log["theta"].append(float(theta_agents[0]))
                log["theta_agents"].append(theta_agents.copy())
                log["cl_history_sizes"].append(cl_history_sizes.copy())
                log["cl_replay_terms"].append(cl_replay_terms.copy())

    for key, value in list(log.items()):
        if isinstance(value, list) and value:
            log[key] = np.array(value)

    return log


def build_run_summary(results: dict[str, dict[str, Any]], metrics: dict[str, FaultMetrics]) -> dict[str, Any]:
    return {
        "runs": {
            key: {
                "mode": value["mode"],
                "fault_cable": int(value["fault_cable"]),
                "fault_time": float(value["fault_time"]),
                "fault_cables": value.get("fault_cables", np.array([], dtype=int)).astype(int).tolist(),
                "fault_times": value.get("fault_times", np.array([], dtype=float)).astype(float).tolist(),
                "fault_event_profiles": value.get("fault_event_profiles", np.array([], dtype=object)).astype(str).tolist(),
                "fault_event_scales": value.get("fault_event_scales", np.array([], dtype=float)).astype(float).tolist(),
                "samples": int(len(value["t"])),
            }
            for key, value in results.items()
        },
        "metrics": {key: metric.to_dict() for key, metric in metrics.items()},
    }


def save_figures(output_dir: Path, results: dict[str, dict[str, Any]], metrics: dict[str, FaultMetrics], fault_time: float, fault_cable: int) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(2, 2, figsize=(14, 9))
    figure.suptitle(
        f"Cable Snap at t = {fault_time:.0f}s | Cable {fault_cable}",
        fontsize=13,
        fontweight="bold",
    )

    axes[0, 0].plot(results["cl_snap"]["t"], results["cl_snap"]["err"] * 100.0, "b-", lw=0.6, alpha=0.8)
    axes[0, 0].axvline(fault_time, color="r", ls="--", lw=1.5, label="Cable snap")
    axes[0, 0].set_title(f"GPAC-CL | RMSE ratio = {metrics['cl_snap'].rmse_ratio:.3f}", fontsize=11)
    axes[0, 0].set_ylabel("Tracking Error [cm]")
    axes[0, 0].legend(fontsize=8)
    axes[0, 0].grid(True, alpha=0.3)

    axes[0, 1].plot(results["l1_snap"]["t"], results["l1_snap"]["err"] * 100.0, "g-", lw=0.6, alpha=0.8)
    axes[0, 1].axvline(fault_time, color="r", ls="--", lw=1.5, label="Cable snap")
    axes[0, 1].set_title(f"GPAC-L1 | RMSE ratio = {metrics['l1_snap'].rmse_ratio:.3f}", fontsize=11)
    axes[0, 1].set_ylabel("Tracking Error [cm]")
    axes[0, 1].legend(fontsize=8)
    axes[0, 1].grid(True, alpha=0.3)

    y_max = max(axes[0, 0].get_ylim()[1], axes[0, 1].get_ylim()[1])
    axes[0, 0].set_ylim([0.0, y_max])
    axes[0, 1].set_ylim([0.0, y_max])

    sigma = results["l1_snap"].get("sigma")
    if sigma is not None and len(sigma) > 0:
        sigma_time = results["l1_snap"]["t"][: len(sigma)]
        for axis, label in enumerate(["x", "y", "z"]):
            axes[1, 0].plot(sigma_time, sigma[:, axis], lw=0.8, label=f"sigma_{label}")
    axes[1, 0].axvline(fault_time, color="r", ls="--", lw=1.5)
    axes[1, 0].set_xlabel("Time [s]")
    axes[1, 0].set_ylabel("sigma_hat [m/s^2]")
    axes[1, 0].set_title("L1 Uncertainty Estimate", fontsize=11)
    axes[1, 0].legend(fontsize=8)
    axes[1, 0].grid(True, alpha=0.3)

    tensions = results["l1_snap"]["tensions"]
    for axis in range(tensions.shape[1]):
        label = f"Cable {axis}"
        if axis == fault_cable:
            label += " (snapped)"
        axes[1, 1].plot(results["l1_snap"]["t"], tensions[:, axis], lw=0.8, label=label)
    axes[1, 1].axvline(fault_time, color="r", ls="--", lw=1.5)
    axes[1, 1].set_xlabel("Time [s]")
    axes[1, 1].set_ylabel("Tension [N]")
    axes[1, 1].set_title("Cable Tensions (L1 mode)", fontsize=11)
    axes[1, 1].legend(fontsize=8)
    axes[1, 1].grid(True, alpha=0.3)

    figure.tight_layout()
    figure.savefig(output_dir / "topology_invariance_root.png", dpi=150, bbox_inches="tight")
    plt.close(figure)

    figure, axis = plt.subplots(figsize=(8, 4.5))
    labels = ["CL\nnominal", "L1\nnominal", "CL\ncable snap", "L1\ncable snap"]
    values = [
        metrics["cl_nom"].nominal_rmse * 100.0,
        metrics["l1_nom"].nominal_rmse * 100.0,
        metrics["cl_snap"].post_fault_rmse * 100.0,
        metrics["l1_snap"].post_fault_rmse * 100.0,
    ]
    colors = ["#4a90d9", "#5cb85c", "#d9534f", "#27ae60"]
    bars = axis.bar(labels, values, color=colors, edgecolor="black", lw=0.5)
    for bar, value in zip(bars, values):
        axis.text(bar.get_x() + bar.get_width() / 2.0, bar.get_height() + 1.0, f"{value:.1f}", ha="center", fontsize=11, fontweight="bold")
    axis.set_ylabel("Payload RMSE [cm]", fontsize=11)
    axis.set_title("Fault Tolerance: CL vs L1", fontsize=13, fontweight="bold")
    axis.grid(True, axis="y", alpha=0.3)
    figure.tight_layout()
    figure.savefig(output_dir / "rmse_comparison_root.png", dpi=150, bbox_inches="tight")
    plt.close(figure)


def main() -> tuple[dict[str, dict[str, Any]], dict[str, FaultMetrics]]:
    parser = argparse.ArgumentParser(description="Root-owned topology-invariance CL-vs-L1 experiment")
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--fault-time", type=float, default=15.0)
    parser.add_argument("--fault-cable", type=int, default=1)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--wind-sigma", type=float, default=0.5)
    parser.add_argument("--l1-omega", type=float, default=20.0)
    parser.add_argument("--l1-predictor-pole", type=float, default=50.0)
    parser.add_argument("--l1-sigma-max", type=float, default=20.0)
    parser.add_argument("--l1-shared-blend", type=float, default=0.0)
    parser.add_argument("--l1-shared-omega", type=float, default=10.0)
    parser.add_argument("--l1-shared-predictor-pole", type=float, default=20.0)
    parser.add_argument("--l1-shared-sigma-max", type=float, default=10.0)
    parser.add_argument("--l1-shared-ramp-time", type=float, default=0.2)
    parser.add_argument("--l1-shared-allocator", type=str, default="equal")
    parser.add_argument("--l1-shared-load-kp-scale", type=float, default=0.5)
    parser.add_argument("--l1-shared-load-kv-scale", type=float, default=0.5)
    parser.add_argument("--l1-local-lateral-weight", type=float, default=1.0)
    parser.add_argument("--l1-local-projection", type=str, default="uniform_xy")
    parser.add_argument("--l1-desired-geometry-mode", type=str, default="off")
    parser.add_argument("--l1-desired-geometry-ramp-time", type=float, default=0.5)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("/workspaces/Tether_Grace/outputs/gpac_fault_tolerance/phase1_python"),
    )
    args = parser.parse_args()

    output_dir = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)

    l1_config = {
        "omega": args.l1_omega,
        "predictor_pole": args.l1_predictor_pole,
        "sigma_max": args.l1_sigma_max,
        "shared_blend": args.l1_shared_blend,
        "shared_omega": args.l1_shared_omega,
        "shared_predictor_pole": args.l1_shared_predictor_pole,
        "shared_sigma_max": args.l1_shared_sigma_max,
        "shared_ramp_time": args.l1_shared_ramp_time,
        "shared_allocator": args.l1_shared_allocator,
        "shared_load_kp_scale": args.l1_shared_load_kp_scale,
        "shared_load_kv_scale": args.l1_shared_load_kv_scale,
        "local_lateral_weight": args.l1_local_lateral_weight,
        "local_projection": args.l1_local_projection,
        "desired_geometry_mode": args.l1_desired_geometry_mode,
        "desired_geometry_ramp_time": args.l1_desired_geometry_ramp_time,
    }

    results = {
        "cl_nom": simulate("cl", -1, 1e10, args.duration, args.seed, args.wind_sigma),
        "l1_nom": simulate("l1", -1, 1e10, args.duration, args.seed, args.wind_sigma, l1_config=l1_config),
        "cl_snap": simulate("cl", args.fault_cable, args.fault_time, args.duration, args.seed, args.wind_sigma),
        "l1_snap": simulate("l1", args.fault_cable, args.fault_time, args.duration, args.seed, args.wind_sigma, l1_config=l1_config),
    }

    metrics = {
        name: compute_fault_metrics(run["t"], run["err"], run["fault_time"])
        for name, run in results.items()
    }

    save_figures(output_dir, results, metrics, args.fault_time, args.fault_cable)

    summary = build_run_summary(results, metrics)
    summary["l1_config"] = l1_config
    (output_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")

    print("=" * 65)
    print("ROOT TOPOLOGY-INVARIANCE EXPERIMENT")
    print("=" * 65)
    print(f"{'Config':<20} {'Nominal':>10} {'Pre':>10} {'Post':>10} {'Ratio':>10} {'Peak':>10}")
    print("-" * 75)
    for name in ["cl_nom", "l1_nom", "cl_snap", "l1_snap"]:
        metric = metrics[name]
        if "nom" in name:
            print(f"{name:<20} {metric.nominal_rmse*100:>9.1f}cm {'---':>10} {'---':>10} {'---':>10} {metric.peak_deviation*100:>9.1f}cm")
        else:
            print(
                f"{name:<20} {'---':>10} {metric.pre_fault_rmse*100:>9.1f}cm {metric.post_fault_rmse*100:>9.1f}cm {metric.rmse_ratio:>10.3f} {metric.peak_deviation*100:>9.1f}cm"
            )

    return results, metrics


if __name__ == "__main__":
    main()