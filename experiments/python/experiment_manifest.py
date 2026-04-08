#!/usr/bin/env python3
from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


ROOT = Path("/workspaces/Tether_Grace")
DEFAULT_FULL_DRAKE_BATCH_MANIFEST = ROOT / "experiments" / "manifests" / "full_drake_reference_batch.json"


@dataclass(frozen=True)
class FaultEvent:
    cable_index: int
    time_seconds: float
    post_fault_scale: float = 0.0
    profile: str = "snap"
    ramp_duration_seconds: float = 0.0

    @classmethod
    def from_dict(cls, payload: dict[str, Any]) -> "FaultEvent":
        return cls(
            cable_index=int(payload["cable_index"]),
            time_seconds=float(payload["time_seconds"]),
            post_fault_scale=float(payload.get("post_fault_scale", 0.0)),
            profile=str(payload.get("profile", "snap")),
            ramp_duration_seconds=float(payload.get("ramp_duration_seconds", 0.0)),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "cable_index": self.cable_index,
            "time_seconds": self.time_seconds,
            "post_fault_scale": self.post_fault_scale,
            "profile": self.profile,
            "ramp_duration_seconds": self.ramp_duration_seconds,
        }


@dataclass(frozen=True)
class ExperimentScenario:
    name: str
    num_agents: int
    duration: float
    cable_lengths: list[float]
    fault_events: list[FaultEvent] = field(default_factory=list)
    description: str = ""
    tags: list[str] = field(default_factory=list)

    # Optional physics / controller overrides (None → use runner defaults)
    max_thrust: float | None = None
    payload_mass: float | None = None
    tension_kp: float | None = None
    disable_tension_ff: bool | None = None
    position_kp: float | None = None
    position_kd: float | None = None
    position_ki: float | None = None
    enable_eso: bool | None = None
    seed: int | None = None

    # Wind disturbance overrides
    enable_wind: bool | None = None
    wind_mean: list[float] | None = None  # [x, y, z] m/s
    wind_sigma: float | None = None       # turbulence intensity scale

    # ESKF + sensor noise overrides
    enable_eskf: bool | None = None
    imu_rate: float | None = None      # Hz
    gps_noise: float | None = None     # position noise stddev [m]
    baro_noise: float | None = None    # altitude noise stddev [m]
    gps_rate: float | None = None      # Hz
    baro_rate: float | None = None     # Hz

    # Trajectory mode
    trajectory_mode: str | None = None  # "figure8", "hover", "point_to_point"

    # Reactive FTC baseline
    enable_reactive_ftc: bool | None = None
    reactive_boost: float | None = None  # boost fraction (e.g. 0.3 = +30%)

    # Oracle baseline
    oracle_load_share: float | None = None  # kg; centralized oracle feedforward

    @property
    def fault_cables(self) -> list[int]:
        return [event.cable_index for event in self.fault_events]

    @property
    def fault_times(self) -> list[float]:
        return [event.time_seconds for event in self.fault_events]

    @classmethod
    def from_dict(cls, payload: dict[str, Any]) -> "ExperimentScenario":
        return cls(
            name=str(payload["name"]),
            num_agents=int(payload["num_agents"]),
            duration=float(payload["duration"]),
            cable_lengths=[float(value) for value in payload["cable_lengths"]],
            fault_events=[FaultEvent.from_dict(event) for event in payload.get("fault_events", [])],
            description=str(payload.get("description", "")),
            tags=[str(value) for value in payload.get("tags", [])],
            max_thrust=payload.get("max_thrust"),
            payload_mass=payload.get("payload_mass"),
            tension_kp=payload.get("tension_kp"),
            disable_tension_ff=payload.get("disable_tension_ff"),
            position_kp=payload.get("position_kp"),
            position_kd=payload.get("position_kd"),
            position_ki=payload.get("position_ki"),
            enable_eso=payload.get("enable_eso"),
            seed=payload.get("seed"),
            enable_wind=payload.get("enable_wind"),
            wind_mean=payload.get("wind_mean"),
            wind_sigma=payload.get("wind_sigma"),
            enable_eskf=payload.get("enable_eskf"),
            imu_rate=payload.get("imu_rate"),
            gps_noise=payload.get("gps_noise"),
            baro_noise=payload.get("baro_noise"),
            gps_rate=payload.get("gps_rate"),
            baro_rate=payload.get("baro_rate"),
            trajectory_mode=payload.get("trajectory_mode"),
            enable_reactive_ftc=payload.get("enable_reactive_ftc"),
            reactive_boost=payload.get("reactive_boost"),
            oracle_load_share=payload.get("oracle_load_share"),
        )

    def to_dict(self) -> dict[str, Any]:
        result: dict[str, Any] = {
            "name": self.name,
            "num_agents": self.num_agents,
            "duration": self.duration,
            "cable_lengths": list(self.cable_lengths),
            "fault_events": [event.to_dict() for event in self.fault_events],
            "description": self.description,
            "tags": list(self.tags),
        }
        for key in ("max_thrust", "payload_mass", "tension_kp", "disable_tension_ff",
                     "position_kp", "position_kd", "position_ki", "enable_eso", "seed",
                     "enable_wind", "wind_mean", "wind_sigma",
                     "enable_eskf", "imu_rate", "gps_noise", "baro_noise",
                     "gps_rate", "baro_rate",
                     "trajectory_mode", "enable_reactive_ftc", "reactive_boost",
                     "oracle_load_share"):
            value = getattr(self, key)
            if value is not None:
                result[key] = value
        return result


def default_full_drake_batch_scenarios() -> list[ExperimentScenario]:
    return [
        ExperimentScenario(
            name="three_drones",
            num_agents=3,
            duration=30.0,
            cable_lengths=[1.00, 1.08, 1.16],
            fault_events=[FaultEvent(cable_index=0, time_seconds=7.0)],
            description="Three-drone single-snap full-Drake reference scenario.",
            tags=["full_drake", "reference_batch"],
        ),
        ExperimentScenario(
            name="five_drones",
            num_agents=5,
            duration=30.0,
            cable_lengths=[1.00, 1.05, 1.10, 1.14, 1.18],
            fault_events=[
                FaultEvent(cable_index=1, time_seconds=7.0),
                FaultEvent(cable_index=3, time_seconds=12.0),
            ],
            description="Five-drone sequential-snap full-Drake reference scenario.",
            tags=["full_drake", "reference_batch"],
        ),
        ExperimentScenario(
            name="seven_drones",
            num_agents=7,
            duration=30.0,
            cable_lengths=[0.98, 1.01, 1.04, 1.07, 1.10, 1.13, 1.16],
            fault_events=[
                FaultEvent(cable_index=1, time_seconds=7.0),
                FaultEvent(cable_index=3, time_seconds=12.0),
                FaultEvent(cable_index=5, time_seconds=14.0),
            ],
            description="Seven-drone sequential-snap full-Drake reference scenario.",
            tags=["full_drake", "reference_batch"],
        ),
    ]


def load_scenarios(manifest_path: Path | None = None) -> list[ExperimentScenario]:
    path = DEFAULT_FULL_DRAKE_BATCH_MANIFEST if manifest_path is None else Path(manifest_path)
    if not path.exists():
        return default_full_drake_batch_scenarios()
    payload = json.loads(path.read_text(encoding="utf-8"))
    return [ExperimentScenario.from_dict(entry) for entry in payload.get("scenarios", [])]


def load_named_scenario(name: str, manifest_path: Path | None = None) -> ExperimentScenario:
    for scenario in load_scenarios(manifest_path):
        if scenario.name == name:
            return scenario
    raise KeyError(f"Scenario '{name}' was not found in manifest {manifest_path or DEFAULT_FULL_DRAKE_BATCH_MANIFEST}")


def write_default_manifest(path: Path = DEFAULT_FULL_DRAKE_BATCH_MANIFEST) -> None:
    payload = {
        "schema_version": 1,
        "description": "Root-owned shared scenario manifest for the canonical 2026-03 full-Drake fault batch.",
        "scenarios": [scenario.to_dict() for scenario in default_full_drake_batch_scenarios()],
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


__all__ = [
    "DEFAULT_FULL_DRAKE_BATCH_MANIFEST",
    "ExperimentScenario",
    "FaultEvent",
    "default_full_drake_batch_scenarios",
    "load_named_scenario",
    "load_scenarios",
    "write_default_manifest",
]