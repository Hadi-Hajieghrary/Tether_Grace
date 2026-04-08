#!/usr/bin/env python3
from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np
from pydrake.all import Box, Cylinder, Rgba, RigidTransform, RotationMatrix, Sphere, StartMeshcat

from topology_invariance_root import Params, simulate


QUAD_COLORS = [
    Rgba(0.85, 0.25, 0.25, 1.0),
    Rgba(0.20, 0.45, 0.85, 1.0),
    Rgba(0.20, 0.65, 0.35, 1.0),
    Rgba(0.85, 0.55, 0.20, 1.0),
    Rgba(0.55, 0.35, 0.85, 1.0),
    Rgba(0.20, 0.70, 0.75, 1.0),
    Rgba(0.85, 0.25, 0.60, 1.0),
    Rgba(0.55, 0.55, 0.20, 1.0),
]


def parse_int_values(raw: str) -> list[int]:
    return [int(part.strip()) for part in raw.split(",") if part.strip()]


def parse_float_values(raw: str) -> list[float]:
    return [float(part.strip()) for part in raw.split(",") if part.strip()]


def make_params(args: argparse.Namespace) -> Params:
    return Params(
        num_agents=args.num_agents,
        cable_lengths=np.array(parse_float_values(args.cable_lengths), dtype=float),
        formation_radius=args.formation_radius,
    )


def make_l1_config(args: argparse.Namespace) -> dict[str, float]:
    return {
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


def run_case(args: argparse.Namespace, params: Params) -> dict[str, np.ndarray]:
    l1_config = make_l1_config(args) if args.mode == "l1" else None
    fault_cables = parse_int_values(args.fault_cables) if args.fault_cables else [args.fault_cable]
    fault_times = parse_float_values(args.fault_times) if args.fault_times else [args.fault_time]
    return simulate(
        args.mode,
        fault_cable=fault_cables,
        fault_time=fault_times,
        duration=args.duration,
        seed=args.seed,
        wind_sigma=args.wind_sigma,
        l1_config=l1_config,
        params=params,
    )


def set_sphere(meshcat, path: str, radius: float, rgba: Rgba) -> None:
    meshcat.SetObject(path, Sphere(radius), rgba)


def hide_object(meshcat, path: str) -> None:
    meshcat.SetObject(path, Sphere(0.001), Rgba(0.0, 0.0, 0.0, 0.0))
    meshcat.SetTransform(path, RigidTransform([0.0, 0.0, -10.0]))


def set_cylinder_between(meshcat, path: str, start: np.ndarray, end: np.ndarray, radius: float, rgba: Rgba) -> None:
    delta = end - start
    length = float(np.linalg.norm(delta))
    if length < 1e-9:
        meshcat.SetObject(path, Sphere(radius), rgba)
        meshcat.SetTransform(path, RigidTransform(start))
        return

    direction = delta / length
    z_axis = np.array([0.0, 0.0, 1.0])
    axis = np.cross(z_axis, direction)
    axis_norm = float(np.linalg.norm(axis))
    dot = float(np.clip(np.dot(z_axis, direction), -1.0, 1.0))
    if axis_norm < 1e-9:
        rotation = RotationMatrix.MakeXRotation(np.pi) if dot < 0.0 else RotationMatrix()
    else:
        axis /= axis_norm
        angle = float(np.arccos(dot))
        rotation = RotationMatrix(RotationMatrix.MakeFromOneVector(direction, 2))

    meshcat.SetObject(path, Cylinder(radius, length), rgba)
    midpoint = 0.5 * (start + end)
    meshcat.SetTransform(path, RigidTransform(rotation, midpoint))


def build_scene(
    meshcat,
    params: Params,
    run: dict[str, np.ndarray],
    show_desired: bool,
    show_paths: bool,
    fault_cables: set[int],
) -> None:
    meshcat.Delete()
    meshcat.SetObject("ground", Box(8.0, 8.0, 0.02), Rgba(0.92, 0.92, 0.92, 1.0))
    meshcat.SetTransform("ground", RigidTransform([0.0, 0.0, -0.01]))

    set_sphere(meshcat, "load/actual", 0.12, Rgba(0.98, 0.63, 0.12, 1.0))
    if show_desired:
        set_sphere(meshcat, "load/desired", 0.09, Rgba(0.20, 0.45, 0.85, 0.45))
    if show_paths:
        meshcat.SetLine("paths/load_actual", run["load_p"].T, 2.0, Rgba(0.98, 0.63, 0.12, 0.85))
        if show_desired:
            meshcat.SetLine("paths/load_desired", run["load_d"].T, 2.0, Rgba(0.20, 0.45, 0.85, 0.55))

    for agent in range(params.num_agents):
        color = QUAD_COLORS[agent % len(QUAD_COLORS)]
        set_sphere(meshcat, f"quads/{agent}/actual", 0.08, color)
        if show_desired:
            set_sphere(meshcat, f"quads/{agent}/desired", 0.05, Rgba(color.r(), color.g(), color.b(), 0.30))
        if show_paths:
            meshcat.SetLine(
                f"paths/quad_{agent}_actual",
                run["quad_p"][:, agent, :].T,
                1.25,
                Rgba(color.r(), color.g(), color.b(), 0.65),
            )
            if show_desired:
                meshcat.SetLine(
                    f"paths/quad_{agent}_desired",
                    run["quad_d"][:, agent, :].T,
                    1.0,
                    Rgba(color.r(), color.g(), color.b(), 0.20),
                )

        label_color = Rgba(0.80, 0.10, 0.10, 1.0) if agent in fault_cables else Rgba(0.10, 0.10, 0.10, 1.0)
        meshcat.SetProperty(f"labels/quad_{agent}", "visible", True)
        meshcat.SetObject(f"labels/quad_{agent}", Sphere(0.001), label_color)
        hide_object(meshcat, f"faults/{agent}/load_end")
        hide_object(meshcat, f"faults/{agent}/quad_end")


def animate(
    meshcat,
    params: Params,
    run: dict[str, np.ndarray],
    sample_stride: int,
    realtime_rate: float,
    record: bool,
    show_desired: bool,
    fault_cables: set[int],
) -> None:
    if record:
        meshcat.StartRecording()

    for sample in range(0, len(run["t"]), sample_stride):
        current_time = float(run["t"][sample])
        meshcat.SetSimulationTime(current_time)
        meshcat.SetRealtimeRate(realtime_rate)

        load_actual = run["load_p"][sample]
        meshcat.SetTransform("load/actual", RigidTransform(load_actual))
        if show_desired:
            load_desired = run["load_d"][sample]
            meshcat.SetTransform("load/desired", RigidTransform(load_desired))
        else:
            load_desired = run["load_d"][sample]

        for agent in range(params.num_agents):
            quad_actual = run["quad_p"][sample, agent]
            cable_active = bool(run["cable_health"][sample, agent] > 0.5)
            quad_color = Rgba(0.85, 0.10, 0.10, 1.0) if not cable_active and agent in fault_cables else QUAD_COLORS[agent % len(QUAD_COLORS)]
            set_sphere(meshcat, f"quads/{agent}/actual", 0.08, quad_color)
            meshcat.SetTransform(f"quads/{agent}/actual", RigidTransform(quad_actual))
            if show_desired:
                quad_desired = run["quad_d"][sample, agent]
                meshcat.SetTransform(f"quads/{agent}/desired", RigidTransform(quad_desired))
            else:
                quad_desired = run["quad_d"][sample, agent]

            actual_color = quad_color
            cable_rgba = Rgba(actual_color.r(), actual_color.g(), actual_color.b(), 0.85 if cable_active else 0.15)
            desired_rgba = Rgba(actual_color.r(), actual_color.g(), actual_color.b(), 0.20)

            if cable_active:
                set_cylinder_between(meshcat, f"cables/{agent}/actual", load_actual, quad_actual, 0.012, cable_rgba)
                hide_object(meshcat, f"faults/{agent}/load_end")
                hide_object(meshcat, f"faults/{agent}/quad_end")
            else:
                hide_object(meshcat, f"cables/{agent}/actual")
                set_sphere(meshcat, f"faults/{agent}/load_end", 0.03, Rgba(0.95, 0.10, 0.10, 1.0))
                set_sphere(meshcat, f"faults/{agent}/quad_end", 0.03, Rgba(0.95, 0.10, 0.10, 1.0))
                meshcat.SetTransform(f"faults/{agent}/load_end", RigidTransform(load_actual))
                meshcat.SetTransform(f"faults/{agent}/quad_end", RigidTransform(quad_actual))
            if show_desired:
                set_cylinder_between(meshcat, f"cables/{agent}/desired", load_desired, quad_desired, 0.006, desired_rgba)

        meshcat.Flush()
        if not record and sample + sample_stride < len(run["t"]):
            dt = float(run["t"][min(sample + sample_stride, len(run["t"]) - 1)] - current_time)
            time.sleep(max(dt / max(realtime_rate, 1e-6), 0.0))

    if record:
        meshcat.StopRecording()
        meshcat.PublishRecording()


def play(
    meshcat,
    params: Params,
    run: dict[str, np.ndarray],
    sample_stride: int,
    realtime_rate: float,
    loop: bool,
    record: bool,
    show_desired: bool,
    fault_cables: set[int],
) -> None:
    first_cycle = True
    while True:
        animate(
            meshcat,
            params,
            run,
            sample_stride=sample_stride,
            realtime_rate=realtime_rate,
            record=record and first_cycle,
            show_desired=show_desired,
            fault_cables=fault_cables,
        )
        first_cycle = False
        if not loop:
            break
        meshcat.SetSimulationTime(0.0)
        meshcat.Flush()
        time.sleep(0.5)


def wait_forever() -> None:
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass


def main() -> None:
    parser = argparse.ArgumentParser(description="Visualize the reduced-order multi-agent payload simulation in Meshcat")
    parser.add_argument("--mode", choices=["cl", "l1"], default="l1")
    parser.add_argument("--num-agents", type=int, default=5)
    parser.add_argument("--cable-lengths", type=str, default="1.04,1.08,1.13,1.10,1.06")
    parser.add_argument("--formation-radius", type=float, default=0.85)
    parser.add_argument("--duration", type=float, default=12.0)
    parser.add_argument("--fault-time", type=float, default=6.0)
    parser.add_argument("--fault-cable", type=int, default=2)
    parser.add_argument("--fault-times", type=str, default="")
    parser.add_argument("--fault-cables", type=str, default="")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--wind-sigma", type=float, default=0.5)
    parser.add_argument("--sample-stride", type=int, default=2)
    parser.add_argument("--realtime-rate", type=float, default=1.0)
    parser.add_argument("--startup-delay", type=float, default=8.0)
    parser.add_argument("--record", action="store_true")
    parser.add_argument("--hold", action="store_true")
    parser.add_argument("--play-once", action="store_true")
    parser.add_argument("--show-desired", action="store_true")
    parser.add_argument("--show-paths", action="store_true")
    parser.add_argument("--l1-omega", type=float, default=3.5)
    parser.add_argument("--l1-predictor-pole", type=float, default=17.0)
    parser.add_argument("--l1-sigma-max", type=float, default=2.0)
    parser.add_argument("--l1-shared-blend", type=float, default=0.5)
    parser.add_argument("--l1-shared-omega", type=float, default=3.0)
    parser.add_argument("--l1-shared-predictor-pole", type=float, default=12.0)
    parser.add_argument("--l1-shared-sigma-max", type=float, default=1.5)
    parser.add_argument("--l1-shared-ramp-time", type=float, default=0.2)
    parser.add_argument("--l1-shared-allocator", type=str, default="z-weighted")
    parser.add_argument("--l1-shared-load-kp-scale", type=float, default=0.5)
    parser.add_argument("--l1-shared-load-kv-scale", type=float, default=0.5)
    parser.add_argument("--l1-local-lateral-weight", type=float, default=0.15)
    parser.add_argument("--l1-local-projection", type=str, default="uniform_xy")
    parser.add_argument("--l1-desired-geometry-mode", type=str, default="healthy_centered_renormalized")
    parser.add_argument("--l1-desired-geometry-ramp-time", type=float, default=0.2)
    args = parser.parse_args()

    params = make_params(args)
    fault_cables = parse_int_values(args.fault_cables) if args.fault_cables else [args.fault_cable]
    for fault_cable in fault_cables:
        if not 0 <= fault_cable < params.num_agents:
            raise ValueError(f"fault cable index {fault_cable} must be in [0, {params.num_agents - 1}]")

    run = run_case(args, params)

    meshcat = StartMeshcat()
    print(f"Meshcat URL: {meshcat.web_url()}", flush=True)
    build_scene(
        meshcat,
        params,
        run,
        show_desired=args.show_desired,
        show_paths=args.show_paths,
        fault_cables=set(fault_cables),
    )

    if args.startup_delay > 0.0:
        print(f"Waiting {args.startup_delay:.1f} seconds before playback so the browser can connect.", flush=True)
        time.sleep(args.startup_delay)

    play(
        meshcat,
        params,
        run,
        sample_stride=max(args.sample_stride, 1),
        realtime_rate=args.realtime_rate,
        loop=not args.play_once,
        record=args.record,
        show_desired=args.show_desired,
        fault_cables=set(fault_cables),
    )

    if args.record:
        print("Meshcat recording published. Use the browser controls to replay the simulation.", flush=True)
    else:
        print("Meshcat scene updated in real time.", flush=True)

    if args.hold or args.record or not args.play_once:
        print("Meshcat server will stay alive until interrupted.", flush=True)
        wait_forever()


if __name__ == "__main__":
    main()