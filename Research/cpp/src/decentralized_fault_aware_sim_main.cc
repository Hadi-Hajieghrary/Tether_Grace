/// @file decentralized_fault_aware_sim_main.cc
///
/// Simulation harness for the decentralised fault-tolerant cooperative
/// lift. Wires together the MultibodyPlant (URDF quadrotors + bead-chain
/// ropes + ground plane), per-drone controllers, fault gates, logging,
/// and the MeshcatVisualizer replay. All controllers are fully local;
/// the harness knows about faults only in order to inject them and to
/// switch the visual rope-hiding state.

#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <drake/common/find_resource.h>
#include <drake/geometry/meshcat.h>
#include <drake/geometry/meshcat_visualizer.h>
#include <drake/geometry/scene_graph.h>
#include <drake/geometry/rgba.h>
#include <drake/math/rigid_transform.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/externally_applied_spatial_force_multiplexer.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/primitives/multiplexer.h>
#include <drake/systems/primitives/demultiplexer.h>
#include <drake/systems/primitives/vector_log_sink.h>
#include <drake/systems/primitives/zero_order_hold.h>

#include "cable_fault_gate.h"
#include "cl_param_estimator.h"
#include "decentralized_local_controller.h"
#include "fault_aware_rope_visualizer.h"
#include "fault_detector.h"
#include "formation_coordinator.h"
#include "meshcat_fault_hider.h"
#include "mpc_local_controller.h"
#include "rope_force_system.h"
#include "rope_segment_tension_probe.h"
#include "rope_utils.h"
#include "safe_hover_controller.h"
#include "control_mode_switcher.h"
#include "trajectory_visualizer.h"

namespace quad_rope_lift {

using drake::geometry::Box;
using drake::geometry::HalfSpace;
using drake::geometry::Meshcat;
using drake::geometry::MeshcatVisualizer;
using drake::geometry::Rgba;
using drake::geometry::Sphere;
using drake::math::RigidTransformd;
using drake::multibody::ContactModel;
using drake::multibody::CoulombFriction;
using drake::multibody::ExternallyAppliedSpatialForceMultiplexer;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::Parser;
using drake::multibody::RigidBody;
using drake::multibody::SpatialInertia;
using drake::multibody::UnitInertia;
using drake::systems::Demultiplexer;
using drake::systems::DiagramBuilder;
using drake::systems::Multiplexer;
using drake::systems::Simulator;
using drake::systems::VectorLogSink;
using drake::systems::ZeroOrderHold;

struct CliOptions {
  int num_quads = 4;
  double duration = 20.0;
  std::string output_dir = "/workspaces/Tether_Grace/Research/decentralized_replays";
  std::string scenario_name = "A_nominal";
  std::string trajectory = "traverse";  // "traverse" | "figure8" | "lemniscate3d"
  int fault_quad_0 = -1;
  double fault_time_0 = -1.0;
  int fault_quad_1 = -1;
  double fault_time_1 = -1.0;
  int fault_quad_2 = -1;
  double fault_time_2 = -1.0;
  // Run the concurrent-learning parameter estimator in parallel.
  bool adaptive = false;
  // Enable the L1 adaptive outer loop inside the baseline controller.
  bool l1_enabled = false;
  // Override the default payload mass (3.0 kg) when ≥ 0.
  double payload_mass_override = -1.0;
  // Controller selection and MPC horizon parameters.
  std::string controller = "baseline";  // "baseline" | "mpc"
  int    mpc_horizon = 5;
  double mpc_tension_max = 100.0;
  // Engage the fault-triggered formation-reshape supervisor.
  bool reshaping_enabled = false;
};

CliOptions ParseCli(int argc, char** argv) {
  CliOptions o;
  for (int i = 1; i < argc; ++i) {
    auto eq = [&](const char* s) { return std::strcmp(argv[i], s) == 0; };
    if (eq("--num-quads") && i + 1 < argc) o.num_quads = std::atoi(argv[++i]);
    else if (eq("--duration") && i + 1 < argc) o.duration = std::atof(argv[++i]);
    else if (eq("--output-dir") && i + 1 < argc) o.output_dir = argv[++i];
    else if (eq("--scenario") && i + 1 < argc) o.scenario_name = argv[++i];
    else if (eq("--fault-0-quad") && i + 1 < argc) o.fault_quad_0 = std::atoi(argv[++i]);
    else if (eq("--fault-0-time") && i + 1 < argc) o.fault_time_0 = std::atof(argv[++i]);
    else if (eq("--fault-1-quad") && i + 1 < argc) o.fault_quad_1 = std::atoi(argv[++i]);
    else if (eq("--fault-1-time") && i + 1 < argc) o.fault_time_1 = std::atof(argv[++i]);
    else if (eq("--fault-2-quad") && i + 1 < argc) o.fault_quad_2 = std::atoi(argv[++i]);
    else if (eq("--fault-2-time") && i + 1 < argc) o.fault_time_2 = std::atof(argv[++i]);
    else if (eq("--trajectory") && i + 1 < argc) o.trajectory = argv[++i];
    else if (eq("--adaptive")) o.adaptive = true;
    else if (eq("--l1-enabled")) o.l1_enabled = true;
    else if (eq("--controller") && i + 1 < argc) o.controller = argv[++i];
    else if (eq("--mpc-horizon") && i + 1 < argc) o.mpc_horizon = std::atoi(argv[++i]);
    else if (eq("--mpc-tension-max") && i + 1 < argc) o.mpc_tension_max = std::atof(argv[++i]);
    else if (eq("--reshaping-enabled")) o.reshaping_enabled = true;
    else if (eq("--payload-mass") && i + 1 < argc) o.payload_mass_override = std::atof(argv[++i]);
  }
  return o;
}

bool IsFaultedAt(const CliOptions& o, int q) {
  return (q == o.fault_quad_0 && o.fault_time_0 >= 0.0) ||
         (q == o.fault_quad_1 && o.fault_time_1 >= 0.0) ||
         (q == o.fault_quad_2 && o.fault_time_2 >= 0.0);
}

double FaultTimeFor(const CliOptions& o, int q) {
  if (q == o.fault_quad_0 && o.fault_time_0 >= 0.0) return o.fault_time_0;
  if (q == o.fault_quad_1 && o.fault_time_1 >= 0.0) return o.fault_time_1;
  if (q == o.fault_quad_2 && o.fault_time_2 >= 0.0) return o.fault_time_2;
  return -1.0;
}

// Build a waypoint trajectory based on the requested shape.
std::vector<TrajectoryWaypoint> BuildTrajectory(
    const std::string& shape, double duration,
    double z_low, double z_high) {
  std::vector<TrajectoryWaypoint> wps;
  if (shape == "lemniscate3d") {
    // 3-D lemniscate: a Bernoulli figure-8 in (x,y) with superimposed
    // vertical oscillation, so the payload traces a continuously-
    // varying 3-D path. Designed for the 5-drone N=5 campaign: gives a
    // non-trivial dynamic reference that is in quasi-steady motion by
    // t=10s and stays dynamic throughout.
    //
    //   x(phi) = a * cos(phi)     / (1 + sin²(phi))
    //   y(phi) = a * sin(phi)cos(phi) / (1 + sin²(phi))
    //   z(phi) = z_high + Az * sin(phi)             (altitude once per loop)
    //
    const double a = 3.0;
    const double T = 12.0;              // lemniscate period (shorter than 2-D
                                        //   figure8 T=16 so each loop is
                                        //   more dynamic)
    const double Az = 0.35;             // altitude oscillation amplitude [m]
    TrajectoryWaypoint w;
    // Pickup & lift: reach (a, 0, z_high) by t=5 s so the system can
    // settle into steady motion before the first fault at t>=10.
    w.position = {a, 0, z_low};  w.arrival_time = 0.0; w.hold_time = 2.0; wps.push_back(w);
    w.position = {a, 0, z_high}; w.arrival_time = 5.0; w.hold_time = 1.0; wps.push_back(w);

    // Trace the 3-D lemniscate densely for the bulk of the mission.
    const double traj_start = 6.0;
    const double traj_end   = duration - 3.0;
    const double sample_dt  = 0.2;
    for (double t = traj_start; t < traj_end; t += sample_dt) {
      const double phi   = (t - traj_start) * 2.0 * M_PI / T;
      const double denom = 1.0 + std::sin(phi) * std::sin(phi);
      const double x     = a * std::cos(phi) / denom;
      const double y     = a * std::sin(phi) * std::cos(phi) / denom;
      const double z     = z_high + Az * std::sin(phi);
      TrajectoryWaypoint wp;
      wp.position = {x, y, z};
      wp.arrival_time = t;
      wp.hold_time = 0.0;
      wps.push_back(wp);
    }
    // Return to start + descend
    TrajectoryWaypoint wp_return;
    wp_return.position = {a, 0, z_high};
    wp_return.arrival_time = duration - 2.0;
    wp_return.hold_time = 0.5;
    wps.push_back(wp_return);

    TrajectoryWaypoint wpf;
    wpf.position = {a, 0, z_low};
    wpf.arrival_time = duration - 0.1;
    wpf.hold_time = 2.0;
    wps.push_back(wpf);

  } else if (shape == "figure8") {
    // Lemniscate of Bernoulli. Larger scale (a = 3 m span of ±3 in x) and
    // longer period (T = 16 s ⇒ peak speed ≈ 1 m/s, peak lateral accel
    // ≈ 0.9 m/s²) so the underslung payload can track the reference
    // without excessive pendulum lag. Sampled densely (every 0.2 s) so
    // the linear-interpolation between waypoints is effectively smooth.
    const double a = 3.0;
    const double T = 16.0;
    TrajectoryWaypoint w;
    // Pickup → lift
    w.position = {a, 0, z_low};  w.arrival_time = 0.0; w.hold_time = 2.0; wps.push_back(w);
    w.position = {a, 0, z_high}; w.arrival_time = 5.0; w.hold_time = 1.0; wps.push_back(w);

    // Trace figure-8 for the bulk of the scenario, starting at (a, 0)
    // (the right extremum of the lemniscate at phi = 0) and running for
    // an integer number of periods that fills the remaining time minus
    // a 3 s descent.
    const double traj_start = 6.0;
    const double traj_end = duration - 3.0;
    const double sample_dt = 0.2;   // 5 waypoints per second
    for (double t = traj_start; t < traj_end; t += sample_dt) {
      const double phi = (t - traj_start) * 2.0 * M_PI / T;
      const double denom = 1.0 + std::sin(phi) * std::sin(phi);
      const double x = a * std::cos(phi) / denom;
      const double y = a * std::sin(phi) * std::cos(phi) / denom;
      TrajectoryWaypoint wp;
      wp.position = {x, y, z_high};
      wp.arrival_time = t;
      wp.hold_time = 0.0;
      wps.push_back(wp);
    }
    // Return to start + descend
    TrajectoryWaypoint wp_return;
    wp_return.position = {a, 0, z_high};
    wp_return.arrival_time = duration - 2.0;
    wp_return.hold_time = 0.5;
    wps.push_back(wp_return);

    TrajectoryWaypoint wpf;
    wpf.position = {a, 0, z_low};
    wpf.arrival_time = duration - 0.1;
    wpf.hold_time = 2.0;
    wps.push_back(wpf);
  } else {
    // Default straight traverse: pickup → lift → traverse → descend
    TrajectoryWaypoint w;
    w.position = {0, 0, z_low};   w.arrival_time = 0.0;              w.hold_time = 2.0; wps.push_back(w);
    w.position = {0, 0, z_high};  w.arrival_time = 5.0;              w.hold_time = 2.0; wps.push_back(w);
    w.position = {3, 0, z_high};  w.arrival_time = duration * 0.60;  w.hold_time = 2.0; wps.push_back(w);
    w.position = {3, 0, z_low};   w.arrival_time = duration * 0.90;  w.hold_time = 2.0; wps.push_back(w);
  }
  return wps;
}

/// Outputs the shared reference (position, velocity) at each time so we can log
/// it alongside the actual payload state. Mirrors the controller's internal
/// ComputeTrajectory but is a standalone LeafSystem.
class ReferenceSource final : public drake::systems::LeafSystem<double> {
 public:
  explicit ReferenceSource(std::vector<TrajectoryWaypoint> wps)
      : waypoints_(std::move(wps)) {
    DeclareVectorOutputPort(
        "p_ref", drake::systems::BasicVector<double>(3),
        &ReferenceSource::CalcPos);
    DeclareVectorOutputPort(
        "v_ref", drake::systems::BasicVector<double>(3),
        &ReferenceSource::CalcVel);
  }
 private:
  void ComputeRef(double t, Eigen::Vector3d* p, Eigen::Vector3d* v) const {
    double seg_start = 0.0;
    for (size_t i = 0; i < waypoints_.size(); ++i) {
      const auto& wp = waypoints_[i];
      const double seg_end = wp.arrival_time;
      const double hold_end = seg_end + wp.hold_time;
      if (t <= seg_end) {
        if (i == 0) { *p = wp.position; *v = Eigen::Vector3d::Zero(); }
        else {
          const auto& prev = waypoints_[i - 1];
          const double dur = seg_end - seg_start;
          if (dur > 1e-6) {
            const double a = (t - seg_start) / dur;
            *p = (1.0 - a) * prev.position + a * wp.position;
            *v = (wp.position - prev.position) / dur;
          } else { *p = wp.position; *v = Eigen::Vector3d::Zero(); }
        }
        return;
      } else if (t <= hold_end) {
        *p = wp.position; *v = Eigen::Vector3d::Zero(); return;
      }
      seg_start = hold_end;
    }
    *p = waypoints_.back().position; *v = Eigen::Vector3d::Zero();
  }
  void CalcPos(const drake::systems::Context<double>& c,
               drake::systems::BasicVector<double>* o) const {
    Eigen::Vector3d p, v; ComputeRef(c.get_time(), &p, &v); o->SetFromVector(p);
  }
  void CalcVel(const drake::systems::Context<double>& c,
               drake::systems::BasicVector<double>* o) const {
    Eigen::Vector3d p, v; ComputeRef(c.get_time(), &p, &v); o->SetFromVector(v);
  }
  std::vector<TrajectoryWaypoint> waypoints_;
};

int DoMain(int argc, char** argv) {
  const CliOptions opts = ParseCli(argc, argv);
  const int N = opts.num_quads;

  std::cout << "=== Decentralized Fault-Aware Cooperative-Lift Sim ===\n"
            << "Scenario: " << opts.scenario_name << "\n"
            << "Duration: " << opts.duration << " s\n"
            << "Drones:   " << N << "\n";
  for (int q = 0; q < N; ++q) {
    if (IsFaultedAt(opts, q)) {
      std::cout << "  Fault @ quad " << q << " at t=" << FaultTimeFor(opts, q) << "s\n";
    }
  }

  // Physics configuration (values match the Tether_Lift baseline).
  const double quadcopter_mass = 1.5;
  const Eigen::Vector3d quadcopter_dimensions(0.3, 0.3, 0.08);
  // Default payload mass 3.0 kg, overridden by --payload-mass for
  // robustness experiments.
  const double payload_mass =
      (opts.payload_mass_override > 0.0) ? opts.payload_mass_override : 3.0;
  const double payload_radius = 0.15;
  // Rope length chosen so drones start with rope slightly slack (no
  // startup impulse): drone-to-payload distance at t=0 is √(r² + Δz²)
  // = √(0.8² + 0.85²) ≈ 1.17 m, so rope_length ≥ 1.2 m leaves the rope
  // slack; pickup happens smoothly as drones ascend.
  const double rope_length = 1.25;
  const double rope_total_mass = 0.2;
  // Realistic 6 mm aramid-core lift sling (e.g., Dyneema / Technora), the
  // common choice for real aerial-transport missions where low stretch is
  // required. Effective end-to-end rope stiffness EA/L ≈ 3 kN/m (much
  // stiffer than braided polyester).
  //   Per-segment (8 segments in series) k_seg = N_seg × k_rope = 25 kN/m.
  //   Damping c_seg = 60 N·s/m → ζ ≈ 1.2 (slightly overdamped → the bead
  //   chain tracks the drone / payload motion without ringing).
  //   Numerical stability: ω_n = √(k_seg/m_bead) ≈ 1000 rad/s ⇒ dt_stable
  //   ≈ 2 ms; sim step 2 × 10⁻⁴ s leaves a factor-10 safety margin.
  //   Static stretch at nominal 7.4 N per rope: 2.4 mm (rope behaves as
  //   essentially rigid at cruise, matching real aerial-sling behaviour).
  const double segment_stiffness = 25000.0;
  const double segment_damping = 60.0;
  const int num_rope_beads = 8;
  const double formation_radius = 0.8;
  const double initial_altitude = 1.0;
  const double final_altitude = 3.0;
  const double gravity = 9.81;
  const double simulation_time_step = 2e-4;  // Matches Tether_Lift baseline
  const double load_per_rope = (payload_mass * gravity) / static_cast<double>(N);

  // ============ TRAJECTORY + HOVER-EQUILIBRIUM GEOMETRY ============
  //
  // `waypoints` describes the PAYLOAD target trajectory. The drones fly in
  // formation directly above the payload. The physical rope does not run
  // between body centres but between attachment points:
  //
  //   drone attach  =  drone_center + quad_attachment_offset
  //                 = drone_center + (0, 0, -dz_quad) where
  //                   dz_quad = quad_half_thickness + rope_stub
  //
  //   payload attach = payload_center + (0.3·r cos φᵢ, 0.3·r sin φᵢ, 0)
  //
  // Therefore the effective horizontal rope span is
  //         r_eff = 0.7 · formation_radius
  // and the effective vertical span is
  //         Δz_attach = rope_drop - dz_quad
  // where rope_drop denotes the drone-centre to payload-centre vertical
  // offset that the controller uses for its slot reference.
  //
  // At static hover equilibrium the rope chord between attachments must
  // slightly exceed the rest length L to carry the payload weight:
  //
  //   L_chord = √(r_eff² + Δz_attach²) = L + δ,
  //   T       = m_L g · L_chord / (N · Δz_attach),
  //   δ       = T / k_eff   (k_eff = k_seg / N_seg).
  //
  // Solving the fixed point gives δ ≈ 3 mm and rope_drop ≈ 1.20 m
  // (matching the original empirical calibration). Spawning the drones at
  // p_payload + (r cos φ, r sin φ, rope_drop) with beads linearly
  // interpolated between the attachment endpoints places the system
  // directly at its static-hover fixed point — no free-fall, no
  // snap-taut, no excitation of the 5 Hz bead-chain axial mode.
  const Eigen::Vector3d quad_attachment_offset(
      0, 0, -quadcopter_dimensions[2] / 2.0 - 0.05);
  const double dz_quad_attach = -quad_attachment_offset.z();  // 0.09 m
  const double r_eff = formation_radius * 0.7;  // 0.7 · r because payload
                                                //   attach is offset by 0.3·r
                                                //   in the same radial direction
                                                //   as the drone slot.
  const double k_seg_eff = segment_stiffness / (num_rope_beads + 1);  // series

  // Fixed-point iteration for δ — converges in <5 iterations.
  double rope_stretch_eq = 0.0;
  double dz_attach_eq = std::sqrt(rope_length * rope_length - r_eff * r_eff);
  for (int it = 0; it < 20; ++it) {
    const double L_chord = rope_length + rope_stretch_eq;
    dz_attach_eq = std::sqrt(std::max(1e-9,
        L_chord * L_chord - r_eff * r_eff));
    const double T_rope = payload_mass * gravity * L_chord / (N * dz_attach_eq);
    rope_stretch_eq = T_rope / k_seg_eff;
  }
  const double L_chord_eq = rope_length + rope_stretch_eq;
  const double rope_drop = dz_attach_eq + dz_quad_attach;
  std::cout << "Hover equilibrium: r_eff = " << r_eff
            << " m, rope stretch δ = " << rope_stretch_eq * 1000
            << " mm, L_chord = " << L_chord_eq * 1000
            << " mm, Δz_attach = " << dz_attach_eq * 1000
            << " mm, rope_drop = " << rope_drop * 1000 << " mm\n";

  std::vector<TrajectoryWaypoint> waypoints =
      BuildTrajectory(opts.trajectory, opts.duration,
                      initial_altitude, final_altitude);

  // Controller waypoints = payload waypoints lifted by rope_drop.
  std::vector<TrajectoryWaypoint> ctrl_waypoints = waypoints;
  for (auto& wp : ctrl_waypoints) wp.position.z() += rope_drop;

  std::cout << "Trajectory: " << opts.trajectory
            << " (" << waypoints.size() << " waypoints, payload frame; "
            << "drone formation flies " << rope_drop << " m higher)\n";

  // ============ DIAGRAM ============
  DiagramBuilder<double> builder;
  auto [plant, scene_graph] = drake::multibody::AddMultibodyPlantSceneGraph(
      &builder, simulation_time_step);
  plant.set_contact_model(ContactModel::kPoint);

  const ModelInstanceIndex model_instance =
      plant.AddModelInstance("fault_aware_system");

  // --- Quadcopters with URDF visual ---
  const SpatialInertia<double> quad_inertia(
      quadcopter_mass, Eigen::Vector3d::Zero(),
      UnitInertia<double>::SolidBox(quadcopter_dimensions[0],
                                    quadcopter_dimensions[1],
                                    quadcopter_dimensions[2]));

  std::vector<const RigidBody<double>*> quadcopter_bodies;
  Parser parser(&plant);

  for (int i = 0; i < N; ++i) {
    const std::string quad_name = "quadcopter_" + std::to_string(i);
    const RigidBody<double>& quad_body =
        plant.AddRigidBody(quad_name, model_instance, quad_inertia);
    quadcopter_bodies.push_back(&quad_body);

    parser.SetAutoRenaming(true);
    auto visual_instances = parser.AddModels(
        drake::FindResourceOrThrow("drake/examples/quadrotor/quadrotor.urdf"));
    const RigidBody<double>& visual_base =
        plant.GetBodyByName("base_link", visual_instances[0]);
    plant.WeldFrames(quad_body.body_frame(), visual_base.body_frame(),
                     RigidTransformd::Identity());
  }

  // --- Payload (red sphere) ---
  const SpatialInertia<double> payload_inertia(
      payload_mass, Eigen::Vector3d::Zero(),
      UnitInertia<double>::SolidSphere(payload_radius));
  const RigidBody<double>& payload_body =
      plant.AddRigidBody("payload", model_instance, payload_inertia);

  const CoulombFriction<double> ground_friction(0.5, 0.3);
  plant.RegisterCollisionGeometry(payload_body, RigidTransformd::Identity(),
                                  Sphere(payload_radius), "payload_collision",
                                  ground_friction);
  plant.RegisterVisualGeometry(payload_body, RigidTransformd::Identity(),
                               Sphere(payload_radius), "payload_visual",
                               Eigen::Vector4d(0.8, 0.2, 0.2, 1.0));

  // --- Ground plane ---
  plant.RegisterCollisionGeometry(plant.world_body(),
                                  RigidTransformd::Identity(), HalfSpace(),
                                  "ground_collision", ground_friction);
  plant.RegisterVisualGeometry(plant.world_body(),
                               RigidTransformd(Eigen::Vector3d(0, 0, -0.02)),
                               Box(10, 10, 0.04), "ground_visual",
                               Eigen::Vector4d(0.7, 0.7, 0.7, 1.0));

  // --- Rope beads with color coding ---
  std::vector<std::vector<const RigidBody<double>*>> bead_chains(N);
  std::vector<RopeParameters> rope_params_vec(N);
  for (int q = 0; q < N; ++q) {
    rope_params_vec[q] = ComputeRopeParameters(
        num_rope_beads, rope_length, rope_total_mass,
        segment_stiffness, segment_damping, true, 0.012);
    const SpatialInertia<double> bead_inertia(
        rope_params_vec[q].bead_mass, Eigen::Vector3d::Zero(),
        UnitInertia<double>::SolidSphere(rope_params_vec[q].bead_radius));
    const double hue = static_cast<double>(q) / N;
    const Eigen::Vector4d bead_color(0.3 + 0.4 * hue, 0.3, 0.3 + 0.4 * (1.0 - hue), 1.0);
    for (int i = 0; i < num_rope_beads; ++i) {
      const std::string name =
          "rope_" + std::to_string(q) + "_bead_" + std::to_string(i);
      const RigidBody<double>& bead =
          plant.AddRigidBody(name, model_instance, bead_inertia);
      plant.RegisterCollisionGeometry(bead, RigidTransformd::Identity(),
                                      Sphere(rope_params_vec[q].bead_radius),
                                      name + "_collision", ground_friction);
      plant.RegisterVisualGeometry(bead, RigidTransformd::Identity(),
                                   Sphere(rope_params_vec[q].bead_radius),
                                   name + "_visual", bead_color);
      bead_chains[q].push_back(&bead);
    }
  }

  plant.Finalize();

  // ============ PER-QUAD ROPE SYSTEMS + TENSION GATES + SCALAR TENSIONS ============
  // quad_attachment_offset is declared earlier (used in geometry solve).

  std::vector<RopeForceSystem*> rope_systems(N);
  std::vector<RopeSegmentTensionProbe*> segment_probes(N);
  std::vector<ZeroOrderHold<double>*> tension_holds(N);
  std::vector<Demultiplexer<double>*> tension_demuxes(N);
  std::vector<CableFaultGate*> force_gates(N, nullptr);
  std::vector<TensionFaultGate*> tension_gates(N, nullptr);
  std::vector<CLParamEstimator*> cl_estimators(N, nullptr);
  const int num_rope_segments = num_rope_beads + 1;  // N_beads + 1 segments

  for (int q = 0; q < N; ++q) {
    const double angle = 2.0 * M_PI * q / N;
    const Eigen::Vector3d payload_attach(
        formation_radius * std::cos(angle) * 0.3,  // small offset on payload
        formation_radius * std::sin(angle) * 0.3, 0);

    rope_systems[q] = builder.AddSystem<RopeForceSystem>(
        plant, *quadcopter_bodies[q], payload_body, bead_chains[q],
        quad_attachment_offset, payload_attach, rope_length,
        rope_params_vec[q].segment_stiffness,
        rope_params_vec[q].segment_damping);
    builder.Connect(plant.get_state_output_port(),
                    rope_systems[q]->get_plant_state_input_port());

    // Per-segment tension diagnostic probe (N_seg outputs, observer only).
    segment_probes[q] = builder.AddSystem<RopeSegmentTensionProbe>(
        plant, *quadcopter_bodies[q], payload_body, bead_chains[q],
        quad_attachment_offset, payload_attach, rope_length,
        rope_params_vec[q].segment_stiffness,
        rope_params_vec[q].segment_damping);
    builder.Connect(plant.get_state_output_port(),
                    segment_probes[q]->get_plant_state_input_port());

    // Optional concurrent-learning parameter estimator (observer only).
    if (opts.adaptive) {
      CLParamEstimatorParams ep;
      ep.rope_rest_length = rope_length;
      ep.formation_radius = formation_radius;
      ep.quad_attachment_z_offset = -quad_attachment_offset.z();
      ep.update_period = simulation_time_step;
      ep.initial_theta << 3.0 / N, rope_params_vec[q].segment_stiffness
                                      / static_cast<double>(num_rope_beads + 1);
      cl_estimators[q] = builder.AddSystem<CLParamEstimator>(
          plant, *quadcopter_bodies[q], payload_body,
          quad_attachment_offset, payload_attach, ep);
      builder.Connect(plant.get_state_output_port(),
                      cl_estimators[q]->get_plant_state_input_port());
    }

    tension_holds[q] = builder.AddSystem<ZeroOrderHold<double>>(
        simulation_time_step, 4);

    // For faulted drones, gate the tension signal so peers see 0 after fault.
    if (IsFaultedAt(opts, q)) {
      tension_gates[q] = builder.AddSystem<TensionFaultGate>(FaultTimeFor(opts, q));
      builder.Connect(rope_systems[q]->get_tension_output_port(),
                      tension_gates[q]->get_tension_input_port());
      builder.Connect(tension_gates[q]->get_tension_output_port(),
                      tension_holds[q]->get_input_port());
    } else {
      builder.Connect(rope_systems[q]->get_tension_output_port(),
                      tension_holds[q]->get_input_port());
    }

    // Demux the 4D [T, fx, fy, fz] into scalar tension (port 0) + 3D force
    tension_demuxes[q] = builder.AddSystem<Demultiplexer<double>>(4, 1);
    builder.Connect(tension_holds[q]->get_output_port(),
                    tension_demuxes[q]->get_input_port(0));

    // Wire the CL estimator's tension input (if enabled for this drone)
    // to the gated, held rope tension — same signal the controller sees.
    if (cl_estimators[q]) {
      builder.Connect(tension_holds[q]->get_output_port(),
                      cl_estimators[q]->get_tension_input_port());
    }
  }

  // ============ PER-DRONE CONTROLLERS ============
  // Two backends, selected at runtime via `--controller`:
  //   baseline — DecentralizedLocalController (single-step QP).
  //   mpc      — MpcLocalController (N-step receding horizon, hard
  //              linearised tension-ceiling, DARE terminal cost).
  // Both publish the same abstract `control_force` output and the same
  // 6-scalar `control_vector` + 13-scalar `diagnostics` signatures, so
  // the rest of the diagram is controller-agnostic.
  const bool use_mpc = (opts.controller == "mpc");
  std::vector<DecentralizedLocalController*> controllers(N, nullptr);
  std::vector<MpcLocalController*>           mpc_controllers(N, nullptr);
  // Uniform accessor — returns the LeafSystem pointer regardless of
  // backend. Used only for wiring / log connections.
  auto ctrl_system = [&](int q) -> drake::systems::LeafSystem<double>* {
    return use_mpc
        ? static_cast<drake::systems::LeafSystem<double>*>(mpc_controllers[q])
        : static_cast<drake::systems::LeafSystem<double>*>(controllers[q]);
  };

  for (int q = 0; q < N; ++q) {
    const double angle = 2.0 * M_PI * q / N;
    const Eigen::Vector3d formation_offset(
        formation_radius * std::cos(angle),
        formation_radius * std::sin(angle), 0);

    if (!use_mpc) {
      DecentralizedLocalController::Params cp;
      cp.formation_offset = formation_offset;
      cp.waypoints = waypoints;
      cp.rope_length = rope_length;
      cp.rope_drop = rope_drop;
      cp.pickup_target_tension_nominal = load_per_rope;
      cp.altitude_kp = 100.0;   cp.altitude_kd = 24.0;
      cp.position_kp = 30.0;    cp.position_kd = 15.0;
      cp.attitude_kp = 25.0;    cp.attitude_kd = 4.0;
      cp.max_tilt = 0.6;
      cp.swing_kd = 0.8;
      cp.swing_offset_max = 0.3;
      cp.w_track = 1.0;
      cp.w_swing = 0.3;
      cp.w_effort = 0.02;
      cp.min_thrust = 0.0;
      cp.max_thrust = 150.0;
      cp.max_torque = 10.0;
      cp.l1_enabled = opts.l1_enabled;
      cp.l1_control_step = simulation_time_step;
      controllers[q] = builder.AddSystem<DecentralizedLocalController>(
          plant, *quadcopter_bodies[q], payload_body, cp);
      controllers[q]->set_mass(quadcopter_bodies[q]->default_mass());
    } else {
      MpcLocalController::Params mp;
      mp.formation_offset = formation_offset;
      mp.waypoints = waypoints;
      mp.rope_length = rope_length;
      mp.rope_drop = rope_drop;
      mp.pickup_target_tension_nominal = load_per_rope;
      mp.altitude_kp = 100.0;   mp.altitude_kd = 24.0;
      mp.position_kp = 30.0;    mp.position_kd = 15.0;
      mp.attitude_kp = 25.0;    mp.attitude_kd = 4.0;
      mp.max_tilt = 0.6;
      mp.swing_kd = 0.8;
      mp.swing_offset_max = 0.3;
      mp.min_thrust = 0.0;
      mp.max_thrust = 150.0;
      mp.max_torque = 10.0;
      mp.horizon_steps = opts.mpc_horizon;
      // MPC prediction step is decoupled from the sim step: 10 ms per
      // horizon step gives a useful 50-ms lookahead at N_p = 5 (the
      // payload pendulum period is ~2.4 s so we still only see ~2 %
      // of a cycle, but the DARE terminal cost absorbs the remainder).
      mp.dt_mpc = 0.01;
      mp.T_max_safe = opts.mpc_tension_max;
      mp.L_eff_body = std::sqrt(
          formation_radius * formation_radius + rope_drop * rope_drop)
          - rope_stretch_eq;          // hover-equilibrium correction
      mp.k_eff = segment_stiffness / (num_rope_beads + 1);
      mpc_controllers[q] = builder.AddSystem<MpcLocalController>(
          plant, *quadcopter_bodies[q], payload_body, mp);
      mpc_controllers[q]->set_mass(quadcopter_bodies[q]->default_mass());
    }

    builder.Connect(plant.get_state_output_port(),
                    ctrl_system(q)->GetInputPort("plant_state"));
    builder.Connect(tension_holds[q]->get_output_port(),
                    ctrl_system(q)->GetInputPort("rope_tension"));
  }

  // ============ PHASE-H: FORMATION RESHAPING (optional) ============
  // Closed-form equiangular-assignment supervisor: subscribes to the
  // N scalar tensions (post-fault-gate demuxes), broadcasts a
  // 3N-vector of dynamic formation offsets interpolated with a quintic
  // smoothstep over T_trans = 5 s. Enabled by `--reshaping-enabled`.
  if (opts.reshaping_enabled) {
    // Build an N-vector of scalar tensions → FaultDetector input.
    auto* tension_mux = builder.AddSystem<Multiplexer<double>>(
        std::vector<int>(N, 1));
    for (int q = 0; q < N; ++q) {
      builder.Connect(tension_demuxes[q]->get_output_port(0),
                      tension_mux->get_input_port(q));
    }
    FaultDetector::Params fd_p;
    fd_p.num_drones = N;
    fd_p.update_period = simulation_time_step;
    auto* fault_detector = builder.AddSystem<FaultDetector>(fd_p);
    builder.Connect(tension_mux->get_output_port(0),
                    fault_detector->get_tensions_input_port());

    FormationCoordinator::Params fc_p;
    fc_p.num_drones = N;
    fc_p.formation_radius = formation_radius;
    fc_p.t_trans = 5.0;
    fc_p.update_period = 5.0e-3;  // 200 Hz update rate for the supervisor
    auto* formation_coord = builder.AddSystem<FormationCoordinator>(fc_p);
    builder.Connect(fault_detector->get_fault_id_output_port(),
                    formation_coord->get_fault_id_input_port());

    // Demux the 3N-vector into per-drone 3-vectors and wire each to
    // its controller's `formation_offset_override` port.
    auto* offset_demux = builder.AddSystem<Demultiplexer<double>>(
        3 * N, 3);
    builder.Connect(
        formation_coord->get_formation_offsets_output_port(),
        offset_demux->get_input_port());
    for (int q = 0; q < N; ++q) {
      builder.Connect(offset_demux->get_output_port(q),
                      ctrl_system(q)->GetInputPort(
                          "formation_offset_override"));
    }
    std::cout << "FormationCoordinator wired: T_trans="
              << fc_p.t_trans << " s, N=" << N << "\n";
  }

  // ============ FORCE COMBINER (Controller + Rope forces) ============
  auto& force_combiner = *builder.AddSystem<ExternallyAppliedSpatialForceMultiplexer>(
      2 * N);

  // Fault-quad safe-hover handover
  std::vector<SafeHoverController*> safe_ctrls(N, nullptr);
  std::vector<ControlModeSwitcher*> switchers(N, nullptr);

  for (int q = 0; q < N; ++q) {
    // Rope forces (possibly gated)
    if (IsFaultedAt(opts, q)) {
      force_gates[q] = builder.AddSystem<CableFaultGate>(FaultTimeFor(opts, q));
      builder.Connect(rope_systems[q]->get_forces_output_port(),
                      force_gates[q]->get_forces_input_port());
      builder.Connect(force_gates[q]->get_forces_output_port(),
                      force_combiner.get_input_port(2 * q + 1));
    } else {
      builder.Connect(rope_systems[q]->get_forces_output_port(),
                      force_combiner.get_input_port(2 * q + 1));
    }

    // Controller forces (with safe-hover handover for fault quad)
    if (IsFaultedAt(opts, q)) {
      const double angle = 2.0 * M_PI * q / N;
      const double safe_r = formation_radius * 2.5;
      const Eigen::Vector3d safe_pos(safe_r * std::cos(angle),
                                     safe_r * std::sin(angle), 5.0);
      safe_ctrls[q] = builder.AddSystem<SafeHoverController>(
          plant, *quadcopter_bodies[q], safe_pos, quadcopter_mass);
      builder.Connect(plant.get_state_output_port(),
                      safe_ctrls[q]->get_plant_state_input_port());
      switchers[q] = builder.AddSystem<ControlModeSwitcher>(FaultTimeFor(opts, q));
      builder.Connect(ctrl_system(q)->GetOutputPort("control_force"),
                      switchers[q]->get_normal_input_port());
      builder.Connect(safe_ctrls[q]->get_control_output_port(),
                      switchers[q]->get_safe_input_port());
      builder.Connect(switchers[q]->get_active_output_port(),
                      force_combiner.get_input_port(2 * q));
    } else {
      builder.Connect(ctrl_system(q)->GetOutputPort("control_force"),
                      force_combiner.get_input_port(2 * q));
    }
  }

  builder.Connect(force_combiner.get_output_port(),
                  plant.get_applied_spatial_force_input_port());

  // ============ REFERENCE SOURCE + LOGGING ============
  auto* ref_source = builder.AddSystem<ReferenceSource>(waypoints);

  // Log plant state (positions + velocities, all bodies)
  auto* state_log = builder.AddSystem<VectorLogSink<double>>(
      plant.get_state_output_port().size());
  builder.Connect(plant.get_state_output_port(), state_log->get_input_port());

  // Log reference trajectory
  auto* ref_pos_log = builder.AddSystem<VectorLogSink<double>>(3);
  auto* ref_vel_log = builder.AddSystem<VectorLogSink<double>>(3);
  builder.Connect(ref_source->get_output_port(0), ref_pos_log->get_input_port());
  builder.Connect(ref_source->get_output_port(1), ref_vel_log->get_input_port());

  // Log per-drone controller vector [tau_x, tau_y, tau_z, f_x, f_y, f_z]
  std::vector<VectorLogSink<double>*> control_logs(N);
  for (int q = 0; q < N; ++q) {
    control_logs[q] = builder.AddSystem<VectorLogSink<double>>(6);
    builder.Connect(ctrl_system(q)->GetOutputPort("control_vector"),
                    control_logs[q]->get_input_port());
  }

  // Log per-drone diagnostics (13 signals; layout documented in each
  // controller's CalcDiagnostics method).
  std::vector<VectorLogSink<double>*> diag_logs(N);
  for (int q = 0; q < N; ++q) {
    diag_logs[q] = builder.AddSystem<VectorLogSink<double>>(13);
    builder.Connect(ctrl_system(q)->GetOutputPort("diagnostics"),
                    diag_logs[q]->get_input_port());
  }

  // Per-rope scalar-tension logs (local signal, demuxed from the 4-D rope
  // tension vector). Each drone ONLY has access to its own rope_tension_i.
  std::vector<VectorLogSink<double>*> tension_logs(N);
  for (int q = 0; q < N; ++q) {
    tension_logs[q] = builder.AddSystem<VectorLogSink<double>>(1);
    builder.Connect(tension_demuxes[q]->get_output_port(0),
                    tension_logs[q]->get_input_port());
  }

  // Per-rope per-segment tensions (observer; never fed to the controller).
  // Expose T_{j,q}(t) for j = 1..N_seg, q = 0..N-1.
  std::vector<VectorLogSink<double>*> segment_tension_logs(N);
  for (int q = 0; q < N; ++q) {
    segment_tension_logs[q] =
        builder.AddSystem<VectorLogSink<double>>(num_rope_segments);
    builder.Connect(segment_probes[q]->get_tensions_output_port(),
                    segment_tension_logs[q]->get_input_port());
  }

  // L1 state logs; only connected when the baseline controller runs
  // with L1 enabled (the MPC does not carry an L1 inner loop).
  std::vector<VectorLogSink<double>*> l1_state_logs(N, nullptr);
  if (opts.l1_enabled && !use_mpc) {
    for (int q = 0; q < N; ++q) {
      l1_state_logs[q] = builder.AddSystem<VectorLogSink<double>>(5);
      builder.Connect(controllers[q]->get_l1_state_output_port(),
                      l1_state_logs[q]->get_input_port());
    }
  }

  // Concurrent-learning diagnostics. Vectors are sized N; the write
  // loop skips drones whose estimator pointer is null.
  std::vector<VectorLogSink<double>*> cl_theta_logs(N, nullptr);
  std::vector<VectorLogSink<double>*> cl_innov_logs(N, nullptr);
  std::vector<VectorLogSink<double>*> cl_rank_logs(N, nullptr);
  std::vector<VectorLogSink<double>*> cl_hist_logs(N, nullptr);
  if (opts.adaptive) {
    for (int q = 0; q < N; ++q) {
      if (!cl_estimators[q]) continue;
      cl_theta_logs[q] = builder.AddSystem<VectorLogSink<double>>(2);
      cl_innov_logs[q] = builder.AddSystem<VectorLogSink<double>>(1);
      cl_rank_logs[q]  = builder.AddSystem<VectorLogSink<double>>(1);
      cl_hist_logs[q]  = builder.AddSystem<VectorLogSink<double>>(1);
      builder.Connect(cl_estimators[q]->get_theta_hat_output_port(),
                      cl_theta_logs[q]->get_input_port());
      builder.Connect(cl_estimators[q]->get_innovation_output_port(),
                      cl_innov_logs[q]->get_input_port());
      builder.Connect(cl_estimators[q]->get_rank_margin_output_port(),
                      cl_rank_logs[q]->get_input_port());
      builder.Connect(cl_estimators[q]->get_history_size_output_port(),
                      cl_hist_logs[q]->get_input_port());
    }
  }

  // ============ MESHCAT VISUALIZATION ============
  auto meshcat = std::make_shared<Meshcat>();
  meshcat->Delete();
  MeshcatVisualizer<double>::AddToBuilder(&builder, scene_graph, meshcat);

  // Continuous rope-line visualizers
  std::vector<Rgba> rope_colors = {
      Rgba(0.2, 0.2, 0.8, 1.0),  // Blue
      Rgba(0.2, 0.8, 0.2, 1.0),  // Green
      Rgba(0.8, 0.8, 0.2, 1.0),  // Yellow
      Rgba(0.8, 0.2, 0.8, 1.0),  // Magenta
  };
  for (int q = 0; q < N; ++q) {
    const double angle = 2.0 * M_PI * q / N;
    const Eigen::Vector3d payload_attach(
        formation_radius * std::cos(angle) * 0.3,
        formation_radius * std::sin(angle) * 0.3, 0);
    std::vector<std::pair<const RigidBody<double>*, Eigen::Vector3d>> pts;
    pts.emplace_back(quadcopter_bodies[q], quad_attachment_offset);
    for (const auto* bead : bead_chains[q]) {
      pts.emplace_back(bead, Eigen::Vector3d::Zero());
    }
    pts.emplace_back(&payload_body, payload_attach);
    const std::string rope_path = "rope_line_" + std::to_string(q);
    const double rope_fault_t = IsFaultedAt(opts, q) ? FaultTimeFor(opts, q) : -1.0;
    // FaultAwareRopeVisualizer draws the rope exactly like RopeVisualizer
    // until fault_time; after that it zeros out the polyline and deletes
    // the path so the Meshcat replay shows no ghost rope.
    auto& rv = *builder.AddSystem<FaultAwareRopeVisualizer>(
        plant, pts, meshcat,
        rope_path, 3.0, rope_colors[q % rope_colors.size()], rope_fault_t);
    builder.Connect(plant.get_state_output_port(),
                    rv.get_plant_state_input_port());

    // Drone breadcrumb trail: hide at fault so the disconnected quad's
    // safe-hover retreat does not leave a misleading trail line.
    if (IsFaultedAt(opts, q)) {
      builder.AddSystem<MeshcatFaultHider>(
          meshcat, "trajectories/drone_" + std::to_string(q), rope_fault_t);
    }
  }

  // Trajectory visualizer: reference-path dashed green line + dynamic
  // position trails for the payload and the 4 drones.
  std::vector<drake::multibody::BodyIndex> drone_indices;
  for (const auto* b : quadcopter_bodies) drone_indices.push_back(b->index());
  tether_lift::TrajectoryVisualizer::Params tvp;
  tvp.show_reference_trajectory = true;
  tvp.reference_color = Rgba(0.0, 0.75, 0.0, 0.85);
  tvp.reference_line_width = 5.0;
  tvp.reference_sample_points = 300;
  tvp.trajectory_duration = opts.duration;
  tvp.show_trails = true;
  tvp.trail_update_period = 0.05;
  tvp.load_trail_color = Rgba(1.0, 0.55, 0.0, 0.95);  // vivid orange
  tvp.load_trail_width = 4.5;
  tvp.load_max_trail_points = 2000;
  tvp.drone_trail_colors = {
      Rgba(0.84, 0.37, 0.0, 0.80),   // drone 0 — vermilion
      Rgba(0.0, 0.62, 0.45, 0.80),   // drone 1 — bluish-green
      Rgba(0.0, 0.45, 0.70, 0.80),   // drone 2 — blue
      Rgba(0.80, 0.47, 0.65, 0.80),  // drone 3 — reddish-purple
  };
  tvp.drone_trail_width = 2.5;
  tvp.drone_max_trail_points = 1500;
  auto& traj_vis = *builder.AddSystem<tether_lift::TrajectoryVisualizer>(
      plant, meshcat, payload_body.index(), drone_indices, tvp);
  builder.Connect(plant.get_state_output_port(),
                  traj_vis.get_plant_state_input());

  // Capture the reference-trajectory lambda for drawing after build.
  auto ref_fn = [wps = waypoints](double t) -> Eigen::Vector3d {
    double seg_start = 0.0;
    for (size_t i = 0; i < wps.size(); ++i) {
      const auto& wp = wps[i];
      const double seg_end = wp.arrival_time;
      const double hold_end = seg_end + wp.hold_time;
      if (t <= seg_end) {
        if (i == 0) return wp.position;
        const auto& pv = wps[i - 1];
        const double dur = seg_end - seg_start;
        if (dur < 1e-6) return wp.position;
        const double a = (t - seg_start) / dur;
        return (1.0 - a) * pv.position + a * wp.position;
      } else if (t <= hold_end) { return wp.position; }
      seg_start = hold_end;
    }
    return wps.back().position;
  };

  // ============ BUILD + RUN ============
  auto diagram = builder.Build();
  Simulator<double> simulator(*diagram);
  auto& context = simulator.get_mutable_context();

  // Initial configuration: system starts in near-static hover equilibrium.
  //
  //   Payload:  spawned at waypoint 0's position (x_p0, y_p0, z_p0), i.e.
  //             already suspended at the start of the mission. This avoids
  //             the artefactual pickup-from-ground transient (no contact
  //             dynamics, no lateral traversal, no bead-chain snap-taut
  //             excitation of the ω_n ≈ 32 rad/s chain mode).
  //
  //   Drones:   each drone spawns at its formation slot directly:
  //                 (x_p0 + r cos φᵢ,  y_p0 + r sin φᵢ,  z_p0 + Δz_taut)
  //             with Δz_taut = √(L² − r²). This is the geometric vertical
  //             offset at which the rope is exactly rest-length — rope
  //             therefore has zero pre-stretch at t = 0 and the payload is
  //             statically suspended by the bead chain acting as a rigid
  //             rod. Quasi-static tension (m_L · g / N) will build up over
  //             the pickup-ramp window as the controller's T_ff term
  //             smoothly cosine-ramps from 0 to the nominal value.
  //
  //   Beads:    placed on the straight chord between drone attachment and
  //             payload attachment, equally spaced — the correct zero-
  //             stretch configuration for a taut rope at t = 0.
  auto& plant_context = diagram->GetMutableSubsystemContext(plant, &context);
  Eigen::VectorXd q0 = plant.GetPositions(plant_context);

  // Waypoint 0 is the initial target payload pose (arrival_time=0).
  const Eigen::Vector3d p_payload_init = waypoints.front().position;

  for (int i = 0; i < N; ++i) {
    const double angle = 2.0 * M_PI * i / N;
    // Drake layout per floating body: q = [quat(4), pos(3)]
    const int off = i * 7;
    q0(off + 0) = 1.0; q0(off + 1) = 0.0; q0(off + 2) = 0.0; q0(off + 3) = 0.0;
    q0(off + 4) = p_payload_init.x() + formation_radius * std::cos(angle);
    q0(off + 5) = p_payload_init.y() + formation_radius * std::sin(angle);
    q0(off + 6) = p_payload_init.z() + rope_drop;
  }
  // Payload position (body N, suspended at waypoint 0).
  q0(N * 7 + 0) = 1.0;
  q0(N * 7 + 4) = p_payload_init.x();
  q0(N * 7 + 5) = p_payload_init.y();
  q0(N * 7 + 6) = p_payload_init.z();

  // Interpolate rope beads on the chord from drone attachment to payload
  // attachment — zero-stretch taut configuration.
  for (int q = 0; q < N; ++q) {
    const double angle = 2.0 * M_PI * q / N;
    const Eigen::Vector3d quad_pos(
        p_payload_init.x() + formation_radius * std::cos(angle),
        p_payload_init.y() + formation_radius * std::sin(angle),
        p_payload_init.z() + rope_drop);
    // Drone attachment is below the drone body by |quad_attachment_offset|.
    const Eigen::Vector3d quad_attach = quad_pos + quad_attachment_offset;
    // Payload attachment is on the payload body, shifted by the small
    // formation-spread offset used when connecting the rope to the payload.
    const Eigen::Vector3d payload_attach_offset(
        formation_radius * std::cos(angle) * 0.3,
        formation_radius * std::sin(angle) * 0.3, 0.0);
    const Eigen::Vector3d pay_pos = p_payload_init + payload_attach_offset;
    for (int b = 0; b < num_rope_beads; ++b) {
      const double frac = (b + 1.0) / (num_rope_beads + 1.0);
      const Eigen::Vector3d bp = quad_attach + frac * (pay_pos - quad_attach);
      const int body_idx = N + 1 + q * num_rope_beads + b;
      const int off = body_idx * 7;
      q0(off + 0) = 1.0;
      q0(off + 4) = bp.x(); q0(off + 5) = bp.y(); q0(off + 6) = bp.z();
    }
  }
  plant.SetPositions(&plant_context, q0);
  plant.SetVelocities(&plant_context,
                      Eigen::VectorXd::Zero(plant.num_velocities()));
  (void)initial_altitude;  // now unused (payload spawn determines altitude)

  // Draw the payload's expected reference trajectory as a static green line.
  // `waypoints` IS the payload trajectory in this design, so no offset needed.
  traj_vis.DrawReferenceTrajectoryFromFunction(
      ref_fn, 0.0, opts.duration, 400);

  std::cout << "Running simulation (target_realtime_rate=0.5)...\n";
  simulator.set_target_realtime_rate(0.5);
  simulator.Initialize();

  meshcat->StartRecording();

  // Advance in chunks for progress reporting
  const double step = 1.0;
  double t = 0.0;
  while (t < opts.duration) {
    t = std::min(t + step, opts.duration);
    simulator.AdvanceTo(t);
    std::cout << "  t = " << t << "s / " << opts.duration << "s\n";
  }

  meshcat->StopRecording();
  meshcat->PublishRecording();

  // Save standalone HTML
  const std::string html_path =
      opts.output_dir + "/scenario_" + opts.scenario_name + ".html";
  std::ofstream html_out(html_path);
  html_out << meshcat->StaticHtml();
  html_out.close();
  std::cout << "\nReplay saved to: " << html_path << "\n";

  // ============ DUMP CSV LOGS ============
  const std::string csv_path =
      opts.output_dir + "/scenario_" + opts.scenario_name + ".csv";
  std::ofstream csv(csv_path);

  const auto& state_data = state_log->FindLog(context);
  const auto& ref_pos_data = ref_pos_log->FindLog(context);
  const auto& ref_vel_data = ref_vel_log->FindLog(context);

  const auto& times = state_data.sample_times();
  const auto& sd = state_data.data();
  const auto& rpd = ref_pos_data.data();
  const auto& rvd = ref_vel_data.data();

  // Header. Diagnostic signals are now the LOCAL controller's QP/swing
  // telemetry (swing_speed, swing_offset, qp_cost, qp_solve_time_us, T_ff,
  // thrust_cmd, tilt_mag, 6 active-set bits) — NOT peer information.
  csv << "time";
  for (int i = 0; i < N; ++i) {
    csv << ",quad" << i << "_x,quad" << i << "_y,quad" << i << "_z";
    csv << ",quad" << i << "_vx,quad" << i << "_vy,quad" << i << "_vz";
  }
  csv << ",payload_x,payload_y,payload_z,payload_vx,payload_vy,payload_vz";
  csv << ",ref_x,ref_y,ref_z,ref_vx,ref_vy,ref_vz";
  for (int i = 0; i < N; ++i) csv << ",tension_" << i;
  for (int i = 0; i < N; ++i) {
    csv << ",swing_speed_" << i << ",swing_offset_" << i << ",qp_cost_" << i
        << ",qp_solve_us_" << i << ",T_ff_" << i << ",thrust_cmd_" << i
        << ",tilt_mag_" << i
        << ",act_ax_lo_" << i << ",act_ax_hi_" << i
        << ",act_ay_lo_" << i << ",act_ay_hi_" << i
        << ",act_az_lo_" << i << ",act_az_hi_" << i;
  }
  for (int i = 0; i < N; ++i) {
    csv << ",tau_x_" << i << ",tau_y_" << i << ",tau_z_" << i;
    csv << ",fx_" << i << ",fy_" << i << ",fz_" << i;
  }
  // Per-rope per-segment tensions: T_{i,j} for drone i, segment j.
  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < num_rope_segments; ++j) {
      csv << ",seg_T_" << i << "_" << j;
    }
  }
  if (opts.adaptive) {
    for (int i = 0; i < N; ++i) {
      csv << ",cl_m_hat_" << i << ",cl_k_hat_" << i
          << ",cl_innovation_" << i << ",cl_rank_margin_" << i
          << ",cl_history_size_" << i;
    }
  }
  if (opts.l1_enabled && !use_mpc) {
    for (int i = 0; i < N; ++i) {
      csv << ",l1_ez_hat_" << i << ",l1_vez_hat_" << i
          << ",l1_sigma_hat_" << i << ",l1_u_ad_" << i
          << ",l1_k_eff_hat_" << i;
    }
  }
  csv << "\n";

  const int num_vel_offset = plant.num_positions();  // q first, then v in state
  // Body indexing: body 0 is world, bodies 1..N are quads, body N+1 is payload, rest are beads
  // Our AddRigidBody sequence gave: quads (0..N-1 in our vec), then payload, then beads
  // In plant.num_bodies() world is 0, so quad 0 is body 1 in Drake's indexing, etc.
  // But q[] layout uses only bodies that have floating joints (all of ours do).
  // Layout: q[i*7 + 4 : i*7 + 7] = position of i-th floating body

  // Resolve where each body sits in q/v
  auto body_q_off = [&](int custom_idx) { return custom_idx * 7; };
  auto body_v_off = [&](int custom_idx) { return num_vel_offset + custom_idx * 6; };

  const int payload_idx = N;  // 0..N-1 are quads, index N is payload

  for (int s = 0; s < sd.cols(); ++s) {
    csv << times(s);
    // Quads
    for (int i = 0; i < N; ++i) {
      const int qo = body_q_off(i) + 4;
      const int vo = body_v_off(i) + 3;
      csv << "," << sd(qo, s) << "," << sd(qo + 1, s) << "," << sd(qo + 2, s);
      csv << "," << sd(vo, s) << "," << sd(vo + 1, s) << "," << sd(vo + 2, s);
    }
    // Payload
    {
      const int qo = body_q_off(payload_idx) + 4;
      const int vo = body_v_off(payload_idx) + 3;
      csv << "," << sd(qo, s) << "," << sd(qo + 1, s) << "," << sd(qo + 2, s);
      csv << "," << sd(vo, s) << "," << sd(vo + 1, s) << "," << sd(vo + 2, s);
    }
    // Reference
    csv << "," << rpd(0, s) << "," << rpd(1, s) << "," << rpd(2, s);
    csv << "," << rvd(0, s) << "," << rvd(1, s) << "," << rvd(2, s);
    // Per-rope scalar tensions (each drone sees ONLY its own)
    for (int i = 0; i < N; ++i) {
      const auto& td = tension_logs[i]->FindLog(context).data();
      csv << "," << td(0, s);
    }
    // Per-drone diagnostics (13 scalars)
    for (int i = 0; i < N; ++i) {
      const auto& d = diag_logs[i]->FindLog(context).data();
      for (int k = 0; k < 13; ++k) csv << "," << d(k, s);
    }
    // Per-drone control vector
    for (int i = 0; i < N; ++i) {
      const auto& c = control_logs[i]->FindLog(context).data();
      csv << "," << c(0, s) << "," << c(1, s) << "," << c(2, s);
      csv << "," << c(3, s) << "," << c(4, s) << "," << c(5, s);
    }
    // Per-rope per-segment tensions
    for (int i = 0; i < N; ++i) {
      const auto& st = segment_tension_logs[i]->FindLog(context).data();
      for (int j = 0; j < num_rope_segments; ++j) csv << "," << st(j, s);
    }
    if (opts.adaptive) {
      for (int i = 0; i < N; ++i) {
        if (!cl_theta_logs[i]) {
          csv << ",0,0,0,0,0";
          continue;
        }
        const auto& th = cl_theta_logs[i]->FindLog(context).data();
        const auto& innov = cl_innov_logs[i]->FindLog(context).data();
        const auto& rank = cl_rank_logs[i]->FindLog(context).data();
        const auto& hist = cl_hist_logs[i]->FindLog(context).data();
        csv << "," << th(0, s) << "," << th(1, s)
            << "," << innov(0, s) << "," << rank(0, s)
            << "," << hist(0, s);
      }
    }
    if (opts.l1_enabled && !use_mpc) {
      for (int i = 0; i < N; ++i) {
        if (!l1_state_logs[i]) {
          csv << ",0,0,0,0,0";
          continue;
        }
        const auto& ls = l1_state_logs[i]->FindLog(context).data();
        csv << "," << ls(0, s) << "," << ls(1, s)
            << "," << ls(2, s) << "," << ls(3, s)
            << "," << ls(4, s);
      }
    }
    csv << "\n";
  }
  csv.close();
  std::cout << "Signal log saved to: " << csv_path
            << " (" << sd.cols() << " samples)\n";

  return 0;
}

}  // namespace quad_rope_lift

int main(int argc, char** argv) {
  return quad_rope_lift::DoMain(argc, argv);
}
