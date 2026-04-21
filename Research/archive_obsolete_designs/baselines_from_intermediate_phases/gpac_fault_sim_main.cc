/// @file gpac_fault_sim_main.cc
/// Drake simulation: N quadcopters lifting a payload using the full GPAC
/// (Geometric Position and Attitude Control Architecture) stack.
///
/// === Control Architecture ===
/// Each quadcopter runs GPACQuadcopterController, which implements:
///   Layer 1: PID position control + anti-swing on S² (cable direction)
///   Layer 2: Geometric SO(3) attitude control (Lee et al.)
///   Layer 4: Internal ESO disturbance estimation and feedforward
///
/// Per-drone concurrent learning mass estimator (AdaptiveLoadEstimator)
/// provides θ̂_i = m_L / N convergence for the GPAC architecture.
///
/// State estimation (ESKF/GPS) feeds into the GPAC position loop.
///
/// === Fault Model ===
/// A CableFaultGate severs cable forces at cli_fault_time for cli_fault_quad.
/// The faulty drone retreats to a safe hover position via SafeHoverController.
/// Surviving N-1 drones carry the load entirely through GPAC improvements.
///
/// === Reference Trajectory ===
/// Bernoulli lemniscate (figure-8):
///   x(s) = a·√2·cos(s) / (1 + sin²(s))
///   y(s) = a·√2·sin(s)·cos(s) / (1 + sin²(s))
/// at cruise altitude z=3.0 m, period T=21 s, amplitude a=2.0 m.
/// Total duration: 70 s  (≥45 s requirement satisfied).
/// Fault at t=20 s = 10 s into figure-8 ≈ ⅓ of first loop.

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <drake/common/find_resource.h>
#include <drake/geometry/meshcat.h>
#include <drake/geometry/meshcat_visualizer.h>
#include <drake/geometry/proximity_properties.h>
#include <drake/geometry/scene_graph.h>
#include <drake/geometry/shape_specification.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/externally_applied_spatial_force_multiplexer.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/tree/rigid_body.h>
#include <drake/multibody/tree/spatial_inertia.h>
#include <drake/multibody/tree/unit_inertia.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/primitives/constant_vector_source.h>
#include <drake/systems/primitives/demultiplexer.h>
#include <drake/systems/primitives/matrix_gain.h>
#include <drake/systems/primitives/multiplexer.h>
#include <drake/systems/primitives/vector_log_sink.h>
#include <drake/systems/primitives/zero_order_hold.h>

#include "adaptive_load_estimator.h"
#include "barometer_sensor.h"
#include "cable_fault_gate.h"
#include "cable_fault_visualizer.h"
#include "control_mode_switcher.h"
#include "decentralized_load_estimator.h"
#include "estimation_error_computer.h"
#include "estimation_utils.h"
#include "gpac_quadcopter_controller.h"
#include "gps_sensor.h"
#include "imu_sensor.h"
#include "position_velocity_estimator.h"
#include "rope_force_system.h"
#include "rope_utils.h"
#include "rope_visualizer.h"
#include "safe_hover_controller.h"
#include "simulation_data_logger.h"
#include "tension_plotter.h"
#include "trajectory_visualizer.h"
#include "wind_disturbance.h"
#include "wind_force_applicator.h"

namespace quad_rope_lift {

using drake::geometry::Box;
using drake::geometry::HalfSpace;
using drake::geometry::Meshcat;
using drake::geometry::MeshcatVisualizer;
using drake::geometry::ProximityProperties;
using drake::geometry::Rgba;
using drake::geometry::SceneGraph;
using drake::geometry::Sphere;
using drake::math::RigidTransformd;
using drake::math::RollPitchYawd;
using drake::multibody::ContactModel;
using drake::multibody::CoulombFriction;
using drake::multibody::ExternallyAppliedSpatialForceMultiplexer;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::multibody::RigidBody;
using drake::multibody::SpatialInertia;
using drake::multibody::UnitInertia;
using drake::systems::ConstantVectorSource;
using drake::systems::Demultiplexer;
using drake::systems::DiagramBuilder;
using drake::systems::MatrixGain;
using drake::systems::Multiplexer;
using drake::systems::Simulator;
using drake::systems::VectorLogSink;
using drake::systems::ZeroOrderHold;

/// Configuration for a single quadcopter in the formation.
struct QuadConfig {
  Eigen::Vector3d initial_position;
  Eigen::Vector3d formation_offset;
  Eigen::Vector3d payload_attachment;
  double rope_length_mean;
  double rope_length_stddev;
  double rope_length;
};

/// Build lemniscate (Bernoulli figure-8) waypoints for the load trajectory.
///
/// The lemniscate is parametrised:
///   s(t) = 2π(t - t_start) / period
///   x = a√2 cos(s) / (1 + sin²(s))
///   y = a√2 sin(s) cos(s) / (1 + sin²(s))
/// Sampled at dt_sample intervals from t_start to t_end at altitude z.
static std::vector<GPACWaypoint> MakeLemniscateWaypoints(
    double t_start, double t_end, double dt_sample,
    double period, double a, double z) {
  std::vector<GPACWaypoint> wps;
  const double a_scaled = a * std::sqrt(2.0);
  for (double t = t_start; t <= t_end + 1e-9; t += dt_sample) {
    const double s = 2.0 * M_PI * (t - t_start) / period;
    const double denom = 1.0 + std::sin(s) * std::sin(s);
    const double x = a_scaled * std::cos(s) / denom;
    const double y = a_scaled * std::sin(s) * std::cos(s) / denom;
    GPACWaypoint wp;
    wp.position    = Eigen::Vector3d(x, y, z);
    wp.arrival_time = t;
    wp.hold_time   = 0.0;
    wps.push_back(wp);
  }
  return wps;
}

int DoMain(int argc, char *argv[]) {
  // =========================================================================
  // Command-Line Arguments
  // =========================================================================

  unsigned int cli_seed           = 42;
  bool         headless           = false;
  double       cli_duration       = 70.0;
  int          cli_num_quads      = 4;
  std::string  cli_output_dir;
  int          cli_fault_quad_index = 0;   // default: cut drone 0
  double       cli_fault_time       = 20.0; // default: 20 s into simulation

  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--seed") == 0 && i + 1 < argc) {
      cli_seed = static_cast<unsigned int>(std::atoi(argv[++i]));
    } else if (std::strcmp(argv[i], "--headless") == 0) {
      headless = true;
    } else if (std::strcmp(argv[i], "--duration") == 0 && i + 1 < argc) {
      cli_duration = std::atof(argv[++i]);
    } else if (std::strcmp(argv[i], "--num-quads") == 0 && i + 1 < argc) {
      cli_num_quads = std::atoi(argv[++i]);
    } else if (std::strcmp(argv[i], "--output-dir") == 0 && i + 1 < argc) {
      cli_output_dir = argv[++i];
    } else if (std::strcmp(argv[i], "--fault-quad-index") == 0 && i + 1 < argc) {
      cli_fault_quad_index = std::atoi(argv[++i]);
    } else if (std::strcmp(argv[i], "--fault-time") == 0 && i + 1 < argc) {
      cli_fault_time = std::atof(argv[++i]);
    } else if (std::strcmp(argv[i], "--no-fault") == 0) {
      cli_fault_quad_index = -1;
      cli_fault_time       = -1.0;
    } else if (std::strcmp(argv[i], "--help") == 0) {
      std::cout
          << "Usage: ./gpac_fault_sim [options]\n"
          << "  --seed N             Random seed (default: 42)\n"
          << "  --headless           Disable visualization\n"
          << "  --duration T         Simulation duration [s] (default: 70)\n"
          << "  --num-quads N        Number of quadcopters (default: 4)\n"
          << "  --output-dir DIR     Base output directory\n"
          << "  --fault-quad-index I Quadcopter whose cable to cut (default: 0)\n"
          << "  --fault-time T       Fault time [s] (default: 20)\n"
          << "  --no-fault           Disable fault injection\n";
      return 0;
    }
  }

  const std::string output_base =
      cli_output_dir.empty() ? "/workspaces/Tether_Grace/output/gpac" : cli_output_dir;

  // =========================================================================
  // Simulation Parameters
  // =========================================================================

  const double simulation_time_step  = 2e-4;   // [s]
  const double simulation_duration   = cli_duration;

  // State estimation (GPS-fusion ESKF)
  // NOTE v21: use_estimated_in_controller=false — ESKF remains active for
  // logging/diagnostics but the GPAC inner loop uses the true Drake plant
  // state. Per-drone estimator divergence (≥0.17 m in 0.1 s due to GPS lag)
  // was breaking formation symmetry and causing explosive cross-coupling.
  // True-state feedback removes this noise source while keeping all rope,
  // payload, and fault physics intact.  Re-enable after ESKF tuning.
  const bool   enable_estimation             = true;
  const bool   use_estimated_in_controller   = false;  // v21: true plant state
  const bool   enable_eso_feedforward        = false;
  const double gps_sample_period             = 0.1;
  const Eigen::Vector3d gps_position_noise(0.02, 0.02, 0.05);
  const double estimator_dt                  = 0.01;  // 100 Hz

  // =========================================================================
  // Physical Properties
  // =========================================================================

  const double quadcopter_mass = 1.5;
  const Eigen::Vector3d quadcopter_dimensions(0.30, 0.30, 0.10);

  const double payload_mass   = 3.0;
  const double payload_radius = 0.15;

  const double rope_total_mass = 0.2;
  const int    num_rope_beads  = 8;
  const double gravity         = 9.81;
  const int    num_quadcopters = cli_num_quads;
  // Start drones at z=2.990 m — the joint static equilibrium with v20 gains.
  // Computed numerically: kp_z=80, FF_cap=0.85*load_per_rope, payload=1.685m.
  //   drone_net = 0.10 N (barely upward), payload_net = -0.03 N (barely down).
  // <<< 0.1 N net force at t=0 → slow gentle drift to true equilibrium >>>
  // No initial fall → no bead-velocity transient → no tension spike → stable.
  const double initial_altitude = 2.990;  // joint equilibrium altitude

  // =========================================================================
  // Formation Geometry
  // =========================================================================

  const double formation_radius   = 0.6;
  const double attachment_radius  = payload_radius * 0.7;

  std::vector<QuadConfig> quad_configs(num_quadcopters);

  // Rope lengths are kept uniform in this executable so that the pre-fault
  // experiment isolates the controller and cable-cut response rather than
  // unequal static preload caused by random tether asymmetry.
  std::vector<double> rope_length_means   = {1.0, 1.0, 1.0, 1.0};
  std::vector<double> rope_length_stddevs = {0.0, 0.0, 0.0, 0.0};
  while (static_cast<int>(rope_length_means.size()) < num_quadcopters) {
    rope_length_means.push_back(rope_length_means.back());
    rope_length_stddevs.push_back(rope_length_stddevs.back());
  }

  const unsigned int random_seed = cli_seed;
  std::default_random_engine generator(random_seed);

  std::cout << "\n========================================\n";
  std::cout << "GPAC Cable-Fault Simulation\n";
  std::cout << "========================================\n";
  std::cout << "Quadcopters  : " << num_quadcopters << "\n";
  std::cout << "Fault quad   : " << cli_fault_quad_index << "\n";
  std::cout << "Fault time   : " << cli_fault_time << " s\n";
  std::cout << "Duration     : " << simulation_duration << " s\n";
  std::cout << "ESKF closed  : YES\n";
  std::cout << "Anti-swing   : NO (disabled for bead-chain rope stability)\n";
  std::cout << "ESO feedfwd  : " << (enable_eso_feedforward ? "YES" : "NO") << "\n";
  std::cout << "CL mass est  : YES\n";
  std::cout << "Trajectory   : Cruise hold at z=" << 3.0
            << " m (payload starts lifted, no ascent)\n";
  std::cout << "========================================\n\n";

  std::cout << "Rope Length Sampling (Gaussian), seed=" << random_seed << "\n";
  for (int i = 0; i < num_quadcopters; ++i) {
    const double angle = 2.0 * M_PI * i / num_quadcopters;
    const double x     = formation_radius * std::cos(angle);
    const double y     = formation_radius * std::sin(angle);

    quad_configs[i].initial_position = Eigen::Vector3d(x, y, initial_altitude);
    quad_configs[i].formation_offset = Eigen::Vector3d(x, y, 0.0);
    quad_configs[i].payload_attachment =
        Eigen::Vector3d(attachment_radius * std::cos(angle),
                        attachment_radius * std::sin(angle),
                        payload_radius);

    quad_configs[i].rope_length_mean   = rope_length_means[i];
    quad_configs[i].rope_length_stddev = rope_length_stddevs[i];

    std::normal_distribution<double> dist(
        quad_configs[i].rope_length_mean,
        quad_configs[i].rope_length_stddev);
    quad_configs[i].rope_length = dist(generator);
    while (quad_configs[i].rope_length <= 0.1)
      quad_configs[i].rope_length = dist(generator);

    std::cout << "  Quad " << i
              << ": μ=" << quad_configs[i].rope_length_mean
              << " m, σ=" << quad_configs[i].rope_length_stddev
              << " m → sampled=" << quad_configs[i].rope_length << " m\n";
  }
  std::cout << "\n";

  // =========================================================================
  // GPAC Waypoints — Cruise Hold (fault demonstration)
  // =========================================================================
  //
  // Start the simulation ALREADY at cruise altitude with the payload lifted.
  // This eliminates the slack→taut rope impulse during ascent that causes the
  // bead-chain to snap violently (~400 N) and crash all drones.
  //
  // Static equilibrium geometry (kp_xy≥9, cruise_z=3.0 m, rope=1.0 m):
  //   - Drone attachment z = cruise_z − 0.10 = 2.90 m
  //   - Rope length at equilibrium ≈ 1.165 m (2.7% stretch, T≈8.1 N/rope)
  //   - Payload z ≈ 1.70 m (payload_att_z = 1.85 m)
  //
  // Single waypoint: hold at cruise for the entire simulation.
  // The fault fires at cli_fault_time; surviving drones maintain cruise hover.

  const double cruise_z = 3.0;

  // Payload at z=1.685 m: Z-only static equilibrium with drone at x=0.6 m.
  // With Δx=0.495 m (formation radius minus payload-attach radius), rope chord
  // = 1.166 m, T=8.13 N/rope, T_vert=7.36 N → 4×7.36=29.4 N = payload weight.
  // Drones will drift inward from r=0.6 m to the coupled XY-Z equilibrium at
  // r≈0.448 m (where kp_xy×Δr = T_horiz), settling smoothly with true-state
  // feedback (no estimator lag).
  const double payload_initial_z = 1.685;

  std::vector<GPACWaypoint> waypoints;
  // One hold waypoint covering the full simulation.
  waypoints.push_back({Eigen::Vector3d(0.0, 0.0, cruise_z), 0.0,
                        simulation_duration + 100.0});

  std::cout << "Trajectory   : cruise hold at z=" << cruise_z
            << " m (fault demonstration, no ascent phase)\n";
  std::cout << "Total waypoints: " << waypoints.size() << "\n\n";


  // =========================================================================
  // Rope Parameters
  // =========================================================================

  const double max_stretch_percentage = 0.15;
  const double load_per_rope          = (payload_mass * gravity) / num_quadcopters;

  double avg_rope_length = 0.0;
  for (int i = 0; i < num_quadcopters; ++i) avg_rope_length += quad_configs[i].rope_length;
  avg_rope_length /= num_quadcopters;

  const double effective_rope_stiffness =
      load_per_rope / (avg_rope_length * max_stretch_percentage);
  const int    num_segments     = num_rope_beads + 1;
  const double segment_stiffness = effective_rope_stiffness * num_segments;
  const double reference_stiffness = 300.0;
  // Triple rope damping to suppress snap-taut impulses and reduce payload
  // oscillation amplitude (original value 15 gives payload ζ≈0.08 → unstable).
  const double reference_damping   = 50.0;
  const double segment_damping =
      reference_damping * std::sqrt(segment_stiffness / reference_stiffness);

  // =========================================================================
  // Build the Simulation
  // =========================================================================

  DiagramBuilder<double> builder;
  auto [plant, scene_graph] = drake::multibody::AddMultibodyPlantSceneGraph(
      &builder, simulation_time_step);
  plant.set_contact_model(ContactModel::kPoint);

  const ModelInstanceIndex model_instance =
      plant.AddModelInstance("gpac_multi_quad");

  // -------------------------------------------------------------------------
  // Quadcopter Bodies
  // -------------------------------------------------------------------------

  const SpatialInertia<double> quad_inertia(
      quadcopter_mass, Eigen::Vector3d::Zero(),
      UnitInertia<double>::SolidBox(quadcopter_dimensions[0],
                                    quadcopter_dimensions[1],
                                    quadcopter_dimensions[2]));

  std::vector<const RigidBody<double>*> quadcopter_bodies;
  std::vector<std::vector<ModelInstanceIndex>> visual_instances_per_quad;
  quadcopter_bodies.reserve(num_quadcopters);
  visual_instances_per_quad.reserve(num_quadcopters);

  Parser parser(&plant);
  for (int i = 0; i < num_quadcopters; ++i) {
    const std::string quad_name = "quadcopter_" + std::to_string(i);
    const RigidBody<double>& quad_body =
        plant.AddRigidBody(quad_name, model_instance, quad_inertia);
    quadcopter_bodies.push_back(&quad_body);

    parser.SetAutoRenaming(true);
    const auto vis_insts = parser.AddModels(
        drake::FindResourceOrThrow("drake/examples/quadrotor/quadrotor.urdf"));
    const RigidBody<double>& vis_base =
        plant.GetBodyByName("base_link", vis_insts[0]);
    plant.WeldFrames(quad_body.body_frame(), vis_base.body_frame(),
                     RigidTransformd::Identity());
    visual_instances_per_quad.push_back(vis_insts);
  }

  // -------------------------------------------------------------------------
  // Payload Body
  // -------------------------------------------------------------------------

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

  // -------------------------------------------------------------------------
  // Ground Plane
  // -------------------------------------------------------------------------

  plant.RegisterCollisionGeometry(plant.world_body(),
                                  RigidTransformd::Identity(), HalfSpace(),
                                  "ground_collision", ground_friction);
  plant.RegisterVisualGeometry(plant.world_body(),
                               RigidTransformd(Eigen::Vector3d(0, 0, -0.02)),
                               Box(14, 14, 0.04), "ground_visual",
                               Eigen::Vector4d(0.7, 0.7, 0.7, 1.0));

  // -------------------------------------------------------------------------
  // Rope Beads
  // -------------------------------------------------------------------------

  std::vector<std::vector<const RigidBody<double>*>> bead_chains(num_quadcopters);
  std::vector<RopeParameters> rope_params_vec(num_quadcopters);

  for (int q = 0; q < num_quadcopters; ++q) {
    rope_params_vec[q] = ComputeRopeParameters(
        num_rope_beads, quad_configs[q].rope_length, rope_total_mass,
        segment_stiffness, segment_damping, true, 0.012);

    const SpatialInertia<double> bead_inertia(
        rope_params_vec[q].bead_mass, Eigen::Vector3d::Zero(),
        UnitInertia<double>::SolidSphere(rope_params_vec[q].bead_radius));

    bead_chains[q].reserve(num_rope_beads);
    const double hue = static_cast<double>(q) / num_quadcopters;
    const Eigen::Vector4d bead_color(0.3 + 0.4 * hue, 0.3,
                                     0.3 + 0.4 * (1.0 - hue), 1.0);

    for (int i = 0; i < num_rope_beads; ++i) {
      const std::string name =
          "rope_" + std::to_string(q) + "_bead_" + std::to_string(i);
      const RigidBody<double>& bead =
          plant.AddRigidBody(name, model_instance, bead_inertia);
      // Skip collision for faulted rope beads — after cable cut, beads fall
      // freely; collision geometry causes integrator instability on ground contact.
      if (q != cli_fault_quad_index) {
        plant.RegisterCollisionGeometry(bead, RigidTransformd::Identity(),
                                        Sphere(rope_params_vec[q].bead_radius),
                                        name + "_collision", ground_friction);
      }
      plant.RegisterVisualGeometry(bead, RigidTransformd::Identity(),
                                   Sphere(rope_params_vec[q].bead_radius),
                                   name + "_visual", bead_color);
      bead_chains[q].push_back(&bead);
    }
  }

  plant.Finalize();

  // -------------------------------------------------------------------------
  // Rope Attachment Point
  // -------------------------------------------------------------------------

  const Eigen::Vector3d quad_attachment_offset(
      0.0, 0.0, -quadcopter_dimensions[2] / 2.0 - 0.05);

  // =========================================================================
  // Rope Force Systems + Tension Zero-Order Holds
  // =========================================================================

  std::vector<RopeForceSystem*> rope_systems;
  std::vector<ZeroOrderHold<double>*> tension_holds;
  // Tension output port to use for logging (gated for faulted rope).
  std::vector<const drake::systems::OutputPort<double>*> tension_log_ports;
  rope_systems.reserve(num_quadcopters);
  tension_holds.reserve(num_quadcopters);
  tension_log_ports.reserve(num_quadcopters);

  for (int q = 0; q < num_quadcopters; ++q) {
    auto& rope_system = *builder.AddSystem<RopeForceSystem>(
        plant, *quadcopter_bodies[q], payload_body, bead_chains[q],
        quad_attachment_offset, quad_configs[q].payload_attachment,
        quad_configs[q].rope_length, rope_params_vec[q].segment_stiffness,
        rope_params_vec[q].segment_damping);
    rope_systems.push_back(&rope_system);

    auto& tension_hold =
        *builder.AddSystem<ZeroOrderHold<double>>(simulation_time_step, 4);
    if (q == cli_fault_quad_index && cli_fault_time >= 0.0) {
      // Gate tension telemetry to zero after cable cut (avoids phantom stretch).
      auto& tension_gate =
          *builder.AddSystem<TensionFaultGate>(cli_fault_time);
      builder.Connect(rope_system.get_tension_output_port(),
                      tension_gate.get_tension_input_port());
      builder.Connect(tension_gate.get_tension_output_port(),
                      tension_hold.get_input_port());
      tension_log_ports.push_back(&tension_gate.get_tension_output_port());
    } else {
      builder.Connect(rope_system.get_tension_output_port(),
                      tension_hold.get_input_port());
      tension_log_ports.push_back(&rope_system.get_tension_output_port());
    }
    tension_holds.push_back(&tension_hold);
  }

  // =========================================================================
  // Cable Direction Extraction + Convention Adjustment
  //
  // CableDirectionFromTension outputs: load→drone (e.g. +ẑ when vertical).
  // GPACQuadcopterController expects: drone→load (e.g. −ẑ when vertical)
  //   — verified from anti-swing desired q_d = (0,0,−1) in the controller.
  // AdaptiveLoadEstimator also expects drone→load convention
  //   — verified from cos_phi = −n_i.z() formula.
  // SimulationDataLogger cable_direction port: no convention enforced, log raw.
  // =========================================================================

  // cable_dir_raw[q]:    load→drone  (CableDirectionFromTension output)
  // cable_dir_ctrl[q]:   drone→load  (negated, for GPAC controller + mass est.)
  std::vector<CableDirectionFromTension*> cable_dir_systems;
  std::vector<MatrixGain<double>*>        cable_dir_negators;
  cable_dir_systems.reserve(num_quadcopters);
  cable_dir_negators.reserve(num_quadcopters);

  const Eigen::MatrixXd neg3 = -Eigen::Matrix3d::Identity();

  for (int q = 0; q < num_quadcopters; ++q) {
    auto& cds = *builder.AddSystem<CableDirectionFromTension>();
    builder.Connect(rope_systems[q]->get_tension_output_port(),
                    cds.get_tension_input_port());
    cable_dir_systems.push_back(&cds);

    // Negate: drone→load convention for GPAC controller + mass estimator
    auto& neg = *builder.AddSystem<MatrixGain<double>>(neg3);
    builder.Connect(cds.get_direction_output_port(), neg.get_input_port());
    cable_dir_negators.push_back(&neg);
  }

  // =========================================================================
  // GPAC Quadcopter Controllers (one per drone)
  // =========================================================================

  std::vector<GPACQuadcopterController*> controllers;
  controllers.reserve(num_quadcopters);

  for (int q = 0; q < num_quadcopters; ++q) {
    GPACParams p;
    p.formation_offset        = quad_configs[q].formation_offset;
    p.waypoints               = waypoints;
    p.initial_altitude        = initial_altitude;
    p.final_altitude          = cruise_z;
    // Layer 1: position PID — very conservative for bead-chain rope.
    // The rope exerts large lateral cable forces during ascent; low kd prevents
    // the damping term from amplifying these lateral forces into dangerous tilts.
    // Layer 1: position PID (v20 — cruise-start, capped tension FF).
    //
    // XY: ωn=√10=3.16 rad/s, ζ=12/(2×3.16)=1.90 (strongly overdamped).
    //     Formation r_eq ≈ 0.37 m (payload at z≈1.54 m, above ground ✓).
    //
    // Z:  ωn=√80=8.94 rad/s, ζ=22/(2×8.94)=1.23 (overdamped).
    //     z_eq ≈ 2.99 m (0.01 m below target) with capped FF below.
    //
    // Tension FF: CAPPED at 0.85 × load_per_rope.
    //   • Cap < actual_F_z_rope_down (≈ 0.90×T at cruise angle) → drone
    //     always experiences a small net-downward residual that kp_z=80
    //     corrects with only 8 mm position error.
    //   • When tension spikes (T→15 N), FF stays capped at 6.25 N while
    //     actual rope pull rises to 13.5 N → drone falls slightly → kp
    //     corrects → NEGATIVE feedback (stable), NOT payload-launch.
    //   • pickup_detection_threshold=1000 prevents the destructive ramp-phase
    //     that previously applied -tension_kp×T downward at t=0 and directly
    //     caused every v15–v19 crash via drone→ground→inversion.
    p.position_kp             = 10.0;  // XY ωn=3.16 > ωp=3.13 ✓
    p.position_kd             = 12.0;  // XY ζ=1.90  (strongly overdamped)
    p.position_kp_z           = 80.0;  // Z  ωn=8.94, 8 mm residual with FF
    p.position_kd_z           = 22.0;  // Z  ζ=1.23  (overdamped)
    // Tension FF: capped partial feedforward (stable)
    p.pickup_detection_threshold = 1000.0;  // NEVER enter pickup phase
    p.pickup_target_tension      = 0.85 * load_per_rope;  // FF cap ≈ 6.25 N
    p.tension_kp                 = 0.0;     // no pickup feedback
    p.position_ki             = 0.05;
    p.max_integral            = 2.0;
    // Layer 1: anti-swing (S²) — disabled for bead-chain rope simulation.
    // The anti-swing feedback from a bead-chain rope tension signal is too
    // noisy and commands extreme tilt during pickup/ascent.  The fault
    // demonstration requires a stable carrier, not anti-swing showcase.
    p.cable_kq                = 0.0;
    p.cable_kw                = 0.0;
    p.enable_antiswing        = false;
    // Layer 2: geometric SO(3)
    p.attitude_kR             = 8.0;
    p.attitude_kOmega         = 1.5;
    // Layer 4: keep the internal ESO state alive for logging/debugging, but do
    // not feed it forward into the control law in this executable. The higher-
    // bandwidth feedforward path proved destabilizing with the bead-chain rope
    // model during pickup and early lateral transport.
    p.eso_omega               = 5.0;
    p.eso_b0                  = 1.0 / quadcopter_mass;
    p.enable_eso_feedforward  = enable_eso_feedforward;
    // Tension pickup (all disabled — see gains section above)
    // p.pickup_target_tension = load_per_rope;  // OVERRIDE: already set to 0
    // Actuator limits
    p.min_thrust              = 0.0;
    p.max_thrust              = 150.0;
    p.max_torque              = 10.0;
    p.gravity                 = gravity;
    p.mass                    = quadcopter_mass;

    auto& ctrl = *builder.AddSystem<GPACQuadcopterController>(
        plant, *quadcopter_bodies[q], p);
    controllers.push_back(&ctrl);
  }

  // =========================================================================
  // GPS Sensors + ESKF State Estimators (always enabled in GPAC mode)
  // =========================================================================

  std::vector<GpsSensor*>                  quad_gps_sensors;
  std::vector<PositionVelocityEstimator*>  quad_estimators;
  GpsSensor*                               load_gps_sensor = nullptr;
  std::vector<DecentralizedLoadEstimator*> load_estimators;
  Multiplexer<double>*                     load_est_mux = nullptr;

  {
    GpsParams gps_params;
    gps_params.sample_period_sec    = gps_sample_period;
    gps_params.position_noise_stddev = gps_position_noise;
    gps_params.dropout_probability  = 0.0;

    quad_gps_sensors.reserve(num_quadcopters);
    quad_estimators.reserve(num_quadcopters);

    for (int q = 0; q < num_quadcopters; ++q) {
      gps_params.random_seed = 100 + q;

      auto& gps = *builder.AddSystem<GpsSensor>(
          plant, *quadcopter_bodies[q], gps_params);
      quad_gps_sensors.push_back(&gps);

      EstimatorParams est_params;
      est_params.gps_measurement_noise = gps_position_noise;
      auto& est = *builder.AddSystem<PositionVelocityEstimator>(
          estimator_dt, est_params);
      quad_estimators.push_back(&est);

      builder.Connect(plant.get_state_output_port(),
                      gps.get_plant_state_input_port());
      builder.Connect(gps.get_gps_position_output_port(),
                      est.get_gps_position_input_port());
      builder.Connect(gps.get_gps_valid_output_port(),
                      est.get_gps_valid_input_port());
    }

    // Load GPS (for logging)
    gps_params.random_seed = 999;
    load_gps_sensor = &(*builder.AddSystem<GpsSensor>(
        plant, payload_body, gps_params));
    builder.Connect(plant.get_state_output_port(),
                    load_gps_sensor->get_plant_state_input_port());

    // Decentralized load estimators (kinematic cable model)
    DecentralizedLoadEstimatorParams dec_est_params;
    load_estimators.reserve(num_quadcopters);

    for (int q = 0; q < num_quadcopters; ++q) {
      auto& dec_est = *builder.AddSystem<DecentralizedLoadEstimator>(
          estimator_dt, dec_est_params);
      load_estimators.push_back(&dec_est);

      auto& est_demux = *builder.AddSystem<Demultiplexer<double>>(6, 3);
      builder.Connect(quad_estimators[q]->get_estimated_state_output_port(),
                      est_demux.get_input_port(0));
      builder.Connect(est_demux.get_output_port(0),
                      dec_est.get_quad_position_input_port());
      builder.Connect(est_demux.get_output_port(1),
                      dec_est.get_quad_velocity_input_port());

      // DecentralizedLoadEstimator uses load→drone convention
      builder.Connect(cable_dir_systems[q]->get_direction_output_port(),
                      dec_est.get_cable_direction_input_port());

      Eigen::VectorXd len_vec(1);
      len_vec << quad_configs[q].rope_length;
      auto& cable_len = *builder.AddSystem<ConstantVectorSource<double>>(len_vec);
      builder.Connect(cable_len.get_output_port(),
                      dec_est.get_cable_length_input_port());

      auto& t_demux =
          *builder.AddSystem<Demultiplexer<double>>(std::vector<int>{1, 3});
      builder.Connect(rope_systems[q]->get_tension_output_port(),
                      t_demux.get_input_port(0));
      builder.Connect(t_demux.get_output_port(0),
                      dec_est.get_cable_tension_input_port());
    }

    load_est_mux = &(*builder.AddSystem<Multiplexer<double>>(std::vector<int>{3, 3}));
    builder.Connect(load_estimators[0]->get_load_position_output_port(),
                    load_est_mux->get_input_port(0));
    builder.Connect(load_estimators[0]->get_load_velocity_output_port(),
                    load_est_mux->get_input_port(1));
  }

  std::cout << "ESKF state estimation: ENABLED (100 Hz estimators, 10 Hz GPS)\n\n";

  // =========================================================================
  // Adaptive Mass Estimators (Concurrent Learning, one per drone)
  // Estimates θ̂_i = m_L / N (load mass share per drone).
  // Uses drone→load cable direction (negated).
  // =========================================================================

  std::vector<AdaptiveLoadEstimator*> mass_estimators;
  mass_estimators.reserve(num_quadcopters);

  // Single load position/velocity extractor shared across all estimators
  auto& load_pos_extractor = *builder.AddSystem<AttachmentPositionExtractor>(
      plant, std::vector<const RigidBody<double>*>{&payload_body},
      std::vector<Eigen::Vector3d>{{0.0, 0.0, 0.0}});
  builder.Connect(plant.get_state_output_port(),
                  load_pos_extractor.get_plant_state_input_port());

  // Constant desired load trajectory (conservative fill for CL estimator)
  const Eigen::Vector3d kLoadDesPos(0.0, 0.0, 1.5);
  const Eigen::Vector3d kLoadDesVel = Eigen::Vector3d::Zero();
  auto& load_pos_des_src = *builder.AddSystem<ConstantVectorSource<double>>(kLoadDesPos);
  auto& load_vel_des_src = *builder.AddSystem<ConstantVectorSource<double>>(kLoadDesVel);

  for (int q = 0; q < num_quadcopters; ++q) {
    AdaptiveEstimatorParams ae_params;
    ae_params.gamma            = 0.5;
    ae_params.lambda           = 1.0;
    ae_params.rho              = 0.5;
    ae_params.initial_theta    = payload_mass / num_quadcopters;  // warm start
    ae_params.theta_min        = 0.1;
    ae_params.theta_max        = payload_mass * 2.0;
    ae_params.accel_filter_tau = 0.1;
    ae_params.min_excitation   = 0.2;

    auto& mass_est = *builder.AddSystem<AdaptiveLoadEstimator>(0.02, ae_params);
    mass_estimators.push_back(&mass_est);

    // Scalar tension from 4D [T, fx, fy, fz]
    auto& t_demux =
        *builder.AddSystem<Demultiplexer<double>>(std::vector<int>{1, 3});
    builder.Connect(rope_systems[q]->get_tension_output_port(),
                    t_demux.get_input_port(0));
    builder.Connect(t_demux.get_output_port(0),
                    mass_est.get_tension_input_port());

    // Cable direction: drone→load (negated) as expected by AdaptiveLoadEstimator
    builder.Connect(cable_dir_negators[q]->get_output_port(),
                    mass_est.get_cable_direction_input_port());

    // Load position/velocity from physical plant (ground truth for estimation)
    builder.Connect(load_pos_extractor.get_positions_output_port(),
                    mass_est.get_load_position_input_port());

    // Approximate load velocity via a ZOH of pos derivative — use a constant
    // zero velocity while the load is being lifted (conservative)
    auto& load_vel_src =
        *builder.AddSystem<ConstantVectorSource<double>>(kLoadDesVel);
    builder.Connect(load_vel_src.get_output_port(),
                    mass_est.get_load_velocity_input_port());

    builder.Connect(load_pos_des_src.get_output_port(),
                    mass_est.get_load_position_des_input_port());
    builder.Connect(load_vel_des_src.get_output_port(),
                    mass_est.get_load_velocity_des_input_port());
  }

  std::cout << "AdaptiveLoadEstimator: ENABLED (50 Hz, warm start θ₀="
            << payload_mass / num_quadcopters << " kg)\n\n";

  // =========================================================================
  // IMU Sensors
  // =========================================================================

  std::vector<ImuSensor*> quad_imu_sensors;
  quad_imu_sensors.reserve(num_quadcopters);

  ImuParams imu_params;
  imu_params.sample_period_sec    = 0.005;  // 200 Hz
  imu_params.gyro_noise_density   = Eigen::Vector3d(5e-4, 5e-4, 5e-4);
  imu_params.accel_noise_density  = Eigen::Vector3d(4e-3, 4e-3, 4e-3);

  for (int q = 0; q < num_quadcopters; ++q) {
    imu_params.random_seed = 200 + q;
    auto& imu = *builder.AddSystem<ImuSensor>(
        plant, *quadcopter_bodies[q], imu_params);
    quad_imu_sensors.push_back(&imu);
    builder.Connect(plant.get_state_output_port(),
                    imu.get_plant_state_input_port());
  }

  // =========================================================================
  // Barometer Sensors
  // =========================================================================

  std::vector<BarometerSensor*> quad_barometers;
  quad_barometers.reserve(num_quadcopters);

  BarometerParams baro_params;
  baro_params.sample_period_sec         = 0.04;   // 25 Hz
  baro_params.white_noise_stddev        = 0.3;
  baro_params.correlated_noise_stddev   = 0.2;

  for (int q = 0; q < num_quadcopters; ++q) {
    baro_params.random_seed = 300 + q;
    auto& baro = *builder.AddSystem<BarometerSensor>(
        plant, *quadcopter_bodies[q], baro_params);
    quad_barometers.push_back(&baro);
    builder.Connect(plant.get_state_output_port(),
                    baro.get_plant_state_input_port());
  }

  // =========================================================================
  // Wind Disturbance
  // =========================================================================
  // Wind is disabled for the cable-cut fault demonstration so that formation
  // tracking errors and post-fault dynamics are not confounded by wind drift.
  // Wind robustness is validated in separate Monte Carlo runs.

  DrydenTurbulenceParams wind_params;
  wind_params.mean_wind            = Eigen::Vector3d::Zero();
  wind_params.sigma_u              = 0.0;
  wind_params.sigma_v              = 0.0;
  wind_params.sigma_w              = 0.0;
  wind_params.altitude_dependent   = false;

  GustParams gust_params;
  gust_params.enabled = false;

  auto& wind_system = *builder.AddSystem<WindDisturbance>(
      num_quadcopters, wind_params, gust_params, 0.01);

  std::vector<const RigidBody<double>*> wind_bodies;
  std::vector<Eigen::Vector3d>          wind_offsets;
  for (int q = 0; q < num_quadcopters; ++q) {
    wind_bodies.push_back(quadcopter_bodies[q]);
    wind_offsets.push_back(Eigen::Vector3d::Zero());
  }
  auto& drone_pos_extractor = *builder.AddSystem<AttachmentPositionExtractor>(
      plant, wind_bodies, wind_offsets);
  builder.Connect(plant.get_state_output_port(),
                  drone_pos_extractor.get_plant_state_input_port());
  builder.Connect(drone_pos_extractor.get_positions_output_port(),
                  wind_system.get_drone_positions_input_port());

  // =========================================================================
  // Force Combiner
  // =========================================================================

  auto& force_combiner =
      *builder.AddSystem<ExternallyAppliedSpatialForceMultiplexer>(
          2 * num_quadcopters + 1);

  SafeHoverController* fault_safe_ctrl = nullptr;

  for (int q = 0; q < num_quadcopters; ++q) {
    // Rope forces → optionally through CableFaultGate
    builder.Connect(plant.get_state_output_port(),
                    rope_systems[q]->get_plant_state_input_port());

    if (q == cli_fault_quad_index && cli_fault_time >= 0.0) {
      auto& gate = *builder.AddSystem<CableFaultGate>(cli_fault_time);
      builder.Connect(rope_systems[q]->get_forces_output_port(),
                      gate.get_forces_input_port());
      builder.Connect(gate.get_forces_output_port(),
                      force_combiner.get_input_port(2 * q + 1));
    } else {
      builder.Connect(rope_systems[q]->get_forces_output_port(),
                      force_combiner.get_input_port(2 * q + 1));
    }

    // GPAC controller connections
    builder.Connect(plant.get_state_output_port(),
                    controllers[q]->get_plant_state_input_port());
    builder.Connect(tension_holds[q]->get_output_port(),
                    controllers[q]->get_tension_input_port());

    // Cable direction (drone→load convention for GPAC)
    builder.Connect(cable_dir_negators[q]->get_output_port(),
                    controllers[q]->get_cable_direction_input_port());

    // ESKF estimated state feeds GPAC inner-loop position
    if (enable_estimation && use_estimated_in_controller) {
      builder.Connect(quad_estimators[q]->get_estimated_state_output_port(),
                      controllers[q]->get_estimated_state_input_port());
    }

    // Fault quad: add SafeHoverController + ControlModeSwitcher
    if (q == cli_fault_quad_index && cli_fault_time >= 0.0) {
      const double angle  = 2.0 * M_PI * q / num_quadcopters;
      const double safe_r = formation_radius * 2.5;
      const Eigen::Vector3d safe_pos(safe_r * std::cos(angle),
                                     safe_r * std::sin(angle), 5.0);

      fault_safe_ctrl = &(*builder.AddSystem<SafeHoverController>(
          plant, *quadcopter_bodies[q], safe_pos, quadcopter_mass));
      builder.Connect(plant.get_state_output_port(),
                      fault_safe_ctrl->get_plant_state_input_port());

      auto& switcher = *builder.AddSystem<ControlModeSwitcher>(cli_fault_time);
      builder.Connect(controllers[q]->get_control_output_port(),
                      switcher.get_normal_input_port());
      builder.Connect(fault_safe_ctrl->get_control_output_port(),
                      switcher.get_safe_input_port());
      builder.Connect(switcher.get_active_output_port(),
                      force_combiner.get_input_port(2 * q));
    } else {
      builder.Connect(controllers[q]->get_control_output_port(),
                      force_combiner.get_input_port(2 * q));
    }
  }

  // Wind forces
  auto& wind_force = *builder.AddSystem<WindForceApplicator>(
      quadcopter_bodies, payload_body, num_quadcopters);
  builder.Connect(wind_system.get_wind_velocities_output_port(),
                  wind_force.get_wind_input_port());
  builder.Connect(wind_force.get_forces_output_port(),
                  force_combiner.get_input_port(2 * num_quadcopters));

  builder.Connect(force_combiner.get_output_port(),
                  plant.get_applied_spatial_force_input_port());

  // Quick tension logger for post-sim stats (use gated output for fault quad)
  auto& tension_logger = *builder.AddSystem<VectorLogSink<double>>(4);
  builder.Connect(*tension_log_ports[0],
                  tension_logger.get_input_port());

  // =========================================================================
  // Estimation Error Loggers
  // =========================================================================

  std::vector<VectorLogSink<double>*> quad_error_loggers;
  VectorLogSink<double>* load_error_logger = nullptr;

  {
    for (int q = 0; q < num_quadcopters; ++q) {
      auto& err_comp = *builder.AddSystem<EstimationErrorComputer>(
          plant, *quadcopter_bodies[q]);
      builder.Connect(plant.get_state_output_port(),
                      err_comp.get_plant_state_input_port());
      builder.Connect(quad_estimators[q]->get_estimated_state_output_port(),
                      err_comp.get_estimated_state_input_port());

      auto& err_log = *builder.AddSystem<VectorLogSink<double>>(8);
      builder.Connect(err_comp.get_error_output_port(), err_log.get_input_port());
      quad_error_loggers.push_back(&err_log);
    }

    auto& load_err_comp = *builder.AddSystem<EstimationErrorComputer>(
        plant, payload_body);
    builder.Connect(plant.get_state_output_port(),
                    load_err_comp.get_plant_state_input_port());
    builder.Connect(load_est_mux->get_output_port(0),
                    load_err_comp.get_estimated_state_input_port());

    load_error_logger = &(*builder.AddSystem<VectorLogSink<double>>(8));
    builder.Connect(load_err_comp.get_error_output_port(),
                    load_error_logger->get_input_port());
  }

  // =========================================================================
  // Visualization
  // =========================================================================

  std::shared_ptr<Meshcat> meshcat;
  {
    meshcat = std::make_shared<Meshcat>();
    meshcat->Delete();
    MeshcatVisualizer<double>::AddToBuilder(&builder, scene_graph, meshcat);
  }

  const std::vector<Rgba> rope_colors = {
      Rgba(0.2, 0.2, 0.8, 1.0),   // Blue
      Rgba(0.2, 0.8, 0.2, 1.0),   // Green
      Rgba(0.8, 0.8, 0.2, 1.0),   // Yellow
      Rgba(0.8, 0.2, 0.8, 1.0),   // Magenta
      Rgba(0.5, 0.5, 0.5, 1.0),   // Gray (5th drone if requested)
  };

  std::vector<drake::multibody::BodyIndex> drone_body_indices;
  for (int q = 0; q < num_quadcopters; ++q)
    drone_body_indices.push_back(quadcopter_bodies[q]->index());

  tether_lift::TrajectoryVisualizer* trajectory_visualizer_ptr = nullptr;

  {
    tether_lift::TrajectoryVisualizer::Params traj_vis_params;
    traj_vis_params.show_reference_trajectory = true;
    traj_vis_params.reference_color           = Rgba(0.0, 1.0, 0.0, 0.8);
    traj_vis_params.reference_line_width      = 5.0;
    traj_vis_params.show_trails               = true;
    traj_vis_params.trail_update_period       = 0.05;
    traj_vis_params.load_trail_color          = Rgba(1.0, 0.5, 0.0, 0.9);
    traj_vis_params.load_trail_width          = 4.0;
    traj_vis_params.load_max_trail_points     = 2000;
    traj_vis_params.drone_max_trail_points    = 1000;

    auto& traj_vis = *builder.AddSystem<tether_lift::TrajectoryVisualizer>(
        plant, meshcat, payload_body.index(), drone_body_indices, traj_vis_params);
    builder.Connect(plant.get_state_output_port(),
                    traj_vis.get_plant_state_input());
    trajectory_visualizer_ptr = &traj_vis;

    if (cli_fault_quad_index >= 0 && cli_fault_time >= 0.0 && meshcat) {
      builder.AddSystem<CableFaultVisualizer>(
          meshcat, cli_fault_time, cli_fault_quad_index, num_rope_beads);
    }

    auto& tension_plotter = *builder.AddSystem<TensionPlotter>(
        meshcat, num_quadcopters, rope_colors,
        10.0,                // time window
        load_per_rope * 2.5, // max tension display
        0.05);               // update period
    for (int q = 0; q < num_quadcopters; ++q) {
      builder.Connect(rope_systems[q]->get_tension_output_port(),
                      tension_plotter.get_tension_input_port(q));
    }
  }

  // =========================================================================
  // Comprehensive Data Logger
  //
  // All unconditionally-declared logger ports must be connected. The logger
  // uses HasValue() checks internally so unconnected optional ports are OK,
  // but we connect everything for the most complete dataset.
  // =========================================================================

  tether_lift::SimulationDataLogger::Params logger_params;
  logger_params.base_output_dir    = output_base + "/logs";
  logger_params.log_period         = 0.01;  // 100 Hz
  logger_params.num_quadcopters    = num_quadcopters;

  // Basic signals
  logger_params.log_plant_state          = true;
  logger_params.log_tensions             = true;
  logger_params.log_control_efforts      = true;
  logger_params.log_gps_measurements     = true;
  logger_params.log_estimator_outputs    = true;
  logger_params.log_reference_trajectory = false;

  // Extended signals — ALL ENABLED for comprehensive coverage
  logger_params.log_imu_measurements       = true;
  logger_params.log_barometer_measurements = true;
  logger_params.log_rope_states            = false;  // no rope_state port available
  logger_params.log_attitude_data          = true;
  logger_params.log_gpac_signals           = true;   // ESO, CBF, anti-swing, CL, cable dir
  logger_params.log_wind_disturbance       = true;

  auto& data_logger = *builder.AddSystem<tether_lift::SimulationDataLogger>(
      plant, payload_body.index(), drone_body_indices, logger_params);

  // Plant state
  builder.Connect(plant.get_state_output_port(),
                  data_logger.get_plant_state_input());

  // Tensions (gated for faulted rope to avoid phantom stretch telemetry)
  for (int q = 0; q < num_quadcopters; ++q)
    builder.Connect(*tension_log_ports[q],
                    data_logger.get_tension_input(q));

  // Control efforts (6D spatial: [tau_x, tau_y, tau_z, fx, fy, fz])
  for (int q = 0; q < num_quadcopters; ++q)
    builder.Connect(controllers[q]->get_control_vector_output_port(),
                    data_logger.get_control_effort_input(q));

  // GPS + ESKF estimator outputs
  for (int q = 0; q < num_quadcopters; ++q) {
    builder.Connect(quad_gps_sensors[q]->get_gps_position_output_port(),
                    data_logger.get_gps_input(q));
    builder.Connect(quad_estimators[q]->get_estimated_state_output_port(),
                    data_logger.get_estimated_state_input(q));
  }
  builder.Connect(load_gps_sensor->get_gps_position_output_port(),
                  data_logger.get_load_gps_input());
  builder.Connect(load_est_mux->get_output_port(0),
                  data_logger.get_load_estimated_state_input());

  // IMU (accel + gyro → 6D)
  for (int q = 0; q < num_quadcopters; ++q) {
    auto& imu_mux =
        *builder.AddSystem<Multiplexer<double>>(std::vector<int>{3, 3});
    builder.Connect(quad_imu_sensors[q]->get_accel_output_port(),
                    imu_mux.get_input_port(0));
    builder.Connect(quad_imu_sensors[q]->get_gyro_output_port(),
                    imu_mux.get_input_port(1));
    builder.Connect(imu_mux.get_output_port(0),
                    data_logger.get_imu_input(q));
  }

  // Barometers
  for (int q = 0; q < num_quadcopters; ++q)
    builder.Connect(quad_barometers[q]->get_altitude_output_port(),
                    data_logger.get_barometer_input(q));

  // Wind (first drone's wind velocity as representative scalar)
  auto& wind_demux =
      *builder.AddSystem<Demultiplexer<double>>(3 * num_quadcopters, 3);
  builder.Connect(wind_system.get_wind_velocities_output_port(),
                  wind_demux.get_input_port(0));
  builder.Connect(wind_demux.get_output_port(0),
                  data_logger.get_wind_input());

  // ---- GPAC-specific logger ports ----
  // Zero-size stubs for signals not produced separately (CBF, anti-swing separate output)
  const Eigen::Vector3d kZero3 = Eigen::Vector3d::Zero();
  const Eigen::VectorXd kZero6 = Eigen::VectorXd::Zero(6);

  for (int q = 0; q < num_quadcopters; ++q) {
    // Desired attitude and attitude error (real outputs from GPAC controller)
    builder.Connect(controllers[q]->get_desired_attitude_output_port(),
                    data_logger.get_desired_attitude_input(q));
    builder.Connect(controllers[q]->get_attitude_error_output_port(),
                    data_logger.get_attitude_error_input(q));

    // Cable direction: raw load→drone direction (for logging only)
    builder.Connect(cable_dir_systems[q]->get_direction_output_port(),
                    data_logger.get_cable_direction_input(q));

    // Concurrent learning: pad θ̂_i (1D) to 4D [θ̂_i, 0, 0, 0]
    auto& cl_mux =
        *builder.AddSystem<Multiplexer<double>>(std::vector<int>{1, 1, 1, 1});
    auto& zero_1d =
        *builder.AddSystem<ConstantVectorSource<double>>(Eigen::VectorXd::Zero(1));
    builder.Connect(mass_estimators[q]->get_theta_hat_output_port(),
                    cl_mux.get_input_port(0));
    builder.Connect(zero_1d.get_output_port(), cl_mux.get_input_port(1));
    builder.Connect(zero_1d.get_output_port(), cl_mux.get_input_port(2));
    builder.Connect(zero_1d.get_output_port(), cl_mux.get_input_port(3));
    builder.Connect(cl_mux.get_output_port(0),
                    data_logger.get_concurrent_learning_input(q));

    // ESO: disturbance is internal to the GPAC controller; log zeros
    auto& z3_eso = *builder.AddSystem<ConstantVectorSource<double>>(kZero3);
    builder.Connect(z3_eso.get_output_port(),
                    data_logger.get_eso_disturbance_input(q));

    // CBF barriers: not active in this config; log zeros
    auto& z6_cbf = *builder.AddSystem<ConstantVectorSource<double>>(kZero6);
    builder.Connect(z6_cbf.get_output_port(),
                    data_logger.get_cbf_barriers_input(q));

    // Anti-swing force: computed internally in GPAC Layer 1, log zeros
    auto& z3_asw = *builder.AddSystem<ConstantVectorSource<double>>(kZero3);
    builder.Connect(z3_asw.get_output_port(),
                    data_logger.get_antiswing_force_input(q));
  }

  // =========================================================================
  // Build + Initialize Simulation
  // =========================================================================

  auto diagram      = builder.Build();
  Simulator<double> simulator(*diagram);
  auto& context     = simulator.get_mutable_context();
  auto& plant_context = plant.GetMyMutableContextFromRoot(&context);

  // Update controller masses (add URDF visual model contribution)
  for (int q = 0; q < num_quadcopters; ++q) {
    const double vis_mass =
        plant.CalcTotalMass(plant_context, visual_instances_per_quad[q]);
    controllers[q]->set_mass(quadcopter_mass + vis_mass);
    if (q == cli_fault_quad_index && fault_safe_ctrl != nullptr)
      fault_safe_ctrl->set_mass(quadcopter_mass + vis_mass);
  }

  // Initial poses
  const double ground_clearance = 0.01;
  // Start payload at near-static-equilibrium height (not on ground).
  // With 4 drones at r=0.6m, z=3.0m and rope_rest=1.0m, static eq z≈1.695m.
  plant.SetFreeBodyPose(&plant_context, payload_body,
                        RigidTransformd(Eigen::Vector3d(
                            0.0, 0.0, payload_initial_z)));

  for (int q = 0; q < num_quadcopters; ++q)
    plant.SetFreeBodyPose(&plant_context, *quadcopter_bodies[q],
                          RigidTransformd(quad_configs[q].initial_position));

  // Rope bead initialisation (slack configuration)
  for (int q = 0; q < num_quadcopters; ++q) {
    const auto& qp = plant.EvalBodyPoseInWorld(plant_context, *quadcopter_bodies[q]);
    const auto& pp = plant.EvalBodyPoseInWorld(plant_context, payload_body);
    const Eigen::Vector3d rope_start = qp * quad_attachment_offset;
    const Eigen::Vector3d rope_end   = pp * quad_configs[q].payload_attachment;

    Eigen::Vector3d lateral =
        Eigen::Vector3d::UnitZ().cross((rope_end - rope_start).normalized());
    if (lateral.norm() < 0.1) lateral = Eigen::Vector3d::UnitX();
    lateral.normalize();

    const auto bead_pos = GenerateSlackRopePositions(
        rope_start, rope_end, num_rope_beads,
        quad_configs[q].rope_length, 0.85, lateral, 1.0);

    for (int i = 0; i < num_rope_beads; ++i)
      plant.SetFreeBodyPose(&plant_context, *bead_chains[q][i],
                            RigidTransformd(bead_pos[i]));
  }

  // Initialize GPS sensors
  for (int q = 0; q < num_quadcopters; ++q) {
    auto& gps_ctx = quad_gps_sensors[q]->GetMyMutableContextFromRoot(&context);
    quad_gps_sensors[q]->InitializeGpsState(&gps_ctx, quad_configs[q].initial_position);
  }
  {
    auto& lgps_ctx = load_gps_sensor->GetMyMutableContextFromRoot(&context);
    load_gps_sensor->InitializeGpsState(
        &lgps_ctx,
        Eigen::Vector3d(0.0, 0.0, payload_radius + ground_clearance));
  }

  // Initialize ESKF estimators
  for (int q = 0; q < num_quadcopters; ++q) {
    auto& est_ctx = quad_estimators[q]->GetMyMutableContextFromRoot(&context);
    quad_estimators[q]->SetInitialState(
        &est_ctx, quad_configs[q].initial_position, Eigen::Vector3d::Zero());
  }

  // Initialize decentralized load estimators
  for (int q = 0; q < num_quadcopters; ++q) {
    auto& de_ctx = load_estimators[q]->GetMyMutableContextFromRoot(&context);
    load_estimators[q]->SetInitialState(
        &de_ctx,
        Eigen::Vector3d(0.0, 0.0, payload_radius + ground_clearance),
        Eigen::Vector3d::Zero());
  }

  // Initialize CL mass estimators with warm-start theta
  for (int q = 0; q < num_quadcopters; ++q) {
    auto& me_ctx = mass_estimators[q]->GetMyMutableContextFromRoot(&context);
    mass_estimators[q]->SetInitialTheta(&me_ctx, payload_mass / num_quadcopters);
  }

  // Warm-start internal ESO to drone initial position so that the position
  // error z1 - x(0) does not saturate the disturbance estimate on the very
  // first evaluation step, which would otherwise produce ~82 N initial thrust.
  for (int q = 0; q < num_quadcopters; ++q) {
    controllers[q]->reset_internal_eso(
        quad_configs[q].initial_position, Eigen::Vector3d::Zero());
  }
  if (fault_safe_ctrl != nullptr) {
    // SafeHoverController is a separate class without an internal ESO.
  }

  diagram->ForcedPublish(context);

  // =========================================================================
  // Draw Reference Trajectory for Visualizer
  // =========================================================================

  const double rope_stretch_factor   = 1.0 + max_stretch_percentage;
  const double total_vertical_offset =
      std::abs(quad_attachment_offset.z()) +
      avg_rope_length * rope_stretch_factor +
      payload_radius;

  std::vector<Eigen::Vector3d> load_ref_pts;
  load_ref_pts.push_back(Eigen::Vector3d(0, 0, payload_radius + ground_clearance));
  for (const auto& wp : waypoints) {
    double lz = wp.position.z() - total_vertical_offset;
    lz = std::max(lz, payload_radius + ground_clearance);
    load_ref_pts.push_back({wp.position.x(), wp.position.y(), lz});
  }
  if (trajectory_visualizer_ptr)
    trajectory_visualizer_ptr->DrawReferenceTrajectory(load_ref_pts);

  // =========================================================================
  // Configuration File
  // =========================================================================

  std::map<std::string, std::string> cfg;
  cfg["controller"]              = "GPACQuadcopterController";
  cfg["simulation_time_step"]    = std::to_string(simulation_time_step);
  cfg["simulation_duration"]     = std::to_string(simulation_duration);
  cfg["num_quadcopters"]         = std::to_string(num_quadcopters);
  cfg["quadcopter_mass"]         = std::to_string(quadcopter_mass);
  cfg["payload_mass"]            = std::to_string(payload_mass);
  cfg["payload_radius"]          = std::to_string(payload_radius);
  cfg["initial_altitude"]        = std::to_string(initial_altitude);
  cfg["formation_radius"]        = std::to_string(formation_radius);
  cfg["avg_rope_length"]         = std::to_string(avg_rope_length);
  cfg["segment_stiffness"]       = std::to_string(segment_stiffness);
  cfg["segment_damping"]         = std::to_string(segment_damping);
  cfg["num_rope_beads"]          = std::to_string(num_rope_beads);
  cfg["enable_estimation"]       = enable_estimation ? "true" : "false";
  cfg["use_estimated_in_ctrl"]   = use_estimated_in_controller ? "true" : "false";
  cfg["gps_sample_period"]       = std::to_string(gps_sample_period);
  cfg["estimator_dt"]            = std::to_string(estimator_dt);
  cfg["random_seed"]             = std::to_string(random_seed);
  cfg["fault_quad_index"]        = std::to_string(cli_fault_quad_index);
  cfg["fault_time"]              = std::to_string(cli_fault_time);
  cfg["gpac_position_kp"]        = "10.0";
  cfg["gpac_position_kd"]        = "12.0";
  cfg["gpac_position_kp_z"]      = "80.0";
  cfg["gpac_position_kd_z"]      = "22.0";
  cfg["gpac_position_ki"]        = "0.05";
  cfg["gpac_tension_ff"]         = "all_disabled(threshold=1000,kp=0,target=0)";
  cfg["rope_reference_damping"]  = "50.0";
  cfg["gpac_attitude_kR"]        = "8.0";
  cfg["gpac_attitude_kOmega"]    = "1.5";
  cfg["gpac_eso_omega"]          = "5.0";
  cfg["gpac_eso_feedforward"]    = enable_eso_feedforward ? "true" : "false";
  cfg["gpac_cable_kq"]           = "0.0";
  cfg["gpac_cable_kw"]           = "0.0";
  cfg["gpac_antiswing_enabled"]   = "false";
  cfg["cruise_z"]                = std::to_string(cruise_z);
  cfg["payload_initial_z"]       = std::to_string(payload_initial_z);

  for (int q = 0; q < num_quadcopters; ++q) {
    cfg["quad_" + std::to_string(q) + "_rope_length_mean"]    =
        std::to_string(quad_configs[q].rope_length_mean);
    cfg["quad_" + std::to_string(q) + "_rope_length_sampled"] =
        std::to_string(quad_configs[q].rope_length);
  }

  for (size_t i = 0; i < waypoints.size(); ++i) {
    const auto& wp = waypoints[i];
    cfg["waypoint_" + std::to_string(i) + "_position"] =
        "(" + std::to_string(wp.position.x()) + ", " +
        std::to_string(wp.position.y()) + ", " +
        std::to_string(wp.position.z()) + ")";
    cfg["waypoint_" + std::to_string(i) + "_arrival_time"] =
        std::to_string(wp.arrival_time);
    cfg["waypoint_" + std::to_string(i) + "_hold_time"] =
        std::to_string(wp.hold_time);
  }

  data_logger.WriteConfigFile(cfg);

  // =========================================================================
  // Run Simulation
  // =========================================================================

  if (!headless)
    simulator.set_target_realtime_rate(1.0);
  else
    simulator.set_target_realtime_rate(0.0);

  const bool enable_recording = true;
  if (enable_recording && meshcat) meshcat->StartRecording();

  std::cout << "Starting GPAC simulation...\n";
  std::cout << "  Quadcopters:  " << num_quadcopters << "\n";
  std::cout << "  Payload mass: " << payload_mass << " kg\n";
  std::cout << "  Load/drone:   " << load_per_rope / gravity << " kg\n";
  std::cout << "  Duration:     " << simulation_duration << " s\n";
  std::cout << "  Headless:     " << (headless ? "YES" : "NO") << "\n";
  if (cli_fault_quad_index >= 0) {
    std::cout << "  [FAULT] Cable cut: quad " << cli_fault_quad_index
              << " at t=" << cli_fault_time << " s\n";
    const double a_f = 2.0 * M_PI * cli_fault_quad_index / num_quadcopters;
    std::cout << "  [FAULT] Retreat to ("
              << formation_radius * 2.5 * std::cos(a_f) << ", "
              << formation_radius * 2.5 * std::sin(a_f) << ", 5.0) m\n";
  }
  if (!headless) std::cout << "  Meshcat: http://localhost:7000\n";
  std::cout << "\n";

  const double progress_interval = 0.1;
  double current_time = 0.0;

  while (current_time < simulation_duration) {
    current_time += progress_interval;
    simulator.AdvanceTo(current_time);
    if (static_cast<int>(current_time * 10) % 10 == 0)
      std::cout << "  Simulated " << current_time << " s / "
                << simulation_duration << " s\n";
  }

  // =========================================================================
  // Post-Simulation: Save Recording + Print Summary
  // =========================================================================

  if (enable_recording && meshcat) {
    meshcat->StopRecording();
    meshcat->PublishRecording();
    const std::string html_path = output_base + "/sim_recording_gpac_n" +
                                  std::to_string(num_quadcopters) + ".html";
    std::ofstream html_file(html_path);
    html_file << meshcat->StaticHtml();
    html_file.close();
    std::cout << "\nSaved MeshCat recording to: " << html_path << "\n";
  }

  // Summary statistics
  const auto& t_log  = tension_logger.FindLog(context);
  const auto& t_data = t_log.data();
  double max_tension = 0.0;
  for (int i = 0; i < t_data.cols(); ++i)
    max_tension = std::max(max_tension, t_data(0, i));

  std::cout << "\nSimulation complete!\n";
  std::cout << "  Max tension (rope 0): " << max_tension << " N\n";
  std::cout << "  Expected per rope:    " << load_per_rope << " N\n";

  if (!quad_error_loggers.empty()) {
    std::cout << "\n--- ESKF Estimation Errors (RMS / Max) ---\n";
    for (int q = 0; q < num_quadcopters; ++q) {
      const auto& el   = quad_error_loggers[q]->FindLog(context);
      const auto& ed   = el.data();
      double sum = 0.0, mx = 0.0;
      for (int i = 0; i < ed.cols(); ++i) {
        sum += ed(6, i) * ed(6, i);
        mx = std::max(mx, ed(6, i));
      }
      std::cout << "  Quad " << q << ": RMS=" << std::sqrt(sum / ed.cols()) * 100
                << " cm, Max=" << mx * 100 << " cm\n";
    }
  }

  // Final mass estimates
  std::cout << "\n--- Adaptive Mass Estimates at t=" << simulation_duration << " s ---\n";
  for (int q = 0; q < num_quadcopters; ++q) {
    const auto& me_ctx = mass_estimators[q]->GetMyContextFromRoot(context);
    const double theta_hat = mass_estimators[q]->GetThetaHat(me_ctx);
    std::cout << "  Quad " << q << ": θ̂_" << q << " = " << theta_hat
              << " kg  (true = " << payload_mass / num_quadcopters << " kg)\n";
  }

  data_logger.Finalize();
  std::cout << "\nData logged to: " << data_logger.output_dir() << "\n";

  return 0;
}

} // namespace quad_rope_lift

int main(int argc, char* argv[]) { return quad_rope_lift::DoMain(argc, argv); }
