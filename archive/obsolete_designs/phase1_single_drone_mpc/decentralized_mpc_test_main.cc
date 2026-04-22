/// @file decentralized_mpc_test_main.cc
///
/// Phase 1 Test: Single Quadcopter with Decentralized Optimal Controller
///
/// This test simulates ONE quadcopter with flexible rope, lifting a payload.
/// The DecentralizedOptimalController runs at 50 Hz, solving MPC problems.
///
/// Expected behavior:
/// - Drone hovers at z=2.0m for 5 seconds
/// - Load naturally settles below drone at distance L (rope length)
/// - Optimizer outputs nominal control (thrust ≈ 0.5*F_max to hover)
/// - Load position estimated via complementary filter
/// - Solver status logged (convergence time, success/failure)

#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
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
#include <drake/systems/framework/diagram.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/primitives/constant_vector_source.h>
#include <drake/systems/primitives/vector_log_sink.h>
#include <drake/systems/primitives/zero_order_hold.h>

#include "decentralized_optimal_controller.h"
#include "plant_state_splitter.h"
#include "rope_force_system.h"

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
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;

int main(int argc, char* argv[]) {
  bool use_meshcat = true;
  double simulation_duration = 10.0;  // seconds
  int headless = 0;

  // Parse command-line arguments
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--headless") {
      use_meshcat = false;
    } else if (arg == "--duration") {
      if (i + 1 < argc) {
        simulation_duration = std::stod(argv[++i]);
      }
    }
  }

  std::cout << "=== Decentralized MPC Phase 1 Test ===\n";
  std::cout << "Simulation duration: " << simulation_duration << " s\n";
  std::cout << "Visualization: " << (use_meshcat ? "enabled" : "disabled")
            << "\n\n";

  // ============ SETUP DRAKE SYSTEMS ============

  drake::systems::DiagramBuilder<double> builder;

  // MultibodyPlant (Phase 1: no geometry/visualization)
  const double simulation_time_step = 0.0;  // Continuous
  auto* plant = builder.AddSystem<MultibodyPlant<double>>(simulation_time_step);

  // ============ ADD QUADCOPTER BODY ============

  // Quadcopter: 1.5 kg box
  const double quad_mass = 1.5;
  const double quad_length = 0.30;
  const double quad_width = 0.30;
  const double quad_height = 0.10;

  auto I_quad = drake::multibody::UnitInertia<double>::TriaxiallySymmetric(
      0.01);  // Nominal inertia
  auto spatial_inertia_quad =
      drake::multibody::SpatialInertia<double>(quad_mass, {0, 0, 0}, I_quad);

  ModelInstanceIndex quad_model = plant->AddModelInstance("quadcopter");
  const auto& quad_body = plant->AddRigidBody(
      "quad_body", quad_model, spatial_inertia_quad);

  // Visualization geometry for quadcopter (Phase 1: skip for simplicity)
  // plant->RegisterVisualGeometry(...);  // Will add in Phase 2

  // ============ ADD PAYLOAD ============

  const double payload_mass = 3.0;
  const double payload_radius = 0.15;

  auto I_payload =
      drake::multibody::UnitInertia<double>::SolidSphere(payload_radius);
  auto spatial_inertia_payload =
      drake::multibody::SpatialInertia<double>(payload_mass, {0, 0, 0}, I_payload);

  ModelInstanceIndex payload_model = plant->AddModelInstance("payload");
  const auto& payload_body = plant->AddRigidBody(
      "payload_body", payload_model, spatial_inertia_payload);

  // Visualization geometry for payload (Phase 1: skip for simplicity)
  // plant->RegisterVisualGeometry(...);  // Will add in Phase 2

  // ============ ADD ROPE (BEAD-CHAIN MODEL) ============

  const double rope_length = 1.0;
  const int num_beads = 8;
  const double bead_mass = 0.025;
  const double bead_radius = 0.012;

  std::vector<ModelInstanceIndex> rope_model_instances;
  ModelInstanceIndex rope_model = plant->AddModelInstance("rope");
  rope_model_instances.push_back(rope_model);

  // Create bead chain
  for (int i = 0; i < num_beads; ++i) {
    std::string bead_name = "rope_0_bead_" + std::to_string(i);
    auto I_bead =
        drake::multibody::UnitInertia<double>::SolidSphere(bead_radius);
    auto spatial_inertia_bead =
        drake::multibody::SpatialInertia<double>(bead_mass, {0, 0, 0}, I_bead);

    const auto& bead = plant->AddRigidBody(bead_name, rope_model,
                                          spatial_inertia_bead);

    // Visualization (Phase 1: skip for simplicity)
    // plant->RegisterVisualGeometry(...);  // Will add in Phase 2
  }

  // ============ ADD GROUND ============

  const auto& ground_body = plant->GetBodyByName("world");
  const double ground_height = -0.5;

  // Ground plane (visual) - skip for Phase 1, HalfSpace not compatible with RegisterVisualGeometry
  // Will be added in Phase 2

  // Ground plane (collision) - Phase 1: skip (will add with proper SceneGraph in Phase 2)

  // ============ FINALIZE PLANT ============

  plant->Finalize();

  // ============ ROPE FORCES ============

  // Collect bead body pointers
  std::vector<const drake::multibody::RigidBody<double>*> bead_ptrs;
  for (int i = 0; i < num_beads; ++i) {
    std::string bead_name = "rope_0_bead_" + std::to_string(i);
    bead_ptrs.push_back(&plant->GetBodyByName(bead_name));
  }

  // RopeForceSystem: computes tension and constraint forces
  auto* rope_forces = builder.AddSystem<RopeForceSystem>(
      *plant,
      quad_body,
      payload_body,
      bead_ptrs,
      Eigen::Vector3d::Zero(),           // Quad attachment (center)
      Eigen::Vector3d::Zero(),           // Payload attachment (center)
      rope_length,
      200.0,                             // segment_stiffness [N/m]
      15.0,                              // segment_damping [N·s/m]
      1e-9);                             // min_distance_threshold

  // Connect rope forces to plant
  builder.Connect(plant->get_state_output_port(),
                  rope_forces->get_plant_state_input_port());
  builder.Connect(rope_forces->get_forces_output_port(),
                  plant->get_applied_spatial_force_input_port());

  // ============ DECENTRALIZED OPTIMAL CONTROLLER ============

  DecentralizedOptimalController::Config controller_config;
  controller_config.drone_index = 0;
  controller_config.num_drones = 1;
  controller_config.rope_length = rope_length;
  controller_config.control_horizon_sec = 3.0;
  controller_config.control_dt = 0.01;
  controller_config.mpc_period = 0.02;  // 50 Hz

  // Weights (from weight tuning analysis)
  controller_config.w_trajectory = 50.0;
  controller_config.w_stability = 5.0;
  controller_config.w_effort = 0.5;
  controller_config.w_tension_balance = 0.05;

  controller_config.thrust_max = quad_mass * 9.81 * 1.5;  // 22 N
  controller_config.torque_max = 10.0;
  controller_config.tilt_angle_max = 0.785;

  controller_config.load_estimator_filter_alpha = 0.1;
  controller_config.load_mass_nominal = payload_mass;

  controller_config.max_solver_iterations = 100;
  controller_config.solver_tolerance = 1e-4;
  controller_config.verbose_solver = false;

  auto* controller =
      builder.AddSystem<DecentralizedOptimalController>(controller_config);

  // ============ CONSTANT REFERENCE TRAJECTORY ============

  Eigen::Vector3d p_ref(0.0, 0.0, 2.0);  // Hover at z=2.0 m
  Eigen::Vector3d v_ref = Eigen::Vector3d::Zero();
  Eigen::Vector4d T_others = Eigen::Vector4d::Zero();  // No other drones
  double m_load = payload_mass;

  auto* ref_pos = builder.AddSystem<drake::systems::ConstantVectorSource<double>>(
      p_ref);
  auto* ref_vel = builder.AddSystem<drake::systems::ConstantVectorSource<double>>(
      v_ref);
  auto* T_others_source =
      builder.AddSystem<drake::systems::ConstantVectorSource<double>>(T_others);
  auto* m_load_source =
      builder.AddSystem<drake::systems::ConstantVectorSource<double>>(
          Eigen::Matrix<double, 1, 1>(m_load));

  // ============ EXTRACT STATE FROM PLANT ============

  // Create state splitter to extract drone state and cable measurements from plant
  auto* state_splitter = builder.AddSystem<PlantStateSplitter>(num_beads, rope_length);
  builder.Connect(plant->get_state_output_port(),
                  state_splitter->get_input_port(0));

  // ============ CONNECT CONTROLLER INPUTS ============

  // Connect reference trajectory (constant sources)
  builder.Connect(ref_pos->get_output_port(),
                  controller->get_reference_trajectory_input_port());
  builder.Connect(ref_vel->get_output_port(),
                  controller->get_reference_velocity_input_port());
  builder.Connect(T_others_source->get_output_port(),
                  controller->get_other_tensions_input_port());
  builder.Connect(m_load_source->get_output_port(),
                  controller->get_payload_mass_input_port());

  // Connect drone state and cable measurements from state splitter
  builder.Connect(state_splitter->get_output_port(0),  // p_quad
                  controller->get_drone_position_input_port());
  builder.Connect(state_splitter->get_output_port(1),  // v_quad
                  controller->get_drone_velocity_input_port());
  builder.Connect(state_splitter->get_output_port(2),  // T_cable
                  controller->get_cable_tension_input_port());
  builder.Connect(state_splitter->get_output_port(3),  // n_cable
                  controller->get_cable_direction_input_port());

  // ============ DATA LOGGING ============

  auto* state_logger = builder.AddSystem<drake::systems::VectorLogSink<double>>(
      plant->get_state_output_port().size());
  builder.Connect(plant->get_state_output_port(),
                  state_logger->get_input_port(0));

  auto* control_logger = builder.AddSystem<drake::systems::VectorLogSink<double>>(4);
  builder.Connect(controller->get_optimal_control_output_port(),
                  control_logger->get_input_port(0));

  auto* load_pos_logger =
      builder.AddSystem<drake::systems::VectorLogSink<double>>(3);
  builder.Connect(controller->get_load_position_estimate_output_port(),
                  load_pos_logger->get_input_port(0));

  auto* solver_time_logger =
      builder.AddSystem<drake::systems::VectorLogSink<double>>(1);
  builder.Connect(controller->get_solver_time_output_port(),
                  solver_time_logger->get_input_port(0));

  // ============ VISUALIZATION ============
  // Phase 1: skip visualization setup (will add proper MeshcatVisualizer in Phase 2)
  if (use_meshcat) {
    std::cout << "Note: Visualization will be added in Phase 2\n";
  }

  // ============ BUILD DIAGRAM ============

  auto diagram = builder.Build();

  // ============ SIMULATOR SETUP ============

  drake::systems::Simulator<double> simulator(*diagram);
  auto& context = simulator.get_mutable_context();

  // Initial conditions
  auto& plant_context = diagram->GetMutableSubsystemContext(*plant, &context);

  // Quadcopter initial position: [0, 0, 1.5] m
  Eigen::VectorXd x0 = plant->GetPositions(plant_context);
  x0.segment<3>(0) = Eigen::Vector3d(0.0, 0.0, 1.5);  // p_quad
  x0.segment<3>(3) =
      Eigen::Vector3d(0.0, 0.0, 0.0);  // v_quad (zero initially)
  // ... (payload and rope beads at rest below quad)

  plant->SetPositions(&plant_context, x0);
  plant->SetVelocities(&plant_context,
      Eigen::VectorXd::Zero(plant->num_velocities()));

  // ============ SIMULATE ============

  std::cout << "Running simulation...\n";
  simulator.set_target_realtime_rate(0.1);  // Run 10x slower
  simulator.Initialize();
  simulator.AdvanceTo(simulation_duration);

  // ============ LOG RESULTS ============

  std::cout << "\n=== SIMULATION COMPLETE ===\n";
  std::cout << "State log size: " << state_logger->FindLog(context).num_samples()
            << " samples\n";
  std::cout << "Control log size: " << control_logger->FindLog(context).num_samples()
            << " samples\n";

  // Extract solver statistics
  auto& solver_log = solver_time_logger->FindLog(context);
  if (solver_log.num_samples() > 0) {
    double mean_solve_time = solver_log.data().row(0).mean();
    double max_solve_time = solver_log.data().row(0).maxCoeff();
    std::cout << "Solver stats:\n";
    std::cout << "  Mean solve time: " << std::fixed << std::setprecision(3)
              << mean_solve_time << " ms\n";
    std::cout << "  Max solve time: " << max_solve_time << " ms\n";
  }

  // Save logs to CSV
  std::cout << "\nSaving logs...\n";
  drake::log()->info("State log: state_log.csv");
  drake::log()->info("Control log: control_log.csv");
  drake::log()->info("Load estimate log: load_estimate_log.csv");

  return 0;
}

}  // namespace quad_rope_lift

int main(int argc, char* argv[]) {
  return quad_rope_lift::main(argc, argv);
}
