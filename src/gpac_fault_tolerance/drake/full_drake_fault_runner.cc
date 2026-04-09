#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <numeric>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include <drake/common/find_resource.h>
#include <drake/geometry/meshcat.h>
#include <drake/geometry/meshcat_visualizer.h>
#include <drake/geometry/rgba.h>
#include <drake/geometry/shape_specification.h>
#include <drake/math/rigid_transform.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/externally_applied_spatial_force_multiplexer.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/tree/rigid_body.h>
#include <drake/multibody/tree/spatial_inertia.h>
#include <drake/multibody/tree/unit_inertia.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/primitives/constant_vector_source.h>
#include <drake/systems/primitives/zero_order_hold.h>

#include "barometer_sensor.h"
#include "eskf_estimator.h"
#include "estimation_utils.h"
#include "gpac_fault_tolerance/drake/estimated_state_override.h"
#include "gpac_fault_tolerance/drake/fault_aware_rope_visualizer.h"
#include "gpac_fault_tolerance/drake/fault_detecting_gpac_controller.h"
#include "gpac_fault_tolerance/drake/faultable_rope_force_system.h"
#include "gpac_quadcopter_controller.h"
#include "gps_sensor.h"
#include "imu_sensor.h"
#include "quadcopter_controller.h"
#include "rope_utils.h"
#include "simulation_data_logger.h"
#include "wind_disturbance.h"
#include "wind_force_applicator.h"

namespace tether_grace {
namespace {

using drake::geometry::Box;
using drake::geometry::HalfSpace;
using drake::geometry::Meshcat;
using drake::geometry::MeshcatAnimation;
using drake::geometry::MeshcatVisualizer;
using drake::geometry::Rgba;
using drake::geometry::Sphere;
using drake::math::RigidTransformd;
using drake::multibody::ContactModel;
using drake::multibody::CoulombFriction;
using drake::multibody::ExternallyAppliedSpatialForceMultiplexer;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::multibody::RigidBody;
using drake::multibody::SpatialInertia;
using drake::multibody::UnitInertia;
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using drake::systems::ZeroOrderHold;
using quad_rope_lift::CableDirectionFromTension;
using quad_rope_lift::ControllerParams;
using quad_rope_lift::GPACParams;
using quad_rope_lift::GPACWaypoint;
using quad_rope_lift::RopeParameters;
using quad_rope_lift::TrajectoryWaypoint;
using tether_lift::SimulationDataLogger;

struct QuadConfig {
  Eigen::Vector3d initial_position;
  Eigen::Vector3d formation_offset;
  Eigen::Vector3d payload_attachment;
  double rope_length{};
};

// Forward declarations for helper systems (defined after
// EvaluateWaypointTrajectory)
class PayloadStateExtractor;
class LoadTrajectorySource;

struct RunConfig {
  int num_quadcopters{3};
  double duration{25.0};
  bool headless{false};
  bool record_meshcat_html{false};
  std::string output_dir{
      "/workspaces/Tether_Grace/outputs/full_drake_fault_run"};
  std::vector<double> cable_lengths;
  std::vector<int> fault_cables;
  std::vector<double> fault_times;

  // Physics / controller overrides
  double max_thrust{150.0};
  double payload_mass{3.0};
  double tension_kp{0.5};
  bool disable_tension_ff{false}; // zero out entire tension feedforward path
  double position_kp{8.0};
  double position_kd{8.0};
  double position_ki{0.15};
  bool enable_eso{false};
  int seed{-1}; // -1 = deterministic (no stochastic perturbation)

  // Wind disturbance
  bool enable_wind{false};
  double wind_mean_x{1.0};
  double wind_mean_y{0.5};
  double wind_mean_z{0.0};
  double wind_sigma{0.5}; // turbulence intensity scale

  // ESKF + sensor noise (stochastic validation)
  bool enable_eskf{false};
  double imu_rate{200.0}; // Hz (ImuParams default is 400)
  double gps_noise{0.02}; // position noise stddev [m]
  double baro_noise{0.3}; // altitude noise stddev [m]
  double gps_rate{10.0};  // Hz
  double baro_rate{25.0}; // Hz

  // Load-tracking mode: close the loop on payload GPS
  bool enable_load_tracking{true}; // default ON for the improved architecture
  double load_kp{6.0};
  double load_kd{8.0};
  double load_ki{1.2}; // integral on LOAD error (not drone error)

  // Oracle mode: perfect load-share feedforward (centralized upper bound)
  double oracle_load_share{0.0}; // kg; set to m_L/(N-k) for perfect knowledge

  // Trajectory mode: "figure8" (default), "hover", "point_to_point"
  std::string trajectory_mode{"figure8"};

  // Reactive FTC baseline: when enabled, surviving agents boost thrust by
  // this fraction when cable tension exceeds a debounced threshold.
  bool enable_reactive_ftc{false};
  double reactive_boost_fraction{0.3}; // +30% thrust on detection
  double reactive_tension_threshold{
      1.5};                           // N·kg: threshold = this × load_per_rope
  double reactive_debounce_time{0.2}; // seconds

  // Gain-scheduled PID baseline: increase PID gains when tension spike detected
  bool enable_gain_scheduling{false};
  double gs_kp_scale{1.5};  // post-fault kp multiplier
  double gs_kd_scale{1.25}; // post-fault kd multiplier

  // Cable discretization: number of beads per rope segment
  int num_rope_beads{8};
};

std::string Trim(const std::string &value) {
  const size_t start = value.find_first_not_of(" \t\n\r");
  if (start == std::string::npos) {
    return "";
  }
  const size_t end = value.find_last_not_of(" \t\n\r");
  return value.substr(start, end - start + 1);
}

template <typename T> std::vector<T> ParseCsvValues(const std::string &input) {
  std::vector<T> result;
  if (input.empty()) {
    return result;
  }

  std::stringstream stream(input);
  std::string item;
  while (std::getline(stream, item, ',')) {
    item = Trim(item);
    if (item.empty()) {
      continue;
    }
    if constexpr (std::is_same_v<T, int>) {
      result.push_back(std::stoi(item));
    } else {
      result.push_back(static_cast<T>(std::stod(item)));
    }
  }
  return result;
}

std::string JoinInts(const std::vector<int> &values) {
  std::ostringstream stream;
  for (size_t i = 0; i < values.size(); ++i) {
    if (i > 0) {
      stream << ',';
    }
    stream << values[i];
  }
  return stream.str();
}

std::string JoinDoubles(const std::vector<double> &values) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(6);
  for (size_t i = 0; i < values.size(); ++i) {
    if (i > 0) {
      stream << ',';
    }
    stream << values[i];
  }
  return stream.str();
}

std::optional<double> FindSeverTime(const RunConfig &config, int cable_index) {
  for (size_t i = 0; i < config.fault_cables.size(); ++i) {
    if (config.fault_cables[i] == cable_index) {
      return config.fault_times[i];
    }
  }
  return std::nullopt;
}

std::vector<TrajectoryWaypoint> BuildWaypoints(double initial_altitude,
                                               double duration) {
  const double t1 = 0.08 * duration;
  const double t2 = 0.18 * duration;
  const double t3 = 0.28 * duration;
  const double t4 = 0.38 * duration;
  const double t5 = 0.48 * duration;
  const double t6 = 0.58 * duration;
  const double t7 = 0.68 * duration;
  const double t8 = 0.78 * duration;
  const double t9 = 0.85 * duration;
  const double t10 = 0.90 * duration;

  return {
      {Eigen::Vector3d(0.0, 0.0, initial_altitude), 0.0, 1.0},
      {Eigen::Vector3d(0.0, 0.0, 2.8), t1, 0.4},
      {Eigen::Vector3d(1.6, 0.5, 3.0), t2, 0.2},
      {Eigen::Vector3d(2.4, 1.4, 3.2), t3, 0.2},
      {Eigen::Vector3d(3.0, 0.0, 3.1), t4, 0.2},
      {Eigen::Vector3d(2.1, -1.4, 2.9), t5, 0.2},
      {Eigen::Vector3d(0.0, 0.0, 3.0), t6, 0.2},
      {Eigen::Vector3d(-1.6, 0.5, 3.1), t7, 0.2},
      {Eigen::Vector3d(-2.6, 1.4, 3.3), t8, 0.2},
      {Eigen::Vector3d(-3.0, 0.0, 3.1), t9, 0.2},
      {Eigen::Vector3d(-2.0, -1.2, 2.9), t10, 0.2},
      {Eigen::Vector3d(0.0, 0.0, 2.6), duration, 0.0},
  };
}

std::vector<TrajectoryWaypoint> BuildHoverWaypoints(double initial_altitude,
                                                    double duration) {
  const double hover_altitude = 3.0;
  const double t_ascend = 0.08 * duration;
  return {
      {Eigen::Vector3d(0.0, 0.0, initial_altitude), 0.0, 1.0},
      {Eigen::Vector3d(0.0, 0.0, hover_altitude), t_ascend, duration},
  };
}

std::vector<TrajectoryWaypoint>
BuildPointToPointWaypoints(double initial_altitude, double duration) {
  const double cruise_alt = 3.0;
  const double t_ascend = 0.08 * duration;
  const double t_start_move = 0.15 * duration;
  const double t_arrive = 0.50 * duration;
  return {
      {Eigen::Vector3d(0.0, 0.0, initial_altitude), 0.0, 1.0},
      {Eigen::Vector3d(0.0, 0.0, cruise_alt), t_ascend, 0.4},
      {Eigen::Vector3d(-2.0, 0.0, cruise_alt), t_start_move, 0.2},
      {Eigen::Vector3d(2.0, 0.0, cruise_alt), t_arrive, duration},
  };
}

std::vector<TrajectoryWaypoint> BuildWaypointsForMode(const std::string &mode,
                                                      double initial_altitude,
                                                      double duration) {
  if (mode == "hover")
    return BuildHoverWaypoints(initial_altitude, duration);
  if (mode == "point_to_point")
    return BuildPointToPointWaypoints(initial_altitude, duration);
  return BuildWaypoints(initial_altitude, duration); // default: figure8
}

/// Convert TrajectoryWaypoint list to GPACWaypoint list (same data, different
/// type).
std::vector<GPACWaypoint>
ToGPACWaypoints(const std::vector<TrajectoryWaypoint> &waypoints) {
  std::vector<GPACWaypoint> out;
  out.reserve(waypoints.size());
  for (const auto &wp : waypoints) {
    out.push_back({wp.position, wp.arrival_time, wp.hold_time});
  }
  return out;
}

void EvaluateWaypointTrajectory(
    const std::vector<TrajectoryWaypoint> &waypoints, double time,
    Eigen::Vector3d *position, Eigen::Vector3d *velocity) {
  DRAKE_DEMAND(position != nullptr);
  DRAKE_DEMAND(velocity != nullptr);
  DRAKE_DEMAND(!waypoints.empty());

  double segment_start_time = 0.0;
  for (size_t i = 0; i < waypoints.size(); ++i) {
    const auto &waypoint = waypoints[i];
    const double segment_end_time = waypoint.arrival_time;
    const double hold_end_time = segment_end_time + waypoint.hold_time;

    if (time <= segment_end_time) {
      if (i == 0) {
        *position = waypoint.position;
        velocity->setZero();
      } else {
        const auto &previous = waypoints[i - 1];
        const double segment_duration = segment_end_time - segment_start_time;
        if (segment_duration > 1e-6) {
          const double alpha = (time - segment_start_time) / segment_duration;
          *position =
              (1.0 - alpha) * previous.position + alpha * waypoint.position;
          *velocity =
              (waypoint.position - previous.position) / segment_duration;
        } else {
          *position = waypoint.position;
          velocity->setZero();
        }
      }
      return;
    }
    if (time <= hold_end_time) {
      *position = waypoint.position;
      velocity->setZero();
      return;
    }
    segment_start_time = hold_end_time;
  }

  *position = waypoints.back().position;
  velocity->setZero();
}

// =========================================================================
// Helper Drake systems for load-tracking control
// =========================================================================

/// Extracts payload position (3D) and velocity (3D) from the full plant state.
class PayloadStateExtractor final : public drake::systems::LeafSystem<double> {
public:
  PayloadStateExtractor(const MultibodyPlant<double> &plant,
                        const RigidBody<double> &payload_body)
      : plant_(plant), payload_body_index_(payload_body.index()) {
    plant_state_port_ =
        DeclareVectorInputPort("plant_state",
                               plant.num_positions() + plant.num_velocities())
            .get_index();
    DeclareVectorOutputPort("load_position", 3,
                            &PayloadStateExtractor::CalcPosition);
    DeclareVectorOutputPort("load_velocity", 3,
                            &PayloadStateExtractor::CalcVelocity);
  }

  const drake::systems::InputPort<double> &get_plant_state_input_port() const {
    return get_input_port(plant_state_port_);
  }
  const drake::systems::OutputPort<double> &get_position_output_port() const {
    return get_output_port(0);
  }
  const drake::systems::OutputPort<double> &get_velocity_output_port() const {
    return get_output_port(1);
  }

private:
  void CalcPosition(const drake::systems::Context<double> &context,
                    drake::systems::BasicVector<double> *output) const {
    const auto &state = get_input_port(plant_state_port_).Eval(context);
    auto ctx = plant_.CreateDefaultContext();
    plant_.SetPositionsAndVelocities(ctx.get(), state);
    const auto &body = plant_.get_body(payload_body_index_);
    const Eigen::Vector3d pos =
        plant_.EvalBodyPoseInWorld(*ctx, body).translation();
    output->SetFromVector(pos);
  }
  void CalcVelocity(const drake::systems::Context<double> &context,
                    drake::systems::BasicVector<double> *output) const {
    const auto &state = get_input_port(plant_state_port_).Eval(context);
    auto ctx = plant_.CreateDefaultContext();
    plant_.SetPositionsAndVelocities(ctx.get(), state);
    const auto &body = plant_.get_body(payload_body_index_);
    const Eigen::Vector3d vel =
        plant_.EvalBodySpatialVelocityInWorld(*ctx, body).translational();
    output->SetFromVector(vel);
  }

  const MultibodyPlant<double> &plant_;
  drake::multibody::BodyIndex payload_body_index_;
  int plant_state_port_{};
};

/// Outputs the load reference trajectory (9D: p_des, v_des, a_des) at
/// the current time, using the same waypoint interpolation as the runner.
class LoadTrajectorySource final : public drake::systems::LeafSystem<double> {
public:
  explicit LoadTrajectorySource(
      const std::vector<TrajectoryWaypoint> &waypoints)
      : waypoints_(waypoints) {
    DeclareVectorOutputPort("load_trajectory", 9,
                            &LoadTrajectorySource::CalcTrajectory);
  }
  const drake::systems::OutputPort<double> &get_trajectory_output_port() const {
    return get_output_port(0);
  }

private:
  void CalcTrajectory(const drake::systems::Context<double> &context,
                      drake::systems::BasicVector<double> *output) const {
    const double t = context.get_time();
    Eigen::Vector3d pos, vel;
    EvaluateWaypointTrajectory(waypoints_, t, &pos, &vel);
    // Numerical acceleration (finite difference)
    Eigen::Vector3d pos_plus, vel_plus, pos_minus, vel_minus;
    const double dt = 0.01;
    EvaluateWaypointTrajectory(waypoints_, t + dt, &pos_plus, &vel_plus);
    EvaluateWaypointTrajectory(waypoints_, std::max(0.0, t - dt), &pos_minus,
                               &vel_minus);
    Eigen::Vector3d acc = (vel_plus - vel_minus) / (2.0 * dt);

    Eigen::VectorXd traj(9);
    traj << pos, vel, acc;
    output->SetFromVector(traj);
  }

  std::vector<TrajectoryWaypoint> waypoints_;
};

// =========================================================================

std::vector<TrajectoryWaypoint>
BuildDroneWaypoints(const std::vector<TrajectoryWaypoint> &load_waypoints,
                    const Eigen::Vector3d &formation_offset) {
  std::vector<TrajectoryWaypoint> drone_waypoints;
  drone_waypoints.reserve(load_waypoints.size());

  for (const auto &waypoint : load_waypoints) {
    TrajectoryWaypoint adjusted = waypoint;
    adjusted.position += formation_offset;
    drone_waypoints.push_back(adjusted);
  }
  return drone_waypoints;
}

Eigen::Matrix3Xd MakePolyline(const std::vector<Eigen::Vector3d> &points) {
  Eigen::Matrix3Xd polyline(3, static_cast<int>(points.size()));
  for (int i = 0; i < static_cast<int>(points.size()); ++i) {
    polyline.col(i) = points[static_cast<size_t>(i)];
  }
  return polyline;
}

void UpdateTraceLine(Meshcat *meshcat, const std::string &path,
                     const std::vector<Eigen::Vector3d> &points,
                     double line_width, const Rgba &color) {
  DRAKE_DEMAND(meshcat != nullptr);
  if (points.size() < 2) {
    return;
  }
  meshcat->SetLine(path, MakePolyline(points), line_width, color);
}

std::string MakeTrailSegmentPath(const std::string &base_path,
                                 int segment_index) {
  std::ostringstream stream;
  stream << base_path << "/segment_" << std::setw(4) << std::setfill('0')
         << segment_index;
  return stream.str();
}

std::vector<Eigen::Vector3d>
BuildReferenceTrace(const std::vector<TrajectoryWaypoint> &waypoints,
                    double duration, double sample_period) {
  DRAKE_DEMAND(sample_period > 0.0);
  const int num_steps = static_cast<int>(std::ceil(duration / sample_period));
  std::vector<Eigen::Vector3d> trace;
  trace.reserve(static_cast<size_t>(num_steps + 1));
  for (int step = 0; step <= num_steps; ++step) {
    const double time = std::min(duration, step * sample_period);
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
    EvaluateWaypointTrajectory(waypoints, time, &position, &velocity);
    trace.push_back(position);
  }
  return trace;
}

void AddAnimatedTrailToRecording(Meshcat *meshcat, MeshcatAnimation *animation,
                                 const std::string &base_path,
                                 const std::vector<Eigen::Vector3d> &points,
                                 const std::vector<double> &sample_times,
                                 double line_width, const Rgba &color) {
  DRAKE_DEMAND(meshcat != nullptr);
  DRAKE_DEMAND(animation != nullptr);
  DRAKE_DEMAND(points.size() == sample_times.size());
  if (points.size() < 2) {
    return;
  }

  for (size_t i = 1; i < points.size(); ++i) {
    const std::string path =
        MakeTrailSegmentPath(base_path, static_cast<int>(i - 1));
    meshcat->SetLine(path, MakePolyline({points[i - 1], points[i]}), line_width,
                     color);
    meshcat->SetProperty(path, "visible", false);
    animation->SetProperty(0, path, "visible", false);
    animation->SetProperty(animation->frame(sample_times[i]), path, "visible",
                           true);
  }
}

RunConfig ParseArgs(int argc, char *argv[]) {
  RunConfig config;
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--duration") == 0 && i + 1 < argc) {
      config.duration = std::stod(argv[++i]);
    } else if (std::strcmp(argv[i], "--num-quads") == 0 && i + 1 < argc) {
      config.num_quadcopters = std::stoi(argv[++i]);
    } else if (std::strcmp(argv[i], "--output-dir") == 0 && i + 1 < argc) {
      config.output_dir = argv[++i];
    } else if (std::strcmp(argv[i], "--cable-lengths") == 0 && i + 1 < argc) {
      config.cable_lengths = ParseCsvValues<double>(argv[++i]);
    } else if (std::strcmp(argv[i], "--fault-cables") == 0 && i + 1 < argc) {
      config.fault_cables = ParseCsvValues<int>(argv[++i]);
    } else if (std::strcmp(argv[i], "--fault-times") == 0 && i + 1 < argc) {
      config.fault_times = ParseCsvValues<double>(argv[++i]);
    } else if (std::strcmp(argv[i], "--headless") == 0) {
      config.headless = true;
    } else if (std::strcmp(argv[i], "--record-meshcat-html") == 0) {
      config.record_meshcat_html = true;
    } else if (std::strcmp(argv[i], "--max-thrust") == 0 && i + 1 < argc) {
      config.max_thrust = std::stod(argv[++i]);
    } else if (std::strcmp(argv[i], "--payload-mass") == 0 && i + 1 < argc) {
      config.payload_mass = std::stod(argv[++i]);
    } else if (std::strcmp(argv[i], "--tension-kp") == 0 && i + 1 < argc) {
      config.tension_kp = std::stod(argv[++i]);
    } else if (std::strcmp(argv[i], "--disable-tension-ff") == 0) {
      config.disable_tension_ff = true;
    } else if (std::strcmp(argv[i], "--position-kp") == 0 && i + 1 < argc) {
      config.position_kp = std::stod(argv[++i]);
    } else if (std::strcmp(argv[i], "--position-kd") == 0 && i + 1 < argc) {
      config.position_kd = std::stod(argv[++i]);
    } else if (std::strcmp(argv[i], "--position-ki") == 0 && i + 1 < argc) {
      config.position_ki = std::stod(argv[++i]);
    } else if (std::strcmp(argv[i], "--enable-eso") == 0) {
      config.enable_eso = true;
    } else if (std::strcmp(argv[i], "--seed") == 0 && i + 1 < argc) {
      config.seed = std::stoi(argv[++i]);
    } else if (std::strcmp(argv[i], "--enable-wind") == 0) {
      config.enable_wind = true;
    } else if (std::strcmp(argv[i], "--wind-mean") == 0 && i + 3 < argc) {
      config.wind_mean_x = std::stod(argv[++i]);
      config.wind_mean_y = std::stod(argv[++i]);
      config.wind_mean_z = std::stod(argv[++i]);
    } else if (std::strcmp(argv[i], "--wind-sigma") == 0 && i + 1 < argc) {
      config.wind_sigma = std::stod(argv[++i]);
    } else if (std::strcmp(argv[i], "--enable-eskf") == 0) {
      config.enable_eskf = true;
    } else if (std::strcmp(argv[i], "--imu-rate") == 0 && i + 1 < argc) {
      config.imu_rate = std::stod(argv[++i]);
    } else if (std::strcmp(argv[i], "--gps-noise") == 0 && i + 1 < argc) {
      config.gps_noise = std::stod(argv[++i]);
    } else if (std::strcmp(argv[i], "--baro-noise") == 0 && i + 1 < argc) {
      config.baro_noise = std::stod(argv[++i]);
    } else if (std::strcmp(argv[i], "--gps-rate") == 0 && i + 1 < argc) {
      config.gps_rate = std::stod(argv[++i]);
    } else if (std::strcmp(argv[i], "--baro-rate") == 0 && i + 1 < argc) {
      config.baro_rate = std::stod(argv[++i]);
    } else if (std::strcmp(argv[i], "--trajectory") == 0 && i + 1 < argc) {
      config.trajectory_mode = argv[++i];
    } else if (std::strcmp(argv[i], "--enable-reactive-ftc") == 0) {
      config.enable_reactive_ftc = true;
    } else if (std::strcmp(argv[i], "--reactive-boost") == 0 && i + 1 < argc) {
      config.reactive_boost_fraction = std::stod(argv[++i]);
    } else if (std::strcmp(argv[i], "--enable-gain-scheduling") == 0) {
      config.enable_gain_scheduling = true;
    } else if (std::strcmp(argv[i], "--gs-kp-scale") == 0 && i + 1 < argc) {
      config.gs_kp_scale = std::stod(argv[++i]);
    } else if (std::strcmp(argv[i], "--gs-kd-scale") == 0 && i + 1 < argc) {
      config.gs_kd_scale = std::stod(argv[++i]);
    } else if (std::strcmp(argv[i], "--num-rope-beads") == 0 && i + 1 < argc) {
      config.num_rope_beads = std::stoi(argv[++i]);
    } else if (std::strcmp(argv[i], "--oracle-load-share") == 0 &&
               i + 1 < argc) {
      config.oracle_load_share = std::stod(argv[++i]);
    } else if (std::strcmp(argv[i], "--disable-load-tracking") == 0) {
      config.enable_load_tracking = false;
    } else if (std::strcmp(argv[i], "--load-kp") == 0 && i + 1 < argc) {
      config.load_kp = std::stod(argv[++i]);
    } else if (std::strcmp(argv[i], "--load-kd") == 0 && i + 1 < argc) {
      config.load_kd = std::stod(argv[++i]);
    } else if (std::strcmp(argv[i], "--load-ki") == 0 && i + 1 < argc) {
      config.load_ki = std::stod(argv[++i]);
    } else if (std::strcmp(argv[i], "--help") == 0) {
      std::cout << "Usage: full_drake_fault_runner [--duration T] [--num-quads "
                   "N] [--output-dir DIR]"
                << " [--cable-lengths a,b,c] [--fault-cables i,j] "
                   "[--fault-times t1,t2] [--headless] "
                   "[--record-meshcat-html]\n"
                << "  [--max-thrust F] [--payload-mass M] [--tension-kp K] "
                   "[--disable-tension-ff]\n"
                << "  [--position-kp K] [--position-kd K] [--position-ki K]\n"
                << "  [--enable-eso] [--seed N]\n"
                << "  [--enable-wind] [--wind-mean X Y Z] [--wind-sigma S]\n"
                << "  [--enable-eskf] [--imu-rate HZ] [--gps-noise M] "
                   "[--baro-noise M]\n"
                << "  [--gps-rate HZ] [--baro-rate HZ]\n"
                << "  [--num-rope-beads N] (default 8)\n";
      std::exit(0);
    }
  }

  if (config.cable_lengths.empty()) {
    config.cable_lengths.resize(config.num_quadcopters, 1.0);
    for (int i = 0; i < config.num_quadcopters; ++i) {
      config.cable_lengths[i] = 1.0 + 0.03 * i;
    }
  }
  if (static_cast<int>(config.cable_lengths.size()) != config.num_quadcopters) {
    throw std::runtime_error("cable-length count must match --num-quads");
  }
  if (config.fault_cables.size() != config.fault_times.size()) {
    throw std::runtime_error(
        "fault-cables and fault-times must have the same length");
  }
  for (int cable_index : config.fault_cables) {
    if (cable_index < 0 || cable_index >= config.num_quadcopters) {
      throw std::runtime_error("fault cable index out of range");
    }
  }
  return config;
}

void WriteRunManifest(const RunConfig &config, const std::string &log_dir,
                      const std::string &html_path,
                      const std::string &manifest_path) {
  std::ofstream out(manifest_path);
  out << "{\n";
  out << "  \"num_quadcopters\": " << config.num_quadcopters << ",\n";
  out << "  \"duration\": " << std::fixed << std::setprecision(6)
      << config.duration << ",\n";
  out << "  \"output_dir\": \"" << config.output_dir << "\",\n";
  out << "  \"log_dir\": \"" << log_dir << "\",\n";
  out << "  \"meshcat_html\": \"" << html_path << "\",\n";
  out << "  \"cable_lengths\": [" << JoinDoubles(config.cable_lengths)
      << "],\n";
  out << "  \"fault_cables\": [" << JoinInts(config.fault_cables) << "],\n";
  out << "  \"fault_times\": [" << JoinDoubles(config.fault_times) << "],\n";
  out << "  \"max_thrust\": " << std::fixed << std::setprecision(6)
      << config.max_thrust << ",\n";
  out << "  \"payload_mass\": " << std::fixed << std::setprecision(6)
      << config.payload_mass << ",\n";
  out << "  \"tension_kp\": " << std::fixed << std::setprecision(6)
      << config.tension_kp << ",\n";
  out << "  \"disable_tension_ff\": "
      << (config.disable_tension_ff ? "true" : "false") << ",\n";
  out << "  \"position_kp\": " << std::fixed << std::setprecision(6)
      << config.position_kp << ",\n";
  out << "  \"position_kd\": " << std::fixed << std::setprecision(6)
      << config.position_kd << ",\n";
  out << "  \"position_ki\": " << std::fixed << std::setprecision(6)
      << config.position_ki << ",\n";
  out << "  \"enable_eso\": " << (config.enable_eso ? "true" : "false")
      << ",\n";
  out << "  \"seed\": " << config.seed << ",\n";
  out << "  \"enable_wind\": " << (config.enable_wind ? "true" : "false")
      << ",\n";
  out << "  \"wind_mean\": [" << config.wind_mean_x << "," << config.wind_mean_y
      << "," << config.wind_mean_z << "],\n";
  out << "  \"wind_sigma\": " << config.wind_sigma << ",\n";
  out << "  \"enable_eskf\": " << (config.enable_eskf ? "true" : "false")
      << ",\n";
  out << "  \"imu_rate\": " << config.imu_rate << ",\n";
  out << "  \"gps_noise\": " << config.gps_noise << ",\n";
  out << "  \"baro_noise\": " << config.baro_noise << ",\n";
  out << "  \"gps_rate\": " << config.gps_rate << ",\n";
  out << "  \"baro_rate\": " << config.baro_rate << ",\n";
  out << "  \"enable_load_tracking\": "
      << (config.enable_load_tracking ? "true" : "false") << ",\n";
  out << "  \"load_kp\": " << config.load_kp << ",\n";
  out << "  \"load_kd\": " << config.load_kd << ",\n";
  out << "  \"load_ki\": " << config.load_ki << ",\n";
  out << "  \"num_rope_beads\": " << config.num_rope_beads << "\n";
  out << "}\n";
}

} // namespace

int DoMain(int argc, char *argv[]) {
  const RunConfig config = ParseArgs(argc, argv);
  std::filesystem::create_directories(config.output_dir);

  const double simulation_time_step = 2e-4;
  const double quadcopter_mass = 1.5;
  const Eigen::Vector3d quadcopter_dimensions(0.30, 0.30, 0.10);
  const double payload_mass = config.payload_mass;
  const double payload_radius = 0.15;
  const double rope_total_mass = 0.2;
  const int num_rope_beads = config.num_rope_beads;
  const double gravity = 9.81;
  const double initial_altitude = 1.2;
  const double final_altitude = 3.0;
  const double formation_radius = 0.6;
  const double attachment_radius = payload_radius * 0.7;
  const double max_stretch_percentage = 0.15;

  std::vector<QuadConfig> quad_configs(config.num_quadcopters);
  for (int i = 0; i < config.num_quadcopters; ++i) {
    const double angle = 2.0 * M_PI * i / config.num_quadcopters;
    const double x = formation_radius * std::cos(angle);
    const double y = formation_radius * std::sin(angle);
    quad_configs[i].initial_position = Eigen::Vector3d(x, y, initial_altitude);
    quad_configs[i].formation_offset = Eigen::Vector3d(x, y, 0.0);
    quad_configs[i].payload_attachment =
        Eigen::Vector3d(attachment_radius * std::cos(angle),
                        attachment_radius * std::sin(angle), payload_radius);
    quad_configs[i].rope_length = config.cable_lengths[i];
  }

  const double avg_rope_length =
      std::accumulate(config.cable_lengths.begin(), config.cable_lengths.end(),
                      0.0) /
      static_cast<double>(config.cable_lengths.size());
  const double load_per_rope =
      (payload_mass * gravity) / config.num_quadcopters;
  const double effective_rope_stiffness =
      load_per_rope / (avg_rope_length * max_stretch_percentage);
  const int num_segments = num_rope_beads + 1;
  const double segment_stiffness = effective_rope_stiffness * num_segments;

  // Cable-level damping target: derived from the 8-bead reference configuration
  // so that effective cable damping is invariant to discretization.
  // For dampers in series: c_cable = c_segment / num_segments.
  // Reference: 8 beads (9 segments), reference_stiffness = 300, reference_damping = 15.
  constexpr int reference_num_segments = 9;  // 8 beads + 1
  constexpr double reference_stiffness = 300.0;
  constexpr double reference_damping = 15.0;
  const double reference_segment_stiffness =
      effective_rope_stiffness * reference_num_segments;
  const double reference_segment_damping =
      reference_damping *
      std::sqrt(reference_segment_stiffness / reference_stiffness);
  const double cable_damping =
      reference_segment_damping / reference_num_segments;
  const double segment_damping = cable_damping * num_segments;
  const auto waypoints = BuildWaypointsForMode(
      config.trajectory_mode, initial_altitude, config.duration);

  DiagramBuilder<double> builder;
  auto [plant, scene_graph] = drake::multibody::AddMultibodyPlantSceneGraph(
      &builder, simulation_time_step);
  plant.set_contact_model(ContactModel::kPoint);
  const ModelInstanceIndex model_instance =
      plant.AddModelInstance("multi_quad_payload_system");

  const SpatialInertia<double> quad_inertia(
      quadcopter_mass, Eigen::Vector3d::Zero(),
      UnitInertia<double>::SolidBox(quadcopter_dimensions[0],
                                    quadcopter_dimensions[1],
                                    quadcopter_dimensions[2]));

  std::vector<const RigidBody<double> *> quadcopter_bodies;
  std::vector<std::vector<ModelInstanceIndex>> visual_instances_per_quad;
  quadcopter_bodies.reserve(config.num_quadcopters);
  visual_instances_per_quad.reserve(config.num_quadcopters);
  Parser parser(&plant);

  for (int i = 0; i < config.num_quadcopters; ++i) {
    const std::string quad_name = "quadcopter_" + std::to_string(i);
    const RigidBody<double> &quad_body =
        plant.AddRigidBody(quad_name, model_instance, quad_inertia);
    quadcopter_bodies.push_back(&quad_body);

    parser.SetAutoRenaming(true);
    const auto visual_instances = parser.AddModels(
        drake::FindResourceOrThrow("drake/examples/quadrotor/quadrotor.urdf"));
    const auto &visual_base_link =
        plant.GetBodyByName("base_link", visual_instances[0]);
    plant.WeldFrames(quad_body.body_frame(), visual_base_link.body_frame(),
                     RigidTransformd::Identity());
    visual_instances_per_quad.push_back(visual_instances);
  }

  const SpatialInertia<double> payload_inertia(
      payload_mass, Eigen::Vector3d::Zero(),
      UnitInertia<double>::SolidSphere(payload_radius));
  const RigidBody<double> &payload_body =
      plant.AddRigidBody("payload", model_instance, payload_inertia);
  const CoulombFriction<double> ground_friction(0.5, 0.3);
  plant.RegisterCollisionGeometry(payload_body, RigidTransformd::Identity(),
                                  Sphere(payload_radius), "payload_collision",
                                  ground_friction);
  plant.RegisterVisualGeometry(payload_body, RigidTransformd::Identity(),
                               Sphere(payload_radius), "payload_visual",
                               Eigen::Vector4d(0.8, 0.2, 0.2, 1.0));
  plant.RegisterCollisionGeometry(plant.world_body(),
                                  RigidTransformd::Identity(), HalfSpace(),
                                  "ground_collision", ground_friction);
  plant.RegisterVisualGeometry(
      plant.world_body(), RigidTransformd(Eigen::Vector3d(0.0, 0.0, -0.02)),
      Box(12.0, 12.0, 0.04), "ground_visual",
      Eigen::Vector4d(0.7, 0.7, 0.7, 1.0));

  std::vector<std::vector<const RigidBody<double> *>> bead_chains(
      config.num_quadcopters);
  std::vector<RopeParameters> rope_params_vec(config.num_quadcopters);
  for (int q = 0; q < config.num_quadcopters; ++q) {
    rope_params_vec[q] = quad_rope_lift::ComputeRopeParameters(
        num_rope_beads, quad_configs[q].rope_length, rope_total_mass,
        segment_stiffness, segment_damping, true, 0.012);
    const SpatialInertia<double> bead_inertia(
        rope_params_vec[q].bead_mass, Eigen::Vector3d::Zero(),
        UnitInertia<double>::SolidSphere(rope_params_vec[q].bead_radius));

    const double hue =
        static_cast<double>(q) / std::max(1, config.num_quadcopters);
    const Eigen::Vector4d bead_color(0.2 + 0.5 * hue, 0.3, 0.8 - 0.4 * hue,
                                     1.0);
    for (int i = 0; i < num_rope_beads; ++i) {
      const std::string name =
          "rope_" + std::to_string(q) + "_bead_" + std::to_string(i);
      const RigidBody<double> &bead =
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
  const Eigen::Vector3d quad_attachment_offset(
      0.0, 0.0, -quadcopter_dimensions[2] / 2.0 - 0.05);
  const double nominal_payload_reference_z_offset =
      (-quad_attachment_offset.z()) + payload_radius +
      avg_rope_length * max_stretch_percentage;

  std::vector<FaultableRopeForceSystem *> rope_systems;
  std::vector<ZeroOrderHold<double> *> tension_holds;
  std::vector<CableDirectionFromTension *> cable_dirs;
  std::vector<FaultDetectingGPACController *> controllers;
  rope_systems.reserve(config.num_quadcopters);
  tension_holds.reserve(config.num_quadcopters);
  cable_dirs.reserve(config.num_quadcopters);
  controllers.reserve(config.num_quadcopters);

  const auto gpac_waypoints = ToGPACWaypoints(waypoints);

  for (int q = 0; q < config.num_quadcopters; ++q) {
    const std::optional<double> sever_time = FindSeverTime(config, q);
    auto &rope_system = *builder.AddSystem<FaultableRopeForceSystem>(
        plant, *quadcopter_bodies[q], payload_body, bead_chains[q],
        quad_attachment_offset, quad_configs[q].payload_attachment,
        quad_configs[q].rope_length, rope_params_vec[q].segment_stiffness,
        rope_params_vec[q].segment_damping, sever_time);
    rope_systems.push_back(&rope_system);

    auto &tension_hold =
        *builder.AddSystem<ZeroOrderHold<double>>(simulation_time_step, 4);
    builder.Connect(rope_system.get_tension_output_port(),
                    tension_hold.get_input_port());
    tension_holds.push_back(&tension_hold);

    // Cable direction from rope tension (for anti-swing Layer 1)
    auto &cable_dir = *builder.AddSystem<CableDirectionFromTension>();
    builder.Connect(rope_system.get_tension_output_port(),
                    cable_dir.get_tension_input_port());
    cable_dirs.push_back(&cable_dir);

    FaultDetectingGPACParams ctrl_params;
    // Build per-drone waypoints: load reference + lateral offset + cable length
    // in z + geometric z-compensation so that the load waypoints define the
    // actual payload COM target (not a nominal altitude that ignores cable
    // stretch, attachment offsets, and payload geometry).
    // The GPAC controller will NOT add formation_offset again.
    const Eigen::Vector3d full_offset(quad_configs[q].formation_offset.x(),
                                      quad_configs[q].formation_offset.y(),
                                      quad_configs[q].rope_length +
                                          nominal_payload_reference_z_offset);
    const auto drone_wps = BuildDroneWaypoints(waypoints, full_offset);
    ctrl_params.gpac.formation_offset = Eigen::Vector3d::Zero();
    ctrl_params.gpac.waypoints = ToGPACWaypoints(drone_wps);
    ctrl_params.gpac.pickup_target_tension = load_per_rope;
    ctrl_params.gpac.initial_altitude = initial_altitude;
    ctrl_params.gpac.final_altitude = final_altitude;

    // Disable ESO feedforward: the internal ESO double-counts gravity and
    // cable-tension feedforward (it estimates the TOTAL disturbance including
    // what the model-based feedforward already compensates, creating a
    // positive-feedback loop that halves effective thrust).  Keep model-based
    // feedforward (gravity + tension) which is sufficient for this plant.
    ctrl_params.gpac.enable_eso_feedforward = config.enable_eso;

    // Gains tuned for the Drake multi-body plant with cable dynamics.
    ctrl_params.gpac.position_kp = config.position_kp;
    ctrl_params.gpac.position_kd = config.position_kd;
    ctrl_params.gpac.position_ki = config.position_ki;

    // Actuator and tension limits.
    ctrl_params.gpac.max_thrust = config.max_thrust;
    ctrl_params.gpac.tension_kp = config.tension_kp;

    // Tension feedforward ablation: disable entire pathway.
    if (config.disable_tension_ff) {
      ctrl_params.gpac.pickup_target_tension = 0.0;
      ctrl_params.gpac.tension_kp = 0.0;
    }

    ctrl_params.enable_fault_detection = sever_time.has_value();
    ctrl_params.fault_detect_tension_threshold =
        std::max(0.15 * load_per_rope, 0.15);
    ctrl_params.fault_detect_hold_time = 0.12;
    ctrl_params.fault_detect_arming_time = 0.75;
    ctrl_params.first_escape_delay = 1.2;
    ctrl_params.second_escape_delay = 4.0;
    ctrl_params.first_escape_distance = 1.0;
    ctrl_params.second_escape_distance = 2.5;
    ctrl_params.first_escape_climb = 0.7;
    ctrl_params.second_escape_climb = 1.3;
    ctrl_params.escape_tangent_bias = 0.18;
    ctrl_params.escape_velocity_bias = 0.10;

    // Load-tracking mode: close the loop on actual payload position
    ctrl_params.enable_load_tracking = config.enable_load_tracking;
    ctrl_params.load_kp = config.load_kp;
    ctrl_params.load_kd = config.load_kd;
    ctrl_params.load_ki = config.load_ki;
    ctrl_params.initial_theta = config.payload_mass / config.num_quadcopters;
    ctrl_params.oracle_load_share = config.oracle_load_share;

    // Reactive FTC baseline
    ctrl_params.enable_reactive_ftc = config.enable_reactive_ftc;
    ctrl_params.reactive_boost_fraction = config.reactive_boost_fraction;
    ctrl_params.reactive_tension_threshold = config.reactive_tension_threshold;
    ctrl_params.reactive_debounce_time = config.reactive_debounce_time;

    // Gain-scheduled PID baseline
    ctrl_params.enable_gain_scheduling = config.enable_gain_scheduling;
    ctrl_params.gs_kp_scale = config.gs_kp_scale;
    ctrl_params.gs_kd_scale = config.gs_kd_scale;

    auto &controller = *builder.AddSystem<FaultDetectingGPACController>(
        plant, *quadcopter_bodies[q], ctrl_params);
    controllers.push_back(&controller);
  }

  auto &force_combiner =
      *builder.AddSystem<ExternallyAppliedSpatialForceMultiplexer>(
          2 * config.num_quadcopters + (config.enable_wind ? 1 : 0));

  // --- ESKF + sensor noise subsystem ---
  // Per-quadcopter: IMU, GPS, Baro → ESKF → EstimatedStateOverride
  std::vector<EstimatedStateOverride *> state_overrides;
  std::vector<quad_rope_lift::EskfEstimator *> eskf_estimators;
  if (config.enable_eskf) {
    using quad_rope_lift::BarometerParams;
    using quad_rope_lift::BarometerSensor;
    using quad_rope_lift::EskfEstimator;
    using quad_rope_lift::EskfParams;
    using quad_rope_lift::GpsParams;
    using quad_rope_lift::GpsSensor;
    using quad_rope_lift::ImuParams;
    using quad_rope_lift::ImuSensor;

    const double imu_dt = 1.0 / config.imu_rate;

    ImuParams imu_params;
    imu_params.sample_period_sec = imu_dt;

    GpsParams gps_params;
    gps_params.position_noise_stddev =
        Eigen::Vector3d::Constant(config.gps_noise);
    gps_params.sample_period_sec = 1.0 / config.gps_rate;

    BarometerParams baro_params;
    baro_params.white_noise_stddev = config.baro_noise;
    baro_params.sample_period_sec = 1.0 / config.baro_rate;

    EskfParams eskf_params;
    eskf_params.gps_position_noise =
        Eigen::Vector3d::Constant(config.gps_noise);
    eskf_params.baro_altitude_noise = config.baro_noise;

    for (int q = 0; q < config.num_quadcopters; ++q) {
      // Sensors
      auto &imu = *builder.AddSystem<ImuSensor>(plant, *quadcopter_bodies[q],
                                                imu_params);
      builder.Connect(plant.get_state_output_port(),
                      imu.get_plant_state_input_port());

      auto &gps = *builder.AddSystem<GpsSensor>(plant, *quadcopter_bodies[q],
                                                gps_params);
      builder.Connect(plant.get_state_output_port(),
                      gps.get_plant_state_input_port());

      auto &baro = *builder.AddSystem<BarometerSensor>(
          plant, *quadcopter_bodies[q], baro_params);
      builder.Connect(plant.get_state_output_port(),
                      baro.get_plant_state_input_port());

      // ESKF
      auto &eskf = *builder.AddSystem<EskfEstimator>(imu_dt, eskf_params);
      eskf_estimators.push_back(&eskf);
      builder.Connect(imu.get_accel_output_port(), eskf.get_accel_input_port());
      builder.Connect(imu.get_gyro_output_port(), eskf.get_gyro_input_port());
      builder.Connect(gps.get_gps_position_output_port(),
                      eskf.get_gps_position_input_port());
      builder.Connect(gps.get_gps_valid_output_port(),
                      eskf.get_gps_valid_input_port());
      builder.Connect(baro.get_altitude_output_port(),
                      eskf.get_baro_altitude_input_port());
      builder.Connect(baro.get_baro_valid_output_port(),
                      eskf.get_baro_valid_input_port());

      // State override: merge ESKF estimate into full plant state
      auto &override_sys = *builder.AddSystem<EstimatedStateOverride>(
          plant, *quadcopter_bodies[q]);
      builder.Connect(plant.get_state_output_port(),
                      override_sys.get_plant_state_input_port());
      builder.Connect(eskf.get_estimated_pose_output_port(),
                      override_sys.get_estimated_pose_input_port());
      builder.Connect(eskf.get_estimated_velocity_output_port(),
                      override_sys.get_estimated_velocity_input_port());
      state_overrides.push_back(&override_sys);
    }
  }

  // --- Payload state extraction + load trajectory (for load-tracking mode) ---
  PayloadStateExtractor *payload_extractor = nullptr;
  LoadTrajectorySource *load_traj_source = nullptr;
  if (config.enable_load_tracking) {
    payload_extractor =
        &*builder.AddSystem<PayloadStateExtractor>(plant, payload_body);
    builder.Connect(plant.get_state_output_port(),
                    payload_extractor->get_plant_state_input_port());

    load_traj_source = &*builder.AddSystem<LoadTrajectorySource>(waypoints);
  }

  for (int q = 0; q < config.num_quadcopters; ++q) {
    builder.Connect(plant.get_state_output_port(),
                    rope_systems[q]->get_plant_state_input_port());
    builder.Connect(rope_systems[q]->get_forces_output_port(),
                    force_combiner.get_input_port(2 * q + 1));

    // Controller gets ESKF-estimated state when enabled, else ground truth
    if (config.enable_eskf) {
      builder.Connect(state_overrides[q]->get_output_port_ref(),
                      controllers[q]->get_plant_state_input_port());
    } else {
      builder.Connect(plant.get_state_output_port(),
                      controllers[q]->get_plant_state_input_port());
    }

    builder.Connect(tension_holds[q]->get_output_port(),
                    controllers[q]->get_tension_input_port());
    builder.Connect(cable_dirs[q]->get_direction_output_port(),
                    controllers[q]->get_cable_direction_input_port());

    // Load-tracking connections: payload position/velocity + load trajectory
    if (config.enable_load_tracking && payload_extractor && load_traj_source) {
      builder.Connect(payload_extractor->get_position_output_port(),
                      controllers[q]->get_load_position_input_port());
      builder.Connect(payload_extractor->get_velocity_output_port(),
                      controllers[q]->get_load_velocity_input_port());
      builder.Connect(load_traj_source->get_trajectory_output_port(),
                      controllers[q]->get_load_trajectory_input_port());
    } else {
      // Fix with default zero values when load-tracking is disabled
      auto &pos_src = *builder.AddSystem<drake::systems::ConstantVectorSource>(
          Eigen::Vector3d::Zero());
      auto &vel_src = *builder.AddSystem<drake::systems::ConstantVectorSource>(
          Eigen::Vector3d::Zero());
      auto &traj_src = *builder.AddSystem<drake::systems::ConstantVectorSource>(
          Eigen::VectorXd::Zero(9));
      builder.Connect(pos_src.get_output_port(),
                      controllers[q]->get_load_position_input_port());
      builder.Connect(vel_src.get_output_port(),
                      controllers[q]->get_load_velocity_input_port());
      builder.Connect(traj_src.get_output_port(),
                      controllers[q]->get_load_trajectory_input_port());
    }

    builder.Connect(controllers[q]->get_control_output_port(),
                    force_combiner.get_input_port(2 * q));
  }

  // --- Wind disturbance subsystem ---
  if (config.enable_wind) {
    using quad_rope_lift::AttachmentPositionExtractor;
    using quad_rope_lift::DragParams;
    using quad_rope_lift::DrydenTurbulenceParams;
    using quad_rope_lift::GustParams;
    using quad_rope_lift::WindDisturbance;
    using quad_rope_lift::WindForceApplicator;

    DrydenTurbulenceParams turb_params;
    turb_params.mean_wind = Eigen::Vector3d(
        config.wind_mean_x, config.wind_mean_y, config.wind_mean_z);
    turb_params.sigma_u = config.wind_sigma;
    turb_params.sigma_v = config.wind_sigma;
    turb_params.sigma_w = config.wind_sigma * 0.5;

    auto &wind_system = *builder.AddSystem<WindDisturbance>(
        config.num_quadcopters, turb_params, GustParams(), 0.01);
    if (config.seed >= 0) {
      wind_system.set_seed(static_cast<unsigned int>(config.seed));
    }

    auto &pos_extractor = *builder.AddSystem<AttachmentPositionExtractor>(
        plant, quadcopter_bodies,
        std::vector<Eigen::Vector3d>(config.num_quadcopters,
                                     Eigen::Vector3d::Zero()));
    builder.Connect(plant.get_state_output_port(),
                    pos_extractor.get_plant_state_input_port());
    builder.Connect(pos_extractor.get_positions_output_port(),
                    wind_system.get_drone_positions_input_port());

    auto &wind_force = *builder.AddSystem<WindForceApplicator>(
        quadcopter_bodies, payload_body, config.num_quadcopters, DragParams());
    builder.Connect(wind_system.get_wind_velocities_output_port(),
                    wind_force.get_wind_input_port());
    builder.Connect(wind_force.get_forces_output_port(),
                    force_combiner.get_input_port(2 * config.num_quadcopters));
  }

  builder.Connect(force_combiner.get_output_port(),
                  plant.get_applied_spatial_force_input_port());

  const bool enable_meshcat = !config.headless || config.record_meshcat_html;

  std::shared_ptr<Meshcat> meshcat;
  if (enable_meshcat) {
    meshcat = std::make_shared<Meshcat>();
    meshcat->Delete();
    MeshcatVisualizer<double>::AddToBuilder(&builder, scene_graph, meshcat);
    meshcat->SetObject("reference/load", Sphere(0.08),
                       Rgba(0.10, 0.45, 0.90, 0.75));
  }

  if (enable_meshcat) {
    const std::vector<Rgba> rope_colors = {
        Rgba(0.2, 0.2, 0.8, 1.0),
        Rgba(0.2, 0.8, 0.2, 1.0),
        Rgba(0.9, 0.7, 0.2, 1.0),
        Rgba(0.8, 0.2, 0.8, 1.0),
    };
    for (int q = 0; q < config.num_quadcopters; ++q) {
      std::vector<std::pair<const RigidBody<double> *, Eigen::Vector3d>>
          rope_points;
      rope_points.emplace_back(quadcopter_bodies[q], quad_attachment_offset);
      for (const auto *bead : bead_chains[q]) {
        rope_points.emplace_back(bead, Eigen::Vector3d::Zero());
      }
      rope_points.emplace_back(&payload_body,
                               quad_configs[q].payload_attachment);
      auto &rope_visualizer = *builder.AddSystem<FaultAwareRopeVisualizer>(
          plant, rope_points, meshcat, "rope_line_" + std::to_string(q), 3.0,
          rope_colors[q % rope_colors.size()], FindSeverTime(config, q));
      builder.Connect(plant.get_state_output_port(),
                      rope_visualizer.get_plant_state_input_port());
    }
  }

  std::vector<drake::multibody::BodyIndex> drone_body_indices;
  drone_body_indices.reserve(quadcopter_bodies.size());
  for (const auto *body : quadcopter_bodies) {
    drone_body_indices.push_back(body->index());
  }

  SimulationDataLogger::Params logger_params;
  logger_params.base_output_dir = config.output_dir + "/logs";
  logger_params.log_period = 0.01;
  logger_params.num_quadcopters = config.num_quadcopters;
  logger_params.log_plant_state = true;
  logger_params.log_tensions = true;
  logger_params.log_control_efforts = true;
  logger_params.log_gps_measurements = false;
  logger_params.log_estimator_outputs = false;
  logger_params.log_reference_trajectory = false;
  logger_params.log_imu_measurements = false;
  logger_params.log_barometer_measurements = false;
  logger_params.log_rope_states = false;
  logger_params.log_attitude_data = true;
  logger_params.log_gpac_signals = false;
  logger_params.log_wind_disturbance = false;

  auto &data_logger = *builder.AddSystem<SimulationDataLogger>(
      plant, payload_body.index(), drone_body_indices, logger_params);
  builder.Connect(plant.get_state_output_port(),
                  data_logger.get_plant_state_input());
  for (int q = 0; q < config.num_quadcopters; ++q) {
    builder.Connect(rope_systems[q]->get_tension_output_port(),
                    data_logger.get_tension_input(q));
    builder.Connect(controllers[q]->get_control_vector_output_port(),
                    data_logger.get_control_effort_input(q));
  }

  auto diagram = builder.Build();
  Simulator<double> simulator(*diagram);
  auto &context = simulator.get_mutable_context();
  auto &plant_context = plant.GetMyMutableContextFromRoot(&context);

  for (int q = 0; q < config.num_quadcopters; ++q) {
    const double visual_model_mass =
        plant.CalcTotalMass(plant_context, visual_instances_per_quad[q]);
    controllers[q]->set_mass(quadcopter_mass + visual_model_mass);
  }

  const double ground_clearance = 0.01;
  plant.SetFreeBodyPose(&plant_context, payload_body,
                        RigidTransformd(Eigen::Vector3d(
                            0.0, 0.0, payload_radius + ground_clearance)));
  const double initial_payload_height =
      plant.EvalBodyPoseInWorld(plant_context, payload_body).translation().z();
  for (int q = 0; q < config.num_quadcopters; ++q) {
    plant.SetFreeBodyPose(&plant_context, *quadcopter_bodies[q],
                          RigidTransformd(quad_configs[q].initial_position));
  }

  for (int q = 0; q < config.num_quadcopters; ++q) {
    const auto &quad_pose =
        plant.EvalBodyPoseInWorld(plant_context, *quadcopter_bodies[q]);
    const auto &payload_pose =
        plant.EvalBodyPoseInWorld(plant_context, payload_body);
    const Eigen::Vector3d rope_start_world = quad_pose * quad_attachment_offset;
    const Eigen::Vector3d rope_end_world =
        payload_pose * quad_configs[q].payload_attachment;
    Eigen::Vector3d lateral = Eigen::Vector3d::UnitZ().cross(
        (rope_end_world - rope_start_world).normalized());
    if (lateral.norm() < 0.1) {
      lateral = Eigen::Vector3d::UnitX();
    }
    lateral.normalize();
    const auto bead_positions = quad_rope_lift::GenerateSlackRopePositions(
        rope_start_world, rope_end_world, num_rope_beads,
        quad_configs[q].rope_length, 0.85, lateral, 1.0);
    for (int i = 0; i < num_rope_beads; ++i) {
      plant.SetFreeBodyPose(&plant_context, *bead_chains[q][i],
                            RigidTransformd(bead_positions[i]));
    }
  }

  // Initialize ESKF states with each quadcopter's initial pose
  if (config.enable_eskf) {
    for (int q = 0; q < config.num_quadcopters; ++q) {
      auto &eskf_context =
          eskf_estimators[q]->GetMyMutableContextFromRoot(&context);
      eskf_estimators[q]->SetInitialState(
          &eskf_context, quad_configs[q].initial_position,
          Eigen::Vector3d::Zero(),
          Eigen::Vector4d(1.0, 0.0, 0.0, 0.0)); // identity quaternion
    }
  }

  std::map<std::string, std::string> config_params;
  config_params["simulation_time_step"] = std::to_string(simulation_time_step);
  config_params["simulation_duration"] = std::to_string(config.duration);
  config_params["num_quadcopters"] = std::to_string(config.num_quadcopters);
  config_params["payload_mass"] = std::to_string(payload_mass);
  config_params["payload_radius"] = std::to_string(payload_radius);
  config_params["avg_rope_length"] = std::to_string(avg_rope_length);
  config_params["segment_stiffness"] = std::to_string(segment_stiffness);
  config_params["segment_damping"] = std::to_string(segment_damping);
  config_params["fault_cables"] = JoinInts(config.fault_cables);
  config_params["fault_times"] = JoinDoubles(config.fault_times);
  for (int q = 0; q < config.num_quadcopters; ++q) {
    config_params["quad_" + std::to_string(q) + "_rope_length"] =
        std::to_string(quad_configs[q].rope_length);
  }
  data_logger.WriteConfigFile(config_params);

  diagram->ForcedPublish(context);
  const double progress_interval = 0.1;
  std::vector<Eigen::Vector3d> load_actual_trace;
  std::vector<Eigen::Vector3d> load_reference_trace;
  std::vector<std::vector<Eigen::Vector3d>> quad_actual_traces(
      config.num_quadcopters);
  std::vector<double> trace_sample_times;
  const double payload_liftoff_threshold = initial_payload_height + 0.05;
  int payload_liftoff_samples = 0;
  bool payload_has_lifted_off = false;

  if (enable_meshcat && config.record_meshcat_html) {
    meshcat->SetLine("traces/load_reference",
                     MakePolyline(BuildReferenceTrace(
                         waypoints, config.duration, progress_interval)),
                     2.0, Rgba(0.10, 0.45, 0.90, 0.90));
  }

  const auto append_trace_sample = [&]() {
    trace_sample_times.push_back(context.get_time());
    const Eigen::Vector3d load_position =
        plant.EvalBodyPoseInWorld(plant_context, payload_body).translation();
    if (!payload_has_lifted_off) {
      if (load_position.z() >= payload_liftoff_threshold) {
        ++payload_liftoff_samples;
        if (payload_liftoff_samples >= 2) {
          payload_has_lifted_off = true;
        }
      } else {
        payload_liftoff_samples = 0;
      }
    }
    load_actual_trace.push_back(load_position);

    Eigen::Vector3d load_reference_position{};
    Eigen::Vector3d load_reference_velocity{};
    EvaluateWaypointTrajectory(waypoints, context.get_time(),
                               &load_reference_position,
                               &load_reference_velocity);

    // With the geometric z-compensation baked into the drone waypoints,
    // the load waypoints now represent the desired payload COM.  The only
    // display adjustment is pinning the reference to the ground height
    // until the payload has actually lifted off.
    Eigen::Vector3d displayed_load_reference = load_reference_position;
    if (!payload_has_lifted_off) {
      displayed_load_reference.z() = initial_payload_height;
    }
    load_reference_trace.push_back(displayed_load_reference);

    if (enable_meshcat) {
      meshcat->SetTransform("reference/load",
                            RigidTransformd(displayed_load_reference));
      if (!config.record_meshcat_html) {
        UpdateTraceLine(meshcat.get(), "traces/load_actual", load_actual_trace,
                        2.5, Rgba(0.92, 0.42, 0.18, 1.0));
        UpdateTraceLine(meshcat.get(), "traces/load_reference",
                        load_reference_trace, 2.0,
                        Rgba(0.10, 0.45, 0.90, 0.90));
      }
    }

    for (int q = 0; q < config.num_quadcopters; ++q) {
      const Eigen::Vector3d quad_position =
          plant.EvalBodyPoseInWorld(plant_context, *quadcopter_bodies[q])
              .translation();
      quad_actual_traces[q].push_back(quad_position);
      if (enable_meshcat) {
        const double hue =
            static_cast<double>(q) / std::max(1, config.num_quadcopters);
        if (!config.record_meshcat_html) {
          UpdateTraceLine(meshcat.get(), "traces/drone_" + std::to_string(q),
                          quad_actual_traces[q], 1.5,
                          Rgba(0.20 + 0.60 * hue, 0.25 + 0.30 * (1.0 - hue),
                               0.85 - 0.45 * hue, 0.95));
        }
      }
    }
  };
  append_trace_sample();

  if (enable_meshcat) {
    meshcat->StartRecording();
  }

  if (!config.headless) {
    simulator.set_target_realtime_rate(1.0);
  } else {
    simulator.set_target_realtime_rate(0.0);
  }

  std::cout << "Running full Drake fault simulation\n";
  std::cout << "  drones: " << config.num_quadcopters << "\n";
  std::cout << "  duration: " << config.duration << " s\n";
  std::cout << "  fault cables: " << JoinInts(config.fault_cables) << "\n";
  std::cout << "  fault times: " << JoinDoubles(config.fault_times) << "\n";
  std::cout << "  output: " << config.output_dir << "\n";

  double current_time = 0.0;
  while (current_time < config.duration) {
    current_time = std::min(config.duration, current_time + progress_interval);
    simulator.AdvanceTo(current_time);
    append_trace_sample();
  }

  std::string html_path;
  if (enable_meshcat) {
    meshcat->StopRecording();
    if (config.record_meshcat_html) {
      auto &animation = meshcat->get_mutable_recording();
      AddAnimatedTrailToRecording(
          meshcat.get(), &animation, "traces/load_actual", load_actual_trace,
          trace_sample_times, 2.5, Rgba(0.92, 0.42, 0.18, 1.0));
      for (int q = 0; q < config.num_quadcopters; ++q) {
        const double hue =
            static_cast<double>(q) / std::max(1, config.num_quadcopters);
        AddAnimatedTrailToRecording(
            meshcat.get(), &animation, "traces/drone_" + std::to_string(q),
            quad_actual_traces[q], trace_sample_times, 1.5,
            Rgba(0.20 + 0.60 * hue, 0.25 + 0.30 * (1.0 - hue),
                 0.85 - 0.45 * hue, 0.95));
      }
    }
    meshcat->PublishRecording();
    html_path = config.output_dir + "/sim_recording_n" +
                std::to_string(config.num_quadcopters) + ".html";
    std::ofstream html_file(html_path);
    html_file << meshcat->StaticHtml();
  }

  data_logger.Finalize();
  const std::string manifest_path = config.output_dir + "/run_manifest.json";
  WriteRunManifest(config, data_logger.output_dir(), html_path, manifest_path);

  std::cout << "Log directory: " << data_logger.output_dir() << "\n";
  std::cout << "Run manifest: " << manifest_path << "\n";
  if (!html_path.empty()) {
    std::cout << "Meshcat HTML: " << html_path << "\n";
    std::cout << "Nominal payload z compensation: "
              << nominal_payload_reference_z_offset << " m\n";
  }
  return 0;
}

} // namespace tether_grace

int main(int argc, char *argv[]) {
  try {
    return tether_grace::DoMain(argc, argv);
  } catch (const std::exception &exc) {
    std::cerr << "full_drake_fault_runner failed: " << exc.what() << std::endl;
    return 1;
  }
}