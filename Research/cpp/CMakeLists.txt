cmake_minimum_required(VERSION 3.16)
project(tether_grace_research CXX)

# ---------------------------------------------------------------------------
# C++ standard
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to Release
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# ---------------------------------------------------------------------------
# External dependencies (same as Tether_Lift baseline)
# ---------------------------------------------------------------------------
find_package(drake CONFIG REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# ---------------------------------------------------------------------------
# Tether_Lift source tree (read-only baseline; never modified)
# ---------------------------------------------------------------------------
set(TETHER_LIFT_ROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/../../Tether_Lift/Research/cpp"
    CACHE PATH "Path to the Tether_Lift Research/cpp directory")

message(STATUS "Tether_Lift source root: ${TETHER_LIFT_ROOT}")

# All Tether_Lift systems except its own main.cc
set(TETHER_LIFT_SOURCES
    ${TETHER_LIFT_ROOT}/src/rope_force_system.cc
    ${TETHER_LIFT_ROOT}/src/quadcopter_controller.cc
    ${TETHER_LIFT_ROOT}/src/rope_visualizer.cc
    ${TETHER_LIFT_ROOT}/src/rope_utils.cc
    ${TETHER_LIFT_ROOT}/src/tension_plotter.cc
    ${TETHER_LIFT_ROOT}/src/gps_sensor.cc
    ${TETHER_LIFT_ROOT}/src/position_velocity_estimator.cc
    ${TETHER_LIFT_ROOT}/src/load_estimator.cc
    ${TETHER_LIFT_ROOT}/src/estimation_utils.cc
    ${TETHER_LIFT_ROOT}/src/estimation_error_computer.cc
    ${TETHER_LIFT_ROOT}/src/imu_sensor.cc
    ${TETHER_LIFT_ROOT}/src/barometer_sensor.cc
    ${TETHER_LIFT_ROOT}/src/eskf_estimator.cc
    ${TETHER_LIFT_ROOT}/src/adaptive_load_estimator.cc
    ${TETHER_LIFT_ROOT}/src/decentralized_load_estimator.cc
    ${TETHER_LIFT_ROOT}/src/adaptive_lift_controller.cc
    ${TETHER_LIFT_ROOT}/src/load_trajectory_generator.cc
    ${TETHER_LIFT_ROOT}/src/cbf_safety_filter.cc
    ${TETHER_LIFT_ROOT}/src/wind_disturbance.cc
    ${TETHER_LIFT_ROOT}/src/wind_force_applicator.cc
    ${TETHER_LIFT_ROOT}/src/required_force_computer.cc
    ${TETHER_LIFT_ROOT}/src/force_allocation_system.cc
    ${TETHER_LIFT_ROOT}/src/drone_trajectory_mapper.cc
    ${TETHER_LIFT_ROOT}/src/extended_load_trajectory_generator.cc
    ${TETHER_LIFT_ROOT}/src/load_tracking_controller.cc
    ${TETHER_LIFT_ROOT}/src/decentralized_drone_controller.cc
    ${TETHER_LIFT_ROOT}/src/trajectory_visualizer.cc
    ${TETHER_LIFT_ROOT}/src/simulation_data_logger.cc
    ${TETHER_LIFT_ROOT}/src/extended_state_observer.cc
    ${TETHER_LIFT_ROOT}/src/gpac_quadcopter_controller.cc
    ${TETHER_LIFT_ROOT}/src/gpac_load_tracking_controller.cc
    ${TETHER_LIFT_ROOT}/src/concurrent_learning_estimator.cc
    ${TETHER_LIFT_ROOT}/src/gpac_cbf_safety_filter.cc
)

# ---------------------------------------------------------------------------
# Active target: the decentralised fault-aware cooperative-lift simulator.
# Per-drone controllers (baseline QP + MPC) wire into Tether_Lift's
# MultibodyPlant, bead-chain rope model, and Meshcat visualiser.
# Superseded targets from earlier design phases live under archive/.
# ---------------------------------------------------------------------------
add_executable(decentralized_fault_aware_sim
    src/decentralized_fault_aware_sim_main.cc
    src/decentralized_local_controller.cc
    src/mpc_local_controller.cc
    src/fault_aware_rope_visualizer.cc
    src/cl_param_estimator.cc
    ${TETHER_LIFT_SOURCES}
)

target_include_directories(decentralized_fault_aware_sim PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${TETHER_LIFT_ROOT}/include
    ${TETHER_LIFT_ROOT}/src
)

target_link_libraries(decentralized_fault_aware_sim PRIVATE
    drake::drake
    Eigen3::Eigen
)

message(STATUS "Drake found at: ${drake_DIR}")
