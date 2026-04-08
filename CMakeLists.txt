cmake_minimum_required(VERSION 3.16)

project(TetherGraceFaultTolerance LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(ENABLE_GPAC_DRAKE_WRAPPERS "Build root-owned Drake wrapper systems when Drake headers and link targets are available" OFF)

enable_testing()

add_library(gpac_fault_tolerance_core
  src/gpac_fault_tolerance/adapters/gpac_seam_adapter.cc
  src/gpac_fault_tolerance/core/cable_snap_profile.cc
  src/gpac_fault_tolerance/core/desired_geometry_redistributor.cc
  src/gpac_fault_tolerance/core/fault_metrics.cc
  src/gpac_fault_tolerance/core/l1_adaptive_core.cc
  src/gpac_fault_tolerance/core/reference_trajectory_mapper.cc
)

target_include_directories(gpac_fault_tolerance_core
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

add_executable(l1_adaptive_core_test
  tests/l1_adaptive_core_test.cc
)
target_link_libraries(l1_adaptive_core_test PRIVATE gpac_fault_tolerance_core)
add_test(NAME l1_adaptive_core_test COMMAND l1_adaptive_core_test)

add_executable(fault_metrics_test
  tests/fault_metrics_test.cc
)
target_link_libraries(fault_metrics_test PRIVATE gpac_fault_tolerance_core)
add_test(NAME fault_metrics_test COMMAND fault_metrics_test)

add_executable(cable_snap_profile_test
  tests/cable_snap_profile_test.cc
)
target_link_libraries(cable_snap_profile_test PRIVATE gpac_fault_tolerance_core)
add_test(NAME cable_snap_profile_test COMMAND cable_snap_profile_test)

add_executable(desired_geometry_redistributor_test
  tests/desired_geometry_redistributor_test.cc
)
target_link_libraries(desired_geometry_redistributor_test PRIVATE gpac_fault_tolerance_core)
add_test(NAME desired_geometry_redistributor_test COMMAND desired_geometry_redistributor_test)

add_executable(reference_trajectory_mapper_test
  tests/reference_trajectory_mapper_test.cc
)
target_link_libraries(reference_trajectory_mapper_test PRIVATE gpac_fault_tolerance_core)
add_test(NAME reference_trajectory_mapper_test COMMAND reference_trajectory_mapper_test)

add_executable(gpac_seam_adapter_test
  tests/gpac_seam_adapter_test.cc
)
target_link_libraries(gpac_seam_adapter_test PRIVATE gpac_fault_tolerance_core)
add_test(NAME gpac_seam_adapter_test COMMAND gpac_seam_adapter_test)

if(ENABLE_GPAC_DRAKE_WRAPPERS)
  find_package(drake CONFIG REQUIRED)
  find_package(Eigen3 3.3 REQUIRED NO_MODULE)

  add_library(gpac_fault_tolerance_drake
    src/gpac_fault_tolerance/drake/cable_health_status_source.cc
    src/gpac_fault_tolerance/drake/cable_multiplier_health_adapter.cc
    src/gpac_fault_tolerance/drake/disturbance_bound_projector.cc
    src/gpac_fault_tolerance/drake/fault_tolerant_reference_mapper.cc
    src/gpac_fault_tolerance/drake/gpac_single_vehicle_wiring.cc
    src/gpac_fault_tolerance/drake/l1_gpac_drake_wrapper.cc
    src/gpac_fault_tolerance/drake/nominal_force_acceleration_probe.cc
    src/gpac_fault_tolerance/drake/stacked_drone_trajectory_demux.cc
  )
  target_include_directories(gpac_fault_tolerance_drake
    PUBLIC
      ${CMAKE_CURRENT_SOURCE_DIR}/src
      ${CMAKE_CURRENT_SOURCE_DIR}/Tether_Lift/Research/cpp/include
      ${CMAKE_CURRENT_SOURCE_DIR}/Tether_Lift/Research/cpp/src
  )
  target_link_libraries(gpac_fault_tolerance_drake PUBLIC gpac_fault_tolerance_core drake::drake Eigen3::Eigen)

  add_executable(full_drake_fault_runner
    src/gpac_fault_tolerance/drake/estimated_state_override.cc
    src/gpac_fault_tolerance/drake/fault_detecting_gpac_controller.cc
    src/gpac_fault_tolerance/drake/fault_aware_rope_visualizer.cc
    src/gpac_fault_tolerance/drake/faultable_rope_force_system.cc
    src/gpac_fault_tolerance/drake/full_drake_fault_runner.cc
    Tether_Lift/Research/cpp/src/barometer_sensor.cc
    Tether_Lift/Research/cpp/src/eskf_estimator.cc
    Tether_Lift/Research/cpp/src/extended_state_observer.cc
    Tether_Lift/Research/cpp/src/estimation_utils.cc
    Tether_Lift/Research/cpp/src/gpac_quadcopter_controller.cc
    Tether_Lift/Research/cpp/src/gps_sensor.cc
    Tether_Lift/Research/cpp/src/imu_sensor.cc
    Tether_Lift/Research/cpp/src/quadcopter_controller.cc
    Tether_Lift/Research/cpp/src/rope_utils.cc
    Tether_Lift/Research/cpp/src/simulation_data_logger.cc
    Tether_Lift/Research/cpp/src/wind_disturbance.cc
    Tether_Lift/Research/cpp/src/wind_force_applicator.cc
  )
  target_include_directories(full_drake_fault_runner
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
      ${CMAKE_CURRENT_SOURCE_DIR}/Tether_Lift/Research/cpp/include
      ${CMAKE_CURRENT_SOURCE_DIR}/Tether_Lift/Research/cpp/src
  )
  target_link_libraries(full_drake_fault_runner
    PRIVATE
      gpac_fault_tolerance_drake
      drake::drake
      Eigen3::Eigen
  )
  set_target_properties(full_drake_fault_runner PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
  )

  # H∞ small-gain verification tool
  add_executable(linearize_plant
    src/gpac_fault_tolerance/drake/linearize_plant.cc
  )
  target_include_directories(linearize_plant
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
      ${CMAKE_CURRENT_SOURCE_DIR}/Tether_Lift/Research/cpp/include
  )
  target_link_libraries(linearize_plant
    PRIVATE
      drake::drake
      Eigen3::Eigen
  )
  set_target_properties(linearize_plant PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
  )
endif()


