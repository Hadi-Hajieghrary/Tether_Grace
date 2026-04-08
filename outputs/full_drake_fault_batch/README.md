# Full Drake Fault Batch

This folder contains only root-owned full-Drake multibody scenario artifacts.

## Top-level files

- batch_manifest.json: scenario manifest for the regenerated batch.
- comparison_metrics.json: structured metrics emitted by the analysis pass.
- comparison_report.md: paper-oriented markdown summary of the regenerated batch.
- comparison_scenarios.csv: one row per scenario.
- comparison_fault_events.csv: one row per fault event.
- comparison_load_tracking_summary.png: batch tracking summary.
- comparison_fault_response_timing.png: mean detection and separation summary.
- comparison_healthy_neighbor_clearance.png: healthy-neighbor clearance summary.
- comparison_load_speed_envelope.png: batch peak-speed summary.
- comparison_load_error_overlay.png: load-tracking overlay across scenarios.
- comparison_fault_event_separation.png: healthy-neighbor clearance across all faults.

## Scenarios
- oracle_n3_1fault: 3 drones, faults at [7.0], cables [0]
  - Samples: 3001
  - Log directory: /workspaces/Tether_Grace/outputs/full_drake_fault_batch/oracle_n3_1fault/logs/20260406_231630
  - Meshcat replay: /workspaces/Tether_Grace/outputs/full_drake_fault_batch/oracle_n3_1fault/full_drake_meshcat_replay.html
  - Timeseries archive: /workspaces/Tether_Grace/outputs/full_drake_fault_batch/oracle_n3_1fault/full_drake_recording.npz
  - Scenario figures: cable_health_schedule.png, cable_tensions.png, final_quad_positions.png, load_speed_and_total_vertical_force.png, payload_tracking_error.png, payload_x_tracking.png, payload_xy_trajectory.png, payload_y_tracking.png, payload_z_offset.png, payload_z_tracking.png, quad_altitude.png, quad_force_norm.png, quad_formation_deviation.png, quad_lateral_force_norm.png, quad_speed_norm.png, quad_tilt_magnitude.png, quad_torque_norm.png, vertical_motion.png, xy_trajectories.png
- oracle_n7_3fault: 7 drones, faults at [7.0, 12.0, 14.0], cables [1, 3, 5]
  - Samples: 3001
  - Log directory: /workspaces/Tether_Grace/outputs/full_drake_fault_batch/oracle_n7_3fault/logs/20260406_232326
  - Meshcat replay: /workspaces/Tether_Grace/outputs/full_drake_fault_batch/oracle_n7_3fault/full_drake_meshcat_replay.html
  - Timeseries archive: /workspaces/Tether_Grace/outputs/full_drake_fault_batch/oracle_n7_3fault/full_drake_recording.npz
  - Scenario figures: cable_health_schedule.png, cable_tensions.png, final_quad_positions.png, load_speed_and_total_vertical_force.png, payload_tracking_error.png, payload_x_tracking.png, payload_xy_trajectory.png, payload_y_tracking.png, payload_z_offset.png, payload_z_tracking.png, quad_altitude.png, quad_force_norm.png, quad_formation_deviation.png, quad_lateral_force_norm.png, quad_speed_norm.png, quad_tilt_magnitude.png, quad_torque_norm.png, vertical_motion.png, xy_trajectories.png
