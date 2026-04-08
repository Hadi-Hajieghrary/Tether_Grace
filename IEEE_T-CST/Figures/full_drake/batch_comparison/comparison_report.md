# Full Drake Batch Comparison Report

This report is generated only from the full-Drake batch artifacts in outputs/full_drake_fault_batch.

## Presentation Figures

- comparison_load_tracking_summary.png: batch-level load-tracking summary.
- comparison_fault_response_timing.png: mean detection latency and final separation summary.
- comparison_healthy_neighbor_clearance.png: batch healthy-neighbor clearance summary.
- comparison_load_speed_envelope.png: batch load-speed summary.
- comparison_load_error_overlay.png: load-tracking error overlay for the full 3/5/7 schedule set.
- comparison_fault_event_separation.png: per-fault healthy-neighbor separation at fault, +2 s, and end of run.

## Scenario Summary

| Scenario | Quads | Faults | Load RMSE [cm] | Post-first-fault peak [cm] | Final load error [cm] | Peak load speed [m/s] |
| --- | ---: | --- | ---: | ---: | ---: | ---: |
| three_drones | 3 | [0] @ [7.0] | 53.86 | 106.07 | 41.74 | 2.69 |
| five_drones | 5 | [1, 3] @ [7.0, 12.0] | 43.60 | 114.70 | 60.62 | 2.69 |
| seven_drones | 7 | [1, 3, 5] @ [7.0, 12.0, 14.0] | 40.76 | 115.54 | 62.02 | 2.97 |

## Fault Detection And Separation

| Scenario | Faulted drone | Fault time [s] | Detection latency [s] | Healthy separation at fault [m] | Healthy separation +2 s [m] | Healthy final separation [m] |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| three_drones | 0 | 7.00 | 0.12 | 0.90 | 1.27 | 2.26 |
| five_drones | 1 | 7.00 | 0.12 | 0.66 | 1.07 | 2.69 |
| five_drones | 3 | 12.00 | 0.12 | 0.65 | 1.96 | 4.68 |
| seven_drones | 1 | 7.00 | 0.12 | 0.50 | 1.05 | 2.06 |
| seven_drones | 3 | 12.00 | 0.12 | 0.50 | 1.90 | 4.73 |
| seven_drones | 5 | 14.00 | 0.12 | 0.50 | 1.69 | 3.82 |

## Notes

- Pickup trigger threshold is inferred from the controller seam at 0.30 N.
- Detection hold time is 0.12 s and the post-pickup arming delay is 0.75 s.
- Detection time here is inferred from tension traces, using the same threshold formula the runner applies to each controller.
- Separation metrics exclude drones whose own cables have already faulted at the sampled time.
- Scenario-level figures are stored under each scenario folder in the figures/ subdirectory.
