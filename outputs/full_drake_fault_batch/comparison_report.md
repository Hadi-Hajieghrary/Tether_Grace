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
| matched_n3_1fault | 3 | [0] @ [7.0] | 48.84 | 107.44 | 42.36 | 2.66 |
| matched_n5_1fault | 5 | [0] @ [7.0] | 40.37 | 112.09 | 53.25 | 2.93 |
| matched_n7_1fault | 7 | [0] @ [7.0] | 38.38 | 111.15 | 49.85 | 3.20 |

## Fault Detection And Separation

| Scenario | Faulted drone | Fault time [s] | Detection latency [s] | Healthy separation at fault [m] | Healthy separation +2 s [m] | Healthy final separation [m] |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| matched_n3_1fault | 0 | 7.00 | 0.12 | 0.92 | 1.31 | 2.24 |
| matched_n5_1fault | 0 | 7.00 | 0.12 | 0.67 | 1.05 | 2.29 |
| matched_n7_1fault | 0 | 7.00 | 0.12 | 0.51 | 0.96 | 2.15 |

## Notes

- Pickup trigger threshold is inferred from the controller seam at 0.30 N.
- Detection hold time is 0.12 s and the post-pickup arming delay is 0.75 s.
- Detection time here is inferred from tension traces, using the same threshold formula the runner applies to each controller.
- Separation metrics exclude drones whose own cables have already faulted at the sampled time.
- Scenario-level figures are stored under each scenario folder in the figures/ subdirectory.
