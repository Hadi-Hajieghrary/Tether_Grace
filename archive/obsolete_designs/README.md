# Archive — obsolete intermediate designs

This tree preserves the source-code artefacts of earlier design phases of
Tether_Grace that were **superseded** by the current fully-local QP-based
controller (`DecentralizedLocalController`) and its harness
`decentralized_fault_aware_sim`.

Nothing here is compiled or referenced by the active code. Each sub-folder
contains a `NOTES.txt` explaining what the files were, why the design was
abandoned, and which current file replaces them. Kept in-repo (per the
clean-up decision of 2026-04-21) so the design evolution is readable
without having to dig through git history.

## Layout

| Folder | What it contains |
|--------|------------------|
| `phase1_single_drone_mpc/` | First SNOPT-based MPC controller; one drone, one rope. |
| `phase2_two_drone_mpc/` | Same MPC extended to two drones with a shared-reference coordination scheme. |
| `phase25_inter_drone_comms/` | Added inter-drone tension communication and a multi-rope force combiner. |
| `phase4_box_only_4drone/` | Early four-drone test harness with dumb-box quadrotor visuals (no URDF). |
| `peer_aware_controller/` | The cascade-PD + peer-aware-`N_alive` controller that immediately preceded the fully-local QP. |
| `baselines_from_intermediate_phases/` | Tether_Lift-baseline harnesses used for regression and comparison during earlier phases. |
| `old_runners_and_scripts/` | Earlier campaign-runner shell scripts and Python post-processing tools. |
| `old_design_docs/` | Markdown docs describing superseded designs. |

## Active code (for reference)

The current active code is at `Research/cpp/src/`:
- `decentralized_fault_aware_sim_main.cc` — harness (physics + Meshcat + logging)
- `decentralized_local_controller.cc` — per-drone fully-local QP controller
- `fault_aware_rope_visualizer.cc` — rope polyline with fault-time hiding

Built via `Research/cpp/CMakeLists.txt` target `decentralized_fault_aware_sim`.
