# Prior campaigns

Raw simulation outputs from controller-development phases that have
since been superseded. These directories are **not** tracked in git
(too large); they are kept on disk so historical runs can be inspected
without rebuilding. Each subfolder contains its original README and a
short NOTES.txt describing what it was used for and what replaced it.

| Subfolder | Superseded by |
|-----------|---------------|
| `4drone_S1_S6/` | `output/5drone_baseline_campaign/` (extended to N=5 + A/B/C/D fault topologies). |
| `decentralized_4drone_development/` | Per-tick diagnostics during the pre-release bring-up of the decentralised controller. No longer referenced. |
| `tether_lift_outputs/` | Upstream Tether_Lift baseline runs (`n4`, `n5`, `n6`). The submodule at `Tether_Lift/` remains the authoritative source. |

Do not cite numbers from this tree in any published work.
