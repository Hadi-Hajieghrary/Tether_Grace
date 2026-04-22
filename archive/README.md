# Archive

Historical material preserved for provenance. Nothing in here is part of
the active codebase, and nothing in here is referenced by the top-level
`README.md`, `Research/README.md`, or
`Research/docs/IMPLEMENTATION_STATUS.md`. If you are onboarding, skip
this directory.

## Layout

| Path | Contents |
|------|----------|
| `obsolete_designs/` | Source code from the five controller iterations that preceded the current decentralised QP baseline (single-drone MPC → two-drone → inter-drone comms → box-only 4-drone → peer-aware). Each subfolder carries a `NOTES.txt` explaining why it was retired. |
| `phase_docs/` | Early Phase-1/Phase-2 planning, risk registers, and Drake-API audit memos from the pre-baseline exploration. |
| `prior_campaigns/4drone_S1_S6/` | Output of the original 6-scenario 4-drone campaign (Tether_Grace/S1..S6). Superseded by the 5-drone A/B/C/D campaign under `output/`. |
| `prior_campaigns/decentralized_4drone_development/` | Intermediate 4-drone replays + source data collected during the decentralised-controller bring-up. |
| `prior_campaigns/tether_lift_outputs/` | Output from the upstream Tether_Lift baseline simulations (`n4`, `n5`, `n6`). |
| `partial_ablation_2026_04/` | Incomplete run of the April 2026 ablation matrix (5 of 16 configurations completed before it was stopped). Retained as historical reference; superseded by the full Monte-Carlo campaign under `output/`. |
| `session_notes/` | Planning notes and superseded LaTeX drafts produced during the April 2026 development sprint. |

## Retention policy

- Items land here when they are *superseded*, not when they are *wrong*.
  Deleted code and deleted results — incorrect or unsalvageable — are
  gone; what remains is the historical path the project walked.
- Nothing in `archive/` is updated in place. If something here needs
  re-activating, copy it back into the active tree rather than editing
  it here.
- Size budget: aim to keep `archive/` under 5 GB. When the next
  campaign supersedes today's `output/`, the current output is moved
  into `archive/prior_campaigns/` and raw trajectory CSVs larger than
  100 MB are pruned first.
