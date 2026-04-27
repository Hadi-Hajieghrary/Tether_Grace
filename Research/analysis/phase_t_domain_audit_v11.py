#!/usr/bin/env python3
"""Phase T Domain Audit — Contract v1.1 criterion.

Gate: for every demonstrated mission,
    max slack run duration    <= tau_slack_max (40 ms)
    slack duty cycle          <= eta           (2.5%)

A "slack run" is a maximal contiguous interval where at least one
surviving cable has T_i < T_taut_min = 0.5 N, restricted to the
post-pickup window t >= t_pickup.

Output: output/phase_t/domain_audit_v11_report.csv
        output/phase_t/domain_audit_v11.log
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
from plot_capability_demo import num_drones
from plot_review_augmentation import first_fault_per_rope

CAMPAIGN = Path("/workspaces/Tether_Grace/output/capability_demo")
OUT = Path("/workspaces/Tether_Grace/output/phase_t")
OUT.mkdir(parents=True, exist_ok=True)

T_PICKUP = 1.0
TAUT_MIN_N = 0.5              # threshold below which a surviving cable is "slack"
TAU_SLACK_MAX_S = 0.040       # max allowed single slack-run duration
ETA_MAX = 0.025               # max allowed slack duty cycle

VARIANTS = [
    "V1_nominal_nowind", "V2_nominal_wind", "V3_single_wind",
    "V4_dual_5s_wind", "V5_dual_10s_wind", "V6_dual_5s_fullstack",
]


def slack_runs(df: pd.DataFrame):
    """Return (max_run_s, duty_cycle, n_runs) over post-pickup window."""
    N = num_drones(df)
    t = df["time"].values
    T = np.stack([df[f"tension_{i}"].values for i in range(N)], axis=1)
    mask = np.ones_like(T, dtype=bool)
    for rope_idx, t_f in first_fault_per_rope(df, N):
        mask[t >= t_f, rope_idx] = False
    T_masked = np.where(mask, T, np.inf)
    Tmin = T_masked.min(axis=1)
    post = t >= T_PICKUP
    t_post = t[post]
    slack_flag = (Tmin[post] < TAUT_MIN_N) & np.isfinite(Tmin[post])

    if not slack_flag.any():
        return 0.0, 0.0, 0
    # run-length encode
    runs = []
    in_run = False
    run_start = None
    for i, b in enumerate(slack_flag):
        if b and not in_run:
            run_start = t_post[i]
            in_run = True
        elif not b and in_run:
            runs.append(t_post[i] - run_start)
            in_run = False
    if in_run:
        runs.append(t_post[-1] - run_start)
    duty = float(np.sum(runs) / (t_post[-1] - t_post[0])) \
        if t_post[-1] > t_post[0] else 0.0
    return float(max(runs)), duty, len(runs)


def main():
    log_path = OUT / "domain_audit_v11.log"
    rows = []
    with open(log_path, "w") as log:
        def tee(msg):
            print(msg)
            log.write(msg + "\n")
        tee("=" * 72)
        tee("Phase T — Domain Audit v1.1")
        tee("=" * 72)
        tee(f"Post-pickup cutoff:     t >= {T_PICKUP} s")
        tee(f"Slack threshold:        T < {TAUT_MIN_N} N")
        tee(f"Max slack-run allowed:  {TAU_SLACK_MAX_S*1000:.0f} ms")
        tee(f"Max slack duty cycle:   {ETA_MAX*100:.1f}%")
        tee("")

        all_pass = True
        demonstrated_pass = True
        for tag in VARIANTS:
            csv = CAMPAIGN / tag / f"scenario_{tag}.csv"
            if not csv.exists():
                tee(f"{tag}: MISSING"); continue
            df = pd.read_csv(csv)
            max_run, duty, n_runs = slack_runs(df)
            run_pass = max_run <= TAU_SLACK_MAX_S
            duty_pass = duty <= ETA_MAX
            both = run_pass and duty_pass

            run_margin = (TAU_SLACK_MAX_S - max_run) * 1000  # ms
            duty_margin_pp = (ETA_MAX - duty) * 100          # percentage points

            rows.append({
                "variant":     tag,
                "max_slack_run_ms":  round(max_run * 1000, 2),
                "duty_cycle_pct":    round(duty * 100, 3),
                "n_runs":            n_runs,
                "run_pass":          run_pass,
                "duty_pass":         duty_pass,
                "gate_pass":         both,
                "run_margin_ms":     round(run_margin, 2),
                "duty_margin_pp":    round(duty_margin_pp, 3),
            })
            status = "PASS" if both else "FAIL"
            tee(f"{tag:<24} max_run={max_run*1000:6.1f}ms "
                f"duty={duty*100:5.2f}% runs={n_runs:4d}  [{status}] "
                f"margins: run={run_margin:+5.1f}ms duty={duty_margin_pp:+5.2f}pp")
            if not both:
                all_pass = False
                if tag in ("V3_single_wind", "V4_dual_5s_wind",
                           "V5_dual_10s_wind"):
                    demonstrated_pass = False

        tee("")
        tee("-" * 72)
        if demonstrated_pass:
            tee("*** GATE v1.1 PASS ***  on demonstrated missions "
                "(V3, V4, V5).  Proceed to Reduction phase.")
        else:
            tee("*** GATE v1.1 FAIL ***  demonstrated missions violate "
                "the criterion.  Re-enter contract phase for v1.2.")
        if not all_pass and demonstrated_pass:
            tee("    NOTE: some NON-demonstrated missions fail; these "
                "are out of scope for MVP but should be documented.")

    pd.DataFrame(rows).to_csv(OUT / "domain_audit_v11_report.csv",
                              index=False)


if __name__ == "__main__":
    main()
