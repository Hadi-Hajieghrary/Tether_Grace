---
name: CL vs L1 Experiment Review
description: "Review a concurrent-learning versus L1 adaptive control experiment, proposal, figure set, or result summary. Use when evaluating cable-snap fault tolerance, topology invariance claims, pre-fault versus post-fault behavior, and whether the evidence is strong enough while keeping all new design work in the root repo and treating submodules as references only."
argument-hint: "Describe the CL-vs-L1 experiment artifact, claim, or figure set to review"
agent: "Safety Validation Reviewer"
tools: [read, search, todo]
---

Review the provided CL-versus-L1 experiment artifact with a fault-tolerance and evidence-first mindset.

Requirements:

- Treat the current Tether_Grace repo as the place for any new design notes, review outputs, or follow-up implementation work.
- Treat submodules such as `Tether_Lift/` as read-only reference baselines unless the user explicitly requests otherwise.
- Prioritize behavioral findings, unsupported claims, and missing validation over stylistic comments.
- If figures or logs are insufficient to support a conclusion, say so explicitly.

Evaluate, when available, these metrics and signals:

- Nominal RMSE
- Pre-fault RMSE
- Post-fault RMSE
- RMSE ratio
- Peak deviation after fault
- Recovery time
- Cable tension floor or tautness violations
- Attitude or tilt margin
- CBF activation or constraint activity
- Actuator or thrust saturation
- Estimator convergence behavior

Output exactly these sections:

1. Experiment scope
2. Compared configurations
3. Metrics observed
4. Findings ranked by severity
5. Unsupported claims or evidence gaps
6. Residual risks
7. Recommended next checks

If a metric is missing, include it under section 5 instead of guessing.
