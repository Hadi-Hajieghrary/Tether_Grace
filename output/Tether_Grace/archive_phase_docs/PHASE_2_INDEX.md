# Phase 2 Planning Index & Navigation Guide

**Created**: April 17, 2026  
**Status**: PLANNING PHASE COMPLETE  
**Next Action**: Start Phase 1 Remediation (Week of April 21)

---

## Documents Prepared

Three complementary documents define Phase 2 strategy:

### 1. PHASE_2_EXECUTIVE_SUMMARY.md
**Length**: 15 KB, 392 lines  
**Audience**: Project stakeholders, management, leadership  
**Read time**: 15 minutes  
**Key content**:
- High-level objectives (what Phase 2 proves)
- 4-week timeline overview
- Success gates & go/no-go criteria
- Risk snapshot & escalation chain
- Deliverables checklist

**Use case**: Share with non-technical stakeholders to understand Phase 2 impact & schedule

---

### 2. PHASE_2_RESEARCH_PLAN.md
**Length**: 67 KB, 1,544 lines  
**Audience**: Technical team, algorithm designers, test engineers  
**Read time**: 60-90 minutes (reference document)  
**Key sections**:

| Section | Purpose | Pages |
|---------|---------|-------|
| **Section 1** | Weekly timeline (Week 1-4 breakdown) | 8 |
| **Section 2** | Phase 1 Remediation Roadmap (4 fixes detailed) | 20 |
|  - Fix 1 | Plant truth extractor (state splitter) | 5 |
|  - Fix 2 | MPC trajectory cost implementation | 6 |
|  - Fix 3 | Discrete state refactor (load estimator) | 5 |
|  - Fix 4 | Rope tautness constraint | 4 |
| **Section 3** | Multi-drone architecture design | 8 |
| **Section 4** | Validation metrics framework | 10 |
| **Section 5** | Risk register & mitigation | 10 |
| **Section 6** | Success criteria checklist | 8 |
| **Section 7** | Phase 3 foresight | 6 |
| **Appendix** | Code templates & snippets | 4 |

**Use case**: Reference during development; check detailed implementation guidance for each fix

---

### 3. PHASE_2_RISK_REGISTER.md
**Length**: 31 KB, 977 lines  
**Audience**: Technical team, project manager, risk owner  
**Read time**: 45 minutes (or consult specific risk when needed)  
**Key content**:

| Risk | Likelihood | Impact | Mitigation Depth |
|------|-----------|--------|------------------|
| **R1: Rope slack infeasible** | HIGH (70%) | CRITICAL | 3 levels + fallback |
| **R2: Estimates diverge** | MEDIUM (40%) | HIGH | 3 levels + fallback |
| **R3: Solver slow** | MEDIUM (35%) | HIGH | 3 levels + fallback |
| **R4: Schedule overrun** | MEDIUM (35%) | MEDIUM | 2 levels + fallback |
| **R5: Wiring bugs** | MEDIUM (30%) | MEDIUM | 2 levels + fallback |
| **R6: No fault detection** | LOW (15%) | MEDIUM | 1 level + fallback |
| **R7: SNOPT missing** | LOW (10%) | MEDIUM | 1 level + fallback |
| **R8: Filter NaN crash** | LOW (8%) | LOW | 1 level |

**Use case**: Consult when risk manifests; escalate using protocol defined in Section 4

---

## How to Use These Documents

### Scenario: "I'm new to Phase 2. Where do I start?"

1. **Read**: PHASE_2_EXECUTIVE_SUMMARY.md (15 min)
   - Understand what Phase 2 proves (implicit coordination, N≥2)
   - See 4-week timeline
   - Review deliverables

2. **Explore**: PHASE_2_RESEARCH_PLAN.md Section 1 (10 min)
   - Details of each week
   - What tests to run
   - Metrics to measure

3. **If assigned to algorithm development**: Read Section 2 (30 min)
   - Detailed implementation of 4 Phase 1 fixes
   - Drake code examples
   - Risk mitigation strategies

4. **If testing**: Read Section 4 (20 min)
   - Validation metrics definitions
   - Test scenarios
   - CSV logging format

---

### Scenario: "I'm implementing Fix 2 (MPC cost function)"

1. **Refer to**: PHASE_2_RESEARCH_PLAN.md, Section 2, Fix 2 (6 pages)
   - Motivation: why cost function matters
   - Implementation: complete code template
   - Drake APIs involved
   - Risk: solver complexity
   - Effort estimate: 4-5 hours

2. **Check**: PHASE_2_RISK_REGISTER.md, R1 & R3 (solver-related)
   - Mitigation for infeasibility (if cost too complex)
   - Mitigation for slow solver (if horizon too long)

3. **After implementation**: Validate with unit tests
   - Cost function decreases over iterations
   - Constraints are feasible for nominal scenarios

---

### Scenario: "Solver keeps failing (status=false). What now?"

1. **First**: Check PHASE_2_RISK_REGISTER.md, **R1: Rope slack makes optimizer infeasible** (6 pages)
   - Detection signals
   - Root causes (ranked by probability)
   - Primary mitigation: add slack variables + penalty
   - Secondary mitigation: reduce horizon
   - Tertiary mitigation: simplify rope model
   - Fallback: use pre-computed lookup table

2. **Action plan**:
   - Try primary mitigation (code change: 30 min)
   - Re-test; measure solver success rate
   - If still failing, implement secondary (1-2 hours)
   - Escalate if all mitigations insufficient

3. **Timeline**: Must resolve by EOD Week 1 (April 25) or escalate to PI

---

### Scenario: "After 2-drone test, load position estimates don't agree (error > 0.3m)"

1. **Check**: PHASE_2_RISK_REGISTER.md, **R2: Load position estimates diverge** (6 pages)
   - Root causes (rope measurement noise, filter tuning, kinematics)
   - Detection analysis (compare p_L_est_0, p_L_est_1, p_L_true in CSV)
   - Primary mitigation: validate measurement accuracy (did in Week 1? Re-check)
   - Secondary mitigation: increase filter gain α (0.1 → 0.25 → 0.5)
   - Tertiary mitigation: implement Kalman filter (replaces complementary filter)

2. **Action plan**:
   - Run analysis script to identify root cause
   - Try secondary mitigation (parameter change: 15 min)
   - Re-test one scenario to verify improvement
   - If successful, re-run all scenarios
   - If still failing, escalate by EOD Week 2

---

### Scenario: "It's Friday (end of Week X). I need to write risk status update"

1. **Template**: Use Risk Dashboard template in PHASE_2_RISK_REGISTER.md, Section 5

2. **Metrics to check**:
   - **Week 1**: Solver success % (target ≥99%), mean solve time (target <10ms)
   - **Week 2**: Two-drone agreement error (target <0.2m), communication verified zero
   - **Week 3**: Three-drone agreement (target <0.3m), convergence τ (target <5s)
   - **Week 4**: Fault detection latency (target <30ms), recovery time (target <2s)

3. **If any metric red/yellow**: Trigger escalation protocol (Section 4, Risk_Register.md)

---

## Key Metrics to Remember

### Success Thresholds (Go/No-Go)

| Metric | Phase 2a Target | Phase 2b Target | Critical? |
|--------|-----------------|-----------------|-----------|
| Solver success rate | ≥ 99% | ≥ 99.5% | YES |
| Mean solve time | < 10 ms | < 8 ms | YES |
| Two-drone agreement error | < 0.2 m | < 0.15 m | YES |
| Three-drone pairwise error | < 0.3 m | < 0.15 m | YES |
| Convergence time constant τ | < 5 s | < 2 s | NO |
| Rope tautness (T ≥ 0) | 100% | 100% | YES |
| No inter-drone communication | Verified | Verified | YES |
| Fault detection latency | < 30 ms | < 20 ms | NO |

---

## Document Relationships

```
┌─────────────────────────────────────────────────────────────┐
│          PHASE_2_RESEARCH_PLAN.md (Main Blueprint)          │
│              [1,544 lines, 67 KB, comprehensive]             │
│                                                               │
│  ├─ Section 1: Weekly Timeline                              │
│  │  └─ Links to Risk Register (R1-R8)                       │
│  │                                                            │
│  ├─ Section 2: Phase 1 Remediation (4 Fixes)                │
│  │  ├─ Fix 1: State Extractor                               │
│  │  ├─ Fix 2: MPC Cost Function                             │
│  │  │  └─ Risk: R1 (infeasibility), R3 (slow solver)       │
│  │  ├─ Fix 3: Discrete State Refactor                       │
│  │  └─ Fix 4: Rope Tautness Constraint                      │
│  │     └─ Risk: R1 (feasibility)                            │
│  │                                                            │
│  ├─ Section 3: Multi-Drone Architecture                     │
│  │  └─ Design specifies state representation                │
│  │  └─ Signal flow diagram (no communication)               │
│  │                                                            │
│  ├─ Section 4: Validation Metrics                           │
│  │  ├─ Definitions (agreement error, convergence τ, etc.)   │
│  │  ├─ Test scenarios (20 scenarios × 10 seeds = 200 runs)  │
│  │  └─ Analysis script (Python pseudocode)                  │
│  │     └─ Links to Success Criteria (Section 6)             │
│  │                                                            │
│  ├─ Section 5: Risk Register Overview                       │
│  │  └─ Refers to PHASE_2_RISK_REGISTER.md for detail        │
│  │                                                            │
│  ├─ Section 6: Success Criteria Checklist                   │
│  │  ├─ Functionality (all 4 remediation items + tests)      │
│  │  ├─ Documentation (4 key documents)                      │
│  │  ├─ Code Quality (Drake conventions, no regressions)     │
│  │  └─ Insights (convergence proof, fault detection)        │
│  │                                                            │
│  └─ Section 7: Phase 3 Foresight                            │
│     └─ Foundation from Phase 2                              │
│                                                               │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│        PHASE_2_EXECUTIVE_SUMMARY.md (Leadership Brief)      │
│              [392 lines, 15 KB, high-level]                  │
│                                                               │
│  ├─ What Phase 2 Proves (implicit coordination, N≥2)        │
│  ├─ 4-Week Execution Plan (sketch of each week)             │
│  ├─ Key Design Decisions (why MPC? why decentralized?)      │
│  ├─ Risk Snapshot (8 risks, likelihoods, mitigations)       │
│  │  └─ Refers to PHASE_2_RISK_REGISTER.md for detail        │
│  ├─ Deliverables (code, data, documentation)                │
│  ├─ Go/No-Go Gates (decision points, criteria)              │
│  └─ Phase 3 Foundation (what Phase 2 enables)               │
│                                                               │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│          PHASE_2_RISK_REGISTER.md (Risk Management)          │
│              [977 lines, 31 KB, detailed tactics]             │
│                                                               │
│  ├─ R1: Rope slack infeasible (70% likely, CRITICAL)        │
│  │  ├─ Detection signals                                     │
│  │  ├─ Root causes (ordered by probability)                 │
│  │  ├─ Primary mitigation (slack variables)                 │
│  │  ├─ Secondary mitigation (reduce horizon)                │
│  │  ├─ Tertiary mitigation (simplify rope model)            │
│  │  ├─ Fallback (lookup table)                              │
│  │  └─ Escalation protocol (when, whom, deadline)           │
│  │                                                            │
│  ├─ R2: Estimates diverge (40% likely, HIGH)                │
│  │  ├─ Root causes (noise, filter, kinematics)              │
│  │  ├─ Primary: validate measurement                        │
│  │  ├─ Secondary: increase filter gain                      │
│  │  ├─ Tertiary: Kalman filter                              │
│  │  └─ Fallback: redefined criterion                        │
│  │                                                            │
│  ├─ R3: Solver time > 50ms (35% likely, HIGH)               │
│  │  ├─ Mitigation: reduce horizon (Week 2)                  │
│  │  ├─ Fallback: lower control frequency                    │
│  │  └─ Escalation: EOD Week 1                               │
│  │                                                            │
│  ├─ R4-R8: Medium/Low risks (similar structure)             │
│  │                                                            │
│  └─ Section 4: Escalation Protocol                          │
│     ├─ Trigger conditions (which metrics trigger alert)      │
│     ├─ Owner/Authority assignments                          │
│     └─ Recovery workflow                                     │
│                                                               │
└─────────────────────────────────────────────────────────────┘

Downstream Documents (Produced during Phase 2):
│
├─ PHASE_2_RESULTS.md (published after Week 2-4)
├─ PHASE_2_ARCHITECTURE.md (system design + diagrams)
├─ PHASE_2_IMPLICIT_COORDINATION_PROOF.md (convergence analysis)
├─ PHASE_3_DESIGN_PREVIEW.md (fault recovery roadmap)
│
└─ Data artifacts:
   ├─ results/phase2/phase2_scenario_*.csv (20-40 CSV files)
   ├─ results/phase2/plots/ (convergence curves, agreement error)
   └─ results/phase2/analysis.py (validation script)
```

---

## Quick Lookup: Which Document Answers...

| Question | Document | Section |
|----------|----------|---------|
| "What does Phase 2 accomplish?" | Executive Summary | "The Big Picture" |
| "How long will Phase 2 take?" | Executive Summary | "Budget & Schedule" |
| "When do we decide to go/no-go?" | Executive Summary | "Go / No-Go Gates" |
| "What are the 4 Phase 1 fixes?" | Research Plan | "Section 2" (detail), Executive Summary |
| "How does implicit coordination work?" | Research Plan | "Section 3" + "Section 4" (metrics) |
| "What tests should we run?" | Research Plan | "Section 4" (test scenarios) |
| "What success metrics matter?" | Research Plan | "Section 4" + "Section 6" |
| "What if the rope gets slack?" | Risk Register | "R1: Rope slack infeasible" |
| "What if solver is too slow?" | Risk Register | "R3: Solver time > 50ms" |
| "What if estimates don't agree?" | Risk Register | "R2: Estimates diverge" |
| "What if tests take too long?" | Risk Register | "R4: Schedule overrun" |
| "What's the fallback if everything fails?" | Risk Register | Each risk's "Fallback" subsection |
| "How do we escalate?" | Risk Register | "Section 4: Escalation Protocol" |
| "What comes after Phase 2?" | Research Plan | "Section 7: Phase 3 Foresight" |

---

## Revision & Update Schedule

### This Plan Is Frozen (Will Not Change)
- Timelines, success criteria, validation metrics
- Risk register (as of April 17, 2026)
- Phase 1 remediation roadmap

### This Plan Evolves (Will Update)
- Weekly risk dashboard (new data each Friday)
- Actual performance vs. targets (tracked in PHASE_2_LOG.md, created during execution)
- Lessons learned (captured in post-execution retrospective)

### How to Propose Changes
If plan needs revision during Phase 2:
1. Document the change request (what, why, impact)
2. Notify project PI and algorithm lead
3. Assess risk impact (does change create new risks? eliminate old ones?)
4. Decision: approve revision or proceed with original plan
5. Update PHASE_2_REVISION_LOG.md (maintained during execution)

---

## File Locations

All Phase 2 documents live in repository root:

```
/workspaces/Tether_Grace/
├── PHASE_2_EXECUTIVE_SUMMARY.md     ← START HERE
├── PHASE_2_RESEARCH_PLAN.md         ← DETAILED ROADMAP
├── PHASE_2_RISK_REGISTER.md         ← RISK MANAGEMENT
├── PHASE_2_INDEX.md                 ← THIS FILE
│
├── Research/cpp/                    ← Source code
│   ├── include/
│   │   ├── decentralized_optimal_controller.h
│   │   └── [state_extractor.h - created Week 1]
│   └── src/
│       ├── decentralized_optimal_controller.cc
│       ├── [state_extractor.cc - created Week 1]
│       └── decentralized_mpc_test_main.cc
│
└── results/phase2/                  ← Data (created during execution)
    ├── phase2_scenario_01.csv
    ├── ...
    └── analysis_results.md
```

---

## Approval & Sign-Off

**Plan Status**: READY FOR EXECUTION ✓

**Prepared by**: Tether_Grace Research Team  
**Date**: April 17, 2026  
**Review date**: April 21, 2026 (after Phase 1 Week 1 kickoff)  
**Final approval**: TBD (project PI sign-off before execution)

---

## Contact & Questions

| Role | Responsibility | Contact |
|------|-----------------|---------|
| **Algorithm Lead** | Phase 1 fixes, MPC optimization, risk escalation | [Name] |
| **Test Lead** | Test harness, data collection, wiring validation | [Name] |
| **Analysis Lead** | Metrics computation, plots, documentation | [Name] |
| **Project PI** | Go/no-go decisions, escalation authority | [Name] |

---

**Last updated**: April 17, 2026  
**Next update**: Friday, April 25, 2026 (end of Week 1)

