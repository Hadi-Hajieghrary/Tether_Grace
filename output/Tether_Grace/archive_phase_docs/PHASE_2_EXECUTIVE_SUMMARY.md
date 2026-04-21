# Phase 2 Executive Summary: Decentralized MPC Multi-Drone Deployment

**Date**: 2026-04-17  
**Status**: RESEARCH PLAN COMPLETE  
**Target Execution**: April 21 - May 16, 2026 (4 weeks)  
**Next Review**: April 21, 2026 (Phase 1 Remediation kickoff)

---

## The Big Picture

**Phase 1** (skeleton complete) built the basic Drake infrastructure: multibody plant, rope physics, controller interfaces. But the MPC controller is **non-functional**:
- No state feedback (tests use zero inputs)
- No optimization objective (empty solver)
- No convergence validation (no discrete state)
- Fallback policy undersized

**Phase 2** fills this gap by:
1. **Enabling closed-loop control** (state splitter extracts robot state)
2. **Implementing MPC objectives** (trajectory tracking + effort minimization)
3. **Validating implicit coordination** (two drones coordinate via rope forces, zero communication)

---

## What Phase 2 Proves

| Claim | Evidence | Timeline |
|-------|----------|----------|
| **Single-drone MPC is stable** | Phase 1 test: 10 scenarios, ≥99% solver success | Week 1-2 |
| **Two drones coordinate implicitly** | Load position estimates agree within 0.2m (no communication) | Week 2 |
| **Implicit coordination scales** | Three drones: pairwise estimate agreement < 0.3m | Week 3 |
| **Faults are detectable** | Cable severance triggers solver infeasibility within 20ms | Week 4 |

---

## 4-Week Execution Plan

### Week 1: Phase 1 Remediation (4 Fixes)
**Goal**: Enable closed-loop feedback; functional MPC optimizer

| Fix | What | Hours | Risk | Impact |
|-----|------|-------|------|--------|
| 1 | Plant state extractor (LeafSystem) | 2-3 | Low | High |
| 2 | MPC cost function (trajectory + effort) | 4-5 | Medium | Critical |
| 3 | Discrete state refactor (load estimator) | 3-4 | Medium | High |
| 4 | Rope tautness constraint | 2-3 | Medium | Critical |

**Success metric**: Phase 1 test achieves ≥99% solver success rate, <10 ms mean solve time

---

### Week 2: Single-Drone Validation + 2-Drone Test
**Goal**: Prove implicit coordination in N=2 system

**Single-drone** (10 scenarios):
- Hover at various heights
- Check: load tracks reference, rope stays taut

**Two-drone** (10 scenarios):
- Both pull same payload (separate ropes)
- Check: estimates p_L_est_0 and p_L_est_1 agree within 0.2m
- Check: no inter-drone communication

**Success metric**: Agreement error < 0.2m at t=10s (steady state)

---

### Week 3: 3-Drone Formation
**Goal**: Validate scalability to N≥3

**Scenarios**: 10 (hover + circular motion)

**Success metric**: Pairwise estimate agreement < 0.3m; convergence time constant < 5 seconds

---

### Week 4: Cable Severance (Phase 3 Preview)
**Goal**: Demonstrate fault detection and graceful degradation

**Scenarios**: 5 (nominal → severance at t=5s → recovery)

**Success metric**: Fault detected within 20ms; load stabilizes within 2 seconds

---

## Key Design Decisions

### 1. Why Decentralized MPC?
- **Alternative**: Centralized optimizer (one solver for all N drones)
  - Pro: Global optimality
  - Con: Single point of failure; all drones depend on one computer
- **Our choice**: Each drone solves independently
  - Pro: Robust to single-drone failure; scales to many drones
  - Con: Suboptimal (no global view); requires implicit coordination mechanism

### 2. How Does Implicit Coordination Work?
```
Drone 0 thinks:                Drone 1 thinks:
"Load is here, I'll pull"      "Load is here, I'll pull"
         ↓                              ↓
    Increases F_0          Increases F_1
         ↓                              ↓
    Rope extends           Rope extends
         ↓                              ↓
    Both observe: T ↑        Both observe: T ↑
         ↓                              ↓
    Load accelerates up
         ↓                              ↓
    Both update: p_L rises
         ↓                              ↓
    Converge to same           Converge to same
    load state                 load state
```

**Key insight**: Rope tension magnitude encodes load state. Both drones observe same tension (indirectly via rope kinematics) → converge to agreement.

### 3. Why MPC, Not PID?
- **MPC advantages**: Anticipates future (3-second horizon); handles constraints (rope tautness); multi-objective (tracking + effort)
- **PID advantages**: Simple, fast, proven
- **Choice**: MPC because Phase 2 validates the mechanism; Phase 3 (fault recovery) benefits from foresight

### 4. Rope Model: Simple vs. Complex
- **Simple**: Massless, inextensible (spring force = k(L_natural - L_current))
- **Complex**: Bead chain (8 segments with mass, damping, coupling)
- **Phase 2 choice**: Use plant's bead-chain (for realism), but estimator uses simple model
  - Ensures estimation is well-posed (less parameters to tune)
  - Allows control to be simple (linear ropeDynamics)

---

## Critical Success Factors

| Factor | Why It Matters | How Phase 2 Validates |
|--------|----------------|-----------------------|
| **Rope stays taut** | Slack rope = uncontrolled system | Monitor T ≥ 0 for all time steps |
| **Solver is fast** | Missed deadlines → instability | Log solve time; target P95 < 25ms |
| **Estimates converge** | Coordination depends on agreement | Log p_L_est_0, p_L_est_1; compare |
| **No communication** | Proves implicit mechanism | Code audit + network monitor |
| **Faults are detectable** | Enables Phase 3 recovery | Cable severance scenario |

---

## Risk Snapshot

| Risk | Likelihood | Impact | Mitigation | Fallback |
|------|-----------|--------|-----------|----------|
| **R1: Rope slack** | HIGH | CRITICAL | Slack variables + penalty | Lookup table |
| **R2: Estimates diverge** | MEDIUM | HIGH | Filter tuning + Kalman | Redefine criterion |
| **R3: Solver slow** | MEDIUM | HIGH | Reduce horizon | Lower frequency |
| **R4: Schedule slip** | MEDIUM | MEDIUM | Parallelize tests | Defer Phase 2b |
| **R5: Wiring bugs** | MEDIUM | MEDIUM | Unit tests + incremental build | Sequential drones |

**Confidence level**: 85% probability of Phase 2 completion on schedule with positive results

---

## Deliverables (End of Phase 2)

### Code
- [ ] `StateExtractor` class (state splitter system)
- [ ] Updated `DecentralizedOptimalController` (MPC with objectives + constraints)
- [ ] `decentralized_mpc_test_2drones` executable
- [ ] `decentralized_mpc_test_3drones` executable
- [ ] All tests pass (no regression)

### Data
- [ ] 40 CSV files (10 scenarios × 2 test configurations)
- [ ] Logs: solver time, status, load estimates, tensions, control efforts

### Documentation
- [ ] **PHASE_2_RESULTS.md** (2 pages): metrics table, convergence analysis
- [ ] **PHASE_2_ARCHITECTURE.md** (2 pages): system diagram, signal flow, implicit coordination mechanism
- [ ] **PHASE_2_IMPLICIT_COORDINATION_PROOF.md** (1-2 pages): mathematical or empirical argument
- [ ] **PHASE_3_DESIGN_PREVIEW.md** (2 pages): cable failure detection, recovery strategy

---

## Budget & Schedule

### Time Investment
| Phase | Task | Hours | Owner |
|-------|------|-------|-------|
| **Week 1** | Phase 1 remediation (4 fixes) | 12-15 | Algorithm lead |
| **Week 1** | Unit tests + integration | 4-5 | Test lead |
| **Week 2** | Single-drone campaign (10 runs) | 2-3 | Automation |
| **Week 2** | 2-drone test harness | 6-8 | Algorithm lead |
| **Week 2** | 2-drone campaign (10 runs) | 2-3 | Automation |
| **Week 2** | Analysis + plots | 4-5 | Analysis lead |
| **Week 3** | 3-drone test harness | 4-5 | Algorithm lead |
| **Week 3** | 3-drone campaign (10 runs) | 2-3 | Automation |
| **Week 3** | Convergence analysis | 4-5 | Analysis lead |
| **Week 4** | Severance test + Phase 3 design | 8-10 | Algorithm lead |
| **Week 4** | Documentation + report | 6-8 | Analysis lead |
| **Total** | | **54-70 hours** | Team of 3 |

---

## Go / No-Go Gates

### After Week 1
**Gate Criteria**:
- Phase 1 test compiles and runs
- Solver success rate ≥ 99%
- Mean solve time < 10 ms
- All 4 remediation items complete

**Decision**: Continue to Week 2 or pause for redesign

---

### After Week 2
**Gate Criteria**:
- Single-drone: 9/10 scenarios successful
- Two-drone: load estimates agree within 0.2m
- Implicit coordination validated (no communication observed)

**Decision**: Continue to Week 3 or redefine scope

---

### After Week 3
**Gate Criteria**:
- Three-drone: pairwise agreement < 0.3m
- Convergence time constant documented (≤ 5s)

**Decision**: Continue to Week 4 (Phase 3 preview) or defer to later

---

### After Week 4
**Gate Criteria**:
- Cable severance detected within 20ms
- Recovery time ≤ 2 seconds
- All documentation complete + publishable

**Decision**: Phase 2 COMPLETE ✓ or Phase 2 EXTENDED (if minor gaps)

---

## Phase 3 Foundation (Prepared by Phase 2)

**Phase 2 provides**:
1. **Baseline implicit coordination behavior** (multi-drone agreement, convergence rate)
2. **Fault detection method** (solver infeasibility OR estimate divergence)
3. **Robust load estimator** (quantified accuracy, noise bounds)
4. **Fallback control policy** (validated for nominal + post-fault scenarios)

**Phase 3 builds on this**:
1. **Progressive cable degradation detection** (trending T or p_L_est divergence)
2. **Thrust reallocation** (CLSR + remaining drones' load capacity)
3. **Coordinated descent** (graceful landing with reduced thrust)
4. **Cascading fault tolerance** (N-2 capability?)

---

## Quality Assurance

### Code Review Checklist
- [ ] State extractor: verified against plant state layout
- [ ] MPC setup: constraints are feasible for nominal scenarios
- [ ] Discrete state: no mutable members in const methods
- [ ] Drake conventions: all LeafSystems follow proper lifecycle

### Test Coverage
- [ ] Unit tests: StateExtractor, cost function formulation, constraint validation
- [ ] Integration tests: Full diagram with plant + controllers + loggers
- [ ] Regression tests: Ensure Phase 1 test still passes with changes

### Data Quality
- [ ] CSV files: correct columns, no missing values
- [ ] Plots: axes labeled, legend clear, save as PDF (publication-ready)
- [ ] Analysis: All claims (convergence, agreement, success rate) validated with code

---

## Communication & Reporting

**Weekly Status** (Friday EOD):
- Short email: Scenarios run, metrics achieved, risks/blockers
- Risk dashboard: Red/yellow/green status of R1-R8

**End of Phase** (May 16):
- Full technical report (PHASE_2_RESULTS.md)
- Plots package (figures/phase2_*)
- Code PR (merge Phase 2 into main after review)

---

## How to Use This Plan

**For developers**:
- Read PHASE_2_RESEARCH_PLAN.md (Section 2) for detailed implementation
- Refer to PHASE_2_RISK_REGISTER.md if any issue arises
- Follow go/no-go gates; escalate if needed

**For project managers**:
- Track progress against 4-week timeline
- Monitor risk dashboard weekly
- Escalate blockers to authority (project PI)

**For stakeholders**:
- This summary provides headline status
- Refer to full plan for detailed architecture/metrics
- Expect publication-ready results by May 16

---

## Success Vision

**By end of Phase 2**:

You can demonstrate to a skeptical audience:

> "Two independent quadcopter controllers, running MPC optimization without inter-drone communication, coordinate a shared payload via implicit rope-tension feedback. Load position estimates from both drones converge to agreement within 0.2 meters. The system scales to three drones with robust implicit coordination, even under cable stress. When a cable severs, the optimizer detects the fault within 20 milliseconds and gracefully initiates recovery."

This is a **genuine scientific contribution** demonstrating:
1. **Implicit coordination** (no explicit communication)
2. **Decentralized control** (each drone autonomous)
3. **Fault robustness** (graceful degradation)
4. **Scalability** (N ≥ 2 validated; N ≥ 3 previewed)

---

## Next Steps

**Today (2026-04-17)**: Team reads plan + risk register; raises questions in standup

**Tomorrow (2026-04-18)**: Code review of Phase 1 remediation strategy; kickoff development

**Monday (2026-04-21)**: Phase 1 remediation begins (Week 1 sprint)

**Friday (2026-04-25)**: End-of-week gate review; go/no-go decision for Week 2

---

## Questions & Contacts

| Topic | Contact |
|-------|---------|
| Algorithm design | Algorithm lead (Drake/MPC expertise) |
| Test harness / infrastructure | Test lead (C++ / Drake integration) |
| Data analysis / validation | Analysis lead (Python / statistical methods) |
| Risk escalation | Project PI |

**Plan review date**: April 21, 2026 (after first week, assess any issues)

---

## Appendix: Quick Reference

### Key Metrics
- **Solver success target**: ≥ 99%
- **Solve time target**: P95 < 25 ms, mean < 10 ms
- **Load estimate agreement**: < 0.2 m (Phase 2a), < 0.3 m (Phase 2b)
- **Convergence time constant**: < 5 seconds
- **Rope tautness**: T ≥ 0 for all time steps (no slack)
- **No communication**: Zero inter-drone messages verified

### File Tree
```
Tether_Grace/
├── PHASE_2_RESEARCH_PLAN.md       [This plan — detailed roadmap]
├── PHASE_2_RISK_REGISTER.md       [Risk management — 8 risks documented]
├── PHASE_2_EXECUTIVE_SUMMARY.md   [This file — high-level overview]
├── Research/cpp/
│   ├── include/
│   │   ├── state_extractor.h      [NEW: state splitter]
│   │   └── decentralized_optimal_controller.h  [MODIFIED: add objectives]
│   ├── src/
│   │   ├── state_extractor.cc     [NEW: implementation]
│   │   ├── decentralized_optimal_controller.cc  [MODIFIED: Phase 1 fixes]
│   │   ├── decentralized_mpc_test_main.cc  [MODIFIED: wire state extractor]
│   │   ├── decentralized_mpc_test_2drones_main.cc  [NEW: 2-drone test]
│   │   └── decentralized_mpc_test_3drones_main.cc  [NEW: 3-drone test]
│   └── CMakeLists.txt  [MODIFIED: add new targets]
├── results/phase2/
│   ├── phase2_scenario_01.csv     [Data: scenario 1]
│   ├── ... (39 more CSV files)
│   └── phase2_analysis_results.txt  [Summary statistics]
└── docs/
    ├── PHASE_2_RESULTS.md         [Published results — 2 pages]
    ├── PHASE_2_ARCHITECTURE.md    [System design — 2 pages]
    └── PHASE_3_DESIGN_PREVIEW.md  [Fault recovery strategy — 2 pages]
```

---

**Plan prepared by**: Tether_Grace Research Team  
**Date**: April 17, 2026  
**Status**: READY FOR EXECUTION  
**Next milestone**: Week 1 Phase 1 Remediation Complete (April 25, 2026)

