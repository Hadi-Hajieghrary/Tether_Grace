# Unknowns Resolution Status

## Executive Takeaways

- U1 remains open because the worst full-Drake / reduced-order CL post-fault RMSE ratio is 3.72, driven by the N=7 case.
- U2 is narrowed to numeric support: all canonical 3/5/7 scenarios satisfy the current thrust-margin check, but no formal sufficient-condition proof exists yet.
- U3 is now quantified across the canonical 3/5/7 scenarios: baseline CL remains theta-biased relative to fault-triggered flush for 12.46 s on average, while forgetting plus flush reduces that to 3.46 s.
- U4 is only partially resolved: reduced-order CL improves monotonically with N on the extended single-fault scaling set (true), but the matched full-Drake trend is mixed rather than monotone.
- U5 now has replayable multi-topology evidence for gradual fray and exponential weakening profiles; CL stays below the tested L1 variant on post-fault RMSE across all replayed degradation scenarios.
- U6 has now been searched more thoroughly: 12 structured non-CL candidates across 5 scenarios produced 0 robust winners.

## U1 Reduced-Order vs Full Drake

- Status: gap
- Max full-Drake / reduced-CL RMSE ratio: 3.72

| Scenario | N | Reduced CL post-fault RMSE [cm] | Full Drake load RMSE [cm] | Ratio |
| --- | ---: | ---: | ---: | ---: |
| three_drones | 3 | 30.35 | 53.37 | 1.76 |
| five_drones | 5 | 19.76 | 38.51 | 1.95 |
| seven_drones | 7 | 12.16 | 45.19 | 3.72 |

## U2 Topology-Invariance Conditions

- Status: numeric_support_only
- Current result is numeric support only, not a formal proof.

| Scenario | N | Margin OK | Sufficient-margin OK | Worst required thrust per agent [N] | Max available [N] | Capacity ratio | Effective capacity ratio | Healthy support fraction | Cost ratio |
| --- | ---: | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| three_drones | 3 | true | true | 29.43 | 51.50 | 1.75 | 1.17 | 0.67 | 1.50 |
| five_drones | 5 | true | true | 24.53 | 51.50 | 2.10 | 1.40 | 0.60 | 1.50 |
| seven_drones | 7 | true | true | 22.07 | 51.50 | 2.33 | 1.56 | 0.57 | 1.50 |

## U3 CL History Corruption

- Status: bias_detected_with_mitigation_signal
- Baseline mean theta-bias duration vs fault-triggered flush: 12.456666666666669
- Forgetting mean theta-bias duration vs fault-triggered flush: 3.946666666666667
- Forgetting+flush mean theta-bias duration vs fault-triggered flush: 3.4616666666666673
- Flush mean history refill time: 4.93

## U4 Generalization Beyond N=3

- Status: reduced_order_supported_full_drake_mixed
- Reduced-order CL RMSE decreases monotonically with N: true

| Scenario | N | Reduced CL post-fault RMSE [cm] | Full Drake load RMSE [cm] |
| --- | ---: | ---: | ---: |
| three_drones | 3 | 30.35 | 53.37 |
| five_drones | 5 | 19.76 | 38.51 |
| seven_drones | 7 | 12.16 | 45.19 |

### Extended Reduced-Order Scaling Set

| Scenario | N | CL post-fault RMSE [cm] | CL peak [cm] | CL spread [cm] |
| --- | ---: | ---: | ---: | ---: |
| topology_n3_single_snap | 3 | 37.11 | 40.96 | 0.00 |
| topology_n5_single_snap | 5 | 28.96 | 42.50 | 0.00 |
| topology_n7_single_snap | 7 | 23.72 | 39.84 | 0.00 |
| topology_n9_single_snap | 9 | 20.87 | 36.07 | 0.00 |
| topology_n11_single_snap | 11 | 20.19 | 35.37 | 0.00 |

## U5 Gradual Cable Degradation

- Status: extended_support

| Scenario | Profiles | CL post-fault RMSE [cm] | L1 post-fault RMSE [cm] | CL peak [cm] | L1 peak [cm] |
| --- | --- | ---: | ---: | ---: | ---: |
| three_drones_linear_fray | fray | 29.63 | 60.91 | 42.69 | 102.02 |
| three_drones_exponential_weaken | exponential | 29.40 | 61.06 | 42.53 | 102.02 |
| five_drones_linear_fray | linear | 22.42 | 95.09 | 36.27 | 153.67 |
| five_drones_exponential_weaken | exponential | 21.84 | 93.58 | 36.19 | 153.35 |
| seven_drones_linear_fray | fray | 19.39 | 73.94 | 34.40 | 89.25 |
| seven_drones_exponential_weaken | exponential | 19.10 | 73.17 | 34.23 | 89.25 |

## U6 Non-CL Adaptive Laws

- Status: thoroughly_tested_not_supported
- Candidate families tested: 6
- Total candidates tested: 12
- Robust winners: 0
- Best balanced candidate: shared_z_weighted | omega=3.00, predictor_pole=10.00, sigma_max=1.50
- Mean delta vs CL post-fault RMSE [cm]: 3.10
- Mean delta vs CL peak deviation [cm]: 18.91

| Scenario | Delta post vs CL [cm] | Delta peak vs CL [cm] | Delta spread vs CL [cm] |
| --- | ---: | ---: | ---: |
| topology_n3_single_snap | 4.44 | 23.23 | 0.00 |
| topology_n5_single_snap | 0.59 | 18.97 | 0.00 |
| topology_n7_single_snap | 0.21 | 11.80 | 0.00 |
| topology_n9_single_snap | 5.85 | 23.32 | 0.00 |
| topology_n11_single_snap | 4.42 | 17.23 | 0.00 |
