# Verification tests

Numerical regression tests that back the formal results of the
reproducibility report (report Chapters 5–8 for the layer-local
theorems; report Chapters 2A and 5B for the Phase-T theorems) and the
archived IEEE supplementary at
[`../../archive/docs_2026_04_23/latex/supplementary.tex`](../../archive/docs_2026_04_23/latex/supplementary.tex).
Each file validates one theorem or proposition against the constants
that actually live in the C++ codebase — so a failure means either
the analytic claim or the implementation has drifted, and must be
triaged before the next campaign is launched.

The tests are Python and deliberately small: they regenerate the
evidence that underpins the paper's math, they do not exercise the
C++ simulator. End-to-end simulator smoke testing is the job of the
scripts under [`../scripts/`](../scripts/).

## Map

| Test | Supplementary reference | What it asserts |
|------|------------------------|-----------------|
| [`test_l1_stability.py`](test_l1_stability.py) | Proposition 1 and Corollary 1 | `A_m` eigenvalues, the closed-form Lyapunov matrix satisfies `A_mᵀ P + P A_m = −I`, the Γ* bound evaluates to 4.75 × 10⁵, and the Euler-discretised σ̂ update contracts below Γ* and diverges above it. |
| [`test_mpc_tension_linearisation.py`](test_mpc_tension_linearisation.py) | Theorem 4 | For 2 000 random perturbations with ‖e_p‖ ≤ 2 cm, the first-order tension linearisation error stays below the second-order Taylor bound (k_eff/(2 d_nom))·‖e_p‖². |
| [`test_dare.py`](test_dare.py) | Theorem 3 | The discrete Riccati equation solved for the MPC's scalar double integrator yields a positive-definite P and a closed-loop spectral radius `ρ(A − BK) < 1`. Uses the actual deployed (Δt, Q, R). |
| [`test_reshape_optimality.py`](test_reshape_optimality.py) | Theorem 5 | For N = 4 → M = 3, a brute-force 1°-grid search over the surviving triple confirms the ±30° symmetric rotation is the minimax peak-tension optimum. |
| [`test_smoothstep_certificate.py`](test_smoothstep_certificate.py) | Proposition 7 | The quintic smoothstep has zero value, first, and second derivatives at both endpoints, peak first derivative 15/8, and the N = 4 → M = 3 minimum pairwise chord distance 1.131 m exceeds the 0.5 m safe-clearance threshold. |

## Running

```bash
# All five tests (stops on the first failure):
for t in Research/tests/test_*.py; do python3 "$t" || break; done
```

Individual test:

```bash
python3 Research/tests/test_dare.py
```

Each script prints `"<test_name>: OK"` on success and raises an
`AssertionError` on failure.

## Conventions

- Constants are sourced from the deployed controller parameters
  (`Research/cpp/include/decentralized_local_controller.h`,
  `mpc_local_controller.h`). Do not duplicate them — import or
  cross-reference instead, and comment the mirror relationship so
  the provenance is obvious.
- Every test file maps one-to-one to a theorem or proposition; the
  docstring states the mapping and the numerical tolerance.
- Tolerances are tight (≤ 1 %) where the claim is analytic, and
  looser (≤ 10 %) where the claim is a Taylor-style bound with
  random sampling.

## Adding a new test

1. Add a new theorem or proposition to the supplementary.
2. Create `Research/tests/test_<short_name>.py` following the same
   pattern: module docstring names the result, the numerical check
   reproduces the evidence, and the `__main__` block prints
   `"test_<short_name>: OK"` on success.
3. Add the file to the table above with a one-line description of
   what it asserts and a pointer to the supplementary reference.

## Dependencies

- Python ≥ 3.10.
- `numpy`, `scipy`.

The devcontainer at the repository root has both installed; no
additional setup is required.

## What is *not* tested here

- End-to-end closed-loop simulation behaviour — that is exercised by
  the campaign runners under `../scripts/` and the publication-figure
  pipeline under `../analysis/ieee/`.
- C++-level correctness of the DARE guard flag and the stale-
  diagnostics refresh — these are verified by running
  `decentralized_fault_aware_sim` in a smoke test; there is no
  C++ unit-test scaffolding in this workspace.
