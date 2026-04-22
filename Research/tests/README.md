# Verification tests

Each file in this folder corresponds to one formal result in
[`../docs/latex/supplementary.tex`](../docs/latex/supplementary.tex).
Running them regenerates the numerical evidence that backs the
theorems and propositions; failures indicate a drift between the
implementation constants and the analytical derivation and must be
triaged before the next campaign launch.

| Test | Formal result |
|------|---------------|
| `test_dare.py` | Theorem 3 (DARE spectral radius < 1) |
| `test_mpc_tension_linearisation.py` | Theorem 4 (Taylor remainder ≤ 0.5 %) |
| `test_l1_stability.py` | Proposition 1, Corollary 1 (Lyapunov matrix, Γ* bound) |
| `test_reshape_optimality.py` | Theorem 5 (30° rotation is globally optimal) |
| `test_smoothstep_certificate.py` | Proposition 7 (C² smoothstep safety) |

Run individually with `python3 tests/test_<name>.py`, or all at once:

```bash
for t in Research/tests/test_*.py; do python3 "$t"; done
```
