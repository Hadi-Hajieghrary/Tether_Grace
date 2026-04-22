# LaTeX sources

| File | Role |
|------|------|
| [`ieee_t_cst.tex`](ieee_t_cst.tex) | IEEE T-CST 14-page manuscript (primary submission). |
| [`supplementary.tex`](supplementary.tex) | Formal proofs (Theorems 1–6, Propositions 1–9) and extended results. |
| [`tether_grace_reference.tex`](tether_grace_reference.tex) | Book-length graduate reference; self-contained presentation of the full system. |
| [`methods_section.tex`](methods_section.tex) · [`results_section.tex`](results_section.tex) | Method and result chapters shared by the reference and the manuscript. |
| `table_II_ablation.tex` · `table_V_significance.tex` | Auto-generated from the campaign CSV; rebuilt with `make tables`. |
| [`Makefile`](Makefile) | Targets: `all`, `reference`, `paper`, `supplementary`, `tables`, `clean`, `distclean`. |

## Build

```bash
cd Research/docs/latex
make all                 # reference + paper + supplementary
make paper               # IEEE T-CST only
make tables              # rebuild Tables II and V from
                         # output/transactions_campaign/publication_metrics.csv
make clean               # remove aux files
```

The Makefile creates empty stub tables when the campaign has not yet
produced `publication_metrics.csv`, so the paper always compiles.

## Dependencies

Standard TeX Live distribution plus `amsmath`, `amssymb`, `amsthm`,
`booktabs`, `siunitx`, `cleveref`, `hyperref`, and the IEEE class
(`IEEEtran`).
