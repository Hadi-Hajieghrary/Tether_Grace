# Tether_Grace — Graduate Reference (LaTeX source)

This folder holds the LaTeX source of the graduate-student reference
document for the Tether_Grace system. The compiled PDF is a
self-contained book-length case study covering the problem framing,
full mathematical model, the fully-local QP controller derivation,
the Drake simulation infrastructure, the six-scenario validation
campaign, and a discussion of novelty and open problems, plus
appendices (parameter table, code-to-theory map, reproducibility
instructions, exercises, and a full evidence-status log).

## File layout

| File | Role |
|------|------|
| [`tether_grace_reference.tex`](tether_grace_reference.tex) | The monolithic LaTeX document. No include/input files — a single `.tex`. |
| [`Makefile`](Makefile) | `make`, `make clean`, `make distclean`. |
| `README.md` | This file. |

## Prerequisites

- **TeX distribution**: TeX Live 2022 or newer (Ubuntu:
  `sudo apt-get install texlive-latex-extra texlive-fonts-recommended
  texlive-science`; macOS: MacTeX).
- Packages used by the document (all part of a standard TeX Live
  installation): `amsmath`, `amssymb`, `bm`, `booktabs`, `enumitem`,
  `graphicx`, `xcolor`, `tcolorbox`, `listings`, `hyperref`,
  `cleveref`, `fancyhdr`, `titlesec`, `tikz`, `cleveref`.

## Compilation

```bash
# preferred (handles cross-refs via two passes automatically)
make

# manual
pdflatex tether_grace_reference.tex
pdflatex tether_grace_reference.tex

# clean intermediate artefacts
make clean

# also remove the output PDF
make distclean
```

The output is a single `tether_grace_reference.pdf` in this folder.
Expected size ≈ 60–80 pages depending on font rendering.

## Content map

The document is organised in six parts plus appendices:

| Part | Chapters | Focus |
|------|----------|-------|
| I Problem Formulation | 1, 2 | motivation, system, research question |
| II Mathematical Foundations | 3, 4, 5 | dynamics, rope model, reference trajectory |
| III Control Design | 6, 7, 8 | decentralisation choice, fully-local controller, fault model |
| IV Implementation in Drake | 9, 10, 11 | framework, code architecture, harness |
| V Validation Campaign | 12, 13 | protocol, results |
| VI Discussion and Extensions | 14, 15, 16 | novelty, limitations, extensions |
| Appendices | A–E | parameters, code-to-theory, reproduce, exercises, evidence log |

## Evidence conventions

Every load-bearing numerical or structural claim is tagged with one of:

- **[O]** Observed — verified in code/log/figure (file cited).
- **[D]** Derived — pencil-and-paper from observed quantities.
- **[I]** Inference — high-confidence conclusion, not provably tight.
- **[E]** Empirical — sim-campaign only; no proof.
- **[L]** Limitation — explicit scope boundary.

The full per-claim table is in Appendix E.

## Companion markdown documentation

Under `Research/docs/` you will also find:

- [`INDEX.md`](../INDEX.md) — navigation hub across the project.
- [`case_study_tether_grace.md`](../case_study_tether_grace.md) — the
  markdown precursor to this LaTeX document.
- [`theory/`](../theory/) — four standalone theory briefs, each with
  code-to-equation mappings.

The LaTeX document subsumes the content of the four theory briefs and
the case-study markdown. Readers preferring a lighter reading can
start with the markdown files; readers wanting the printable,
single-artefact reference should compile this.
