# IEEE T-CST Paper Package

This folder is intended to be a stand-alone manuscript package.

## Contents

- `Main.tex`: paper entry point.
- `Sections/`: section text included by `Main.tex`.
- `Figures/`: local figure source files used by the paper.
- `References/References.bib`: local bibliography database.
- `IEEEtran.cls`: local class file used by the manuscript.

## Stand-alone policy

- Any figure referenced by the paper must live inside `Figures/`.
- Any bibliography entry cited by the paper must live inside `References/References.bib`.
- The paper text should not depend on external markdown, output reports, or source files outside this folder at build time.
- Numerical results may be copied into the manuscript from the canonical experiment artifacts, but the compiled paper should not require those artifacts to exist.

## Current figure files

- `Figures/paper_scope_overview.tex`
- `Figures/evidence_pipeline.tex`
- `Figures/unknowns_status_summary.tex`

These figure files are included directly with `\input{...}` from `Main.tex` through the section files.

## Typical build commands

If a TeX distribution is installed, a standard local build is:

```bash
cd IEEE_T-CST
pdflatex -interaction=nonstopmode Main.tex
bibtex Main
pdflatex -interaction=nonstopmode Main.tex
pdflatex -interaction=nonstopmode Main.tex
```

`latexmk -pdf Main.tex` is also acceptable when available.

## Build environment

The dev container provides `pdflatex`, `bibtex`, and `latexmk` on the `PATH`. A full compile cycle is:

```bash
cd IEEE_T-CST
pdflatex -interaction=nonstopmode Main.tex
bibtex Main
pdflatex -interaction=nonstopmode Main.tex
pdflatex -interaction=nonstopmode Main.tex
```