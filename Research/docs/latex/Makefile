## Compile the Tether_Grace graduate-reference PDF.
## Usage:
##   make          # compile with pdflatex (two passes for cross-refs)
##   make clean    # remove intermediate LaTeX artefacts
##   make distclean# also remove the output PDF

SRC := tether_grace_reference.tex
PDF := $(SRC:.tex=.pdf)
AUX := *.aux *.log *.out *.toc *.lof *.lot *.bbl *.blg *.fls *.fdb_latexmk

.PHONY: all clean distclean

all: $(PDF)

$(PDF): $(SRC)
	pdflatex -interaction=nonstopmode $(SRC)
	pdflatex -interaction=nonstopmode $(SRC)

clean:
	rm -f $(AUX)

distclean: clean
	rm -f $(PDF)
