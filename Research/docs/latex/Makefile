## Build the Tether_Grace LaTeX outputs.
##
## Targets:
##   all            compile the graduate-reference, IEEE T-CST paper, and supplementary
##   reference      compile only the graduate reference
##   paper          compile only the IEEE T-CST manuscript
##   supplementary  compile only the supplementary
##   tables         regenerate the auto-built LaTeX tables from the campaign CSV
##   clean          remove LaTeX aux files
##   distclean      also remove the built PDFs

PDFLATEX := pdflatex -interaction=nonstopmode -halt-on-error

REFERENCE := tether_grace_reference
PAPER     := ieee_t_cst
SUPP      := supplementary

AUX := *.aux *.log *.out *.toc *.lof *.lot *.bbl *.blg *.fls *.fdb_latexmk

CAMPAIGN_METRICS := ../../../output/transactions_campaign/publication_metrics.csv
TABLES_DIR       := .

.PHONY: all reference paper supplementary tables clean distclean

all: reference paper supplementary

reference: $(REFERENCE).pdf
paper: $(PAPER).pdf
supplementary: $(SUPP).pdf

$(REFERENCE).pdf: $(REFERENCE).tex methods_section.tex results_section.tex
	$(PDFLATEX) $<
	$(PDFLATEX) $<

$(PAPER).pdf: $(PAPER).tex table_II_ablation.tex table_V_significance.tex
	$(PDFLATEX) $<
	$(PDFLATEX) $<

$(SUPP).pdf: $(SUPP).tex
	$(PDFLATEX) $<
	$(PDFLATEX) $<

tables:
	python3 ../../analysis/ieee/build_tables.py $(CAMPAIGN_METRICS) $(TABLES_DIR)

# Provide empty stubs so the paper compiles before the campaign runs.
table_II_ablation.tex:
	printf '%% placeholder; regenerate with `make tables` after the campaign\n\\begin{table}[t]\\centering\\caption{(placeholder)}\\end{table}\n' > $@

table_V_significance.tex:
	printf '%% placeholder; regenerate with `make tables` after the campaign\n\\begin{table}[t]\\centering\\caption{(placeholder)}\\end{table}\n' > $@

clean:
	rm -f $(AUX)

distclean: clean
	rm -f $(REFERENCE).pdf $(PAPER).pdf $(SUPP).pdf
	rm -f table_II_ablation.tex table_V_significance.tex
