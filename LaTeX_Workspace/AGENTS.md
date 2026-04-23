# AGENTS

## Scope

- This file applies to `LaTeX_Workspace/`.
- This folder is the LaTeX manuscript workspace for the orbital rendezvous coursework paper.
- `main.tex` and `main.bib` are the default active manuscript and bibliography unless the user states another target.
- `TempExample.tex` and `TempExample.bib` are template/reference files; do not treat them as the active paper unless explicitly requested.
- `aaas.cls`, `cite.sty`, and `GB.cpx` are vendored template/style files for the unofficial Acta Aeronautica et Astronautica Sinica template; do not edit them unless the user asks for template-level changes.
- `image/` stores manuscript figures and source-ready image assets.
- `build/` stores LaTeX build products; do not hand-edit generated `.aux`, `.bbl`, `.blg`, `.fdb_latexmk`, `.fls`, `.log`, `.out`, `.synctex.gz`, `.xdv`, or PDF outputs.

## Project Sources

- Inherit the repository-level `AGENTS.md` rules.
- For orbital-mechanics theory, notation, coordinate definitions, and modeling assumptions, consult `../docs/orbital_knowledge.md` before drafting or editing technical content.
- The authoritative local theory sources are `../docs/课程PPT_2025-2026学年第二学期.pdf` and `../docs/课程PPT_补充二体.pdf`.
- If the PDFs are unavailable, use `../docs/orbital_knowledge.md` as the default authoritative fallback.
- Keep manuscript equations and symbols consistent with the MATLAB coursework scripts, especially CW/LVLH convention: `x` radial, `y` along-track, and `z` cross-track.
- Treat `../MATLAB_Workspace/CW_Transfer/cw_stm.m` as the canonical CW state-transition implementation and `../MATLAB_Workspace/CW_Transfer/TransferParams.m` as the canonical multi-impulse transfer parameter container when discussing those models.
- If a manuscript edit changes a project formula, symbol definition, or modeling assumption rather than merely explaining it, update `../docs/orbital_knowledge.md` in the same change.

## Writing

- Write manuscript prose primarily in Chinese academic style, with English terms added only where useful for clarity or standard terminology.
- Keep statements defensible and traceable to the coursework theory, scripts, references, or explicitly provided numerical results.
- Do not invent simulation results, parameter values, dates, figures, or citations.
- Avoid unsupported novelty or priority claims such as "首次", "填补空白", or "具有极高学术价值" unless the user supplies evidence and asks for that framing.
- Define variables at first use. Prefer inline "式中：" explanations after equations unless the user asks for a separate symbol table.
- Keep units in SI form and use the existing `siunitx` workflow, such as `\SI{...}{...}` and `\si{...}`, where it fits the surrounding text.
- Keep calculations in radians unless the text is explicitly presenting angles in degrees, and always state displayed angle units clearly.

## LaTeX

- Preserve the existing `\documentclass[10.5pt,twocolumn]{aaas}` template structure unless the user asks to change journal/template format.
- Use the VSCode LaTeX Workshop settings in `../.vscode/settings.json` as the authoritative compile workflow.
- Use `latexmk` as the default compiler recipe. The expected command shape is `latexmk -pdfxe -bibtexfudge- -synctex=1 -interaction=nonstopmode -file-line-error -outdir=build %DOCFILE%`.
- Run LaTeX compilation from `LaTeX_Workspace/` with `build/` as the build/output directory through `-outdir=build`, matching the VSCode settings.
- Do not treat `build/` as an editable source tree; source edits belong in `main.tex`, `main.bib`, and related non-generated project files.
- Keep BibTeX resolution compatible with the VSCode settings by using `BIBINPUTS=.;..;` when invoking `latexmk` or standalone `bibtex`.
- If standalone BibTeX is needed for diagnosis, run it against `build/%DOCFILE%`, matching the VSCode tool configuration.
- Treat the older `xelatex -> bibtex -> xelatex -> xelatex` workflow in `README.md` as a fallback/reference workflow, not the default.
- Do not switch to `pdflatex`, `lualatex`, `biblatex`, or `biber` unless explicitly requested.
- Add packages only when needed, and prefer packages already loaded by `aaas.cls` or `main.tex`.
- Keep LaTeX comments in Chinese when adding or editing manuscript comments.
- Use `\citeAAAS{...}` for journal-style superscript citations in body text unless nearby text intentionally uses another citation form.
- Keep bibliography commands consistent with the active manuscript and active `.bib` file.
- Follow nearby equation, figure, table, theorem, and section structure. Do not broadly refactor template layout, title block, header/footer definitions, or two-column behavior for a local content edit.
- Use explicit LaTeX math markup at the call site for vectors, matrices, subscripts, superscripts, and units; do not rely on hidden automatic formatting rules.

## Figures And Tables

- Place new manuscript figures under `image/` unless the user specifies another asset layout.
- Use relative paths compatible with the existing template style, such as `./image/name.png`.
- Do not generate or regenerate figures by running MATLAB, Octave, or any MATLAB-compatible execution path.
- If figure data must come from MATLAB scripts, ask the user to run MATLAB manually and provide the exported figure or numerical result.
- Keep captions concise, technical, and aligned with the surrounding Chinese manuscript style.
- Tables should use the existing table packages and local style before introducing new table frameworks.

## Bibliography

- Use BibTeX entries in the active `.bib` file; do not migrate the project to another bibliography system without user approval.
- Preserve existing citation keys where possible.
- When adding references, verify bibliographic metadata from reliable sources when available, and do not fabricate DOI, journal, volume, issue, page, or year fields.
- Keep all cited references present in the `.bib` file and remove unused sample citations only when the user asks for cleanup or when replacing template text with real manuscript content.

## Validation

- Never execute MATLAB from the agent in this repository, including full scripts, snippets, syntax probes, `matlab -batch`, `matlab -r`, Octave, or any MATLAB-compatible execution path.
- LaTeX compilation with `latexmk` using the VSCode-style XeLaTeX/BibTeX workflow is allowed when needed and available, but it is not a substitute for MATLAB numerical verification.
- If LaTeX compilation is unavailable, validate edits by static checks: balanced environments, labels/citations/bibliography consistency, image paths, and package compatibility with XeLaTeX.
- If numerical or runtime verification is needed, ask the user to run the relevant MATLAB workflow manually and report the result.
