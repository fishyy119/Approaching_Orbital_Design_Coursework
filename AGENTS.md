# AGENTS

## Scope

- This repository is an orbital rendezvous coursework project centered on two-body motion and CW-based proximity transfer.
- `LaTeX_Workspace/` is the manuscript workspace for the orbital rendezvous coursework paper.
- `LaTeX_Workspace/main.tex` and `LaTeX_Workspace/main.bib` are the default active manuscript and bibliography unless the user states another target.
- `LaTeX_Workspace/TempExample.tex` and `LaTeX_Workspace/TempExample.bib` are template/reference files; do not treat them as the active paper unless explicitly requested.
- `LaTeX_Workspace/aaas.cls`, `LaTeX_Workspace/cite.sty`, and `LaTeX_Workspace/GB.cpx` are vendored template/style files; do not edit them unless the user asks for template-level changes.
- `LaTeX_Workspace/image/` stores manuscript figures and source-ready image assets.
- `LaTeX_Workspace/build/` stores LaTeX build products; do not hand-edit generated `.aux`, `.bbl`, `.blg`, `.fdb_latexmk`, `.fls`, `.log`, `.out`, `.synctex.gz`, `.xdv`, or PDF outputs.
- `MATLAB_Workspace/Two_Body/` contains absolute-orbit and classical two-body scripts.
- `MATLAB_Workspace/CW_Transfer/` contains CW relative-motion, rendezvous transfer, and control scripts.
- `MATLAB_Workspace/Final_Project/Remote_Guidance/` contains final-project remote-guidance search, numerical verification, trajectory plotting, and animation scripts.
- `MATLAB_Workspace/Final_Project/Proximity_Guidance/` contains final-project proximity-guidance / near-field planning scripts.
- `MATLAB_Workspace/+utils/` contains shared plotting helpers.
- For theory questions and for edits under `MATLAB_Workspace/Two_Body/` or `MATLAB_Workspace/CW_Transfer/`, consult `docs/orbital_knowledge.md` first.
- The authoritative theory sources are `docs/课程PPT_2025-2026学年第二学期.pdf` and `docs/课程PPT_补充二体.pdf`.
- If those local PDFs are unavailable in the current workspace, treat `docs/orbital_knowledge.md` as the default authoritative fallback until the user manually restores them.

## General

- Follow nearby files for naming, section layout, constants, and plotting style; do not introduce new conventions or broad-refactor unrelated scripts unless the user explicitly asks.
- Preserve the current folder split between `Two_Body/`, `CW_Transfer/`, `Final_Project/Remote_Guidance/`, `Final_Project/Proximity_Guidance/`, and `+utils/`.
- Do not add automatic MATLAB path-management logic such as `addpath` into project scripts; assume the user maintains MATLAB search paths manually.

## LaTeX Manuscript

- For manuscript theory, notation, coordinate definitions, and modeling assumptions, consult `docs/orbital_knowledge.md` before drafting or editing technical content.
- Keep manuscript equations and symbols consistent with the MATLAB coursework scripts, especially the CW/LVLH convention: `x` radial, `y` along-track, and `z` cross-track.
- Treat `MATLAB_Workspace/CW_Transfer/cw_stm.m` as the canonical CW state-transition implementation and `MATLAB_Workspace/CW_Transfer/TransferParams.m` as the canonical multi-impulse transfer parameter container when discussing those models.
- If a manuscript edit changes a project formula, symbol definition, or modeling assumption rather than merely explaining it, update `docs/orbital_knowledge.md` in the same change.
- Write manuscript prose primarily in Chinese academic style, with English terms added only where useful for clarity or standard terminology.
- Keep statements defensible and traceable to the coursework theory, scripts, references, or explicitly provided numerical results.
- Do not invent simulation results, parameter values, dates, figures, or citations.
- Avoid unsupported novelty or priority claims such as "首次", "填补空白", or "具有极高学术价值" unless the user supplies evidence and asks for that framing.
- Define variables at first use. Prefer inline "式中：" explanations after equations unless the user asks for a separate symbol table.
- Keep units in SI form and use the existing `siunitx` workflow, such as `\SI{...}{...}` and `\si{...}`, where it fits the surrounding text.
- Keep calculations in radians unless the text is explicitly presenting angles in degrees, and always state displayed angle units clearly.
- Preserve the existing `\documentclass[10.5pt,twocolumn]{aaas}` template structure unless the user asks to change journal/template format.
- Use the VSCode LaTeX Workshop settings in `.vscode/settings.json` as the authoritative compile workflow when compilation is explicitly requested.
- If compilation is explicitly requested, use `latexmk` as the default compiler recipe. The expected command shape is `latexmk -pdfxe -bibtexfudge- -synctex=1 -interaction=nonstopmode -file-line-error -outdir=build %DOCFILE%`, run from `LaTeX_Workspace/`.
- Keep BibTeX resolution compatible with the VSCode settings by using `BIBINPUTS=.;..;` when invoking `latexmk` or standalone `bibtex`.
- If standalone BibTeX is needed for diagnosis after an explicit compile request, run it against `build/%DOCFILE%`, matching the VSCode tool configuration.
- Treat the older `xelatex -> bibtex -> xelatex -> xelatex` workflow in `LaTeX_Workspace/README.md` as a fallback/reference workflow, not the default.
- Do not switch to `pdflatex`, `lualatex`, `biblatex`, or `biber` unless explicitly requested.
- Add packages only when needed, and prefer packages already loaded by `aaas.cls` or `main.tex`.
- Keep LaTeX comments in Chinese when adding or editing manuscript comments.
- Use `\citeAAAS{...}` for journal-style superscript citations in body text unless nearby text intentionally uses another citation form.
- Keep bibliography commands consistent with the active manuscript and active `.bib` file.
- Follow nearby equation, figure, table, theorem, and section structure. Do not broadly refactor template layout, title block, header/footer definitions, or two-column behavior for a local content edit.
- Use explicit LaTeX math markup at the call site for vectors, matrices, subscripts, superscripts, and units; do not rely on hidden automatic formatting rules.
- Place new manuscript figures under `LaTeX_Workspace/image/` unless the user specifies another asset layout.
- Use relative paths compatible with the existing template style, such as `./image/name.png`.
- Do not generate or regenerate figures by running MATLAB, Octave, or any MATLAB-compatible execution path.
- If figure data must come from MATLAB scripts, ask the user to run MATLAB manually and provide the exported figure or numerical result.
- Keep captions concise, technical, and aligned with the surrounding Chinese manuscript style.
- Tables should use the existing table packages and local style before introducing new table frameworks.
- Use BibTeX entries in the active `.bib` file; do not migrate the project to another bibliography system without user approval.
- Preserve existing citation keys where possible.
- When adding references, verify bibliographic metadata from reliable sources when available, and do not fabricate DOI, journal, volume, issue, page, or year fields.
- Keep all cited references present in the `.bib` file and remove unused sample citations only when the user asks for cleanup or when replacing template text with real manuscript content.

## MATLAB

- Write MATLAB comments in Chinese.
- Keep script structure explicit with `%%` blocks for parameter setup, computation, plotting, and local helper functions.
- Prefer reusing `MATLAB_Workspace/+utils/` for figure defaults, paper-sized figures, colormaps, and mixed Chinese/English text formatting instead of reimplementing per-script styling.
- For plot text in this repository, keep the `tex` interpreter workflow unless the user explicitly asks for a different interpreter.
- Treat `MATLAB_Workspace/+utils/formatMixedFontText.m` as a helper only for Chinese/English body-font switching; do not add math parsing or automatic math-style logic into it.
- In MATLAB `tex` interpreter text, spaces are rendered literally and must not be used as command separators; do not place separator spaces immediately after `tex` commands, but keep intentional visible spaces between Chinese, English, units, and formula blocks when they improve readability.
- For mathematical symbols and variable styling in plot labels, titles, legends, and colorbars, write explicit `tex` markup at the call site, such as `{\itt}_{\itd}` or `\Delta{\itv}`, instead of expecting automatic formatting, and add visible spaces only outside the `tex` command spans when needed.
- Keep physical constants, symbols, and coordinate definitions consistent with nearby scripts and `docs/orbital_knowledge.md`.
- Keep angles in radians inside calculations unless a script is explicitly presenting degrees.
- In CW-related files, treat relative position and velocity as LVLH/CW-frame quantities with `x` radial, `y` along-track, `z` cross-track, matching `cw_stm.m` and `o8.m`.
- Treat `cw_stm.m` as the canonical CW state-transition implementation and `TransferParams.m` as the canonical multi-impulse parameter container.
- When changing formulas, symbols, or modeling assumptions, update `docs/orbital_knowledge.md` in the same change.

## Validation

- Never execute MATLAB from the agent in this repository.
- This prohibition includes full scripts, partial snippets, syntax probes, `matlab -batch`, `matlab -r`, Octave, or any other MATLAB-compatible execution path.
- For edits under `LaTeX_Workspace/`, do not run `latexmk`, `xelatex`, `bibtex`, or any other LaTeX compilation command unless the user explicitly requests compilation in the current conversation.
- For LaTeX manuscript edits, default to static checks only, such as balanced environments, labels and references, citation and bibliography consistency, image paths, and obvious package compatibility.
- If numerical execution or runtime verification is needed, ask the user to run MATLAB manually and report the result back.
- There is no current automated test harness. Validate by checking symbolic consistency, dimensions, and alignment with nearby scripts.
