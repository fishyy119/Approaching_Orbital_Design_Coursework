# AGENTS

## Scope

- This repository is an orbital rendezvous coursework project centered on two-body motion and CW-based proximity transfer.
- `MATLAB_Workspace/Two_Body/` contains absolute-orbit and classical two-body scripts.
- `MATLAB_Workspace/CW_Transfer/` contains CW relative-motion, rendezvous transfer, and control scripts.
- `MATLAB_Workspace/+utils/` contains shared plotting helpers.
- For theory questions and for edits under `MATLAB_Workspace/Two_Body/` or `MATLAB_Workspace/CW_Transfer/`, consult `docs/orbital_knowledge.md` first.
- The authoritative theory sources are `docs/课程PPT_2025-2026学年第二学期.pdf` and `docs/课程PPT_补充二体.pdf`.
- If those local PDFs are unavailable in the current workspace, treat `docs/orbital_knowledge.md` as the default authoritative fallback until the user manually restores them.

## General

- Follow nearby files for naming, section layout, constants, and plotting style; do not introduce new conventions or broad-refactor unrelated scripts unless the user explicitly asks.
- Preserve the current folder split between `Two_Body/`, `CW_Transfer/`, and `+utils/`.

## MATLAB

- Write MATLAB comments in Chinese.
- Keep script structure explicit with `%%` blocks for parameter setup, computation, plotting, and local helper functions.
- Prefer reusing `MATLAB_Workspace/+utils/` for figure defaults, paper-sized figures, colormaps, and mixed Chinese/English text formatting instead of reimplementing per-script styling.
- Keep physical constants, symbols, and coordinate definitions consistent with nearby scripts and `docs/orbital_knowledge.md`.
- Keep angles in radians inside calculations unless a script is explicitly presenting degrees.
- In CW-related files, treat relative position and velocity as LVLH/CW-frame quantities with `x` radial, `y` along-track, `z` cross-track, matching `cw_stm.m` and `o8.m`.
- Treat `cw_stm.m` as the canonical CW state-transition implementation and `TransferParams.m` as the canonical multi-impulse parameter container.
- When changing formulas, symbols, or modeling assumptions, update `docs/orbital_knowledge.md` in the same change.

## Validation

- Never execute MATLAB from the agent in this repository.
- This prohibition includes full scripts, partial snippets, syntax probes, `matlab -batch`, `matlab -r`, Octave, or any other MATLAB-compatible execution path.
- If numerical execution or runtime verification is needed, ask the user to run MATLAB manually and report the result back.
- There is no current automated test harness. Validate by checking symbolic consistency, dimensions, and alignment with nearby scripts.
