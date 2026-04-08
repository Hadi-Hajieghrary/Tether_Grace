---
name: Root Repo Ownership
description: "Use when planning, designing, reviewing, or implementing software in this workspace. Keeps new software design, experiments, docs, and code in the current Tether_Grace repo and treats git submodules like Tether_Lift as reference-only unless the user explicitly asks to modify them."
applyTo: "**"
---

# Root Repo Ownership

- Treat the current Tether_Grace repository as the home for all new design work, implementation, experiments, scripts, prompts, skills, agents, and documentation.
- Treat any path declared in `.gitmodules` as read-only unless the user explicitly asks to modify that submodule.
- Use submodules such as `Tether_Lift/` only as references for APIs, algorithms, file layout ideas, imports, or behavioral baselines.
- Do not place new architecture, controllers, experiment harnesses, validation logic, or design docs inside `Tether_Lift/` or any other submodule.
- When implementation is needed, create or extend root-owned paths in this repo such as `src/`, `experiments/`, `docs/`, `analysis/`, or other top-level directories that belong to Tether_Grace.
- If a feature depends on submodule behavior, define a root-repo adapter, wrapper, copied interface contract, or documented integration boundary rather than editing the submodule.
- When comparing against submodule code, state clearly what is reused by import or reference and what is newly designed in this repo.
- If the best available example lives in a submodule, inspect it, cite the path in the explanation, and then implement the corresponding design in the root repo.
- If a request appears to require submodule edits, pause and ask before modifying any submodule-tracked file.
