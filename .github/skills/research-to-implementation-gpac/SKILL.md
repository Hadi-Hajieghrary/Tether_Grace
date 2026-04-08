---
name: research-to-implementation-gpac
description: 'Turn research problem statements and reference solutions into root-repo implementation plans, interfaces, experiments, and validation artifacts for GPAC, L1 adaptive control, cable faults, and topology-invariance work. Use when translating problem_definition.pdf and references into software design without modifying git submodules such as Tether_Lift.'
argument-hint: 'Describe the research idea, reference artifact, and the implementation deliverable you want in the root repo'
user-invocable: true
---

# Research To Implementation GPAC

## What This Skill Produces

Use this skill to convert research material into root-owned engineering outputs such as:

- A requirements decomposition
- A root-repo software architecture
- An interface or adapter design
- An experiment plan
- A validation matrix
- A phased implementation roadmap
- A review of what should remain reference-only in submodules

## Ownership Rule

- The current Tether_Grace repo owns new software design and implementation work.
- Git submodules such as `Tether_Lift/` are reference-only unless the user explicitly authorizes changes to them.
- If a reference implementation exists only in a submodule, treat it as source material to inspect and port, not as the place to add the new work.

## When To Use

Use this skill when the task involves one or more of the following:

- Translating `problem_definition.pdf` into software requirements
- Turning `references/` material into root-repo source code, docs, experiments, or interfaces
- Porting an idea from `Tether_Lift/` without editing the submodule
- Designing root-owned wrappers, adapters, experiment runners, or analysis tools around GPAC or L1 concepts
- Building evidence for cable-snap fault tolerance or topology invariance

## Required Inputs

Collect the following before locking in a design:

- The research objective or claim
- The reference artifacts to inspect
- The desired root-repo deliverable
- The submodule paths that may be relevant as references
- The validation evidence the user expects

If any are missing, ask concise follow-up questions and state temporary assumptions clearly.

## Procedure

1. Extract the research intent.
   - Restate the objective in engineering terms.
   - Identify the control, estimation, fault, experiment, or validation target.
   - Separate the claim from the evidence currently available.

2. Establish ownership boundaries.
   - List which files and folders belong to the current repo.
   - List which paths are submodules and therefore reference-only.
   - Refuse silent drift of implementation into submodules.

3. Mine reference material.
   - Read `problem_definition.pdf` and the relevant files in `references/`.
   - Inspect submodule code only to understand existing APIs, equations, wiring patterns, or behavior.
   - Record what is reusable by import, what must be wrapped, and what must be reimplemented in the root repo.

4. Define the root-repo artifact set.
   - Decide whether the output belongs in root-level `src/`, `experiments/`, `docs/`, `analysis/`, or another root-owned directory.
   - Propose module boundaries, file layout, interfaces, and data flow.
   - Prefer thin and explicit integration boundaries with submodules.

5. Plan the implementation.
   - Map each requirement to a root-owned file or directory.
   - Identify imports, copied interfaces, adapters, wrappers, or scripts needed to interact with reference code.
   - Keep the plan incremental and testable.

6. Define the validation evidence.
   - Specify which metrics, figures, logs, or experiments prove success.
   - For CL-versus-L1 or fault-tolerance work, include pre-fault and post-fault metrics, recovery behavior, and constraint activity where relevant.
   - State what remains unverified.

7. Deliver a decision-ready output.
   - Start with the recommended root-repo design.
   - Follow with reference touchpoints, implementation phases, and validation criteria.
   - End with explicit next actions.

## Decision Rules

- Never place new implementation work inside a submodule unless the user explicitly requests it.
- If a submodule contains the only working example, port the idea into this repo instead of extending the submodule.
- If integration with a submodule is required, prefer wrappers, adapters, scripts, or documented import boundaries.
- If the research claim outruns the available evidence, surface the gap instead of implying completion.
- If several implementation homes are possible, choose the one that keeps ownership, testing, and documentation in the current repo.

## Response Pattern

Prefer this structure unless the user requests a different deliverable:

1. Recommended root-repo design
2. Reference touchpoints
3. Ownership boundaries
4. Implementation phases
5. Validation evidence
6. Open risks and next actions

## Quality Bar

A strong output from this skill should:

- Keep submodule boundaries explicit and respected
- Translate research language into testable implementation steps
- Show where each new artifact lives in the root repo
- Preserve traceability from claim to validation evidence
- End with a concrete, incremental execution path

## Example Prompts

- `/research-to-implementation-gpac Turn the cable-snap references into a root-repo experiment harness and validation plan without editing Tether_Lift.`
- `/research-to-implementation-gpac Port the L1 adaptive control idea from the references into a new root-owned software design for this repo.`
- `/research-to-implementation-gpac Compare what should stay as a Tether_Lift reference and what should become new implementation in Tether_Grace.`